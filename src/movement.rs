use aabb::Aabb;
use best::BestMultiSet;
use bump::max_bump;
use cgmath::{Vector2, vec2};
use collide::Collision;
use shape::Shape;

const EPSILON: f64 = 0.01;
const JUMP_TEST_MOVEMENT: Vector2<f64> = Vector2 { x: 0., y: EPSILON };

#[derive(Default)]
pub struct MovementContext {
    closest_collisions: BestMultiSet<Collision>,
}

pub type ClosestCollisions<'a> = &'a BestMultiSet<Collision>;

pub type EntityId = u32;

#[derive(Debug)]
pub struct ShapePosition<'a> {
    pub entity_id: EntityId,
    pub position: Vector2<f64>,
    pub shape: &'a Shape,
}

impl<'a> ShapePosition<'a> {
    fn aabb(&self) -> Aabb {
        self.shape.aabb(self.position)
    }
    fn movement_aabb(&self, movement: Vector2<f64>) -> Aabb {
        let current_aabb = self.aabb();
        let moved_aabb = self.shape.aabb(self.position + movement);
        current_aabb.union(&moved_aabb)
    }
    fn movement_collision_test(
        &self,
        other: ShapePosition,
        movement: Vector2<f64>,
        closest_collisions: &mut BestMultiSet<Collision>,
    ) {
        self.shape.movement_collision_test(
            self.position,
            other.shape,
            other.position,
            movement,
            closest_collisions,
        )
    }
}

pub trait ForEachShapePosition {
    fn for_each<F: FnMut(ShapePosition)>(&self, aabb: Aabb, f: F);
}

pub struct Movement {
    pub position: Vector2<f64>,
    pub velocity: Vector2<f64>,
}

impl MovementContext {
    fn for_each_collision<F, G>(
        &mut self,
        shape_position: ShapePosition,
        movement: Vector2<f64>,
        for_each_shape_position: &F,
        mut f: G,
    ) where
        F: ForEachShapePosition,
        G: FnMut(EntityId, Collision),
    {
        self.closest_collisions.clear();
        for_each_shape_position.for_each(
            shape_position.movement_aabb(movement),
            |other_shape_position: ShapePosition| {
                let other_entity_id = other_shape_position.entity_id;
                if other_entity_id != shape_position.entity_id {
                    shape_position.movement_collision_test(
                        other_shape_position,
                        movement,
                        &mut self.closest_collisions,
                    );
                    if let Some(collision) = self.closest_collisions.drain().next() {
                        f(other_entity_id, collision);
                    }
                }
            },
        );
    }
    fn closest_collisions<F>(
        &mut self,
        shape_position: ShapePosition,
        movement: Vector2<f64>,
        for_each_shape_position: &F,
    ) -> ClosestCollisions
    where
        F: ForEachShapePosition,
    {
        self.closest_collisions.clear();
        for_each_shape_position.for_each(
            shape_position.movement_aabb(movement),
            |other_shape_position: ShapePosition| {
                if other_shape_position.entity_id != shape_position.entity_id {
                    shape_position.movement_collision_test(
                        other_shape_position,
                        movement,
                        &mut self.closest_collisions,
                    );
                }
            },
        );

        &self.closest_collisions
    }

    pub fn can_jump<F>(
        &mut self,
        shape_position: ShapePosition,
        for_each_shape_position: &F,
    ) -> bool
    where
        F: ForEachShapePosition,
    {
        !self.closest_collisions(
            shape_position,
            JUMP_TEST_MOVEMENT,
            for_each_shape_position,
        ).is_empty()
    }
    pub fn position_after_allowed_movement<F>(
        &mut self,
        shape_position: ShapePosition,
        movement: Vector2<f64>,
        for_each_shape_position: &F,
    ) -> Movement
    where
        F: ForEachShapePosition,
    {
        let mut state = MovementStateMachine::new(movement, shape_position.position);
        let env = MovementEnv {
            for_each_shape_position,
            original: shape_position,
        };
        loop {
            if let Some(movement) = state.step(&env, self) {
                return movement;
            }
        }
    }
    pub fn displacement_after_movement<F>(
        &mut self,
        shape_position: ShapePosition,
        movement: Vector2<f64>,
        for_each_shape_position: &F,
        displacements: &mut Vec<(EntityId, Vector2<f64>)>,
    ) where
        F: ForEachShapePosition,
    {
        self.for_each_collision(
            shape_position,
            movement,
            for_each_shape_position,
            |entity_id, collision| {
                let displacement_movement = collision
                    .left_solid_edge_collision
                    .movement_following_collision(movement);
                displacements.push((entity_id, displacement_movement));
            },
        );
    }
}

struct MovementEnv<'a, F: 'a + ForEachShapePosition> {
    for_each_shape_position: &'a F,
    original: ShapePosition<'a>,
}

impl<'a, F: ForEachShapePosition> MovementEnv<'a, F> {
    fn shape_position(&self, position: Vector2<f64>) -> ShapePosition {
        ShapePosition {
            position,
            ..self.original
        }
    }
    fn closest_collisions<'b>(
        &self,
        position: Vector2<f64>,
        movement: Vector2<f64>,
        ctx: &'b mut MovementContext,
    ) -> ClosestCollisions<'b> {
        ctx.closest_collisions(
            self.shape_position(position),
            movement,
            self.for_each_shape_position,
        )
    }
}

struct MovementStateMachine {
    movement: Vector2<f64>,
    position: Vector2<f64>,
    bump: Option<Vector2<f64>>,
    velocity_correction: Vector2<f64>,
    remaining: u8,
}

impl MovementStateMachine {
    fn new(movement: Vector2<f64>, position: Vector2<f64>) -> Self {
        const MAX_ITERATIONS: u8 = 16;
        Self {
            movement,
            position,
            bump: None,
            velocity_correction: vec2(0., 0.),
            remaining: MAX_ITERATIONS,
        }
    }
    fn to_movement(&self, original_position: Vector2<f64>) -> Movement {
        Movement {
            position: self.position,
            velocity: self.position - original_position + self.velocity_correction,
        }
    }
    fn step<F>(
        &mut self,
        env: &MovementEnv<F>,
        ctx: &mut MovementContext,
    ) -> Option<Movement>
    where
        F: ForEachShapePosition,
    {
        if self.remaining == 0 {
            return Some(self.to_movement(env.original.position));
        }
        match self.bump {
            Some(bump) => {
                let closest = env.closest_collisions(self.position, bump, ctx);
                match closest.first() {
                    Some(_closest) => {
                        return Some(self.to_movement(env.original.position))
                    }
                    None => {
                        self.position += bump;
                        self.velocity_correction -= bump;
                        self.bump = None;
                    }
                }
            }
            None => {
                let closest_collisions =
                    env.closest_collisions(self.position, self.movement, ctx);
                match closest_collisions.first() {
                    None => {
                        self.position += self.movement;
                        return Some(self.to_movement(env.original.position));
                    }
                    Some(closest) => {
                        self.position += closest
                            .left_solid_edge_collision
                            .movement_to_collision(self.movement);
                        match max_bump(closest_collisions) {
                            None => {
                                self.movement = closest
                                    .left_solid_edge_collision
                                    .slide(self.movement);
                            }
                            Some(max_bump) => {
                                self.bump = Some(max_bump.vector());
                                self.movement = closest
                                    .left_solid_edge_collision
                                    .movement_following_collision(self.movement);
                            }
                        }
                    }
                }
            }
        };
        self.remaining -= 1;
        None
    }
}
