use aabb::Aabb;
use best::BestMultiSet;
use cgmath::Vector2;
use collide::{channels, CollisionInfo};
use shape::Shape;

#[derive(Default)]
pub struct MovementContext {
    closest_collisions: BestMultiSet<CollisionInfo>,
}

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
        closest_collisions: &mut BestMultiSet<CollisionInfo>,
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
    pub can_jump: bool,
}

impl MovementContext {
    fn allowed_movement_step<F>(
        &mut self,
        shape_position: ShapePosition,
        movement: Vector2<f64>,
        for_each_shape_position: &F,
    ) where
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
    }

    pub fn position_after_allowed_movement<F>(
        &mut self,
        shape_position: ShapePosition,
        mut movement: Vector2<f64>,
        for_each_shape_position: &F,
    ) -> Movement
    where
        F: ForEachShapePosition,
    {
        let mut can_jump = false;
        let mut position = shape_position.position;
        const MAX_ITERATIONS: usize = 16;
        for _ in 0..MAX_ITERATIONS {
            self.allowed_movement_step(
                ShapePosition {
                    position,
                    ..shape_position
                },
                movement,
                for_each_shape_position,
            );
            if self.closest_collisions
                .iter()
                .any(|c| c.moving_edge_channels() & channels::FLOOR != 0)
            {
                can_jump = true;
            }
            match self.closest_collisions.iter().next() {
                None => {
                    position += movement;
                    break;
                }
                Some(collision_with_slide) => {
                    position += collision_with_slide.movement_to_collision(movement);
                    movement = collision_with_slide.slide(movement);
                }
            }
        }
        Movement {
            position,
            velocity: position - shape_position.position,
            can_jump,
        }
    }
}
