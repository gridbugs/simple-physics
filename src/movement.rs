use aabb::Aabb;
use best::BestMultiSet;
use cgmath::{InnerSpace, Vector2, vec2};
use collide::{channels, flags, CollisionInfo};
use shape::Shape;
use std::cmp::Ordering;
use left_solid_edge::StartOrEnd;

const EPSILON: f64 = 0.01;

#[derive(Default)]
pub struct MovementContext {
    closest_collisions: BestMultiSet<CollisionInfo>,
}

pub type EntityId = u32;
const BUMP_DISTANCE_PX: f64 = 2.;
const BUMP_DISTANCE_PX2: f64 = BUMP_DISTANCE_PX * BUMP_DISTANCE_PX;

#[derive(Debug)]
pub struct ShapePosition<'a> {
    pub entity_id: EntityId,
    pub position: Vector2<f64>,
    pub shape: &'a Shape,
}

struct Bump {
    distance2: f64,
    direction: Vector2<f64>,
}

impl Bump {
    fn vector(&self) -> Vector2<f64> {
        self.direction.normalize_to(self.distance2.sqrt())
    }
}

fn bump(collision_info: &CollisionInfo) -> Option<Bump> {
    if collision_info.moving_edge_vector.flags & flags::BUMP_START != 0 {
        if let Some(edge_collision_position) = collision_info
            .collision
            .moving_edge_min_collision_position()
        {
            if edge_collision_position.which_part_of_other_edge == StartOrEnd::Start {
                let multiplier = edge_collision_position.how_far_along_this_edge;
                let distance2 = collision_info.moving_edge_vector.vector.magnitude2()
                    * multiplier * multiplier;
                if distance2 <= BUMP_DISTANCE_PX2 {
                    return Some(Bump {
                        distance2: distance2 + EPSILON,
                        direction: collision_info.moving_edge_vector.vector,
                    });
                }
            }
        }
    }
    if collision_info.moving_edge_vector.flags & flags::BUMP_END != 0 {
        if let Some(edge_collision_position) = collision_info
            .collision
            .moving_edge_max_collision_position()
        {
            if edge_collision_position.which_part_of_other_edge == StartOrEnd::End {
                let multiplier = 1. - edge_collision_position.how_far_along_this_edge;
                let distance2 = collision_info.moving_edge_vector.vector.magnitude2()
                    * multiplier * multiplier;
                if distance2 <= BUMP_DISTANCE_PX2 {
                    return Some(Bump {
                        distance2: distance2 + EPSILON,
                        direction: -collision_info.moving_edge_vector.vector,
                    });
                }
            }
        }
    }

    None
}

fn bottom_collision(collision_info: &CollisionInfo) -> bool {
    if collision_info.moving_edge_vector.channels & channels::FLOOR != 0 {
        return true;
    }

    if collision_info.moving_edge_vector.flags & flags::FLOOR_START != 0
        && collision_info.collision.moving_start()
    {
        return true;
    }
    if collision_info.moving_edge_vector.flags & flags::FLOOR_END != 0
        && collision_info.collision.moving_end()
    {
        return true;
    }

    false
}

fn jump_predicate(collision_info: &CollisionInfo) -> bool {
    if !bottom_collision(collision_info) {
        return false;
    }

    let dot = collision_info
        .stationary_edge_vector
        .vector
        .normalize()
        .dot(vec2(0., 1.))
        .abs();

    if dot > 0.9 {
        return false;
    }

    true
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

enum MovementState {
    Bump {
        bump_movement: Vector2<f64>,
        original_movement: Vector2<f64>,
    },
    Movement(Vector2<f64>),
}

impl MovementState {
    fn vector(&self) -> Vector2<f64> {
        match self {
            MovementState::Bump { bump_movement, .. } => *bump_movement,
            MovementState::Movement(movement) => *movement,
        }
    }
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
        movement: Vector2<f64>,
        for_each_shape_position: &F,
    ) -> Movement
    where
        F: ForEachShapePosition,
    {
        let mut movement_state = MovementState::Movement(movement);
        let mut can_jump = false;
        let mut position = shape_position.position;
        let mut velocity_correction = vec2(0., 0.);
        const MAX_ITERATIONS: usize = 16;
        for _ in 0..MAX_ITERATIONS {
            let movement_vector = movement_state.vector();
            self.allowed_movement_step(
                ShapePosition {
                    position,
                    ..shape_position
                },
                movement_vector,
                for_each_shape_position,
            );
            let next_movement = match self.closest_collisions.iter().next() {
                None => {
                    position += movement_vector;
                    None
                }
                Some(collision_info) => {
                    position += collision_info
                        .collision
                        .movement_to_collision(movement_vector);

                    if self.closest_collisions.iter().any(jump_predicate) {
                        can_jump = true;
                    }

                    if let Some(max_bump) = self.closest_collisions
                        .iter()
                        .filter_map(bump)
                        .max_by(|a, b| {
                            a.distance2
                                .partial_cmp(&b.distance2)
                                .unwrap_or(Ordering::Less)
                        }) {
                        let bump_movement = max_bump.vector();
                        velocity_correction -= bump_movement;
                        let next_movement = collision_info
                            .collision
                            .movement_following_collision(movement_vector);
                        Some(MovementState::Bump {
                            bump_movement,
                            original_movement: next_movement,
                        })
                    } else {
                        let next_movement =
                            collision_info.collision.slide(movement_vector);
                        Some(MovementState::Movement(next_movement))
                    }
                }
            };
            movement_state = match (next_movement, movement_state) {
                (None, MovementState::Movement(_)) => break,
                (
                    None,
                    MovementState::Bump {
                        original_movement, ..
                    },
                ) => MovementState::Movement(original_movement),
                (Some(movement), _) => movement,
            };
        }
        Movement {
            position,
            velocity: position - shape_position.position + velocity_correction,
            can_jump,
        }
    }
}
