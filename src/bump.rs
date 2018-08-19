use cgmath::{InnerSpace, Vector2};
use collide::{flags, Collision};
use left_solid_edge::StartOrEnd;
use movement::ClosestCollisions;
use std::cmp::Ordering;

const EPSILON: f64 = 0.01;
const BUMP_DISTANCE_PX: f64 = 2.;
const BUMP_DISTANCE_PX2: f64 = BUMP_DISTANCE_PX * BUMP_DISTANCE_PX;

pub struct Bump {
    distance2: f64,
    direction: Vector2<f64>,
}

impl Bump {
    pub fn vector(&self) -> Vector2<f64> {
        self.direction.normalize_to(self.distance2.sqrt())
    }
}

pub fn max_bump(closest_collisions: ClosestCollisions) -> Option<Bump> {
    closest_collisions
        .iter()
        .filter_map(bump)
        .max_by(|a, b| {
            a.distance2
                .partial_cmp(&b.distance2)
                .unwrap_or(Ordering::Less)
        })
}

fn bump(collision_info: &Collision) -> Option<Bump> {
    if collision_info.moving_edge_vector.flags & flags::BUMP_START != 0 {
        if let Some(edge_collision_position) = collision_info
            .left_solid_edge_collision
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
            .left_solid_edge_collision
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
