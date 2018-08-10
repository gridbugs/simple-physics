use aabb::Aabb;
use best::BestMultiSet;
use cgmath::Vector2;
use left_solid_edge::{LeftSolidEdge, LeftSolidEdgeCollision};
use std::cmp::Ordering;

pub struct Collision {
    pub collision: LeftSolidEdgeCollision,
    pub moving_edge_vector: EdgeVector,
    pub stationary_edge_vector: EdgeVector,
}

impl PartialEq for Collision {
    fn eq(&self, rhs: &Self) -> bool {
        self.collision.eq(&rhs.collision)
    }
}
impl PartialOrd for Collision {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        self.collision.partial_cmp(&rhs.collision)
    }
}

pub type Flags = u32;
pub type Channels = u32;

pub mod channels {
    use super::*;
    pub const MAIN: Channels = 1 << 0;
    pub const FLOOR: Channels = 1 << 1;
}

pub mod flags {
    use super::*;
    pub const FLOOR_END: Flags = 1 << 0;
    pub const FLOOR_START: Flags = 1 << 1;
    pub const BUMP_END: Flags = 1 << 2;
    pub const BUMP_START: Flags = 1 << 3;
}

#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub left_solid_edge: LeftSolidEdge,
    pub channels: Channels,
    pub flags: Flags,
}

pub struct EdgeVector {
    pub vector: Vector2<f64>,
    pub channels: Channels,
    pub flags: Flags,
}

impl Edge {
    pub fn new(start: Vector2<f64>, end: Vector2<f64>) -> Self {
        Edge {
            left_solid_edge: LeftSolidEdge::new(start, end),
            channels: channels::MAIN,
            flags: 0,
        }
    }
    pub fn with_channels(self, channels: Channels) -> Self {
        Self { channels, ..self }
    }
    pub fn with_flags(self, flags: Flags) -> Self {
        Self { flags, ..self }
    }
    pub fn start(&self) -> Vector2<f64> {
        self.left_solid_edge.start
    }
    pub fn end(&self) -> Vector2<f64> {
        self.left_solid_edge.end
    }
    pub fn vector(&self) -> Vector2<f64> {
        self.left_solid_edge.vector()
    }
    pub fn edge_vector(&self) -> EdgeVector {
        EdgeVector {
            vector: self.vector(),
            channels: self.channels,
            flags: self.flags,
        }
    }
}

pub trait Collide {
    fn aabb(&self, top_left: Vector2<f64>) -> Aabb;
    fn for_each_left_solid_edge_facing<F: FnMut(Edge)>(
        &self,
        direction: Vector2<f64>,
        f: F,
    );

    fn for_each_movement_collision<StationaryShape, F>(
        &self,
        position: Vector2<f64>,
        stationary_shape: &StationaryShape,
        stationary_position: Vector2<f64>,
        movement: Vector2<f64>,
        mut f: F,
    ) where
        Self: Sized,
        StationaryShape: Collide,
        F: FnMut(Collision),
    {
        self.for_each_left_solid_edge_facing(movement, |moving_rel_edge| {
            let moving_edge = moving_rel_edge.left_solid_edge.add_vector(position);
            stationary_shape.for_each_left_solid_edge_facing(
                -movement,
                |stationary_rel_edge| {
                    if moving_rel_edge.channels & stationary_rel_edge.channels == 0 {
                        return;
                    }
                    let stationary_edge = stationary_rel_edge
                        .left_solid_edge
                        .add_vector(stationary_position);
                    if let Some(collision_movement) = moving_edge
                        .collide_with_stationary_edge(&stationary_edge, movement)
                    {
                        let collision_info = Collision {
                            collision: collision_movement,
                            moving_edge_vector: moving_rel_edge.edge_vector(),
                            stationary_edge_vector: stationary_rel_edge.edge_vector(),
                        };
                        f(collision_info);
                    }
                },
            );
        });
    }
    fn movement_collision_test<StationaryShape>(
        &self,
        position: Vector2<f64>,
        stationary_shape: &StationaryShape,
        stationary_position: Vector2<f64>,
        movement: Vector2<f64>,
        closest_collisions: &mut BestMultiSet<Collision>,
    ) where
        Self: Sized,
        StationaryShape: Collide,
    {
        self.for_each_movement_collision(
            position,
            stationary_shape,
            stationary_position,
            movement,
            |collision_movement| {
                closest_collisions.insert_lt(collision_movement);
            },
        );
    }
}
