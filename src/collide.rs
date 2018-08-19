use aabb::Aabb;
use best::BestMultiSet;
use cgmath::Vector2;
use left_solid_edge::{LeftSolidEdge, LeftSolidEdgeCollision};
use movement::EntityId;
use std::cmp::Ordering;

const EPSILON: f64 = 0.001;

#[derive(Debug)]
pub struct Collision {
    pub left_solid_edge_collision: LeftSolidEdgeCollision,
    pub moving_edge_vector: EdgeVector,
    pub stationary_edge_vector: EdgeVector,
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
    pub const BUMP_END: Flags = 1 << 2;
    pub const BUMP_START: Flags = 1 << 3;
}

#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub left_solid_edge: LeftSolidEdge,
    pub channels: Channels,
    pub flags: Flags,
}

#[derive(Debug)]
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

pub struct CollidePosition<'a, C: 'a + Collide> {
    pub collide: &'a C,
    pub position: Vector2<f64>,
    pub entity_id: EntityId,
}

impl<'a, C: Collide> CollidePosition<'a, C> {
    fn for_each_movement_collision<Stationary, F>(
        &self,
        stationary: CollidePosition<Stationary>,
        movement: Vector2<f64>,
        mut f: F,
    ) where
        Stationary: Collide,
        F: FnMut(Collision),
    {
        self.collide
            .for_each_left_solid_edge_facing(movement, |moving_rel_edge| {
                let moving_edge = moving_rel_edge
                    .left_solid_edge
                    .add_vector(self.position);
                stationary.collide.for_each_left_solid_edge_facing(
                    -movement,
                    |stationary_rel_edge| {
                        if moving_rel_edge.channels & stationary_rel_edge.channels == 0 {
                            return;
                        }
                        let stationary_edge = stationary_rel_edge
                            .left_solid_edge
                            .add_vector(stationary.position);
                        if let Some(left_solid_edge_collision) = moving_edge
                            .collide_with_stationary_edge(&stationary_edge, movement)
                        {
                            let collision_info = Collision {
                                left_solid_edge_collision,
                                moving_edge_vector: moving_rel_edge.edge_vector(),
                                stationary_edge_vector: stationary_rel_edge.edge_vector(),
                            };
                            f(collision_info);
                        }
                    },
                );
            });
    }

    pub fn movement_collision_test<Stationary>(
        &self,
        stationary: CollidePosition<Stationary>,
        movement: Vector2<f64>,
        closest_collisions: &mut BestMultiSet<Collision>,
    ) where
        Stationary: Collide,
    {
        self.for_each_movement_collision(stationary, movement, |collision_movement| {
            closest_collisions.insert_lt(collision_movement);
        });
    }
}

pub trait Collide {
    fn aabb(&self, top_left: Vector2<f64>) -> Aabb;
    fn for_each_left_solid_edge_facing<F: FnMut(Edge)>(
        &self,
        direction: Vector2<f64>,
        f: F,
    );

    fn for_each_movement_collision<Stationary, F>(
        &self,
        position: Vector2<f64>,
        stationary_shape: &Stationary,
        stationary_position: Vector2<f64>,
        movement: Vector2<f64>,
        mut f: F,
    ) where
        Self: Sized,
        Stationary: Collide,
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
                    if let Some(left_solid_edge_collision) = moving_edge
                        .collide_with_stationary_edge(&stationary_edge, movement)
                    {
                        let collision = Collision {
                            left_solid_edge_collision,
                            moving_edge_vector: moving_rel_edge.edge_vector(),
                            stationary_edge_vector: stationary_rel_edge.edge_vector(),
                        };
                        f(collision);
                    }
                },
            );
        });
    }
    fn movement_collision_test<Stationary>(
        &self,
        position: Vector2<f64>,
        stationary_shape: &Stationary,
        stationary_position: Vector2<f64>,
        movement: Vector2<f64>,
        closest_collisions: &mut BestMultiSet<Collision>,
    ) where
        Self: Sized,
        Stationary: Collide,
    {
        self.for_each_movement_collision(
            position,
            stationary_shape,
            stationary_position,
            movement,
            |collision| {
                closest_collisions.insert_lt_by(collision, |a, b| {
                    let delta = a.left_solid_edge_collision.movement_multiplier()
                        - b.left_solid_edge_collision.movement_multiplier();
                    if delta.abs() < EPSILON {
                        Ordering::Equal
                    } else if delta > 0. {
                        Ordering::Greater
                    } else {
                        Ordering::Less
                    }
                });
            },
        );
    }
}
