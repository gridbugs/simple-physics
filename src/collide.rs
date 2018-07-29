use aabb::Aabb;
use best::BestMultiSet;
use cgmath::Vector2;
use left_solid_edge::{EdgeVertexCollisions, LeftSolidEdge};
use std::cmp::Ordering;

pub struct CollisionInfo {
    collision: EdgeVertexCollisions,
    moving_edge: Edge,
    stationary_edge: Edge,
}

impl PartialEq for CollisionInfo {
    fn eq(&self, rhs: &Self) -> bool {
        self.collision.eq(&rhs.collision)
    }
}
impl PartialOrd for CollisionInfo {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        self.collision.partial_cmp(&rhs.collision)
    }
}

impl CollisionInfo {
    pub fn movement_to_collision(&self, movement_attempt: Vector2<f64>) -> Vector2<f64> {
        self.collision.movement_to_collision(movement_attempt)
    }

    pub fn slide(&self, movement_attempt: Vector2<f64>) -> Vector2<f64> {
        self.collision.slide(movement_attempt)
    }

    pub fn moving_edge(&self) -> Edge {
        self.moving_edge
    }

    pub fn stationary_edge(&self) -> Edge {
        self.stationary_edge
    }
}

pub mod channels {
    pub const MAIN: u32 = 1 << 0;
    pub const FLOOR: u32 = 1 << 1;
}

#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub left_solid_edge: LeftSolidEdge,
    pub channels: u32,
}

impl Edge {
    pub fn new(start: Vector2<f64>, end: Vector2<f64>) -> Self {
        Edge {
            left_solid_edge: LeftSolidEdge::new(start, end),
            channels: channels::MAIN,
        }
    }
    pub fn with_channels(self, channels: u32) -> Self {
        Self { channels, ..self }
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
        F: FnMut(CollisionInfo),
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
                        let collision_info = CollisionInfo {
                            collision: collision_movement,
                            moving_edge: moving_rel_edge,
                            stationary_edge: stationary_rel_edge,
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
        closest_collisions: &mut BestMultiSet<CollisionInfo>,
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
