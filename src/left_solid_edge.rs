use cgmath::{InnerSpace, Vector2};
use std::cmp::Ordering;
use best::BestSet;

pub const EPSILON: f64 = 0.000001;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeftSolidEdge {
    pub start: Vector2<f64>,
    pub end: Vector2<f64>,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct CollisionWithSlide {
    movement_multiplier: f64,
    edge_vector: Vector2<f64>,
}

impl PartialOrd for CollisionWithSlide {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        self.movement_multiplier
            .abs()
            .partial_cmp(&rhs.movement_multiplier.abs())
    }
}

impl CollisionWithSlide {
    pub fn movement_to_collision(&self, movement_attempt: Vector2<f64>) -> Vector2<f64> {
        movement_attempt * self.movement_multiplier
    }
    pub fn slide(&self, movement_attempt: Vector2<f64>) -> Vector2<f64> {
        let remaining_movement = movement_attempt * (1. - self.movement_multiplier);
        remaining_movement.project_on(self.edge_vector)
    }
    fn reverse(self) -> Self {
        Self {
            movement_multiplier: self.movement_multiplier,
            edge_vector: -self.edge_vector,
        }
    }
}

pub fn vector2_cross_product(v: Vector2<f64>, w: Vector2<f64>) -> f64 {
    v.x * w.y - v.y * w.x
}

pub enum WhichEnd {
    Start,
    End,
}

impl LeftSolidEdge {
    pub fn new(start: Vector2<f64>, end: Vector2<f64>) -> Self {
        Self { start, end }
    }
    pub fn add_vector(&self, vector: Vector2<f64>) -> Self {
        Self {
            start: self.start + vector,
            end: self.end + vector,
        }
    }

    fn vector(&self) -> Vector2<f64> {
        self.end - self.start
    }

    pub fn collide_with_stationary_edge(
        &self,
        other: &Self,
        movement: Vector2<f64>,
    ) -> Option<CollisionWithSlide> {
        let moving_edge_vector = self.vector();
        let moving_edge_vector_cross_movement =
            vector2_cross_product(moving_edge_vector, movement);
        if moving_edge_vector_cross_movement > -EPSILON {
            return None;
        }
        let stationary_edge_vector = other.vector();
        let stationary_edge_vector_cross_movement =
            vector2_cross_product(stationary_edge_vector, movement);
        if stationary_edge_vector_cross_movement < EPSILON {
            return None;
        }
        let a =
            self.vertex_collision_edge_is_moving(other.start, movement, WhichEnd::Start);
        let b = self.vertex_collision_edge_is_moving(other.end, movement, WhichEnd::End);
        let c = other.vertex_collision_vertex_is_moving(
            self.start,
            movement,
            WhichEnd::Start,
        );
        let d =
            other.vertex_collision_vertex_is_moving(self.end, movement, WhichEnd::End);

        let mut closest = BestSet::new();
        a.map(|a| closest.insert_le(a));
        b.map(|b| closest.insert_le(b));
        c.map(|c| closest.insert_le(c));
        d.map(|d| closest.insert_le(d));
        closest.into_value()
    }

    fn vertex_collision_edge_is_moving(
        &self,
        vertex: Vector2<f64>,
        edge_movement: Vector2<f64>,
        which_end: WhichEnd,
    ) -> Option<CollisionWithSlide> {
        self.vertex_collision_vertex_is_moving(vertex, -edge_movement, which_end)
            .map(|c| c.reverse())
    }

    fn vertex_collision_vertex_is_moving(
        &self,
        vertex: Vector2<f64>,
        vertex_movement: Vector2<f64>,
        which_end: WhichEnd,
    ) -> Option<CollisionWithSlide> {
        let edge_vector = self.vector();
        let cross = vector2_cross_product(vertex_movement, edge_vector);

        let vertex_to_start = self.start - vertex;

        let edge_vector_multiplier =
            vector2_cross_product(vertex_to_start, vertex_movement) / cross;
        let (min_multiplier, max_multiplier) = match which_end {
            WhichEnd::Start => (EPSILON, 1. + EPSILON),
            WhichEnd::End => (-EPSILON, 1. - EPSILON),
        };
        if edge_vector_multiplier < min_multiplier
            || edge_vector_multiplier > max_multiplier
        {
            return None;
        }

        let movement_multiplier =
            vector2_cross_product(vertex_to_start, edge_vector) / cross;

        if movement_multiplier < -EPSILON || movement_multiplier > 1. + EPSILON {
            return None;
        }

        Some(CollisionWithSlide {
            movement_multiplier,
            edge_vector,
        })
    }
}
