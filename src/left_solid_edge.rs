use best::BestSet;
use cgmath::{InnerSpace, Vector2};
use std::cmp::Ordering;

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
}

fn vector2_cross_product(v: Vector2<f64>, w: Vector2<f64>) -> f64 {
    v.x * w.y - v.y * w.x
}

pub const EPSILON: f64 = 0.000001;
const START_MIN_MULTIPLIER: f64 = EPSILON;
const START_MAX_MULTIPLIER: f64 = 1. + EPSILON;
const END_MIN_MULTIPLIER: f64 = -EPSILON;
const END_MAX_MULTIPLIER: f64 = 1. - EPSILON;

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
        let stationary_edge_vector = other.vector();
        let stationary_edge_cross =
            vector2_cross_product(movement, stationary_edge_vector);
        if stationary_edge_cross > -EPSILON {
            return None;
        }
        let moving_edge_vector = self.vector();
        let reverse_movement = -movement;
        let moving_edge_cross =
            vector2_cross_product(reverse_movement, moving_edge_vector);
        if moving_edge_cross > -EPSILON {
            return None;
        }
        let mut closest = BestSet::new();
        if let Some(collision) = self.vertex_collision_vertex_is_moving(
            other.start,
            reverse_movement,
            START_MIN_MULTIPLIER,
            START_MAX_MULTIPLIER,
            moving_edge_vector,
            moving_edge_cross,
        ) {
            closest.insert_le(collision);
        }
        if let Some(collision) = self.vertex_collision_vertex_is_moving(
            other.end,
            reverse_movement,
            END_MIN_MULTIPLIER,
            END_MAX_MULTIPLIER,
            moving_edge_vector,
            moving_edge_cross,
        ) {
            closest.insert_le(collision);
        }
        if let Some(collision) = other.vertex_collision_vertex_is_moving(
            self.start,
            movement,
            START_MIN_MULTIPLIER,
            START_MAX_MULTIPLIER,
            stationary_edge_vector,
            stationary_edge_cross,
        ) {
            closest.insert_le(collision);
        }
        if let Some(collision) = other.vertex_collision_vertex_is_moving(
            self.end,
            movement,
            END_MIN_MULTIPLIER,
            END_MAX_MULTIPLIER,
            stationary_edge_vector,
            stationary_edge_cross,
        ) {
            closest.insert_le(collision);
        }
        closest.into_value()
    }

    fn vertex_collision_vertex_is_moving(
        &self,
        vertex: Vector2<f64>,
        vertex_movement: Vector2<f64>,
        min_edge_multiplier: f64,
        max_edge_multiplier: f64,
        edge_vector: Vector2<f64>,
        cross: f64,
    ) -> Option<CollisionWithSlide> {
        let vertex_to_start = self.start - vertex;
        let edge_vector_multiplier =
            vector2_cross_product(vertex_to_start, vertex_movement) / cross;
        if edge_vector_multiplier < min_edge_multiplier
            || edge_vector_multiplier > max_edge_multiplier
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
