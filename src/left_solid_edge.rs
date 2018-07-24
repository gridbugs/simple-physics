use cgmath::{InnerSpace, Vector2, vec2};
use std::cmp::Ordering;
use best::BestSetNonEmpty;

pub const EPSILON: f64 = 0.000001;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeftSolidEdge {
    pub start: Vector2<f64>,
    pub end: Vector2<f64>,
}

#[derive(Debug, PartialEq)]
pub struct Vector2WithMagnitude2 {
    vector: Vector2<f64>,
    magnitude2: f64,
}

impl Vector2WithMagnitude2 {
    fn zero() -> Self {
        Self {
            vector: vec2(0., 0.),
            magnitude2: 0.,
        }
    }
    fn from_vector(vector: Vector2<f64>) -> Self {
        Self {
            vector,
            magnitude2: vector.magnitude2(),
        }
    }
    fn reverse(&self) -> Self {
        Self {
            vector: -self.vector,
            magnitude2: self.magnitude2,
        }
    }
    pub fn vector(&self) -> Vector2<f64> {
        self.vector
    }
    pub fn magnitude2(&self) -> f64 {
        self.magnitude2
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum NoCollision {
    ImpossibleDirection,
    OutsideMovement,
    OutsideEdge,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Collision {
    MoveUntilEdgeThenSlide,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum WhatMoved {
    Vertex,
    Edge,
}

#[derive(Debug, PartialEq)]
pub struct MovementWithSlide {
    pub movement: Vector2WithMagnitude2,
    pub slide: Vector2WithMagnitude2,
}
impl MovementWithSlide {
    pub fn new_just_movement(movement: Vector2<f64>) -> Self {
        Self {
            movement: Vector2WithMagnitude2::from_vector(movement),
            slide: Vector2WithMagnitude2::zero(),
        }
    }
    fn reverse(&self) -> Self {
        Self {
            movement: self.movement.reverse(),
            slide: self.slide.reverse(),
        }
    }
}
impl PartialOrd for MovementWithSlide {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        self.movement
            .magnitude2
            .partial_cmp(&rhs.movement.magnitude2)
    }
}

#[derive(Debug, PartialEq)]
pub struct CollisionMovement {
    pub movement: MovementWithSlide,
    pub collision: Result<Collision, NoCollision>,
    pub what_moved: WhatMoved,
}

impl PartialOrd for CollisionMovement {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        match self.movement
            .movement
            .magnitude2
            .partial_cmp(&rhs.movement.movement.magnitude2)
        {
            Some(Ordering::Equal) => rhs.movement
                .slide
                .magnitude2
                .partial_cmp(&self.movement.slide.magnitude2),
            other => other,
        }
    }
}

impl CollisionMovement {
    fn from_movement_vector(movement: Vector2<f64>, no_collision: NoCollision) -> Self {
        let movement = MovementWithSlide {
            movement: Vector2WithMagnitude2::from_vector(movement),
            slide: Vector2WithMagnitude2::zero(),
        };
        Self {
            movement,
            collision: Err(no_collision),
            what_moved: WhatMoved::Vertex,
        }
    }
    fn reverse(&self) -> Self {
        Self {
            movement: self.movement.reverse(),
            collision: self.collision,
            what_moved: WhatMoved::Edge,
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
    ) -> CollisionMovement {
        let moving_edge_vector = self.vector();
        let moving_edge_vector_cross_movement =
            vector2_cross_product(moving_edge_vector, movement);
        if moving_edge_vector_cross_movement > -EPSILON {
            return CollisionMovement::from_movement_vector(
                movement,
                NoCollision::ImpossibleDirection,
            );
        }
        let stationary_edge_vector = other.vector();
        let stationary_edge_vector_cross_movement =
            vector2_cross_product(stationary_edge_vector, movement);
        if stationary_edge_vector_cross_movement < EPSILON {
            return CollisionMovement::from_movement_vector(
                movement,
                NoCollision::ImpossibleDirection,
            );
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

        let mut closest = BestSetNonEmpty::new(a);
        closest.insert_le(b);
        closest.insert_le(c);
        closest.insert_le(d);
        closest.into_value()
    }

    fn vertex_collision_edge_is_moving(
        &self,
        vertex: Vector2<f64>,
        edge_movement: Vector2<f64>,
        which_end: WhichEnd,
    ) -> CollisionMovement {
        self.vertex_collision_vertex_is_moving(vertex, -edge_movement, which_end)
            .reverse()
    }

    fn vertex_collision_vertex_is_moving(
        &self,
        vertex: Vector2<f64>,
        vertex_movement: Vector2<f64>,
        which_end: WhichEnd,
    ) -> CollisionMovement {
        let edge_vector = self.vector();
        let cross = vector2_cross_product(vertex_movement, edge_vector);

        let vertex_to_start = self.start - vertex;

        let edge_vertex_multiplier =
            vector2_cross_product(vertex_to_start, vertex_movement) / cross;
        let (min_multiplier, max_multiplier) = match which_end {
            WhichEnd::Start => (EPSILON, 1. + EPSILON),
            WhichEnd::End => (-EPSILON, 1. - EPSILON),
        };
        if edge_vertex_multiplier < min_multiplier
            || edge_vertex_multiplier > max_multiplier
        {
            return CollisionMovement::from_movement_vector(
                vertex_movement,
                NoCollision::OutsideEdge,
            );
        }

        let edge_direction = edge_vector.normalize();

        let movement_multiplier =
            vector2_cross_product(vertex_to_start, edge_vector) / cross;

        if movement_multiplier < -EPSILON || movement_multiplier > 1. + EPSILON {
            return CollisionMovement::from_movement_vector(
                vertex_movement,
                NoCollision::OutsideMovement,
            );
        }

        let movement_multiplier = (movement_multiplier).max(0.);
        let movement = vertex_movement * movement_multiplier;

        let remaining_movement = vertex_movement * (1. - movement_multiplier).max(0.);
        CollisionMovement {
            movement: MovementWithSlide {
                movement: Vector2WithMagnitude2::from_vector(movement),
                slide: Vector2WithMagnitude2::from_vector(
                    remaining_movement.project_on(edge_direction),
                ),
            },
            collision: Ok(Collision::MoveUntilEdgeThenSlide),
            what_moved: WhatMoved::Vertex,
        }
    }
}
