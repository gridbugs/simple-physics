use cgmath::{InnerSpace, Vector2, vec2};
use std::cmp::Ordering;

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
    ParallelNonColinear,
    ColinearNonOverlapping,
    OutsideMovement,
    OutsideEdge,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Collision {
    ColinearCourseAdjustment,
    SlideAlongEdge,
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

    pub fn vertex_collision_edge_is_moving(
        &self,
        vertex: Vector2<f64>,
        edge_movement: Vector2<f64>,
    ) -> CollisionMovement {
        self.vertex_collision_vertex_is_moving(vertex, -edge_movement)
            .reverse()
    }

    pub fn vertex_collision_vertex_is_moving(
        &self,
        vertex: Vector2<f64>,
        vertex_movement: Vector2<f64>,
    ) -> CollisionMovement {
        let edge_vector = self.vector();
        let cross = vector2_cross_product(vertex_movement, edge_vector);

        if cross > EPSILON {
            return CollisionMovement::from_movement_vector(
                vertex_movement,
                NoCollision::ImpossibleDirection,
            );
        }

        let vertex_to_start = self.start - vertex;
        if cross > -EPSILON {
            if vector2_cross_product(vertex_to_start, vertex_movement).abs() > EPSILON {
                return CollisionMovement::from_movement_vector(
                    vertex_movement,
                    NoCollision::ParallelNonColinear,
                );
            }

            let divisor = vertex_movement.dot(vertex_movement);
            let movement_multiplier_a = vertex_to_start.dot(vertex_movement) / divisor;
            let movement_multiplier_b =
                (vertex_to_start + edge_vector).dot(vertex_movement) / divisor;

            let (movement_multiplier_min, movement_multiplier_max) =
                if movement_multiplier_a < movement_multiplier_b {
                    (movement_multiplier_a, movement_multiplier_b)
                } else {
                    (movement_multiplier_b, movement_multiplier_a)
                };

            if movement_multiplier_max < -EPSILON
                || movement_multiplier_min > 1. + EPSILON
            {
                return CollisionMovement::from_movement_vector(
                    vertex_movement,
                    NoCollision::ColinearNonOverlapping,
                );
            }

            let edge_direction = edge_vector.normalize();
            let corrected_movement = vertex_movement.project_on(edge_direction);
            let corrected_vertex =
                self.start - vertex_to_start.project_on(edge_direction);
            let corrected_destination = corrected_vertex + corrected_movement;
            let actual_movement = (corrected_destination - vertex)
                .normalize_to(vertex_movement.magnitude() - EPSILON);
            let movement = CollisionMovement {
                movement: MovementWithSlide {
                    movement: Vector2WithMagnitude2::from_vector(actual_movement),
                    slide: Vector2WithMagnitude2::zero(),
                },
                collision: Ok(Collision::ColinearCourseAdjustment),
                what_moved: WhatMoved::Vertex,
            };
            return movement;
        }

        let edge_vertex_multiplier =
            vector2_cross_product(vertex_to_start, vertex_movement) / cross;
        if edge_vertex_multiplier < -EPSILON || edge_vertex_multiplier > 1. + EPSILON {
            return CollisionMovement::from_movement_vector(
                vertex_movement,
                NoCollision::OutsideEdge,
            );
        }

        let edge_direction = edge_vector.normalize();

        let movement_multiplier =
            vector2_cross_product(vertex_to_start, edge_vector) / cross;

        if movement_multiplier < EPSILON && movement_multiplier > -EPSILON {
            return CollisionMovement {
                movement: MovementWithSlide {
                    movement: Vector2WithMagnitude2::zero(),
                    slide: Vector2WithMagnitude2::from_vector(
                        vertex_movement.project_on(edge_direction),
                    ),
                },
                collision: Ok(Collision::SlideAlongEdge),
                what_moved: WhatMoved::Vertex,
            };
        }

        if movement_multiplier < -EPSILON || movement_multiplier > 1. + EPSILON {
            return CollisionMovement::from_movement_vector(
                vertex_movement,
                NoCollision::OutsideMovement,
            );
        }

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

#[cfg(test)]
mod test {
    use super::*;
    use cgmath::vec2;

    fn edge(s: Vector2<f64>, e: Vector2<f64>) -> LeftSolidEdge {
        LeftSolidEdge::new(s, e)
    }

    const M: f64 = 100.;
    fn mul(x: f64) -> f64 {
        (x * M).floor()
    }

    #[test]
    fn basic_collision() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(5., 5.);
        let m = vec2(-10., 0.);
        let c = e.vertex_collision_vertex_is_moving(v, m);
        let movement = c.movement.movement.vector();
        assert_eq!(mul(movement.x), mul(-5.));
        assert_eq!(mul(movement.y), mul(0.));
    }

    #[test]
    fn basic_collision_wrong_direction() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(-5., 5.);
        let m = vec2(10., 0.);
        assert_eq!(
            e.vertex_collision_vertex_is_moving(v, m)
                .collision
                .unwrap_err(),
            NoCollision::ImpossibleDirection
        );
    }

    #[test]
    fn basic_collision_missed() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(5., 15.);
        let m = vec2(-10., 0.);
        match e.vertex_collision_vertex_is_moving(v, m)
            .collision
            .unwrap_err()
        {
            NoCollision::OutsideEdge => (),
            _ => panic!(),
        }
    }

    #[test]
    fn parallel() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(1., 0.);
        let m = vec2(0., 1.);
        match e.vertex_collision_vertex_is_moving(v, m)
            .collision
            .unwrap_err()
        {
            NoCollision::ParallelNonColinear => (),
            _ => panic!(),
        }
    }

    #[test]
    fn colinear() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(0., 11.);
        let m = vec2(0., -4.);
        let c = e.vertex_collision_vertex_is_moving(v, m);
        let movement = c.movement.movement.vector();
        assert_eq!(mul(movement.x), mul(0.));
        assert_eq!(mul(movement.y), mul(-4.));
    }

    #[test]
    fn slide() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(2., 0.);
        let m = vec2(-4., 4.);
        let c = e.vertex_collision_vertex_is_moving(v, m);
        let movement = c.movement.movement.vector();
        let slide = c.movement.slide.vector();
        assert_eq!(mul(movement.x), mul(-2.));
        assert_eq!(mul(movement.y), mul(2.));
        assert_eq!(mul(slide.x), mul(0.));
        assert_eq!(mul(slide.y), mul(2.));
    }

    #[test]
    fn start_on_edge() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(0., 5.);
        let m = vec2(-4., 4.);
        let c = e.vertex_collision_vertex_is_moving(v, m);
        let movement = c.movement.movement.vector();
        let slide = c.movement.slide.vector();
        assert_eq!(mul(movement.x), mul(0.));
        assert_eq!(mul(movement.y), mul(0.));
        assert_eq!(mul(slide.x), mul(0.));
        assert_eq!(mul(slide.y), mul(4.));
    }

    #[test]
    fn escape_from_edge() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(0., 5.);
        let m = vec2(4., 4.);
        assert_eq!(
            e.vertex_collision_vertex_is_moving(v, m)
                .collision
                .unwrap_err(),
            NoCollision::ImpossibleDirection
        );
    }

    #[test]
    fn edge_is_moving() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(5., 5.);
        let m = vec2(10., 2.);
        let c = e.vertex_collision_edge_is_moving(v, m);
        let movement = c.movement.movement.vector();
        let slide = c.movement.slide.vector();
        assert_eq!(mul(movement.x), mul(5.));
        assert_eq!(mul(movement.y), mul(1.));
        assert_eq!(mul(slide.x), mul(0.));
        assert_eq!(mul(slide.y), mul(1.));
    }
}
