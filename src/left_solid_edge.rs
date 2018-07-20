use cgmath::{vec2, InnerSpace, Vector2};

pub struct LeftSolidEdge {
    start: Vector2<f32>,
    end: Vector2<f32>,
}

pub struct Vector2WithMagnitude2 {
    vector: Vector2<f32>,
    magnitude2: f32,
}

impl Vector2WithMagnitude2 {
    fn zero() -> Self {
        Self {
            vector: vec2(0., 0.),
            magnitude2: 0.,
        }
    }
    fn from_vector(vector: Vector2<f32>) -> Self {
        Self {
            vector,
            magnitude2: vector.magnitude2(),
        }
    }
    pub fn vector(&self) -> Vector2<f32> {
        self.vector
    }
    pub fn magnitude2(&self) -> f32 {
        self.magnitude2
    }
}

pub struct Collision {
    pub allowed_movement: Vector2WithMagnitude2,
    pub slide: Vector2WithMagnitude2,
}

pub trait WhatIsMoving {}

pub struct EdgeIsMoving;
pub struct VertexIsMoving;

impl WhatIsMoving for EdgeIsMoving {}
impl WhatIsMoving for VertexIsMoving {}

pub const EPSILON: f32 = 0.01;

fn vector2_cross_product(v: Vector2<f32>, w: Vector2<f32>) -> f32 {
    v.x * w.y - v.y * w.x
}

impl LeftSolidEdge {
    pub fn new(start: Vector2<f32>, end: Vector2<f32>) -> Self {
        Self { start, end }
    }
    fn vector(&self) -> Vector2<f32> {
        self.end - self.start
    }

    pub fn collision(
        &self,
        vertex: Vector2<f32>,
        vertex_movement: Vector2<f32>,
    ) -> Option<Collision> {
        let edge_vector = self.vector();
        let cross = vector2_cross_product(vertex_movement, edge_vector);

        if cross > EPSILON {
            return None;
        }

        let vertex_to_start = self.start - vertex;
        if cross > -EPSILON {
            if vector2_cross_product(vertex_to_start, vertex_movement).abs() > EPSILON {
                return None;
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
                return None;
            }

            let edge_direction = edge_vector.normalize();
            return Some(Collision {
                allowed_movement: Vector2WithMagnitude2::from_vector(
                    vertex_movement.project_on(edge_direction),
                ),
                slide: Vector2WithMagnitude2::zero(),
            });
        }

        let movement_multiplier =
            vector2_cross_product(vertex_to_start, edge_vector) / cross;
        if movement_multiplier < -EPSILON || movement_multiplier > 1. + EPSILON {
            return None;
        }

        let edge_vertex_multiplier =
            vector2_cross_product(vertex_to_start, vertex_movement) / cross;
        if edge_vertex_multiplier < -EPSILON || edge_vertex_multiplier > 1. + EPSILON {
            return None;
        }

        let edge_direction = edge_vector.normalize();

        if movement_multiplier < EPSILON {
            return Some(Collision {
                allowed_movement: Vector2WithMagnitude2::zero(),
                slide: Vector2WithMagnitude2::from_vector(
                    vertex_movement.project_on(edge_direction),
                ),
            });
        }

        let allowed_movement = vertex_movement * movement_multiplier;
        let remaining_movement = vertex_movement * (1. - movement_multiplier).max(0.);
        Some(Collision {
            allowed_movement: Vector2WithMagnitude2::from_vector(allowed_movement),
            slide: Vector2WithMagnitude2::from_vector(
                remaining_movement.project_on(edge_direction),
            ),
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use cgmath::vec2;

    fn edge(s: Vector2<f32>, e: Vector2<f32>) -> LeftSolidEdge {
        LeftSolidEdge::new(s, e)
    }

    const M: f32 = 100.;
    fn mul(x: f32) -> f32 {
        (x * M).floor()
    }

    #[test]
    fn basic_collision() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(5., 5.);
        let m = vec2(-10., 0.);
        let c = e.collision(v, m).unwrap();
        let allowed_movement = c.allowed_movement.vector();
        assert_eq!(mul(allowed_movement.x), mul(-5.));
        assert_eq!(mul(allowed_movement.y), mul(0.));
    }

    #[test]
    fn basic_collision_wrong_direction() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(-5., 5.);
        let m = vec2(10., 0.);
        assert!(e.collision(v, m).is_none());
    }

    #[test]
    fn basic_collision_missed() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(5., 15.);
        let m = vec2(-10., 0.);
        assert!(e.collision(v, m).is_none());
    }

    #[test]
    fn parallel() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(1., 0.);
        let m = vec2(0., 1.);
        assert!(e.collision(v, m).is_none());
    }

    #[test]
    fn colinear() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(0., 11.);
        let m = vec2(0., -4.);
        let c = e.collision(v, m).unwrap();
        let allowed_movement = c.allowed_movement.vector();
        assert_eq!(mul(allowed_movement.x), mul(0.));
        assert_eq!(mul(allowed_movement.y), mul(-4.));
    }

    #[test]
    fn slide() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(2., 0.);
        let m = vec2(-4., 4.);
        let c = e.collision(v, m).unwrap();
        let allowed_movement = c.allowed_movement.vector();
        let slide = c.slide.vector();
        assert_eq!(mul(allowed_movement.x), mul(-2.));
        assert_eq!(mul(allowed_movement.y), mul(2.));
        assert_eq!(mul(slide.x), mul(0.));
        assert_eq!(mul(slide.y), mul(2.));
    }

    #[test]
    fn start_on_edge() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(0., 5.);
        let m = vec2(-4., 4.);
        let c = e.collision(v, m).unwrap();
        let allowed_movement = c.allowed_movement.vector();
        let slide = c.slide.vector();
        assert_eq!(mul(allowed_movement.x), mul(0.));
        assert_eq!(mul(allowed_movement.y), mul(0.));
        assert_eq!(mul(slide.x), mul(0.));
        assert_eq!(mul(slide.y), mul(4.));
    }

    #[test]
    fn escape_from_edge() {
        let e = edge(vec2(0., 0.), vec2(0., 10.));
        let v = vec2(0., 5.);
        let m = vec2(4., 4.);
        assert!(e.collision(v, m).is_none());
    }
}
