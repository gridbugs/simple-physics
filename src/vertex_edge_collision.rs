use cgmath::{InnerSpace, Vector2, vec2};
use line_segment::LineSegment;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionInfo {
    pub magnitude2: f32,
    pub allowed_movement: Vector2<f32>,
    pub slide_movement: Option<Vector2<f32>>,
}

impl CollisionInfo {
    pub fn zero() -> Self {
        Self {
            magnitude2: 0.,
            allowed_movement: vec2(0., 0.),
            slide_movement: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Collision {
    EdgeCollision(CollisionInfo),
    NoMovement,
    StartInsideEdge,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NoCollision {
    ColinearNonOverlapping,
    ParallelNonColinear,
    NonParallelNonIntersecting,
    ZeroLengthEdge,
}

fn vector2_cross_product(v: Vector2<f32>, w: Vector2<f32>) -> f32 {
    v.x * w.y - v.y * w.x
}

pub const EPSILON: f32 = 0.01;
const PADDING: f32 = 0.2;

fn apply_padding(allowed_movement: Vector2<f32>) -> Vector2<f32> {
    if allowed_movement.magnitude() < PADDING {
        vec2(0., 0.)
    } else {
        allowed_movement - allowed_movement.normalize_to(PADDING)
    }
}

#[derive(Debug, Clone, Copy)]
pub enum WhatIsMoving {
    VertexMovingTowardsEdge,
    EdgeMovingTowardsVertex,
}

impl WhatIsMoving {
    fn multiplier(self) -> f32 {
        match self {
            WhatIsMoving::VertexMovingTowardsEdge => 1.,
            WhatIsMoving::EdgeMovingTowardsVertex => -1.,
        }
    }
}

pub fn vertex_edge_collision(
    vertex: Vector2<f32>,
    movement: Vector2<f32>,
    edge: LineSegment,
    what_is_moving: WhatIsMoving,
) -> Result<Collision, NoCollision> {
    if movement.x.abs() < EPSILON && movement.y.abs() < EPSILON {
        return Ok(Collision::NoMovement);
    }
    let edge_vector = edge.vector();
    if edge_vector.x.abs() < EPSILON && edge_vector.y.abs() < EPSILON {
        return Err(NoCollision::ZeroLengthEdge);
    }
    let cross = vector2_cross_product(movement, edge_vector);
    let vertex_to_edge_start = edge.start - vertex;
    let cross_abs = cross.abs();
    if cross_abs < EPSILON {
        if vector2_cross_product(vertex_to_edge_start, movement).abs() < EPSILON {
            let mult_a_x_movement_len2 = vertex_to_edge_start.dot(movement);
            let mult_b_x_movement_len2 = (vertex_to_edge_start + edge_vector).dot(movement);
            let (mult_min_x_movement_len2, mult_max_x_movement_len2) =
                if mult_a_x_movement_len2 < mult_b_x_movement_len2 {
                    (mult_a_x_movement_len2, mult_b_x_movement_len2)
                } else {
                    (mult_b_x_movement_len2, mult_a_x_movement_len2)
                };
            let movement_len2 = movement.magnitude2();
            if mult_max_x_movement_len2 < -EPSILON
                || mult_min_x_movement_len2 > movement_len2 + EPSILON
            {
                return Err(NoCollision::ColinearNonOverlapping);;
            }
            if mult_min_x_movement_len2 < -EPSILON {
                return Ok(Collision::StartInsideEdge);
            }
            let allowed_movement_x_movement_len2 = movement * mult_min_x_movement_len2;
            assert!(movement_len2 != 0.);
            let allowed_movement = allowed_movement_x_movement_len2 / movement_len2;
            let with_padding = apply_padding(allowed_movement);
            Ok(Collision::EdgeCollision(CollisionInfo {
                magnitude2: with_padding.magnitude2(),
                allowed_movement: with_padding * what_is_moving.multiplier(),
                slide_movement: None,
            }))
        } else {
            Err(NoCollision::ParallelNonColinear)
        }
    } else {
        let cross_sign = cross.signum();
        let vertex_multiplier_x_cross = vector2_cross_product(vertex_to_edge_start, edge_vector);
        let vertex_multiplier_x_cross_abs = vertex_multiplier_x_cross * cross_sign;
        if vertex_multiplier_x_cross_abs < EPSILON {
            if vertex_multiplier_x_cross_abs > -EPSILON {
                return Ok(Collision::StartInsideEdge);
            }
            return Err(NoCollision::NonParallelNonIntersecting);
        }
        if vertex_multiplier_x_cross_abs > cross_abs + EPSILON {
            return Err(NoCollision::NonParallelNonIntersecting);
        }
        let edge_multiplier_x_cross = vector2_cross_product(vertex_to_edge_start, movement);
        let edge_multiplier_x_cross_abs = edge_multiplier_x_cross * cross_sign;
        if edge_multiplier_x_cross_abs < -EPSILON {
            return Err(NoCollision::NonParallelNonIntersecting);
        }
        if edge_multiplier_x_cross_abs > cross_abs + EPSILON {
            return Err(NoCollision::NonParallelNonIntersecting);
        }
        let movement_to_intersection_point_x_cross = movement * vertex_multiplier_x_cross;
        assert!(cross != 0.);
        let allowed_movement = movement_to_intersection_point_x_cross / cross;
        let edge_direction = edge_vector.normalize();
        let with_padding = apply_padding(allowed_movement);
        let remaining_movement = movement - with_padding;
        let slide_movement = remaining_movement.project_on(edge_direction);
        let multiplier = what_is_moving.multiplier();
        Ok(Collision::EdgeCollision(CollisionInfo {
            magnitude2: with_padding.magnitude2(),
            allowed_movement: with_padding * multiplier,
            slide_movement: Some(slide_movement * multiplier),
        }))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use cgmath::vec2;

    fn v(x: f32, y: f32) -> Vector2<f32> {
        vec2(x, y)
    }
    fn ls(start: Vector2<f32>, end: Vector2<f32>) -> LineSegment {
        LineSegment::new(start, end)
    }
    fn unwrap_collision(collision: Collision) -> Vector2<f32> {
        match collision {
            Collision::EdgeCollision(CollisionInfo {
                allowed_movement, ..
            }) => allowed_movement,
            _ => panic!(),
        }
    }

    fn assert_veq(v: Vector2<f32>, w: Vector2<f32>) {
        const MULT: f32 = 1000.;
        assert_eq!((v.x * MULT).floor(), (w.x * MULT).floor());
        assert_eq!((v.y * MULT).floor(), (w.y * MULT).floor());
    }

    fn vertex_moving_towards_edge(
        vertex: Vector2<f32>,
        movement: Vector2<f32>,
        edge: LineSegment,
    ) -> Result<Collision, NoCollision> {
        vertex_edge_collision(
            vertex,
            movement,
            edge,
            WhatIsMoving::VertexMovingTowardsEdge,
        )
    }

    fn edge_moving_towards_vertex(
        vertex: Vector2<f32>,
        movement: Vector2<f32>,
        edge: LineSegment,
    ) -> Result<Collision, NoCollision> {
        vertex_edge_collision(
            vertex,
            movement,
            edge,
            WhatIsMoving::EdgeMovingTowardsVertex,
        )
    }

    #[test]
    fn basic() {
        {
            assert_veq(
                unwrap_collision(
                    vertex_moving_towards_edge(v(0., 0.), v(3., 3.), ls(v(0., 4.), v(4., 0.)))
                        .unwrap(),
                ),
                v(2., 2.) + v(-1., -1.).normalize_to(PADDING),
            );
        }
        assert_eq!(
            vertex_moving_towards_edge(v(0., 0.), v(2., 2.), ls(v(0., 5.), v(5., 0.))),
            Err(NoCollision::NonParallelNonIntersecting)
        );
        assert_eq!(
            vertex_moving_towards_edge(v(2., 3.), v(2., 2.), ls(v(0., 5.), v(5., 0.))),
            Ok(Collision::StartInsideEdge)
        );
        assert_eq!(
            vertex_moving_towards_edge(v(-1., -4.), v(2., 0.), ls(v(0., 0.), v(0., 4.))),
            Err(NoCollision::NonParallelNonIntersecting)
        );
    }

    #[test]
    fn parallel() {
        assert_eq!(
            vertex_moving_towards_edge(v(0., 0.), v(2., 1.), ls(v(1., 1.), v(3., 2.))),
            Err(NoCollision::ParallelNonColinear)
        );
        assert_eq!(
            vertex_moving_towards_edge(v(0., 0.), v(2., 1.), ls(v(4., 2.), v(8., 4.))),
            Err(NoCollision::ColinearNonOverlapping)
        );
        assert_veq(
            unwrap_collision(
                vertex_moving_towards_edge(v(0., 0.), v(2., 1.), ls(v(2., 1.), v(8., 4.))).unwrap(),
            ),
            v(2., 1.) - v(2., 1.).normalize_to(PADDING),
        );
        assert_eq!(
            vertex_moving_towards_edge(v(2., 1.), v(2., 1.), ls(v(0., 0.), v(8., 4.))),
            Ok(Collision::StartInsideEdge)
        );
    }

    #[test]
    fn slide() {
        let f = |ls| {
            let vertex = v(0., 1.);
            let left = v(-10., -2.);
            let right = v(10., -2.);
            let left_collision = vertex_moving_towards_edge(vertex, left, ls).unwrap();
            let right_collision = vertex_moving_towards_edge(vertex, right, ls).unwrap();
            match left_collision {
                Collision::EdgeCollision(CollisionInfo {
                    allowed_movement,
                    slide_movement: Some(slide_movement),
                    ..
                }) => {
                    assert_veq(
                        allowed_movement,
                        v(-5., 0.) + v(0., 1.).normalize_to(PADDING) - vertex,
                    );
                    assert_veq(slide_movement, v(-5., 0.));
                }
                _ => panic!(),
            }
            match right_collision {
                Collision::EdgeCollision(CollisionInfo {
                    allowed_movement,
                    slide_movement: Some(slide_movement),
                    ..
                }) => {
                    assert_veq(
                        allowed_movement,
                        v(5., 0.) + v(0., 1.).normalize_to(PADDING) - vertex,
                    );
                    assert_veq(slide_movement, v(5., 0.));
                }
                _ => panic!(),
            }
        };
        f(ls(v(-10., 0.), v(10., 0.)));
        f(ls(v(10., 0.), v(-10., 0.)));
    }
}
