use cgmath::{InnerSpace, Vector2};
use std::cmp::Ordering;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeftSolidEdge {
    pub start: Vector2<f64>,
    pub end: Vector2<f64>,
}

#[derive(Debug, PartialEq, Clone, Copy)]
struct CollisionWithSlide {
    movement_multiplier: f64,
    edge_vector: Vector2<f64>,
}

pub enum EdgePosition {
    Zero,
    One(f64),
    Two { min: f64, max: f64 },
}

impl EdgePosition {
    fn new(a: Option<VertexCollision>, b: Option<VertexCollision>) -> Self {
        match (a, b) {
            (None, None) => EdgePosition::Zero,
            (Some(x), None) | (None, Some(x)) => {
                EdgePosition::One(x.edge_vector_multiplier)
            }
            (Some(a), Some(b)) => {
                if a.edge_vector_multiplier < b.edge_vector_multiplier {
                    EdgePosition::Two {
                        min: a.edge_vector_multiplier,
                        max: b.edge_vector_multiplier,
                    }
                } else {
                    EdgePosition::Two {
                        min: b.edge_vector_multiplier,
                        max: a.edge_vector_multiplier,
                    }
                }
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct WhichVertices(u8);
const MOVING_START: usize = 0;
const MOVING_END: usize = 1;
const STATIONARY_START: usize = 2;
const STATIONARY_END: usize = 3;

impl WhichVertices {
    pub fn moving_start(self) -> bool {
        self.0 & (1 << MOVING_START) != 0
    }
    pub fn moving_end(self) -> bool {
        self.0 & (1 << MOVING_END) != 0
    }
    pub fn _stationary_start(self) -> bool {
        self.0 & (1 << STATIONARY_START) != 0
    }
    pub fn _stationary_end(self) -> bool {
        self.0 & (1 << STATIONARY_END) != 0
    }
}

#[derive(Debug, Clone, Copy)]
struct VertexCollision {
    movement_multiplier: f64,
    edge_vector_multiplier: f64,
    edge_vector: Vector2<f64>,
    which_vertices: WhichVertices,
}

pub struct EdgeVertexCollisions {
    pub _stationary_edge_position: EdgePosition,
    pub _moving_edge_position: EdgePosition,
    pub which_vertices: WhichVertices,
    collision_with_slide: CollisionWithSlide,
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

impl EdgeVertexCollisions {
    pub fn movement_to_collision(&self, movement_attempt: Vector2<f64>) -> Vector2<f64> {
        self.collision_with_slide
            .movement_to_collision(movement_attempt)
    }

    pub fn slide(&self, movement_attempt: Vector2<f64>) -> Vector2<f64> {
        self.collision_with_slide.slide(movement_attempt)
    }
}

impl PartialEq for EdgeVertexCollisions {
    fn eq(&self, rhs: &Self) -> bool {
        self.collision_with_slide
            .eq(&rhs.collision_with_slide)
    }
}
impl PartialOrd for EdgeVertexCollisions {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        self.collision_with_slide
            .partial_cmp(&rhs.collision_with_slide)
    }
}

fn vector2_cross_product(v: Vector2<f64>, w: Vector2<f64>) -> f64 {
    v.x * w.y - v.y * w.x
}

#[derive(Debug, Clone, Copy)]
struct Multipliers {
    min: f64,
    max: f64,
}

#[derive(Debug, Clone, Copy)]
struct EdgeVectorAndCross {
    vector: Vector2<f64>,
    cross: f64,
}

impl EdgeVectorAndCross {
    fn new(edge: &LeftSolidEdge, movement: Vector2<f64>) -> Self {
        let vector = edge.vector();
        let cross = vector2_cross_product(movement, vector);
        Self { vector, cross }
    }
}

pub const EPSILON: f64 = 0.000001;

const START_MULTIPLIERS: Multipliers = Multipliers {
    min: EPSILON,
    max: 1. + EPSILON,
};

const END_MULTIPLIERS: Multipliers = Multipliers {
    min: -EPSILON,
    max: 1. - EPSILON,
};

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

    pub fn vector(&self) -> Vector2<f64> {
        self.end - self.start
    }

    pub fn collide_with_stationary_edge(
        &self,
        other: &Self,
        movement: Vector2<f64>,
    ) -> Option<EdgeVertexCollisions> {
        let stationary = EdgeVectorAndCross::new(other, movement);
        if stationary.cross > -EPSILON {
            return None;
        }
        let reverse_movement = -movement;
        let moving = EdgeVectorAndCross::new(self, -movement);
        if moving.cross > -EPSILON {
            return None;
        }
        let vertex_collisions = [
            self.collide_moving_vertex(
                other.start,
                reverse_movement,
                START_MULTIPLIERS,
                moving,
                WhichVertices(1 << STATIONARY_START),
            ),
            self.collide_moving_vertex(
                other.end,
                reverse_movement,
                END_MULTIPLIERS,
                moving,
                WhichVertices(1 << STATIONARY_END),
            ),
            other.collide_moving_vertex(
                self.start,
                movement,
                START_MULTIPLIERS,
                stationary,
                WhichVertices(1 << MOVING_START),
            ),
            other.collide_moving_vertex(
                self.end,
                movement,
                END_MULTIPLIERS,
                stationary,
                WhichVertices(1 << MOVING_END),
            ),
        ];
        let (min_movement, edge_vector) = vertex_collisions
            .iter()
            .filter_map(|c| c.map(|c| (c.movement_multiplier, c.edge_vector)))
            .min_by(|&(ref a, _), &(ref b, _)| {
                a.partial_cmp(b).unwrap_or(Ordering::Equal)
            })?;

        let movement_filter = |c: VertexCollision| {
            if c.movement_multiplier > min_movement {
                None
            } else {
                Some(c)
            }
        };

        let moving_start = vertex_collisions[MOVING_START].and_then(movement_filter);
        let moving_end = vertex_collisions[MOVING_END].and_then(movement_filter);
        let stationary_start =
            vertex_collisions[STATIONARY_START].and_then(movement_filter);
        let stationary_end = vertex_collisions[STATIONARY_END].and_then(movement_filter);

        let _moving_edge_position = EdgePosition::new(moving_start, moving_end);
        let _stationary_edge_position =
            EdgePosition::new(stationary_start, stationary_end);

        let which_vertices = WhichVertices(
            moving_start.map(|c| c.which_vertices.0).unwrap_or(0)
                | moving_end.map(|c| c.which_vertices.0).unwrap_or(0)
                | stationary_start
                    .map(|c| c.which_vertices.0)
                    .unwrap_or(0)
                | stationary_end
                    .map(|c| c.which_vertices.0)
                    .unwrap_or(0),
        );

        let collision_with_slide = CollisionWithSlide {
            movement_multiplier: min_movement,
            edge_vector,
        };

        Some(EdgeVertexCollisions {
            collision_with_slide,
            _moving_edge_position,
            _stationary_edge_position,
            which_vertices,
        })
    }

    fn collide_moving_vertex(
        &self,
        vertex: Vector2<f64>,
        vertex_movement: Vector2<f64>,
        edge_multipliers: Multipliers,
        evc: EdgeVectorAndCross,
        which_vertices: WhichVertices,
    ) -> Option<VertexCollision> {
        let vertex_to_start = self.start - vertex;
        let edge_vector_multiplier =
            vector2_cross_product(vertex_to_start, vertex_movement) / evc.cross;
        if edge_vector_multiplier < edge_multipliers.min
            || edge_vector_multiplier > edge_multipliers.max
        {
            return None;
        }
        let movement_multiplier =
            vector2_cross_product(vertex_to_start, evc.vector) / evc.cross;
        if movement_multiplier < -EPSILON || movement_multiplier > 1. + EPSILON {
            return None;
        }
        Some(VertexCollision {
            movement_multiplier,
            edge_vector_multiplier,
            edge_vector: evc.vector,
            which_vertices,
        })
    }
}
