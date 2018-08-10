use cgmath::{InnerSpace, Vector2};
use std::cmp::Ordering;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeftSolidEdge {
    pub start: Vector2<f64>,
    pub end: Vector2<f64>,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum StartOrEnd {
    Start,
    End,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct EdgeCollisionPosition {
    pub which_part_of_other_edge: StartOrEnd,
    pub how_far_along_this_edge: f64,
}

#[derive(Debug, PartialEq, Clone, Copy)]
enum EdgeCollisions {
    Zero,
    One(EdgeCollisionPosition),
    Two {
        min: EdgeCollisionPosition,
        max: EdgeCollisionPosition,
    },
}

impl EdgeCollisions {
    fn new(
        other_edge_start: &Option<VertexCollision>,
        other_edge_end: &Option<VertexCollision>,
    ) -> Self {
        match (other_edge_start, other_edge_end) {
            (None, None) => EdgeCollisions::Zero,
            (Some(start), None) => EdgeCollisions::One(EdgeCollisionPosition {
                which_part_of_other_edge: StartOrEnd::Start,
                how_far_along_this_edge: start.edge_vector_multiplier,
            }),
            (None, Some(end)) => EdgeCollisions::One(EdgeCollisionPosition {
                which_part_of_other_edge: StartOrEnd::End,
                how_far_along_this_edge: end.edge_vector_multiplier,
            }),
            (Some(start), Some(end)) => EdgeCollisions::Two {
                min: EdgeCollisionPosition {
                    which_part_of_other_edge: StartOrEnd::End,
                    how_far_along_this_edge: end.edge_vector_multiplier,
                },
                max: EdgeCollisionPosition {
                    which_part_of_other_edge: StartOrEnd::Start,
                    how_far_along_this_edge: start.edge_vector_multiplier,
                },
            },
        }
    }

    fn min_edge_collision_position(&self) -> Option<EdgeCollisionPosition> {
        match self {
            EdgeCollisions::Zero => None,
            EdgeCollisions::One(m) => Some(*m),
            EdgeCollisions::Two { min, .. } => Some(*min),
        }
    }
    fn max_edge_collision_position(&self) -> Option<EdgeCollisionPosition> {
        match self {
            EdgeCollisions::Zero => None,
            EdgeCollisions::One(m) => Some(*m),
            EdgeCollisions::Two { max, .. } => Some(*max),
        }
    }
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct LeftSolidEdgeCollision {
    moving_edge_collisions: EdgeCollisions,
    stationary_edge_collisions: EdgeCollisions,
    movement_multiplier: f64,
    edge_vector: Vector2<f64>,
}

impl PartialOrd for LeftSolidEdgeCollision {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        self.movement_multiplier
            .abs()
            .partial_cmp(&rhs.movement_multiplier.abs())
    }
}

impl LeftSolidEdgeCollision {
    pub fn movement_to_collision(&self, movement_attempt: Vector2<f64>) -> Vector2<f64> {
        movement_attempt * self.movement_multiplier
    }
    pub fn movement_following_collision(
        &self,
        movement_attempt: Vector2<f64>,
    ) -> Vector2<f64> {
        movement_attempt * (1. - self.movement_multiplier)
    }
    pub fn slide(&self, movement_attempt: Vector2<f64>) -> Vector2<f64> {
        self.movement_following_collision(movement_attempt)
            .project_on(self.edge_vector)
    }
    pub fn moving_edge_min_collision_position(&self) -> Option<EdgeCollisionPosition> {
        self.moving_edge_collisions
            .min_edge_collision_position()
    }
    pub fn moving_edge_max_collision_position(&self) -> Option<EdgeCollisionPosition> {
        self.moving_edge_collisions
            .max_edge_collision_position()
    }
}

const MOVING_START: usize = 0;
const MOVING_END: usize = 1;
const STATIONARY_START: usize = 2;
const STATIONARY_END: usize = 3;

#[derive(Debug, Clone, Copy)]
struct VertexCollision {
    movement_multiplier: f64,
    edge_vector_multiplier: f64,
    edge_vector: Vector2<f64>,
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

    fn collide_moving_vertex(
        &self,
        vertex: Vector2<f64>,
        vertex_movement: Vector2<f64>,
        edge_multipliers: Multipliers,
        evc: EdgeVectorAndCross,
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
        })
    }

    pub fn collide_with_stationary_edge(
        &self,
        other: &Self,
        movement: Vector2<f64>,
    ) -> Option<LeftSolidEdgeCollision> {
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
            ),
            self.collide_moving_vertex(
                other.end,
                reverse_movement,
                END_MULTIPLIERS,
                moving,
            ),
            other.collide_moving_vertex(
                self.start,
                movement,
                START_MULTIPLIERS,
                stationary,
            ),
            other.collide_moving_vertex(self.end, movement, END_MULTIPLIERS, stationary),
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

        let moving_edge_collisions = EdgeCollisions::new(&moving_start, &moving_end);
        let stationary_edge_collisions =
            EdgeCollisions::new(&stationary_start, &stationary_end);

        Some(LeftSolidEdgeCollision {
            moving_edge_collisions,
            stationary_edge_collisions,
            movement_multiplier: min_movement,
            edge_vector,
        })
    }
}
