use aabb::Aabb;
use best::BestMap;
use cgmath::{vec2, InnerSpace, Vector2};
use line_segment::LineSegment;
use vertex_edge_collision::{self, Collision, CollisionInfo, WhatIsMoving};

fn for_each_single_direction_collision<A, B, F>(
    shape: &A,
    position: Vector2<f32>,
    other_shape: &B,
    other_position: Vector2<f32>,
    movement: Vector2<f32>,
    reverse_movement: Vector2<f32>,
    f: &mut F,
    what_is_moving: WhatIsMoving,
) where
    A: Collide,
    B: Collide,
    F: FnMut(Collision),
{
    shape.for_each_vertex_facing(movement, |rel_vertex| {
        let abs_vertex = rel_vertex + position;
        other_shape.for_each_edge_facing(reverse_movement, |rel_edge| {
            let abs_edge = rel_edge.add_vector(other_position);
            match vertex_edge_collision::vertex_edge_collision(
                abs_vertex,
                movement,
                abs_edge,
                what_is_moving,
            ) {
                Ok(collision) => f(collision),
                Err(_) => (),
            }
        });
    });
}

pub trait Collide {
    fn aabb(&self, top_left: Vector2<f32>) -> Aabb;
    fn for_each_edge_facing<F: FnMut(LineSegment)>(&self, direction: Vector2<f32>, f: F);
    fn for_each_vertex_facing<F: FnMut(Vector2<f32>)>(&self, direction: Vector2<f32>, f: F);
    fn for_each_movement_collision<StationaryShape, F>(
        &self,
        position: Vector2<f32>,
        stationary_shape: &StationaryShape,
        stationary_position: Vector2<f32>,
        movement: Vector2<f32>,
        mut f: F,
    ) where
        Self: Sized,
        StationaryShape: Collide,
        F: FnMut(Collision),
    {
        let reverse_movement = -movement;
        for_each_single_direction_collision(
            self,
            position,
            stationary_shape,
            stationary_position,
            movement,
            reverse_movement,
            &mut f,
            WhatIsMoving::VertexMovingTowardsEdge,
        );
        for_each_single_direction_collision(
            stationary_shape,
            stationary_position,
            self,
            position,
            reverse_movement,
            movement,
            &mut f,
            WhatIsMoving::EdgeMovingTowardsVertex,
        );
    }
    fn movement_collision_test<StationaryShape>(
        &self,
        position: Vector2<f32>,
        stationary_shape: &StationaryShape,
        stationary_position: Vector2<f32>,
        movement: Vector2<f32>,
    ) -> Option<CollisionInfo>
    where
        Self: Sized,
        StationaryShape: Collide,
    {
        let mut shortest_movement = BestMap::new();
        self.for_each_movement_collision(
            position,
            stationary_shape,
            stationary_position,
            movement,
            |collision| match collision {
                Collision::StartInsideEdge => (),
                Collision::EdgeCollision(info) => {
                    shortest_movement.insert_le(info.magnitude2, info);
                }
            },
        );
        shortest_movement.into_value()
    }
}
