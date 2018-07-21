use aabb::Aabb;
use best::{BestMap, BestSet};
use cgmath::Vector2;
use left_solid_edge::{CollisionMovement, LeftSolidEdge, MovementWithSlide};
use line_segment::LineSegment;
use vertex_edge_collision::{self, Collision, CollisionInfo, WhatIsMoving};

fn for_each_single_direction_collision<A, B, F>(
    shape: &A,
    position: Vector2<f64>,
    other_shape: &B,
    other_position: Vector2<f64>,
    movement: Vector2<f64>,
    reverse_movement: Vector2<f64>,
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
                Ok(collision) => {
                    f(collision);
                }
                Err(_) => (),
            }
        });
    });
}

pub trait Collide {
    fn aabb(&self, top_left: Vector2<f64>) -> Aabb;
    fn for_each_edge_facing<F: FnMut(LineSegment)>(&self, direction: Vector2<f64>, f: F);
    fn for_each_vertex_facing<F: FnMut(Vector2<f64>)>(
        &self,
        direction: Vector2<f64>,
        f: F,
    );
    fn for_each_left_solid_edge_facing<F: FnMut(LeftSolidEdge)>(
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
        position: Vector2<f64>,
        stationary_shape: &StationaryShape,
        stationary_position: Vector2<f64>,
        movement: Vector2<f64>,
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
                Collision::NoMovement => {
                    shortest_movement.insert_le(0., CollisionInfo::zero())
                }
                Collision::EdgeCollision(info) => {
                    shortest_movement.insert_le(info.magnitude2, info);
                }
            },
        );
        shortest_movement.into_value()
    }

    fn for_each_movement_collision_<StationaryShape, F>(
        &self,
        position: Vector2<f64>,
        stationary_shape: &StationaryShape,
        stationary_position: Vector2<f64>,
        movement: Vector2<f64>,
        mut f: F,
    ) where
        Self: Sized,
        StationaryShape: Collide + ::std::fmt::Debug,
        F: FnMut(CollisionMovement),
    {
        self.for_each_vertex_facing(movement, |rel_vertex| {
            let abs_vertex = rel_vertex + position;
            stationary_shape.for_each_left_solid_edge_facing(-movement, |rel_edge| {
                let abs_edge = rel_edge.add_vector(stationary_position);
                let collision_movement =
                    abs_edge.vertex_collision_vertex_is_moving(abs_vertex, movement);
                f(collision_movement);
            });
        });
        self.for_each_left_solid_edge_facing(movement, |rel_edge| {
            let abs_edge = rel_edge.add_vector(position);
            stationary_shape.for_each_vertex_facing(-movement, |rel_vertex| {
                //                println!("{:?} {:?}", stationary_shape, rel_vertex);
                let abs_vertex = rel_vertex + stationary_position;
                let collision_movement =
                    abs_edge.vertex_collision_edge_is_moving(abs_vertex, movement);
                f(collision_movement);
            });
        });
    }
    fn movement_collision_test_<StationaryShape>(
        &self,
        position: Vector2<f64>,
        stationary_shape: &StationaryShape,
        stationary_position: Vector2<f64>,
        movement: Vector2<f64>,
    ) -> MovementWithSlide
    where
        Self: Sized,
        StationaryShape: Collide + ::std::fmt::Debug,
    {
        let mut shortest_movement = BestSet::new();

        self.for_each_movement_collision_(
            position,
            stationary_shape,
            stationary_position,
            movement,
            |collision_movement| shortest_movement.insert_le(collision_movement),
        );

        println!(
            "{:?}",
            shortest_movement.get().unwrap().movement.movement.vector()
        );

        println!("shortest {:?}", shortest_movement.get().unwrap().collision);

        shortest_movement
            .into_value()
            .map(|c| c.movement)
            .unwrap_or_else(|| MovementWithSlide::new_just_movement(movement))
    }
}
