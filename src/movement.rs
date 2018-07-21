use aabb::Aabb;
use best::{BestMap, BestSet};
use cgmath::Vector2;
use left_solid_edge::{CollisionMovement, MovementWithSlide, EPSILON};
use shape::Shape;
use vertex_edge_collision::CollisionInfo;

pub type EntityId = u32;

#[derive(Debug)]
pub struct ShapePosition<'a> {
    pub entity_id: EntityId,
    pub position: Vector2<f32>,
    pub shape: &'a Shape,
}

impl<'a> ShapePosition<'a> {
    fn aabb(&self) -> Aabb {
        self.shape.aabb(self.position)
    }
    fn movement_aabb(&self, movement: Vector2<f32>) -> Aabb {
        let current_aabb = self.aabb();
        let moved_aabb = self.shape.aabb(self.position + movement);
        current_aabb.union(&moved_aabb)
    }
    fn movement_collision_test(
        &self,
        other: ShapePosition,
        movement: Vector2<f32>,
    ) -> MovementWithSlide {
        self.shape.movement_collision_test(
            self.position,
            other.shape,
            other.position,
            movement,
        )
    }
}

pub trait ForEachShapePosition {
    fn for_each<F: FnMut(ShapePosition)>(&self, aabb: Aabb, f: F);
}

fn allowed_movement_step<F>(
    shape_position: ShapePosition,
    movement: Vector2<f32>,
    for_each_shape_position: &F,
) -> MovementWithSlide
where
    F: ForEachShapePosition,
{
    let mut shortest_movement = BestSet::new();
    for_each_shape_position.for_each(
        shape_position.movement_aabb(movement),
        |other_shape_position: ShapePosition| {
            if other_shape_position.entity_id != shape_position.entity_id {
                let movement = shape_position
                    .movement_collision_test(other_shape_position, movement);
                shortest_movement.insert_le(movement);
            }
        },
    );
    shortest_movement
        .into_value()
        .unwrap_or_else(|| MovementWithSlide::new_just_movement(movement))
}

pub fn position_after_allowde_movement<F>(
    shape_position: ShapePosition,
    mut movement: Vector2<f32>,
    for_each_shape_position: &F,
) -> Vector2<f32>
where
    F: ForEachShapePosition,
{
    let mut position = shape_position.position;
    const MAX_ITERATIONS: usize = 16;
    for _ in 0..MAX_ITERATIONS {
        let allowed_movement = allowed_movement_step(
            ShapePosition {
                position,
                ..shape_position
            },
            movement,
            for_each_shape_position,
        );
        position += allowed_movement.movement.vector();
        if allowed_movement.slide.magnitude2() > EPSILON {
            movement = allowed_movement.slide.vector();
        } else {
            break;
        }
    }
    position
}
