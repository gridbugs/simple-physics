use aabb::Aabb;
use best::BestSet;
use cgmath::Vector2;
use left_solid_edge::CollisionWithSlide;
use shape::Shape;

pub type EntityId = u32;

#[derive(Debug)]
pub struct ShapePosition<'a> {
    pub entity_id: EntityId,
    pub position: Vector2<f64>,
    pub shape: &'a Shape,
}

impl<'a> ShapePosition<'a> {
    fn aabb(&self) -> Aabb {
        self.shape.aabb(self.position)
    }
    fn movement_aabb(&self, movement: Vector2<f64>) -> Aabb {
        let current_aabb = self.aabb();
        let moved_aabb = self.shape.aabb(self.position + movement);
        current_aabb.union(&moved_aabb)
    }
    fn movement_collision_test(
        &self,
        other: ShapePosition,
        movement: Vector2<f64>,
    ) -> Option<CollisionWithSlide> {
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
    movement: Vector2<f64>,
    for_each_shape_position: &F,
) -> Option<CollisionWithSlide>
where
    F: ForEachShapePosition,
{
    let mut shortest_movement = BestSet::new();
    for_each_shape_position.for_each(
        shape_position.movement_aabb(movement),
        |other_shape_position: ShapePosition| {
            if other_shape_position.entity_id != shape_position.entity_id {
                if let Some(movement) =
                    shape_position.movement_collision_test(other_shape_position, movement)
                {
                    shortest_movement.insert_le(movement);
                }
            }
        },
    );
    shortest_movement.into_value()
}

pub fn position_after_allowde_movement<F>(
    shape_position: ShapePosition,
    mut movement: Vector2<f64>,
    for_each_shape_position: &F,
) -> Vector2<f64>
where
    F: ForEachShapePosition,
{
    let mut position = shape_position.position;
    const MAX_ITERATIONS: usize = 16;
    for _ in 0..MAX_ITERATIONS {
        match allowed_movement_step(
            ShapePosition {
                position,
                ..shape_position
            },
            movement,
            for_each_shape_position,
        ) {
            None => {
                position += movement;
                break;
            }
            Some(collision_with_slide) => {
                position += collision_with_slide.movement_to_collision(movement);
                movement = collision_with_slide.slide(movement);
            }
        }
    }
    position
}
