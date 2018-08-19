use aabb::Aabb;
use axis_aligned_rect;
use best::BestMultiSet;
use cgmath::Vector2;
use collide::{Collide, CollidePosition, Collision};
use line_segment::LineSegment;
use movement::EntityId;

#[derive(Debug, Clone)]
pub enum Shape {
    AxisAlignedRect(axis_aligned_rect::AxisAlignedRect),
    LineSegment(LineSegment),
}

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
    pub fn movement_aabb(&self, movement: Vector2<f64>) -> Aabb {
        let current_aabb = self.aabb();
        let moved_aabb = self.shape.aabb(self.position + movement);
        current_aabb.union(&moved_aabb)
    }

    fn stationary_collision_test<C: Collide>(
        &self,
        moving: CollidePosition<C>,
        movement: Vector2<f64>,
        closest_collisions: &mut BestMultiSet<Collision>,
    ) {
        let position = self.position;
        let entity_id = self.entity_id;
        match self.shape {
            Shape::AxisAlignedRect(stationary) => {
                let collide_position = CollidePosition {
                    collide: stationary,
                    position,
                    entity_id,
                };
                moving.movement_collision_test(
                    collide_position,
                    movement,
                    closest_collisions,
                );
            }
            Shape::LineSegment(stationary) => {
                let collide_position = CollidePosition {
                    collide: stationary,
                    position,
                    entity_id,
                };
                moving.movement_collision_test(
                    collide_position,
                    movement,
                    closest_collisions,
                );
            }
        }
    }

    pub fn movement_collision_test(
        &self,
        stationary: ShapePosition,
        movement: Vector2<f64>,
        closest_collisions: &mut BestMultiSet<Collision>,
    ) {
        let position = self.position;
        let entity_id = self.entity_id;
        match self.shape {
            Shape::AxisAlignedRect(moving) => {
                let collide_position = CollidePosition {
                    collide: moving,
                    position,
                    entity_id,
                };
                stationary.stationary_collision_test(
                    collide_position,
                    movement,
                    closest_collisions,
                )
            }
            Shape::LineSegment(moving) => {
                let collide_position = CollidePosition {
                    collide: moving,
                    position,
                    entity_id,
                };
                stationary.stationary_collision_test(
                    collide_position,
                    movement,
                    closest_collisions,
                )
            }
        }
    }
}

impl Shape {
    pub fn aabb(&self, top_left: Vector2<f64>) -> Aabb {
        match self {
            &Shape::AxisAlignedRect(ref rect) => rect.aabb(top_left),
            &Shape::LineSegment(ref line_segment) => line_segment.aabb(top_left),
        }
    }
}
