use aabb::Aabb;
use axis_aligned_rect::AxisAlignedRect;
use cgmath::Vector2;
use collide::Collide;
use left_solid_edge::CollisionWithSlide;
use line_segment::LineSegment;

#[derive(Debug, Clone)]
pub enum Shape {
    AxisAlignedRect(AxisAlignedRect),
    LineSegment(LineSegment),
}

impl Shape {
    pub fn aabb(&self, top_left: Vector2<f64>) -> Aabb {
        match self {
            &Shape::AxisAlignedRect(ref rect) => rect.aabb(top_left),
            &Shape::LineSegment(ref line_segment) => line_segment.aabb(top_left),
        }
    }
    pub fn movement_collision_test(
        &self,
        position: Vector2<f64>,
        stationary: &Self,
        stationary_position: Vector2<f64>,
        movement_vector: Vector2<f64>,
    ) -> Option<CollisionWithSlide> {
        match self {
            &Shape::AxisAlignedRect(ref moving) => match stationary {
                &Shape::AxisAlignedRect(ref stationary) => moving
                    .movement_collision_test_(
                        position,
                        stationary,
                        stationary_position,
                        movement_vector,
                    ),
                &Shape::LineSegment(ref stationary) => moving.movement_collision_test_(
                    position,
                    stationary,
                    stationary_position,
                    movement_vector,
                ),
            },
            &Shape::LineSegment(_) => panic!(),
        }
    }
}
