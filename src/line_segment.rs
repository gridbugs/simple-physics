use aabb::Aabb;
use cgmath::{vec2, Vector2};
use collide::Collide;

#[derive(Debug, Clone, Copy)]
pub struct LineSegment {
    pub start: Vector2<f32>,
    pub end: Vector2<f32>,
}

impl LineSegment {
    pub fn new(start: Vector2<f32>, end: Vector2<f32>) -> Self {
        Self { start, end }
    }
    pub fn add_vector(&self, vector: Vector2<f32>) -> Self {
        Self {
            start: self.start + vector,
            end: self.end + vector,
        }
    }
    pub fn vector(&self) -> Vector2<f32> {
        self.end - self.start
    }
}

impl Collide for LineSegment {
    fn aabb(&self, top_left: Vector2<f32>) -> Aabb {
        let start = self.start + top_left;
        let end = self.end + top_left;
        let x_min = start.x.min(end.x);
        let x_max = start.x.max(end.x);
        let y_min = start.y.min(end.y);
        let y_max = start.y.max(end.y);
        let top_left = vec2(x_min, y_min);
        let bottom_right = vec2(x_max, y_max);
        Aabb::new(top_left, bottom_right - top_left)
    }
    fn for_each_edge_facing<F: FnMut(LineSegment)>(&self, _direction: Vector2<f32>, mut f: F) {
        f(*self);
    }
    fn for_each_vertex_facing<F: FnMut(Vector2<f32>)>(&self, _direction: Vector2<f32>, mut f: F) {
        f(self.start);
        f(self.end);
    }
}
