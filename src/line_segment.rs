use aabb::Aabb;
use cgmath::{vec2, InnerSpace, Vector2};
use collide::{Collide, Edge};

const WIDTH: f64 = 0.1;

#[derive(Debug, Clone, Copy)]
pub struct LineSegment {
    pub start: Vector2<f64>,
    pub end: Vector2<f64>,
}

impl LineSegment {
    pub fn new_both_solid(start: Vector2<f64>, end: Vector2<f64>) -> Self {
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
    fn left_solid_edge(&self) -> Edge {
        Edge::new(self.start, self.end)
    }
    fn left_solid_edge_flipped(&self) -> Edge {
        Edge::new(self.end, self.start)
    }
}

impl Collide for LineSegment {
    fn aabb(&self, top_left: Vector2<f64>) -> Aabb {
        let start = self.start + top_left;
        let end = self.end + top_left;
        // XXX the +/- 1 is because of how edges are computed.
        // Currentyl they push the corners of the thin rectangle
        // representing the line outside the normal bounding box.
        let x_min = start.x.min(end.x) - 1.;
        let x_max = start.x.max(end.x) + 1.;
        let y_min = start.y.min(end.y) - 1.;
        let y_max = start.y.max(end.y) + 1.;
        let top_left = vec2(x_min, y_min);
        let bottom_right = vec2(x_max, y_max);
        Aabb::new(top_left, bottom_right - top_left)
    }

    fn for_each_left_solid_edge_facing<F: FnMut(Edge)>(
        &self,
        _direction: Vector2<f64>,
        mut f: F,
    ) {
        let vector = self.vector();
        let left = vec2(-vector.y, vector.x).normalize_to(WIDTH);
        let a = self.add_vector(left).left_solid_edge_flipped();
        let b = self.add_vector(-left).left_solid_edge();
        f(a);
        f(b);
        f(Edge::new(a.end(), b.start()));
        f(Edge::new(b.end(), a.start()));
    }
}
