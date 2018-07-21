use cgmath::{Vector2, vec2};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    top_left: Vector2<f64>,
    size: Vector2<f64>,
}

pub struct AabbSplitFour {
    pub top_left: Aabb,
    pub top_right: Aabb,
    pub bottom_left: Aabb,
    pub bottom_right: Aabb,
}

impl Aabb {
    pub fn new(top_left: Vector2<f64>, size: Vector2<f64>) -> Self {
        Self { top_left, size }
    }
    pub fn from_centre_and_half_size(
        centre: Vector2<f64>,
        half_size: Vector2<f64>,
    ) -> Self {
        let top_left = centre - half_size;
        let size = half_size * 2.;
        Self::new(top_left, size)
    }
    fn bottom_right_coord(&self) -> Vector2<f64> {
        self.top_left + self.size
    }
    pub fn from_union(a: &Aabb, b: &Aabb) -> Self {
        let top_left = vec2(
            a.top_left.x.min(b.top_left.x),
            a.top_left.y.min(b.top_left.y),
        );
        let a_bottom_right_coord = a.bottom_right_coord();
        let b_bottom_right_coord = b.bottom_right_coord();
        let bottom_right_coord = vec2(
            a_bottom_right_coord.x.max(b_bottom_right_coord.x),
            a_bottom_right_coord.y.max(b_bottom_right_coord.y),
        );
        let size = bottom_right_coord - top_left;
        Self::new(top_left, size)
    }
    pub fn union(&self, other: &Self) -> Self {
        Self::from_union(self, other)
    }
    pub fn size(&self) -> Vector2<f64> {
        self.size
    }
    pub fn is_intersecting(&self, other: &Aabb) -> bool {
        self.top_left.x + self.size.x >= other.top_left.x
            && other.top_left.x + other.size.x >= self.top_left.x
            && self.top_left.y + self.size.y >= other.top_left.y
            && other.top_left.y + other.size.y >= self.top_left.y
    }
    pub fn centre(&self) -> Vector2<f64> {
        self.top_left + self.size / 2.
    }
    pub fn split_four(&self) -> AabbSplitFour {
        let size = self.size / 2.;
        AabbSplitFour {
            top_left: Self::new(self.top_left, size),
            top_right: Self::new(vec2(self.top_left.x + size.x, self.top_left.y), size),
            bottom_left: Self::new(vec2(self.top_left.x, self.top_left.y + size.y), size),
            bottom_right: Self::new(self.top_left + size, size),
        }
    }
    pub fn double_about_centre(&self) -> Self {
        Self::from_centre_and_half_size(self.centre(), self.size)
    }
}
