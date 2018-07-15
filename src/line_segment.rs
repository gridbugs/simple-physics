use cgmath::Vector2;

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
