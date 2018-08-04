use aabb::Aabb;
use cgmath::{Vector2, vec2};
use collide::{channels, flags, Collide, Edge};
use left_solid_edge::EPSILON;

#[derive(Debug, Clone, Copy)]
enum Metadata {
    Main,
    Character,
    FloorOnly,
}

impl Metadata {
    fn top_channel(self) -> u32 {
        match self {
            Metadata::FloorOnly => channels::FLOOR,
            Metadata::Character => channels::MAIN,
            _ => channels::MAIN,
        }
    }
    fn bottom_channel(self) -> u32 {
        match self {
            Metadata::FloorOnly => 0,
            Metadata::Character => channels::MAIN | channels::FLOOR,
            _ => channels::MAIN,
        }
    }
    fn left_channel(self) -> u32 {
        match self {
            Metadata::FloorOnly => 0,
            Metadata::Character => channels::MAIN,
            _ => channels::MAIN,
        }
    }
    fn right_channel(self) -> u32 {
        match self {
            Metadata::FloorOnly => 0,
            Metadata::Character => channels::MAIN,
            _ => channels::MAIN,
        }
    }
    fn top_flags(self) -> u32 {
        match self {
            _ => 0,
        }
    }
    fn bottom_flags(self) -> u32 {
        match self {
            _ => 0,
        }
    }
    fn left_flags(self) -> u32 {
        match self {
            Metadata::Character => flags::FLOOR_START | flags::BUMP_START,
            _ => 0,
        }
    }
    fn right_flags(self) -> u32 {
        match self {
            Metadata::Character => flags::FLOOR_END | flags::BUMP_END,
            _ => 0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct AxisAlignedRect {
    dimensions: Vector2<f64>,
    metadata: Metadata,
}

impl AxisAlignedRect {
    pub fn new(dimensions: Vector2<f64>) -> Self {
        Self {
            dimensions,
            metadata: Metadata::Main,
        }
    }
    pub fn new_character(dimensions: Vector2<f64>) -> Self {
        Self {
            dimensions,
            metadata: Metadata::Character,
        }
    }
    pub fn new_floor_only(dimensions: Vector2<f64>) -> Self {
        Self {
            dimensions,
            metadata: Metadata::FloorOnly,
        }
    }
    fn top_left(&self) -> Vector2<f64> {
        vec2(0., 0.)
    }
    fn top_right(&self) -> Vector2<f64> {
        vec2(self.dimensions.x, 0.)
    }
    fn bottom_left(&self) -> Vector2<f64> {
        vec2(0., self.dimensions.y)
    }
    fn bottom_right(&self) -> Vector2<f64> {
        self.dimensions
    }
    fn top(&self) -> Edge {
        Edge::new(self.top_left(), self.top_right())
    }
    fn right(&self) -> Edge {
        Edge::new(self.top_right(), self.bottom_right())
    }
    fn bottom(&self) -> Edge {
        Edge::new(self.bottom_right(), self.bottom_left())
    }
    fn left(&self) -> Edge {
        Edge::new(self.bottom_left(), self.top_left())
    }

    pub fn dimensions(&self) -> Vector2<f64> {
        self.dimensions
    }
}

impl Collide for AxisAlignedRect {
    fn aabb(&self, top_left: Vector2<f64>) -> Aabb {
        Aabb::new(top_left, self.dimensions)
    }
    fn for_each_left_solid_edge_facing<F: FnMut(Edge)>(
        &self,
        direction: Vector2<f64>,
        mut f: F,
    ) {
        if direction.y > -EPSILON {
            f(self.bottom()
                .with_channels(self.metadata.bottom_channel())
                .with_flags(self.metadata.bottom_flags()))
        }
        if direction.y < EPSILON {
            f(self.top()
                .with_channels(self.metadata.top_channel())
                .with_flags(self.metadata.top_flags()))
        }
        if direction.x > -EPSILON {
            f(self.right()
                .with_channels(self.metadata.right_channel())
                .with_flags(self.metadata.right_flags()))
        }
        if direction.x < EPSILON {
            f(self.left()
                .with_channels(self.metadata.left_channel())
                .with_flags(self.metadata.left_flags()))
        }
    }
}
