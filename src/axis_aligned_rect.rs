use aabb::Aabb;
use cgmath::{Vector2, vec2};
use collide::Collide;
use line_segment::LineSegment;
use vertex_edge_collision::EPSILON;
use left_solid_edge::LeftSolidEdge;

#[derive(Debug, Clone)]
pub struct AxisAlignedRect {
    dimensions: Vector2<f64>,
}

impl AxisAlignedRect {
    pub fn new(dimensions: Vector2<f64>) -> Self {
        Self { dimensions }
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
    fn top(&self) -> LineSegment {
        LineSegment::new_left_solid(self.top_left(), self.top_right())
    }
    fn right(&self) -> LineSegment {
        LineSegment::new_left_solid(self.top_right(), self.bottom_right())
    }
    fn bottom(&self) -> LineSegment {
        LineSegment::new_left_solid(self.bottom_right(), self.bottom_left())
    }
    fn left(&self) -> LineSegment {
        LineSegment::new_left_solid(self.bottom_left(), self.top_left())
    }
    fn top_(&self) -> LeftSolidEdge {
        LeftSolidEdge::new(self.top_left(), self.top_right())
    }
    fn right_(&self) -> LeftSolidEdge {
        LeftSolidEdge::new(self.top_right(), self.bottom_right())
    }
    fn bottom_(&self) -> LeftSolidEdge {
        LeftSolidEdge::new(self.bottom_right(), self.bottom_left())
    }
    fn left_(&self) -> LeftSolidEdge {
        LeftSolidEdge::new(self.bottom_left(), self.top_left())
    }

    pub fn dimensions(&self) -> Vector2<f64> {
        self.dimensions
    }
}

impl Collide for AxisAlignedRect {
    fn aabb(&self, top_left: Vector2<f64>) -> Aabb {
        Aabb::new(top_left, self.dimensions)
    }
    fn for_each_vertex_facing<F>(&self, direction: Vector2<f64>, mut f: F)
    where
        F: FnMut(Vector2<f64>),
    {
        f(self.top_left());
        f(self.top_right());
        f(self.bottom_left());
        f(self.bottom_right());
        /*
        if direction.y > -EPSILON {
            f(self.bottom_left());
            f(self.bottom_right());
            if direction.x > -EPSILON {
                f(self.top_right());
            }
            if direction.x < EPSILON {
                f(self.top_left());
            }
        }
        if direction.y < EPSILON {
            f(self.top_left());
            f(self.top_right());
            if direction.x > -EPSILON {
                f(self.bottom_right());
            }
            if direction.x < EPSILON {
                f(self.bottom_left());
            }
        }*/
    }
    fn for_each_edge_facing<F>(&self, direction: Vector2<f64>, mut f: F)
    where
        F: FnMut(LineSegment),
    {
        if direction.y > -EPSILON {
            f(self.bottom())
        }
        if direction.y < EPSILON {
            f(self.top())
        }
        if direction.x > -EPSILON {
            f(self.right())
        }
        if direction.x < EPSILON {
            f(self.left())
        }
    }
    fn for_each_left_solid_edge_facing<F: FnMut(LeftSolidEdge)>(
        &self,
        direction: Vector2<f64>,
        mut f: F,
    ) {
        f(self.bottom_());
        f(self.top_());
        f(self.left_());
        f(self.right_());
        /*
        if direction.y > -EPSILON {
            f(self.bottom_())
        }
        if direction.y < EPSILON {
            f(self.top_())
        }
        if direction.x > -EPSILON {
            f(self.right_())
        }
        if direction.x < EPSILON {
            f(self.left_())
        } */
    }
}
