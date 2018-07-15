#![feature(nonzero)]

extern crate cgmath;
extern crate num;

mod aabb;
mod collision;
mod line_segment;
mod loose_quad_tree;

use cgmath::vec2;

fn main() {
    let mut qt = loose_quad_tree::LooseQuadTree::new(vec2(10., 10.));
    qt.insert(aabb::Aabb::new(vec2(1., 1.), vec2(2., 2.)), ());
    let fe = qt.for_each_intersection(aabb::Aabb::new(vec2(1.5, 1.5), vec2(1., 1.)));
    let x = |aabb: &aabb::Aabb, _: &()| {
        println!("{:?}", aabb);
    };
    fe(x);
    fe(x);
}
