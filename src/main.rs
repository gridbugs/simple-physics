#![feature(nonzero)]

extern crate best;
extern crate cgmath;
extern crate fnv;
extern crate num;
#[macro_use]
extern crate gfx;
extern crate gfx_device_gl;
extern crate gfx_window_glutin;
extern crate glutin;

mod aabb;
mod axis_aligned_rect;
mod collide;
mod game;
mod graphics;
mod line_segment;
mod loose_quad_tree;
mod movement;
mod shape;
mod vertex_edge_collision;

use cgmath::vec2;

fn main() {
    let mut qt = loose_quad_tree::LooseQuadTree::new(vec2(10., 10.));
    qt.insert(aabb::Aabb::new(vec2(1., 1.), vec2(2., 2.)), ());
    qt.for_each_intersection(aabb::Aabb::new(vec2(1.5, 1.5), vec2(1., 1.)), |aabb, _| {
        println!("{:?}", aabb);
    });
}
