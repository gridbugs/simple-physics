use aabb::*;
use cgmath::{vec2, Vector2};
use num::Zero;
use std::num::NonZeroUsize;

#[derive(Debug, Clone)]
pub struct LooseQuadTree<T> {
    seq: u64,
    nodes: Vec<Node<T>>,
    size: Vector2<f32>,
    next_free: usize,
}

#[derive(Debug, Clone)]
struct Node<T> {
    items: Vec<(Aabb, T)>,
    child_offset: Option<NonZeroUsize>,
    seq: u64,
}

impl<T> Default for Node<T> {
    fn default() -> Self {
        Self {
            items: Vec::new(),
            child_offset: None,
            seq: 0,
        }
    }
}
impl<T> Node<T> {
    fn reuse(&mut self, seq: u64) {
        self.items.clear();
        self.child_offset = None;
        self.seq = seq;
    }
}

impl<T> LooseQuadTree<T> {
    const TOP_LEFT: usize = 0;
    const TOP_RIGHT: usize = 1;
    const BOTTOM_LEFT: usize = 2;
    const BOTTOM_RIGHT: usize = 3;
    const NUM_CHILDREN: usize = 4;

    pub fn new(size: Vector2<f32>) -> Self {
        Self {
            seq: 1,
            nodes: vec![Default::default()],
            size,
            next_free: 1,
        }
    }

    pub fn clear(&mut self) {
        self.seq += 1;
        self.nodes[0].reuse(self.seq);
    }

    pub fn insert(&mut self, aabb: Aabb, t: T) {
        let mut centre = aabb.centre();
        let mut index = 0;
        let mut max_size = self.size / 2.;
        let nodes = &mut self.nodes;
        let next_free = &mut self.next_free;
        loop {
            let child_offset = {
                while nodes.len() <= index {
                    nodes.push(Default::default());
                }
                let mut node = &mut nodes[index];
                if node.seq != self.seq {
                    node.reuse(self.seq);
                }
                let size = aabb.size();
                if size.x > max_size.x || size.y > max_size.y {
                    node.items.push((aabb, t));
                    break;
                }
                node.child_offset
                    .get_or_insert_with(|| {
                        let free = *next_free;
                        *next_free += Self::NUM_CHILDREN;
                        NonZeroUsize::new(free).expect("unexpected state")
                    })
                    .get() as usize
            };
            if centre.x < max_size.x {
                if centre.y < max_size.y {
                    index = child_offset + Self::TOP_LEFT;
                } else {
                    index = child_offset + Self::BOTTOM_LEFT;
                    centre.y = centre.y - max_size.y;
                }
            } else {
                if centre.y < max_size.y {
                    index = child_offset + Self::TOP_RIGHT;
                    centre.x = centre.x - max_size.x;
                } else {
                    index = child_offset + Self::BOTTOM_RIGHT;
                    centre = centre - max_size;
                }
            }
            max_size = max_size / 2.
        }
    }

    fn for_each_intersection_rec<F: FnMut(&Aabb, &T)>(
        nodes: &[Node<T>],
        current_index: usize,
        current_node_aabb: Aabb,
        aabb_to_test: &Aabb,
        f: &mut F,
    ) {
        if let Some(node) = nodes.get(current_index) {
            for &(ref aabb, ref t) in node.items.iter() {
                if aabb.is_intersecting(aabb_to_test) {
                    f(aabb, t);
                }
            }
            if let Some(child_offset) = node.child_offset {
                let child_offset = child_offset.get() as usize;
                let AabbSplitFour {
                    top_left,
                    top_right,
                    bottom_left,
                    bottom_right,
                } = current_node_aabb.split_four();
                if top_left
                    .double_about_centre()
                    .is_intersecting(&aabb_to_test)
                {
                    Self::for_each_intersection_rec(
                        nodes,
                        child_offset + Self::TOP_LEFT,
                        top_left,
                        aabb_to_test,
                        f,
                    );
                }
                if top_right
                    .double_about_centre()
                    .is_intersecting(&aabb_to_test)
                {
                    Self::for_each_intersection_rec(
                        nodes,
                        child_offset + Self::TOP_RIGHT,
                        top_right,
                        aabb_to_test,
                        f,
                    );
                }
                if bottom_left
                    .double_about_centre()
                    .is_intersecting(&aabb_to_test)
                {
                    Self::for_each_intersection_rec(
                        nodes,
                        child_offset + Self::BOTTOM_LEFT,
                        bottom_left,
                        aabb_to_test,
                        f,
                    );
                }
                if bottom_right
                    .double_about_centre()
                    .is_intersecting(&aabb_to_test)
                {
                    Self::for_each_intersection_rec(
                        nodes,
                        child_offset + Self::BOTTOM_RIGHT,
                        bottom_right,
                        aabb_to_test,
                        f,
                    );
                }
            }
        }
    }

    pub fn for_each_intersection<'a, F: FnMut(&Aabb, &T)>(&'a self, aabb: Aabb) -> impl 'a + Fn(F) {
        let root_aabb = Aabb::new(vec2(Zero::zero(), Zero::zero()), self.size);
        move |mut f| Self::for_each_intersection_rec(&self.nodes, 0, root_aabb, &aabb, &mut f)
    }
}
