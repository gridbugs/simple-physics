use aabb::Aabb;
use best::BestMap;
use cgmath::Vector2;
use loose_quad_tree::LooseQuadTree;
use movement::{EntityId, ForEachShapePosition, ShapePosition};
use shape::Shape;
use std::collections::HashMap;

pub struct GameState {
    position: HashMap<EntityId, Vector2<f32>>,
    shape: HashMap<EntityId, Shape>,
    quad_tree: LooseQuadTree<EntityId>,
}

impl ForEachShapePosition for GameState {
    fn for_each<F: FnMut(ShapePosition)>(&self, aabb: Aabb, mut f: F) {
        self.quad_tree
            .for_each_intersection(aabb, |_aabb, &entity_id| {
                let shape_position = ShapePosition {
                    entity_id,
                    shape: self.shape.get(&entity_id).unwrap(),
                    position: *self.position.get(&entity_id).unwrap(),
                };
                f(shape_position);
            });
    }
}
