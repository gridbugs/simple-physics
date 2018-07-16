use aabb::Aabb;
use axis_aligned_rect::AxisAlignedRect;
use cgmath::{vec2, ElementWise, InnerSpace, Vector2};
use fnv::FnvHashMap;
use line_segment::LineSegment;
use loose_quad_tree::LooseQuadTree;
use movement::{self, EntityId, ForEachShapePosition, ShapePosition};
use shape::Shape;

fn clamp(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}

pub struct InputModel {
    left: f32,
    right: f32,
    up: f32,
    down: f32,
}

impl Default for InputModel {
    fn default() -> Self {
        Self {
            left: 0.,
            right: 0.,
            up: 0.,
            down: 0.,
        }
    }
}

impl InputModel {
    pub fn set_left(&mut self, value: f32) {
        self.left = clamp(value, 0., 1.);
    }
    pub fn set_right(&mut self, value: f32) {
        self.right = clamp(value, 0., 1.);
    }
    pub fn set_up(&mut self, value: f32) {
        self.up = clamp(value, 0., 1.);
    }
    pub fn set_down(&mut self, value: f32) {
        self.down = clamp(value, 0., 1.);
    }
    fn horizontal(&self) -> f32 {
        self.right - self.left
    }
    fn vertical(&self) -> f32 {
        self.down - self.up
    }
    fn movement(&self) -> Vector2<f32> {
        let raw = vec2(self.horizontal(), self.vertical());
        if raw.magnitude2() > 1. {
            raw.normalize()
        } else {
            raw
        }
    }
}

pub struct RenderUpdate<'a> {
    pub position: Vector2<f32>,
    pub shape: &'a Shape,
    pub colour: [f32; 3],
}

fn update_player_velocity(
    current_velocity: Vector2<f32>,
    input_model: &InputModel,
) -> Vector2<f32> {
    const MULTIPLIER: Vector2<f32> = Vector2 { x: 0.1, y: 0.5 };
    current_velocity + input_model.movement().mul_element_wise(MULTIPLIER) + vec2(0., 0.1)
}

#[derive(Default)]
struct EntityIdAllocator {
    next: u32,
}

impl EntityIdAllocator {
    fn allocate(&mut self) -> EntityId {
        let id = self.next;
        self.next += 1;
        id
    }
    fn reset(&mut self) {
        self.next = 0;
    }
}

#[derive(Debug)]
struct EntityCommon {
    position: Vector2<f32>,
    shape: Shape,
    colour: [f32; 3],
}

impl EntityCommon {
    fn new(position: Vector2<f32>, shape: Shape, colour: [f32; 3]) -> Self {
        Self {
            position,
            shape,
            colour,
        }
    }
    fn aabb(&self) -> Aabb {
        self.shape.aabb(self.position)
    }
}

#[derive(Default)]
pub struct GameStateChanges {
    position: Vec<(EntityId, Vector2<f32>)>,
    velocity: Vec<(EntityId, Vector2<f32>)>,
}

pub struct GameState {
    player_id: Option<EntityId>,
    entity_id_allocator: EntityIdAllocator,
    common: FnvHashMap<EntityId, EntityCommon>,
    velocity: FnvHashMap<EntityId, Vector2<f32>>,
    quad_tree: LooseQuadTree<EntityId>,
}

impl ForEachShapePosition for GameState {
    fn for_each<F: FnMut(ShapePosition)>(&self, aabb: Aabb, mut f: F) {
        self.quad_tree
            .for_each_intersection(aabb, |_aabb, &entity_id| {
                let common = self.common.get(&entity_id).unwrap();
                let shape_position = ShapePosition {
                    entity_id,
                    shape: &common.shape,
                    position: common.position,
                };
                f(shape_position);
            });
    }
}

impl GameState {
    pub fn new(size_hint: Vector2<f32>) -> Self {
        Self {
            player_id: None,
            entity_id_allocator: Default::default(),
            common: Default::default(),
            velocity: Default::default(),
            quad_tree: LooseQuadTree::new(size_hint),
        }
    }
    fn clear(&mut self) {
        self.player_id = None;
        self.entity_id_allocator.reset();
        self.common.clear();
        self.velocity.clear();
        self.quad_tree.clear();
    }
    fn add_static_solid(&mut self, common: EntityCommon) -> EntityId {
        let id = self.entity_id_allocator.allocate();
        self.quad_tree.insert(common.aabb(), id);
        self.common.insert(id, common);
        id
    }
    fn add_common(&mut self, common: EntityCommon) -> EntityId {
        let id = self.entity_id_allocator.allocate();
        self.common.insert(id, common);
        id
    }
    pub fn init_demo(&mut self) {
        self.clear();
        let player_id = self.add_common(EntityCommon::new(
            vec2(300., -200.),
            Shape::AxisAlignedRect(AxisAlignedRect::new(vec2(32., 64.))),
            [1., 0., 0.],
        ));
        self.player_id = Some(player_id);
        self.velocity.insert(player_id, vec2(0., 0.));
        self.add_static_solid(EntityCommon::new(
            vec2(50., 200.),
            Shape::AxisAlignedRect(AxisAlignedRect::new(vec2(400., 20.))),
            [1., 1., 0.],
        ));
        self.add_static_solid(EntityCommon::new(
            vec2(150., 250.),
            Shape::AxisAlignedRect(AxisAlignedRect::new(vec2(500., 20.))),
            [1., 1., 0.],
        ));
        self.add_static_solid(EntityCommon::new(
            vec2(50., 450.),
            Shape::AxisAlignedRect(AxisAlignedRect::new(vec2(100., 20.))),
            [1., 1., 0.],
        ));
        self.add_static_solid(EntityCommon::new(
            vec2(50., 500.),
            Shape::AxisAlignedRect(AxisAlignedRect::new(vec2(800., 20.))),
            [1., 1., 0.],
        ));
        self.add_static_solid(EntityCommon::new(
            vec2(20., 20.),
            Shape::LineSegment(LineSegment::new(vec2(0., 0.), vec2(50., 100.))),
            [0., 1., 0.],
        ));
        self.add_static_solid(EntityCommon::new(
            vec2(200., 20.),
            Shape::LineSegment(LineSegment::new(vec2(0., 0.), vec2(300., 200.))),
            [0., 1., 0.],
        ));
        self.add_static_solid(EntityCommon::new(
            vec2(200., 20.),
            Shape::LineSegment(LineSegment::new(vec2(0., 90.), vec2(300., 200.))),
            [0., 1., 0.],
        ));
        self.add_static_solid(EntityCommon::new(
            vec2(900., 200.),
            Shape::LineSegment(LineSegment::new(vec2(0., 0.), vec2(-300., 200.))),
            [0., 1., 0.],
        ));
        self.add_static_solid(EntityCommon::new(
            vec2(300., 500.),
            Shape::LineSegment(LineSegment::new(vec2(0., 0.), vec2(-30., -30.))),
            [0., 1., 0.],
        ));
    }
    pub fn update(&mut self, input_model: &InputModel, changes: &mut GameStateChanges) {
        let player_id = self.player_id.expect("No player id");
        if let Some(velocity) = self.velocity.get_mut(&player_id) {
            *velocity = update_player_velocity(*velocity, input_model);
        }
        for (id, velocity) in self.velocity.iter() {
            if let Some(common) = self.common.get(id) {
                let shape_position = ShapePosition {
                    entity_id: *id,
                    position: common.position,
                    shape: &common.shape,
                };
                let new_position =
                    movement::position_after_allowde_movement(shape_position, *velocity, self);
                changes.velocity.push((*id, new_position - common.position));
                changes.position.push((*id, new_position));
            }
        }
        for (id, position) in changes.position.drain(..) {
            if let Some(common) = self.common.get_mut(&id) {
                common.position = position;
            }
        }
        for (id, velocity) in changes.velocity.drain(..) {
            self.velocity.insert(id, velocity);
        }
    }
    pub fn render_updates(&self) -> impl Iterator<Item = RenderUpdate> {
        self.common.values().map(|common| RenderUpdate {
            position: common.position,
            shape: &common.shape,
            colour: common.colour,
        })
    }
}
