#![allow(clippy::type_complexity)]

pub use ::rapier2d::prelude::nalgebra;
use bevy_ecs::prelude::*;
use macroquad::prelude::*;
pub use nalgebra::vector;
use rapier2d::prelude as rapier2d;

use crate::CoreStages;

const PIXEL_TO_METER: f32 = 100.0;
const GRAVITY: f32 = 10.0;

fn macroquad_to_rapier2d(pos: &Position, shape: &Shape) -> rapier2d::Vector<f32> {
  if let Shape::Cuboid { size } = shape {
    rapier2d::vector![
      (pos.value.x + size.x / 2.0) / PIXEL_TO_METER,
      (pos.value.y + size.y / 2.0) / PIXEL_TO_METER
    ]
  } else {
    rapier2d::vector![pos.value.x / PIXEL_TO_METER, pos.value.y / PIXEL_TO_METER]
  }
}

#[derive(Resource)]
pub struct RapierCtx {
  physics_pipeline: rapier2d::PhysicsPipeline,
  gravity: rapier2d::Vector<rapier2d::Real>,
  integration_parameters: rapier2d::IntegrationParameters,
  islands: rapier2d::IslandManager,
  broad_phase: rapier2d::BroadPhase,
  narrow_phase: rapier2d::NarrowPhase,
  impulse_joints: rapier2d::ImpulseJointSet,
  multibody_joints: rapier2d::MultibodyJointSet,
  ccd_solver: rapier2d::CCDSolver,
  query_pipeline: Option<rapier2d::QueryPipeline>,
  hooks: Box<dyn rapier2d::PhysicsHooks>,
  events: Box<dyn rapier2d::EventHandler>,
}

impl RapierCtx {
  pub fn get_contacts(&self, handle: &ColliderHandle) -> Vec<&rapier2d::ContactPair> {
    self.narrow_phase.contacts_with(handle.0).collect()
  }

  pub fn is_colliding_with(
    &self,
    a: &ColliderHandle,
    b: &ColliderHandle,
  ) -> Option<&rapier2d::ContactPair> {
    self.narrow_phase.contact_pair(a.0, b.0)
  }

  pub fn get_intersections(
    &self,
    handle: &ColliderHandle,
  ) -> Vec<(rapier2d::ColliderHandle, rapier2d::ColliderHandle, bool)> {
    self.narrow_phase.intersections_with(handle.0).collect()
  }

  pub fn is_intersecting_with(&self, a: &ColliderHandle, b: &ColliderHandle) -> bool {
    self.narrow_phase.intersection_pair(a.0, b.0).unwrap_or(false)
  }

  pub fn remove_collider(&mut self, colliders: &mut Colliders, rigid_bodies: &mut RigidBodies, handle: &ColliderHandle) {
    colliders.set.remove(handle.0, &mut self.islands, &mut rigid_bodies.set, true);
  }

  pub fn remove_rigid_body(&mut self, colliders: &mut Colliders, rigid_bodies: &mut RigidBodies, handle: &RigidBodyHandle) {
    rigid_bodies.set.remove(handle.0, &mut self.islands, &mut colliders.set, &mut self.impulse_joints, &mut self.multibody_joints, true);
  }

  fn step(&mut self, colliders: &mut rapier2d::ColliderSet, bodies: &mut rapier2d::RigidBodySet) {
    self.physics_pipeline.step(
      &self.gravity,
      &self.integration_parameters,
      &mut self.islands,
      &mut self.broad_phase,
      &mut self.narrow_phase,
      bodies,
      colliders,
      &mut self.impulse_joints,
      &mut self.multibody_joints,
      &mut self.ccd_solver,
      self.query_pipeline.as_mut(),
      self.hooks.as_ref(),
      self.events.as_ref(),
    )
  }
}

#[derive(Resource)]
pub struct Colliders {
  set: rapier2d::ColliderSet,
}

impl Colliders {
  pub fn get_mut(&mut self, handle: &ColliderHandle) -> &mut rapier2d::Collider {
    self.set.get_mut(handle.0).unwrap()
  }
}

#[derive(Resource)]
pub struct RigidBodies {
  set: rapier2d::RigidBodySet,
}

impl RigidBodies {
  pub fn get_mut(&mut self, handle: &RigidBodyHandle) -> &mut rapier2d::RigidBody {
    self.set.get_mut(handle.0).unwrap()
  }
}

#[derive(Component)]
pub struct Position {
  pub value: Vec2,
}

#[derive(Component)]
pub enum Shape {
  Ball { r: f32 },
  Cuboid { size: Vec2 },
}

impl From<Vec2> for Position {
  fn from(value: Vec2) -> Self { Position { value } }
}

impl From<Position> for Vec2 {
  fn from(value: Position) -> Self { value.value }
}

#[derive(Component)]
pub struct ColliderHandle(rapier2d::ColliderHandle);

#[derive(Bundle)]
pub struct ColliderBuilder {
  collider: ColliderWrapper,
  shape: Shape,
}

#[derive(Component)]
struct ColliderWrapper(rapier2d::Collider);

impl ColliderBuilder {
  pub fn ball(r: f32) -> ColliderBuilder {
    ColliderBuilder {
      collider: ColliderWrapper(rapier2d::ColliderBuilder::ball(r / PIXEL_TO_METER).build()),
      shape: Shape::Ball { r },
    }
  }

  pub fn cuboid(size: Vec2) -> ColliderBuilder {
    ColliderBuilder {
      collider: ColliderWrapper(
        rapier2d::ColliderBuilder::cuboid(
          size.x / 2.0 / PIXEL_TO_METER,
          size.y / 2.0 / PIXEL_TO_METER,
        )
        .build(),
      ),
      shape: Shape::Cuboid { size },
    }
  }

  pub fn with_as_sensor(mut self) -> ColliderBuilder {
    self.collider.0.set_sensor(true);
    self
  }

  pub fn with_friction(mut self, coefficient: f32) -> ColliderBuilder {
    self.collider.0.set_friction(coefficient);
    self
  }

  pub fn with_restitution(mut self, coefficient: f32) -> ColliderBuilder {
    self.collider.0.set_restitution(coefficient);
    self
  }
}

#[derive(Component)]
pub struct RigidBodyHandle(rapier2d::RigidBodyHandle);

#[derive(Component)]
pub struct RigidBodyBuilder {
  rigid_body: rapier2d::RigidBody,
}

impl RigidBodyBuilder {
  pub fn fixed() -> RigidBodyBuilder {
    RigidBodyBuilder { rigid_body: rapier2d::RigidBodyBuilder::fixed().build() }
  }

  pub fn dynamic() -> RigidBodyBuilder {
    RigidBodyBuilder { rigid_body: rapier2d::RigidBodyBuilder::dynamic().build() }
  }
}

fn add_colliders(
  mut commands: Commands,
  mut colliders: ResMut<Colliders>,
  colliders_iter: Query<(Entity, &ColliderWrapper, &Position, &Shape), Without<RigidBodyBuilder>>,
) {
  for (ent, collider, pos, shape) in &colliders_iter {
    let mut collider = collider.0.clone();
    collider.set_translation(macroquad_to_rapier2d(pos, shape));

    let handle = colliders.set.insert(collider);
    commands.entity(ent).insert(ColliderHandle(handle)).remove::<ColliderWrapper>();
  }
}

fn add_rigid_bodies(
  mut commands: Commands,
  mut colliders: ResMut<Colliders>,
  mut bodies: ResMut<RigidBodies>,
  rigid_bodies: Query<(Entity, &RigidBodyBuilder, Option<&ColliderWrapper>, &Position, &Shape)>,
) {
  for (ent, rigid_body, collider, pos, shape) in &rigid_bodies {
    let mut rigid_body = rigid_body.rigid_body.clone();
    rigid_body.set_translation(macroquad_to_rapier2d(pos, shape), true);

    let handle = bodies.set.insert(rigid_body);
    if let Some(collider) = collider {
      let handle = colliders.set.insert_with_parent(collider.0.clone(), handle, &mut bodies.set);
      commands.entity(ent).insert(ColliderHandle(handle)).remove::<ColliderWrapper>();
    }

    commands.entity(ent).insert(RigidBodyHandle(handle)).remove::<RigidBodyBuilder>();
  }
}

fn world_step(
  mut rapier_ctx: ResMut<RapierCtx>,
  mut colliders: ResMut<Colliders>,
  mut bodies: ResMut<RigidBodies>,
) {
  rapier_ctx.step(&mut colliders.set, &mut bodies.set);
}

fn sync_pos(
  mut bodies: ResMut<RigidBodies>,
  mut rigid_bodies: Query<(&RigidBodyHandle, &mut Position, &Shape)>,
) {
  for (handle, mut pos, shape) in &mut rigid_bodies {
    let body = bodies.set.get_mut(handle.0).unwrap();

    if pos.is_changed() {
      body.set_translation(macroquad_to_rapier2d(&pos, shape), true);
    } else {
      let pos = pos.bypass_change_detection();

      pos.value.x = body.translation().x * PIXEL_TO_METER;
      pos.value.y = body.translation().y * PIXEL_TO_METER;

      if let Shape::Cuboid { size } = shape {
        pos.value -= *size / 2.0;
      }
    }
  }
}

#[derive(StageLabel)]
enum RapierStage {
  Initialize,
  Step,
  Sync,
}

pub fn add_rapier_plugin(world: &mut World, schedule: &mut Schedule) {
  let rapier_ctx = RapierCtx {
    physics_pipeline: rapier2d::PhysicsPipeline::new(),
    gravity: rapier2d::vector![0.0, GRAVITY],
    integration_parameters: rapier2d::IntegrationParameters::default(),
    islands: rapier2d::IslandManager::new(),
    broad_phase: rapier2d::BroadPhase::new(),
    narrow_phase: rapier2d::NarrowPhase::new(),
    impulse_joints: rapier2d::ImpulseJointSet::new(),
    multibody_joints: rapier2d::MultibodyJointSet::new(),
    ccd_solver: rapier2d::CCDSolver::new(),
    query_pipeline: None,
    hooks: Box::new(()),
    events: Box::new(()),
  };
  let colliders = Colliders { set: rapier2d::ColliderSet::new() };
  let bodies = RigidBodies { set: rapier2d::RigidBodySet::new() };

  world.insert_resource(rapier_ctx);
  world.insert_resource(colliders);
  world.insert_resource(bodies);

  schedule.add_stage_before(
    CoreStages::Update,
    RapierStage::Sync,
    SystemStage::parallel().with_system(sync_pos),
  );
  schedule.add_stage_before(
    RapierStage::Sync,
    RapierStage::Step,
    SystemStage::parallel().with_system(world_step),
  );
  schedule.add_stage_before(
    RapierStage::Step,
    RapierStage::Initialize,
    SystemStage::parallel().with_system(add_colliders).with_system(add_rigid_bodies),
  );
}
