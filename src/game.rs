use crate::physics::*;
use crate::CoreStages;
use bevy_ecs::prelude::*;
use macroquad::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum GameState {
  InGame,
}

#[derive(Resource)]
struct Level {
  id: usize,
  length: f32,
}

#[derive(Resource, Default)]
pub struct Camera {
  pub camera2d: Camera2D,
}

#[derive(Component)]
struct Player {
  speed: f32,
  jump: f32,
  is_grounded: bool,
}

#[derive(Component)]
struct Star;

#[derive(Bundle)]
struct StarBundle {
  id: Star,
  position: Position,
  rigid_body: RigidBodyBuilder,
  #[bundle]
  collider: ColliderBuilder,
}

impl StarBundle {
  pub fn new(pos: Vec2) -> StarBundle {
    StarBundle {
      id: Star,
      position: Position { value: pos },
      rigid_body: RigidBodyBuilder::fixed(),
      collider: ColliderBuilder::ball(17.5).with_as_sensor(),
    }
  }
}

#[derive(Component)]
struct Platform;

#[derive(Bundle)]
struct PlatformBundle {
  id: Platform,
  position: Position,
  rigid_body: RigidBodyBuilder,
  #[bundle]
  collider: ColliderBuilder,
}

impl PlatformBundle {
  pub fn new(rect: Rect) -> PlatformBundle {
    PlatformBundle {
      id: Platform,
      position: Position { value: rect.point() },
      rigid_body: RigidBodyBuilder::fixed(),
      collider: ColliderBuilder::cuboid(rect.size()).with_friction(0.0).with_restitution(0.0),
    }
  }
}

fn setup_player(mut commands: Commands) {
  commands
    .spawn(Player { speed: 2.0, jump: 5.0, is_grounded: false })
    .insert(Position { value: vec2(50.0, 32.0) })
    .insert(RigidBodyBuilder::dynamic())
    .insert(ColliderBuilder::cuboid(vec2(32.0, 32.0)).with_friction(0.0).with_restitution(0.0));
}

fn setup_level(mut commands: Commands, mut level: ResMut<Level>) {
  match level.id {
    1 => {
      commands.spawn(PlatformBundle::new(Rect::new(
        0.0,
        screen_height() - 32.0,
        screen_width(),
        32.0,
      )));
      commands.spawn(PlatformBundle::new(Rect::new(0.0, 0.0, 32.0, screen_height())));
      commands.spawn(PlatformBundle::new(Rect::new(0.0, 0.0, screen_width(), 32.0)));
      commands.spawn(PlatformBundle::new(Rect::new(
        screen_width() - 32.0,
        0.0,
        32.0,
        screen_height(),
      )));

      commands.spawn(PlatformBundle::new(Rect::new(
        screen_width() - 232.0,
        screen_height() - 64.0,
        200.0,
        32.0,
      )));

      commands.spawn(StarBundle::new(vec2(screen_width() - 64.0, screen_height() - 96.0)));

      level.length = screen_width();
    },
    2 => {
      commands.spawn(PlatformBundle::new(Rect::new(
        0.0,
        screen_height() - 32.0,
        screen_width() * 1.5,
        32.0,
      )));
      commands.spawn(PlatformBundle::new(Rect::new(0.0, 0.0, 32.0, screen_height())));
      commands.spawn(PlatformBundle::new(Rect::new(0.0, 0.0, screen_width() * 1.5, 32.0)));
      commands.spawn(PlatformBundle::new(Rect::new(
        screen_width() * 1.5 - 32.0,
        0.0,
        32.0,
        screen_height(),
      )));

      commands.spawn(PlatformBundle::new(Rect::new(
        screen_width() * 1.5 / 2.0 - 450.0,
        screen_height() / 2.0 + 150.0,
        200.0,
        32.0,
      )));

      commands.spawn(PlatformBundle::new(Rect::new(
        screen_width() * 1.5 / 2.0 - 100.0,
        screen_height() / 2.0 + 100.0,
        200.0,
        32.0,
      )));

      commands.spawn(PlatformBundle::new(Rect::new(
        screen_width() * 1.5 / 2.0 + 250.0,
        screen_height() / 2.0 + 50.0,
        200.0,
        32.0,
      )));

      commands.spawn(StarBundle::new(vec2(
        screen_width() * 1.5 / 2.0 + 350.0,
        screen_height() / 2.0 - 64.0,
      )));

      level.length = screen_width() * 1.5;
    },

    _ => (),
  }
}

fn move_player(mut rigid_bodies: ResMut<RigidBodies>, players: Query<(&Player, &RigidBodyHandle)>) {
  let x = (is_key_down(KeyCode::D) || is_key_down(KeyCode::Right)) as i8 as f32
    - (is_key_down(KeyCode::A) || is_key_down(KeyCode::Left)) as i8 as f32;
  let y =
    is_key_pressed(KeyCode::W) || is_key_pressed(KeyCode::Space) || is_key_pressed(KeyCode::Up);

  for (player, player_body) in &players {
    let player_body = rigid_bodies.get_mut(player_body);
    let linvel = player_body.linvel();

    player_body.set_linvel(
      vector![x * player.speed, if y && player.is_grounded { -player.jump } else { linvel.y }],
      true,
    );
  }
}

fn update_player_is_grounded(
  rapier_ctx: Res<RapierCtx>,
  mut players: Query<(&mut Player, &Position, &ColliderHandle)>,
  colliders: Query<(&Position, &ColliderHandle), Without<Player>>,
) {
  for (mut player, player_pos, player_handle) in &mut players {
    player.is_grounded = false;
    for (collider_pos, collider_handle) in &colliders {
      if let Some(contact_pair) = rapier_ctx.is_colliding_with(player_handle, collider_handle) {
        if contact_pair.has_any_active_contact && player_pos.value.y < collider_pos.value.y {
          player.is_grounded = true;
          break;
        }
      }
    }
  }
}

fn player_on_collide_with_star(
  rapier_ctx: Res<RapierCtx>,
  mut level: ResMut<Level>,
  mut game_state: ResMut<State<GameState>>,
  mut players: Query<&ColliderHandle, With<Player>>,
  stars: Query<&ColliderHandle, With<Star>>,
) {
  for player_handle in &mut players {
    for star_handle in &stars {
      if rapier_ctx.is_intersecting_with(player_handle, star_handle) {
        level.id += 1;
        game_state.overwrite_restart();
        break;
      }
    }
  }
}

fn sync_camera(
  mut camera: ResMut<Camera>,
  level: Res<Level>,
  players: Query<(&Position, &Shape), With<Player>>,
) {
  if let Some((player_pos, player_shape)) = players.iter().next() {
    let player_center = if let Shape::Cuboid { size } = player_shape {
      player_pos.value + *size / 2.0
    } else {
      player_pos.value
    };
    let x = if player_center.x > screen_width() / 2.0 {
      player_center.x - screen_width() / 2.0
    } else {
      0.0
    };

    if player_center.x + screen_width() / 2.0 < level.length {
      camera.camera2d =
        Camera2D::from_display_rect(Rect::new(x, 0.0, screen_width(), screen_height()));
    }
  }
}

fn draw_shapes(shapes: Query<(&Shape, &Position)>) {
  for (shape, pos) in &shapes {
    match shape {
      // Temporarily draw all shapes as red
      Shape::Cuboid { size } => draw_rectangle(pos.value.x, pos.value.y, size.x, size.y, RED),
      Shape::Ball { r } => draw_circle(pos.value.x, pos.value.y, *r, RED),
    }
  }
}

fn remove_physics(
  mut rapier_ctx: ResMut<RapierCtx>,
  mut colliders: ResMut<Colliders>,
  mut rigid_bodies: ResMut<RigidBodies>,
  entities: Query<(Option<&ColliderHandle>, Option<&RigidBodyHandle>)>,
) {
  for (collider, body) in &entities {
    if let Some(collider) = collider {
      rapier_ctx.remove_collider(&mut colliders, &mut rigid_bodies, collider);
    }
    if let Some(body) = body {
      rapier_ctx.remove_rigid_body(&mut colliders, &mut rigid_bodies, body);
    }
  }
}

fn despawn_all(mut commands: Commands, entities: Query<Entity>) {
  for entity in &entities {
    commands.entity(entity).despawn();
  }
}

pub fn add_game_plugin(world: &mut World, schedule: &mut Schedule) {
  world.insert_resource(State::new(GameState::InGame));
  world.insert_resource(Camera::default());
  world.insert_resource(Level { id: 1, length: screen_width() });

  schedule.add_system_set_to_stage(CoreStages::Update, State::<GameState>::get_driver());
  schedule.add_system_set_to_stage(CoreStages::Render, State::<GameState>::get_driver());

  schedule.add_system_set_to_stage(
    CoreStages::Update,
    SystemSet::on_enter(GameState::InGame).with_system(setup_player).with_system(setup_level),
  );
  schedule.add_system_set_to_stage(
    CoreStages::Update,
    SystemSet::on_update(GameState::InGame)
      .with_system(move_player)
      .with_system(update_player_is_grounded)
      .with_system(player_on_collide_with_star)
      .with_system(sync_camera),
  );
  schedule.add_system_set_to_stage(
    CoreStages::Update,
    SystemSet::on_exit(GameState::InGame).with_system(remove_physics).with_system(despawn_all),
  );

  schedule.add_system_set_to_stage(
    CoreStages::Render,
    SystemSet::on_update(GameState::InGame).with_system(draw_shapes),
  );
}
