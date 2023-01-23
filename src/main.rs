mod game;
mod physics;

use bevy_ecs::prelude::*;
use game::{
  add_game_plugin,
  Camera,
};
use macroquad::prelude::*;
use physics::add_rapier_plugin;

fn wincfg() -> Conf {
  Conf { window_title: "Stlar".to_string(), window_resizable: false, ..Default::default() }
}

#[derive(StageLabel)]
enum CoreStages {
  Update,
  Render,
}

#[macroquad::main(wincfg())]
async fn main() {
  let mut world = World::new();

  let mut schedule = Schedule::default();

  schedule.add_stage(CoreStages::Update, SystemStage::parallel());
  schedule.add_stage_after(CoreStages::Update, CoreStages::Render, SystemStage::parallel());

  add_rapier_plugin(&mut world, &mut schedule);
  add_game_plugin(&mut world, &mut schedule);

  loop {
    clear_background(WHITE);

    let camera = world.resource::<Camera>();
    set_camera(&camera.camera2d);

    schedule.run(&mut world);

    next_frame().await;
  }
}
