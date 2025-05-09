use render::MeshShape;
use world_gen::GroundMesh;

use crate::game::*;
use crate::physics::*;
use crate::float_precision::*;
use crate::player::*;
use crate::shapes::*;

use std::rc::Rc;
use std::cell::RefCell;

mod game;
mod heap;
mod math;
mod physics;
mod hashable;
mod player;
mod render;
mod float_precision;
mod world_gen;
mod shapes;

fn main() {
    let player_mesh = MeshShape::Sphere(Sphere::new(Vector3f64::new(0.0, 0.0, 0.0), 1.0));
    let rb = DynamicBody::new(Vector3f64::new(0.0, 0.0, 0.0), Rc::new(RefCell::new(player_mesh)));
    let player = Player::new(rb.pos, 10.0, 0.1, CameraType::ThirdPerson(5.0), rb);
    let mut game = Game::new(player, vec![], vec![], (640, 480), "Game".to_string());

    game.game_loop();
}
