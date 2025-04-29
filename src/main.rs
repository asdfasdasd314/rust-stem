use crate::game::*;
use crate::physics::*;
use crate::float_precision::*;
use crate::player::*;
use crate::render::*;

use std::rc::Rc;
use std::cell::RefCell;

mod game;
mod heap;
mod math_util;
mod physics;
mod hashable;
mod player;
mod render;
mod float_precision;
mod world_gen;

fn main() {
    let player_rect = RectangularPrism::new(Vector3f64::new(0.0, 0.0, 0.0), 1.0, 1.0, 1.0);
    let rb = DynamicBody::new(player_rect.root, Rc::new(RefCell::new(player_rect)));
    let player = Player::new(rb.pos, 10.0, 0.1, CameraType::ThirdPerson(5.0), rb);
    let mut game = Game::new(player, vec![], vec![], (640, 480), "Game".to_string());

    game.game_loop();
}
