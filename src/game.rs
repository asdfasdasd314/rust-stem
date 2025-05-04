use crate::physics::*;
use crate::player::*;
use crate::world_gen::*;
use crate::float_precision::*;
use raylib::prelude::*;

#[derive(Debug)]
enum CollisionObject {
    Dynamic(usize),
    Static(usize),
}

// This stores the game state and effectively runs everything that has to do with overall game logic
pub struct Game {
    player: Player,

    dynamic_objects: Vec<DynamicBody>,
    static_objects: Vec<StaticBody>,

    cursor_shown: bool,
    in_debug_mode: bool,

    raylib_handle: RaylibHandle,
    raylib_thread: RaylibThread,
}

impl Game {
    // Handles all the setup for a game (window, player, world, etc.)
    pub fn new(
        player: Player,
        dynamic_objects: Vec<DynamicBody>,
        static_objects: Vec<StaticBody>,
        window_size: (i32, i32),
        window_title: String,
    ) -> Game {
        let (rl, thread) = raylib::init()
            .size(window_size.0, window_size.1)
            .title(&window_title)
            .resizable()
            .build();
        let mut game = Game {
            player,
            dynamic_objects,
            static_objects,
            cursor_shown: false,
            in_debug_mode: false,
            raylib_handle: rl,
            raylib_thread: thread,
        };

        game.raylib_handle.hide_cursor();
        game.raylib_handle.disable_cursor();

        game
    }

    // Perhaps this could change in the future, but the game loop loops forever and never returns
    pub fn game_loop(&mut self) {
        self.generate_world();
        while !self.raylib_handle.window_should_close() {
            // Calculate delta time
            let delta_time = self.raylib_handle.get_frame_time();

            if self.raylib_handle.is_key_pressed(KeyboardKey::KEY_ENTER) {
                self.raylib_handle.toggle_fullscreen();
            }
            if self.raylib_handle.is_key_pressed(KeyboardKey::KEY_TAB) {
                if self.cursor_shown {
                    self.raylib_handle.hide_cursor();
                    self.raylib_handle.disable_cursor();
                } else {
                    self.raylib_handle.show_cursor();
                }
                self.cursor_shown = !self.cursor_shown;
            }
            if self.raylib_handle.is_key_pressed(KeyboardKey::KEY_ZERO) {
                self.in_debug_mode = !self.in_debug_mode;
            }

            // Do user input and movement
            self.player.update(&self.raylib_handle, delta_time as f64);

            let collisions: Vec<(CollisionObject, Vector3f64)> =
                self.find_colliding_objects();
            if !collisions.is_empty() {
                self.run_collisions(collisions);
            }

            // Begin rendering
            let mut draw_handle = self.raylib_handle.begin_drawing(&self.raylib_thread);

            draw_handle.clear_background(Color::RAYWHITE);

            // This covers everything that is rendered in three dimensions
            {
                let mut draw_handle_3d = draw_handle.begin_mode3D(self.player.camera);

                for object in &self.static_objects {
                    object.get_mesh().render(&mut draw_handle_3d, self.in_debug_mode);
                }

                for object in &self.dynamic_objects {
                    object.get_mesh().render(&mut draw_handle_3d, self.in_debug_mode);
                }

                match self.player.camera_type {
                    CameraType::FirstPerson => {}
                    CameraType::ThirdPerson(_) => {
                        self.player.dynamic_body.get_mesh().render(&mut draw_handle_3d, self.in_debug_mode);
                    }
                }
            }

            draw_handle.draw_fps(10, 10);
        }
    }

    pub fn add_static_object(&mut self, new_object: StaticBody) {
        self.static_objects.push(new_object);
    }

    /**
    Returns the first collision object that is said to have "initialized" the collision (either a player or other dynamic body in the world)
    Second entry in the return is the second collision that is "hit" by the first collision object (either a dynamic object or static body, and if it were multiplayer it could be another player object)
    Third entry is the mtv which is applied to the first object to fix intersections between objects
     */
    fn find_colliding_objects(&self) -> Vec<(CollisionObject, Vector3f64)> {
        /// mtv = minimum translation vector
        /// 
        /// A collision can only occur if something is moving, so only the first one must be dynamic, the other can be anything physical
        fn calculate_mtv(obj1: &DynamicBody, obj2: &dyn Physical) -> Option<Vector3f64> {
            let radius1= f64_round(obj1.get_bounding_circle_radius());
            let radius2 = f64_round(obj2.get_bounding_circle_radius());
            let distance = f64_round((obj1.get_center() - obj2.get_center()).length());
            if radius1 + radius2 < distance {
                return None;
            }
            obj1.collides_with(obj2)
        }

        // Check player collisions
        let mut collisions: Vec<(CollisionObject, Vector3f64)> = Vec::new();
        for i in 0..self.dynamic_objects.len() {
            if let Some(mtv) = calculate_mtv(
                &self.player.dynamic_body,
                &self.dynamic_objects[i],
            ) {
                collisions.push((CollisionObject::Dynamic(i), mtv))
            }
        }
        for i in 0..self.static_objects.len() {
            if let Some(mtv) = calculate_mtv(
                &self.player.dynamic_body,
                &self.static_objects[i],
            ) {
                collisions.push((CollisionObject::Static(i), mtv))
            }
        }

        collisions
    }

    fn run_collisions(
        &mut self,
        collisions: Vec<(CollisionObject, Vector3f64)>,
    ) {
        for collision in collisions {
            self.player.move_by(collision.1);
        }
    }

    fn generate_world(&mut self) {
        let height_map = generate_height_map();
        let world_mesh = create_mesh_from_height_map(height_map, Vector2f64::new(0.0, 0.0), 4.0, 4.0);

        // Add the mesh to the world so it will be rendered
        for mesh in world_mesh {
            let center = mesh.borrow().get_center();
            let static_body = StaticBody::new(center, mesh);
            self.add_static_object(static_body);
        }
    }
}
