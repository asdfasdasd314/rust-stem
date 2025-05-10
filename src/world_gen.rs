use std::cell::RefCell;
use std::rc::Rc;

use crate::float_precision::*;
use crate::shapes::Polygon;
use chrono::Utc;
use noise::{NoiseFn, Simplex};
use rand::seq::SliceRandom;
use rand::thread_rng;
use raylib::prelude::*;

#[derive(Debug, Clone)]
pub struct GroundMesh {
    pub bottom_left_pos: Vector2f64,

    // These describe the distance between two points in the heightmap
    pub dx: f64,
    pub dz: f64,

    // Heights of the points on the mesh
    pub h1: f64, // 1st quadrant, upper left
    pub h2: f64, // 2nd quadrant, upper right
    pub h3: f64, // 3rd quadrant, lower right
    pub h4: f64, // 4th quadrant, lower left

    pub color: raylib::color::Color,
}

impl GroundMesh {
    pub fn new(
        bottom_left_pos: Vector2f64,
        dx: f64,
        dz: f64,
        h1: f64,
        h2: f64,
        h3: f64,
        h4: f64,
    ) -> Self {
        let colors = [Color::GREEN,
            Color::BROWN,
            Color::BLACK,
            Color::ORANGE,
            Color::YELLOW,
            Color::PURPLE,
            Color::BLUE];

        let mut rng = thread_rng();

        let random_color = colors.choose(&mut rng).unwrap();

        Self {
            bottom_left_pos,
            dx,
            dz,
            h1,
            h2,
            h3,
            h4,
            color: *random_color,
        }
    }

    pub fn get_vertices(&self) -> Vec<Vector3f64> {
        vec![
            Vector3f64::new(self.bottom_left_pos.x, self.h1, self.bottom_left_pos.y),
            Vector3f64::new(
                self.bottom_left_pos.x + self.dx,
                self.h2,
                self.bottom_left_pos.y,
            ),
            Vector3f64::new(
                self.bottom_left_pos.x + self.dx,
                self.h3,
                self.bottom_left_pos.y + self.dz,
            ),
            Vector3f64::new(
                self.bottom_left_pos.x,
                self.h4,
                self.bottom_left_pos.y + self.dz,
            ),
        ]
    }

    pub fn get_center(&self) -> Vector3f64 {
        let average_height = (self.h1 + self.h2 + self.h3 + self.h4) / 4.0;
        Vector3f64::new(
            self.bottom_left_pos.x + self.dx / 2.0,
            average_height,
            self.bottom_left_pos.y + self.dz / 2.0,
        )
    }

    pub fn get_polygons(&self) -> Vec<Polygon> {
        let vertices = self.get_vertices();

        // Because it's not guaranteed that each 4 points will be coplanar, we create two traingles that contain all the points
        let triangle1 = vec![vertices[0], vertices[1], vertices[2]];
        let triangle2 = vec![vertices[2], vertices[3], vertices[0]];
        vec![Polygon::new(triangle1), Polygon::new(triangle2)]
    }
}

pub fn generate_height_map() -> (Vec<Vec<f64>>, u32) {
    // Initialize Simplex noise generator
    let seed = Utc::now().timestamp() as u32;
    let simplex = Simplex::new(seed);

    // Heightmap dimensions
    let width = 100;
    let height = 100;

    // Generate heightmap using Simplex Noise
    let mut heights = vec![vec![0.0; height]; width];
    for i in 0..width {
        for j in 0..height {
            let noise_value = simplex.get([i as f64 / 20.0, j as f64 / 20.0]) * 10.0;
            heights[i][j] = noise_value;
        }
    }

    (heights, seed)
}

/**
Creates a mesh from a height map, but because we use the separating axis theorem, the height map has to be guaranteed to be a convex shape
So for each little grid of four points we have to create an individual mesh shape
 */
pub fn create_mesh_from_height_map(
    height_map: Vec<Vec<f64>>,
    start_pos: Vector2f64,
    dx: f64,
    dz: f64,
) -> Vec<Rc<RefCell<GroundMesh>>> {
    let mut all_meshes: Vec<Rc<RefCell<GroundMesh>>> =
        Vec::with_capacity((height_map.len() - 1) * (height_map[0].len() - 1));
    for i in 0..height_map.len() - 1 {
        for j in 0..height_map[0].len() - 1 {
            let ground_mesh = GroundMesh::new(
                Vector2f64::new(start_pos.x + i as f64 * dx, start_pos.y + j as f64 * dz),
                dx,
                dz,
                height_map[i][j],
                height_map[i + 1][j],
                height_map[i + 1][j + 1],
                height_map[i][j + 1],
            );
            all_meshes.push(Rc::new(RefCell::new(ground_mesh)));
        }
    }
    all_meshes
}
