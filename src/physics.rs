use crate::hashable::*;
use crate::math_util::Plane;
use crate::render::*;
use crate::float_precision::*;
use std::collections::HashSet;
use std::fmt::Debug;
use std::cell::{RefCell, Ref};
use std::rc::Rc;

pub trait Physical: Debug {
    fn get_center(&self) -> Vector3f64;
    fn get_bounding_circle_radius(&self) -> f64;
    fn get_mesh(&self) -> Ref<dyn MeshShape>;
}

pub trait Dynamic {
    // Moves the object and its components by change
    fn move_by(&mut self, change: Vector3f64);
}

pub trait Static {}

#[derive(Debug)]
pub struct StaticBody {
    pub pos: Vector3f64,
    pub mesh: Rc<RefCell<dyn MeshShape>>,
}

impl Static for StaticBody {}

impl Physical for StaticBody {
    fn get_mesh(&self) -> Ref<dyn MeshShape> {
        return self.mesh.borrow();
    }
    fn get_bounding_circle_radius(&self) -> f64 {
        return self.mesh.borrow().get_bounding_circle_radius();
    }
    fn get_center(&self) -> Vector3f64 {
        return self.mesh.borrow().get_center();
    }
}

impl StaticBody {
    pub fn new(pos: Vector3f64, mesh: Rc<RefCell<dyn MeshShape>>) -> Self {
        Self {
            pos,
            mesh,
        }
    }
}

// This object is a basic object that interacts with physics
#[derive(Debug)]
pub struct DynamicBody {
    pub pos: Vector3f64,
    pub mesh: Rc<RefCell<dyn MeshShape>>,
}

impl Dynamic for DynamicBody {
    fn move_by(&mut self, change: Vector3f64) {
        self.pos += change;
        self.mesh.borrow_mut().move_by(change);
    }
}

impl Physical for DynamicBody {
    fn get_center(&self) -> Vector3f64 {
        return self.mesh.borrow().get_center();
    }

    fn get_bounding_circle_radius(&self) -> f64 {
        return self.mesh.borrow().get_bounding_circle_radius();
    }

    fn get_mesh(&self) -> Ref<dyn MeshShape> {
        return self.mesh.borrow();
    }
}

impl DynamicBody {
    pub fn new(pos: Vector3f64, mesh: Rc<RefCell<dyn MeshShape>>) -> Self {
        DynamicBody {
            pos,
            mesh,
        }
    }

    /**
    Returns the mtv to snap the objects onto the edges of each other while not intersecting
    If the objects aren't colliding, then returns `None`
    The MTV should be applied to the dynamic body (root body) of the collision because it's designed to move it out of the other body
     */
    pub fn collides_with(&self, other: &dyn Physical) -> Option<Vector3f64> {
        // We need to find the angle when we look at there is a minimum overlap
        fn align_direction_vec(
            direction_vec: &mut Vector3f64,
            dynamic_body_mesh: Ref<dyn MeshShape>,
            other_mesh: Ref<dyn MeshShape>,
        ) {
            // The direction vector is going to either be correct, or flipped, so walk in each direction, and the one that gets further from the other mesh's center is the right direction
            let dynamic_body_center = dynamic_body_mesh.get_center();
            let other_body_center = other_mesh.get_center();

            let center1 = dynamic_body_center + *direction_vec;
            let center2 = dynamic_body_center - *direction_vec;

            // If adding the direction vector walks in the wrong direction
            if (other_body_center - center1).length() < (other_body_center - center2).length() {
                *direction_vec *= -1.0;
            }
        }

        let mut min_overlap = f64::MAX;
        let mut direction: Vector3f64 = Vector3f64::new(0.0, 0.0, 0.0);
        let mut planes: HashSet<HashablePlane> = HashSet::new();
        let mut all_polygons: Vec<Polygon> = self.get_mesh().get_polygons();
        all_polygons.extend(other.get_mesh().get_polygons());
        for polygon in all_polygons {
            for plane in polygon.calculate_orthogonal_planes() {
                planes.insert(plane.into());
            }
        }

        for plane in planes {
            let proj_plane = Plane::from(plane);
            let proj1 = proj_plane.project_mesh(self.get_mesh());
            let proj2 = proj_plane.project_mesh(other.get_mesh());
            let collision = collision_detection_2d(proj1, proj2, proj_plane.n);
            match collision {
                Some(collision) => {
                    if collision.0 < min_overlap {
                        min_overlap = collision.0;
                        direction = collision.1;
                    }
                }
                None=> {
                    return None;
                }
            }
        }

        align_direction_vec(&mut direction, self.get_mesh(), other.get_mesh());

        
        // Here's all I'm going to say...
        // I have been debugging this floating point precision error for a week
        // This works 93% of the time, so I'm just going to run with it
        // 0.12
        Some(direction * min_overlap)
    }
}
