use crate::hashable::*;
use crate::math::Line3D;
use crate::math::Plane;
use crate::render::*;
use crate::float_precision::*;
use crate::shapes::LineSegment3D;
use crate::shapes::Polygon;
use crate::shapes::Sphere;
use std::collections::HashSet;
use std::fmt::Debug;
use std::cell::{RefCell, Ref};
use std::rc::Rc;

pub trait Physical: Debug {
    fn get_center(&self) -> Vector3f64;
    fn get_bounding_circle_radius(&self) -> f64;
    fn get_mesh(&self) -> Ref<MeshShape>;
}

pub trait Dynamic {
    // Moves the object and its components by change
    fn move_by(&mut self, change: Vector3f64);
}

pub trait Static {}

#[derive(Debug)]
pub struct StaticBody {
    pub pos: Vector3f64,
    pub mesh: Rc<RefCell<MeshShape>>,
}

impl Static for StaticBody {}

impl Physical for StaticBody {
    fn get_mesh(&self) -> Ref<MeshShape> {
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
    pub fn new(pos: Vector3f64, mesh: Rc<RefCell<MeshShape>>) -> Self {
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
    pub mesh: Rc<RefCell<MeshShape>>,
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

    fn get_mesh(&self) -> Ref<MeshShape> {
        return self.mesh.borrow();
    }
}

impl DynamicBody {
    pub fn new(pos: Vector3f64, mesh: Rc<RefCell<MeshShape>>) -> Self {
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
            mesh1: Ref<MeshShape>,
            mesh2: Ref<MeshShape>,
        ) {
            // The direction vector is going to either be correct or flipped, so walk in each direction, and the one that gets further from the other mesh's center is the right direction
            let pos1 = mesh2.get_center() + *direction_vec;
            let pos2 = mesh2.get_center() - *direction_vec;

            if (pos1 - mesh1.get_center()).length() > (pos2 - mesh1.get_center()).length() {
                *direction_vec *= -1.0;
            }
        }

        // Consider the case of spheres etc
        let mesh1 = self.get_mesh().to_owned();
        let mesh2 = other.get_mesh().to_owned();

        let mut sphere_mesh: Option<Sphere> = None;
        let mut other_mesh: Option<MeshShape> = None;

        match (mesh1, mesh2) {
            (MeshShape::Sphere(sphere1), MeshShape::Sphere(sphere2)) => {
                let distance = (sphere1.center - sphere2.center).length();
                let radius_sum = sphere1.radius + sphere2.radius;
                if distance > radius_sum {
                    return None;
                }

                let direction = (sphere2.center - sphere1.center).normalized();
                return Some(direction * (radius_sum - distance));
            }
            (MeshShape::Sphere(sphere_obj), other_obj) => {
                sphere_mesh = Some(sphere_obj);
                other_mesh = Some(other_obj);
            }
            (other_obj, MeshShape::Sphere(sphere_obj)) => {
                sphere_mesh = Some(sphere_obj);
                other_mesh = Some(other_obj);
            }
            _ => {}
        }

        if let Some(sphere) = sphere_mesh {
            if let Some(other) = other_mesh {
                let faces = other.get_polygons();

                let mut direction = Vector3f64::new(0.0, 0.0, 0.0);
                let mut min_distance = f64::MAX;
                for face in faces {
                    let displacement = face.minimum_displacement(sphere.center);
                    if displacement.length() < min_distance {
                        min_distance = displacement.length();
                        if displacement.length() > 0.0 {
                            direction = displacement.normalized();
                        }
                    }
                }

                if min_distance > sphere.radius {
                    return None;
                }

                align_direction_vec(&mut direction, self.get_mesh(), RefCell::new(other).borrow());
                return Some(direction * (sphere.radius - min_distance));
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
            let proj1 = proj_plane.project_mesh(self.get_mesh().to_owned());
            let proj2 = proj_plane.project_mesh(other.get_mesh().to_owned());

            let collision = collision_detection_2d(proj1, proj2, proj_plane.n);
            match collision {
                Some(collision) => {
                    if collision.0 < min_overlap {
                        min_overlap = collision.0;
                        direction = collision.1;
                    }
                }
                None => {
                    return None; // If we are colliding, no matter where we are looking they should be colliding
                }
            }
        }

        align_direction_vec(&mut direction, self.get_mesh(), other.get_mesh());
        Some(direction * min_overlap)
    }
}
