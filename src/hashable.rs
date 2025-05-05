use crate::math::*;
use crate::float_precision::*;
use std::hash::{Hash, Hasher};
use ordered_float::OrderedFloat;

#[derive(Eq, PartialEq)]
pub struct HashablePlane {
    pub p0: HashableVector3,
    pub n: HashableVector3,
    pub d: OrderedFloat<f64>,
}

#[derive(Eq, PartialEq, Hash)]
pub struct HashableVector3 {
    pub x: OrderedFloat<f64>,
    pub y: OrderedFloat<f64>,
    pub z: OrderedFloat<f64>,
}

#[derive(Eq, PartialEq, Hash)]
pub struct HashableVector2 {
    pub x: OrderedFloat<f64>,
    pub y: OrderedFloat<f64>,
}

impl HashablePlane {
    pub fn new(p0: HashableVector3, n: HashableVector3, d: OrderedFloat<f64>) -> Self {
        Self {
            p0,
            n,
            d,
        }
    }
}

impl Hash for HashablePlane {
    fn hash<H: Hasher>(&self, state: &mut H) {
        // We only want to hash the normal when checking if two planes are the same (at least for my use case)
        // Also this is kind of asking a lot, but I want the normals to be sanitized before they even get here
        self.n.hash(state);
    }
}

impl From<HashablePlane> for Plane {
    fn from(hp: HashablePlane) -> Self {
        Plane::from_point_and_normal(hp.p0.into(), hp.n.into())
    }
}

impl From<Plane> for HashablePlane {
    fn from(p: Plane) -> Self {
        HashablePlane::new(p.p0.into(), p.n.into(), OrderedFloat(p.d))
    }
}

impl HashableVector3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            x: OrderedFloat(x),
            y: OrderedFloat(y),
            z: OrderedFloat(z),
        }
    }
}

impl From<HashableVector3> for Vector3f64 {
    fn from(hv: HashableVector3) -> Self {
        Vector3f64::new(hv.x.0, hv.y.0, hv.z.0)
    }
}

impl From<Vector3f64> for HashableVector3 {
    fn from(v: Vector3f64) -> HashableVector3 {
        HashableVector3::new(v.x, v.y, v.z)
    }
}

impl HashableVector2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            x: OrderedFloat(x),
            y: OrderedFloat(y),
        }
    }
}

impl From<HashableVector2> for Vector2f64 {
    fn from(hv: HashableVector2) -> Self {
        Vector2f64::new(hv.x.0, hv.y.0)
    }
}

impl From<Vector2f64> for HashableVector2 {
    fn from(v: Vector2f64) -> HashableVector2 {
        HashableVector2::new(v.x, v.y)
    }
}
