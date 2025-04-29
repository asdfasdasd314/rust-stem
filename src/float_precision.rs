use raylib::prelude::{Vector2, Vector3};
use std::ops::{Mul, Div, Add, Sub, AddAssign, MulAssign};

/*
This stuff is for handling floating point precision errors
*/

// `PRECISION` is the number of decimals of precision
pub const PRECISION: i32 = -1;

pub fn f64_round(x: f64) -> f64 {
    if PRECISION < 0 {
        return x;
    }
    (x * (10.0_f64).powi(PRECISION)).round() / (10.0_f64).powi(PRECISION)
}

pub fn broad_equal(x: f64, y: f64) -> bool {
    // I'm going to try using relative error and compare it to f64::EPSILON
    // Alternatively, they could be subtracted and then compared to f64::EPSILON, but I think this will ensure precision further out(?)
    // If the two are nearly the same then it doesn't matter which one is divided because they will be roughly the same, just differing by a floating point precision amount
    ((x - y) / x).abs() <= f64::EPSILON
}

pub fn precise_equal(x: f64, y: f64) -> bool {
    // Because there are situations where equality down to potentially millionths is necessary, this one uses mantissa digits to determine if they are roughly equal
    // This won't come up most of the time for my use case, but just in case
    (x - y).abs() <= f64::EPSILON
}

pub fn vector3_round(v: Vector3f64) -> Vector3f64 {
    Vector3f64::new(
        f64_round(v.x),
        f64_round(v.y),
        f64_round(v.z),
    )
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub struct Vector2f64 {
    pub x: f64,
    pub y: f64,
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub struct Vector3f64 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector2f64 {
    pub fn new(x: f64, y: f64) -> Self {
        Vector2f64 {
            x,
            y,
        }
    }

    pub fn length(&self) -> f64 {
        ((self.x * self.x) + (self.y * self.y)).sqrt()
    }

    pub fn dot(&self, other: Vector2f64) -> f64 {
        self.x * other.x + self.y * other.y
    }

    pub fn normalized(&self) -> Vector2f64 {
        *self / self.length() 
    }
}

impl Vector3f64 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vector3f64 {
            x,
            y,
            z,
        }
    }

    pub fn length(&self) -> f64 {
        ((self.x * self.x) + (self.y * self.y) + (self.z * self.z)).sqrt()
    }

    pub fn cross(&self, other: Vector3f64) -> Vector3f64 {
        Vector3f64 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn dot(&self, other: Vector3f64) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z 
    }

    pub fn normalized(&self) -> Vector3f64 {
        *self / self.length()
    }
}

impl From<Vector2> for Vector2f64 {
    fn from(v: Vector2) -> Vector2f64 {
        Vector2f64::new(v.x as f64, v.y as f64)
    } 
}

impl From<Vector3> for Vector3f64 {
    fn from(v: Vector3) -> Vector3f64 {
        Vector3f64::new(v.x as f64, v.y as f64, v.z as f64)
    } 
}

impl From<Vector2f64> for Vector2 {
    fn from(v: Vector2f64) -> Vector2 {
        Vector2::new(v.x as f32, v.y as f32)
    } 
}

impl From<Vector3f64> for Vector3 {
    fn from(v: Vector3f64) -> Vector3 {
        Vector3::new(v.x as f32, v.y as f32, v.z as f32)
    } 
}

impl Mul<f64> for Vector2f64 {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self::Output {
        Vector2f64::new(self.x * scalar, self.y * scalar)
    }
}

impl Mul<f64> for Vector3f64 {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self::Output {
        Vector3f64::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}

impl MulAssign<f64> for Vector2f64 {
    fn mul_assign(&mut self, scalar: f64) {
        self.x *= scalar;
        self.y *= scalar;
    }
}

impl MulAssign<f64> for Vector3f64 {
    fn mul_assign(&mut self, scalar: f64) {
        self.x *= scalar;
        self.y *= scalar;
        self.z *= scalar;
    }
}

impl Div<f64> for Vector2f64 {
    type Output = Self;
    fn div(self, scalar: f64) -> Self::Output {
        Vector2f64::new(self.x / scalar, self.y / scalar)
    }
}

impl Div<f64> for Vector3f64 {
    type Output = Self;
    fn div(self, scalar: f64) -> Self::Output {
        Vector3f64::new(self.x / scalar, self.y / scalar, self.z / scalar)
    }
}

impl Add<Vector2f64> for Vector2f64 {
    type Output = Self;

    fn add(self, v: Vector2f64) -> Self::Output {
        Vector2f64::new(self.x + v.x, self.y + v.y)
    }
}

impl Add<Vector3f64> for Vector3f64 {
    type Output = Self;

    fn add(self, v: Vector3f64) -> Self::Output {
        Vector3f64::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}

impl Sub<Vector2f64> for Vector2f64 {
    type Output = Self;

    fn sub(self, v: Vector2f64) -> Self::Output {
        Vector2f64::new(self.x - v.x, self.y - v.y)
    }
}

impl Sub<Vector3f64> for Vector3f64 {
    type Output = Self;

    fn sub(self, v: Vector3f64) -> Self::Output {
        Vector3f64::new(self.x - v.x, self.y - v.y, self.z - v.z)
    }
}

impl AddAssign<Vector2f64> for Vector2f64 {
    fn add_assign(&mut self, v: Vector2f64) {
        self.x += v.x;
        self.y += v.y;
    }
}

impl AddAssign<Vector3f64> for Vector3f64 {
    fn add_assign(&mut self, v: Vector3f64) {
        self.x += v.x;
        self.y += v.y;
        self.z += v.z;
    }
}
