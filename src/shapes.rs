use crate::{float_precision::Vector3f64, math::{Line3D, Plane}};

#[derive(Debug, Clone)]
pub struct LineSegment3D {
    pub point1: Vector3f64,
    pub point2: Vector3f64,
}

impl LineSegment3D {
    pub fn new(point1: Vector3f64, point2: Vector3f64) -> Self {
        Self {
            point1,
            point2,
        }
    }
    
    pub fn length(&self) -> f64 {
        (self.point2 - self.point1).length()
    }

    /**
    The two line segments passed in should be colinear
    Calculates the amount of overlap between two line segments (returns `None` if they don't overlap)

    Requires the input be rounded
     */
    pub fn calculate_min_translation(
        &self,
        other: &LineSegment3D,
    ) -> Option<f64> {
        let base_line = Line3D::from_line_segment(self);

        let mut t1 = base_line.find_t_from_point(other.point1);
        let mut t2 = base_line.find_t_from_point(other.point2);

        // Swap them if t1 is greater than t2
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }

        if t1 < 0.0 && t2 < 0.0 || t1 > 1.0 && t2 > 1.0 {
            return None;
        }

        let distance = 
        if t1 > 0.0 && t1 < 1.0 && t2 > 0.0 && t2 < 1.0 {
            t2.min(1.0 - t1)
        }
        else if t1 < 0.0 {
            t2
        }
        else if t2 > 1.0 {
            1.0 - t1
        }
        else {
            t2
        };

        Some(distance * base_line.v.length())
    }
}

#[derive(Debug, Clone)]
pub struct Polygon {
    pub points: Vec<Vector3f64>,
}

#[derive(Debug, Clone)]
pub struct Circle {
    pub center: Vector3f64,
    pub radius: f64,
    pub normal: Vector3f64,
}

impl Circle {
    pub fn new(center: Vector3f64, radius: f64, normal: Vector3f64) -> Self {
        Self { center, radius, normal }
    }
}

impl Polygon {
    /**
    Points have to be sorted being passed to the polygon constructor
     */
    pub fn new(points: Vec<Vector3f64>) -> Self {
        Polygon { points }
    }

    pub fn get_edges(&self) -> Vec<LineSegment3D> {
        let mut edges: Vec<LineSegment3D> = Vec::with_capacity(self.points.len());
        for i in 0..self.points.len() {
            edges.push(LineSegment3D::new(self.points[i], self.points[(i + 1) % self.points.len()]));
        }
        edges
    }

    /**
    Calculates any of the orthogonal planes necessary for the separating axis theorem
    Some of these planes could be the same as ones already checked, so this isn't quite optimal
     */
    pub fn calculate_orthogonal_planes(&self) -> Vec<Plane> {
        let polygon_as_plane = self.convert_to_plane();
        let mut planes: Vec<Plane> = Vec::with_capacity(self.points.len());
        for edge in self.get_edges() {
            let edge_vec = edge.point2 - edge.point1;
            let new_normal = polygon_as_plane.n.cross(edge_vec);
            planes.push(Plane::from_point_and_normal(edge.point1, new_normal));
        }
        planes
    }

    pub fn convert_to_plane(&self) -> Plane {
        // Can't calculate a plane if there are less than 3 points
        assert!(self.points.len() >= 3);

        // Just make it the first point because why not, we just need *a* point
        let p0 = self.points.first().unwrap();

        // Calculate cross product

        // Every polygon should have three points, so should be able to safely retrieve the second
        // and third points in the vector
        let p1 = self.points.get(1).unwrap();
        let p2 = self.points.get(2).unwrap();

        let v1 = Vector3f64::new(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);

        let v2 = Vector3f64::new(p2.x - p0.x, p2.y - p0.y, p2.z - p0.z);

        let cross = v1.cross(v2).normalized();

        Plane::from_point_and_normal(*p0, cross)
    }
}

#[derive(Debug, Clone)]
pub struct Sphere {
    pub center: Vector3f64,
    pub radius: f64,
}

impl Sphere {
    pub fn new(center: Vector3f64, radius: f64) -> Self {
        Self { center, radius }
    }
}

#[derive(Debug, Clone)]
pub struct RectangularPrism {
    // The root is an absolute point and is the bottom left point of the front face
    pub root: Vector3f64,

    pub length: f64,
    pub width: f64,
    pub height: f64,

    pub bounding_circle_radius: f64,
}

impl RectangularPrism {
    pub fn new(root: Vector3f64, length: f64, width: f64, height: f64) -> Self {
        // Calculate the furthest point
        let bounding_circle_radius = Vector3f64::new(width / 2.0, height / 2.0, length / 2.0).length();
        RectangularPrism {
            root,
            length,
            width,
            height,
            bounding_circle_radius,
        }
    }

    pub fn get_vertices(&self) -> Vec<Vector3f64> {
        vec![
            Vector3f64::new(self.root.x, self.root.y, self.root.z),
            Vector3f64::new(self.root.x + self.width, self.root.y, self.root.z),
            Vector3f64::new(self.root.x, self.root.y + self.height, self.root.z),
            Vector3f64::new(
                self.root.x + self.width,
                self.root.y + self.height,
                self.root.z,
            ),
            Vector3f64::new(self.root.x, self.root.y, self.root.z + self.length),
            Vector3f64::new(
                self.root.x + self.width,
                self.root.y,
                self.root.z + self.length,
            ),
            Vector3f64::new(
                self.root.x,
                self.root.y + self.height,
                self.root.z + self.length,
            ),
            Vector3f64::new(
                self.root.x + self.width,
                self.root.y + self.height,
                self.root.z + self.length,
            ),
        ]
    }
}