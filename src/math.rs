use std::cell::Ref;
use std::collections::HashSet;
use crate::hashable::HashableVector2;
use crate::heap::custom_heap::*;
use crate::render::*;
use crate::float_precision::*;
use crate::shapes::Circle;
use crate::shapes::{LineSegment3D, Polygon};

pub fn f64_min(nums: &[f64]) -> f64 {
    let mut min = nums[0];
    for num in nums {
        if *num < min {
            min = *num
        }
    }
    min
}

pub fn f64_max(nums: &[f64]) -> f64 {
    let mut max = nums[0];
    for num in nums {
        if *num > max {
            max = *num
        }
    }
    max
}

pub fn calculate_cos_of_angle(vec1: &Vector2f64, vec2: &Vector2f64) -> f64 {
    if vec1.length() == 0.0 || vec2.length() == 0.0 {
        return 1.0;
    }

    vec1.dot(*vec2) / (vec1.length() * vec2.length())
}

#[derive(Debug, Clone)]
pub struct Plane {
    // These are the variables necessary to define a plane in 3 space
    // ax + by + cz = d
    // n = <a, b, c>
    pub n: Vector3f64,
    pub d: f64,

    // The point (x0, y0, z0) is an absolute point
    pub p0: Vector3f64,
}

impl Plane {
    // A plane can be uniquely defined with a normal vector and a point
    pub fn from_point_and_normal(p0: Vector3f64, n: Vector3f64) -> Self {
        let mut normal = n.normalized();
        if normal.y < 0.0 {
            normal *= -1.0;
        }
        let d = p0.x * normal.x + p0.y * normal.y + p0.z * normal.z;
        Self { n: normal, d, p0 }
    }

    /**
    Projects a point in 3-space onto the plane
     */
    pub fn project_point(&self, point: &Vector3f64) -> Vector3f64 {
        let d_vec = *point - self.p0;
        if precise_equal(d_vec.length(), 0.0) {
            return *point;
        }
        
        let proj = self.n * d_vec.dot(self.n) / (self.n.dot(self.n));

        *point - proj
    }

    pub fn project_mesh(&self, mesh: MeshShape) -> SATAble2D {
        // Project each individual point onto the plane
        let projected_points: Vec<Vector3f64>;
        match mesh {
            MeshShape::RectangularPrism(prism) => {
                projected_points = prism
                    .get_vertices()
                    .iter()
                    .map(|point| {
                        self.project_point(point)
                    })
                    .collect();
            },
            MeshShape::GroundMesh(ground_mesh) => {
                projected_points = ground_mesh
                    .get_vertices()
                    .iter()
                    .map(|point| {
                        self.project_point(point)
                    })
                    .collect();
            },
            MeshShape::Sphere(sphere) => {
                return SATAble2D::Circle(Circle::new(sphere.center, sphere.radius, self.n));
            },
        };

        // Convert the points to 2D
        let point_projector = TwoDimensionalPointProjector::new(self.clone());
        let mut two_dimensional_points = point_projector.project_into_2d(&projected_points);

        two_dimensional_points.iter_mut().for_each(|point| {
            point.0.x = f64_round(point.0.x);
            point.0.y = f64_round(point.0.y);
        });

        // Get the axis for which all points will be compared to
        let comparison_axis = ComparisonAxis::new(&two_dimensional_points);

        let sorted_points = sort_points_clockwise(&mut two_dimensional_points, &comparison_axis);

        // Perform Graham's check to get the points of the polygon
        let bounding_points = graham_scan(&sorted_points);

        assert!(bounding_points.len() > 1);
        let projected_points = point_projector.project_into_3d(&bounding_points, &projected_points);
        if projected_points.len() == 2 {
            SATAble2D::LineSegment3D(LineSegment3D::new(projected_points[0], projected_points[1]))
        }
        else {
            SATAble2D::Polygon(Polygon::new(projected_points))
        }
    }
}

#[derive(Debug, Clone)]
pub struct Line3D {
    // These are the variables necessary to define the parametric equations of a line in 3 space
    // x: x0 + at
    // y: y0 + yt
    // z: z0 + zt
    pub p0: Vector3f64,
    pub v: Vector3f64,
}

impl Line3D {
    pub fn from_point_and_parallel_vec(p0: Vector3f64, v: Vector3f64) -> Self {
        Self { p0, v }
    }

    pub fn from_line_segment(line_segment: &LineSegment3D) -> Self {
        let vec = line_segment.point2 - line_segment.point1;
        Line3D::from_point_and_parallel_vec(line_segment.point1, vec)
    }

    /**
     * Finds the value of t such the equations for the point of the line are satasfied for the given point
     */
    pub fn find_t_from_point(&self, point: Vector3f64) -> f64 {
        let vec2 = point - self.p0;
        let abs_t = vec2.length() / self.v.length();

        // Now determine the direction
        let p1 = self.p0 + vec2;
        let p2 = self.p0 - vec2;

        // Determine which point is closer to the point in the direction of the terminal point of the root vec
        let terminal_point = self.p0 + self.v;

        if (terminal_point - p1).length() > (terminal_point - p2).length() {
            // If we get closer when we subtract the vector, then we are going away, so t is negative
            abs_t * -1.0
        } else {
            abs_t
        }
    }

    pub fn project_satable_object(&self, obj: &SATAble2D) -> LineSegment3D {
        let points: Vec<Vector3f64>;
        match obj {
            SATAble2D::LineSegment3D(line_segment) => {
                points = vec![line_segment.point1, line_segment.point2];
            },
            SATAble2D::Polygon(polygon) => {
                points = polygon.points.clone();
            },
            SATAble2D::Circle(circle) => {
                let distance = self.v.normalized() * circle.radius;
                return LineSegment3D::new(self.p0 + distance, self.p0 - distance);
            },
        }

        let line_dir = self.v.normalized();
        let mut t_min = f64::INFINITY;
        let mut t_max = f64::NEG_INFINITY;

        for point in points {
            let vec_to_point = point - self.p0;

            let t = vec_to_point.dot(line_dir);

            if t < t_min {
                t_min = t;
            }
            if t > t_max {
                t_max = t;
            }
        }

        let start_point = self.p0 + line_dir * t_min;
        let end_point = self.p0 + line_dir * t_max;

        LineSegment3D::new(start_point, end_point)
    }
}

#[derive(Debug)]
pub struct ComparisonAxis {
    pub root_point: Vector2f64,
    pub helper_point: Vector2f64,
}

fn find_root_point(points: &[(Vector2f64, usize)]) -> &(Vector2f64, usize) {
    let mut min_index: usize = 0;
    for i in 0..points.len() {
        if points[i].0.y < points[min_index].0.y || (points[i].0.y == points[min_index].0.y && points[i].0.x < points[min_index].0.x) {
            min_index = i;
        }
    }

    &points[min_index]
}

pub fn sort_points_clockwise(points: &mut [(Vector2f64, usize)], comparison_axis: &ComparisonAxis) -> Vec<(Vector2f64, usize)> {
    // First remove duplicate points
    let mut unique_points: Vec<(Vector2f64, usize)> = Vec::new();
    let mut seen = HashSet::new();
    for point in points.iter() {
        if seen.insert(HashableVector2::from(point.0)) {
            unique_points.push(*point);
        }
    }

    // Now sort the unique points
    let mut sorting_points: Vec<(Vector2f64, usize)> = vec![(Vector2f64::new(0.0, 0.0), 0); unique_points.len() - 1];

    let mut new_i: usize = 0;
    let mut root_point_i: usize = 0;
    for point in unique_points.iter() {
        if point.0 != comparison_axis.root_point {
            sorting_points[new_i] = *point;
            new_i += 1;
        } else {
            root_point_i = new_i;
        }
    }

    // Sort without the root point because that can mess things up
    heapsort(comparison_axis, &mut sorting_points);

    // Create a new array with the correct size
    let mut new_points: Vec<(Vector2f64, usize)> = vec![(Vector2f64::new(0.0, 0.0), 0); sorting_points.len() + 1];
    new_points[0] = (comparison_axis.root_point, root_point_i);
    new_points[1..(sorting_points.len() + 1)].copy_from_slice(&sorting_points[..]);

    new_points
}

fn is_counter_clockwise(p: Vector2f64, q: Vector2f64, r: Vector2f64) -> bool {
    (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x) < 0.0
}

pub fn graham_scan(sorted_points: &[(Vector2f64, usize)]) -> Vec<(Vector2f64, usize)> {
    let mut stack: Vec<(Vector2f64, usize)> = Vec::new();

    for point in sorted_points {
        while stack.len() > 1 && is_counter_clockwise(
            stack[stack.len() - 1].0,
            stack[stack.len() - 2].0,
            point.0
        ) {
            _ = stack.pop();
        }

        stack.push(*point);
    }

    // Check if the last point is colinear with the first two points
    if stack.len() > 2 && !is_counter_clockwise(
        stack[0].0,
        stack[stack.len() - 2].0,
        stack[stack.len() - 1].0
    ) {
        _ = stack.pop();
    }

    stack
}

impl ComparisonAxis {
    pub fn new(points: &[(Vector2f64, usize)]) -> ComparisonAxis {
        let root_point = find_root_point(points);
        let helper_point = Vector2f64::new(root_point.0.x - 1.0, root_point.0.y);
        ComparisonAxis { helper_point, root_point: root_point.0 }
    }
}

// Planes that can be described by a single basis vector
#[derive(Debug)]
pub enum BasePlane {
    XZ,
    YZ,
    XY,
}

pub struct TwoDimensionalPointProjector {
    base_plane: BasePlane,
}

impl TwoDimensionalPointProjector {
    pub fn new(plane: Plane) -> Self {
        // We don't actually need j hat
        let i = Vector3f64::new(1.0, 0.0, 0.0);
        //let j = Vector3f64::new(0.0, 1.0, 0.0,);
        let k = Vector3f64::new(0.0, 0.0, 1.0);

        // If the dot of the normal and the unit vector that defines the base plane is not the normal, then it's not parallel
        // i -> yz plane, j -> xz, k -> xy

        let magnitude_normal = plane.n.length();
        let base_plane: BasePlane;
        if plane.n.dot(k).abs() == magnitude_normal {
            base_plane = BasePlane::XY;
        } else if plane.n.dot(i).abs() == magnitude_normal {
            base_plane = BasePlane::YZ;
        } else {
            base_plane = BasePlane::XZ;
        }

        Self {
            base_plane,
        }
    }

    // When we project into 2 dimensions, we can just strip one of the coordinates, but when projecting out to remove a lot of the floating point precision
    // we can just keep an index of the points we want to define the outer object, so we keep track of the index each one maps to from dimension to dimension
    pub fn project_into_2d(&self, points: &[Vector3f64]) -> Vec<(Vector2f64, usize)> {
        match self.base_plane {
            BasePlane::XY => {
                // We map onto the x y plane, so remove the z coordinate
                points
                    .iter()
                    .enumerate()
                    .map(|(index, point)| (Vector2f64::new(point.x, point.y), index))
                    .collect()
            }
            BasePlane::XZ => {
                // We map onto the x z plane, so remove the y coordinate
                points
                    .iter()
                    .enumerate()
                    .map(|(index, point)| (Vector2f64::new(point.x, point.z), index))
                    .collect()
            }
            BasePlane::YZ => {
                // We map onto the y z plane, so remove the x coordinate
                points
                    .iter()
                    .enumerate()
                    .map(|(index, point)| (Vector2f64::new(point.y, point.z), index))
                    .collect()
            }
        }
    }

    /**
    Takes the three initial points that can be used to define the points, the actual points that were mapped to 2D, and the plane that all the points were mapped onto to calculate the positions of the points in 3-space
     */
    pub fn project_into_3d(
        &self,
        points: &[(Vector2f64, usize)],
        original_points: &[Vector3f64],
    ) -> Vec<Vector3f64> {
        points
            .iter()
            .map(|(_, index)| original_points[*index])
            .collect()
    }
}
