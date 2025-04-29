use std::cell::Ref;
use crate::heap::custom_heap::*;
use crate::render::*;
use crate::float_precision::*;

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

pub enum PlaneIntersection {
    Line(Line3D),
    Infinite, // In this case the planes contain each other
    None,
}

pub enum LineIntersection {
    Point(Vector2f64),
    Infinite, // In this case the lines are parallel and contain each other
    None,
}

// Theta sweeps across the x-z plane starting at the positive x-axis and goes up to the positive y-axis
// Phi comes down from the positive y-axis
pub struct SphericalAngle {
    pub theta: f64,
    pub phi: f64,
}

impl SphericalAngle {
    pub const fn new(theta: f64, phi: f64) -> SphericalAngle {
        SphericalAngle { theta, phi }
    }
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
    // A plane can also be uniquely defined by three points
    pub fn from_three_points(points: (&Vector3f64, &Vector3f64, &Vector3f64)) -> Self {
        let u = *points.2 - *points.0;
        let v = *points.1 - *points.0;

        let mut normal = u.cross(v).normalized();
        if normal.y < 0.0 {
            normal *= -1.0;
        }

        Self {
            n: normal,
            d: normal.x * points.0.x + normal.y * points.0.y + normal.z * points.0.z,
            p0: *points.0,
        }
    }

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

    pub fn project_mesh(&self, mesh: Ref<dyn MeshShape>) -> Box<dyn SATAble2D> {
        // Project each individual point onto the plane
        let projected_points: Vec<Vector3f64> = mesh
            .get_vertices()
            .iter()
            .map(|point| {
                self.project_point(point)
            })
            .collect();

        // Convert the points to 2D
        let point_projector = TwoDimensionalPointProjector::new(self.clone());
        let mut two_dimensional_points = point_projector.project_into_2d(&projected_points);

        two_dimensional_points.iter_mut().for_each(|point| {
            point.0.x = f64_round(point.0.x);
            point.0.y = f64_round(point.0.y);
        });

        // Get the axis for which all points will be compared to
        let comparison_axis = ComparisonAxis::new(&two_dimensional_points);

        // Perform Graham's check to get the points of the polygon

        // Sort based on the root point, this also removes duplicate points
        heapsort(&comparison_axis, &mut two_dimensional_points);
        let bounding_points = graham_scan(&two_dimensional_points);

        assert!(bounding_points.len() > 1);
        let projected_points = point_projector.project_into_3d(&bounding_points, &projected_points);
        if projected_points.len() == 2 {
            Box::new(LineSegment3D::new(projected_points[0], projected_points[1]))
        } else {
            Box::new(Polygon::new(projected_points))
        }
    }

    pub fn intersects(&self, other: &Plane) -> bool {
        // Two planes intersect if they are not parallel, or if they are parallel then they exist at the same point
        if self.n.x.abs() != other.n.x.abs()
            || self.n.y.abs() != other.n.y.abs()
            || self.n.z.abs() != other.n.z.abs()
        {
            return true;
        }

        // Check if the planes contain the same point
        // ax0 = d0 = ax1 = d1
        self.d / self.p0.x == other.d / other.p0.x
    }

    /**
    This function is basically useless, because it was part of the old collision detection but I don't really want to remove it
    This function assumes it has been checked that the planes intersect
     */
    pub fn calculate_line_intersection(&self, other: &Plane) -> PlaneIntersection {
        if !self.intersects(other) {
            panic!("The planes passed to calculate_line_intersection_between_planes should intersect at some point, and this should be checked");
        }

        let parallel = self.n.cross(other.n);

        // We just need a point, so we need to find a point that both plane1 and plane2 have

        // I'll call this TODO, but for now we can just say that they for sure they share a point a, so
        // let's just say it happens at z = 0
        // Now we have a0x + b0y = d0 = a1x + b1y = d1

        // We now have a system of equations that has a single solution instead of infinitely many
        // solutions (we just picked an arbitrary point)

        let line1: Line2D = 
            if self.n.x == 0.0 {
                Line2D::from_two_points(
                    Vector2f64::new(0.0, self.n.y),
                    Vector2f64::new(1.0, self.n.y),
                )
            } else {
                Line2D::from_two_points(
                    Vector2f64::new(0.0, self.n.y),
                    Vector2f64::new(self.n.x, 0.0),
                )
            };

        let line2: Line2D = 
            if other.n.x == 0.0 {
                Line2D::from_two_points(
                    Vector2f64::new(0.0, other.n.y),
                    Vector2f64::new(1.0, other.n.y),
                )
            } else {
                Line2D::from_two_points(
                    Vector2f64::new(0.0, other.n.y),
                    Vector2f64::new(other.n.x, 0.0),
                )
            };

        let line_intersection = line1.find_intersection(&line2);

        match line_intersection {
            LineIntersection::Point(point) => {
                // This is the most basic case where we have an actual point
                // Remember z0 = 0
                let p0 = Vector3f64::new(point.x, point.y, 0.0);
                PlaneIntersection::Line(Line3D::from_point_and_parallel_vec(p0, parallel))
            }
            LineIntersection::Infinite => {
                PlaneIntersection::Infinite
            }
            LineIntersection::None => {
                // We should have already panicked, there is no conceivable reason to end up here
                panic!("There is no possible way the logic should end up here because this test case was tested for twice now");
            }
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

    /**
     * Finds the point for the given t
     */
    pub fn find_point_from_t(&self, t: f64) -> Vector3f64 {
        Vector3f64::new(
            self.p0.x + self.v.x * t,
            self.p0.y + self.v.y * t,
            self.p0.z + self.v.z * t,
        )
    }

    pub fn project_polygon(&self, polygon: &Polygon) -> LineSegment3D {
        let line_dir = self.v.normalized();
        let points = &polygon.points;
        let mut t_min = f64::INFINITY;
        let mut t_max = f64::NEG_INFINITY;

        for point in points {
            let vec_to_point = *point - self.p0;

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

    pub fn project_satable_object(&self, obj: &Box<dyn SATAble2D>) -> LineSegment3D {
        let points = obj.get_vertices();

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

    /**
    Helper function for finding a projection
     */
    fn project_point_onto_line(&self, point: Vector3f64) -> Vector3f64 {
        let u = point - self.p0;
        // u_initial + projv(u) = projected point
        self.p0 + self.v * self.v.dot(u) / self.v.dot(self.v)
    }
}

// To avoid issues with divide by zero and whatnot we can just define every single like this where a line can uniquely be defined by any point on the line and a vector in the direction of the line
// This also works 100% perfect for vertical lines so there are no special cases, it is a bit more memory though (only maybe like 8 bytes though I think, 3 f64s vs 4 f64s + a little bit for pointers and whatnot)
#[derive(Debug)]
pub struct Line2D {
    p0: Vector2f64,
    v: Vector2f64,
}

impl Line2D {
    pub fn from_two_points(point1: Vector2f64, point2: Vector2f64) -> Self {
        Self {
            p0: point1,
            v: point2 - point1,
        }
    }

    pub fn find_intersection(&self, other: &Line2D) -> LineIntersection {
        // Check if the vector is parallel
        if self.v.dot(other.v.normalized()) == self.v.length() {
            // If the dot product of one v and the other one normalized is the magnitude of the first one then they are parallel because math

            let projected_tail = other.project_point(self.p0 + self.v);

            // If this is true then the projection did nothing, meaning that the lines are overlapping
            if projected_tail == self.p0 + self.v {
                return LineIntersection::Infinite;
            } else {
                return LineIntersection::None;
            }
        }
        let displacement_line = Line2D::from_two_points(self.p0, other.p0);
        let t: f64 = 
            // First check which projection to use
            if displacement_line.v.dot(self.v) == 0.0 {
                // If the lines are parallel project onto self.v
                let proj_other_to_self = self.project_point(other.v);
                proj_other_to_self.length() / self.v.length()
            } else {
                // It's theoretically possible we go past the point, so pick two points equidistant away and choose the direction that gets closer
                let point1 = self.p0 + self.v;
                let point2 = self.p0 - self.v;

                let proj_p1 = other.project_point(point1);
                let proj_p2 = other.project_point(point2);

                let p1_magnitude = (proj_p1 - point1).length();
                let p2_magnitude = (proj_p2 - point2).length();
                (p2_magnitude - p1_magnitude) / f64_min(&[p1_magnitude, p2_magnitude])
            };

        LineIntersection::Point(self.p0 + self.v * t)
    }

    fn project_point(&self, point: Vector2f64) -> Vector2f64 {
        let u = point - self.p0;
        // u_initial + projv(u) = projected point
        self.p0 + self.v * self.v.dot(u) / self.v.dot(self.v)
    }
}

// p0 is the bottom-leftmost point that can contain the polygon, this is usually a combination of the leftmost and bottommost points, but in the case that they are the same then it's that point 1 unit left in the x direction
// p1 is the root point of the polygon
#[derive(Debug)]
pub struct ComparisonAxis {
    pub p0: Vector2f64,
    pub p1: Vector2f64,
}

impl ComparisonAxis {
    pub fn new(points: &[(Vector2f64, usize)]) -> ComparisonAxis {
        let mut min_x_index = 0;
        let mut min_y_index = 0;
        for i in 0..points.len() {
            if points[i].0.y < points[min_y_index].0.y || (points[i].0.y == points[min_y_index].0.y && points[i].0.x < points[min_y_index].0.x) {
                min_y_index = i;
            }

            // We don't any more complicated logic for this because we just need the x-coordinate
            if points[i].0.x < points[min_x_index].0.x {
                min_x_index = i;
            }
        }

        let p1 = points[min_y_index].0;
        let p0 = Vector2f64::new(p1.x - 1.0, p1.y);

        ComparisonAxis { p0, p1 }
    }
}

/**
An implementation of Graham's scan that defines the convex polygon from a set of points in O(n) time
The reason the overall algorithm is nlogn is because the sorting takes nlogn
Yes I understand I'm working with a limited number of points, and often the data is nearly sorted so heapsort
and this worrying about time complexity is wrong, but I am doing this project TO LEARN, so it doesn't matter
 */
fn graham_scan(projected_points: &[(Vector2f64, usize)]) -> Vec<(Vector2f64, usize)> {
    fn is_counter_clockwise(p: &Vector2f64, q: &Vector2f64, r: &Vector2f64) -> bool {
        let cross = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
        if broad_equal(cross, 0.0) {
            return false;
        }
        (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x) < 0.0
    }

    let mut stack: Vec<(Vector2f64, usize)> = Vec::new();
    for point in projected_points {
        // The check for the length greater than 1 is done so it's possible to verify the angle
        while stack.len() > 1
            && !is_counter_clockwise(
                &stack[stack.len() - 1].0,
                &stack[stack.len() - 2].0,
                &point.0,
            )
        {
            stack.pop();
        }

        stack.push(*point);
    }

    // If we only have two points we may have a line, wihich is a valid polygon for my code
    if stack.len() > 2
        && !is_counter_clockwise(
            &stack[stack.len() - 1].0,
            &stack[stack.len() - 2].0,
            &stack[0].0,
        )
    {
        stack.pop();
    }

    stack
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

// Calculates the difference in between the angles of the vectors using the base_vec as the base
// **in degrees**
pub fn calculate_difference_in_angle(
    base_vec: &Vector3f64,
    compare_to_vec: &Vector3f64,
) -> SphericalAngle {
    // To make things easier, we'll transform these vectors so that base_vec is on the z-axis
    // We can do this by calculating the angle the base_vec makes with the origin
    // We know x, y, and z, so we can use conversions between ro, theta, and phi to find these changes in angle
    let base_vec_magnitude = base_vec.length();
    let compare_to_vec_magnitude = compare_to_vec.length();

    let base_vec_theta = (base_vec.y / (base_vec.x.powi(2) + base_vec.y.powi(2)).sqrt()).asin();
    let base_vec_phi = base_vec.z / base_vec_magnitude;

    let compare_to_vec_theta =
        (compare_to_vec.y / (compare_to_vec.x.powi(2) + compare_to_vec.y.powi(2)).sqrt()).asin();
    let compare_to_vec_phi = compare_to_vec.z / compare_to_vec_magnitude;

    SphericalAngle {
        theta: compare_to_vec_theta - base_vec_theta,
        phi: compare_to_vec_phi - base_vec_phi,
    }
}

// This function checks that given some angles, they surround the relative origin they were calculated from
pub fn check_angles_surround_relative_origin(angles: Vec<SphericalAngle>) -> bool {
    // TODO For now, brute force
    for i in 0..(angles.len() - 1) {
        for j in (i + 1)..angles.len() {
            // We want to check if the sign flips across the pair in both the theta and phi
            let angle1 = &angles[i];
            let angle2 = &angles[j];

            // Sign should be flipped, so check for that (the product should be less than 0)
            if angle1.theta * angle2.theta <= 0.0 && angle1.phi * angle2.phi <= 0.0 {
                return true;
            }
        }
    }

    false
}
