use crate::math_util::*;
use crate::float_precision::*;
use raylib::prelude::*;
use std::fmt::Debug;

#[derive(Debug)]
pub struct LineSegment3D {
    pub point1: Vector3f64,
    pub point2: Vector3f64,
}

impl SATAble2D for LineSegment3D {
    fn compute_orthogonal_axes(&self, coplanar_normal: &Vector3f64) -> Vec<Line3D> {
        let axis_vec = coplanar_normal.cross(self.point2 - self.point1);
        let axis = Line3D::from_point_and_parallel_vec(self.point1, axis_vec);
        vec![axis]
    }

    fn get_vertices(&self) -> Vec<Vector3f64> {
        vec![self.point1, self.point2] 
    }
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
    pub fn calculate_overlap(
        &self,
        other: &LineSegment3D,
    ) -> Option<f64> {
        let base_line = Line3D::from_line_segment(self);

        // I don't think this is right?
        let t1 = base_line.find_t_from_point(other.point1);
        let t2 = base_line.find_t_from_point(other.point2);
        if t1 < 0.0 && t2 < 0.0 || t1 > 1.0 && t2 > 1.0 {
            return None;
        }
        if !(0.0..=1.0).contains(&t1) {
            Some(f64_min(&[1.0 - t2, t2]) * base_line.v.length())
        }
        else if !(0.0..=1.0).contains(&t2) {
            Some(f64_min(&[1.0 - t1, t1]) * base_line.v.length())
        }
        else if t1 < t2 {
            Some(f64_min(&[t2, 1.0 - t1]) * base_line.v.length())
        }
        else {
            Some(f64_min(&[t1, 1.0 - t2]) * base_line.v.length())
        }
    }
}

pub trait SATAble2D: Debug {
    fn compute_orthogonal_axes(&self, coplanar_normal: &Vector3f64) -> Vec<Line3D>;
    fn get_vertices(&self) -> Vec<Vector3f64>;
}

pub fn collision_detection_2d(obj1: Box<dyn SATAble2D>, obj2: Box<dyn SATAble2D>, coplanar_normal: Vector3f64) -> Option<(f64, Vector3f64)> {
    let obj1_axes = obj1.compute_orthogonal_axes(&coplanar_normal);
    let obj2_axes = obj2.compute_orthogonal_axes(&coplanar_normal);
    let projection_axes: Vec<Line3D> = [obj1_axes.as_slice(), obj2_axes.as_slice()].concat();
    
    let mut res: (f64, Vector3f64) = (f64::MAX, Vector3f64::new(0.0, 0.0, 0.0,));
    for axis in projection_axes {
        let line_segment1: LineSegment3D = axis.project_satable_object(&obj1);
        let line_segment2: LineSegment3D = axis.project_satable_object(&obj2);

        let rounded_segment1 = LineSegment3D::new(vector3_round(line_segment1.point1), vector3_round(line_segment1.point2));
        let rounded_segment2 = LineSegment3D::new(vector3_round(line_segment2.point1), vector3_round(line_segment2.point2));
        let overlap: Option<f64> =
        if rounded_segment1.length() > rounded_segment2.length() {
            rounded_segment1.calculate_overlap(&rounded_segment2)
        }
        else {
            rounded_segment2.calculate_overlap(&rounded_segment1)
        };
        match overlap {
            Some(overlap) => {
                if overlap < res.0 {
                    res.0 = overlap;
                    res.1 = axis.v.normalized();
                }
            }
            None => {
                return None;
            }
        }
    }

    Some(res)
}

// This is only really a useful abstraction for rendering or calculating collisions #[derive(Debug)]
#[derive(Debug)]
pub struct Polygon {
    // These points are absolute
    pub points: Vec<Vector3f64>,
}

impl SATAble2D for Polygon {
    fn compute_orthogonal_axes(&self, coplanar_normal: &Vector3f64) -> Vec<Line3D> {
        let edges = self.get_edges();
        edges.iter().map(|edge| {
            Line3D::from_point_and_parallel_vec(edge.point1, coplanar_normal.cross(edge.point2 - edge.point1))
        }).collect()
    }

    fn get_vertices(&self) -> Vec<Vector3f64> {
        self.points.clone()       
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
    Calculates a single possible orthogonal plane that can be used for the separating axis theorem (this is much more optimal, but I'm not quite sure if it's actually correct for anything other than cubes)
     */
    pub fn calculate_orthogonal_plane(&self) -> Plane {
        let polygon_as_plane = self.convert_to_plane();
        let edges = self.get_edges();
        let edge_vec = edges[0].point2 - edges[0].point1;
        let new_normal = polygon_as_plane.n.cross(edge_vec).normalized();
        Plane::from_point_and_normal(edges[0].point1, new_normal)
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

    // While it isn't precise for extremely fast objects, this basically computes the closest way we could move the object to remove the overlap, and considering this is written in Rust I think this is okay
    /**
    Returns `None` if the shapes don't overlap
    Otherwise returns the minimum overlap between shapes and the axis on which that minimum overlap was found
    */
    fn separating_axis_theorem(
        &self,
        other_obj: &Polygon,
    ) -> Option<(f64, Vector3f64)> {
        let plane = self.convert_to_plane();
        let mut overlap = f64::MAX;
        let mut axis: Option<Vector3f64> = None;
        for i in 0..self.points.len() {
            let line_segment = &self.get_edges()[i];
            let u = line_segment.point2 - line_segment.point1;
            if u == Vector3f64::new(0.0, 0.0, 0.0,) {
                continue;
            }
            let normal = u.cross(plane.n);
            let projection_line = Line3D::from_point_and_parallel_vec(line_segment.point1, normal);
            let line_segment1 = projection_line.project_polygon(self);
            let line_segment2 = projection_line.project_polygon(other_obj);

            let rounded_segment1 = LineSegment3D::new(vector3_round(line_segment1.point1), vector3_round(line_segment1.point2));
            let rounded_segment2 = LineSegment3D::new(vector3_round(line_segment2.point1), vector3_round(line_segment2.point2));
            let o = rounded_segment1.calculate_overlap(&rounded_segment2);
            match o {
                Some(o) => {
                    if axis.is_none() || o < overlap {
                        overlap = o;
                        axis = Some(projection_line.v);
                    }
                }
                None => return None,
            }
        }

        Some((overlap, axis.unwrap()))
    }

    pub fn collides_with(&self, other: &Polygon) -> Option<(f64, Vector3f64)> {
        let mut min_axis: (f64, Vector3f64);

        let sat1 = self.separating_axis_theorem(other);
        match &sat1 {
            Some(axis) => min_axis = *axis,
            None => return None,
        }

        let sat2 = other.separating_axis_theorem(self);
        match sat2 {
            Some(axis) => {
                if axis.0 < min_axis.0 {
                    min_axis = axis;
                }
            }
            None => return None,
        }

        Some(min_axis)
    }
}

pub trait MeshShape: Debug {
    fn move_by(&mut self, change: Vector3f64);
    fn get_vertices(&self) -> Vec<Vector3f64>;
    fn get_polygons(&self) -> Vec<Polygon>;
    fn get_center(&self) -> Vector3f64;
    fn get_bounding_circle_radius(&self) -> f64;
    fn render(&self, draw_handle: &mut RaylibMode3D<'_, RaylibDrawHandle<'_>>, in_debug: bool);
}

#[derive(Debug)]
pub struct RectangularPrism {
    // The root is an absolute point and is the bottom left point of the front face
    pub root: Vector3f64,

    pub length: f64,
    pub width: f64,
    pub height: f64,

    bounding_circle_radius: f64,
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
}

impl MeshShape for RectangularPrism {
    fn move_by(&mut self, change: Vector3f64) {
        self.root += change;
    }

    fn get_vertices(&self) -> Vec<Vector3f64> {
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

    fn get_polygons(&self) -> Vec<Polygon> {
        let vertices = self.get_vertices();

        // Define all of the polygons
        vec![
            Polygon::new(vec![vertices[0], vertices[1], vertices[3], vertices[2]]),
            Polygon::new(vec![vertices[4], vertices[5], vertices[7], vertices[6]]),
            Polygon::new(vec![vertices[0], vertices[1], vertices[5], vertices[4]]),
            Polygon::new(vec![vertices[2], vertices[3], vertices[7], vertices[6]]),
            Polygon::new(vec![vertices[0], vertices[2], vertices[6], vertices[4]]),
            Polygon::new(vec![vertices[1], vertices[3], vertices[7], vertices[5]]),
        ]
    }

    fn get_center(&self) -> Vector3f64 {
        Vector3f64::new(
            self.root.x + self.width / 2.0,
            self.root.y + self.height / 2.0,
            self.root.z + self.length / 2.0,
        )
    }

    fn get_bounding_circle_radius(&self) -> f64 {
        self.bounding_circle_radius
    }

    fn render(&self, draw_mode: &mut RaylibMode3D<'_, RaylibDrawHandle<'_>>, in_debug: bool) {
        let polygons = self.get_polygons();

        let mut color_index = 0;
        let colors = [
            Color::RED,
        ];
        for polygon in &polygons {
            let polygon_points = &polygon.points;

            // TODO: For some reason this works, but I think it shouldn't...
            for i in 1..polygon_points.len() - 1 {
                // Draw the front side
                draw_mode.draw_triangle3D(
                    Vector3::from(polygon_points[0]),
                    Vector3::from(polygon_points[i]),
                    Vector3::from(polygon_points[i + 1]),
                    colors[color_index],
                );

                // Draw the back side
                draw_mode.draw_triangle3D(
                    Vector3::from(polygon_points[i + 1]),
                    Vector3::from(polygon_points[i]),
                    Vector3::from(polygon_points[0]),
                    colors[color_index],
                );
            }

            color_index += 1;
            color_index %= colors.len();
        }

        if in_debug {
            draw_wireframe(draw_mode, polygons);
        }
    }
}

pub fn draw_wireframe(d: &mut RaylibMode3D<'_, RaylibDrawHandle<'_>>, polygons: Vec<Polygon>) {
    for polygon in polygons {
        let points = polygon.points;
        for i in 0..points.len() {
            d.draw_line_3D(Vector3::from(points[0]), Vector3::from(points[i]), Color::BLACK);
        }
    }
}
