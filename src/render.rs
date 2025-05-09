use crate::math::*;
use crate::float_precision::*;
use crate::shapes::Circle;
use crate::shapes::LineSegment3D;
use crate::shapes::Polygon;
use crate::shapes::RectangularPrism;
use crate::shapes::Sphere;
use crate::world_gen::GroundMesh;
use raylib::prelude::*;
use std::fmt::Debug;

pub enum SATAble2D {
    LineSegment3D(LineSegment3D),
    Polygon(Polygon),
    Circle(Circle),
}

impl SATAble2D {
    pub fn compute_orthogonal_axes(&self, coplanar_normal: &Vector3f64) -> Vec<Line3D> {
        match self {
            SATAble2D::LineSegment3D(line_segment) => {
                let axis_vec = coplanar_normal.cross(line_segment.point2 - line_segment.point1);
                let axis = Line3D::from_point_and_parallel_vec(line_segment.point1, axis_vec);
                vec![axis]
            },
            SATAble2D::Polygon(polygon) => {
                let edges = polygon.get_edges();
                edges.iter().map(|edge| {
                    let axis_vec = coplanar_normal.cross(edge.point2 - edge.point1);
                    Line3D::from_point_and_parallel_vec(edge.point1, axis_vec)
                }).collect()
            },
            SATAble2D::Circle(_) => {
                // A circle has infinite axes so we can just resort to the other shape
                vec![]
            },
        }
    }

    pub fn get_center(&self) -> Vector3f64 {
        match self {
            SATAble2D::LineSegment3D(line_segment) => {
                (line_segment.point1 + line_segment.point2) / 2.0
            },
            SATAble2D::Polygon(polygon) => {
                let mut center = Vector3f64::new(0.0, 0.0, 0.0,);
                for point in polygon.points.iter() {
                    center += *point;
                }
                center / polygon.points.len() as f64
            },
            SATAble2D::Circle(circle) => {
                circle.center
            },
        }
    }

    pub fn get_sides(&self) -> Vec<LineSegment3D> {
        match self {
            SATAble2D::LineSegment3D(line_segment) => {
                vec![LineSegment3D::new(line_segment.point1, line_segment.point2)]
            },
            SATAble2D::Polygon(polygon) => {
                polygon.get_edges().iter().map(|edge| {
                    LineSegment3D::new(edge.point1, edge.point2)
                }).collect()
            },
            SATAble2D::Circle(_) => {
                vec![]
            },
        }
    }
}

pub fn collision_detection_2d(obj1: SATAble2D, obj2: SATAble2D, coplanar_normal: Vector3f64) -> Option<(f64, Vector3f64)> {
    let obj1_axes = obj1.compute_orthogonal_axes(&coplanar_normal);
    let obj2_axes = obj2.compute_orthogonal_axes(&coplanar_normal);
    let projection_axes: Vec<Line3D> = [obj1_axes.as_slice(), obj2_axes.as_slice()].concat();

    let mut circle: Option<&Circle> = None;
    let mut other: Option<&SATAble2D> = None;

    match (&obj1, &obj2) {
        (SATAble2D::Circle(circle1), SATAble2D::Circle(circle2)) => {
            let distance = (circle1.center - circle2.center).length();
            if distance > circle1.radius + circle2.radius {
                return None;
            }

            let overlap = circle1.radius + circle2.radius - distance;
            let direction = (circle2.center - circle1.center).normalized();
            return Some((overlap, direction));
        }
        (SATAble2D::Circle(circle_obj), other_obj) => {
            circle = Some(circle_obj);
            other = Some(other_obj);
        }
        (other_obj, SATAble2D::Circle(circle_obj)) => {
            circle = Some(circle_obj);
            other = Some(other_obj);
        }
        _ => {}
    }

    if let Some(circle) = circle {
        if let Some(other) = other {
            let sides = other.get_sides();
            let mut min_displacement = Vector3f64::new(f64::MAX, f64::MAX, f64::MAX);
            for side in sides {
                let displacement = side.minimum_displacement(circle.center);
                if displacement.length() < min_displacement.length() {
                    min_displacement = displacement;
                }
            }

            let overlap = circle.radius - min_displacement.length();
            if overlap < 0.0 {
                return None;
            }

            let direction = min_displacement.normalized() * -1.0;
            return Some((overlap, direction));
        }
    }

    let mut res: (f64, Vector3f64) = (f64::MAX, Vector3f64::new(0.0, 0.0, 0.0,));
    for axis in &projection_axes {
        let line_segment1: LineSegment3D = axis.project_satable_object(&obj1);
        let line_segment2: LineSegment3D = axis.project_satable_object(&obj2);

        let rounded_segment1 = LineSegment3D::new(vector3_round(line_segment1.point1), vector3_round(line_segment1.point2));
        let rounded_segment2 = LineSegment3D::new(vector3_round(line_segment2.point1), vector3_round(line_segment2.point2));
        let overlap: Option<f64> =
        if rounded_segment1.length() > rounded_segment2.length() {
            rounded_segment1.calculate_min_translation(&rounded_segment2)
        }
        else {
            rounded_segment2.calculate_min_translation(&rounded_segment1)
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

#[derive(Debug, Clone)]
pub enum MeshShape {
    RectangularPrism(RectangularPrism),
    GroundMesh(GroundMesh),
    Sphere(Sphere),
}

impl MeshShape {
    pub fn move_by(&mut self, change: Vector3f64) {
        match self {
            MeshShape::RectangularPrism(prism) => {
                prism.root += change;
            },
            MeshShape::Sphere(sphere) => {
                sphere.center += change;
            },
            MeshShape::GroundMesh(ground_mesh) => {
                ground_mesh.bottom_left_pos += Vector2f64::new(change.x, change.z);
                ground_mesh.h1 += change.y;
                ground_mesh.h2 += change.y;
                ground_mesh.h3 += change.y;
                ground_mesh.h4 += change.y;
            },
        }
    }
    pub fn get_bounding_circle_radius(&self) -> f64 {
        match self {
            MeshShape::RectangularPrism(prism) => {
                prism.bounding_circle_radius
            },
            MeshShape::Sphere(sphere) => {
                sphere.radius
            },
            MeshShape::GroundMesh(ground_mesh) => {
                let average_height = (ground_mesh.h1 + ground_mesh.h2 + ground_mesh.h3 + ground_mesh.h4) / 4.0;
                let max_height = f64_max(&[ground_mesh.h1, ground_mesh.h2, ground_mesh.h3, ground_mesh.h4]);
                (ground_mesh.dx * ground_mesh.dx + ground_mesh.dz * ground_mesh.dz + max_height - average_height).sqrt()
            },
        }
    }

    pub fn get_center(&self) -> Vector3f64 {
        match self {
            MeshShape::RectangularPrism(prism) => {
                Vector3f64::new(
                    prism.root.x + prism.width / 2.0,
                    prism.root.y + prism.height / 2.0,
                    prism.root.z + prism.length / 2.0,
                )
            },
            MeshShape::Sphere(sphere) => {
                sphere.center
            },
            MeshShape::GroundMesh(ground_mesh) => {
                ground_mesh.get_center()
            },
        }
    }

    pub fn get_polygons(&self) -> Vec<Polygon> {
        match self {
            MeshShape::RectangularPrism(prism) => {
                let vertices = prism.get_vertices();

                // Define all of the polygons
                vec![
                    Polygon::new(vec![vertices[0], vertices[1], vertices[3], vertices[2]]),
                    Polygon::new(vec![vertices[4], vertices[5], vertices[7], vertices[6]]),
                    Polygon::new(vec![vertices[0], vertices[1], vertices[5], vertices[4]]),
                    Polygon::new(vec![vertices[2], vertices[3], vertices[7], vertices[6]]),
                    Polygon::new(vec![vertices[0], vertices[2], vertices[6], vertices[4]]),
                    Polygon::new(vec![vertices[1], vertices[3], vertices[7], vertices[5]]),
                ]
            },
            MeshShape::Sphere(_) => {
                // A sphere has infinite polygons, so just return an empty vector
                vec![]
            },
            MeshShape::GroundMesh(ground_mesh) => {
                let vertices = ground_mesh.get_vertices();

                // Because it's not guaranteed that each 4 points will be coplanar, we create two traingles that contain all the points
                let triangle1 = vec![vertices[0], vertices[1], vertices[2]];
                let triangle2 = vec![vertices[2], vertices[3], vertices[0]];
                vec![Polygon::new(triangle1), Polygon::new(triangle2)]
            },
        }
    }

    pub fn render(&self, draw_handle: &mut RaylibMode3D<'_, RaylibDrawHandle<'_>>) {
        match self {
            MeshShape::RectangularPrism(prism) => {
                draw_handle.draw_cube(
                    Vector3::from(self.get_center()),
                    prism.width as f32,
                    prism.height as f32,
                    prism.length as f32,
                    Color::RED,
                );
            },
            MeshShape::Sphere(sphere) => {
                draw_handle.draw_sphere(
                    Vector3::from(sphere.center),
                    sphere.radius as f32,
                    Color::RED,
                );
            },
            MeshShape::GroundMesh(ground_mesh) => {
                let vertices = ground_mesh.get_vertices();

                for i in 1..vertices.len() - 1 {
                    draw_handle.draw_triangle3D(
                        Vector3::from(vertices[0]),
                        Vector3::from(vertices[i]),
                        Vector3::from(vertices[i + 1]),
                        ground_mesh.color,
                    );
                    draw_handle.draw_triangle3D(
                        Vector3::from(vertices[0]),
                        Vector3::from(vertices[i + 1]),
                        Vector3::from(vertices[i]),
                        ground_mesh.color,
                    );
                }
            },
        }
    }
}