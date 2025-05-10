use crate::math::*;
use crate::float_precision::*;
use crate::shapes::Circle;
use crate::shapes::Cylinder;
use crate::shapes::LineSegment3D;
use crate::shapes::Polygon;
use crate::shapes::RectangularPrism;
use crate::shapes::Sphere;
use crate::world_gen::GroundMesh;
use raylib::prelude::*;
use std::fmt::Debug;

#[derive(Debug, Clone)]
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

enum Cases2D {
    CircleCircle(Circle, Circle),
    CircleOther(Circle, SATAble2D),
    OtherCircle(SATAble2D, Circle),
    OtherOther(SATAble2D, SATAble2D),
}

pub fn collision_detection_2d(obj1: SATAble2D, obj2: SATAble2D, coplanar_normal: Vector3f64) -> Option<Vector3f64> {
    // We will set these to the correct shapes to consider fewer cases
    let case = match (obj1.clone(), obj2.clone()) {
        (SATAble2D::Circle(circle1), SATAble2D::Circle(circle2)) => Cases2D::CircleCircle(circle1, circle2),
        (SATAble2D::Circle(circle1), _) => Cases2D::CircleOther(circle1, obj2),
        (_, SATAble2D::Circle(circle2)) => Cases2D::OtherCircle(obj1, circle2),
        (_, _) => Cases2D::OtherOther(obj1, obj2),
    };

    match case {
        Cases2D::CircleCircle(circle1, circle2) => {
            let distance = (circle1.center - circle2.center).length();
            if distance > circle1.radius + circle2.radius {
                return None;
            }

            let overlap = circle1.radius + circle2.radius - distance;
            let direction = (circle2.center - circle1.center).normalized();
            return Some(direction * overlap);
        }
        Cases2D::CircleOther(circle, other) => {
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

            let direction = min_displacement.normalized();
            return Some(direction * overlap);
        }
        Cases2D::OtherCircle(other, circle) => {
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

            let direction = min_displacement.normalized();
            return Some(direction * overlap);
        }
        Cases2D::OtherOther(other1, other2) => {
            let axes1 = other1.compute_orthogonal_axes(&coplanar_normal);
            let axes2 = other2.compute_orthogonal_axes(&coplanar_normal);
            let projection_axes = [axes1, axes2].concat();

            let mut min_displacement = Vector3f64::new(f64::MAX, f64::MAX, f64::MAX);
            for axis in &projection_axes {
                let line_segment1: LineSegment3D = axis.project_satable_object(&other1.clone());
                let line_segment2: LineSegment3D = axis.project_satable_object(&other2.clone());

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
                        if overlap < min_displacement.length() {
                            min_displacement = axis.v.normalized() * overlap;
                        }
                    }
                    None => {
                        return None;
                    }
                }
            }

            Some(min_displacement)
        }
    }

}
