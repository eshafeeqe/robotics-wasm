use nalgebra::Matrix4;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Point3D { x, y, z }
    }

    pub fn origin() -> Self {
        Point3D { x: 0.0, y: 0.0, z: 0.0 }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Transform3D {
    // 4x4 homogeneous transformation matrix
    matrix: Matrix4<f64>,
}

impl Transform3D {
    pub fn identity() -> Self {
        Transform3D {
            matrix: Matrix4::identity(),
        }
    }

    pub fn rotation_x(angle: f64) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        #[rustfmt::skip]
        let matrix = Matrix4::new(
            1.0,   0.0,    0.0,   0.0,
            0.0, cos_a, -sin_a,   0.0,
            0.0, sin_a,  cos_a,   0.0,
            0.0,   0.0,    0.0,   1.0,
        );

        Transform3D { matrix }
    }

    pub fn rotation_y(angle: f64) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        #[rustfmt::skip]
        let matrix = Matrix4::new(
             cos_a, 0.0, sin_a, 0.0,
               0.0, 1.0,   0.0, 0.0,
            -sin_a, 0.0, cos_a, 0.0,
               0.0, 0.0,   0.0, 1.0,
        );

        Transform3D { matrix }
    }

    pub fn rotation_z(angle: f64) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        #[rustfmt::skip]
        let matrix = Matrix4::new(
            cos_a, -sin_a, 0.0, 0.0,
            sin_a,  cos_a, 0.0, 0.0,
              0.0,    0.0, 1.0, 0.0,
              0.0,    0.0, 0.0, 1.0,
        );

        Transform3D { matrix }
    }

    pub fn translation(x: f64, y: f64, z: f64) -> Self {
        #[rustfmt::skip]
        let matrix = Matrix4::new(
            1.0, 0.0, 0.0, x,
            0.0, 1.0, 0.0, y,
            0.0, 0.0, 1.0, z,
            0.0, 0.0, 0.0, 1.0,
        );

        Transform3D { matrix }
    }

    pub fn compose(&self, other: &Transform3D) -> Self {
        Transform3D {
            matrix: self.matrix * other.matrix,
        }
    }

    pub fn transform_point(&self, point: &Point3D) -> Point3D {
        // Apply transformation to homogeneous coordinates [x, y, z, 1]
        let homogeneous = self.matrix * nalgebra::Vector4::new(point.x, point.y, point.z, 1.0);

        Point3D {
            x: homogeneous.x,
            y: homogeneous.y,
            z: homogeneous.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < EPSILON
    }

    fn point_approx_eq(a: &Point3D, b: &Point3D) -> bool {
        approx_eq(a.x, b.x) && approx_eq(a.y, b.y) && approx_eq(a.z, b.z)
    }

    #[test]
    fn test_identity_transform() {
        let identity = Transform3D::identity();
        let point = Point3D::new(3.0, 4.0, 5.0);
        let transformed = identity.transform_point(&point);

        assert!(point_approx_eq(&transformed, &point));
    }

    #[test]
    fn test_translation() {
        let trans = Transform3D::translation(2.0, 3.0, 4.0);
        let point = Point3D::new(1.0, 1.0, 1.0);
        let transformed = trans.transform_point(&point);

        assert!(point_approx_eq(&transformed, &Point3D::new(3.0, 4.0, 5.0)));
    }

    #[test]
    fn test_rotation_z_90_degrees() {
        let rot = Transform3D::rotation_z(PI / 2.0);
        let point = Point3D::new(1.0, 0.0, 0.0);
        let transformed = rot.transform_point(&point);

        // Rotating (1, 0, 0) by 90° around Z should give (0, 1, 0)
        assert!(point_approx_eq(&transformed, &Point3D::new(0.0, 1.0, 0.0)));
    }

    #[test]
    fn test_rotation_z_180_degrees() {
        let rot = Transform3D::rotation_z(PI);
        let point = Point3D::new(1.0, 0.0, 0.0);
        let transformed = rot.transform_point(&point);

        // Rotating (1, 0, 0) by 180° around Z should give (-1, 0, 0)
        assert!(point_approx_eq(&transformed, &Point3D::new(-1.0, 0.0, 0.0)));
    }

    #[test]
    fn test_rotation_x_90_degrees() {
        let rot = Transform3D::rotation_x(PI / 2.0);
        let point = Point3D::new(0.0, 1.0, 0.0);
        let transformed = rot.transform_point(&point);

        // Rotating (0, 1, 0) by 90° around X should give (0, 0, 1)
        assert!(point_approx_eq(&transformed, &Point3D::new(0.0, 0.0, 1.0)));
    }

    #[test]
    fn test_rotation_y_90_degrees() {
        let rot = Transform3D::rotation_y(PI / 2.0);
        let point = Point3D::new(1.0, 0.0, 0.0);
        let transformed = rot.transform_point(&point);

        // Rotating (1, 0, 0) by 90° around Y should give (0, 0, -1)
        assert!(point_approx_eq(&transformed, &Point3D::new(0.0, 0.0, -1.0)));
    }

    #[test]
    fn test_compose_rotation_then_translation() {
        // First rotate 90° around Z, then translate by (2, 0, 0)
        let rot = Transform3D::rotation_z(PI / 2.0);
        let trans = Transform3D::translation(2.0, 0.0, 0.0);
        let combined = trans.compose(&rot);

        let point = Point3D::new(1.0, 0.0, 0.0);
        let transformed = combined.transform_point(&point);

        // (1, 0, 0) rotated 90° around Z = (0, 1, 0), then translated by (2, 0, 0) = (2, 1, 0)
        assert!(point_approx_eq(&transformed, &Point3D::new(2.0, 1.0, 0.0)));
    }

    #[test]
    fn test_identity_composition() {
        let identity = Transform3D::identity();
        let trans = Transform3D::translation(3.0, 4.0, 5.0);

        let composed1 = identity.compose(&trans);
        let composed2 = trans.compose(&identity);

        let point = Point3D::new(1.0, 1.0, 1.0);
        let result1 = composed1.transform_point(&point);
        let result2 = composed2.transform_point(&point);
        let expected = trans.transform_point(&point);

        assert!(point_approx_eq(&result1, &expected));
        assert!(point_approx_eq(&result2, &expected));
    }
}
