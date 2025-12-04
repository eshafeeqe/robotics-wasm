use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    pub fn new(x: f64, y: f64) -> Self {
        Point2D { x, y }
    }

    pub fn origin() -> Self {
        Point2D { x: 0.0, y: 0.0 }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Transform2D {
    // 3x3 homogeneous transformation matrix stored in row-major order
    // [m00 m01 m02]   [cos(θ) -sin(θ)  tx]
    // [m10 m11 m12] = [sin(θ)  cos(θ)  ty]
    // [m20 m21 m22]   [0       0       1 ]
    data: [f64; 9],
}

impl Transform2D {
    pub fn identity() -> Self {
        Transform2D {
            data: [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ],
        }
    }

    pub fn rotation(angle: f64) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        Transform2D {
            data: [
                cos_a, -sin_a, 0.0,
                sin_a,  cos_a, 0.0,
                0.0,    0.0,   1.0,
            ],
        }
    }

    pub fn translation(x: f64, y: f64) -> Self {
        Transform2D {
            data: [
                1.0, 0.0, x,
                0.0, 1.0, y,
                0.0, 0.0, 1.0,
            ],
        }
    }

    // Matrix composition: self * other
    // Used to chain transformations
    pub fn compose(&self, other: &Transform2D) -> Self {
        let a = &self.data;
        let b = &other.data;

        Transform2D {
            data: [
                // Row 0
                a[0] * b[0] + a[1] * b[3] + a[2] * b[6],
                a[0] * b[1] + a[1] * b[4] + a[2] * b[7],
                a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
                // Row 1
                a[3] * b[0] + a[4] * b[3] + a[5] * b[6],
                a[3] * b[1] + a[4] * b[4] + a[5] * b[7],
                a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
                // Row 2
                a[6] * b[0] + a[7] * b[3] + a[8] * b[6],
                a[6] * b[1] + a[7] * b[4] + a[8] * b[7],
                a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
            ],
        }
    }

    pub fn transform_point(&self, point: &Point2D) -> Point2D {
        let m = &self.data;

        // Apply homogeneous transformation
        // [x']   [m00 m01 m02]   [x]
        // [y'] = [m10 m11 m12] * [y]
        // [1 ]   [m20 m21 m22]   [1]

        Point2D {
            x: m[0] * point.x + m[1] * point.y + m[2],
            y: m[3] * point.x + m[4] * point.y + m[5],
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

    fn point_approx_eq(a: &Point2D, b: &Point2D) -> bool {
        approx_eq(a.x, b.x) && approx_eq(a.y, b.y)
    }

    #[test]
    fn test_identity_transform() {
        let identity = Transform2D::identity();
        let point = Point2D::new(3.0, 4.0);
        let transformed = identity.transform_point(&point);

        assert!(point_approx_eq(&transformed, &point));
    }

    #[test]
    fn test_translation() {
        let trans = Transform2D::translation(2.0, 3.0);
        let point = Point2D::new(1.0, 1.0);
        let transformed = trans.transform_point(&point);

        assert!(point_approx_eq(&transformed, &Point2D::new(3.0, 4.0)));
    }

    #[test]
    fn test_rotation_90_degrees() {
        let rot = Transform2D::rotation(PI / 2.0);
        let point = Point2D::new(1.0, 0.0);
        let transformed = rot.transform_point(&point);

        // Rotating (1, 0) by 90° should give (0, 1)
        assert!(point_approx_eq(&transformed, &Point2D::new(0.0, 1.0)));
    }

    #[test]
    fn test_rotation_180_degrees() {
        let rot = Transform2D::rotation(PI);
        let point = Point2D::new(1.0, 0.0);
        let transformed = rot.transform_point(&point);

        // Rotating (1, 0) by 180° should give (-1, 0)
        assert!(point_approx_eq(&transformed, &Point2D::new(-1.0, 0.0)));
    }

    #[test]
    fn test_compose_rotation_then_translation() {
        // First rotate 90°, then translate by (2, 0)
        let rot = Transform2D::rotation(PI / 2.0);
        let trans = Transform2D::translation(2.0, 0.0);
        let combined = trans.compose(&rot);

        let point = Point2D::new(1.0, 0.0);
        let transformed = combined.transform_point(&point);

        // (1, 0) rotated 90° = (0, 1), then translated by (2, 0) = (2, 1)
        assert!(point_approx_eq(&transformed, &Point2D::new(2.0, 1.0)));
    }

    #[test]
    fn test_compose_translation_then_rotation() {
        // First translate by (1, 0), then rotate 90°
        let trans = Transform2D::translation(1.0, 0.0);
        let rot = Transform2D::rotation(PI / 2.0);
        let combined = rot.compose(&trans);

        let point = Point2D::new(1.0, 0.0);
        let transformed = combined.transform_point(&point);

        // (1, 0) translated by (1, 0) = (2, 0), then rotated 90° = (0, 2)
        assert!(point_approx_eq(&transformed, &Point2D::new(0.0, 2.0)));
    }

    #[test]
    fn test_identity_composition() {
        let identity = Transform2D::identity();
        let trans = Transform2D::translation(3.0, 4.0);

        let composed1 = identity.compose(&trans);
        let composed2 = trans.compose(&identity);

        let point = Point2D::new(1.0, 1.0);
        let result1 = composed1.transform_point(&point);
        let result2 = composed2.transform_point(&point);
        let expected = trans.transform_point(&point);

        assert!(point_approx_eq(&result1, &expected));
        assert!(point_approx_eq(&result2, &expected));
    }
}
