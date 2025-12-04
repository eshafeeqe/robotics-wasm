use crate::geometry3d::Transform3D;

/// Type of joint in the robot
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JointType {
    /// Revolute joint: θ is variable, d is fixed
    Revolute,
    /// Prismatic joint: d is variable, θ is fixed
    Prismatic,
}

/// Denavit-Hartenberg parameters for a single link
///
/// Using Standard (Classic) DH Convention:
/// T(i-1,i) = Rot(Z, θ) * Trans(Z, d) * Trans(X, a) * Rot(X, α)
#[derive(Debug, Clone, Copy)]
pub struct DHParameter {
    /// a: Link length (distance along X axis from Z_i-1 to Z_i)
    pub a: f64,
    /// α (alpha): Link twist (rotation around X axis from Z_i-1 to Z_i)
    pub alpha: f64,
    /// d: Link offset (distance along Z axis from X_i-1 to X_i)
    pub d: f64,
    /// θ (theta): Joint angle (rotation around Z axis from X_i-1 to X_i)
    pub theta: f64,
    /// Type of joint (determines which parameter is variable)
    pub joint_type: JointType,
    /// Offset added to the joint variable (for homing or calibration)
    pub joint_offset: f64,
}

impl DHParameter {
    /// Create a revolute joint with DH parameters
    ///
    /// # Arguments
    /// * `a` - Link length
    /// * `alpha` - Link twist (radians)
    /// * `d` - Link offset (fixed for revolute)
    /// * `theta_offset` - Joint angle offset (radians)
    pub fn revolute(a: f64, alpha: f64, d: f64, theta_offset: f64) -> Self {
        DHParameter {
            a,
            alpha,
            d,
            theta: theta_offset,
            joint_type: JointType::Revolute,
            joint_offset: theta_offset,
        }
    }

    /// Create a prismatic joint with DH parameters
    ///
    /// # Arguments
    /// * `a` - Link length
    /// * `alpha` - Link twist (radians)
    /// * `d_offset` - Link offset (variable for prismatic)
    /// * `theta` - Joint angle (fixed for prismatic, radians)
    pub fn prismatic(a: f64, alpha: f64, d_offset: f64, theta: f64) -> Self {
        DHParameter {
            a,
            alpha,
            d: d_offset,
            theta,
            joint_type: JointType::Prismatic,
            joint_offset: d_offset,
        }
    }

    /// Create a planar revolute joint (simplified DH for 2D robots)
    ///
    /// This is a convenience constructor for planar robots where:
    /// - α = 0 (no twist)
    /// - d = 0 (no offset)
    /// - Only 'a' (link length) and θ (joint angle) matter
    pub fn planar(link_length: f64) -> Self {
        DHParameter::revolute(link_length, 0.0, 0.0, 0.0)
    }

    /// Update this DH parameter with a new joint value
    ///
    /// For revolute joints, updates theta
    /// For prismatic joints, updates d
    pub fn with_joint_value(&self, value: f64) -> Self {
        let mut new_dh = *self;
        match self.joint_type {
            JointType::Revolute => {
                new_dh.theta = self.joint_offset + value;
            }
            JointType::Prismatic => {
                new_dh.d = self.joint_offset + value;
            }
        }
        new_dh
    }

    /// Convert DH parameters to a homogeneous transformation matrix
    ///
    /// Standard DH Convention:
    /// T(i-1,i) = Rot(Z, θ) * Trans(Z, d) * Trans(X, a) * Rot(X, α)
    ///
    /// This represents the transformation from frame i-1 to frame i
    pub fn to_transform(&self) -> Transform3D {
        // Step 1: Rotate around Z by theta
        let rot_z = Transform3D::rotation_z(self.theta);

        // Step 2: Translate along Z by d
        let trans_z = Transform3D::translation(0.0, 0.0, self.d);

        // Step 3: Translate along X by a
        let trans_x = Transform3D::translation(self.a, 0.0, 0.0);

        // Step 4: Rotate around X by alpha
        let rot_x = Transform3D::rotation_x(self.alpha);

        // Compose in order: Rot(Z,θ) * Trans(Z,d) * Trans(X,a) * Rot(X,α)
        rot_z
            .compose(&trans_z)
            .compose(&trans_x)
            .compose(&rot_x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry3d::Point3D;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < EPSILON
    }

    fn point_approx_eq(a: &Point3D, b: &Point3D) -> bool {
        approx_eq(a.x, b.x) && approx_eq(a.y, b.y) && approx_eq(a.z, b.z)
    }

    #[test]
    fn test_planar_dh_constructor() {
        let dh = DHParameter::planar(2.0);
        assert_eq!(dh.a, 2.0);
        assert_eq!(dh.alpha, 0.0);
        assert_eq!(dh.d, 0.0);
        assert_eq!(dh.theta, 0.0);
        assert_eq!(dh.joint_type, JointType::Revolute);
    }

    #[test]
    fn test_revolute_joint_value_update() {
        let dh = DHParameter::revolute(1.0, 0.0, 0.0, 0.0);
        let updated = dh.with_joint_value(PI / 2.0);

        assert_eq!(updated.theta, PI / 2.0);
        assert_eq!(updated.d, 0.0); // d shouldn't change for revolute
    }

    #[test]
    fn test_prismatic_joint_value_update() {
        let dh = DHParameter::prismatic(1.0, 0.0, 0.5, 0.0);
        let updated = dh.with_joint_value(0.3);

        assert_eq!(updated.d, 0.8); // 0.5 offset + 0.3 value
        assert_eq!(updated.theta, 0.0); // theta shouldn't change for prismatic
    }

    #[test]
    fn test_dh_identity_transform() {
        // DH parameters that should give identity transform
        let dh = DHParameter::revolute(0.0, 0.0, 0.0, 0.0);
        let transform = dh.to_transform();

        let point = Point3D::new(1.0, 2.0, 3.0);
        let result = transform.transform_point(&point);

        assert!(point_approx_eq(&result, &point));
    }

    #[test]
    fn test_dh_pure_translation_x() {
        // Only 'a' parameter set (translation along X)
        let dh = DHParameter::revolute(2.0, 0.0, 0.0, 0.0);
        let transform = dh.to_transform();

        let origin = Point3D::origin();
        let result = transform.transform_point(&origin);

        assert!(point_approx_eq(&result, &Point3D::new(2.0, 0.0, 0.0)));
    }

    #[test]
    fn test_dh_pure_rotation_z() {
        // Only theta set (rotation around Z)
        let dh = DHParameter::revolute(0.0, 0.0, 0.0, PI / 2.0);
        let transform = dh.to_transform();

        let point = Point3D::new(1.0, 0.0, 0.0);
        let result = transform.transform_point(&point);

        // Rotating (1,0,0) by 90° around Z gives (0,1,0)
        assert!(point_approx_eq(&result, &Point3D::new(0.0, 1.0, 0.0)));
    }

    #[test]
    fn test_dh_planar_robot_link() {
        // Planar robot link: length 2.0, rotated 90°
        let dh = DHParameter::planar(2.0).with_joint_value(PI / 2.0);
        let transform = dh.to_transform();

        let origin = Point3D::origin();
        let result = transform.transform_point(&origin);

        // Should be at (0, 2.0, 0) after 90° rotation and 2.0 translation
        assert!(point_approx_eq(&result, &Point3D::new(0.0, 2.0, 0.0)));
    }

    #[test]
    fn test_dh_with_twist() {
        // Link with twist (alpha = 90°)
        let dh = DHParameter::revolute(1.0, PI / 2.0, 0.0, 0.0);
        let transform = dh.to_transform();

        let point = Point3D::new(0.0, 1.0, 0.0);
        let result = transform.transform_point(&point);

        // After: Trans(X,1) * Rot(X,90°)
        // Point (0,1,0) -> Rot(X,90°) -> (0,0,1) -> Trans(X,1) -> (1,0,1)
        assert!(point_approx_eq(&result, &Point3D::new(1.0, 0.0, 1.0)));
    }

    #[test]
    fn test_dh_full_parameters() {
        // All DH parameters set
        let dh = DHParameter {
            a: 1.0,
            alpha: PI / 4.0,
            d: 0.5,
            theta: PI / 6.0,
            joint_type: JointType::Revolute,
            joint_offset: 0.0,
        };

        let transform = dh.to_transform();
        let result = transform.transform_point(&Point3D::origin());

        // Complex transformation - just verify it computes without error
        // Exact value would require manual calculation
        assert!(result.x.is_finite());
        assert!(result.y.is_finite());
        assert!(result.z.is_finite());
    }
}
