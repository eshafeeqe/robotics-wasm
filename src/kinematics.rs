use crate::geometry3d::{Point3D, Transform3D};
use crate::robot::RobotArm;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct JointPosition3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl JointPosition3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        JointPosition3D { x, y, z }
    }

    pub fn from_point(point: &Point3D) -> Self {
        JointPosition3D {
            x: point.x,
            y: point.y,
            z: point.z,
        }
    }
}

/// Computes forward kinematics for a planar robot arm using 3D transforms
///
/// Returns positions of all joints including base and end-effector
/// For a 2-DOF robot: [base, joint1, joint2, end-effector] (4 positions)
/// Note: Planar robots have Z=0 for all positions
pub fn forward_kinematics(robot: &RobotArm) -> Vec<JointPosition3D> {
    let mut positions = Vec::new();
    let mut current_transform = Transform3D::identity();

    // Add base position at origin
    let base = current_transform.transform_point(&Point3D::origin());
    positions.push(JointPosition3D::from_point(&base));

    // For each link, compute the joint position using 3D transforms
    // Planar robots rotate around Z-axis and translate along X-axis (in the XY plane)
    for i in 0..robot.num_joints() {
        // Rotate around Z-axis by joint angle
        let rotation = Transform3D::rotation_z(robot.joint_angles[i]);
        current_transform = current_transform.compose(&rotation);

        // Translate along X-axis by link length (Z component is 0)
        let translation = Transform3D::translation(robot.link_lengths[i], 0.0, 0.0);
        current_transform = current_transform.compose(&translation);

        // Record position after this link
        let joint_pos = current_transform.transform_point(&Point3D::origin());
        positions.push(JointPosition3D::from_point(&joint_pos));
    }

    positions
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < EPSILON
    }

    #[test]
    fn test_fk_zero_angles() {
        // Robot with two links of length 2.0 and 1.5
        // Both angles at 0 - should extend horizontally along X-axis
        let mut robot = RobotArm::new(vec![2.0, 1.5]);
        robot.set_joint_angles(vec![0.0, 0.0]);

        let positions = forward_kinematics(&robot);

        assert_eq!(positions.len(), 3); // base, joint1, end-effector

        // Base at origin
        assert!(approx_eq(positions[0].x, 0.0));
        assert!(approx_eq(positions[0].y, 0.0));
        assert!(approx_eq(positions[0].z, 0.0));

        // Joint 1 at (2.0, 0.0, 0.0)
        assert!(approx_eq(positions[1].x, 2.0));
        assert!(approx_eq(positions[1].y, 0.0));
        assert!(approx_eq(positions[1].z, 0.0));

        // End-effector at (3.5, 0.0, 0.0)
        assert!(approx_eq(positions[2].x, 3.5));
        assert!(approx_eq(positions[2].y, 0.0));
        assert!(approx_eq(positions[2].z, 0.0));
    }

    #[test]
    fn test_fk_90_degree_first_joint() {
        // First joint rotated 90°, second at 0°
        let mut robot = RobotArm::new(vec![2.0, 1.5]);
        robot.set_joint_angles(vec![PI / 2.0, 0.0]);

        let positions = forward_kinematics(&robot);

        // Base at origin
        assert!(approx_eq(positions[0].x, 0.0));
        assert!(approx_eq(positions[0].y, 0.0));
        assert!(approx_eq(positions[0].z, 0.0));

        // Joint 1 should be at (0, 2.0, 0) after rotating 90°
        assert!(approx_eq(positions[1].x, 0.0));
        assert!(approx_eq(positions[1].y, 2.0));
        assert!(approx_eq(positions[1].z, 0.0));

        // End-effector should be at (0, 3.5, 0) - extending upward
        assert!(approx_eq(positions[2].x, 0.0));
        assert!(approx_eq(positions[2].y, 3.5));
        assert!(approx_eq(positions[2].z, 0.0));
    }

    #[test]
    fn test_fk_both_joints_90_degrees() {
        // Both joints at 90°
        let mut robot = RobotArm::new(vec![2.0, 1.5]);
        robot.set_joint_angles(vec![PI / 2.0, PI / 2.0]);

        let positions = forward_kinematics(&robot);

        // Base at origin
        assert!(approx_eq(positions[0].x, 0.0));
        assert!(approx_eq(positions[0].y, 0.0));
        assert!(approx_eq(positions[0].z, 0.0));

        // Joint 1 at (0, 2.0, 0)
        assert!(approx_eq(positions[1].x, 0.0));
        assert!(approx_eq(positions[1].y, 2.0));
        assert!(approx_eq(positions[1].z, 0.0));

        // End-effector should bend back to (-1.5, 2.0, 0)
        assert!(approx_eq(positions[2].x, -1.5));
        assert!(approx_eq(positions[2].y, 2.0));
        assert!(approx_eq(positions[2].z, 0.0));
    }

    #[test]
    fn test_fk_folded_configuration() {
        // First joint at 0°, second at -180° (folded back)
        let mut robot = RobotArm::new(vec![2.0, 1.5]);
        robot.set_joint_angles(vec![0.0, PI]);

        let positions = forward_kinematics(&robot);

        // Base at origin
        assert!(approx_eq(positions[0].x, 0.0));
        assert!(approx_eq(positions[0].y, 0.0));
        assert!(approx_eq(positions[0].z, 0.0));

        // Joint 1 at (2.0, 0.0, 0)
        assert!(approx_eq(positions[1].x, 2.0));
        assert!(approx_eq(positions[1].y, 0.0));
        assert!(approx_eq(positions[1].z, 0.0));

        // End-effector should be at (0.5, 0.0, 0) - folded back
        assert!(approx_eq(positions[2].x, 0.5));
        assert!(approx_eq(positions[2].y, 0.0));
        assert!(approx_eq(positions[2].z, 0.0));
    }
}
