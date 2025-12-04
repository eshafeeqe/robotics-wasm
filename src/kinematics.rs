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

/// Computes forward kinematics for a robot arm
///
/// Supports two modes:
/// 1. Simple planar robots (link_lengths + joint_angles)
/// 2. DH parameter-based robots (full 3D specification)
///
/// Returns positions of all joints including base and end-effector
pub fn forward_kinematics(robot: &RobotArm) -> Vec<JointPosition3D> {
    if robot.uses_dh_params() {
        forward_kinematics_dh(robot)
    } else {
        forward_kinematics_planar(robot)
    }
}

/// Forward kinematics using DH parameters
///
/// Chains DH transformations to compute joint positions in 3D space
fn forward_kinematics_dh(robot: &RobotArm) -> Vec<JointPosition3D> {
    let mut positions = Vec::new();
    let mut current_transform = Transform3D::identity();

    // Add base position at origin
    positions.push(JointPosition3D::from_point(&Point3D::origin()));

    // Get DH parameters with current joint values
    if let Some(dh_params) = robot.get_dh_with_current_values() {
        for dh in dh_params {
            // Apply DH transformation for this link
            let link_transform = dh.to_transform();
            current_transform = current_transform.compose(&link_transform);

            // Record joint position
            let joint_pos = current_transform.transform_point(&Point3D::origin());
            positions.push(JointPosition3D::from_point(&joint_pos));
        }
    }

    positions
}

/// Forward kinematics for simple planar robots
///
/// Uses 3D transforms but constrains motion to XY plane (Z=0)
/// This is the legacy method from Phase 1/2
fn forward_kinematics_planar(robot: &RobotArm) -> Vec<JointPosition3D> {
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
    use crate::dh_parameters::DHParameter;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < EPSILON
    }

    // ===== Tests for simple planar robots =====

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

    // ===== Tests for DH parameter robots =====

    #[test]
    fn test_fk_dh_planar_robot() {
        // Create planar robot using DH parameters
        let mut robot = RobotArm::planar(vec![2.0, 1.5]);
        robot.set_joint_angles(vec![0.0, 0.0]);

        let positions = forward_kinematics(&robot);

        // Should give same results as simple planar robot
        assert_eq!(positions.len(), 3);
        assert!(approx_eq(positions[2].x, 3.5));
        assert!(approx_eq(positions[2].y, 0.0));
        assert!(approx_eq(positions[2].z, 0.0));
    }

    #[test]
    fn test_fk_dh_planar_90_degrees() {
        // Planar DH robot with first joint at 90°
        let mut robot = RobotArm::planar(vec![2.0, 1.5]);
        robot.set_joint_angles(vec![PI / 2.0, 0.0]);

        let positions = forward_kinematics(&robot);

        // Should match simple planar robot
        assert!(approx_eq(positions[2].x, 0.0));
        assert!(approx_eq(positions[2].y, 3.5));
        assert!(approx_eq(positions[2].z, 0.0));
    }

    #[test]
    fn test_fk_dh_3d_robot_with_twist() {
        // Create a 2-DOF robot with twist in second joint
        let dh_params = vec![
            DHParameter::revolute(1.0, 0.0, 0.0, 0.0),      // First link: planar
            DHParameter::revolute(1.0, PI / 2.0, 0.0, 0.0), // Second link: 90° twist
        ];

        let mut robot = RobotArm::from_dh_params(dh_params);
        robot.set_joint_angles(vec![0.0, 0.0]);

        let positions = forward_kinematics(&robot);

        assert_eq!(positions.len(), 3);

        // First joint at (1, 0, 0)
        assert!(approx_eq(positions[1].x, 1.0));
        assert!(approx_eq(positions[1].y, 0.0));
        assert!(approx_eq(positions[1].z, 0.0));

        // End-effector at (2, 0, 0) - twist doesn't affect position without rotation
        assert!(approx_eq(positions[2].x, 2.0));
        assert!(approx_eq(positions[2].y, 0.0));
        assert!(approx_eq(positions[2].z, 0.0));
    }

    #[test]
    fn test_fk_dh_vertical_offset() {
        // Robot with vertical offset (d parameter)
        let dh_params = vec![
            DHParameter::revolute(0.0, 0.0, 0.5, 0.0), // Base with vertical offset
            DHParameter::revolute(1.0, 0.0, 0.0, 0.0), // Horizontal link
        ];

        let mut robot = RobotArm::from_dh_params(dh_params);
        robot.set_joint_angles(vec![0.0, 0.0]);

        let positions = forward_kinematics(&robot);

        // First joint should be at (0, 0, 0.5) due to d parameter
        assert!(approx_eq(positions[1].x, 0.0));
        assert!(approx_eq(positions[1].y, 0.0));
        assert!(approx_eq(positions[1].z, 0.5));

        // End-effector at (1, 0, 0.5)
        assert!(approx_eq(positions[2].x, 1.0));
        assert!(approx_eq(positions[2].y, 0.0));
        assert!(approx_eq(positions[2].z, 0.5));
    }
}
