use crate::geometry::{Point2D, Transform2D};
use crate::robot::RobotArm;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct JointPosition {
    pub x: f64,
    pub y: f64,
}

impl JointPosition {
    pub fn new(x: f64, y: f64) -> Self {
        JointPosition { x, y }
    }

    pub fn from_point(point: &Point2D) -> Self {
        JointPosition {
            x: point.x,
            y: point.y,
        }
    }
}

/// Computes forward kinematics for a planar robot arm
///
/// Returns positions of all joints including base and end-effector
/// For a 2-DOF robot: [base, joint1, joint2, end-effector] (4 positions)
pub fn forward_kinematics(robot: &RobotArm) -> Vec<JointPosition> {
    let mut positions = Vec::new();
    let mut current_transform = Transform2D::identity();

    // Add base position at origin
    let base = current_transform.transform_point(&Point2D::origin());
    positions.push(JointPosition::from_point(&base));

    // For each link, compute the joint position
    for i in 0..robot.num_joints() {
        // Rotate by joint angle
        let rotation = Transform2D::rotation(robot.joint_angles[i]);
        current_transform = current_transform.compose(&rotation);

        // Translate along link length (along local X-axis)
        let translation = Transform2D::translation(robot.link_lengths[i], 0.0);
        current_transform = current_transform.compose(&translation);

        // Record position after this link
        let joint_pos = current_transform.transform_point(&Point2D::origin());
        positions.push(JointPosition::from_point(&joint_pos));
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

        // Joint 1 at (2.0, 0.0)
        assert!(approx_eq(positions[1].x, 2.0));
        assert!(approx_eq(positions[1].y, 0.0));

        // End-effector at (3.5, 0.0)
        assert!(approx_eq(positions[2].x, 3.5));
        assert!(approx_eq(positions[2].y, 0.0));
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

        // Joint 1 should be at (0, 2.0) after rotating 90°
        assert!(approx_eq(positions[1].x, 0.0));
        assert!(approx_eq(positions[1].y, 2.0));

        // End-effector should be at (0, 3.5) - extending upward
        assert!(approx_eq(positions[2].x, 0.0));
        assert!(approx_eq(positions[2].y, 3.5));
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

        // Joint 1 at (0, 2.0)
        assert!(approx_eq(positions[1].x, 0.0));
        assert!(approx_eq(positions[1].y, 2.0));

        // End-effector should bend back to (-1.5, 2.0)
        assert!(approx_eq(positions[2].x, -1.5));
        assert!(approx_eq(positions[2].y, 2.0));
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

        // Joint 1 at (2.0, 0.0)
        assert!(approx_eq(positions[1].x, 2.0));
        assert!(approx_eq(positions[1].y, 0.0));

        // End-effector should be at (0.5, 0.0) - folded back
        assert!(approx_eq(positions[2].x, 0.5));
        assert!(approx_eq(positions[2].y, 0.0));
    }
}
