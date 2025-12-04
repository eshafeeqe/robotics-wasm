use crate::dh_parameters::DHParameter;

/// Robot arm configuration
///
/// Can be configured in two ways:
/// 1. Simple planar robot: link_lengths + joint_angles (Phase 1/2 style)
/// 2. DH parameters: Full Denavit-Hartenberg specification (Phase 2b+)
pub struct RobotArm {
    /// Optional DH parameters (if using DH convention)
    pub dh_params: Option<Vec<DHParameter>>,
    /// Simple link lengths (used if dh_params is None)
    pub link_lengths: Vec<f64>,
    /// Current joint values (angles for revolute, distances for prismatic)
    pub joint_angles: Vec<f64>,
}

impl RobotArm {
    /// Create a simple planar robot from link lengths
    ///
    /// This creates a robot where each link rotates in the XY plane (planar motion).
    /// Equivalent to DH parameters with α=0, d=0 for all joints.
    pub fn new(link_lengths: Vec<f64>) -> Self {
        let num_joints = link_lengths.len();
        RobotArm {
            dh_params: None,
            link_lengths,
            joint_angles: vec![0.0; num_joints],
        }
    }

    /// Create a robot from DH parameters
    ///
    /// This allows full 3D robot specification using Denavit-Hartenberg convention.
    pub fn from_dh_params(dh_params: Vec<DHParameter>) -> Self {
        let num_joints = dh_params.len();
        RobotArm {
            dh_params: Some(dh_params),
            link_lengths: vec![], // Not used for DH robots
            joint_angles: vec![0.0; num_joints],
        }
    }

    /// Create a planar robot using DH parameter representation
    ///
    /// This is a convenience method that creates DH parameters for a planar robot.
    /// Useful for transitioning from simple to DH-based representation.
    pub fn planar(link_lengths: Vec<f64>) -> Self {
        let dh_params: Vec<DHParameter> = link_lengths
            .iter()
            .map(|&length| DHParameter::planar(length))
            .collect();

        let num_joints = dh_params.len();
        RobotArm {
            dh_params: Some(dh_params),
            link_lengths: vec![],
            joint_angles: vec![0.0; num_joints],
        }
    }

    /// Set the joint angles/positions for all joints
    pub fn set_joint_angles(&mut self, angles: Vec<f64>) {
        if angles.len() == self.joint_angles.len() {
            self.joint_angles = angles;
        }
    }

    /// Get the number of joints in the robot
    pub fn num_joints(&self) -> usize {
        self.joint_angles.len()
    }

    /// Check if this robot uses DH parameters
    pub fn uses_dh_params(&self) -> bool {
        self.dh_params.is_some()
    }

    /// Get DH parameters with current joint values applied
    ///
    /// Returns None if robot doesn't use DH parameters
    pub fn get_dh_with_current_values(&self) -> Option<Vec<DHParameter>> {
        self.dh_params.as_ref().map(|dh_params| {
            dh_params
                .iter()
                .zip(self.joint_angles.iter())
                .map(|(dh, &value)| dh.with_joint_value(value))
                .collect()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_robot_creation() {
        let robot = RobotArm::new(vec![2.0, 1.5]);
        assert_eq!(robot.num_joints(), 2);
        assert_eq!(robot.link_lengths, vec![2.0, 1.5]);
        assert_eq!(robot.joint_angles, vec![0.0, 0.0]);
        assert!(!robot.uses_dh_params());
    }

    #[test]
    fn test_set_joint_angles() {
        let mut robot = RobotArm::new(vec![2.0, 1.5]);
        robot.set_joint_angles(vec![1.57, 0.78]);
        assert_eq!(robot.joint_angles, vec![1.57, 0.78]);
    }

    #[test]
    fn test_set_joint_angles_wrong_length() {
        let mut robot = RobotArm::new(vec![2.0, 1.5]);
        robot.set_joint_angles(vec![1.57]);
        // Should not update if length doesn't match
        assert_eq!(robot.joint_angles, vec![0.0, 0.0]);
    }

    #[test]
    fn test_planar_robot_with_dh() {
        let robot = RobotArm::planar(vec![2.0, 1.5]);
        assert_eq!(robot.num_joints(), 2);
        assert!(robot.uses_dh_params());
    }

    #[test]
    fn test_dh_robot_creation() {
        let dh_params = vec![
            DHParameter::revolute(1.0, 0.0, 0.0, 0.0),
            DHParameter::revolute(1.5, 0.0, 0.0, 0.0),
        ];

        let robot = RobotArm::from_dh_params(dh_params);
        assert_eq!(robot.num_joints(), 2);
        assert!(robot.uses_dh_params());
    }

    #[test]
    fn test_get_dh_with_current_values() {
        let robot = RobotArm::planar(vec![2.0, 1.5]);
        let dh_params = robot.get_dh_with_current_values();

        assert!(dh_params.is_some());
        let params = dh_params.unwrap();
        assert_eq!(params.len(), 2);
        assert_eq!(params[0].a, 2.0);
        assert_eq!(params[1].a, 1.5);
    }

    #[test]
    fn test_simple_robot_no_dh() {
        let robot = RobotArm::new(vec![2.0, 1.5]);
        assert!(robot.get_dh_with_current_values().is_none());
    }
}
