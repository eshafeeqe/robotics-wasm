pub struct RobotArm {
    pub link_lengths: Vec<f64>,
    pub joint_angles: Vec<f64>,
}

impl RobotArm {
    pub fn new(link_lengths: Vec<f64>) -> Self {
        let num_joints = link_lengths.len();
        RobotArm {
            link_lengths,
            joint_angles: vec![0.0; num_joints],
        }
    }

    pub fn set_joint_angles(&mut self, angles: Vec<f64>) {
        if angles.len() == self.joint_angles.len() {
            self.joint_angles = angles;
        }
    }

    pub fn num_joints(&self) -> usize {
        self.link_lengths.len()
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
}
