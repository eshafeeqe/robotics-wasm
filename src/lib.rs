use wasm_bindgen::prelude::*;

mod geometry3d;
mod robot;
mod kinematics;
mod dh_parameters;

use robot::RobotArm;
use kinematics::{forward_kinematics, JointPosition3D};
use dh_parameters::DHParameter;

// Browser console logging
#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

#[wasm_bindgen]
pub struct RobotSimulator {
    robot: RobotArm,
}

#[wasm_bindgen]
impl RobotSimulator {
    /// Create a simple 2-DOF robot (backwards compatibility)
    #[wasm_bindgen(constructor)]
    pub fn new(link1_length: f64, link2_length: f64) -> Self {
        log(&format!(
            "Creating robot with link lengths: L1={}, L2={}",
            link1_length, link2_length
        ));

        RobotSimulator {
            robot: RobotArm::new(vec![link1_length, link2_length]),
        }
    }

    /// Create a robot from an array of link lengths (arbitrary-DOF, simple mode)
    pub fn new_simple(link_lengths: JsValue) -> Result<RobotSimulator, JsValue> {
        let lengths: Vec<f64> = serde_wasm_bindgen::from_value(link_lengths)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse link lengths: {}", e)))?;

        log(&format!(
            "Creating {}-DOF robot with link lengths: {:?}",
            lengths.len(),
            lengths
        ));

        Ok(RobotSimulator {
            robot: RobotArm::new(lengths),
        })
    }

    /// Create a robot from DH parameters (arbitrary-DOF, DH mode)
    pub fn new_dh(dh_params: JsValue) -> Result<RobotSimulator, JsValue> {
        let params: Vec<DHParameter> = serde_wasm_bindgen::from_value(dh_params)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse DH parameters: {}", e)))?;

        log(&format!(
            "Creating {}-DOF robot with DH parameters",
            params.len()
        ));

        Ok(RobotSimulator {
            robot: RobotArm::from_dh_params(params),
        })
    }

    /// Create a planar robot using DH representation (arbitrary-DOF)
    pub fn new_planar(link_lengths: JsValue) -> Result<RobotSimulator, JsValue> {
        let lengths: Vec<f64> = serde_wasm_bindgen::from_value(link_lengths)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse link lengths: {}", e)))?;

        log(&format!(
            "Creating {}-DOF planar robot (DH mode) with link lengths: {:?}",
            lengths.len(),
            lengths
        ));

        Ok(RobotSimulator {
            robot: RobotArm::planar(lengths),
        })
    }

    /// Set joint angles (2-DOF, backwards compatibility)
    pub fn set_angles(&mut self, theta1: f64, theta2: f64) {
        self.robot.set_joint_angles(vec![theta1, theta2]);
    }

    /// Set joint angles from array (arbitrary-DOF)
    pub fn set_angles_array(&mut self, angles: JsValue) -> Result<(), JsValue> {
        let angle_vec: Vec<f64> = serde_wasm_bindgen::from_value(angles)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse angles: {}", e)))?;

        self.robot.set_joint_angles(angle_vec);
        Ok(())
    }

    /// Get the number of joints in the robot
    pub fn num_joints(&self) -> usize {
        self.robot.num_joints()
    }

    pub fn get_joint_positions(&self) -> JsValue {
        let positions = forward_kinematics(&self.robot);

        // Convert Vec<JointPosition3D> to JavaScript array
        serde_wasm_bindgen::to_value(&positions)
            .unwrap_or_else(|_| JsValue::NULL)
    }

    pub fn get_end_effector_position(&self) -> JsValue {
        let positions = forward_kinematics(&self.robot);

        if let Some(end_effector) = positions.last() {
            serde_wasm_bindgen::to_value(&end_effector)
                .unwrap_or_else(|_| JsValue::NULL)
        } else {
            JsValue::NULL
        }
    }
}
