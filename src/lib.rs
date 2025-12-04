use wasm_bindgen::prelude::*;

mod geometry3d;
mod robot;
mod kinematics;
mod dh_parameters;

use robot::RobotArm;
use kinematics::{forward_kinematics, JointPosition3D};

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

    pub fn set_angles(&mut self, theta1: f64, theta2: f64) {
        self.robot.set_joint_angles(vec![theta1, theta2]);
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
