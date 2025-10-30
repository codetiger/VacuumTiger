//! Mock motor driver for testing

use crate::drivers::MotorDriver;
use crate::error::Result;
use crate::types::{Odometry, Velocity};
use std::sync::{Arc, Mutex};

/// Mock motor driver
#[derive(Clone)]
pub struct MockMotorDriver {
    state: Arc<Mutex<MockMotorState>>,
}

#[derive(Debug, Clone)]
struct MockMotorState {
    linear: f32,
    angular: f32,
    left: f32,
    right: f32,
    encoder_left: i32,
    encoder_right: i32,
}

impl MockMotorDriver {
    /// Create new mock motor driver
    pub fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(MockMotorState {
                linear: 0.0,
                angular: 0.0,
                left: 0.0,
                right: 0.0,
                encoder_left: 0,
                encoder_right: 0,
            })),
        }
    }

    /// Get current velocity
    pub fn get_velocity(&self) -> (f32, f32) {
        let state = self.state.lock().unwrap();
        (state.linear, state.angular)
    }

    /// Get wheel velocities
    pub fn get_wheel_velocity(&self) -> (f32, f32) {
        let state = self.state.lock().unwrap();
        (state.left, state.right)
    }
}

impl Default for MockMotorDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorDriver for MockMotorDriver {
    fn set_velocity(&mut self, linear: f32, angular: f32) -> Result<()> {
        let mut state = self.state.lock().unwrap();
        state.linear = linear;
        state.angular = angular;
        Ok(())
    }

    fn set_wheel_velocity(&mut self, left: f32, right: f32) -> Result<()> {
        let mut state = self.state.lock().unwrap();
        state.left = left;
        state.right = right;
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        let mut state = self.state.lock().unwrap();
        state.linear = 0.0;
        state.angular = 0.0;
        state.left = 0.0;
        state.right = 0.0;
        Ok(())
    }

    fn emergency_stop(&mut self) -> Result<()> {
        self.stop()
    }

    fn get_odometry(&mut self) -> Result<Odometry> {
        let state = self.state.lock().unwrap();
        Ok(Odometry {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            velocity: Velocity::new(state.linear, state.angular),
            encoder_left: state.encoder_left,
            encoder_right: state.encoder_right,
        })
    }
}
