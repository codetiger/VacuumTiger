//! Motor driver trait

use crate::error::Result;
use crate::types::Odometry;
use std::any::Any;

/// Motor controller driver trait
pub trait MotorDriver: Send + Any {
    /// Set differential drive velocity
    ///
    /// # Arguments
    /// * `linear` - Linear velocity in m/s
    /// * `angular` - Angular velocity in rad/s
    #[allow(dead_code)] // Part of trait API, may be used by future implementations
    fn set_velocity(&mut self, linear: f32, angular: f32) -> Result<()>;

    /// Set individual wheel velocities
    ///
    /// # Arguments
    /// * `left` - Left wheel velocity in m/s
    /// * `right` - Right wheel velocity in m/s
    fn set_wheel_velocity(&mut self, left: f32, right: f32) -> Result<()>;

    /// Stop all motors immediately
    #[allow(dead_code)] // Part of trait API, may be used by future implementations
    fn stop(&mut self) -> Result<()>;

    /// Emergency stop (may require re-initialization)
    #[allow(dead_code)] // Part of trait API, may be used by future implementations
    fn emergency_stop(&mut self) -> Result<()>;

    /// Get current odometry
    fn get_odometry(&mut self) -> Result<Odometry>;

    /// Set air pump power (0-100%)
    fn set_air_pump(&mut self, power: u8) -> Result<()> {
        let _ = power;
        Err(crate::Error::NotSupported(
            "Air pump control not supported".to_string(),
        ))
    }

    /// Set side brush speed (0-100%)
    fn set_side_brush(&mut self, speed: u8) -> Result<()> {
        let _ = speed;
        Err(crate::Error::NotSupported(
            "Side brush control not supported".to_string(),
        ))
    }

    /// Set rolling brush speed (0-100%)
    fn set_rolling_brush(&mut self, speed: u8) -> Result<()> {
        let _ = speed;
        Err(crate::Error::NotSupported(
            "Rolling brush control not supported".to_string(),
        ))
    }
}
