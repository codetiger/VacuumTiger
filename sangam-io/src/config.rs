//! Configuration for SangamIO hardware abstraction layer

use std::time::Duration;

/// Configuration parameters for SangamIO
#[derive(Debug, Clone)]
pub struct SangamConfig {
    // === Physical Parameters ===
    /// Distance between wheels (meters)
    pub wheel_base: f32,

    /// Wheel radius (meters)
    pub wheel_radius: f32,

    /// Encoder ticks per wheel revolution
    pub ticks_per_revolution: f32,

    // === Motion Constraints ===
    /// Maximum linear velocity (m/s)
    pub max_linear_velocity: f32,

    /// Maximum angular velocity (rad/s)
    pub max_angular_velocity: f32,

    /// Linear acceleration limit (m/s²)
    pub linear_acceleration: f32,

    /// Angular acceleration limit (rad/s²)
    pub angular_acceleration: f32,

    /// Emergency deceleration rate (m/s²) - higher than normal
    pub emergency_deceleration: f32,

    // === Control Loop ===
    /// Control loop frequency (Hz)
    pub control_frequency: u32,

    /// Control loop period
    pub control_period: Duration,

    // === Timeouts ===
    /// Lidar stabilization delay after GD32 init
    pub lidar_stabilization_delay: Duration,
}

impl SangamConfig {
    /// Default configuration for CRL-200S robot
    pub fn crl200s_defaults() -> Self {
        let control_frequency = 50;
        Self {
            // Physical parameters (calibrated for CRL-200S)
            wheel_base: 0.235,
            wheel_radius: 0.0325,
            ticks_per_revolution: 1560.0,

            // Motion constraints (conservative defaults)
            max_linear_velocity: 0.5,    // 0.5 m/s max forward speed
            max_angular_velocity: 2.0,   // 2.0 rad/s max rotation
            linear_acceleration: 0.3,    // 0.3 m/s² acceleration
            angular_acceleration: 1.5,   // 1.5 rad/s² angular accel
            emergency_deceleration: 1.0, // 1.0 m/s² emergency stop

            // Control loop
            control_frequency,
            control_period: Duration::from_millis(1000 / control_frequency as u64),

            // Timeouts
            lidar_stabilization_delay: Duration::from_millis(1400),
        }
    }

    /// Convert encoder ticks to distance
    pub fn ticks_to_meters(&self, ticks: i32) -> f32 {
        (ticks as f32 / self.ticks_per_revolution)
            * (2.0 * std::f32::consts::PI * self.wheel_radius)
    }

    /// Calculate differential drive kinematics
    /// Returns (left_wheel_speed, right_wheel_speed) in m/s
    pub fn differential_drive_kinematics(&self, linear: f32, angular: f32) -> (f32, f32) {
        let left = linear - (angular * self.wheel_base / 2.0);
        let right = linear + (angular * self.wheel_base / 2.0);
        (left, right)
    }
}

impl Default for SangamConfig {
    fn default() -> Self {
        Self::crl200s_defaults()
    }
}
