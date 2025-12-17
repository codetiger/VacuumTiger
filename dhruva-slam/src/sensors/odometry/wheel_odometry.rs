//! Wheel odometry from encoder ticks.
//!
//! Converts differential drive wheel encoder readings to pose deltas
//! using differential drive kinematics.
//!
//! Note: Some utility methods are defined for future use.

use crate::core::types::Pose2D;

/// Configuration for wheel odometry.
#[derive(Debug, Clone, Copy)]
pub struct WheelOdometryConfig {
    /// Encoder ticks per meter of wheel travel.
    ///
    /// This is calculated as: ticks_per_revolution / (π × wheel_diameter)
    pub ticks_per_meter: f32,

    /// Distance between wheel centers in meters.
    ///
    /// For the CRL-200S, this is approximately 0.17m.
    pub wheel_base: f32,
}

impl Default for WheelOdometryConfig {
    fn default() -> Self {
        Self {
            // Default values for CRL-200S (estimated, needs calibration)
            ticks_per_meter: 1000.0,
            wheel_base: 0.17,
        }
    }
}

/// Wheel odometry calculator.
///
/// Maintains encoder state and computes pose deltas from tick changes.
/// Handles 16-bit encoder wraparound correctly.
///
/// # Differential Drive Kinematics
///
/// For differential drive robots:
/// - Both wheels forward equally → straight line motion
/// - Wheels moving in opposite directions → rotation in place
/// - Unequal wheel motion → arc motion
///
/// The pose delta is computed in the robot's local frame, where:
/// - x = forward
/// - y = left
/// - theta = counter-clockwise rotation
#[derive(Debug)]
pub struct WheelOdometry {
    config: WheelOdometryConfig,
    last_left: Option<u16>,
    last_right: Option<u16>,
}

impl WheelOdometry {
    /// Create a new wheel odometry calculator.
    pub fn new(config: WheelOdometryConfig) -> Self {
        Self {
            config,
            last_left: None,
            last_right: None,
        }
    }

    /// Update with new encoder readings.
    ///
    /// Returns `None` on the first call (initializes state).
    /// Returns `Some(delta)` on subsequent calls with the pose change.
    ///
    /// The returned pose delta is in the robot's local frame at the
    /// start of the motion.
    pub fn update(&mut self, left: u16, right: u16) -> Option<Pose2D> {
        let result = match (self.last_left, self.last_right) {
            (Some(prev_left), Some(prev_right)) => {
                Some(self.compute_delta(prev_left, prev_right, left, right))
            }
            _ => None,
        };

        self.last_left = Some(left);
        self.last_right = Some(right);
        result
    }

    /// Compute the signed tick delta handling 16-bit wraparound.
    ///
    /// Uses wrapping subtraction to correctly handle the case where
    /// the encoder wraps from 65535 to 0 (or vice versa).
    #[inline]
    fn tick_delta(current: u16, previous: u16) -> i32 {
        // wrapping_sub handles overflow correctly
        // Cast to i16 interprets large positive differences as negative
        let diff = current.wrapping_sub(previous) as i16;
        diff as i32
    }

    /// Compute pose delta from encoder tick changes.
    fn compute_delta(&self, prev_left: u16, prev_right: u16, left: u16, right: u16) -> Pose2D {
        // Get tick deltas (signed, handles wraparound)
        let delta_left_ticks = Self::tick_delta(left, prev_left);
        let delta_right_ticks = Self::tick_delta(right, prev_right);

        // Convert to meters
        let delta_left = delta_left_ticks as f32 / self.config.ticks_per_meter;
        let delta_right = delta_right_ticks as f32 / self.config.ticks_per_meter;

        // Compute pose delta using differential drive kinematics
        self.differential_drive_delta(delta_left, delta_right)
    }

    /// Compute pose delta using differential drive kinematics.
    ///
    /// Given left and right wheel displacements (in meters), compute
    /// the resulting pose change in the robot's local frame.
    fn differential_drive_delta(&self, delta_left: f32, delta_right: f32) -> Pose2D {
        // Threshold for considering motion as straight-line
        const STRAIGHT_THRESHOLD: f32 = 1e-6;

        let delta_theta = (delta_right - delta_left) / self.config.wheel_base;

        if delta_theta.abs() < STRAIGHT_THRESHOLD {
            // Straight line motion
            let delta_x = (delta_left + delta_right) / 2.0;
            Pose2D::new(delta_x, 0.0, 0.0)
        } else {
            // Arc motion
            // Radius to the center of the robot (ICC - Instantaneous Center of Curvature)
            let radius = (delta_left + delta_right) / (2.0 * delta_theta);

            // Pose change in local frame
            let delta_x = radius * delta_theta.sin();
            let delta_y = radius * (1.0 - delta_theta.cos());

            Pose2D::new(delta_x, delta_y, delta_theta)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, PI};

    fn test_config() -> WheelOdometryConfig {
        WheelOdometryConfig {
            ticks_per_meter: 1000.0,
            wheel_base: 0.2, // 20cm wheel base for easy math
        }
    }

    #[test]
    fn test_first_update_returns_none() {
        let mut odom = WheelOdometry::new(test_config());
        assert!(odom.update(0, 0).is_none());
    }

    #[test]
    fn test_straight_forward() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // Both wheels move 1000 ticks = 1 meter forward
        let delta = odom.update(1000, 1000).unwrap();

        assert_relative_eq!(delta.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(delta.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(delta.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_straight_backward() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(1000, 1000);

        // Both wheels move backward (tick count decreases)
        let delta = odom.update(0, 0).unwrap();

        assert_relative_eq!(delta.x, -1.0, epsilon = 1e-6);
        assert_relative_eq!(delta.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(delta.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_rotation_in_place_ccw() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // For rotation in place, wheels move in opposite directions
        // For π/2 rotation (90°) CCW:
        // arc_length = (wheel_base/2) * theta = 0.1 * π/2 ≈ 0.157m
        // ticks = arc_length * ticks_per_meter ≈ 157 ticks
        let arc_length = 0.1 * FRAC_PI_2;
        let ticks = (arc_length * 1000.0) as i32;

        // Left wheel backward, right wheel forward for CCW rotation
        let left = (0i32 - ticks) as u16; // wrapping to near 65535
        let right = ticks as u16;

        let delta = odom.update(left, right).unwrap();

        // Should have rotated ~90° CCW with minimal translation
        assert_relative_eq!(delta.theta, FRAC_PI_2, epsilon = 0.01);
        // x and y should be near zero for pure rotation
        assert!(delta.x.abs() < 0.01);
    }

    #[test]
    fn test_rotation_in_place_cw() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // For clockwise rotation: left forward, right backward
        let arc_length = 0.1 * FRAC_PI_2;
        let ticks = (arc_length * 1000.0) as i32;

        let left = ticks as u16;
        let right = (0i32 - ticks) as u16;

        let delta = odom.update(left, right).unwrap();

        // Should have rotated ~90° CW (negative theta)
        assert_relative_eq!(delta.theta, -FRAC_PI_2, epsilon = 0.01);
    }

    #[test]
    fn test_arc_motion() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // Right wheel moves more = arc to the left
        // This is a gentle left turn
        let delta = odom.update(100, 200).unwrap();

        // Should have positive forward motion and positive theta (CCW)
        assert!(delta.x > 0.0);
        assert!(delta.theta > 0.0);
    }

    #[test]
    fn test_encoder_wraparound_forward() {
        let mut odom = WheelOdometry::new(test_config());

        // Start near max value
        odom.update(65500, 65500);

        // Wrap around to small values (36 + 36 = 72 ticks forward each)
        let delta = odom.update(0, 0).unwrap();

        // Should be ~36 ticks = 0.036m forward
        let expected = 36.0 / 1000.0;
        assert_relative_eq!(delta.x, expected, epsilon = 0.001);
        assert_relative_eq!(delta.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_encoder_wraparound_backward() {
        let mut odom = WheelOdometry::new(test_config());

        // Start at small values
        odom.update(100, 100);

        // Wrap to large values (going backward)
        // 100 - 65436 = -65336 as u16 arithmetic, but as i16 = -100
        let delta = odom.update(0, 0).unwrap();

        // Should be -0.1m backward
        assert_relative_eq!(delta.x, -0.1, epsilon = 0.001);
    }

    #[test]
    fn test_half_rotation() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // 180° rotation in place (π radians)
        // Each wheel travels: arc = (wheel_base / 2) * theta
        // For π radians: arc = 0.1 * π ≈ 0.314m per wheel
        // Ticks = arc * ticks_per_meter = 0.314 * 1000 ≈ 314
        let arc_per_wheel = 0.1 * PI; // wheel_base/2 * π
        let ticks = (arc_per_wheel * 1000.0) as i32;

        // Left backward, right forward for CCW rotation
        let left = (0i32 - ticks) as u16;
        let right = ticks as u16;

        let delta = odom.update(left, right).unwrap();

        // Should have rotated ~π (half rotation)
        // Note: theta gets normalized to [-π, π] by Pose2D::new
        assert_relative_eq!(delta.theta.abs(), PI, epsilon = 0.1);
    }

    #[test]
    fn test_full_rotation_is_identity() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // Full 360° rotation (2π radians) should result in ~0 normalized theta
        // because Pose2D normalizes theta to [-π, π]
        let arc_per_wheel = 0.1 * 2.0 * PI;
        let ticks = (arc_per_wheel * 1000.0) as i32;

        let left = (0i32 - ticks) as u16;
        let right = ticks as u16;

        let delta = odom.update(left, right).unwrap();

        // Full rotation normalizes to ~0
        assert_relative_eq!(delta.theta, 0.0, epsilon = 0.1);
        // Position should still be near zero for pure rotation
        assert!(delta.x.abs() < 0.01);
    }

    // ========================================================================
    // Edge Case Tests
    // ========================================================================

    #[test]
    fn test_maximum_encoder_values() {
        let mut odom = WheelOdometry::new(test_config());

        // Start at maximum values
        odom.update(65535, 65535);

        // Stay at maximum (no motion)
        let delta = odom.update(65535, 65535).unwrap();
        assert_eq!(delta.x, 0.0);
        assert_eq!(delta.theta, 0.0);
    }

    #[test]
    fn test_alternating_forward_backward() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(1000, 1000);

        // Forward 0.1m
        let d1 = odom.update(1100, 1100).unwrap();
        assert_relative_eq!(d1.x, 0.1, epsilon = 0.001);

        // Backward 0.1m
        let d2 = odom.update(1000, 1000).unwrap();
        assert_relative_eq!(d2.x, -0.1, epsilon = 0.001);

        // Forward again
        let d3 = odom.update(1100, 1100).unwrap();
        assert_relative_eq!(d3.x, 0.1, epsilon = 0.001);
    }

    #[test]
    fn test_very_small_tick_change() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // Single tick on each wheel = 0.001m
        let delta = odom.update(1, 1).unwrap();
        assert_relative_eq!(delta.x, 0.001, epsilon = 1e-6);
        assert_relative_eq!(delta.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(delta.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_asymmetric_one_wheel_stopped() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // Only right wheel moves forward
        let delta = odom.update(0, 100).unwrap();

        // Should pivot around left wheel (CCW turn)
        assert!(delta.theta > 0.0, "Should rotate CCW");
        assert!(delta.x > 0.0, "Should move forward");
        assert!(delta.y > 0.0, "Should drift left");
    }

    #[test]
    fn test_asymmetric_left_wheel_only() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // Only left wheel moves forward
        let delta = odom.update(100, 0).unwrap();

        // Should pivot around right wheel (CW turn)
        assert!(delta.theta < 0.0, "Should rotate CW");
        assert!(delta.x > 0.0, "Should move forward");
        assert!(delta.y < 0.0, "Should drift right");
    }

    #[test]
    fn test_large_single_step() {
        let mut odom = WheelOdometry::new(test_config());

        odom.update(0, 0);

        // Large forward jump: 10m = 10000 ticks
        let delta = odom.update(10000, 10000).unwrap();
        assert_relative_eq!(delta.x, 10.0, epsilon = 0.01);
    }

    #[test]
    fn test_wraparound_at_boundary() {
        let mut odom = WheelOdometry::new(test_config());

        // Exactly at wraparound point
        odom.update(65535, 65535);

        // Cross to 0
        let delta = odom.update(0, 0).unwrap();
        assert_relative_eq!(delta.x, 0.001, epsilon = 0.0001); // 1 tick

        // And continue
        let delta2 = odom.update(100, 100).unwrap();
        assert_relative_eq!(delta2.x, 0.1, epsilon = 0.001);
    }
}
