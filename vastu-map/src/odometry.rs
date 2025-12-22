//! Wheel odometry for differential drive robots.
//!
//! Computes robot pose changes from wheel encoder ticks using
//! standard differential drive kinematics with arc-based integration.
//!
//! # Example
//!
//! ```rust
//! use vastu_map::odometry::WheelOdometry;
//!
//! // Create odometry tracker (0.3m wheel base, 1000 ticks/meter)
//! let mut odom = WheelOdometry::new(0.3, 1000.0);
//!
//! // Process encoder readings
//! let delta = odom.update(100, 100); // Straight forward
//! println!("Moved: ({:.3}, {:.3}, {:.3})", delta.x, delta.y, delta.theta);
//!
//! // Get cumulative pose
//! let total = odom.cumulative();
//! println!("Total: ({:.3}, {:.3}, {:.3})", total.x, total.y, total.theta);
//! ```

use crate::core::Pose2D;

/// Differential drive wheel odometry.
///
/// Tracks robot pose from wheel encoder ticks. Uses arc-based motion model
/// for accurate integration, especially during turns.
///
/// # Coordinate Frame
///
/// Follows ROS REP-103 convention:
/// - X-forward (positive X is in front of robot)
/// - Y-left (positive Y is to the left)
/// - Theta counter-clockwise positive
#[derive(Clone, Debug)]
pub struct WheelOdometry {
    /// Distance between wheel centers (meters).
    wheel_base: f32,

    /// Encoder ticks per meter of wheel travel.
    ticks_per_meter: f32,

    /// Last left encoder reading (ticks).
    last_left: i32,

    /// Last right encoder reading (ticks).
    last_right: i32,

    /// Cumulative pose from origin.
    cumulative_pose: Pose2D,

    /// Whether the odometry has been initialized with first reading.
    initialized: bool,
}

impl WheelOdometry {
    /// Create a new wheel odometry tracker.
    ///
    /// # Arguments
    ///
    /// * `wheel_base` - Distance between wheel centers in meters
    /// * `ticks_per_meter` - Encoder ticks per meter of wheel travel
    ///
    /// # Panics
    ///
    /// Panics if `wheel_base` or `ticks_per_meter` is not positive.
    pub fn new(wheel_base: f32, ticks_per_meter: f32) -> Self {
        assert!(wheel_base > 0.0, "wheel_base must be positive");
        assert!(ticks_per_meter > 0.0, "ticks_per_meter must be positive");

        Self {
            wheel_base,
            ticks_per_meter,
            last_left: 0,
            last_right: 0,
            cumulative_pose: Pose2D::identity(),
            initialized: false,
        }
    }

    /// Process encoder readings and return pose delta since last update.
    ///
    /// On the first call, initializes the encoder baselines and returns identity pose.
    /// Subsequent calls compute the pose change using arc-based motion model.
    ///
    /// # Arguments
    ///
    /// * `left_ticks` - Current left wheel encoder reading (ticks)
    /// * `right_ticks` - Current right wheel encoder reading (ticks)
    ///
    /// # Returns
    ///
    /// Pose delta in robot frame (how the robot moved since last update).
    pub fn update(&mut self, left_ticks: i32, right_ticks: i32) -> Pose2D {
        if !self.initialized {
            // First reading - just store baselines
            self.last_left = left_ticks;
            self.last_right = right_ticks;
            self.initialized = true;
            return Pose2D::identity();
        }

        // Compute tick deltas
        let delta_left_ticks = left_ticks - self.last_left;
        let delta_right_ticks = right_ticks - self.last_right;

        // Update stored values
        self.last_left = left_ticks;
        self.last_right = right_ticks;

        // Convert to meters
        let delta_left = delta_left_ticks as f32 / self.ticks_per_meter;
        let delta_right = delta_right_ticks as f32 / self.ticks_per_meter;

        // Differential drive kinematics
        let delta_theta = (delta_right - delta_left) / self.wheel_base;
        let delta_linear = (delta_left + delta_right) / 2.0;

        // Robot-frame delta (for return value, used by SLAM's compose())
        let (dx_robot, dy_robot) = if delta_theta.abs() < 1e-6 {
            // Straight motion: forward is +X in robot frame
            (delta_linear, 0.0)
        } else {
            // Arc motion in robot's starting frame
            let radius = delta_linear / delta_theta;
            (
                radius * delta_theta.sin(),         // Forward displacement
                radius * (1.0 - delta_theta.cos()), // Lateral displacement (left +)
            )
        };

        // World-frame delta (for cumulative pose tracking)
        let theta = self.cumulative_pose.theta;
        let (sin_t, cos_t) = theta.sin_cos();
        let dx_world = dx_robot * cos_t - dy_robot * sin_t;
        let dy_world = dx_robot * sin_t + dy_robot * cos_t;

        // Create delta pose (in robot frame for compose())
        let delta = Pose2D::new(dx_robot, dy_robot, delta_theta);

        // Update cumulative pose (world frame)
        self.cumulative_pose = Pose2D::new(
            self.cumulative_pose.x + dx_world,
            self.cumulative_pose.y + dy_world,
            crate::core::math::normalize_angle(self.cumulative_pose.theta + delta_theta),
        );

        delta
    }

    /// Get the cumulative pose from origin.
    ///
    /// This is the total pose accumulated since creation or last reset.
    #[inline]
    pub fn cumulative(&self) -> Pose2D {
        self.cumulative_pose
    }

    /// Reset odometry to origin.
    ///
    /// Clears the cumulative pose but keeps the current encoder readings
    /// as the new baseline.
    pub fn reset(&mut self) {
        self.cumulative_pose = Pose2D::identity();
    }

    /// Reset odometry completely.
    ///
    /// Clears both cumulative pose and encoder baselines.
    /// The next `update()` call will re-initialize.
    pub fn reset_all(&mut self) {
        self.cumulative_pose = Pose2D::identity();
        self.last_left = 0;
        self.last_right = 0;
        self.initialized = false;
    }

    /// Get the wheel base (distance between wheels).
    #[inline]
    pub fn wheel_base(&self) -> f32 {
        self.wheel_base
    }

    /// Get the ticks per meter conversion factor.
    #[inline]
    pub fn ticks_per_meter(&self) -> f32 {
        self.ticks_per_meter
    }

    /// Check if odometry has been initialized.
    #[inline]
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::PI;

    #[test]
    fn test_new() {
        let odom = WheelOdometry::new(0.3, 1000.0);
        assert_eq!(odom.wheel_base(), 0.3);
        assert_eq!(odom.ticks_per_meter(), 1000.0);
        assert!(!odom.is_initialized());
    }

    #[test]
    #[should_panic(expected = "wheel_base must be positive")]
    fn test_new_invalid_wheel_base() {
        WheelOdometry::new(0.0, 1000.0);
    }

    #[test]
    #[should_panic(expected = "ticks_per_meter must be positive")]
    fn test_new_invalid_ticks_per_meter() {
        WheelOdometry::new(0.3, 0.0);
    }

    #[test]
    fn test_first_update_initializes() {
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        let delta = odom.update(100, 100);

        assert!(odom.is_initialized());
        assert_eq!(delta, Pose2D::identity());
    }

    #[test]
    fn test_straight_forward() {
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        // Initialize
        odom.update(0, 0);

        // Move 1 meter forward (1000 ticks on each wheel)
        let delta = odom.update(1000, 1000);

        assert_relative_eq!(delta.x, 1.0, epsilon = 0.001);
        assert_relative_eq!(delta.y, 0.0, epsilon = 0.001);
        assert_relative_eq!(delta.theta, 0.0, epsilon = 0.001);

        let cum = odom.cumulative();
        assert_relative_eq!(cum.x, 1.0, epsilon = 0.001);
        assert_relative_eq!(cum.y, 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_straight_backward() {
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        odom.update(0, 0);

        // Move 0.5 meter backward
        let delta = odom.update(-500, -500);

        assert_relative_eq!(delta.x, -0.5, epsilon = 0.001);
        assert_relative_eq!(delta.y, 0.0, epsilon = 0.001);
        assert_relative_eq!(delta.theta, 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_rotate_in_place_ccw() {
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        odom.update(0, 0);

        // Rotate counter-clockwise: left wheel back, right wheel forward
        // For 90° rotation: arc = wheel_base * theta / 2 for each wheel
        // theta = PI/2, arc = 0.3 * PI/2 / 2 = 0.2356m per wheel
        // ticks = 0.2356 * 1000 = 235.6
        let ticks = (0.3 * PI / 4.0 * 1000.0) as i32; // ~236
        let delta = odom.update(-ticks, ticks);

        // Should rotate ~90° with minimal translation
        assert_relative_eq!(delta.theta, PI / 2.0, epsilon = 0.01);
        // Translation should be minimal for pure rotation
        assert!(delta.x.abs() < 0.01);
    }

    #[test]
    fn test_arc_motion() {
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        odom.update(0, 0);

        // Arc motion: right wheel travels further than left
        // This should result in a left turn
        let delta = odom.update(800, 1200);

        // Should have forward motion and counter-clockwise rotation
        assert!(delta.x > 0.0);
        assert!(delta.theta > 0.0);
    }

    #[test]
    fn test_cumulative_multiple_updates() {
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        odom.update(0, 0);

        // Move forward 1m
        odom.update(1000, 1000);
        // Move forward another 1m
        odom.update(2000, 2000);

        let cum = odom.cumulative();
        assert_relative_eq!(cum.x, 2.0, epsilon = 0.001);
        assert_relative_eq!(cum.y, 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_reset() {
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        odom.update(0, 0);
        odom.update(1000, 1000);

        assert!(odom.cumulative().x > 0.0);

        odom.reset();

        let cum = odom.cumulative();
        assert_eq!(cum.x, 0.0);
        assert_eq!(cum.y, 0.0);
        assert_eq!(cum.theta, 0.0);

        // Should still be initialized
        assert!(odom.is_initialized());
    }

    #[test]
    fn test_reset_all() {
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        odom.update(0, 0);
        odom.update(1000, 1000);

        odom.reset_all();

        assert!(!odom.is_initialized());

        // Next update should re-initialize
        let delta = odom.update(500, 500);
        assert_eq!(delta, Pose2D::identity());
        assert!(odom.is_initialized());
    }

    #[test]
    fn test_encoder_overflow_handling() {
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        // Start near max i32
        odom.update(i32::MAX - 500, i32::MAX - 500);

        // Overflow (in real hardware, this would wrap)
        // For now, just ensure it doesn't panic
        let delta = odom.update(i32::MAX - 400, i32::MAX - 400);

        // Should show forward motion of 100 ticks = 0.1m
        assert_relative_eq!(delta.x, 0.1, epsilon = 0.001);
    }

    #[test]
    fn test_delta_is_robot_frame_after_rotation() {
        // This test verifies that update() returns robot-frame delta,
        // not world-frame. After rotating 90°, moving forward should
        // give delta.x > 0 (forward in robot frame), not delta.y > 0.
        let mut odom = WheelOdometry::new(0.3, 1000.0);

        // Initialize
        odom.update(0, 0);

        // Rotate 90° CCW (in-place rotation)
        // arc = wheel_base * theta / 2 = 0.3 * π/2 / 2 = 0.2356m per wheel
        let ticks = (0.3 * PI / 4.0 * 1000.0) as i32; // ~236 ticks
        let left = -ticks;
        let right = ticks;
        odom.update(left, right);

        // Verify we're now at ~90° heading
        let cum = odom.cumulative();
        assert_relative_eq!(cum.theta, PI / 2.0, epsilon = 0.02);

        // Now move forward 0.5m (equal wheel ticks)
        let forward_ticks = 500;
        let delta = odom.update(left + forward_ticks, right + forward_ticks);

        // Delta should be in ROBOT frame: forward is +X regardless of world heading
        // If bug existed (world frame), delta would be (0, 0.5) since robot faces +Y
        assert_relative_eq!(delta.x, 0.5, epsilon = 0.01); // Forward in robot frame
        assert_relative_eq!(delta.y, 0.0, epsilon = 0.01); // No lateral movement
        assert_relative_eq!(delta.theta, 0.0, epsilon = 0.01); // No rotation

        // But cumulative should be in WORLD frame: robot at ~(0, 0.5)
        let cum2 = odom.cumulative();
        assert_relative_eq!(cum2.x, 0.0, epsilon = 0.05); // Started near origin
        assert_relative_eq!(cum2.y, 0.5, epsilon = 0.05); // Moved in +Y direction
    }
}
