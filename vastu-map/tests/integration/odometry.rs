//! Odometry simulation from encoder ticks
//!
//! Simulates odometry estimation using differential drive kinematics,
//! with configurable drift and noise for testing SLAM robustness.

use vastu_map::core::Pose2D;

/// Odometry drift configuration.
#[derive(Clone, Debug)]
pub struct OdometryConfig {
    /// Systematic drift per meter traveled (e.g., 0.02 = 2% drift)
    pub drift_per_meter: f32,
    /// Angular drift per radian rotated
    pub angular_drift_per_rad: f32,
}

impl Default for OdometryConfig {
    fn default() -> Self {
        Self {
            drift_per_meter: 0.0,
            angular_drift_per_rad: 0.0,
        }
    }
}

impl OdometryConfig {
    /// Create a clean odometry config with no drift.
    pub fn clean() -> Self {
        Self::default()
    }
}

/// Odometry estimator that tracks robot pose from encoder ticks.
///
/// Uses differential drive kinematics to compute pose changes from
/// left/right wheel encoder tick differences.
pub struct OdometryEstimator {
    config: OdometryConfig,
    accumulated_pose: Pose2D,
    last_left_ticks: u16,
    last_right_ticks: u16,
    wheel_base: f32,
    ticks_per_meter: f32,
    initialized: bool,
}

impl OdometryEstimator {
    /// Create a new odometry estimator.
    ///
    /// # Arguments
    /// * `wheel_base` - Distance between wheels in meters
    /// * `ticks_per_meter` - Encoder ticks per meter of travel
    /// * `config` - Drift/noise configuration
    pub fn new(wheel_base: f32, ticks_per_meter: f32, config: OdometryConfig) -> Self {
        Self {
            config,
            accumulated_pose: Pose2D::identity(),
            last_left_ticks: 0,
            last_right_ticks: 0,
            wheel_base,
            ticks_per_meter,
            initialized: false,
        }
    }

    /// Initialize with starting pose and encoder values.
    pub fn initialize(&mut self, start_pose: Pose2D, left_ticks: u16, right_ticks: u16) {
        self.accumulated_pose = start_pose;
        self.last_left_ticks = left_ticks;
        self.last_right_ticks = right_ticks;
        self.initialized = true;
    }

    /// Update with new encoder ticks, returns the accumulated pose.
    ///
    /// Handles u16 wrapping correctly.
    pub fn update(&mut self, left_ticks: u16, right_ticks: u16) -> Pose2D {
        if !self.initialized {
            self.last_left_ticks = left_ticks;
            self.last_right_ticks = right_ticks;
            self.initialized = true;
            return self.accumulated_pose;
        }

        // Handle u16 wrapping
        let delta_left = left_ticks.wrapping_sub(self.last_left_ticks) as i16;
        let delta_right = right_ticks.wrapping_sub(self.last_right_ticks) as i16;

        self.last_left_ticks = left_ticks;
        self.last_right_ticks = right_ticks;

        // Convert ticks to meters
        let left_dist = delta_left as f32 / self.ticks_per_meter;
        let right_dist = delta_right as f32 / self.ticks_per_meter;

        // Apply drift
        let left_dist = left_dist * (1.0 + self.config.drift_per_meter);
        let right_dist = right_dist * (1.0 + self.config.drift_per_meter);

        // Differential drive kinematics
        let linear = (left_dist + right_dist) / 2.0;
        let angular = (right_dist - left_dist) / self.wheel_base;

        // Apply angular drift
        let angular = angular * (1.0 + self.config.angular_drift_per_rad);

        // Compute pose delta in current robot frame
        let (dx, dy, dtheta) = if angular.abs() < 1e-6 {
            // Straight line motion
            (linear, 0.0, 0.0)
        } else {
            // Arc motion
            let radius = linear / angular;
            (
                radius * angular.sin(),
                radius * (1.0 - angular.cos()),
                angular,
            )
        };

        // Transform to world frame and accumulate
        let cos_theta = self.accumulated_pose.theta.cos();
        let sin_theta = self.accumulated_pose.theta.sin();

        self.accumulated_pose.x += dx * cos_theta - dy * sin_theta;
        self.accumulated_pose.y += dx * sin_theta + dy * cos_theta;
        self.accumulated_pose.theta += dtheta;

        // Normalize theta to [-PI, PI]
        self.accumulated_pose.theta = normalize_angle(self.accumulated_pose.theta);

        self.accumulated_pose
    }
}

/// Normalize angle to [-PI, PI] range.
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle;
    while a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    }
    while a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_straight_motion() {
        let mut odom = OdometryEstimator::new(0.233, 4464.0, OdometryConfig::clean());
        odom.initialize(Pose2D::identity(), 0, 0);

        // Move forward 1 meter (4464 ticks on each wheel)
        let pose = odom.update(4464, 4464);

        assert!((pose.x - 1.0).abs() < 0.01);
        assert!(pose.y.abs() < 0.01);
        assert!(pose.theta.abs() < 0.01);
    }

    #[test]
    fn test_rotation() {
        let mut odom = OdometryEstimator::new(0.233, 4464.0, OdometryConfig::clean());
        odom.initialize(Pose2D::identity(), 0, 0);

        // Rotate in place (opposite wheel motion)
        // For 90 degree turn: arc_length = wheel_base/2 * PI/2
        let arc_ticks = (0.233 / 2.0 * PI / 2.0 * 4464.0) as u16;
        let pose = odom.update(65535 - arc_ticks + 1, arc_ticks); // Left backward, right forward

        // Should have rotated approximately PI/2
        assert!((pose.theta - PI / 2.0).abs() < 0.1);
    }

    #[test]
    fn test_wrapping() {
        let mut odom = OdometryEstimator::new(0.233, 4464.0, OdometryConfig::clean());
        odom.initialize(Pose2D::identity(), 65530, 65530);

        // Wrap around
        let pose = odom.update(10, 10);

        // Should have moved forward ~16 ticks worth
        let expected_dist = 16.0 / 4464.0;
        assert!((pose.x - expected_dist).abs() < 0.001);
    }
}
