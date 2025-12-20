//! Motion model for pose prediction.
//!
//! This is a utility struct for callers (e.g., dhruva-slam) to use when
//! computing initial pose estimates for ICP when odometry is unavailable
//! or unreliable.
//!
//! # Usage
//!
//! ```rust,ignore
//! use vastu_map::MotionModel;
//!
//! let mut motion = MotionModel::new();
//!
//! // After each successful ICP:
//! motion.update(icp_result.pose, current_time);
//!
//! // For next scan, if odometry unavailable:
//! let predicted = motion.predict(next_time);
//! let result = icp.match_scan(&points, &lines, predicted, &config);
//! ```

use crate::core::{Pose2D, math::angle_diff};

/// Constant-velocity motion model for pose prediction.
///
/// Maintains an exponentially-smoothed velocity estimate based on
/// observed pose changes, and uses this to predict future poses.
#[derive(Clone, Debug)]
pub struct MotionModel {
    /// Last observed pose.
    last_pose: Pose2D,
    /// Smoothed velocity estimate (vx, vy, omega).
    velocity: (f32, f32, f32),
    /// Timestamp of last observation.
    last_time: f64,
    /// Smoothing factor for velocity updates (0-1).
    /// Higher = more smoothing (slower response to changes).
    smoothing: f32,
    /// Whether the model has been initialized with at least one observation.
    initialized: bool,
}

impl MotionModel {
    /// Create a new motion model with default smoothing (0.7).
    pub fn new() -> Self {
        Self {
            last_pose: Pose2D::identity(),
            velocity: (0.0, 0.0, 0.0),
            last_time: 0.0,
            smoothing: 0.7,
            initialized: false,
        }
    }

    /// Create a motion model with custom smoothing factor.
    ///
    /// # Arguments
    /// * `smoothing` - Smoothing factor in [0, 1).
    ///   - 0.0 = no smoothing (velocity tracks instantly)
    ///   - 0.9 = high smoothing (slow response to velocity changes)
    ///     Default is 0.7 for a balance of responsiveness and stability.
    pub fn with_smoothing(smoothing: f32) -> Self {
        Self {
            smoothing: smoothing.clamp(0.0, 0.99),
            ..Self::new()
        }
    }

    /// Predict pose at a given time based on current velocity estimate.
    ///
    /// Returns identity pose if not yet initialized.
    ///
    /// # Arguments
    /// * `current_time` - Time at which to predict (seconds)
    pub fn predict(&self, current_time: f64) -> Pose2D {
        if !self.initialized {
            return Pose2D::identity();
        }

        let dt = (current_time - self.last_time) as f32;
        if dt <= 0.0 {
            return self.last_pose;
        }

        Pose2D::new(
            self.last_pose.x + self.velocity.0 * dt,
            self.last_pose.y + self.velocity.1 * dt,
            self.last_pose.theta + self.velocity.2 * dt,
        )
    }

    /// Update the motion model with a new pose observation.
    ///
    /// This updates the internal velocity estimate using exponential smoothing.
    ///
    /// # Arguments
    /// * `new_pose` - Observed pose (typically from ICP result)
    /// * `current_time` - Time of observation (seconds)
    pub fn update(&mut self, new_pose: Pose2D, current_time: f64) {
        if self.initialized {
            let dt = (current_time - self.last_time) as f32;
            if dt > 0.001 {
                // Compute instantaneous velocity
                let new_vx = (new_pose.x - self.last_pose.x) / dt;
                let new_vy = (new_pose.y - self.last_pose.y) / dt;
                let new_omega = angle_diff(self.last_pose.theta, new_pose.theta) / dt;

                // Apply exponential smoothing
                let s = self.smoothing;
                self.velocity.0 = s * self.velocity.0 + (1.0 - s) * new_vx;
                self.velocity.1 = s * self.velocity.1 + (1.0 - s) * new_vy;
                self.velocity.2 = s * self.velocity.2 + (1.0 - s) * new_omega;
            }
        }

        self.last_pose = new_pose;
        self.last_time = current_time;
        self.initialized = true;
    }

    /// Reset the motion model to uninitialized state.
    pub fn reset(&mut self) {
        *self = Self::with_smoothing(self.smoothing);
    }

    /// Get the current velocity estimate (vx, vy, omega).
    ///
    /// Returns (0, 0, 0) if not initialized.
    pub fn velocity(&self) -> (f32, f32, f32) {
        if self.initialized {
            self.velocity
        } else {
            (0.0, 0.0, 0.0)
        }
    }

    /// Get the linear speed (magnitude of translational velocity).
    pub fn speed(&self) -> f32 {
        let (vx, vy, _) = self.velocity();
        (vx * vx + vy * vy).sqrt()
    }

    /// Get the angular velocity (omega).
    pub fn angular_velocity(&self) -> f32 {
        self.velocity().2
    }

    /// Check if the model has been initialized.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get the last observed pose.
    pub fn last_pose(&self) -> Pose2D {
        self.last_pose
    }

    /// Get the timestamp of the last observation.
    pub fn last_time(&self) -> f64 {
        self.last_time
    }

    /// Set the smoothing factor.
    pub fn set_smoothing(&mut self, smoothing: f32) {
        self.smoothing = smoothing.clamp(0.0, 0.99);
    }
}

impl Default for MotionModel {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_new_model_not_initialized() {
        let model = MotionModel::new();
        assert!(!model.is_initialized());
        assert_eq!(model.velocity(), (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_predict_uninitialized_returns_identity() {
        let model = MotionModel::new();
        let predicted = model.predict(1.0);
        assert_relative_eq!(predicted.x, 0.0);
        assert_relative_eq!(predicted.y, 0.0);
        assert_relative_eq!(predicted.theta, 0.0);
    }

    #[test]
    fn test_first_update_initializes() {
        let mut model = MotionModel::new();
        model.update(Pose2D::new(1.0, 2.0, 0.5), 0.0);

        assert!(model.is_initialized());
        assert_relative_eq!(model.last_pose().x, 1.0);
        assert_relative_eq!(model.last_pose().y, 2.0);
        // First update has no velocity estimate
        assert_eq!(model.velocity(), (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_velocity_estimation() {
        let mut model = MotionModel::with_smoothing(0.0); // No smoothing for predictable test

        // First observation
        model.update(Pose2D::new(0.0, 0.0, 0.0), 0.0);

        // Second observation: moved 1m in x over 1 second
        model.update(Pose2D::new(1.0, 0.0, 0.0), 1.0);

        let (vx, vy, omega) = model.velocity();
        assert_relative_eq!(vx, 1.0, epsilon = 0.01);
        assert_relative_eq!(vy, 0.0, epsilon = 0.01);
        assert_relative_eq!(omega, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_prediction() {
        let mut model = MotionModel::with_smoothing(0.0);

        model.update(Pose2D::new(0.0, 0.0, 0.0), 0.0);
        model.update(Pose2D::new(1.0, 0.0, 0.0), 1.0);

        // Predict 1 second ahead
        let predicted = model.predict(2.0);
        assert_relative_eq!(predicted.x, 2.0, epsilon = 0.01);
        assert_relative_eq!(predicted.y, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_smoothing_effect() {
        let mut model = MotionModel::with_smoothing(0.5);

        model.update(Pose2D::new(0.0, 0.0, 0.0), 0.0);
        model.update(Pose2D::new(1.0, 0.0, 0.0), 1.0); // vx = 1.0

        // With smoothing=0.5: new_vx = 0.5 * 0 + 0.5 * 1.0 = 0.5
        let (vx, _, _) = model.velocity();
        assert_relative_eq!(vx, 0.5, epsilon = 0.01);

        // Another update with same velocity
        model.update(Pose2D::new(2.0, 0.0, 0.0), 2.0); // instantaneous vx = 1.0

        // new_vx = 0.5 * 0.5 + 0.5 * 1.0 = 0.75
        let (vx, _, _) = model.velocity();
        assert_relative_eq!(vx, 0.75, epsilon = 0.01);
    }

    #[test]
    fn test_reset() {
        let mut model = MotionModel::with_smoothing(0.8);
        model.update(Pose2D::new(1.0, 1.0, 1.0), 1.0);
        model.update(Pose2D::new(2.0, 2.0, 2.0), 2.0);

        model.reset();

        assert!(!model.is_initialized());
        assert_eq!(model.velocity(), (0.0, 0.0, 0.0));
        // Smoothing should be preserved
        assert_relative_eq!(model.smoothing, 0.8);
    }

    #[test]
    fn test_speed() {
        let mut model = MotionModel::with_smoothing(0.0);

        model.update(Pose2D::new(0.0, 0.0, 0.0), 0.0);
        model.update(Pose2D::new(3.0, 4.0, 0.0), 1.0);

        // Speed should be sqrt(3^2 + 4^2) = 5
        assert_relative_eq!(model.speed(), 5.0, epsilon = 0.01);
    }

    #[test]
    fn test_angular_velocity() {
        let mut model = MotionModel::with_smoothing(0.0);

        model.update(Pose2D::new(0.0, 0.0, 0.0), 0.0);
        model.update(Pose2D::new(0.0, 0.0, 0.5), 1.0);

        assert_relative_eq!(model.angular_velocity(), 0.5, epsilon = 0.01);
    }
}
