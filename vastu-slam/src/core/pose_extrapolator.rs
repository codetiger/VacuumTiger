//! Pose extrapolation with odometry and IMU fusion.
//!
//! This module implements Cartographer-style pose extrapolation, combining
//! wheel odometry (primary) with IMU data (secondary) for improved pose
//! estimation.
//!
//! # Algorithm
//!
//! The PoseExtrapolator maintains:
//!
//! 1. **Odometry Queue**: Recent odometry measurements for velocity estimation
//! 2. **Pose Queue**: Recent scan-matched poses for drift correction
//! 3. **IMU Tracker**: Gyroscope-based orientation tracking
//!
//! # Priority Scheme
//!
//! - **Translation**: 100% from wheel odometry (most accurate for ground robots)
//! - **Rotation**: Weighted blend of odometry and IMU
//!   - Default: 70% odometry + 30% IMU
//!   - Configurable via `imu_rotation_weight`
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_slam::core::{PoseExtrapolator, PoseExtrapolatorConfig, ImuMeasurement, Pose2D};
//!
//! let config = PoseExtrapolatorConfig::default();
//! let mut extrapolator = PoseExtrapolator::new(config);
//!
//! // Feed sensor data at high rate
//! extrapolator.add_odometry(odom_delta, timestamp_us);
//! extrapolator.add_imu(&imu);
//!
//! // Extrapolate pose when needed (e.g., at lidar scan time)
//! let predicted_pose = extrapolator.extrapolate_pose(scan_timestamp_us);
//!
//! // After scan matching, add corrected pose
//! extrapolator.add_pose(matched_pose, timestamp_us);
//! ```

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use super::imu_tracker::{ImuTracker, ImuTrackerConfig};
use super::{ImuMeasurement, Pose2D};

/// Configuration for the pose extrapolator.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseExtrapolatorConfig {
    /// Duration to keep poses in the queue for velocity estimation (seconds).
    ///
    /// Longer durations give smoother velocity estimates but may be less
    /// responsive to changes.
    ///
    /// Default: 0.5 seconds
    #[serde(default = "default_pose_queue_duration")]
    pub pose_queue_duration: f32,

    /// Duration to keep odometry in the queue (seconds).
    ///
    /// Default: 0.5 seconds
    #[serde(default = "default_odom_queue_duration")]
    pub odom_queue_duration: f32,

    /// IMU gravity time constant (seconds).
    ///
    /// Passed to the internal ImuTracker.
    ///
    /// Default: 10.0 seconds
    #[serde(default = "default_imu_gravity_time_constant")]
    pub imu_gravity_time_constant: f32,

    /// Weight for IMU rotation vs odometry rotation [0.0 - 1.0].
    ///
    /// - 0.0: Use only odometry for rotation
    /// - 1.0: Use only IMU for rotation
    /// - 0.3: 70% odometry + 30% IMU (default, odometry primary)
    ///
    /// Default: 0.3
    #[serde(default = "default_imu_rotation_weight")]
    pub imu_rotation_weight: f32,
}

fn default_pose_queue_duration() -> f32 {
    0.5
}
fn default_odom_queue_duration() -> f32 {
    0.5
}
fn default_imu_gravity_time_constant() -> f32 {
    10.0
}
fn default_imu_rotation_weight() -> f32 {
    0.3
}

impl Default for PoseExtrapolatorConfig {
    fn default() -> Self {
        Self {
            pose_queue_duration: 0.5,
            odom_queue_duration: 0.5,
            imu_gravity_time_constant: 10.0,
            imu_rotation_weight: 0.3,
        }
    }
}

/// A pose with timestamp.
#[derive(Clone, Copy, Debug)]
pub struct TimedPose {
    /// The pose in world frame.
    pub pose: Pose2D,
    /// Timestamp in microseconds when this pose was recorded.
    pub timestamp_us: u64,
}

/// An odometry delta with timestamp.
#[derive(Clone, Copy, Debug)]
pub struct TimedOdometry {
    /// Pose delta in robot frame (dx forward, dy left, dtheta CCW)
    pub delta: Pose2D,
    /// Timestamp when this odometry was measured
    pub timestamp_us: u64,
}

/// Pose extrapolator with odometry and IMU fusion.
///
/// Combines wheel odometry (primary for translation) with IMU
/// (secondary for rotation) using Cartographer-style algorithms.
#[derive(Clone, Debug)]
pub struct PoseExtrapolator {
    /// Configuration
    config: PoseExtrapolatorConfig,

    /// IMU tracker for orientation
    imu_tracker: ImuTracker,

    /// Recent poses from scan matching (for velocity estimation)
    pose_queue: VecDeque<TimedPose>,

    /// Recent odometry deltas
    odom_queue: VecDeque<TimedOdometry>,

    /// Current pose estimate
    current_pose: Pose2D,

    /// Last pose timestamp
    last_pose_timestamp_us: Option<u64>,

    /// Cumulative odometry since last pose update
    cumulative_odom: Pose2D,

    /// Last odometry timestamp
    last_odom_timestamp_us: Option<u64>,

    /// Estimated linear velocity (m/s) in world frame [vx, vy]
    linear_velocity: [f32; 2],

    /// Estimated angular velocity (rad/s)
    angular_velocity: f32,

    /// IMU yaw at last pose update (for relative tracking)
    imu_yaw_at_last_pose: f32,
}

impl PoseExtrapolator {
    /// Create a new pose extrapolator with the given configuration.
    pub fn new(config: PoseExtrapolatorConfig) -> Self {
        let imu_config = ImuTrackerConfig {
            gravity_time_constant: config.imu_gravity_time_constant,
            initial_gravity: [0.0, 0.0, 9.81],
        };

        Self {
            config,
            imu_tracker: ImuTracker::new(imu_config),
            pose_queue: VecDeque::new(),
            odom_queue: VecDeque::new(),
            current_pose: Pose2D::default(),
            last_pose_timestamp_us: None,
            cumulative_odom: Pose2D::default(),
            last_odom_timestamp_us: None,
            linear_velocity: [0.0, 0.0],
            angular_velocity: 0.0,
            imu_yaw_at_last_pose: 0.0,
        }
    }

    /// Add an IMU measurement (secondary for rotation).
    pub fn add_imu(&mut self, imu: &ImuMeasurement) {
        self.imu_tracker.add_imu(imu);
    }

    /// Add an odometry delta (primary for translation).
    ///
    /// The delta should be in robot frame (dx forward, dy left, dtheta CCW).
    pub fn add_odometry(&mut self, delta: Pose2D, timestamp_us: u64) {
        // Store in queue
        self.odom_queue.push_back(TimedOdometry {
            delta,
            timestamp_us,
        });

        // Trim old entries
        let cutoff_us =
            timestamp_us.saturating_sub((self.config.odom_queue_duration * 1_000_000.0) as u64);
        while self
            .odom_queue
            .front()
            .is_some_and(|o| o.timestamp_us < cutoff_us)
        {
            self.odom_queue.pop_front();
        }

        // Accumulate odometry
        self.cumulative_odom = self.cumulative_odom.compose(&delta);
        self.last_odom_timestamp_us = Some(timestamp_us);

        // Update velocity estimate from odometry
        self.update_velocity_from_odometry();
    }

    /// Add a pose from scan matching (for velocity estimation and drift correction).
    ///
    /// Call this after successful scan matching to update the extrapolator
    /// with the corrected pose.
    pub fn add_pose(&mut self, pose: Pose2D, timestamp_us: u64) {
        // Store in queue
        self.pose_queue.push_back(TimedPose { pose, timestamp_us });

        // Trim old entries
        let cutoff_us =
            timestamp_us.saturating_sub((self.config.pose_queue_duration * 1_000_000.0) as u64);
        while self
            .pose_queue
            .front()
            .is_some_and(|p| p.timestamp_us < cutoff_us)
        {
            self.pose_queue.pop_front();
        }

        // Update current pose
        self.current_pose = pose;
        self.last_pose_timestamp_us = Some(timestamp_us);

        // Reset cumulative odometry
        self.cumulative_odom = Pose2D::default();

        // Record IMU yaw at this pose
        self.imu_yaw_at_last_pose = self.imu_tracker.yaw();

        // Update velocity estimate from pose history
        self.update_velocity_from_poses();
    }

    /// Extrapolate the pose to the given timestamp.
    ///
    /// Uses odometry for translation and a weighted blend of odometry
    /// and IMU for rotation.
    pub fn extrapolate_pose(&self, timestamp_us: u64) -> Pose2D {
        // Start with current pose
        let mut pose = self.current_pose;

        // Apply cumulative odometry (translation from odometry, rotation blended)
        let odom_theta = self.cumulative_odom.theta;

        // Get IMU rotation delta since last pose
        let imu_theta_delta = self.imu_tracker.yaw() - self.imu_yaw_at_last_pose;

        // Blend odometry and IMU rotation
        let blended_theta = (1.0 - self.config.imu_rotation_weight) * odom_theta
            + self.config.imu_rotation_weight * imu_theta_delta;

        // Create blended delta
        let blended_delta = Pose2D::new(
            self.cumulative_odom.x,
            self.cumulative_odom.y,
            blended_theta,
        );

        // Apply delta to pose
        pose = pose.compose(&blended_delta);

        // Extrapolate beyond last odometry using velocity
        if let Some(last_odom_ts) = self.last_odom_timestamp_us
            && timestamp_us > last_odom_ts
        {
            let dt = (timestamp_us - last_odom_ts) as f32 / 1_000_000.0;
            if dt < 1.0 {
                // Reasonable extrapolation window
                let dx = self.linear_velocity[0] * dt;
                let dy = self.linear_velocity[1] * dt;
                let dtheta = self.angular_velocity * dt;
                pose = pose.compose(&Pose2D::new(dx, dy, dtheta));
            }
        }

        pose
    }

    /// Get the current pose estimate (without extrapolation).
    #[inline]
    pub fn current_pose(&self) -> Pose2D {
        self.current_pose
    }

    /// Get the estimated linear velocity [vx, vy] in m/s (world frame).
    #[inline]
    pub fn linear_velocity(&self) -> [f32; 2] {
        self.linear_velocity
    }

    /// Get the estimated angular velocity in rad/s.
    #[inline]
    pub fn angular_velocity(&self) -> f32 {
        self.angular_velocity
    }

    /// Get a reference to the internal IMU tracker.
    #[inline]
    pub fn imu_tracker(&self) -> &ImuTracker {
        &self.imu_tracker
    }

    /// Get a mutable reference to the internal IMU tracker.
    #[inline]
    pub fn imu_tracker_mut(&mut self) -> &mut ImuTracker {
        &mut self.imu_tracker
    }

    /// Set the initial pose.
    ///
    /// Call this before starting to set the initial robot position.
    pub fn set_initial_pose(&mut self, pose: Pose2D, timestamp_us: u64) {
        self.current_pose = pose;
        self.last_pose_timestamp_us = Some(timestamp_us);
        self.cumulative_odom = Pose2D::default();
        self.imu_yaw_at_last_pose = self.imu_tracker.yaw();
        self.pose_queue.clear();
        self.odom_queue.clear();
    }

    /// Reset the extrapolator to initial state.
    pub fn reset(&mut self) {
        self.imu_tracker.reset();
        self.pose_queue.clear();
        self.odom_queue.clear();
        self.current_pose = Pose2D::default();
        self.last_pose_timestamp_us = None;
        self.cumulative_odom = Pose2D::default();
        self.last_odom_timestamp_us = None;
        self.linear_velocity = [0.0, 0.0];
        self.angular_velocity = 0.0;
        self.imu_yaw_at_last_pose = 0.0;
    }

    /// Update velocity estimate from odometry queue.
    fn update_velocity_from_odometry(&mut self) {
        if self.odom_queue.len() < 2 {
            return;
        }

        let first = self.odom_queue.front().unwrap();
        let last = self.odom_queue.back().unwrap();

        let dt = (last.timestamp_us.saturating_sub(first.timestamp_us)) as f32 / 1_000_000.0;
        if dt < 0.01 {
            return; // Not enough time span
        }

        // Sum all deltas in the queue
        let mut total_delta = Pose2D::default();
        for odom in &self.odom_queue {
            total_delta = total_delta.compose(&odom.delta);
        }

        // Compute velocity (in robot frame, then rotate to world frame)
        let vx_robot = total_delta.x / dt;
        let vy_robot = total_delta.y / dt;
        self.angular_velocity = total_delta.theta / dt;

        // Rotate to world frame using current heading
        let cos_theta = self.current_pose.theta.cos();
        let sin_theta = self.current_pose.theta.sin();
        self.linear_velocity[0] = vx_robot * cos_theta - vy_robot * sin_theta;
        self.linear_velocity[1] = vx_robot * sin_theta + vy_robot * cos_theta;
    }

    /// Update velocity estimate from pose queue.
    fn update_velocity_from_poses(&mut self) {
        if self.pose_queue.len() < 2 {
            return;
        }

        let first = self.pose_queue.front().unwrap();
        let last = self.pose_queue.back().unwrap();

        let dt = (last.timestamp_us.saturating_sub(first.timestamp_us)) as f32 / 1_000_000.0;
        if dt < 0.01 {
            return; // Not enough time span
        }

        // Compute velocity from pose difference
        let dx = last.pose.x - first.pose.x;
        let dy = last.pose.y - first.pose.y;
        let dtheta = normalize_angle(last.pose.theta - first.pose.theta);

        self.linear_velocity[0] = dx / dt;
        self.linear_velocity[1] = dy / dt;
        self.angular_velocity = dtheta / dt;
    }
}

/// Normalize angle to [-PI, PI] range.
#[inline]
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

    #[test]
    fn test_pose_extrapolator_creation() {
        let config = PoseExtrapolatorConfig::default();
        let extrapolator = PoseExtrapolator::new(config);

        assert_eq!(extrapolator.current_pose(), Pose2D::default());
        assert_eq!(extrapolator.linear_velocity(), [0.0, 0.0]);
        assert_eq!(extrapolator.angular_velocity(), 0.0);
    }

    #[test]
    fn test_set_initial_pose() {
        let mut extrapolator = PoseExtrapolator::new(PoseExtrapolatorConfig::default());

        let initial = Pose2D::new(1.0, 2.0, 0.5);
        extrapolator.set_initial_pose(initial, 1000000);

        assert_eq!(extrapolator.current_pose().x, 1.0);
        assert_eq!(extrapolator.current_pose().y, 2.0);
        assert!((extrapolator.current_pose().theta - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_odometry_accumulation() {
        let mut extrapolator = PoseExtrapolator::new(PoseExtrapolatorConfig::default());

        extrapolator.set_initial_pose(Pose2D::default(), 0);

        // Add odometry: move forward 1m
        extrapolator.add_odometry(Pose2D::new(0.5, 0.0, 0.0), 100000);
        extrapolator.add_odometry(Pose2D::new(0.5, 0.0, 0.0), 200000);

        let predicted = extrapolator.extrapolate_pose(200000);

        assert!((predicted.x - 1.0).abs() < 0.1, "x = {}", predicted.x);
        assert!((predicted.y - 0.0).abs() < 0.1, "y = {}", predicted.y);
    }

    #[test]
    fn test_pose_update() {
        let mut extrapolator = PoseExtrapolator::new(PoseExtrapolatorConfig::default());

        extrapolator.set_initial_pose(Pose2D::default(), 0);

        // Add some odometry
        extrapolator.add_odometry(Pose2D::new(1.0, 0.0, 0.1), 100000);

        // Add corrected pose from scan matching
        let matched_pose = Pose2D::new(0.95, 0.05, 0.08); // Slightly different
        extrapolator.add_pose(matched_pose, 100000);

        // Current pose should be the matched pose
        assert_eq!(extrapolator.current_pose().x, 0.95);
        assert_eq!(extrapolator.current_pose().y, 0.05);
    }

    #[test]
    fn test_imu_rotation_blending() {
        let config = PoseExtrapolatorConfig {
            imu_rotation_weight: 0.5, // 50% IMU, 50% odometry
            ..Default::default()
        };
        let mut extrapolator = PoseExtrapolator::new(config);

        extrapolator.set_initial_pose(Pose2D::default(), 0);

        // Add IMU readings with rotation
        for i in 0..10 {
            let imu = ImuMeasurement::new(
                i * 10000, // 10ms intervals
                [0.0, 0.0, 9.81],
                [0.0, 0.0, 1.0], // 1 rad/s yaw
            );
            extrapolator.add_imu(&imu);
        }

        // Add odometry with different rotation
        extrapolator.add_odometry(Pose2D::new(0.1, 0.0, 0.2), 100000); // 0.2 rad from odom

        let predicted = extrapolator.extrapolate_pose(100000);

        // IMU would give ~0.09 rad (9 samples * 0.01s * 1 rad/s)
        // Odom gives 0.2 rad
        // Blended: 0.5 * 0.2 + 0.5 * 0.09 ≈ 0.145
        // This is approximate due to timing
        assert!(predicted.theta.abs() > 0.05, "theta = {}", predicted.theta);
        assert!(predicted.theta.abs() < 0.3, "theta = {}", predicted.theta);
    }

    #[test]
    fn test_velocity_estimation() {
        let mut extrapolator = PoseExtrapolator::new(PoseExtrapolatorConfig::default());

        extrapolator.set_initial_pose(Pose2D::default(), 0);

        // Add several odometry samples (moving at 1 m/s forward)
        for i in 1..=10 {
            // 0.1m every 100ms = 1 m/s
            extrapolator.add_odometry(Pose2D::new(0.1, 0.0, 0.0), i * 100000);
        }

        let vel = extrapolator.linear_velocity();
        // Should be approximately 1 m/s in x (allowing some tolerance for edge effects)
        assert!(vel[0] > 0.8, "vx = {}", vel[0]);
        assert!(vel[0] < 1.5, "vx = {}", vel[0]);
        assert!(vel[1].abs() < 0.1, "vy = {}", vel[1]);
    }

    #[test]
    fn test_reset() {
        let mut extrapolator = PoseExtrapolator::new(PoseExtrapolatorConfig::default());

        extrapolator.set_initial_pose(Pose2D::new(5.0, 5.0, 1.0), 1000000);
        extrapolator.add_odometry(Pose2D::new(1.0, 0.0, 0.1), 2000000);

        extrapolator.reset();

        assert_eq!(extrapolator.current_pose(), Pose2D::default());
        assert_eq!(extrapolator.linear_velocity(), [0.0, 0.0]);
    }
}
