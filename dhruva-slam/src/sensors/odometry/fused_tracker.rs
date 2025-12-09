//! Fused pose tracking for SLAM + odometry integration.
//!
//! Manages dual pose chains: raw odometry and SLAM-corrected poses.
//! Provides high-frequency pose updates that stay consistent with SLAM-built maps.
//!
//! # Problem
//!
//! When SLAM runs at 5Hz but odometry runs at 110Hz, visualizations need
//! high-frequency updates that are consistent with the SLAM-built map.
//! Simply publishing raw odometry causes the robot marker to drift relative to the map.
//!
//! # Solution
//!
//! FusedPoseTracker maintains:
//! - SLAM base pose: The last SLAM-corrected pose
//! - Odometry snapshot: The raw odometry at the time of SLAM update
//! - Current odometry: The current raw odometry pose
//!
//! The fused pose is: `slam_base_pose + (current_odom - odom_snapshot)`
//!
//! This keeps high-frequency updates aligned with the SLAM-built map.
//!
//! # Example
//!
//! ```
//! use dhruva_slam::sensors::odometry::FusedPoseTracker;
//! use dhruva_slam::Pose2D;
//!
//! let mut tracker = FusedPoseTracker::new();
//!
//! // At 110Hz: update with raw odometry
//! tracker.update_odometry_delta(&Pose2D::new(0.1, 0.0, 0.0));
//! let pose = tracker.current_pose(); // Returns fused pose for visualization
//!
//! // At 5Hz: SLAM provides corrected pose
//! let slam_pose = Pose2D::new(0.095, 0.002, 0.01); // Slightly corrected
//! tracker.update_slam(&slam_pose);
//!
//! // Subsequent odometry updates now build on SLAM-corrected pose
//! tracker.update_odometry_delta(&Pose2D::new(0.1, 0.0, 0.0));
//! let fused = tracker.current_pose(); // ~(0.195, 0.002, 0.01)
//! ```

use crate::core::types::{Pose2D, PoseTracker};

/// Tracks both raw odometry and SLAM-corrected poses.
///
/// Provides high-frequency pose estimates that stay consistent with
/// SLAM-built maps by combining SLAM corrections with odometry deltas.
#[derive(Debug, Clone)]
pub struct FusedPoseTracker {
    /// Tracks raw odometry pose.
    odom_tracker: PoseTracker,
    /// SLAM-corrected base pose (updated at SLAM rate, e.g., 5Hz).
    slam_base_pose: Pose2D,
    /// Odometry pose at last SLAM update (for computing delta).
    odom_at_slam: Pose2D,
    /// Whether SLAM has provided at least one correction.
    slam_initialized: bool,
}

impl FusedPoseTracker {
    /// Create a new fused pose tracker.
    pub fn new() -> Self {
        Self {
            odom_tracker: PoseTracker::new(),
            slam_base_pose: Pose2D::identity(),
            odom_at_slam: Pose2D::identity(),
            slam_initialized: false,
        }
    }

    /// Create with an initial pose.
    pub fn with_initial(pose: Pose2D) -> Self {
        Self {
            odom_tracker: PoseTracker::with_initial(pose),
            slam_base_pose: pose,
            odom_at_slam: pose,
            slam_initialized: false,
        }
    }

    /// Update with a pose delta from odometry.
    ///
    /// Call this at high frequency (e.g., 110Hz) with incremental motion.
    pub fn update_odometry_delta(&mut self, delta: &Pose2D) {
        self.odom_tracker.update(delta);
    }

    /// Update with an absolute odometry pose.
    ///
    /// Use this when your odometry source provides absolute poses
    /// rather than deltas (e.g., from ComplementaryFilter).
    pub fn update_odometry(&mut self, odom_pose: &Pose2D) {
        self.odom_tracker.set(*odom_pose);
    }

    /// Update with a SLAM-corrected pose.
    ///
    /// Call this at SLAM rate (e.g., 5Hz) after scan matching.
    /// This anchors future odometry updates to the corrected pose.
    pub fn update_slam(&mut self, slam_pose: &Pose2D) {
        self.slam_base_pose = *slam_pose;
        self.odom_at_slam = self.odom_tracker.pose();
        self.slam_initialized = true;
    }

    /// Get the current fused pose for visualization.
    ///
    /// Returns SLAM-corrected pose plus odometry delta since last SLAM update.
    /// Before SLAM is initialized, returns raw odometry.
    pub fn current_pose(&self) -> Pose2D {
        if !self.slam_initialized {
            return self.odom_tracker.pose();
        }

        // Compute odometry delta since last SLAM update
        let current_odom = self.odom_tracker.pose();
        let odom_delta = self.odom_at_slam.inverse().compose(&current_odom);

        // Apply delta to SLAM-corrected base
        self.slam_base_pose.compose(&odom_delta)
    }

    /// Get the odometry delta since last SLAM update.
    ///
    /// Useful for providing initial guess to scan matcher.
    pub fn odom_delta_since_slam(&self) -> Pose2D {
        self.odom_at_slam
            .inverse()
            .compose(&self.odom_tracker.pose())
    }

    /// Get the raw odometry pose (without SLAM correction).
    pub fn raw_odom_pose(&self) -> Pose2D {
        self.odom_tracker.pose()
    }

    /// Get the last SLAM-corrected base pose.
    pub fn slam_base_pose(&self) -> Pose2D {
        self.slam_base_pose
    }

    /// Check if SLAM has been initialized (at least one correction received).
    pub fn is_slam_initialized(&self) -> bool {
        self.slam_initialized
    }

    /// Reset tracker to identity.
    pub fn reset(&mut self) {
        self.odom_tracker.reset();
        self.slam_base_pose = Pose2D::identity();
        self.odom_at_slam = Pose2D::identity();
        self.slam_initialized = false;
    }

    /// Reset tracker to a specific pose.
    pub fn reset_to(&mut self, pose: Pose2D) {
        self.odom_tracker.reset_to(pose);
        self.slam_base_pose = pose;
        self.odom_at_slam = pose;
        self.slam_initialized = false;
    }

    /// Get mutable access to the underlying odometry tracker.
    ///
    /// Useful for taking snapshots or other advanced operations.
    pub fn odom_tracker_mut(&mut self) -> &mut PoseTracker {
        &mut self.odom_tracker
    }

    /// Get reference to the underlying odometry tracker.
    pub fn odom_tracker(&self) -> &PoseTracker {
        &self.odom_tracker
    }
}

impl Default for FusedPoseTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::PI;

    #[test]
    fn test_fused_pose_tracker_new() {
        let tracker = FusedPoseTracker::new();
        let pose = tracker.current_pose();
        assert_relative_eq!(pose.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 0.0, epsilon = 1e-6);
        assert!(!tracker.is_slam_initialized());
    }

    #[test]
    fn test_fused_pose_tracker_with_initial() {
        let initial = Pose2D::new(1.0, 2.0, 0.5);
        let tracker = FusedPoseTracker::with_initial(initial);
        let pose = tracker.current_pose();
        assert_relative_eq!(pose.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 2.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 0.5, epsilon = 1e-6);
    }

    #[test]
    fn test_fused_pose_tracker_odometry_only() {
        let mut tracker = FusedPoseTracker::new();

        // Before SLAM init, current_pose returns raw odometry
        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));

        let pose = tracker.current_pose();
        assert_relative_eq!(pose.x, 2.0, epsilon = 1e-5);
        assert!(!tracker.is_slam_initialized());
    }

    #[test]
    fn test_fused_pose_tracker_slam_correction() {
        let mut tracker = FusedPoseTracker::new();

        // Move with odometry
        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));

        // SLAM provides corrected pose (slightly different)
        tracker.update_slam(&Pose2D::new(0.95, 0.05, 0.01));
        assert!(tracker.is_slam_initialized());

        // Current pose should be SLAM pose (no odom delta yet)
        let pose = tracker.current_pose();
        assert_relative_eq!(pose.x, 0.95, epsilon = 1e-5);
        assert_relative_eq!(pose.y, 0.05, epsilon = 1e-5);
        assert_relative_eq!(pose.theta, 0.01, epsilon = 1e-5);
    }

    #[test]
    fn test_fused_pose_tracker_fusion() {
        let mut tracker = FusedPoseTracker::new();

        // Move with odometry
        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));

        // SLAM provides correction
        tracker.update_slam(&Pose2D::new(0.95, 0.05, 0.0));

        // More odometry movement
        tracker.update_odometry_delta(&Pose2D::new(0.5, 0.0, 0.0));

        // Fused pose should be SLAM base + odom delta
        let pose = tracker.current_pose();
        assert_relative_eq!(pose.x, 0.95 + 0.5, epsilon = 1e-5);
        assert_relative_eq!(pose.y, 0.05, epsilon = 1e-5);
    }

    #[test]
    fn test_fused_pose_tracker_update_odometry_absolute() {
        let mut tracker = FusedPoseTracker::new();

        // Use absolute pose updates (like from ComplementaryFilter)
        tracker.update_odometry(&Pose2D::new(1.0, 0.0, 0.0));

        // SLAM correction
        tracker.update_slam(&Pose2D::new(0.95, 0.05, 0.0));

        // More odometry
        tracker.update_odometry(&Pose2D::new(1.5, 0.0, 0.0));

        // Delta = 1.5 - 1.0 = 0.5
        // Fused = 0.95 + 0.5 = 1.45
        let pose = tracker.current_pose();
        assert_relative_eq!(pose.x, 1.45, epsilon = 1e-5);
        assert_relative_eq!(pose.y, 0.05, epsilon = 1e-5);
    }

    #[test]
    fn test_fused_pose_tracker_odom_delta_since_slam() {
        let mut tracker = FusedPoseTracker::new();

        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.update_slam(&Pose2D::new(1.0, 0.0, 0.0));

        tracker.update_odometry_delta(&Pose2D::new(0.3, 0.1, 0.05));

        let delta = tracker.odom_delta_since_slam();
        assert_relative_eq!(delta.x, 0.3, epsilon = 1e-5);
        assert_relative_eq!(delta.y, 0.1, epsilon = 1e-5);
        assert_relative_eq!(delta.theta, 0.05, epsilon = 1e-5);
    }

    #[test]
    fn test_fused_pose_tracker_raw_vs_fused() {
        let mut tracker = FusedPoseTracker::new();

        // Move 1m forward
        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));

        // SLAM says we're actually at (0.9, 0.1)
        tracker.update_slam(&Pose2D::new(0.9, 0.1, 0.0));

        // Move another 0.5m
        tracker.update_odometry_delta(&Pose2D::new(0.5, 0.0, 0.0));

        // Raw odom: 1.5, 0.0
        let raw = tracker.raw_odom_pose();
        assert_relative_eq!(raw.x, 1.5, epsilon = 1e-5);
        assert_relative_eq!(raw.y, 0.0, epsilon = 1e-5);

        // Fused: 0.9 + 0.5 = 1.4, y = 0.1
        let fused = tracker.current_pose();
        assert_relative_eq!(fused.x, 1.4, epsilon = 1e-5);
        assert_relative_eq!(fused.y, 0.1, epsilon = 1e-5);
    }

    #[test]
    fn test_fused_pose_tracker_with_rotation() {
        let mut tracker = FusedPoseTracker::new();

        // Move forward then rotate
        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.update_odometry_delta(&Pose2D::new(0.0, 0.0, PI / 2.0));

        // SLAM provides same pose
        tracker.update_slam(&tracker.raw_odom_pose());

        // Move forward in rotated frame
        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));

        // The delta should be in local frame, applied to SLAM pose
        let fused = tracker.current_pose();
        // After 90 rotation, forward motion should be in +Y direction
        assert_relative_eq!(fused.x, 1.0, epsilon = 1e-4);
        assert_relative_eq!(fused.y, 1.0, epsilon = 1e-4);
    }

    #[test]
    fn test_fused_pose_tracker_multiple_slam_updates() {
        let mut tracker = FusedPoseTracker::new();

        // First SLAM cycle
        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.update_slam(&Pose2D::new(0.95, 0.0, 0.0));

        // Second SLAM cycle
        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.update_slam(&Pose2D::new(1.9, 0.02, 0.0));

        // Third cycle - odom only
        tracker.update_odometry_delta(&Pose2D::new(0.5, 0.0, 0.0));

        let fused = tracker.current_pose();
        // Should be slam_base (1.9, 0.02) + delta (0.5, 0)
        assert_relative_eq!(fused.x, 2.4, epsilon = 1e-5);
        assert_relative_eq!(fused.y, 0.02, epsilon = 1e-5);
    }

    #[test]
    fn test_fused_pose_tracker_reset() {
        let mut tracker = FusedPoseTracker::new();

        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.update_slam(&Pose2D::new(1.0, 0.0, 0.0));
        assert!(tracker.is_slam_initialized());

        tracker.reset();

        assert!(!tracker.is_slam_initialized());
        let pose = tracker.current_pose();
        assert_relative_eq!(pose.x, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_fused_pose_tracker_reset_to() {
        let mut tracker = FusedPoseTracker::new();

        tracker.update_odometry_delta(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.update_slam(&Pose2D::new(1.0, 0.0, 0.0));

        tracker.reset_to(Pose2D::new(5.0, 5.0, 1.0));

        assert!(!tracker.is_slam_initialized());
        let pose = tracker.current_pose();
        assert_relative_eq!(pose.x, 5.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 5.0, epsilon = 1e-6);
    }
}
