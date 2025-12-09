//! Pose accumulation and delta tracking.
//!
//! Provides utilities for tracking robot pose over time and computing
//! deltas between reference points (e.g., between scan timestamps).

use super::Pose2D;

/// Tracks pose accumulation and provides delta computation.
///
/// Useful for tracking odometry pose and computing deltas
/// between scan timestamps for scan matching.
///
/// # Example
///
/// ```
/// use dhruva_slam::core::types::PoseTracker;
/// use dhruva_slam::Pose2D;
///
/// let mut tracker = PoseTracker::new();
///
/// // Accumulate odometry deltas
/// tracker.update(&Pose2D::new(0.1, 0.0, 0.0));
/// tracker.update(&Pose2D::new(0.1, 0.0, 0.05));
///
/// // Take snapshot when processing a scan
/// tracker.take_snapshot();
///
/// // Continue accumulating
/// tracker.update(&Pose2D::new(0.1, 0.0, 0.0));
///
/// // Get delta since last scan for scan matching
/// let delta = tracker.delta_since_snapshot();
/// ```
#[derive(Debug, Clone)]
pub struct PoseTracker {
    /// Current accumulated pose.
    current: Pose2D,
    /// Pose at last snapshot (for delta computation).
    snapshot: Pose2D,
}

impl PoseTracker {
    /// Create a new pose tracker starting at identity.
    pub fn new() -> Self {
        Self {
            current: Pose2D::identity(),
            snapshot: Pose2D::identity(),
        }
    }

    /// Create starting at a specific pose.
    pub fn with_initial(pose: Pose2D) -> Self {
        Self {
            current: pose,
            snapshot: pose,
        }
    }

    /// Update with a pose delta (compose onto current).
    ///
    /// This is used when you have incremental motion updates.
    pub fn update(&mut self, delta: &Pose2D) {
        self.current = self.current.compose(delta);
    }

    /// Set absolute pose (e.g., from odometry filter that tracks global pose).
    ///
    /// This is used when the odometry source provides absolute poses
    /// rather than deltas.
    pub fn set(&mut self, pose: Pose2D) {
        self.current = pose;
    }

    /// Get current accumulated pose.
    pub fn pose(&self) -> Pose2D {
        self.current
    }

    /// Get the snapshot pose.
    pub fn snapshot_pose(&self) -> Pose2D {
        self.snapshot
    }

    /// Compute delta since last snapshot.
    ///
    /// Returns the transform from snapshot pose to current pose.
    /// This is typically used to get the odometry delta between scans.
    pub fn delta_since_snapshot(&self) -> Pose2D {
        self.snapshot.inverse().compose(&self.current)
    }

    /// Take a snapshot (for next delta computation).
    ///
    /// Call this after processing a scan to mark the reference point
    /// for the next delta computation.
    pub fn take_snapshot(&mut self) {
        self.snapshot = self.current;
    }

    /// Reset both current and snapshot to identity.
    pub fn reset(&mut self) {
        self.current = Pose2D::identity();
        self.snapshot = Pose2D::identity();
    }

    /// Reset to a specific pose (sets both current and snapshot).
    pub fn reset_to(&mut self, pose: Pose2D) {
        self.current = pose;
        self.snapshot = pose;
    }
}

impl Default for PoseTracker {
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
    fn test_pose_tracker_initial_state() {
        let tracker = PoseTracker::new();
        let pose = tracker.pose();
        assert_relative_eq!(pose.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_tracker_with_initial() {
        let initial = Pose2D::new(1.0, 2.0, 0.5);
        let tracker = PoseTracker::with_initial(initial);
        let pose = tracker.pose();
        assert_relative_eq!(pose.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 2.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 0.5, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_tracker_update() {
        let mut tracker = PoseTracker::new();
        tracker.update(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.update(&Pose2D::new(1.0, 0.0, 0.0));

        let pose = tracker.pose();
        assert_relative_eq!(pose.x, 2.0, epsilon = 1e-5);
        assert_relative_eq!(pose.y, 0.0, epsilon = 1e-5);
    }

    #[test]
    fn test_pose_tracker_set() {
        let mut tracker = PoseTracker::new();
        tracker.update(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.set(Pose2D::new(5.0, 3.0, 1.0));

        let pose = tracker.pose();
        assert_relative_eq!(pose.x, 5.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 3.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_tracker_delta_since_snapshot() {
        let mut tracker = PoseTracker::new();
        tracker.update(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.take_snapshot();
        tracker.update(&Pose2D::new(0.5, 0.3, 0.1));

        let delta = tracker.delta_since_snapshot();
        assert_relative_eq!(delta.x, 0.5, epsilon = 1e-5);
        assert_relative_eq!(delta.y, 0.3, epsilon = 1e-5);
        assert_relative_eq!(delta.theta, 0.1, epsilon = 1e-5);
    }

    #[test]
    fn test_pose_tracker_delta_with_rotation() {
        let mut tracker = PoseTracker::new();

        // Move forward, then rotate 90 degrees
        tracker.update(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.update(&Pose2D::new(0.0, 0.0, PI / 2.0));
        tracker.take_snapshot();

        // Move forward in the new direction (which is now +Y in world frame)
        tracker.update(&Pose2D::new(1.0, 0.0, 0.0));

        let delta = tracker.delta_since_snapshot();
        // Delta should be (1, 0, 0) in local frame
        assert_relative_eq!(delta.x, 1.0, epsilon = 1e-5);
        assert_relative_eq!(delta.y, 0.0, epsilon = 1e-5);
    }

    #[test]
    fn test_pose_tracker_reset() {
        let mut tracker = PoseTracker::new();
        tracker.update(&Pose2D::new(5.0, 3.0, 1.0));
        tracker.take_snapshot();
        tracker.reset();

        let pose = tracker.pose();
        assert_relative_eq!(pose.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 0.0, epsilon = 1e-6);

        let snapshot = tracker.snapshot_pose();
        assert_relative_eq!(snapshot.x, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_tracker_reset_to() {
        let mut tracker = PoseTracker::new();
        tracker.update(&Pose2D::new(5.0, 3.0, 1.0));
        tracker.reset_to(Pose2D::new(10.0, 20.0, 0.5));

        let pose = tracker.pose();
        assert_relative_eq!(pose.x, 10.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 20.0, epsilon = 1e-6);

        // Delta should be zero after reset_to
        let delta = tracker.delta_since_snapshot();
        assert_relative_eq!(delta.x, 0.0, epsilon = 1e-5);
        assert_relative_eq!(delta.y, 0.0, epsilon = 1e-5);
    }

    #[test]
    fn test_pose_tracker_multiple_snapshots() {
        let mut tracker = PoseTracker::new();

        // First segment
        tracker.update(&Pose2D::new(1.0, 0.0, 0.0));
        tracker.take_snapshot();

        // Second segment
        tracker.update(&Pose2D::new(2.0, 0.0, 0.0));
        let delta1 = tracker.delta_since_snapshot();
        assert_relative_eq!(delta1.x, 2.0, epsilon = 1e-5);

        tracker.take_snapshot();

        // Third segment
        tracker.update(&Pose2D::new(0.5, 0.0, 0.0));
        let delta2 = tracker.delta_since_snapshot();
        assert_relative_eq!(delta2.x, 0.5, epsilon = 1e-5);
    }
}
