//! Pose accumulation and delta tracking.
//!
//! Provides utilities for tracking robot pose over time.

use super::Pose2D;

/// Tracks pose accumulation.
#[derive(Debug, Clone)]
pub struct PoseTracker {
    /// Current accumulated pose.
    current: Pose2D,
    /// Pose at last snapshot.
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

    /// Update with a pose delta (compose onto current).
    pub fn update(&mut self, delta: &Pose2D) {
        self.current = self.current.compose(delta);
    }

    /// Set absolute pose.
    pub fn set(&mut self, pose: Pose2D) {
        self.current = pose;
    }

    /// Get current accumulated pose.
    pub fn pose(&self) -> Pose2D {
        self.current
    }

    /// Take a snapshot (for next delta computation).
    pub fn take_snapshot(&mut self) {
        self.snapshot = self.current;
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

    #[test]
    fn test_pose_tracker_initial_state() {
        let tracker = PoseTracker::new();
        let pose = tracker.pose();
        assert_relative_eq!(pose.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_pose_tracker_set() {
        let mut tracker = PoseTracker::new();
        tracker.set(Pose2D::new(5.0, 3.0, 1.0));

        let pose = tracker.pose();
        assert_relative_eq!(pose.x, 5.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 3.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 1.0, epsilon = 1e-6);
    }
}
