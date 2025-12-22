//! Trait definitions for loop closure detection.

use crate::core::{Point2D, Pose2D};
use crate::features::{Corner2D, Line2D};

use super::detector::LoopClosureDetector;
use super::types::LoopClosure;

/// Trait for loop closure detection algorithms.
///
/// Implementations detect when the robot revisits a previously mapped area,
/// providing constraints for pose graph optimization.
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::loop_closure::{LoopDetector, LoopClosureDetector, LoopClosureConfig};
/// use vastu_map::core::Pose2D;
///
/// let mut detector = LoopClosureDetector::new(LoopClosureConfig::default());
///
/// if let Some(closure) = detector.detect(pose, &lines, &corners, &points) {
///     println!("Loop detected with confidence {}", closure.confidence);
/// }
/// ```
pub trait LoopDetector: Send + Sync {
    /// Process an observation and detect potential loop closures.
    ///
    /// # Arguments
    /// * `pose` - Current robot pose in world frame
    /// * `lines` - Extracted lines in world frame
    /// * `corners` - Extracted corners in world frame
    /// * `points` - Scan points in robot frame (for ICP verification)
    ///
    /// # Returns
    /// A loop closure if one is detected, None otherwise.
    fn detect(
        &mut self,
        pose: Pose2D,
        lines: &[Line2D],
        corners: &[Corner2D],
        points: &[Point2D],
    ) -> Option<LoopClosure>;

    /// Clear all stored state (keyframes, history, etc.).
    fn clear(&mut self);

    /// Check if loop detection is currently possible.
    fn can_detect(&self) -> bool;

    /// Get the number of stored keyframes.
    fn keyframe_count(&self) -> usize;
}

impl LoopDetector for LoopClosureDetector {
    fn detect(
        &mut self,
        pose: Pose2D,
        lines: &[Line2D],
        corners: &[Corner2D],
        points: &[Point2D],
    ) -> Option<LoopClosure> {
        self.process(pose, lines, corners, points)
    }

    fn clear(&mut self) {
        LoopClosureDetector::clear(self)
    }

    fn can_detect(&self) -> bool {
        self.can_detect_loop()
    }

    fn keyframe_count(&self) -> usize {
        self.keyframes().len()
    }
}
