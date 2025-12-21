//! Traits for scan matching algorithms.
//!
//! This module defines the `ScanMatcher` trait which allows different
//! scan matching algorithms to be used interchangeably.

use crate::core::{Point2D, Pose2D};
use crate::features::{Line2D, LineCollection};

use super::correspondence::MatchResult;

/// Trait for scan matching algorithms.
///
/// Implementations match scan points against a map of line features
/// to estimate the robot's pose.
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::matching::{ScanMatcher, PointToLineIcp, IcpConfig};
/// use vastu_map::core::{Point2D, Pose2D};
/// use vastu_map::features::Line2D;
///
/// let mut matcher = PointToLineIcp::with_config(IcpConfig::default());
/// let points = vec![Point2D::new(1.0, 0.1)];
/// let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0))];
///
/// let result = matcher.match_scan(&points, &lines, Pose2D::identity());
/// ```
pub trait ScanMatcher: Send + Sync {
    /// Match scan points against map lines.
    ///
    /// # Arguments
    /// * `points` - Scan points (in robot frame or world frame depending on implementation)
    /// * `lines` - Map lines in world frame
    /// * `initial_pose` - Initial pose estimate (typically from odometry)
    ///
    /// # Returns
    /// Match result with optimized pose, covariance, and confidence.
    fn match_scan(
        &mut self,
        points: &[Point2D],
        lines: &[Line2D],
        initial_pose: Pose2D,
    ) -> MatchResult;

    /// Match scan against a LineCollection (SoA format for SIMD).
    ///
    /// Default implementation converts to Line2D slice and calls `match_scan`.
    fn match_scan_collection(
        &mut self,
        points: &[Point2D],
        lines: &LineCollection,
        initial_pose: Pose2D,
    ) -> MatchResult {
        let lines_vec = lines.to_lines();
        self.match_scan(points, &lines_vec, initial_pose)
    }
}

// PointToLineIcp implements ScanMatcher via its inherent methods
// We can't add a blanket impl here due to Rust's orphan rules,
// so we implement it directly on the type.

impl ScanMatcher for super::PointToLineIcp {
    fn match_scan(
        &mut self,
        points: &[Point2D],
        lines: &[Line2D],
        initial_pose: Pose2D,
    ) -> MatchResult {
        // Use the inherent match_scan method
        super::PointToLineIcp::match_scan(self, points, lines, initial_pose)
    }

    fn match_scan_collection(
        &mut self,
        points: &[Point2D],
        lines: &LineCollection,
        initial_pose: Pose2D,
    ) -> MatchResult {
        self.match_scan_batch(points, lines, initial_pose)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::matching::IcpConfig;

    fn make_room_lines() -> Vec<Line2D> {
        vec![
            Line2D::new(Point2D::new(-2.0, -2.0), Point2D::new(2.0, -2.0)),
            Line2D::new(Point2D::new(2.0, -2.0), Point2D::new(2.0, 2.0)),
            Line2D::new(Point2D::new(2.0, 2.0), Point2D::new(-2.0, 2.0)),
            Line2D::new(Point2D::new(-2.0, 2.0), Point2D::new(-2.0, -2.0)),
        ]
    }

    fn make_scan_points() -> Vec<Point2D> {
        let mut points = Vec::new();
        for i in -5..=5 {
            let x = i as f32 * 0.3;
            points.push(Point2D::new(x, -2.0));
            points.push(Point2D::new(x, 2.0));
        }
        for i in -5..=5 {
            let y = i as f32 * 0.3;
            points.push(Point2D::new(-2.0, y));
            points.push(Point2D::new(2.0, y));
        }
        points
    }

    #[test]
    fn test_scan_matcher_trait() {
        let mut matcher = super::super::PointToLineIcp::with_config(IcpConfig::default());
        let lines = make_room_lines();
        let points = make_scan_points();

        let result = matcher.match_scan(&points, &lines, Pose2D::identity());

        assert!(result.converged);
    }

    #[test]
    fn test_trait_object() {
        // Verify trait can be used as a trait object
        let mut matcher: Box<dyn ScanMatcher> = Box::new(
            super::super::PointToLineIcp::with_config(IcpConfig::default()),
        );

        let lines = make_room_lines();
        let points = make_scan_points();

        let result = matcher.match_scan(&points, &lines, Pose2D::identity());
        assert!(result.converged);
    }

    #[test]
    fn test_collection_method() {
        let mut matcher = super::super::PointToLineIcp::with_config(IcpConfig::default());
        let lines = make_room_lines();
        let collection = LineCollection::from_lines(&lines);
        let points = make_scan_points();

        let result = matcher.match_scan_collection(&points, &collection, Pose2D::identity());

        assert!(result.converged);
    }
}
