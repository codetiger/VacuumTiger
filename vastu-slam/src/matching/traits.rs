//! Traits for scan matching algorithms.
//!
//! This module provides a unified interface for different scan matching
//! implementations (correlative, branch-and-bound, etc.).

use crate::core::{LidarScan, Pose2D};
use crate::grid::GridStorage;

use super::ScanMatchResult;

/// Trait for scan-to-map matching algorithms.
///
/// Different scan matching algorithms (correlative, branch-and-bound, ICP, etc.)
/// can implement this trait to provide a unified interface.
///
/// # Example
///
/// ```ignore
/// use vastu_slam::matching::{ScanMatcher, CorrelativeMatcher};
///
/// fn localize<M: ScanMatcher>(
///     matcher: &M,
///     scan: &LidarScan,
///     initial_pose: Pose2D,
///     map: &GridStorage,
/// ) -> Pose2D {
///     let result = matcher.match_scan(scan, initial_pose, map);
///     if result.converged {
///         result.pose
///     } else {
///         initial_pose
///     }
/// }
/// ```
pub trait ScanMatcher: Send + Sync {
    /// Match a lidar scan against the map.
    ///
    /// # Arguments
    /// * `scan` - The lidar scan to match
    /// * `initial_pose` - Initial pose estimate (from odometry)
    /// * `map` - The occupancy grid to match against
    ///
    /// # Returns
    /// The match result containing the refined pose and confidence
    fn match_scan(
        &self,
        scan: &LidarScan,
        initial_pose: Pose2D,
        map: &GridStorage,
    ) -> ScanMatchResult;

    /// Get the name of this matcher for logging/debugging
    fn name(&self) -> &str;

    /// Get the minimum score threshold for this matcher
    fn min_score(&self) -> f32;
}
