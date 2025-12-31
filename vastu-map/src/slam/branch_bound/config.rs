//! Branch-and-bound configuration and result types.

use crate::core::Pose2D;

/// Number of resolution levels in the precomputed hierarchy.
/// Level 0 = original resolution, Level N = 4^N times coarser.
pub const NUM_LEVELS: usize = 4;

/// Configuration for branch-and-bound matching.
#[derive(Clone, Debug)]
pub struct BranchBoundConfig {
    /// Search window in X (meters).
    pub search_x: f32,
    /// Search window in Y (meters).
    pub search_y: f32,
    /// Search window in theta (radians).
    pub search_theta: f32,
    /// Angular resolution (radians per bin).
    pub angular_resolution: f32,
    /// Minimum score to consider a match valid.
    pub min_score: f32,
    /// Sensor offset (x, y) in robot frame.
    pub sensor_offset: (f32, f32),
}

impl Default for BranchBoundConfig {
    fn default() -> Self {
        Self {
            search_x: 0.3,
            search_y: 0.3,
            search_theta: 0.3,
            angular_resolution: 0.02, // ~1.15 degrees
            min_score: 0.0,
            sensor_offset: (-0.110, 0.0),
        }
    }
}

/// Result of branch-and-bound matching.
#[derive(Clone, Debug)]
pub struct BranchBoundResult {
    /// Best matching pose.
    pub pose: Pose2D,
    /// Score of the best match.
    pub score: f32,
    /// Number of nodes expanded.
    pub nodes_expanded: usize,
    /// Whether a valid match was found.
    pub converged: bool,
}
