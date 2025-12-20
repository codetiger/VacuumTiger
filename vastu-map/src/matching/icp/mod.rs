//! Point-to-Line Iterative Closest Point (ICP) algorithm.
//!
//! This module implements the full scan-to-map matching pipeline:
//!
//! 1. **Correspondence Search**: Find nearest map lines for each scan point
//! 2. **Outlier Rejection**: Use RANSAC or distance threshold
//! 3. **Pose Optimization**: Gauss-Newton solver for point-to-line error
//! 4. **Iteration**: Repeat until convergence
//!
//! # Point-to-Line vs Point-to-Point
//!
//! Point-to-Line ICP converges faster than Point-to-Point because:
//! - Points can slide along lines in the tangent direction
//! - Only the perpendicular distance matters
//! - Better handles structured environments (walls, edges)

mod config;
mod matcher;

// Re-export public API
pub use config::{CoarseSearchConfig, IcpConfig, OutlierRejection};
pub use matcher::{PointToLineIcp, match_scan, match_scan_with_config};
