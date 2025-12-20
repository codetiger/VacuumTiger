//! Point-to-Line Iterative Closest Point (ICP) algorithm.
//!
//! This module implements the full scan-to-map matching pipeline:
//!
//! 1. **Correspondence Search**: Find nearest map lines for each scan point
//! 2. **Robust Weighting**: Apply M-estimator weights via IRLS to down-weight outliers
//! 3. **Pose Optimization**: Levenberg-Marquardt solver for weighted point-to-line error
//! 4. **Iteration**: Repeat until convergence
//!
//! # Point-to-Line vs Point-to-Point
//!
//! Point-to-Line ICP converges faster than Point-to-Point because:
//! - Points can slide along lines in the tangent direction
//! - Only the perpendicular distance matters
//! - Better handles structured environments (walls, edges)
//!
//! # Robust Cost Functions
//!
//! Use `RobustCostFunction` with IRLS for soft outlier rejection:
//! - **Huber** (default): Good balance between robustness and efficiency
//! - **Cauchy**: Heavy-tailed, good for cluttered environments
//! - **Tukey**: Zero weight for large residuals (aggressive rejection)
//! - **Geman-McClure**: Redescending M-estimator for multimodal errors

mod config;
mod matcher;

// Re-export public API
pub use config::{CoarseSearchConfig, IcpConfig, MultiResolutionConfig};
pub use matcher::{PointToLineIcp, match_scan, match_scan_multiresolution, match_scan_with_config};
