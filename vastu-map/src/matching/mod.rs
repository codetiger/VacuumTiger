//! Scan-to-map matching algorithms.
//!
//! This module provides algorithms for matching lidar scans against a map
//! of line features. The primary algorithm is Point-to-Line ICP.
//!
//! # Components
//!
//! - **Correspondence**: Types for point-to-line associations
//! - **Nearest Neighbor**: Find correspondences by distance
//! - **RANSAC**: Robust pose estimation with outlier rejection
//! - **Gauss-Newton**: Nonlinear least squares optimization
//! - **Point-to-Line ICP**: Full scan matching pipeline
//! - **Scratch Space**: Pre-allocated buffers for zero-allocation ICP
//!
//! # Usage
//!
//! ```rust,ignore
//! use vastu_map::matching::{match_scan, IcpConfig};
//! use vastu_map::core::{Point2D, Pose2D};
//! use vastu_map::features::Line2D;
//!
//! // Map lines
//! let lines = vec![
//!     Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
//!     Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
//! ];
//!
//! // Scan points (in robot frame)
//! let points = vec![
//!     Point2D::new(1.0, 0.05),  // Near first line
//!     Point2D::new(2.0, -0.02), // Near first line
//!     Point2D::new(4.95, 2.0),  // Near second line
//! ];
//!
//! // Match scan against map
//! let initial_pose = Pose2D::new(0.0, 0.0, 0.0);  // From odometry
//! let result = match_scan(&points, &lines, initial_pose);
//!
//! if result.converged {
//!     println!("Matched pose: ({:.3}, {:.3}, {:.3})",
//!         result.pose.x, result.pose.y, result.pose.theta);
//!     println!("Confidence: {:.2}", result.confidence);
//! }
//! ```
//!
//! # Performance: Zero-Allocation Matching
//!
//! For best performance in hot paths, use `IcpScratchSpace` to reuse buffers:
//!
//! ```rust,ignore
//! use vastu_map::matching::{IcpScratchSpace, PointToLineIcp};
//!
//! let mut scratch = IcpScratchSpace::default_capacity();
//! let icp = PointToLineIcp::new();
//!
//! for scan in scans {
//!     let result = icp.match_scan_with_scratch(&scan, &map, pose, &mut scratch);
//! }
//! ```

pub mod correspondence;
pub mod gauss_newton;
pub mod nearest_neighbor;
pub mod point_to_line_icp;
pub mod ransac;
pub mod scratch;

// Re-export main types
pub use correspondence::{
    Correspondence, CorrespondenceSet, IDENTITY_COVARIANCE, MatchResult, PoseCovariance,
};
pub use gauss_newton::{GaussNewtonConfig, GaussNewtonResult, optimize_pose_fast};
pub use nearest_neighbor::{
    NearestNeighborConfig, find_correspondences, find_correspondences_batch,
    find_correspondences_spatial, find_correspondences_weighted, find_correspondences_with_angle,
};
pub use point_to_line_icp::{
    CoarseSearchConfig, IcpConfig, OutlierRejection, PointToLineIcp, match_scan,
    match_scan_with_config,
};
pub use ransac::{RansacConfig, RansacResult, estimate_pose_ransac};
pub use scratch::IcpScratchSpace;
