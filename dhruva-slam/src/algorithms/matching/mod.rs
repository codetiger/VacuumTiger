//! Scan matching module (Phase 4).
//!
//! Provides algorithms for aligning laser scans to estimate relative motion.
//!
//! # Algorithms
//!
//! - [`PointToPointIcp`]: Classic Iterative Closest Point algorithm
//! - [`PointToLineIcp`]: Point-to-line ICP for structured environments
//! - [`CorrelativeMatcher`]: Exhaustive search for handling large initial errors
//! - [`MultiResolutionMatcher`]: Hierarchical correlative matching for efficiency
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::scan_matching::{PointToPointIcp, PointToLineIcp, IcpConfig, ScanMatcher};
//! use dhruva_slam::types::{PointCloud2D, Pose2D};
//!
//! // Use point-to-point ICP for general environments
//! let config = IcpConfig::default();
//! let icp = PointToPointIcp::new(config);
//!
//! // Use point-to-line ICP for structured environments (walls)
//! let p2l_icp = PointToLineIcp::new(PointToLineIcpConfig::default());
//!
//! let result = icp.match_scans(&source, &target, &initial_guess);
//! if result.converged {
//!     println!("Transform: {:?}", result.transform);
//! }
//! ```

mod correlative;
mod icp;
mod multi_resolution;
mod point_to_line_icp;

pub use correlative::{CorrelativeConfig, CorrelativeMatcher};
pub use icp::{IcpConfig, PointToPointIcp};
pub use multi_resolution::{MultiResolutionConfig, MultiResolutionMatcher};
pub use point_to_line_icp::{PointToLineIcp, PointToLineIcpConfig};
// HybridP2LMatcher is defined in this module and exported directly

use crate::core::types::{Covariance2D, PointCloud2D, Pose2D};

/// Result of a scan matching operation.
#[derive(Debug, Clone)]
pub struct ScanMatchResult {
    /// Estimated transform from source to target frame.
    pub transform: Pose2D,

    /// Covariance of the transform estimate (uncertainty).
    pub covariance: Covariance2D,

    /// Match quality score (0.0 = bad, 1.0 = perfect).
    ///
    /// Computed from mean squared error of correspondences.
    pub score: f32,

    /// Whether the algorithm converged successfully.
    pub converged: bool,

    /// Number of iterations performed.
    pub iterations: u32,

    /// Mean squared error of final correspondences.
    pub mse: f32,
}

impl Default for ScanMatchResult {
    fn default() -> Self {
        Self {
            transform: Pose2D::identity(),
            covariance: Covariance2D::diagonal(1.0, 1.0, 0.1),
            score: 0.0,
            converged: false,
            iterations: 0,
            mse: f32::MAX,
        }
    }
}

impl ScanMatchResult {
    /// Create a failed result with identity transform.
    pub fn failed() -> Self {
        Self::default()
    }

    /// Create a successful result.
    ///
    /// Note: Covariance is set to zero because computing it from the ICP Hessian
    /// requires storing the full correspondence set and recomputing the Jacobian.
    /// For most SLAM use cases, the match score provides sufficient quality indication.
    /// A proper covariance estimate would improve loop closure uncertainty propagation.
    pub fn success(transform: Pose2D, score: f32, iterations: u32, mse: f32) -> Self {
        Self {
            transform,
            covariance: Covariance2D::zero(),
            score,
            converged: true,
            iterations,
            mse,
        }
    }
}

/// Trait for scan matching algorithms.
pub trait ScanMatcher {
    /// Align source point cloud to target point cloud.
    ///
    /// # Arguments
    ///
    /// * `source` - The point cloud to be transformed
    /// * `target` - The reference point cloud
    /// * `initial_guess` - Initial transform estimate (source frame → target frame)
    ///
    /// # Returns
    ///
    /// A `ScanMatchResult` containing the estimated transform and quality metrics.
    fn match_scans(
        &self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult;
}

/// Combined matcher that uses correlative for initialization and ICP for refinement.
///
/// This is the recommended matcher for production use:
/// 1. Correlative matcher finds approximate alignment (handles large errors)
/// 2. ICP refines to sub-centimeter accuracy
#[derive(Debug)]
pub struct HybridMatcher {
    correlative: CorrelativeMatcher,
    icp: PointToPointIcp,
    /// If true, always run correlative first. If false, only on ICP failure.
    always_correlative: bool,
}

/// Configuration for hybrid matcher.
#[derive(Debug, Clone, Default)]
pub struct HybridMatcherConfig {
    pub correlative: CorrelativeConfig,
    pub icp: IcpConfig,
    /// If true, always run correlative first.
    pub always_correlative: bool,
}

impl HybridMatcher {
    /// Create a new hybrid matcher.
    pub fn new(config: HybridMatcherConfig) -> Self {
        Self {
            correlative: CorrelativeMatcher::new(config.correlative),
            icp: PointToPointIcp::new(config.icp),
            always_correlative: config.always_correlative,
        }
    }
}

impl ScanMatcher for HybridMatcher {
    fn match_scans(
        &self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        let guess = if self.always_correlative {
            // Always use correlative for initial alignment
            let corr_result = self.correlative.match_scans(source, target, initial_guess);
            if corr_result.converged {
                corr_result.transform
            } else {
                *initial_guess
            }
        } else {
            *initial_guess
        };

        // Try ICP with the guess
        let icp_result = self.icp.match_scans(source, target, &guess);

        if icp_result.converged {
            icp_result
        } else if !self.always_correlative {
            // ICP failed, try correlative
            let corr_result = self.correlative.match_scans(source, target, initial_guess);
            if corr_result.converged {
                // Refine with ICP
                self.icp.match_scans(source, target, &corr_result.transform)
            } else {
                ScanMatchResult::failed()
            }
        } else {
            ScanMatchResult::failed()
        }
    }
}

/// Configuration for hybrid Point-to-Line matcher.
#[derive(Debug, Clone)]
pub struct HybridP2LMatcherConfig {
    pub correlative: CorrelativeConfig,
    pub p2l_icp: PointToLineIcpConfig,
    /// If true, always run correlative first. If false, only on ICP failure.
    pub always_correlative: bool,
}

impl Default for HybridP2LMatcherConfig {
    fn default() -> Self {
        Self {
            correlative: CorrelativeConfig {
                // Wider search window for rotation recovery
                search_window_x: 0.3,
                search_window_y: 0.3,
                search_window_theta: 0.5, // ±28° for rotation recovery
                linear_resolution: 0.03,
                angular_resolution: 0.03, // ~1.7° steps
                grid_resolution: 0.05,
                min_score: 0.4, // Lower threshold since we refine with ICP
            },
            p2l_icp: PointToLineIcpConfig::default(),
            always_correlative: true, // Always use correlative for robustness
        }
    }
}

/// Combined matcher using correlative search + Point-to-Line ICP.
///
/// Best for structured environments (indoor with walls):
/// 1. Correlative search handles large initial errors (especially rotation)
/// 2. Point-to-Line ICP provides sub-centimeter accuracy refinement
#[derive(Debug)]
pub struct HybridP2LMatcher {
    correlative: CorrelativeMatcher,
    p2l_icp: PointToLineIcp,
    always_correlative: bool,
}

impl HybridP2LMatcher {
    /// Create a new hybrid Point-to-Line matcher.
    pub fn new(config: HybridP2LMatcherConfig) -> Self {
        Self {
            correlative: CorrelativeMatcher::new(config.correlative),
            p2l_icp: PointToLineIcp::new(config.p2l_icp),
            always_correlative: config.always_correlative,
        }
    }
}

impl ScanMatcher for HybridP2LMatcher {
    fn match_scans(
        &self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        let guess = if self.always_correlative {
            // Always use correlative for initial alignment
            let corr_result = self.correlative.match_scans(source, target, initial_guess);
            if corr_result.converged {
                corr_result.transform
            } else {
                // Correlative failed, try ICP with original guess
                *initial_guess
            }
        } else {
            *initial_guess
        };

        // Refine with Point-to-Line ICP
        let icp_result = self.p2l_icp.match_scans(source, target, &guess);

        if icp_result.converged {
            icp_result
        } else if !self.always_correlative {
            // ICP failed without correlative, try correlative first
            let corr_result = self.correlative.match_scans(source, target, initial_guess);
            if corr_result.converged {
                // Refine correlative result with ICP
                self.p2l_icp
                    .match_scans(source, target, &corr_result.transform)
            } else {
                ScanMatchResult::failed()
            }
        } else {
            // Both correlative and ICP failed
            ScanMatchResult::failed()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;

    #[test]
    fn test_scan_match_result_default() {
        let result = ScanMatchResult::default();
        assert!(!result.converged);
        assert_eq!(result.transform.x, 0.0);
    }

    #[test]
    fn test_scan_match_result_success() {
        let transform = Pose2D::new(1.0, 2.0, 0.5);
        let result = ScanMatchResult::success(transform, 0.95, 10, 0.001);

        assert!(result.converged);
        assert_eq!(result.transform.x, 1.0);
        assert_eq!(result.score, 0.95);
        assert_eq!(result.iterations, 10);
    }

    /// Create room with slight noise to avoid k-d tree bucket issues
    fn create_room_cloud(n: usize, width: f32, height: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::new();
        let points_per_wall = n / 4;

        for i in 0..points_per_wall {
            let x = (i as f32 / points_per_wall as f32) * width;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(x, noise)); // Bottom
        }
        for i in 0..points_per_wall {
            let y = (i as f32 / points_per_wall as f32) * height;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(width + noise, y)); // Right
        }
        for i in 0..points_per_wall {
            let x = width - (i as f32 / points_per_wall as f32) * width;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(x, height + noise)); // Top
        }
        for i in 0..points_per_wall {
            let y = height - (i as f32 / points_per_wall as f32) * height;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(noise, y)); // Left
        }
        cloud
    }

    #[test]
    fn test_hybrid_p2l_large_rotation() {
        use approx::assert_relative_eq;

        // Test large rotation (25°) - correlative should find it
        let source = create_room_cloud(100, 4.0, 3.0);
        let transform = Pose2D::new(0.0, 0.0, 0.44); // ~25 degrees
        let target = source.transform(&transform);

        let matcher = HybridP2LMatcher::new(HybridP2LMatcherConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should converge for large rotation");
        assert_relative_eq!(result.transform.theta, 0.44, epsilon = 0.08);
    }

    #[test]
    fn test_hybrid_p2l_combined_large_motion() {
        use approx::assert_relative_eq;

        // Test combined translation + rotation
        let source = create_room_cloud(100, 4.0, 3.0);
        let transform = Pose2D::new(0.2, 0.15, 0.35); // ~20 degrees + translation
        let target = source.transform(&transform);

        let matcher = HybridP2LMatcher::new(HybridP2LMatcherConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(
            result.converged,
            "Should converge for combined large motion"
        );
        assert_relative_eq!(result.transform.x, 0.2, epsilon = 0.08);
        assert_relative_eq!(result.transform.y, 0.15, epsilon = 0.08);
        assert_relative_eq!(result.transform.theta, 0.35, epsilon = 0.08);
    }
}
