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
mod icp_common;
mod multi_resolution;
mod point_to_line_icp;

pub use correlative::{CorrelativeConfig, CorrelativeMatcher};
pub use icp::{CachedKdTree, IcpConfig, PointToPointIcp, RobustKernel};
pub use icp_common::{build_kdtree, mse_to_score};
pub use multi_resolution::{MultiResolutionConfig, MultiResolutionMatcher};
pub use point_to_line_icp::{PointToLineIcp, PointToLineIcpConfig};
// HybridP2LMatcher is defined in this module and exported directly

use crate::core::types::{Covariance2D, PointCloud2D, Pose2D};

/// Apply encoder weight to blend between identity and full odometry.
///
/// Blends the initial guess based on encoder_weight:
/// - 1.0: Trust odometry fully
/// - 0.0: Use identity (no prior motion)
/// - Intermediate values: Linear interpolation
fn weighted_initial_guess(odom_guess: &Pose2D, encoder_weight: f32) -> Pose2D {
    if encoder_weight >= 0.99 {
        *odom_guess
    } else if encoder_weight <= 0.01 {
        Pose2D::identity()
    } else {
        Pose2D::new(
            odom_guess.x * encoder_weight,
            odom_guess.y * encoder_weight,
            odom_guess.theta * encoder_weight,
        )
    }
}

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
    ///
    /// # Note
    ///
    /// Takes `&mut self` to allow matchers to reuse internal buffers for performance.
    fn match_scans(
        &mut self,
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
        &mut self,
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
    /// Encoder weight for initial guess (0.0 = identity, 1.0 = full odometry).
    /// Higher values trust encoder odometry more for the initial guess.
    pub encoder_weight: f32,
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
            encoder_weight: 1.0,      // Trust encoder fully by default (same as slam_benchmark)
        }
    }
}

/// Combined matcher using correlative search + Point-to-Line ICP.
///
/// Best for structured environments (indoor with walls):
/// 1. Correlative search handles large initial errors (especially rotation)
/// 2. Point-to-Line ICP provides sub-centimeter accuracy refinement
///
/// The encoder_weight parameter controls how much the odometry initial guess
/// is trusted vs identity. Higher values give more weight to encoder odometry.
#[derive(Debug)]
pub struct HybridP2LMatcher {
    correlative: CorrelativeMatcher,
    p2l_icp: PointToLineIcp,
    always_correlative: bool,
    encoder_weight: f32,
}

impl HybridP2LMatcher {
    /// Create a new hybrid Point-to-Line matcher.
    pub fn new(config: HybridP2LMatcherConfig) -> Self {
        Self {
            correlative: CorrelativeMatcher::new(config.correlative),
            p2l_icp: PointToLineIcp::new(config.p2l_icp),
            always_correlative: config.always_correlative,
            encoder_weight: config.encoder_weight,
        }
    }
}

impl ScanMatcher for HybridP2LMatcher {
    fn match_scans(
        &mut self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        // Apply encoder weight to initial guess
        let weighted_guess = weighted_initial_guess(initial_guess, self.encoder_weight);

        let guess = if self.always_correlative {
            // Always use correlative for initial alignment
            let corr_result = self
                .correlative
                .match_scans(source, target, &weighted_guess);
            if corr_result.converged {
                corr_result.transform
            } else {
                // Correlative failed, try ICP with weighted guess
                weighted_guess
            }
        } else {
            weighted_guess
        };

        // Refine with Point-to-Line ICP
        let icp_result = self.p2l_icp.match_scans(source, target, &guess);

        if icp_result.converged {
            icp_result
        } else if !self.always_correlative {
            // ICP failed without correlative, try correlative first
            let corr_result = self
                .correlative
                .match_scans(source, target, &weighted_guess);
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

/// Configuration for hybrid Multi-Resolution + Robust ICP matcher.
///
/// Based on benchmarks, this combination provides:
/// - Multi-Resolution: 0.74cm accuracy, robust to poor initial guess
/// - Robust ICP: Fast refinement with good accuracy when given good initial guess
#[derive(Debug, Clone)]
pub struct HybridMultiResConfig {
    /// Multi-resolution correlative config
    pub multi_res: MultiResolutionConfig,
    /// Robust ICP config for refinement
    pub icp: IcpConfig,
    /// Whether to always run multi-resolution first
    pub always_multi_res: bool,
    /// Encoder weight for initial guess (0.0 = identity, 1.0 = full odometry)
    pub encoder_weight: f32,
}

impl Default for HybridMultiResConfig {
    fn default() -> Self {
        Self {
            multi_res: MultiResolutionConfig {
                num_levels: 3,
                coarse_search_window_xy: 0.15,
                coarse_search_window_theta: 0.15,
                fine_resolution: 0.005,
                fine_angular_resolution: 0.005,
                fine_grid_resolution: 0.02,
                min_score: 0.4,
                resolution_multiplier: 2.0,
                window_shrink_factor: 2.0,
            },
            icp: IcpConfig {
                max_iterations: 30,
                translation_epsilon: 0.001,
                rotation_epsilon: 0.001,
                max_correspondence_distance: 0.3,
                min_correspondences: 10,
                outlier_ratio: 0.1,
                robust_kernel: RobustKernel::Welsch,
                kernel_scale: 0.10,
                bidirectional_check: true,
                damping_factor: 0.8,
            },
            always_multi_res: true,
            encoder_weight: 0.8, // Encoder-weighted initial guess
        }
    }
}

/// Combined matcher using Multi-Resolution Correlative + Robust ICP.
///
/// Recommended for production SLAM:
/// 1. Multi-Resolution handles large initial errors efficiently (~0.85ms)
/// 2. Robust ICP refines with Welsch kernel for sub-cm accuracy
///
/// The encoder_weight parameter controls how much the odometry initial guess
/// is trusted vs identity. Higher values give more weight to encoder odometry.
#[derive(Debug)]
pub struct HybridMultiResMatcher {
    multi_res: MultiResolutionMatcher,
    icp: PointToPointIcp,
    always_multi_res: bool,
    encoder_weight: f32,
}

impl HybridMultiResMatcher {
    /// Create a new hybrid Multi-Resolution + ICP matcher.
    pub fn new(config: HybridMultiResConfig) -> Self {
        Self {
            multi_res: MultiResolutionMatcher::new(config.multi_res),
            icp: PointToPointIcp::new(config.icp),
            always_multi_res: config.always_multi_res,
            encoder_weight: config.encoder_weight,
        }
    }
}

impl ScanMatcher for HybridMultiResMatcher {
    fn match_scans(
        &mut self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        // Apply encoder weight to initial guess
        let weighted_guess = weighted_initial_guess(initial_guess, self.encoder_weight);

        let guess = if self.always_multi_res {
            // Always use multi-resolution for initial alignment
            let multi_result = self.multi_res.match_scans(source, target, &weighted_guess);
            if multi_result.converged {
                multi_result.transform
            } else {
                weighted_guess
            }
        } else {
            weighted_guess
        };

        // Refine with Robust ICP
        let icp_result = self.icp.match_scans(source, target, &guess);

        if icp_result.converged {
            icp_result
        } else if !self.always_multi_res {
            // ICP failed without multi-res, try multi-res first
            let multi_result = self.multi_res.match_scans(source, target, &weighted_guess);
            if multi_result.converged {
                // Refine multi-res result with ICP
                self.icp
                    .match_scans(source, target, &multi_result.transform)
            } else {
                ScanMatchResult::failed()
            }
        } else {
            // Both multi-res and ICP failed
            ScanMatchResult::failed()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;

    /// Create a room-shaped point cloud (four walls).
    fn create_room(n: usize, width: f32, height: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::new();
        let points_per_wall = n / 4;

        // Bottom wall
        for i in 0..points_per_wall {
            let x = (i as f32 / points_per_wall as f32) * width;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(x, noise));
        }
        // Right wall
        for i in 0..points_per_wall {
            let y = (i as f32 / points_per_wall as f32) * height;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(width + noise, y));
        }
        // Top wall
        for i in 0..points_per_wall {
            let x = width - (i as f32 / points_per_wall as f32) * width;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(x, height + noise));
        }
        // Left wall
        for i in 0..points_per_wall {
            let y = height - (i as f32 / points_per_wall as f32) * height;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(noise, y));
        }
        cloud
    }

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

    #[test]
    fn test_hybrid_p2l_large_rotation() {
        use approx::assert_relative_eq;

        // Test large rotation (25°) - correlative should find it
        let source = create_room(100, 4.0, 3.0);
        let transform = Pose2D::new(0.0, 0.0, 0.44); // ~25 degrees
        let target = source.transform(&transform);

        let mut matcher = HybridP2LMatcher::new(HybridP2LMatcherConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should converge for large rotation");
        assert_relative_eq!(result.transform.theta, 0.44, epsilon = 0.08);
    }

    #[test]
    fn test_hybrid_p2l_combined_large_motion() {
        use approx::assert_relative_eq;

        // Test combined translation + rotation
        let source = create_room(100, 4.0, 3.0);
        let transform = Pose2D::new(0.2, 0.15, 0.35); // ~20 degrees + translation
        let target = source.transform(&transform);

        let mut matcher = HybridP2LMatcher::new(HybridP2LMatcherConfig::default());
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
