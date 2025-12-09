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
//! # Transform Semantics (IMPORTANT!)
//!
//! Understanding the returned transform is critical for correct SLAM integration.
//!
//! **The matcher aligns `source` points to overlap with `target` points.**
//!
//! The returned `transform` answers: "What transform, when applied to source points,
//! makes them overlap with target points?"
//!
//! In the context of sequential SLAM (scan-to-scan matching):
//! - `source` = current scan (from robot's current position)
//! - `target` = previous scan (from robot's previous position)
//!
//! For a robot moving **forward**:
//! - Current scan sees objects **closer** than previous scan
//! - To align current→previous, we need to **push points forward** (positive X)
//! - Therefore, `transform.x > 0` for forward motion
//!
//! **The transform IS the robot motion from previous to current position!**
//!
//! ## Usage Pattern
//!
//! ```ignore
//! // Correct usage (NO inversion needed):
//! let match_result = matcher.match_scans(&current_scan, &previous_scan, &odom_delta);
//!
//! // Use transform directly as robot motion
//! let robot_motion = match_result.transform;
//! slam_pose = slam_pose.compose(&robot_motion);
//! ```
//!
//! ## Common Mistake
//!
//! A common mistake is inverting the transform, thinking it's the "scan alignment":
//!
//! ```ignore
//! // WRONG! This inverts the robot motion:
//! let robot_motion = match_result.transform.inverse();  // DON'T DO THIS!
//! ```
//!
//! If you find poses drifting in the wrong direction, check your transform handling.
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
mod dynamic;
mod hybrid;
mod icp;
mod icp_common;
mod multi_resolution;
mod point_to_line_icp;
#[cfg(test)]
pub mod test_utils;
mod validation;

pub use correlative::{CorrelativeConfig, CorrelativeMatcher};
pub use dynamic::{DynMatcher, DynMatcherConfig, MatcherType};
pub use hybrid::{HybridConfig, HybridMatcher, weighted_initial_guess};
pub use icp::{CachedKdTree, IcpConfig, PointToPointIcp, RobustKernel};
pub use icp_common::{
    ConvergenceTracker, IcpConvergenceConfig, MIN_POINTS_FOR_RELIABLE_MATCH, build_kdtree,
    mse_to_score, thresholds, validate_scan_sizes,
};
pub use multi_resolution::{MultiResolutionConfig, MultiResolutionMatcher};
pub use point_to_line_icp::{PointToLineIcp, PointToLineIcpConfig};
pub use validation::{
    RejectionReason, ScanMatchValidator, ScanMatchValidatorConfig, ValidationResult,
};

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
///
/// # Transform Semantics
///
/// The returned transform represents **robot motion from target time to source time**.
///
/// For sequential SLAM where:
/// - `source` = current scan (robot at time t+1)
/// - `target` = previous scan (robot at time t)
///
/// The transform is the robot motion from t to t+1. Use it directly:
/// ```ignore
/// let result = matcher.match_scans(&current_scan, &previous_scan, &odom_guess);
/// slam_pose = slam_pose.compose(&result.transform);  // Correct!
/// ```
///
/// **Do NOT invert the transform** - this is a common mistake that causes
/// poses to drift in the wrong direction.
pub trait ScanMatcher {
    /// Align source point cloud to target point cloud.
    ///
    /// Finds the transform T such that `source.transform(&T)` overlaps with `target`.
    ///
    /// # Arguments
    ///
    /// * `source` - The point cloud to be transformed (typically: current scan)
    /// * `target` - The reference point cloud (typically: previous scan)
    /// * `initial_guess` - Initial transform estimate (e.g., odometry delta)
    ///
    /// # Returns
    ///
    /// A [`ScanMatchResult`] containing:
    /// - `transform`: The robot motion from target→source time (use directly, don't invert!)
    /// - `score`: Match quality (0.0=bad, 1.0=perfect)
    /// - `converged`: Whether the algorithm converged successfully
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

// ============================================================================
// Convenient type aliases for common hybrid matcher combinations
// ============================================================================

/// Configuration for hybrid Correlative + P2P ICP matcher.
#[derive(Debug, Clone, Default)]
pub struct HybridIcpMatcherConfig {
    pub correlative: CorrelativeConfig,
    pub icp: IcpConfig,
    /// If true, always run correlative first.
    pub always_correlative: bool,
}

impl HybridIcpMatcherConfig {
    /// Convert to generic HybridConfig.
    pub fn to_hybrid_config(&self) -> HybridConfig {
        HybridConfig {
            always_coarse: self.always_correlative,
            encoder_weight: 1.0,
        }
    }
}

/// Combined matcher using correlative search + Point-to-Point ICP.
///
/// This is a type alias for the generic `HybridMatcher`.
pub type HybridIcpMatcher = HybridMatcher<CorrelativeMatcher, PointToPointIcp>;

impl HybridIcpMatcher {
    /// Create a new hybrid ICP matcher from config.
    pub fn from_config(config: HybridIcpMatcherConfig) -> Self {
        let hybrid_config = config.to_hybrid_config();
        HybridMatcher::new(
            CorrelativeMatcher::new(config.correlative),
            PointToPointIcp::new(config.icp),
            hybrid_config,
        )
    }
}

/// Configuration for hybrid Point-to-Line matcher.
///
/// Use with `HybridP2LMatcher` type alias.
#[derive(Debug, Clone)]
pub struct HybridP2LMatcherConfig {
    pub correlative: CorrelativeConfig,
    pub p2l_icp: PointToLineIcpConfig,
    /// If true, always run correlative first. If false, only on ICP failure.
    pub always_correlative: bool,
    /// Encoder weight for initial guess (0.0 = identity, 1.0 = full odometry).
    pub encoder_weight: f32,
}

impl Default for HybridP2LMatcherConfig {
    fn default() -> Self {
        Self {
            correlative: CorrelativeConfig {
                search_window_x: 0.3,
                search_window_y: 0.3,
                search_window_theta: 0.5,
                linear_resolution: 0.03,
                angular_resolution: 0.03,
                grid_resolution: 0.05,
                min_score: 0.4,
            },
            p2l_icp: PointToLineIcpConfig::default(),
            always_correlative: true,
            encoder_weight: 1.0,
        }
    }
}

impl HybridP2LMatcherConfig {
    /// Convert to generic HybridConfig.
    pub fn to_hybrid_config(&self) -> HybridConfig {
        HybridConfig {
            always_coarse: self.always_correlative,
            encoder_weight: self.encoder_weight,
        }
    }
}

/// Combined matcher using correlative search + Point-to-Line ICP.
///
/// Best for structured environments (indoor with walls).
/// This is a type alias for the generic `HybridMatcher`.
pub type HybridP2LMatcher = HybridMatcher<CorrelativeMatcher, PointToLineIcp>;

impl HybridP2LMatcher {
    /// Create a new hybrid Point-to-Line matcher from config.
    pub fn from_config(config: HybridP2LMatcherConfig) -> Self {
        let hybrid_config = config.to_hybrid_config();
        HybridMatcher::new(
            CorrelativeMatcher::new(config.correlative),
            PointToLineIcp::new(config.p2l_icp),
            hybrid_config,
        )
    }
}

/// Configuration for hybrid Multi-Resolution + Robust ICP matcher.
#[derive(Debug, Clone)]
pub struct HybridMultiResConfig {
    pub multi_res: MultiResolutionConfig,
    pub icp: IcpConfig,
    pub always_multi_res: bool,
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
            encoder_weight: 0.8,
        }
    }
}

impl HybridMultiResConfig {
    /// Convert to generic HybridConfig.
    pub fn to_hybrid_config(&self) -> HybridConfig {
        HybridConfig {
            always_coarse: self.always_multi_res,
            encoder_weight: self.encoder_weight,
        }
    }
}

/// Combined matcher using Multi-Resolution Correlative + Robust ICP.
///
/// Recommended for production SLAM.
/// This is a type alias for the generic `HybridMatcher`.
pub type HybridMultiResMatcher = HybridMatcher<MultiResolutionMatcher, PointToPointIcp>;

impl HybridMultiResMatcher {
    /// Create a new hybrid Multi-Resolution + ICP matcher from config.
    pub fn from_config(config: HybridMultiResConfig) -> Self {
        let hybrid_config = config.to_hybrid_config();
        HybridMatcher::new(
            MultiResolutionMatcher::new(config.multi_res),
            PointToPointIcp::new(config.icp),
            hybrid_config,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_utils::create_room;

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

        let mut matcher = HybridP2LMatcher::from_config(HybridP2LMatcherConfig::default());
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

        let mut matcher = HybridP2LMatcher::from_config(HybridP2LMatcherConfig::default());
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
