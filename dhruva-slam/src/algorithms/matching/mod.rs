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
//!
//! Note: Some types and utility methods are defined for future use.

mod correlative;
mod dynamic;
mod hybrid;
mod icp;
mod icp_common;
mod multi_resolution;
mod point_to_line_icp;
mod robust_kernels;
#[cfg(test)]
pub mod test_utils;

// Individual matcher implementations
pub use correlative::{CorrelativeConfig, CorrelativeMatcher};
pub use dynamic::{DynMatcher, DynMatcherConfig, MatcherType};
pub use hybrid::{HybridConfig, HybridMatcher};
pub use icp::{IcpConfig, PointToPointIcp};
pub use multi_resolution::{MultiResolutionConfig, MultiResolutionMatcher};
pub use point_to_line_icp::{PointToLineIcp, PointToLineIcpConfig};
pub use robust_kernels::RobustKernel;

use crate::core::types::{PointCloud2D, Pose2D};

/// Result of a scan matching operation.
#[derive(Debug, Clone)]
pub struct ScanMatchResult {
    /// Estimated transform from source to target frame.
    pub transform: Pose2D,

    /// Match quality score (0.0 = bad, 1.0 = perfect).
    ///
    /// Computed from mean squared error of correspondences.
    pub score: f32,

    /// Whether the algorithm converged successfully.
    pub converged: bool,

    /// Number of iterations performed.
    pub iterations: u32,
}

impl Default for ScanMatchResult {
    fn default() -> Self {
        Self {
            transform: Pose2D::identity(),
            score: 0.0,
            converged: false,
            iterations: 0,
        }
    }
}

impl ScanMatchResult {
    /// Create a failed result with identity transform.
    pub fn failed() -> Self {
        Self::default()
    }

    /// Create a successful result.
    pub fn success(transform: Pose2D, score: f32, iterations: u32) -> Self {
        Self {
            transform,
            score,
            converged: true,
            iterations,
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
        let result = ScanMatchResult::success(transform, 0.95, 10);

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

// ============================================================================
// Phase 2: ICP Transform Recovery Tests
// ============================================================================

#[cfg(test)]
mod icp_recovery_tests {
    use super::*;
    use crate::algorithms::matching::test_utils::{create_line, create_room};
    use crate::core::types::Point2D;
    use approx::assert_relative_eq;

    #[test]
    fn test_icp_identity_transform() {
        let cloud = create_room(100, 4.0, 3.0);

        let mut matcher = PointToPointIcp::new(IcpConfig::default());
        let result = matcher.match_scans(&cloud, &cloud, &Pose2D::identity());

        assert!(result.converged, "ICP should converge for identical clouds");
        assert_relative_eq!(result.transform.x, 0.0, epsilon = 0.02);
        assert_relative_eq!(result.transform.y, 0.0, epsilon = 0.02);
        assert_relative_eq!(result.transform.theta, 0.0, epsilon = 0.02);
    }

    #[test]
    fn test_hybrid_p2l_known_translation() {
        // Use HybridP2LMatcher which is the recommended production matcher
        let source = create_room(100, 4.0, 3.0);
        let known_transform = Pose2D::new(0.15, 0.10, 0.0);
        let target = source.transform(&known_transform);

        let mut matcher = HybridP2LMatcher::from_config(HybridP2LMatcherConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(
            result.converged,
            "HybridP2L should converge for translation"
        );
        assert_relative_eq!(result.transform.x, 0.15, epsilon = 0.06);
        assert_relative_eq!(result.transform.y, 0.10, epsilon = 0.06);
    }

    #[test]
    fn test_hybrid_p2l_known_rotation() {
        let source = create_room(100, 4.0, 3.0);
        let known_transform = Pose2D::new(0.0, 0.0, 0.2);
        let target = source.transform(&known_transform);

        let mut matcher = HybridP2LMatcher::from_config(HybridP2LMatcherConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "HybridP2L should converge for rotation");
        assert_relative_eq!(result.transform.theta, 0.2, epsilon = 0.06);
    }

    #[test]
    fn test_hybrid_p2l_known_combined_transform() {
        let source = create_room(100, 4.0, 3.0);
        let known_transform = Pose2D::new(0.1, 0.08, 0.15);
        let target = source.transform(&known_transform);

        let mut matcher = HybridP2LMatcher::from_config(HybridP2LMatcherConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(
            result.converged,
            "HybridP2L should converge for combined transform"
        );
        assert_relative_eq!(result.transform.x, 0.1, epsilon = 0.06);
        assert_relative_eq!(result.transform.y, 0.08, epsilon = 0.06);
        assert_relative_eq!(result.transform.theta, 0.15, epsilon = 0.06);
    }

    #[test]
    fn test_hybrid_p2l_with_odometry_initial_guess() {
        let source = create_room(100, 4.0, 3.0);
        let known_transform = Pose2D::new(0.25, 0.20, 0.3);
        let target = source.transform(&known_transform);

        // Simulate odometry providing a good initial guess
        let initial_guess = Pose2D::new(0.22, 0.18, 0.27);

        let mut matcher = HybridP2LMatcher::from_config(HybridP2LMatcherConfig::default());
        let result = matcher.match_scans(&source, &target, &initial_guess);

        assert!(
            result.converged,
            "HybridP2L should converge with odometry guess"
        );
        assert_relative_eq!(result.transform.x, 0.25, epsilon = 0.06);
        assert_relative_eq!(result.transform.y, 0.20, epsilon = 0.06);
        assert_relative_eq!(result.transform.theta, 0.3, epsilon = 0.06);
    }

    #[test]
    fn test_p2l_icp_known_rotation() {
        let source = create_room(100, 4.0, 3.0);
        let known_transform = Pose2D::new(0.0, 0.0, 0.15);
        let target = source.transform(&known_transform);

        let mut matcher = PointToLineIcp::new(PointToLineIcpConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "P2L ICP should converge for rotation");
        assert_relative_eq!(result.transform.theta, 0.15, epsilon = 0.04);
    }

    #[test]
    fn test_multi_resolution_known_transform() {
        let source = create_room(100, 4.0, 3.0);
        let known_transform = Pose2D::new(0.12, 0.08, 0.1);
        let target = source.transform(&known_transform);

        let mut matcher = MultiResolutionMatcher::new(MultiResolutionConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Multi-res should converge");
        assert_relative_eq!(result.transform.x, 0.12, epsilon = 0.05);
        assert_relative_eq!(result.transform.y, 0.08, epsilon = 0.05);
        assert_relative_eq!(result.transform.theta, 0.1, epsilon = 0.05);
    }

    #[test]
    fn test_correlative_large_initial_error() {
        let source = create_room(100, 4.0, 3.0);
        let known_transform = Pose2D::new(0.2, 0.15, 0.3);
        let target = source.transform(&known_transform);

        let mut matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(
            result.converged,
            "Correlative should handle large initial error"
        );
        assert_relative_eq!(result.transform.x, 0.2, epsilon = 0.06);
        assert_relative_eq!(result.transform.y, 0.15, epsilon = 0.06);
        assert_relative_eq!(result.transform.theta, 0.3, epsilon = 0.06);
    }

    // ========================================================================
    // Phase 4.2: Empty/Sparse Cloud Handling
    // ========================================================================

    #[test]
    fn test_icp_empty_source() {
        let source = PointCloud2D::new();
        let target = create_room(100, 4.0, 3.0);

        let mut matcher = PointToPointIcp::new(IcpConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(!result.converged, "ICP should fail on empty source");
    }

    #[test]
    fn test_icp_empty_target() {
        let source = create_room(100, 4.0, 3.0);
        let target = PointCloud2D::new();

        let mut matcher = PointToPointIcp::new(IcpConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(!result.converged, "ICP should fail on empty target");
    }

    #[test]
    fn test_icp_sparse_clouds() {
        // Very sparse: just 5 points on a line
        let source = create_line(5, 1.0);
        let target = create_line(5, 1.0);

        let mut matcher = PointToPointIcp::new(IcpConfig {
            min_correspondences: 3,
            ..Default::default()
        });
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        // May converge but with poor score due to insufficient constraints
        if result.converged {
            assert!(
                result.score < 0.9,
                "Sparse clouds should have lower confidence"
            );
        }
    }

    // ========================================================================
    // Scan Overlay Accumulation Tests
    // ========================================================================

    /// Test sequential scan overlay with small rotations (simulates robot rotating in place).
    ///
    /// This test catches accumulated rotation errors that would cause diagonal skew
    /// in SLAM maps.
    ///
    /// Transform semantics (from module docs):
    /// - source = current scan (robot's current position)
    /// - target = previous scan (robot's previous position)
    /// - returned transform = robot motion from previous to current
    ///
    /// When robot rotates +θ CCW:
    /// - Objects appear rotated -θ in robot's local frame (relative to previous)
    /// - current_scan has objects at -θ relative to previous_scan
    /// - Alignment transform to make current match previous is +θ
    /// - So returned transform.theta = +θ = robot motion ✓
    #[test]
    fn test_accumulated_rotation_error() {
        // Create a room-shaped scan (stable reference geometry)
        // This represents what a robot at pose (0,0,0) sees
        let reference_scan = create_room(200, 4.0, 3.0);

        // Simulate 36 small rotations of 10° each = 360° total
        let num_steps = 36;
        let step_rotation = std::f32::consts::PI / 18.0; // 10 degrees

        let mut matcher = HybridP2LMatcher::from_config(HybridP2LMatcherConfig::default());

        let mut accumulated_robot_theta = 0.0f32;
        let mut accumulated_error = 0.0f32;

        for i in 0..num_steps {
            // Robot motion: rotate by +step_rotation
            let robot_motion = Pose2D::new(0.0, 0.0, step_rotation);

            // Previous scan: what robot saw at accumulated_robot_theta
            // Objects appear at -accumulated_robot_theta in robot frame
            let previous_scan =
                reference_scan.transform(&Pose2D::new(0.0, 0.0, -accumulated_robot_theta));

            // Robot rotates by +step_rotation
            accumulated_robot_theta += step_rotation;

            // Current scan: what robot sees at new pose
            // Objects appear at -accumulated_robot_theta in robot frame
            let current_scan =
                reference_scan.transform(&Pose2D::new(0.0, 0.0, -accumulated_robot_theta));

            // Run scan matching with robot motion as initial guess
            let result = matcher.match_scans(&current_scan, &previous_scan, &robot_motion);

            assert!(
                result.converged,
                "Step {}: scan matching should converge",
                i
            );

            // The returned transform should equal the robot motion
            let rotation_error = (result.transform.theta - step_rotation).abs();
            accumulated_error += rotation_error;

            // Each step should have very small error
            assert!(
                rotation_error < 0.02, // ~1.1 degrees per step
                "Step {}: rotation error {:.3} rad ({:.1}°) too large, got {:.3} rad expected {:.3} rad",
                i,
                rotation_error,
                rotation_error.to_degrees(),
                result.transform.theta,
                step_rotation
            );
        }

        // Total accumulated error after full rotation should be small
        let avg_error_per_step = accumulated_error / num_steps as f32;
        println!(
            "Accumulated rotation error over 360°: {:.3} rad ({:.1}°), avg per step: {:.4} rad ({:.2}°)",
            accumulated_error,
            accumulated_error.to_degrees(),
            avg_error_per_step,
            avg_error_per_step.to_degrees()
        );

        // Total error should be less than 10° for 360° rotation
        assert!(
            accumulated_error < 0.175, // ~10 degrees
            "Total accumulated rotation error {:.3} rad ({:.1}°) too large",
            accumulated_error,
            accumulated_error.to_degrees()
        );
    }

    /// Test sequential translation with scan overlay (simulates robot driving straight).
    #[test]
    fn test_accumulated_translation_error() {
        // Create a corridor-like geometry (walls on both sides)
        // Add tiny y-variation to avoid kiddo bucket issues with collinear points
        let mut corridor = PointCloud2D::new();
        for i in 0..100 {
            let x = (i as f32 / 99.0) * 10.0; // 10m corridor
            let noise = (i as f32) * 0.0001; // Tiny variation
            corridor.push(Point2D::new(x, noise)); // Bottom wall
            corridor.push(Point2D::new(x, 2.0 + noise)); // Top wall
        }

        // Simulate 10 steps of 0.5m forward motion
        let num_steps = 10;
        let step_translation = 0.5f32;

        let mut matcher = HybridP2LMatcher::from_config(HybridP2LMatcherConfig::default());

        let mut accumulated_x = 0.0f32;
        let mut accumulated_error = 0.0f32;

        for i in 0..num_steps {
            // Ground truth robot motion: move forward
            let robot_motion = Pose2D::new(step_translation, 0.0, 0.0);

            // Previous scan: what robot saw at accumulated_x position
            // Robot at position X sees corridor shifted by -X in robot frame
            let previous_scan = corridor.transform(&Pose2D::new(-accumulated_x, 0.0, 0.0));

            // Robot moves forward
            accumulated_x += step_translation;

            // Current scan: what robot sees at new position
            let current_scan = corridor.transform(&Pose2D::new(-accumulated_x, 0.0, 0.0));

            // Run scan matching
            let result = matcher.match_scans(&current_scan, &previous_scan, &robot_motion);

            assert!(
                result.converged,
                "Step {}: scan matching should converge",
                i
            );

            // Check translation error
            let translation_error = ((result.transform.x - step_translation).powi(2)
                + result.transform.y.powi(2))
            .sqrt();
            accumulated_error += translation_error;

            assert!(
                translation_error < 0.05, // 5cm per step
                "Step {}: translation error {:.3}m too large, got ({:.3}, {:.3}) expected ({:.3}, 0)",
                i,
                translation_error,
                result.transform.x,
                result.transform.y,
                step_translation
            );
        }

        println!(
            "Accumulated translation error over {}m: {:.3}m",
            num_steps as f32 * step_translation,
            accumulated_error
        );

        // Total error should be less than 10% of distance traveled
        let total_distance = num_steps as f32 * step_translation;
        assert!(
            accumulated_error < total_distance * 0.1,
            "Total accumulated translation error {:.3}m > 10% of {:.1}m",
            accumulated_error,
            total_distance
        );
    }
}
