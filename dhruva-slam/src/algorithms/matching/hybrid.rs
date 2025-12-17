//! Generic hybrid matcher combining coarse and fine matchers.
//!
//! Provides a generic hybrid matching strategy that combines:
//! 1. A coarse matcher for handling large initial errors
//! 2. A fine matcher for sub-centimeter accuracy refinement
//!
//! This replaces the previous separate `HybridMatcher`, `HybridP2LMatcher`,
//! and `HybridMultiResMatcher` implementations with a single generic type.

use super::{ScanMatchResult, ScanMatcher};
use crate::core::types::{PointCloud2D, Pose2D};

/// Apply encoder weight to blend between identity and full odometry.
///
/// Blends the initial guess based on encoder_weight:
/// - 1.0: Trust odometry fully
/// - 0.0: Use identity (no prior motion)
/// - Intermediate values: Linear interpolation
#[inline]
pub fn weighted_initial_guess(odom_guess: &Pose2D, encoder_weight: f32) -> Pose2D {
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

/// Configuration for generic hybrid matcher.
#[derive(Debug, Clone)]
pub struct HybridConfig {
    /// If true, always run coarse matcher first.
    /// If false, only use coarse matcher when fine matcher fails.
    pub always_coarse: bool,

    /// Encoder weight for initial guess (0.0 = identity, 1.0 = full odometry).
    /// Higher values trust encoder odometry more for the initial guess.
    pub encoder_weight: f32,
}

impl Default for HybridConfig {
    fn default() -> Self {
        Self {
            always_coarse: true,
            encoder_weight: 1.0,
        }
    }
}

/// Generic hybrid matcher combining a coarse and fine matcher.
///
/// The matching strategy is:
/// 1. If `always_coarse` is true, run coarse matcher first to find initial alignment
/// 2. Refine with fine matcher for sub-centimeter accuracy
/// 3. If fine matcher fails and `always_coarse` is false, try coarse matcher as fallback
///
/// # Type Parameters
///
/// * `C` - Coarse matcher type (e.g., `CorrelativeMatcher`, `MultiResolutionMatcher`)
/// * `F` - Fine matcher type (e.g., `PointToPointIcp`, `PointToLineIcp`)
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::algorithms::matching::{
///     HybridMatcher, HybridConfig, CorrelativeMatcher, PointToLineIcp,
///     CorrelativeConfig, PointToLineIcpConfig,
/// };
///
/// let hybrid: HybridMatcher<CorrelativeMatcher, PointToLineIcp> = HybridMatcher::new(
///     CorrelativeMatcher::new(CorrelativeConfig::default()),
///     PointToLineIcp::new(PointToLineIcpConfig::default()),
///     HybridConfig::default(),
/// );
/// ```
#[derive(Debug)]
pub struct HybridMatcher<C: ScanMatcher, F: ScanMatcher> {
    coarse: C,
    fine: F,
    config: HybridConfig,
}

impl<C: ScanMatcher, F: ScanMatcher> HybridMatcher<C, F> {
    /// Create a new hybrid matcher.
    ///
    /// # Arguments
    ///
    /// * `coarse` - Coarse matcher for handling large initial errors
    /// * `fine` - Fine matcher for accurate refinement
    /// * `config` - Hybrid matcher configuration
    pub fn new(coarse: C, fine: F, config: HybridConfig) -> Self {
        Self {
            coarse,
            fine,
            config,
        }
    }
}

impl<C: ScanMatcher, F: ScanMatcher> ScanMatcher for HybridMatcher<C, F> {
    fn match_scans(
        &mut self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        // Apply encoder weight to initial guess
        let weighted_guess = weighted_initial_guess(initial_guess, self.config.encoder_weight);

        let guess = if self.config.always_coarse {
            // Always use coarse matcher for initial alignment
            let coarse_result = self.coarse.match_scans(source, target, &weighted_guess);
            if coarse_result.converged {
                coarse_result.transform
            } else {
                // Coarse failed, try fine with weighted guess
                weighted_guess
            }
        } else {
            weighted_guess
        };

        // Refine with fine matcher
        let fine_result = self.fine.match_scans(source, target, &guess);

        if fine_result.converged {
            fine_result
        } else if !self.config.always_coarse {
            // Fine failed without coarse, try coarse as fallback
            let coarse_result = self.coarse.match_scans(source, target, &weighted_guess);
            if coarse_result.converged {
                // Refine coarse result with fine matcher
                self.fine
                    .match_scans(source, target, &coarse_result.transform)
            } else {
                ScanMatchResult::failed()
            }
        } else {
            // Both coarse and fine failed
            ScanMatchResult::failed()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::matching::{
        CorrelativeConfig, CorrelativeMatcher, PointToLineIcp, PointToLineIcpConfig,
    };
    use crate::core::types::Point2D;
    use approx::assert_relative_eq;

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
    fn test_weighted_initial_guess_full_weight() {
        let odom = Pose2D::new(1.0, 2.0, 0.5);
        let result = weighted_initial_guess(&odom, 1.0);
        assert_relative_eq!(result.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(result.y, 2.0, epsilon = 1e-6);
        assert_relative_eq!(result.theta, 0.5, epsilon = 1e-6);
    }

    #[test]
    fn test_weighted_initial_guess_zero_weight() {
        let odom = Pose2D::new(1.0, 2.0, 0.5);
        let result = weighted_initial_guess(&odom, 0.0);
        assert_relative_eq!(result.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(result.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(result.theta, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_weighted_initial_guess_half_weight() {
        let odom = Pose2D::new(1.0, 2.0, 0.6);
        let result = weighted_initial_guess(&odom, 0.5);
        assert_relative_eq!(result.x, 0.5, epsilon = 1e-6);
        assert_relative_eq!(result.y, 1.0, epsilon = 1e-6);
        assert_relative_eq!(result.theta, 0.3, epsilon = 1e-6);
    }

    #[test]
    fn test_hybrid_matcher_large_rotation() {
        // Test large rotation (25°) - correlative should find it
        let source = create_room(100, 4.0, 3.0);
        let transform = Pose2D::new(0.0, 0.0, 0.44); // ~25 degrees
        let target = source.transform(&transform);

        let config = HybridConfig {
            always_coarse: true,
            encoder_weight: 1.0,
        };

        let mut matcher: HybridMatcher<CorrelativeMatcher, PointToLineIcp> = HybridMatcher::new(
            CorrelativeMatcher::new(CorrelativeConfig {
                search_window_x: 0.3,
                search_window_y: 0.3,
                search_window_theta: 0.5,
                linear_resolution: 0.03,
                angular_resolution: 0.03,
                grid_resolution: 0.05,
                min_score: 0.4,
            }),
            PointToLineIcp::new(PointToLineIcpConfig::default()),
            config,
        );

        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should converge for large rotation");
        assert_relative_eq!(result.transform.theta, 0.44, epsilon = 0.08);
    }

    #[test]
    fn test_hybrid_matcher_combined_large_motion() {
        // Test combined translation + rotation
        let source = create_room(100, 4.0, 3.0);
        let transform = Pose2D::new(0.2, 0.15, 0.35); // ~20 degrees + translation
        let target = source.transform(&transform);

        let config = HybridConfig::default();

        let mut matcher: HybridMatcher<CorrelativeMatcher, PointToLineIcp> = HybridMatcher::new(
            CorrelativeMatcher::new(CorrelativeConfig {
                search_window_x: 0.3,
                search_window_y: 0.3,
                search_window_theta: 0.5,
                linear_resolution: 0.03,
                angular_resolution: 0.03,
                grid_resolution: 0.05,
                min_score: 0.4,
            }),
            PointToLineIcp::new(PointToLineIcpConfig::default()),
            config,
        );

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
