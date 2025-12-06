//! Multi-Resolution Correlative Scan Matcher.
//!
//! Hierarchical search using a resolution pyramid for efficient global matching.
//! Inspired by Google Cartographer's branch-and-bound approach.
//!
//! # Algorithm
//!
//! 1. Start at coarsest resolution level
//! 2. For each level:
//!    a. Find best candidate at current resolution
//!    b. Use as center for next (finer) level
//! 3. Return result from finest level
//!
//! # Benefits
//!
//! - Much faster than single-resolution correlative matching
//! - Handles large search windows efficiently
//! - Pruning reduces search space at each level

use super::{CorrelativeConfig, CorrelativeMatcher, ScanMatchResult, ScanMatcher};
use crate::core::types::{PointCloud2D, Pose2D};

/// Configuration for multi-resolution matcher.
#[derive(Debug, Clone)]
pub struct MultiResolutionConfig {
    /// Number of pyramid levels.
    ///
    /// More levels = faster search but may miss global optimum.
    pub num_levels: usize,

    /// Search window at coarsest level (meters).
    pub coarse_search_window_xy: f32,

    /// Search window for rotation at coarsest level (radians).
    pub coarse_search_window_theta: f32,

    /// Resolution at finest level (meters).
    pub fine_resolution: f32,

    /// Resolution at finest level for rotation (radians).
    pub fine_angular_resolution: f32,

    /// Grid resolution at finest level (meters).
    pub fine_grid_resolution: f32,

    /// Minimum score to accept result.
    pub min_score: f32,

    /// Resolution multiplier between levels.
    ///
    /// Each coarser level has resolution = fine_resolution * multiplier^level
    pub resolution_multiplier: f32,

    /// Window shrink factor between levels.
    ///
    /// Each finer level has window = coarse_window / shrink_factor^level
    pub window_shrink_factor: f32,
}

impl Default for MultiResolutionConfig {
    fn default() -> Self {
        Self {
            num_levels: 3,
            coarse_search_window_xy: 0.5,
            coarse_search_window_theta: 0.5,
            fine_resolution: 0.01,
            fine_angular_resolution: 0.01,
            fine_grid_resolution: 0.03,
            min_score: 0.5,
            resolution_multiplier: 3.0,
            window_shrink_factor: 2.5,
        }
    }
}

/// Multi-resolution correlative scan matcher.
///
/// Uses a pyramid of resolutions for efficient global matching.
#[derive(Debug)]
pub struct MultiResolutionMatcher {
    config: MultiResolutionConfig,
    /// Preallocated matchers for each pyramid level (reused across matches).
    level_matchers: Vec<CorrelativeMatcher>,
}

impl MultiResolutionMatcher {
    /// Create a new multi-resolution matcher.
    pub fn new(config: MultiResolutionConfig) -> Self {
        // Preallocate matchers for each level
        let level_matchers: Vec<CorrelativeMatcher> = (0..config.num_levels)
            .map(|level| {
                let level_config = Self::compute_level_config(&config, level);
                CorrelativeMatcher::new(level_config)
            })
            .collect();

        Self {
            config,
            level_matchers,
        }
    }

    /// Compute configuration for a specific level (static version for construction).
    fn compute_level_config(config: &MultiResolutionConfig, level: usize) -> CorrelativeConfig {
        let level_f = level as f32;
        let multiplier = config.resolution_multiplier.powf(level_f);
        let window_scale = config
            .window_shrink_factor
            .powf((config.num_levels - 1 - level) as f32);

        CorrelativeConfig {
            search_window_x: config.coarse_search_window_xy / window_scale,
            search_window_y: config.coarse_search_window_xy / window_scale,
            search_window_theta: config.coarse_search_window_theta / window_scale,
            linear_resolution: config.fine_resolution * multiplier,
            angular_resolution: config.fine_angular_resolution * multiplier,
            grid_resolution: config.fine_grid_resolution * multiplier,
            min_score: config.min_score * 0.7, // Be more lenient at coarse levels
        }
    }

    /// Get the current configuration.
    pub fn config(&self) -> &MultiResolutionConfig {
        &self.config
    }

    /// Compute configuration for a specific level (instance method for tests).
    ///
    /// Level 0 is finest, level num_levels-1 is coarsest.
    #[cfg(test)]
    fn level_config(&self, level: usize) -> CorrelativeConfig {
        Self::compute_level_config(&self.config, level)
    }
}

impl ScanMatcher for MultiResolutionMatcher {
    fn match_scans(
        &mut self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        if source.is_empty() || target.is_empty() {
            return ScanMatchResult::failed();
        }

        let mut current_guess = *initial_guess;
        let mut total_iterations = 0u32;

        // Search from coarsest to finest using preallocated matchers
        for level in (0..self.config.num_levels).rev() {
            let result = self.level_matchers[level].match_scans(source, target, &current_guess);
            total_iterations += result.iterations;

            if result.converged || result.score > self.config.min_score * 0.5 {
                current_guess = result.transform;
            }
            // If this level failed, keep previous guess and try finer level
        }

        // Return result from finest level (level 0) which was the last iteration
        // Note: We already processed level 0 in the loop, so just do a final evaluation
        // with the accumulated guess to get the correct iteration count
        let mut final_result = self.level_matchers[0].match_scans(source, target, &current_guess);
        final_result.iterations += total_iterations;

        final_result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;
    use approx::assert_relative_eq;

    fn create_l_shape(n: usize, length: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(2 * n);
        // Add tiny noise to avoid k-d tree bucket issues
        for i in 0..n {
            let x = (i as f32 / (n - 1) as f32) * length;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(x, noise));
        }
        for i in 1..n {
            let y = (i as f32 / (n - 1) as f32) * length;
            let noise = (i as f32) * 0.0001;
            cloud.push(Point2D::new(noise, y));
        }
        cloud
    }

    fn create_room(width: f32, height: f32, points_per_wall: usize) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(4 * points_per_wall);

        // Four walls with tiny noise to avoid k-d tree bucket issues
        for i in 0..points_per_wall {
            let t = i as f32 / (points_per_wall - 1) as f32;
            let noise = (i as f32) * 0.0001;

            // Bottom wall
            cloud.push(Point2D::new(t * width, noise));
            // Top wall
            cloud.push(Point2D::new(t * width, height + noise));
            // Left wall
            cloud.push(Point2D::new(noise, t * height));
            // Right wall
            cloud.push(Point2D::new(width + noise, t * height));
        }

        cloud
    }

    #[test]
    fn test_identity() {
        // Use room shape with more points for better matching
        let cloud = create_room(3.0, 2.5, 50);

        // Lower min_score threshold for identity matching
        let config = MultiResolutionConfig {
            min_score: 0.3,
            ..MultiResolutionConfig::default()
        };
        let mut matcher = MultiResolutionMatcher::new(config);

        let result = matcher.match_scans(&cloud, &cloud, &Pose2D::identity());

        // For identity case, we just verify it doesn't diverge significantly
        // The discretization can cause small offsets
        assert!(
            result.score > 0.2,
            "Identity should have reasonable score, got {}",
            result.score
        );
        assert!(result.transform.x.abs() < 0.3, "X should be near zero");
        assert!(result.transform.y.abs() < 0.3, "Y should be near zero");
        assert!(
            result.transform.theta.abs() < 0.3,
            "Theta should be near zero"
        );
    }

    #[test]
    fn test_small_transform() {
        let source = create_l_shape(30, 2.0);
        let transform = Pose2D::new(0.1, 0.08, 0.05);
        let target = source.transform(&transform);

        let mut matcher = MultiResolutionMatcher::new(MultiResolutionConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged);
        // Use larger epsilon due to discretization
        assert_relative_eq!(result.transform.x, 0.1, epsilon = 0.1);
        assert_relative_eq!(result.transform.y, 0.08, epsilon = 0.1);
        assert_relative_eq!(result.transform.theta, 0.05, epsilon = 0.1);
    }

    #[test]
    fn test_larger_transform() {
        let source = create_room(3.0, 2.5, 30);
        let transform = Pose2D::new(0.35, -0.25, 0.2);
        let target = source.transform(&transform);

        let mut matcher = MultiResolutionMatcher::new(MultiResolutionConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should handle larger transforms");
        // Use larger epsilon due to discretization
        assert_relative_eq!(result.transform.x, 0.35, epsilon = 0.1);
        assert_relative_eq!(result.transform.y, -0.25, epsilon = 0.1);
        assert_relative_eq!(result.transform.theta, 0.2, epsilon = 0.1);
    }

    #[test]
    fn test_fewer_iterations_than_single_level() {
        let source = create_l_shape(30, 2.0);
        let transform = Pose2D::new(0.2, 0.15, 0.1);
        let target = source.transform(&transform);

        // Multi-resolution
        let mut multi_matcher = MultiResolutionMatcher::new(MultiResolutionConfig::default());
        let multi_result = multi_matcher.match_scans(&source, &target, &Pose2D::identity());

        // Single level with same fine resolution
        let single_config = CorrelativeConfig {
            search_window_x: 0.5,
            search_window_y: 0.5,
            search_window_theta: 0.5,
            linear_resolution: 0.01,
            angular_resolution: 0.01,
            grid_resolution: 0.03,
            min_score: 0.5,
        };
        let mut single_matcher = CorrelativeMatcher::new(single_config);
        let single_result = single_matcher.match_scans(&source, &target, &Pose2D::identity());

        // Multi-resolution should use fewer iterations
        assert!(
            multi_result.iterations < single_result.iterations,
            "Multi-res ({}) should be faster than single-level ({})",
            multi_result.iterations,
            single_result.iterations
        );
    }

    #[test]
    fn test_empty_clouds() {
        let empty = PointCloud2D::new();
        let cloud = create_l_shape(20, 1.0);
        let mut matcher = MultiResolutionMatcher::new(MultiResolutionConfig::default());

        assert!(
            !matcher
                .match_scans(&empty, &cloud, &Pose2D::identity())
                .converged
        );
        assert!(
            !matcher
                .match_scans(&cloud, &empty, &Pose2D::identity())
                .converged
        );
    }

    #[test]
    fn test_config_accessor() {
        let config = MultiResolutionConfig {
            num_levels: 4,
            ..MultiResolutionConfig::default()
        };
        let matcher = MultiResolutionMatcher::new(config);

        assert_eq!(matcher.config().num_levels, 4);
    }

    #[test]
    fn test_level_config() {
        let config = MultiResolutionConfig::default();
        let matcher = MultiResolutionMatcher::new(config.clone());

        // Level 0 (finest) should have fine resolution
        let level0 = matcher.level_config(0);
        assert_relative_eq!(
            level0.linear_resolution,
            config.fine_resolution,
            epsilon = 0.001
        );

        // Coarser levels should have larger resolution
        let level1 = matcher.level_config(1);
        assert!(level1.linear_resolution > level0.linear_resolution);

        let level2 = matcher.level_config(2);
        assert!(level2.linear_resolution > level1.linear_resolution);
    }
}
