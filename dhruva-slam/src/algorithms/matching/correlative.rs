//! Correlative Scan Matcher.
//!
//! Exhaustive search over a discretized pose space to find the best alignment.
//! Robust to large initial pose errors but slower than ICP.
//!
//! # Algorithm
//!
//! 1. Build a lookup table (grid) from target point cloud
//! 2. For each candidate pose (x, y, θ) in search window:
//!    a. Transform source points by candidate pose
//!    b. Score alignment by counting points near target
//! 3. Return pose with highest score
//!
//! # Use Cases
//!
//! - Initial alignment when odometry is unreliable
//! - Recovery from tracking loss
//! - Global localization in known map

use super::{ScanMatchResult, ScanMatcher};
use crate::core::simd::Float4;
use crate::core::types::{PointCloud2D, Pose2D};

/// Configuration for correlative scan matcher.
#[derive(Debug, Clone)]
pub struct CorrelativeConfig {
    /// Search window half-width in X (meters).
    pub search_window_x: f32,

    /// Search window half-width in Y (meters).
    pub search_window_y: f32,

    /// Search window half-width in theta (radians).
    pub search_window_theta: f32,

    /// Linear search resolution (meters).
    pub linear_resolution: f32,

    /// Angular search resolution (radians).
    pub angular_resolution: f32,

    /// Grid resolution for scoring (meters).
    ///
    /// Points within this distance of target are considered hits.
    pub grid_resolution: f32,

    /// Minimum score (fraction of points matched) to consider successful.
    pub min_score: f32,
}

impl Default for CorrelativeConfig {
    fn default() -> Self {
        Self {
            search_window_x: 0.3,     // ±30cm
            search_window_y: 0.3,     // ±30cm
            search_window_theta: 0.3, // ±17°
            linear_resolution: 0.02,  // 2cm steps
            angular_resolution: 0.02, // ~1.1° steps
            grid_resolution: 0.05,    // 5cm grid cells
            min_score: 0.5,           // At least 50% of points must match
        }
    }
}

/// Grid parameters for scoring (avoids too many function arguments).
#[derive(Clone, Copy)]
struct GridParams {
    origin_x: f32,
    origin_y: f32,
    width: usize,
    height: usize,
    resolution: f32,
}

/// Grid-based lookup table for fast scoring.
struct ScoreGrid {
    /// Grid dimensions
    width: usize,
    height: usize,
    /// Grid origin (minimum x, y)
    origin_x: f32,
    origin_y: f32,
    /// Cell size
    resolution: f32,
}

impl ScoreGrid {
    /// Build a score grid from target point cloud using an external buffer.
    ///
    /// The buffer is cleared, resized if needed, and populated with the grid data.
    fn from_cloud_into(
        cloud: &PointCloud2D,
        resolution: f32,
        padding: f32,
        buffer: &mut Vec<bool>,
    ) -> Self {
        if cloud.is_empty() {
            buffer.clear();
            return Self {
                width: 0,
                height: 0,
                origin_x: 0.0,
                origin_y: 0.0,
                resolution,
            };
        }

        // Find bounds
        let (min, max) = cloud.bounds().unwrap();
        let origin_x = min.x - padding;
        let origin_y = min.y - padding;
        let max_x = max.x + padding;
        let max_y = max.y + padding;

        let width = ((max_x - origin_x) / resolution).ceil() as usize + 1;
        let height = ((max_y - origin_y) / resolution).ceil() as usize + 1;
        let required_size = width * height;

        // Resize buffer if needed (amortized O(1) - only grows)
        if buffer.len() < required_size {
            buffer.resize(required_size, false);
        }

        // Clear relevant portion
        for cell in buffer[..required_size].iter_mut() {
            *cell = false;
        }

        // Mark cells near target points
        for i in 0..cloud.len() {
            let cx = ((cloud.xs[i] - origin_x) / resolution) as isize;
            let cy = ((cloud.ys[i] - origin_y) / resolution) as isize;

            // Mark this cell and neighbors for better hit detection
            for dx in -1..=1 {
                for dy in -1..=1 {
                    let nx = cx + dx;
                    let ny = cy + dy;
                    if nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height {
                        buffer[ny as usize * width + nx as usize] = true;
                    }
                }
            }
        }

        Self {
            width,
            height,
            origin_x,
            origin_y,
            resolution,
        }
    }
}

/// Correlative scan matcher.
///
/// Performs exhaustive search over pose space to find best alignment.
#[derive(Debug)]
pub struct CorrelativeMatcher {
    config: CorrelativeConfig,
    /// Preallocated grid buffer (reused across matches).
    grid_buffer: Vec<bool>,
    /// Preallocated buffer for transformed cloud (reused across matches).
    transformed_xs: Vec<f32>,
    transformed_ys: Vec<f32>,
}

impl CorrelativeMatcher {
    /// Typical scan size for pre-allocation.
    const TYPICAL_SCAN_POINTS: usize = 360;

    /// Create a new correlative matcher with the given configuration.
    pub fn new(config: CorrelativeConfig) -> Self {
        // Pre-allocate grid buffer for typical room size (~10m x 10m at 5cm resolution)
        // 200 x 200 = 40,000 cells
        Self {
            config,
            grid_buffer: Vec::with_capacity(40_000),
            transformed_xs: Vec::with_capacity(Self::TYPICAL_SCAN_POINTS),
            transformed_ys: Vec::with_capacity(Self::TYPICAL_SCAN_POINTS),
        }
    }

    /// Get the current configuration.
    pub fn config(&self) -> &CorrelativeConfig {
        &self.config
    }

    /// SIMD-accelerated rotation transform of point cloud.
    ///
    /// Rotates source points by theta and stores in preallocated buffers.
    /// Translation is applied separately during scoring for efficiency.
    #[inline]
    fn transform_rotate_simd(&mut self, source: &PointCloud2D, theta: f32) {
        let n = source.len();

        // Ensure buffers are large enough
        self.transformed_xs.clear();
        self.transformed_ys.clear();
        self.transformed_xs.resize(n, 0.0);
        self.transformed_ys.resize(n, 0.0);

        let (sin_t, cos_t) = theta.sin_cos();
        let sin_v = Float4::splat(sin_t);
        let cos_v = Float4::splat(cos_t);

        let chunks = n / 4;

        // Process 4 points at a time with SIMD
        for i in 0..chunks {
            let base = i * 4;

            // Load 4 x and y coordinates
            let xs = Float4::new(source.xs[base..base + 4].try_into().unwrap());
            let ys = Float4::new(source.ys[base..base + 4].try_into().unwrap());

            // Rotate: x' = x*cos - y*sin, y' = x*sin + y*cos
            let new_xs = xs * cos_v - ys * sin_v;
            let new_ys = xs * sin_v + ys * cos_v;

            // Store results
            let xs_arr = new_xs.to_array();
            let ys_arr = new_ys.to_array();
            self.transformed_xs[base..base + 4].copy_from_slice(&xs_arr);
            self.transformed_ys[base..base + 4].copy_from_slice(&ys_arr);
        }

        // Handle remainder with scalar
        for i in (chunks * 4)..n {
            self.transformed_xs[i] = source.xs[i] * cos_t - source.ys[i] * sin_t;
            self.transformed_ys[i] = source.xs[i] * sin_t + source.ys[i] * cos_t;
        }
    }

    /// Score pre-rotated points against grid with translation offset.
    ///
    /// Uses SIMD for index computation but scalar for grid lookup.
    /// Takes grid parameters directly to avoid borrow checker issues.
    #[inline]
    fn score_with_grid_params(
        &self,
        grid_cells: &[bool],
        params: GridParams,
        tx: f32,
        ty: f32,
    ) -> f32 {
        if self.transformed_xs.is_empty() || grid_cells.is_empty() {
            return 0.0;
        }

        let n = self.transformed_xs.len();
        let mut hits = 0u32;

        let inv_res = 1.0 / params.resolution;

        // SIMD setup for index calculation
        let tx_v = Float4::splat(tx);
        let ty_v = Float4::splat(ty);
        let ox_v = Float4::splat(params.origin_x);
        let oy_v = Float4::splat(params.origin_y);
        let inv_res_v = Float4::splat(inv_res);

        let chunks = n / 4;

        // Process 4 points at a time
        for i in 0..chunks {
            let base = i * 4;

            // Load pre-rotated coordinates
            let rxs = Float4::new(self.transformed_xs[base..base + 4].try_into().unwrap());
            let rys = Float4::new(self.transformed_ys[base..base + 4].try_into().unwrap());

            // Apply translation and compute grid indices
            let world_xs = rxs + tx_v;
            let world_ys = rys + ty_v;
            let cell_xs = (world_xs - ox_v) * inv_res_v;
            let cell_ys = (world_ys - oy_v) * inv_res_v;

            // Convert to integer indices and check bounds
            let cxs = cell_xs.to_array();
            let cys = cell_ys.to_array();

            for j in 0..4 {
                let cx = cxs[j] as isize;
                let cy = cys[j] as isize;

                if cx >= 0
                    && cy >= 0
                    && (cx as usize) < params.width
                    && (cy as usize) < params.height
                    && grid_cells[cy as usize * params.width + cx as usize]
                {
                    hits += 1;
                }
            }
        }

        // Handle remainder
        for i in (chunks * 4)..n {
            let world_x = self.transformed_xs[i] + tx;
            let world_y = self.transformed_ys[i] + ty;
            let cx = ((world_x - params.origin_x) * inv_res) as isize;
            let cy = ((world_y - params.origin_y) * inv_res) as isize;

            if cx >= 0
                && cy >= 0
                && (cx as usize) < params.width
                && (cy as usize) < params.height
                && grid_cells[cy as usize * params.width + cx as usize]
            {
                hits += 1;
            }
        }

        hits as f32 / n as f32
    }
}

impl ScanMatcher for CorrelativeMatcher {
    fn match_scans(
        &mut self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        if source.is_empty() || target.is_empty() {
            return ScanMatchResult::failed();
        }

        // Extract config values to avoid borrowing self during iteration
        let grid_resolution = self.config.grid_resolution;
        let linear_resolution = self.config.linear_resolution;
        let angular_resolution = self.config.angular_resolution;
        let search_window_x = self.config.search_window_x;
        let search_window_y = self.config.search_window_y;
        let search_window_theta = self.config.search_window_theta;
        let min_score = self.config.min_score;

        // Build score grid from target using preallocated buffer
        // Take ownership of grid buffer to allow using self for other operations
        let mut grid_buffer = std::mem::take(&mut self.grid_buffer);
        let grid = ScoreGrid::from_cloud_into(
            target,
            grid_resolution,
            search_window_x.max(search_window_y),
            &mut grid_buffer,
        );

        // Extract grid properties for scoring
        let grid_params = GridParams {
            origin_x: grid.origin_x,
            origin_y: grid.origin_y,
            width: grid.width,
            height: grid.height,
            resolution: grid.resolution,
        };

        // Calculate search steps
        let x_steps = (search_window_x / linear_resolution).ceil() as i32;
        let y_steps = (search_window_y / linear_resolution).ceil() as i32;
        let t_steps = (search_window_theta / angular_resolution).ceil() as i32;

        let num_candidates = ((2 * t_steps + 1) * (2 * x_steps + 1) * (2 * y_steps + 1)) as u32;

        // Find best candidate using optimized theta-first iteration
        // For each theta: transform once (SIMD), then score all (x,y) translations
        let mut best_score = 0.0f32;
        let mut best_pose = *initial_guess;

        for ti in -t_steps..=t_steps {
            let theta = initial_guess.theta + ti as f32 * angular_resolution;

            // SIMD-accelerated rotation transform (done once per theta)
            self.transform_rotate_simd(source, theta);

            // Score all (x, y) translations at this theta
            for xi in -x_steps..=x_steps {
                let x = initial_guess.x + xi as f32 * linear_resolution;

                for yi in -y_steps..=y_steps {
                    let y = initial_guess.y + yi as f32 * linear_resolution;

                    // Inline scoring to avoid borrow issues
                    let score = self.score_with_grid_params(&grid_buffer, grid_params, x, y);

                    if score > best_score {
                        best_score = score;
                        best_pose = Pose2D::new(x, y, theta);
                    }
                }
            }
        }

        // Return grid buffer for reuse
        self.grid_buffer = grid_buffer;

        // Check if score meets threshold
        if best_score >= min_score {
            ScanMatchResult {
                transform: best_pose,
                covariance: crate::core::types::Covariance2D::diagonal(
                    linear_resolution.powi(2),
                    linear_resolution.powi(2),
                    angular_resolution.powi(2),
                ),
                score: best_score,
                converged: true,
                iterations: num_candidates,
                mse: (1.0 - best_score) * grid_resolution.powi(2),
            }
        } else {
            ScanMatchResult {
                transform: best_pose,
                covariance: crate::core::types::Covariance2D::diagonal(1.0, 1.0, 0.5),
                score: best_score,
                converged: false,
                iterations: num_candidates,
                mse: (1.0 - best_score),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;
    use approx::assert_relative_eq;

    fn create_l_shape(n: usize, length: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(2 * n);
        // Add tiny noise to avoid k-d tree bucket issues with perfectly aligned points
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

    #[test]
    fn test_identity_transform() {
        let cloud = create_l_shape(30, 1.5);
        let mut matcher = CorrelativeMatcher::new(CorrelativeConfig::default());

        let result = matcher.match_scans(&cloud, &cloud, &Pose2D::identity());

        assert!(result.converged);
        assert!(result.score > 0.8);
        // Correlative matcher is discretized, so results may not be exactly zero
        // Default resolution is 2cm linear, 2° angular
        assert_relative_eq!(result.transform.x, 0.0, epsilon = 0.1);
        assert_relative_eq!(result.transform.y, 0.0, epsilon = 0.1);
        assert_relative_eq!(result.transform.theta, 0.0, epsilon = 0.1);
    }

    #[test]
    fn test_translation_recovery() {
        let source = create_l_shape(30, 1.5);
        // Use transform that aligns with grid resolution
        let transform = Pose2D::new(0.10, 0.08, 0.0);
        let target = source.transform(&transform);

        let mut matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should find translation");
        // Use larger epsilon due to grid discretization
        assert_relative_eq!(result.transform.x, 0.10, epsilon = 0.1);
        assert_relative_eq!(result.transform.y, 0.08, epsilon = 0.1);
    }

    #[test]
    fn test_rotation_recovery() {
        let source = create_l_shape(30, 1.5);
        // Use transform that aligns with angular resolution
        let transform = Pose2D::new(0.0, 0.0, 0.10);
        let target = source.transform(&transform);

        let mut matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should find rotation");
        // Use larger epsilon due to angular discretization
        assert_relative_eq!(result.transform.theta, 0.10, epsilon = 0.1);
    }

    #[test]
    fn test_combined_transform_recovery() {
        let source = create_l_shape(30, 1.5);
        // Use transform that aligns with grid resolution
        let transform = Pose2D::new(0.10, -0.06, 0.08);
        let target = source.transform(&transform);

        let mut matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should find combined transform");
        // Use larger epsilon due to discretization
        assert_relative_eq!(result.transform.x, 0.10, epsilon = 0.1);
        assert_relative_eq!(result.transform.y, -0.06, epsilon = 0.1);
        assert_relative_eq!(result.transform.theta, 0.08, epsilon = 0.1);
    }

    #[test]
    fn test_empty_clouds() {
        let empty = PointCloud2D::new();
        let cloud = create_l_shape(20, 1.0);
        let mut matcher = CorrelativeMatcher::new(CorrelativeConfig::default());

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
    fn test_search_window() {
        // Transform outside search window should not be found
        let source = create_l_shape(30, 1.5);
        let transform = Pose2D::new(0.5, 0.5, 0.0); // Outside default ±0.3m window
        let target = source.transform(&transform);

        let mut matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        // Should not converge because transform is outside search window
        assert!(
            result.score < 0.5,
            "Should not find transform outside window"
        );
    }

    #[test]
    fn test_wider_search_window() {
        let source = create_l_shape(30, 1.5);
        let transform = Pose2D::new(0.4, 0.4, 0.0);
        let target = source.transform(&transform);

        let config = CorrelativeConfig {
            search_window_x: 0.5,
            search_window_y: 0.5,
            linear_resolution: 0.03,
            ..CorrelativeConfig::default()
        };
        let mut matcher = CorrelativeMatcher::new(config);
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should find transform with wider window");
        // Use larger epsilon due to coarser grid resolution (0.03)
        assert_relative_eq!(result.transform.x, 0.4, epsilon = 0.1);
        assert_relative_eq!(result.transform.y, 0.4, epsilon = 0.1);
    }

    #[test]
    fn test_config_accessor() {
        let config = CorrelativeConfig {
            search_window_x: 0.5,
            ..CorrelativeConfig::default()
        };
        let matcher = CorrelativeMatcher::new(config);

        assert_eq!(matcher.config().search_window_x, 0.5);
    }

    // ========================================================================
    // Convergence and Quality Tests
    // ========================================================================
    //
    // Note: Correlative scan matching is a grid-based exhaustive search.
    // Its accuracy is fundamentally limited by the grid resolution.
    // These tests verify convergence behavior, not strict accuracy.

    #[test]
    fn test_correlative_converges_for_small_transform() {
        // Verify correlative matcher can find a small translation
        let source = create_l_shape(40, 3.0);
        let true_transform = Pose2D::new(0.10, 0.0, 0.0);
        let target = source.transform(&true_transform);

        let config = CorrelativeConfig {
            linear_resolution: 0.02,
            angular_resolution: 0.02,
            ..CorrelativeConfig::default()
        };
        let mut matcher = CorrelativeMatcher::new(config);
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        // Should converge and find approximate transform
        assert!(result.converged, "Correlative should converge");
        assert!(result.score > 0.5, "Should achieve reasonable score");

        // Transform should be in the right direction (positive X)
        assert!(
            result.transform.x > 0.0,
            "Should find positive X transform, got {}",
            result.transform.x
        );
    }

    #[test]
    fn test_correlative_handles_various_transforms() {
        // Verify convergence for various transform types
        let source = create_l_shape(40, 3.0);

        let test_cases = [
            ("pure X translation", Pose2D::new(0.06, 0.0, 0.0)),
            ("pure Y translation", Pose2D::new(0.0, 0.06, 0.0)),
            ("pure rotation", Pose2D::new(0.0, 0.0, 0.06)),
            ("combined", Pose2D::new(0.04, 0.04, 0.04)),
        ];

        let mut matcher = CorrelativeMatcher::new(CorrelativeConfig::default());

        for (name, true_transform) in &test_cases {
            let target = source.transform(true_transform);
            let result = matcher.match_scans(&source, &target, &Pose2D::identity());

            assert!(
                result.converged,
                "Should converge for {}: {:?}",
                name, true_transform
            );
            assert!(
                result.score > 0.5,
                "Should achieve reasonable score for {}, got {}",
                name,
                result.score
            );
        }
    }

    #[test]
    fn test_correlative_resolution_affects_search() {
        // Higher resolution should search more finely
        let source = create_l_shape(40, 3.0);
        let true_transform = Pose2D::new(0.06, 0.04, 0.04);
        let target = source.transform(&true_transform);

        // Coarse resolution
        let coarse_config = CorrelativeConfig {
            linear_resolution: 0.04,
            angular_resolution: 0.04,
            ..CorrelativeConfig::default()
        };
        let mut coarse = CorrelativeMatcher::new(coarse_config);
        let coarse_result = coarse.match_scans(&source, &target, &Pose2D::identity());

        // Fine resolution
        let fine_config = CorrelativeConfig {
            linear_resolution: 0.02,
            angular_resolution: 0.02,
            ..CorrelativeConfig::default()
        };
        let mut fine = CorrelativeMatcher::new(fine_config);
        let fine_result = fine.match_scans(&source, &target, &Pose2D::identity());

        // Both should converge
        assert!(coarse_result.converged, "Coarse matcher should converge");
        assert!(fine_result.converged, "Fine matcher should converge");

        // Both should achieve reasonable scores
        assert!(
            coarse_result.score > 0.5,
            "Coarse should have reasonable score"
        );
        assert!(fine_result.score > 0.5, "Fine should have reasonable score");
    }

    #[test]
    fn test_correlative_score_quality() {
        let source = create_l_shape(30, 2.0);

        // Perfect match should have high score
        let mut matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let perfect = matcher.match_scans(&source, &source, &Pose2D::identity());

        assert!(
            perfect.score > 0.9,
            "Perfect alignment should have score > 0.9, got {:.3}",
            perfect.score
        );

        // Small offset should still have reasonable score
        let offset_target = source.transform(&Pose2D::new(0.04, 0.02, 0.02));
        let offset_result = matcher.match_scans(&source, &offset_target, &Pose2D::identity());

        assert!(
            offset_result.converged && offset_result.score > 0.6,
            "Small offset should converge with score > 0.6, got {:.3}",
            offset_result.score
        );
    }
}
