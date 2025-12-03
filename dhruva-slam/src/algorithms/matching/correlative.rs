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

/// Grid-based lookup table for fast scoring.
struct ScoreGrid {
    /// Grid cells: true if near a target point
    cells: Vec<bool>,
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
    /// Build a score grid from target point cloud.
    fn from_cloud(cloud: &PointCloud2D, resolution: f32, padding: f32) -> Self {
        if cloud.is_empty() {
            return Self {
                cells: vec![],
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

        let mut cells = vec![false; width * height];

        // Mark cells near target points
        for point in &cloud.points {
            let cx = ((point.x - origin_x) / resolution) as isize;
            let cy = ((point.y - origin_y) / resolution) as isize;

            // Mark this cell and neighbors for better hit detection
            for dx in -1..=1 {
                for dy in -1..=1 {
                    let nx = cx + dx;
                    let ny = cy + dy;
                    if nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height {
                        cells[ny as usize * width + nx as usize] = true;
                    }
                }
            }
        }

        Self {
            cells,
            width,
            height,
            origin_x,
            origin_y,
            resolution,
        }
    }

    /// Check if a point is near a target point.
    #[inline]
    fn is_hit(&self, x: f32, y: f32) -> bool {
        if self.cells.is_empty() {
            return false;
        }

        let cx = ((x - self.origin_x) / self.resolution) as isize;
        let cy = ((y - self.origin_y) / self.resolution) as isize;

        if cx >= 0 && cy >= 0 && (cx as usize) < self.width && (cy as usize) < self.height {
            self.cells[cy as usize * self.width + cx as usize]
        } else {
            false
        }
    }

    /// Score a transformed point cloud against this grid.
    fn score(&self, source: &PointCloud2D, transform: &Pose2D) -> f32 {
        if source.is_empty() {
            return 0.0;
        }

        let (sin_t, cos_t) = transform.theta.sin_cos();
        let mut hits = 0;

        for point in &source.points {
            let tx = transform.x + point.x * cos_t - point.y * sin_t;
            let ty = transform.y + point.x * sin_t + point.y * cos_t;

            if self.is_hit(tx, ty) {
                hits += 1;
            }
        }

        hits as f32 / source.len() as f32
    }
}

/// Correlative scan matcher.
///
/// Performs exhaustive search over pose space to find best alignment.
#[derive(Debug, Clone)]
pub struct CorrelativeMatcher {
    config: CorrelativeConfig,
}

impl CorrelativeMatcher {
    /// Create a new correlative matcher with the given configuration.
    pub fn new(config: CorrelativeConfig) -> Self {
        Self { config }
    }

    /// Get the current configuration.
    pub fn config(&self) -> &CorrelativeConfig {
        &self.config
    }

    /// Generate candidate poses within search window.
    fn generate_candidates(&self, center: &Pose2D) -> Vec<Pose2D> {
        let mut candidates = Vec::new();

        let x_steps = (self.config.search_window_x / self.config.linear_resolution).ceil() as i32;
        let y_steps = (self.config.search_window_y / self.config.linear_resolution).ceil() as i32;
        let t_steps =
            (self.config.search_window_theta / self.config.angular_resolution).ceil() as i32;

        for ti in -t_steps..=t_steps {
            let theta = center.theta + ti as f32 * self.config.angular_resolution;

            for xi in -x_steps..=x_steps {
                let x = center.x + xi as f32 * self.config.linear_resolution;

                for yi in -y_steps..=y_steps {
                    let y = center.y + yi as f32 * self.config.linear_resolution;

                    candidates.push(Pose2D::new(x, y, theta));
                }
            }
        }

        candidates
    }
}

impl ScanMatcher for CorrelativeMatcher {
    fn match_scans(
        &self,
        source: &PointCloud2D,
        target: &PointCloud2D,
        initial_guess: &Pose2D,
    ) -> ScanMatchResult {
        if source.is_empty() || target.is_empty() {
            return ScanMatchResult::failed();
        }

        // Build score grid from target
        let grid = ScoreGrid::from_cloud(
            target,
            self.config.grid_resolution,
            self.config.search_window_x.max(self.config.search_window_y),
        );

        // Generate candidates
        let candidates = self.generate_candidates(initial_guess);
        let num_candidates = candidates.len() as u32;

        // Find best candidate
        let mut best_score = 0.0f32;
        let mut best_pose = *initial_guess;

        for candidate in candidates {
            let score = grid.score(source, &candidate);

            if score > best_score {
                best_score = score;
                best_pose = candidate;
            }
        }

        // Check if score meets threshold
        if best_score >= self.config.min_score {
            ScanMatchResult {
                transform: best_pose,
                covariance: crate::core::types::Covariance2D::diagonal(
                    self.config.linear_resolution.powi(2),
                    self.config.linear_resolution.powi(2),
                    self.config.angular_resolution.powi(2),
                ),
                score: best_score,
                converged: true,
                iterations: num_candidates,
                mse: (1.0 - best_score) * self.config.grid_resolution.powi(2),
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

    fn create_L_shape(n: usize, length: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(2 * n);
        for i in 0..n {
            let x = (i as f32 / (n - 1) as f32) * length;
            cloud.push(Point2D::new(x, 0.0));
        }
        for i in 1..n {
            let y = (i as f32 / (n - 1) as f32) * length;
            cloud.push(Point2D::new(0.0, y));
        }
        cloud
    }

    #[test]
    fn test_identity_transform() {
        let cloud = create_L_shape(30, 1.5);
        let matcher = CorrelativeMatcher::new(CorrelativeConfig::default());

        let result = matcher.match_scans(&cloud, &cloud, &Pose2D::identity());

        assert!(result.converged);
        assert!(result.score > 0.8);
        assert_relative_eq!(result.transform.x, 0.0, epsilon = 0.05);
        assert_relative_eq!(result.transform.y, 0.0, epsilon = 0.05);
        assert_relative_eq!(result.transform.theta, 0.0, epsilon = 0.05);
    }

    #[test]
    fn test_translation_recovery() {
        let source = create_L_shape(30, 1.5);
        let transform = Pose2D::new(0.15, 0.10, 0.0);
        let target = source.transform(&transform);

        let matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should find translation");
        assert_relative_eq!(result.transform.x, 0.15, epsilon = 0.05);
        assert_relative_eq!(result.transform.y, 0.10, epsilon = 0.05);
    }

    #[test]
    fn test_rotation_recovery() {
        let source = create_L_shape(30, 1.5);
        let transform = Pose2D::new(0.0, 0.0, 0.15);
        let target = source.transform(&transform);

        let matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should find rotation");
        assert_relative_eq!(result.transform.theta, 0.15, epsilon = 0.05);
    }

    #[test]
    fn test_combined_transform_recovery() {
        let source = create_L_shape(30, 1.5);
        let transform = Pose2D::new(0.12, -0.08, 0.1);
        let target = source.transform(&transform);

        let matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should find combined transform");
        assert_relative_eq!(result.transform.x, 0.12, epsilon = 0.05);
        assert_relative_eq!(result.transform.y, -0.08, epsilon = 0.05);
        assert_relative_eq!(result.transform.theta, 0.1, epsilon = 0.05);
    }

    #[test]
    fn test_empty_clouds() {
        let empty = PointCloud2D::new();
        let cloud = create_L_shape(20, 1.0);
        let matcher = CorrelativeMatcher::new(CorrelativeConfig::default());

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
        let source = create_L_shape(30, 1.5);
        let transform = Pose2D::new(0.5, 0.5, 0.0); // Outside default ±0.3m window
        let target = source.transform(&transform);

        let matcher = CorrelativeMatcher::new(CorrelativeConfig::default());
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        // Should not converge because transform is outside search window
        assert!(
            result.score < 0.5,
            "Should not find transform outside window"
        );
    }

    #[test]
    fn test_wider_search_window() {
        let source = create_L_shape(30, 1.5);
        let transform = Pose2D::new(0.4, 0.4, 0.0);
        let target = source.transform(&transform);

        let config = CorrelativeConfig {
            search_window_x: 0.5,
            search_window_y: 0.5,
            linear_resolution: 0.03,
            ..CorrelativeConfig::default()
        };
        let matcher = CorrelativeMatcher::new(config);
        let result = matcher.match_scans(&source, &target, &Pose2D::identity());

        assert!(result.converged, "Should find transform with wider window");
        assert_relative_eq!(result.transform.x, 0.4, epsilon = 0.06);
        assert_relative_eq!(result.transform.y, 0.4, epsilon = 0.06);
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
}
