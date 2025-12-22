//! Configuration for VectorMap SLAM.
//!
//! All configuration parameters have sensible defaults for indoor robot navigation.

use serde::{Deserialize, Serialize};

/// Lidar measurement noise model.
///
/// Models measurement uncertainty as a function of range:
/// σ(r) = σ_base + σ_range × r²
///
/// This allows weighting correspondences by their measurement quality.
/// Close-range measurements are more reliable than far-range ones.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct LidarNoiseModel {
    /// Base measurement standard deviation (meters).
    /// Minimum uncertainty even at zero range.
    /// Default: 0.01m (1cm)
    pub sigma_base: f32,

    /// Range-dependent noise coefficient.
    /// Uncertainty grows quadratically with range.
    /// Default: 0.001 (1mm per m² of range)
    pub sigma_range: f32,
}

impl Default for LidarNoiseModel {
    fn default() -> Self {
        Self {
            sigma_base: 0.01,   // 1cm base uncertainty
            sigma_range: 0.001, // 1mm per m²
        }
    }
}

impl LidarNoiseModel {
    /// Create a noise model with custom parameters.
    pub fn new(sigma_base: f32, sigma_range: f32) -> Self {
        Self {
            sigma_base,
            sigma_range,
        }
    }

    /// Compute measurement standard deviation at a given range.
    #[inline]
    pub fn sigma(&self, range: f32) -> f32 {
        self.sigma_base + self.sigma_range * range * range
    }

    /// Compute weight for a measurement at given range.
    /// Weight = 1 / σ²
    #[inline]
    pub fn weight(&self, range: f32) -> f32 {
        let sigma = self.sigma(range);
        1.0 / (sigma * sigma)
    }

    /// Compute weights for a batch of ranges.
    pub fn weights(&self, ranges: &[f32]) -> Vec<f32> {
        ranges.iter().map(|&r| self.weight(r)).collect()
    }

    /// Compute weights for a batch of ranges into a pre-allocated buffer (zero-allocation).
    ///
    /// # Panics
    /// Panics if `weights.len() != ranges.len()`.
    ///
    /// # Example
    /// ```rust,ignore
    /// let model = LidarNoiseModel::default();
    /// let ranges = vec![1.0, 2.0, 3.0];
    /// let mut weights = vec![0.0; ranges.len()];
    ///
    /// model.compute_weights_into(&ranges, &mut weights);
    /// ```
    #[inline]
    pub fn compute_weights_into(&self, ranges: &[f32], weights: &mut [f32]) {
        debug_assert_eq!(
            ranges.len(),
            weights.len(),
            "weights buffer must match ranges length"
        );
        for (r, w) in ranges.iter().zip(weights.iter_mut()) {
            *w = self.weight(*r);
        }
    }

    /// Compute weights for a PointCloud2D based on point distances from origin.
    ///
    /// This is useful for weighting correspondences during scan matching.
    ///
    /// # Arguments
    /// * `xs` - X coordinates of points
    /// * `ys` - Y coordinates of points
    /// * `weights` - Output buffer for weights (must match points length)
    ///
    /// # Example
    /// ```rust,ignore
    /// let model = LidarNoiseModel::default();
    /// let cloud = PointCloud2D::from_points(&points);
    /// let mut weights = vec![0.0; cloud.len()];
    ///
    /// model.compute_weights_for_cloud(&cloud.xs, &cloud.ys, &mut weights);
    /// ```
    #[inline]
    pub fn compute_weights_for_cloud(&self, xs: &[f32], ys: &[f32], weights: &mut [f32]) {
        debug_assert_eq!(xs.len(), ys.len(), "xs and ys must have same length");
        debug_assert_eq!(
            xs.len(),
            weights.len(),
            "weights buffer must match points length"
        );

        for i in 0..xs.len() {
            let range = (xs[i] * xs[i] + ys[i] * ys[i]).sqrt();
            weights[i] = self.weight(range);
        }
    }
}

/// Configuration for exploration mode with point-cloud-based line re-fitting.
///
/// When exploration mode is enabled, VectorMap stores raw scan data and
/// periodically re-fits lines from accumulated point clouds. This eliminates
/// first-scan bias in line extraction.
///
/// # Example
/// ```rust,ignore
/// use vastu_map::config::ExplorationConfig;
///
/// let config = ExplorationConfig::default()
///     .with_refit_interval(10)
///     .with_gap_threshold(0.3);
///
/// map.enable_exploration_mode(config);
/// ```
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExplorationConfig {
    /// Re-fit lines every N observations.
    /// Default: 10
    pub refit_interval: usize,

    /// Minimum points per line to trigger re-fitting.
    /// Lines with fewer associated points keep their current geometry.
    /// Default: 10
    pub min_points_per_line: usize,

    /// Maximum perpendicular distance for point-to-line association (meters).
    /// Points farther than this from any line are not associated.
    /// Default: 0.10m (10cm)
    pub max_point_distance: f32,

    /// Gap threshold for splitting lines (meters).
    /// If consecutive points along a line have a gap larger than this,
    /// the line is split into multiple segments.
    /// Default: 0.30m (30cm)
    pub gap_threshold: f32,

    /// Noise model for range-based weighting during re-fitting.
    /// Closer points get higher weight.
    pub noise_model: LidarNoiseModel,
}

impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            refit_interval: 10,
            min_points_per_line: 10,
            max_point_distance: 0.10,
            gap_threshold: 0.30,
            noise_model: LidarNoiseModel::default(),
        }
    }
}

impl ExplorationConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the re-fit interval (every N observations).
    pub fn with_refit_interval(mut self, interval: usize) -> Self {
        self.refit_interval = interval;
        self
    }

    /// Set the minimum points per line for re-fitting.
    pub fn with_min_points(mut self, min_points: usize) -> Self {
        self.min_points_per_line = min_points;
        self
    }

    /// Set the maximum point-to-line distance for association.
    pub fn with_max_point_distance(mut self, distance: f32) -> Self {
        self.max_point_distance = distance;
        self
    }

    /// Set the gap threshold for line splitting.
    pub fn with_gap_threshold(mut self, gap: f32) -> Self {
        self.gap_threshold = gap;
        self
    }

    /// Set the noise model for range-based weighting.
    pub fn with_noise_model(mut self, model: LidarNoiseModel) -> Self {
        self.noise_model = model;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lidar_noise_model_default() {
        let model = LidarNoiseModel::default();
        assert_eq!(model.sigma_base, 0.01);
        assert_eq!(model.sigma_range, 0.001);
    }

    #[test]
    fn test_lidar_noise_model_sigma() {
        let model = LidarNoiseModel::default();

        // At 0m range, sigma = sigma_base = 0.01
        assert!((model.sigma(0.0) - 0.01).abs() < 1e-6);

        // At 1m range, sigma = 0.01 + 0.001 * 1 = 0.011
        assert!((model.sigma(1.0) - 0.011).abs() < 1e-6);

        // At 5m range, sigma = 0.01 + 0.001 * 25 = 0.035
        assert!((model.sigma(5.0) - 0.035).abs() < 1e-6);
    }

    #[test]
    fn test_lidar_noise_model_weight() {
        let model = LidarNoiseModel::default();

        // Weight at 0m = 1 / (0.01)^2 = 10000
        let w0 = model.weight(0.0);
        assert!((w0 - 10000.0).abs() < 1.0);

        // Weight decreases with range
        let w1 = model.weight(1.0);
        let w5 = model.weight(5.0);
        assert!(w0 > w1);
        assert!(w1 > w5);
    }

    #[test]
    fn test_lidar_noise_model_weights_batch() {
        let model = LidarNoiseModel::default();
        let ranges = vec![0.5, 1.0, 2.0, 5.0];
        let weights = model.weights(&ranges);

        assert_eq!(weights.len(), 4);
        // Weights should decrease with range
        assert!(weights[0] > weights[1]);
        assert!(weights[1] > weights[2]);
        assert!(weights[2] > weights[3]);
    }

    #[test]
    fn test_compute_weights_into() {
        let model = LidarNoiseModel::default();
        let ranges = vec![0.5, 1.0, 2.0, 5.0];
        let mut weights = vec![0.0; ranges.len()];

        model.compute_weights_into(&ranges, &mut weights);

        // Should match allocating version
        let expected = model.weights(&ranges);
        for (w, e) in weights.iter().zip(expected.iter()) {
            assert!((w - e).abs() < 1e-6);
        }
    }

    #[test]
    fn test_compute_weights_for_cloud() {
        let model = LidarNoiseModel::default();
        let xs = vec![1.0, 0.0, 3.0, 4.0];
        let ys = vec![0.0, 2.0, 4.0, 0.0];
        let mut weights = vec![0.0; xs.len()];

        model.compute_weights_for_cloud(&xs, &ys, &mut weights);

        // Check against manual range computation
        let ranges: Vec<f32> = xs
            .iter()
            .zip(ys.iter())
            .map(|(x, y)| (x * x + y * y).sqrt())
            .collect();
        let expected = model.weights(&ranges);

        for (w, e) in weights.iter().zip(expected.iter()) {
            assert!((w - e).abs() < 1e-6);
        }
    }

    #[test]
    fn test_exploration_config_default() {
        let config = ExplorationConfig::default();

        assert_eq!(config.refit_interval, 10);
        assert_eq!(config.min_points_per_line, 10);
        assert!((config.max_point_distance - 0.10).abs() < 1e-6);
        assert!((config.gap_threshold - 0.30).abs() < 1e-6);
    }

    #[test]
    fn test_exploration_config_builder() {
        let config = ExplorationConfig::new()
            .with_refit_interval(20)
            .with_min_points(15)
            .with_max_point_distance(0.15)
            .with_gap_threshold(0.5);

        assert_eq!(config.refit_interval, 20);
        assert_eq!(config.min_points_per_line, 15);
        assert!((config.max_point_distance - 0.15).abs() < 1e-6);
        assert!((config.gap_threshold - 0.5).abs() < 1e-6);
    }
}
