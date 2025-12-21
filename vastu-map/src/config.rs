//! Configuration for VectorMap SLAM.
//!
//! All configuration parameters have sensible defaults for indoor robot navigation.

use std::f32::consts::PI;

/// Lidar measurement noise model.
///
/// Models measurement uncertainty as a function of range:
/// σ(r) = σ_base + σ_range × r²
///
/// This allows weighting correspondences by their measurement quality.
/// Close-range measurements are more reliable than far-range ones.
#[derive(Clone, Copy, Debug)]
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

/// Configuration for VectorMap SLAM.
///
/// All parameters have sensible defaults. Create with `Default::default()`
/// and modify as needed.
#[derive(Clone, Debug)]
pub struct VectorMapConfig {
    // ─────────────────────────────────────────────────────────────────────────
    // Input Filtering
    // ─────────────────────────────────────────────────────────────────────────
    /// Minimum quality value for scan points (0-255).
    /// Points below this threshold are filtered out.
    /// Default: 10
    pub min_scan_quality: u8,

    /// Minimum valid range in meters.
    /// Points closer than this are filtered out.
    /// Default: 0.15m
    pub min_scan_range: f32,

    /// Maximum valid range in meters.
    /// Points farther than this are filtered out.
    /// Default: 12.0m
    pub max_scan_range: f32,

    // ─────────────────────────────────────────────────────────────────────────
    // Line Extraction (Split-and-Merge)
    // ─────────────────────────────────────────────────────────────────────────
    /// Maximum perpendicular distance from point to line for splitting.
    /// Points deviating more than this trigger a split.
    /// Default: 0.05m (5cm)
    pub split_distance_threshold: f32,

    /// Minimum line length to keep.
    /// Shorter lines are discarded.
    /// Default: 0.10m (10cm)
    pub min_line_length: f32,

    /// Minimum number of points to form a line.
    /// Default: 5
    pub min_points_per_line: usize,

    /// Maximum gap between consecutive points to be considered continuous.
    /// Gaps larger than this start a new line segment.
    /// Default: 0.30m (30cm)
    pub max_point_gap: f32,

    // ─────────────────────────────────────────────────────────────────────────
    // Corner Detection
    // ─────────────────────────────────────────────────────────────────────────
    /// Minimum angle between lines to form a corner (radians).
    /// Default: 30° (π/6)
    pub corner_angle_min: f32,

    /// Maximum angle between lines to form a corner (radians).
    /// Default: 150° (5π/6)
    pub corner_angle_max: f32,

    /// Maximum distance between line endpoints to form a corner.
    /// Default: 0.05m (5cm)
    pub corner_distance_threshold: f32,

    // ─────────────────────────────────────────────────────────────────────────
    // Line Association (Map Integration)
    // ─────────────────────────────────────────────────────────────────────────
    /// Maximum angular difference between line directions for association (radians).
    /// Default: 10° (π/18)
    pub association_angle_tolerance: f32,

    /// Maximum perpendicular distance between lines for association.
    /// Default: 0.05m (5cm)
    pub association_distance_tolerance: f32,

    /// Minimum overlap ratio required for merging lines.
    /// Default: 0.3 (30%)
    pub association_overlap_ratio: f32,

    // ─────────────────────────────────────────────────────────────────────────
    // Scan Matching
    // ─────────────────────────────────────────────────────────────────────────
    /// Maximum iterations for Point-to-Line ICP.
    /// Default: 50
    pub icp_max_iterations: usize,

    /// Convergence threshold for ICP (translation change in meters).
    /// ICP stops when pose change is below this.
    /// Default: 0.001m (1mm)
    pub icp_convergence_threshold: f32,

    /// Maximum rotation change for ICP convergence (radians).
    /// Default: 0.001 rad (~0.06°)
    pub icp_rotation_threshold: f32,

    /// Maximum correspondence distance for ICP.
    /// Points farther from any line are treated as outliers.
    /// Default: 0.30m (30cm)
    pub icp_max_correspondence_dist: f32,

    /// Minimum percentage of points with valid correspondences.
    /// Below this, scan matching is considered failed.
    /// Default: 0.3 (30%)
    pub icp_min_correspondence_ratio: f32,

    /// Number of RANSAC iterations for coarse alignment.
    /// Default: 100
    pub ransac_iterations: usize,

    /// Inlier distance threshold for RANSAC.
    /// Default: 0.05m (5cm)
    pub ransac_inlier_threshold: f32,

    /// Lidar noise model for correspondence weighting.
    /// Used to weight measurements by their estimated uncertainty.
    pub noise_model: LidarNoiseModel,

    // ─────────────────────────────────────────────────────────────────────────
    // Confidence Thresholds
    // ─────────────────────────────────────────────────────────────────────────
    /// Minimum confidence to accept a scan match.
    /// Below this, odometry is used instead.
    /// Default: 0.3
    pub min_match_confidence: f32,
}

impl Default for VectorMapConfig {
    fn default() -> Self {
        Self {
            // Input filtering
            min_scan_quality: 10,
            min_scan_range: 0.15,
            max_scan_range: 12.0,

            // Line extraction
            split_distance_threshold: 0.05,
            min_line_length: 0.10,
            min_points_per_line: 5,
            max_point_gap: 0.30,

            // Corner detection
            corner_angle_min: PI / 6.0,       // 30°
            corner_angle_max: 5.0 * PI / 6.0, // 150°
            corner_distance_threshold: 0.05,

            // Line association
            association_angle_tolerance: PI / 18.0, // 10°
            association_distance_tolerance: 0.05,
            association_overlap_ratio: 0.3,

            // Scan matching
            icp_max_iterations: 50,
            icp_convergence_threshold: 0.001,
            icp_rotation_threshold: 0.001,
            icp_max_correspondence_dist: 0.30,
            icp_min_correspondence_ratio: 0.3,
            ransac_iterations: 100,
            ransac_inlier_threshold: 0.05,
            noise_model: LidarNoiseModel::default(),

            // Confidence
            min_match_confidence: 0.3,
        }
    }
}

impl VectorMapConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for min_scan_quality.
    pub fn with_min_scan_quality(mut self, value: u8) -> Self {
        self.min_scan_quality = value;
        self
    }

    /// Builder-style setter for scan range.
    pub fn with_scan_range(mut self, min: f32, max: f32) -> Self {
        self.min_scan_range = min;
        self.max_scan_range = max;
        self
    }

    /// Builder-style setter for split distance threshold.
    pub fn with_split_threshold(mut self, value: f32) -> Self {
        self.split_distance_threshold = value;
        self
    }

    /// Builder-style setter for minimum line length.
    pub fn with_min_line_length(mut self, value: f32) -> Self {
        self.min_line_length = value;
        self
    }

    /// Builder-style setter for ICP parameters.
    pub fn with_icp_params(mut self, max_iterations: usize, convergence_threshold: f32) -> Self {
        self.icp_max_iterations = max_iterations;
        self.icp_convergence_threshold = convergence_threshold;
        self
    }

    /// Builder-style setter for RANSAC iterations.
    pub fn with_ransac_iterations(mut self, value: usize) -> Self {
        self.ransac_iterations = value;
        self
    }

    /// Builder-style setter for lidar noise model.
    pub fn with_noise_model(mut self, model: LidarNoiseModel) -> Self {
        self.noise_model = model;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = VectorMapConfig::default();

        assert_eq!(config.min_scan_quality, 10);
        assert_eq!(config.min_scan_range, 0.15);
        assert_eq!(config.max_scan_range, 12.0);
        assert_eq!(config.split_distance_threshold, 0.05);
        assert_eq!(config.min_line_length, 0.10);
        assert_eq!(config.min_points_per_line, 5);
        assert_eq!(config.icp_max_iterations, 50);
        assert_eq!(config.ransac_iterations, 100);
    }

    #[test]
    fn test_builder_pattern() {
        let config = VectorMapConfig::new()
            .with_min_scan_quality(20)
            .with_scan_range(0.10, 8.0)
            .with_split_threshold(0.03)
            .with_min_line_length(0.15)
            .with_icp_params(100, 0.0005)
            .with_ransac_iterations(200);

        assert_eq!(config.min_scan_quality, 20);
        assert_eq!(config.min_scan_range, 0.10);
        assert_eq!(config.max_scan_range, 8.0);
        assert_eq!(config.split_distance_threshold, 0.03);
        assert_eq!(config.min_line_length, 0.15);
        assert_eq!(config.icp_max_iterations, 100);
        assert_eq!(config.icp_convergence_threshold, 0.0005);
        assert_eq!(config.ransac_iterations, 200);
    }

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
    fn test_config_with_noise_model() {
        let custom_model = LidarNoiseModel::new(0.02, 0.002);
        let config = VectorMapConfig::new().with_noise_model(custom_model);

        assert_eq!(config.noise_model.sigma_base, 0.02);
        assert_eq!(config.noise_model.sigma_range, 0.002);
    }
}
