//! Configuration for VectorMap SLAM.
//!
//! All configuration parameters have sensible defaults for indoor robot navigation.

use std::f32::consts::PI;

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
}
