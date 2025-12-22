//! Scan preprocessing for point cloud filtering.
//!
//! Provides outlier removal and downsampling operations for lidar scans.
//!
//! # Example
//!
//! ```rust
//! use vastu_map::preprocessing::{PreprocessingConfig, preprocess, remove_outliers, downsample};
//! use vastu_map::core::PointCloud2D;
//!
//! // Create a point cloud with some outliers
//! let mut cloud = PointCloud2D::new();
//! cloud.push(0.0, 0.0);
//! cloud.push(0.1, 0.0);
//! cloud.push(0.2, 0.0);
//! cloud.push(10.0, 10.0); // Outlier
//!
//! // Remove outliers
//! let config = PreprocessingConfig::default();
//! let clean = remove_outliers(&cloud, &config);
//! assert!(clean.len() < cloud.len()); // Outlier removed
//! ```

use crate::core::PointCloud2D;
use std::collections::HashMap;

/// Configuration for scan preprocessing.
#[derive(Clone, Debug)]
pub struct PreprocessingConfig {
    /// Radius for neighbor search in outlier removal (meters).
    /// Default: 0.1m
    pub outlier_radius: f32,

    /// Minimum number of neighbors required to keep a point.
    /// Points with fewer neighbors are removed as outliers.
    /// Default: 2
    pub min_neighbors: usize,

    /// Voxel size for downsampling (meters).
    /// Set to 0.0 to disable downsampling.
    /// Default: 0.0 (disabled)
    pub downsample_resolution: f32,
}

impl Default for PreprocessingConfig {
    fn default() -> Self {
        Self {
            outlier_radius: 0.1,
            min_neighbors: 2,
            downsample_resolution: 0.0,
        }
    }
}

impl PreprocessingConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for outlier radius.
    pub fn with_outlier_radius(mut self, radius: f32) -> Self {
        self.outlier_radius = radius;
        self
    }

    /// Builder-style setter for minimum neighbors.
    pub fn with_min_neighbors(mut self, min: usize) -> Self {
        self.min_neighbors = min;
        self
    }

    /// Builder-style setter for downsample resolution.
    pub fn with_downsample_resolution(mut self, resolution: f32) -> Self {
        self.downsample_resolution = resolution;
        self
    }
}

/// Remove isolated outlier points using radius-based filtering.
///
/// Keeps points that have at least `config.min_neighbors` other points
/// within `config.outlier_radius` distance.
///
/// # Arguments
///
/// * `cloud` - Input point cloud
/// * `config` - Preprocessing configuration
///
/// # Returns
///
/// New point cloud with outliers removed.
///
/// # Complexity
///
/// O(n²) where n is the number of points. Sufficient for typical lidar scans (~360 points).
pub fn remove_outliers(cloud: &PointCloud2D, config: &PreprocessingConfig) -> PointCloud2D {
    let mut result = PointCloud2D::with_capacity(cloud.len());
    remove_outliers_into(cloud, config, &mut result);
    result
}

/// Zero-allocation variant of outlier removal.
///
/// Writes results to a pre-allocated output buffer.
pub fn remove_outliers_into(
    cloud: &PointCloud2D,
    config: &PreprocessingConfig,
    out: &mut PointCloud2D,
) {
    out.clear();

    if cloud.is_empty() {
        return;
    }

    let radius_sq = config.outlier_radius * config.outlier_radius;
    let n = cloud.len();

    // For each point, count neighbors within radius
    for i in 0..n {
        let xi = cloud.xs[i];
        let yi = cloud.ys[i];

        let mut neighbor_count = 0;

        for j in 0..n {
            if i == j {
                continue;
            }

            let dx = cloud.xs[j] - xi;
            let dy = cloud.ys[j] - yi;
            let dist_sq = dx * dx + dy * dy;

            if dist_sq <= radius_sq {
                neighbor_count += 1;
                if neighbor_count >= config.min_neighbors {
                    break; // Early exit - we have enough neighbors
                }
            }
        }

        if neighbor_count >= config.min_neighbors {
            out.push(xi, yi);
        }
    }
}

/// Reduce point density using voxel grid downsampling.
///
/// Divides space into grid cells of the given resolution and keeps
/// the first point encountered in each cell.
///
/// # Arguments
///
/// * `cloud` - Input point cloud
/// * `resolution` - Voxel grid cell size in meters
///
/// # Returns
///
/// Downsampled point cloud.
///
/// # Complexity
///
/// O(n) where n is the number of points.
pub fn downsample(cloud: &PointCloud2D, resolution: f32) -> PointCloud2D {
    let mut result = PointCloud2D::with_capacity(cloud.len());
    downsample_into(cloud, resolution, &mut result);
    result
}

/// Zero-allocation variant of downsampling.
///
/// Writes results to a pre-allocated output buffer.
pub fn downsample_into(cloud: &PointCloud2D, resolution: f32, out: &mut PointCloud2D) {
    out.clear();

    if cloud.is_empty() || resolution <= 0.0 {
        // If resolution is invalid, copy all points
        if resolution <= 0.0 {
            out.xs.extend_from_slice(&cloud.xs);
            out.ys.extend_from_slice(&cloud.ys);
        }
        return;
    }

    let inv_resolution = 1.0 / resolution;

    // Hash map to track which cells have been occupied
    // Key: (grid_x, grid_y) as i32 tuple converted to single i64
    let mut occupied: HashMap<(i32, i32), bool> = HashMap::with_capacity(cloud.len());

    for i in 0..cloud.len() {
        let x = cloud.xs[i];
        let y = cloud.ys[i];

        // Compute grid cell indices
        let gx = (x * inv_resolution).floor() as i32;
        let gy = (y * inv_resolution).floor() as i32;

        // Keep first point in each cell
        if occupied.insert((gx, gy), true).is_none() {
            out.push(x, y);
        }
    }
}

/// Apply full preprocessing pipeline.
///
/// Applies outlier removal followed by downsampling (if resolution > 0).
///
/// # Arguments
///
/// * `cloud` - Input point cloud
/// * `config` - Preprocessing configuration
///
/// # Returns
///
/// Preprocessed point cloud.
pub fn preprocess(cloud: &PointCloud2D, config: &PreprocessingConfig) -> PointCloud2D {
    let mut result = PointCloud2D::with_capacity(cloud.len());
    preprocess_into(cloud, config, &mut result);
    result
}

/// Zero-allocation variant of full preprocessing.
///
/// Uses an internal temporary buffer for intermediate results.
pub fn preprocess_into(cloud: &PointCloud2D, config: &PreprocessingConfig, out: &mut PointCloud2D) {
    // First, remove outliers
    let mut temp = PointCloud2D::with_capacity(cloud.len());
    remove_outliers_into(cloud, config, &mut temp);

    // Then, downsample if enabled
    if config.downsample_resolution > 0.0 {
        downsample_into(&temp, config.downsample_resolution, out);
    } else {
        // Just copy the outlier-filtered result
        out.clear();
        out.xs.extend_from_slice(&temp.xs);
        out.ys.extend_from_slice(&temp.ys);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_line_cloud() -> PointCloud2D {
        let mut cloud = PointCloud2D::new();
        // Line of points at y=0, x from 0 to 1 in 0.05 increments
        for i in 0..21 {
            cloud.push(i as f32 * 0.05, 0.0);
        }
        cloud
    }

    fn make_cloud_with_outlier() -> PointCloud2D {
        let mut cloud = make_line_cloud();
        // Add an outlier far from the line
        cloud.push(10.0, 10.0);
        cloud
    }

    #[test]
    fn test_config_default() {
        let config = PreprocessingConfig::default();
        assert_eq!(config.outlier_radius, 0.1);
        assert_eq!(config.min_neighbors, 2);
        assert_eq!(config.downsample_resolution, 0.0);
    }

    #[test]
    fn test_config_builder() {
        let config = PreprocessingConfig::new()
            .with_outlier_radius(0.2)
            .with_min_neighbors(3)
            .with_downsample_resolution(0.05);

        assert_eq!(config.outlier_radius, 0.2);
        assert_eq!(config.min_neighbors, 3);
        assert_eq!(config.downsample_resolution, 0.05);
    }

    #[test]
    fn test_remove_outliers_empty() {
        let cloud = PointCloud2D::new();
        let config = PreprocessingConfig::default();

        let result = remove_outliers(&cloud, &config);

        assert!(result.is_empty());
    }

    #[test]
    fn test_remove_outliers_keeps_dense_points() {
        let cloud = make_line_cloud();
        let config = PreprocessingConfig::default();

        let result = remove_outliers(&cloud, &config);

        // Most points should be kept (they have neighbors)
        // Edge points might be removed
        assert!(result.len() >= cloud.len() - 4);
    }

    #[test]
    fn test_remove_outliers_removes_isolated() {
        let cloud = make_cloud_with_outlier();
        let config = PreprocessingConfig::default();

        let result = remove_outliers(&cloud, &config);

        // The outlier at (10, 10) should be removed
        assert!(result.len() < cloud.len());

        // Check that the outlier is not in the result
        for i in 0..result.len() {
            assert!(result.xs[i] < 5.0, "Outlier should be removed");
        }
    }

    #[test]
    fn test_remove_outliers_into() {
        let cloud = make_cloud_with_outlier();
        let config = PreprocessingConfig::default();

        let mut result = PointCloud2D::new();
        remove_outliers_into(&cloud, &config, &mut result);

        assert!(result.len() < cloud.len());
    }

    #[test]
    fn test_downsample_empty() {
        let cloud = PointCloud2D::new();

        let result = downsample(&cloud, 0.1);

        assert!(result.is_empty());
    }

    #[test]
    fn test_downsample_zero_resolution() {
        let cloud = make_line_cloud();

        let result = downsample(&cloud, 0.0);

        // Should copy all points when resolution is 0
        assert_eq!(result.len(), cloud.len());
    }

    #[test]
    fn test_downsample_reduces_density() {
        let cloud = make_line_cloud(); // 21 points from 0 to 1 at 0.05 spacing

        // Downsample with 0.1 resolution - should reduce by ~half
        let result = downsample(&cloud, 0.1);

        assert!(result.len() < cloud.len());
        assert!(result.len() >= 10); // At least 10 cells from x=0 to x=1
    }

    #[test]
    fn test_downsample_into() {
        let cloud = make_line_cloud();

        let mut result = PointCloud2D::new();
        downsample_into(&cloud, 0.1, &mut result);

        assert!(result.len() < cloud.len());
    }

    #[test]
    fn test_downsample_preserves_first_point() {
        let mut cloud = PointCloud2D::new();
        cloud.push(0.0, 0.0);
        cloud.push(0.01, 0.01); // Same cell at 0.1 resolution
        cloud.push(0.02, 0.02); // Same cell

        let result = downsample(&cloud, 0.1);

        // Should keep only the first point
        assert_eq!(result.len(), 1);
        assert_eq!(result.xs[0], 0.0);
        assert_eq!(result.ys[0], 0.0);
    }

    #[test]
    fn test_preprocess_outlier_only() {
        let cloud = make_cloud_with_outlier();
        let config = PreprocessingConfig::default(); // downsample disabled

        let result = preprocess(&cloud, &config);

        // Should remove outlier
        assert!(result.len() < cloud.len());
    }

    #[test]
    fn test_preprocess_full_pipeline() {
        let cloud = make_cloud_with_outlier();
        let config = PreprocessingConfig::new()
            .with_outlier_radius(0.1)
            .with_min_neighbors(2)
            .with_downsample_resolution(0.1);

        let result = preprocess(&cloud, &config);

        // Should remove outlier AND downsample
        assert!(result.len() < cloud.len() - 5);
    }

    #[test]
    fn test_preprocess_into() {
        let cloud = make_cloud_with_outlier();
        let config = PreprocessingConfig::new().with_downsample_resolution(0.1);

        let mut result = PointCloud2D::new();
        preprocess_into(&cloud, &config, &mut result);

        assert!(result.len() < cloud.len());
    }

    #[test]
    fn test_negative_coordinates() {
        let mut cloud = PointCloud2D::new();
        cloud.push(-1.0, -1.0);
        cloud.push(-0.95, -0.95);
        cloud.push(-0.9, -0.9);
        cloud.push(1.0, 1.0);
        cloud.push(1.05, 1.05);
        cloud.push(1.1, 1.1);

        let config = PreprocessingConfig::new()
            .with_outlier_radius(0.2)
            .with_min_neighbors(2);

        let result = remove_outliers(&cloud, &config);

        // Should keep points in both clusters
        assert_eq!(result.len(), 6);
    }
}
