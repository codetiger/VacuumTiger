//! Scan preprocessing module (Phase 3).
//!
//! Provides filters and converters for transforming raw LiDAR scans
//! into clean point clouds ready for scan matching.
//!
//! # Pipeline
//!
//! The typical preprocessing pipeline:
//!
//! ```text
//! LaserScan → RangeFilter → OutlierFilter → AngularDownsampler → PointCloud2D
//! ```
//!
//! # ScanFilter Trait
//!
//! All scan filters implement the [`ScanFilter`] trait for consistent interface:
//!
//! ```ignore
//! use dhruva_slam::preprocessing::{ScanFilter, RangeFilter, RangeFilterConfig};
//!
//! let filter = RangeFilter::new(RangeFilterConfig::default());
//! let filtered = filter.filter(&scan);
//! println!("Filter '{}' applied", filter.name());
//! ```
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::preprocessing::{ScanPreprocessor, PreprocessorConfig};
//! use dhruva_slam::types::LaserScan;
//!
//! let config = PreprocessorConfig::default();
//! let preprocessor = ScanPreprocessor::new(config);
//!
//! let scan = LaserScan::from_lidar_scan(&lidar_data);
//! let cloud = preprocessor.process(&scan);
//!
//! println!("Processed {} points to {} points", scan.len(), cloud.len());
//! ```

mod converter;
mod downsampler;
mod dynamic_filter;
mod outlier;
mod range_filter;

pub use converter::ScanConverter;
pub use downsampler::{AngularDownsampler, AngularDownsamplerConfig};
pub use dynamic_filter::{DynamicFilter, DynamicFilterConfig, DynamicFilterStats};
pub use outlier::{OutlierFilter, OutlierFilterConfig};
pub use range_filter::{RangeFilter, RangeFilterConfig};

use crate::core::types::{LaserScan, PointCloud2D};

/// Trait for scan filtering operations.
///
/// All scan filters implement this trait, providing a consistent interface
/// for applying filters to laser scans.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::preprocessing::{ScanFilter, RangeFilter};
///
/// fn apply_filter<F: ScanFilter>(filter: &F, scan: &LaserScan) -> LaserScan {
///     println!("Applying filter: {}", filter.name());
///     filter.filter(scan)
/// }
/// ```
pub trait ScanFilter: Send + Sync {
    /// Apply the filter to a laser scan, returning a filtered scan.
    fn filter(&self, scan: &LaserScan) -> LaserScan;

    /// Get the name of this filter for diagnostics.
    fn name(&self) -> &'static str;
}

// Implement ScanFilter for RangeFilter
impl ScanFilter for RangeFilter {
    fn filter(&self, scan: &LaserScan) -> LaserScan {
        self.apply(scan)
    }

    fn name(&self) -> &'static str {
        "RangeFilter"
    }
}

// Implement ScanFilter for OutlierFilter
impl ScanFilter for OutlierFilter {
    fn filter(&self, scan: &LaserScan) -> LaserScan {
        self.apply(scan)
    }

    fn name(&self) -> &'static str {
        "OutlierFilter"
    }
}

// Implement ScanFilter for AngularDownsampler
impl ScanFilter for AngularDownsampler {
    fn filter(&self, scan: &LaserScan) -> LaserScan {
        self.apply(scan)
    }

    fn name(&self) -> &'static str {
        "AngularDownsampler"
    }
}

/// Lidar mounting offset from robot center.
///
/// When the lidar is not mounted at the robot's center of rotation,
/// scan points need to be transformed to the robot frame.
///
/// This follows the standard SLAM approach (GMapping, Hector SLAM, Cartographer, SLAM Toolbox)
/// where a TF transform from `laser_frame` to `base_link` is provided.
///
/// The offset has two components:
/// 1. **Mounting position** (robot-specific): Where the lidar housing is mounted (X, Y translation)
/// 2. **Optical offset** (sensor-specific): Distance from housing center to laser origin (radial)
///
/// The optical offset is the distance from the lidar's geometric center to the rotating
/// scanner's optical center. This offset is in the **radial direction** (along each laser beam),
/// so it must be added to range measurements, not as a fixed X/Y translation.
///
/// # Coordinate Frame (ROS REP-103)
/// - X positive = forward
/// - Y positive = left
/// - Theta positive = counter-clockwise
///
/// # Example
/// ```ignore
/// // Lidar housing is 93.6mm behind robot center
/// // Rotating scanner is 25.8mm from housing geometric center (radial)
/// let offset = LidarOffset {
///     mounting_x: -0.0936,       // 93.6mm behind (calibrated)
///     mounting_y: 0.0,           // housing centered on Y axis
///     optical_offset: 0.0258,    // 25.8mm radial offset (added to ranges)
///     angle_offset: 0.0,
/// };
/// ```
#[derive(Debug, Clone, Copy)]
pub struct LidarOffset {
    /// Lidar housing X position relative to robot center (meters).
    /// Positive = forward, negative = behind.
    pub mounting_x: f32,

    /// Lidar housing Y position relative to robot center (meters).
    /// Positive = left, negative = right.
    pub mounting_y: f32,

    /// Optical center offset from housing geometric center (meters).
    /// This is sensor-specific - the distance from the housing center to the
    /// rotating scanner's optical center. This offset is RADIAL (along each
    /// laser beam direction), so it's added to range measurements.
    /// Positive = laser origin is closer to housing center than measured ranges indicate.
    pub optical_offset: f32,

    /// Angular offset of lidar's 0° relative to robot forward (radians).
    /// Positive = counter-clockwise when viewed from above.
    pub angle_offset: f32,
}

impl Default for LidarOffset {
    fn default() -> Self {
        Self {
            mounting_x: 0.0,
            mounting_y: 0.0,
            optical_offset: 0.0,
            angle_offset: 0.0,
        }
    }
}

impl LidarOffset {
    /// Create a new lidar offset with all parameters.
    pub fn new(mounting_x: f32, mounting_y: f32, optical_offset: f32, angle_offset: f32) -> Self {
        Self {
            mounting_x,
            mounting_y,
            optical_offset,
            angle_offset,
        }
    }

    /// Create from simple (x, y, theta) values for backwards compatibility.
    pub fn from_total(x: f32, y: f32, theta: f32) -> Self {
        Self {
            mounting_x: x,
            mounting_y: y,
            optical_offset: 0.0,
            angle_offset: theta,
        }
    }

    /// Get the mounting X offset.
    #[inline]
    pub fn total_x(&self) -> f32 {
        self.mounting_x
    }

    /// Get the mounting Y offset.
    #[inline]
    pub fn total_y(&self) -> f32 {
        self.mounting_y
    }

    /// Get total angular offset.
    #[inline]
    pub fn total_theta(&self) -> f32 {
        self.angle_offset
    }

    /// Get the radial optical offset (to be added to range measurements).
    #[inline]
    pub fn radial_offset(&self) -> f32 {
        self.optical_offset
    }

    /// Check if offset is effectively zero (no transform needed).
    pub fn is_zero(&self) -> bool {
        self.total_x().abs() < 1e-6
            && self.total_y().abs() < 1e-6
            && self.total_theta().abs() < 1e-6
            && self.optical_offset.abs() < 1e-6
    }
}

/// Configuration for the scan preprocessor pipeline.
#[derive(Debug, Clone, Default)]
pub struct PreprocessorConfig {
    /// Range filter configuration
    pub range_filter: RangeFilterConfig,
    /// Outlier filter configuration
    pub outlier_filter: OutlierFilterConfig,
    /// Downsampler configuration
    pub downsampler: AngularDownsamplerConfig,
    /// Lidar mounting offset from robot center
    pub lidar_offset: LidarOffset,
}

/// Complete scan preprocessing pipeline.
///
/// Combines range filtering, outlier removal, downsampling, and
/// polar-to-Cartesian conversion into a single processor.
///
/// Also applies lidar offset correction to transform points from
/// lidar frame to robot frame.
pub struct ScanPreprocessor {
    range_filter: RangeFilter,
    outlier_filter: OutlierFilter,
    downsampler: AngularDownsampler,
    lidar_offset: LidarOffset,
}

impl ScanPreprocessor {
    /// Create a new preprocessor with the given configuration.
    pub fn new(config: PreprocessorConfig) -> Self {
        Self {
            range_filter: RangeFilter::new(config.range_filter),
            outlier_filter: OutlierFilter::new(config.outlier_filter),
            downsampler: AngularDownsampler::new(config.downsampler),
            lidar_offset: config.lidar_offset,
        }
    }

    /// Process a laser scan through the full pipeline.
    ///
    /// Steps:
    /// 1. Apply range filter (remove invalid ranges)
    /// 2. Apply outlier filter (remove noise spikes)
    /// 3. Apply downsampling (reduce point count)
    /// 4. Convert to Cartesian point cloud (with radial optical offset correction)
    /// 5. Apply lidar mounting offset transform (lidar frame → robot frame)
    pub fn process(&self, scan: &LaserScan) -> PointCloud2D {
        // Step 1: Range filter
        let filtered = self.range_filter.apply(scan);

        // Step 2: Outlier filter
        let denoised = self.outlier_filter.apply(&filtered);

        // Step 3: Downsample
        let downsampled = self.downsampler.apply(&denoised);

        // Step 4: Convert to point cloud (apply radial optical offset to ranges)
        let cloud = ScanConverter::to_point_cloud_with_offset(
            &downsampled,
            self.lidar_offset.radial_offset(),
        );

        // Step 5: Apply lidar mounting offset transform (X/Y translation + rotation)
        self.apply_mounting_offset(cloud)
    }

    /// Process scan without downsampling (for debugging/visualization).
    pub fn process_full_resolution(&self, scan: &LaserScan) -> PointCloud2D {
        let filtered = self.range_filter.apply(scan);
        let denoised = self.outlier_filter.apply(&filtered);
        let cloud =
            ScanConverter::to_point_cloud_with_offset(&denoised, self.lidar_offset.radial_offset());
        self.apply_mounting_offset(cloud)
    }

    /// Apply only range filtering and conversion.
    pub fn process_minimal(&self, scan: &LaserScan) -> PointCloud2D {
        let filtered = self.range_filter.apply(scan);
        let cloud =
            ScanConverter::to_point_cloud_with_offset(&filtered, self.lidar_offset.radial_offset());
        self.apply_mounting_offset(cloud)
    }

    /// Apply lidar mounting offset transform to convert from lidar frame to robot frame.
    ///
    /// The lidar housing is mounted at position (mounting_x, mounting_y) relative to
    /// robot center, with rotation angle_offset. Points in lidar frame need to be:
    /// 1. Rotated by angle_offset
    /// 2. Translated by (mounting_x, mounting_y)
    ///
    /// Note: The radial optical offset is applied separately during polar-to-Cartesian
    /// conversion (added to range measurements), not here.
    ///
    /// This transforms: P_robot = R(theta) * P_lidar + mounting_offset
    fn apply_mounting_offset(&self, mut cloud: PointCloud2D) -> PointCloud2D {
        // Check if mounting offset is zero (skip radial offset check - it's handled elsewhere)
        let has_mounting_offset = self.lidar_offset.total_x().abs() >= 1e-6
            || self.lidar_offset.total_y().abs() >= 1e-6
            || self.lidar_offset.total_theta().abs() >= 1e-6;

        if !has_mounting_offset {
            return cloud;
        }

        let (sin_t, cos_t) = self.lidar_offset.total_theta().sin_cos();
        let ox = self.lidar_offset.total_x();
        let oy = self.lidar_offset.total_y();

        for i in 0..cloud.len() {
            let x = cloud.xs[i];
            let y = cloud.ys[i];

            // Rotate then translate: P_robot = R * P_lidar + mounting_offset
            cloud.xs[i] = x * cos_t - y * sin_t + ox;
            cloud.ys[i] = x * sin_t + y * cos_t + oy;
        }

        cloud
    }

    /// Get the configured lidar offset.
    pub fn lidar_offset(&self) -> &LidarOffset {
        &self.lidar_offset
    }
}

impl Default for ScanPreprocessor {
    fn default() -> Self {
        Self::new(PreprocessorConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_scan(n_points: usize) -> LaserScan {
        use std::f32::consts::TAU;

        let angle_increment = TAU / n_points as f32;
        let ranges: Vec<f32> = (0..n_points)
            .map(|i| {
                // Simulate a circular room at 5m radius with some noise
                5.0 + 0.1 * (i as f32 * 0.1).sin()
            })
            .collect();

        LaserScan::new(
            0.0,
            TAU - angle_increment,
            angle_increment,
            0.15,
            12.0,
            ranges,
        )
    }

    #[test]
    fn test_preprocessor_reduces_points() {
        let scan = create_test_scan(500);
        let preprocessor = ScanPreprocessor::default();

        let cloud = preprocessor.process(&scan);

        // Should reduce from 500 to approximately 200
        assert!(cloud.len() < scan.len());
        assert!(cloud.len() > 100);
        assert!(cloud.len() < 300);
    }

    #[test]
    fn test_preprocessor_full_resolution() {
        let scan = create_test_scan(500);
        let preprocessor = ScanPreprocessor::default();

        let cloud = preprocessor.process_full_resolution(&scan);

        // Should have similar point count (minus invalid/outliers)
        assert!(cloud.len() > 400); // Most points should survive
    }

    #[test]
    fn test_empty_scan() {
        let scan = LaserScan::default();
        let preprocessor = ScanPreprocessor::default();

        let cloud = preprocessor.process(&scan);

        assert!(cloud.is_empty());
    }
}
