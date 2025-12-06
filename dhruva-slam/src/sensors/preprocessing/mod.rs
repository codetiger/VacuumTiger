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

/// Configuration for the scan preprocessor pipeline.
#[derive(Debug, Clone, Default)]
pub struct PreprocessorConfig {
    /// Range filter configuration
    pub range_filter: RangeFilterConfig,
    /// Outlier filter configuration
    pub outlier_filter: OutlierFilterConfig,
    /// Downsampler configuration
    pub downsampler: AngularDownsamplerConfig,
}

/// Complete scan preprocessing pipeline.
///
/// Combines range filtering, outlier removal, downsampling, and
/// polar-to-Cartesian conversion into a single processor.
pub struct ScanPreprocessor {
    range_filter: RangeFilter,
    outlier_filter: OutlierFilter,
    downsampler: AngularDownsampler,
}

impl ScanPreprocessor {
    /// Create a new preprocessor with the given configuration.
    pub fn new(config: PreprocessorConfig) -> Self {
        Self {
            range_filter: RangeFilter::new(config.range_filter),
            outlier_filter: OutlierFilter::new(config.outlier_filter),
            downsampler: AngularDownsampler::new(config.downsampler),
        }
    }

    /// Process a laser scan through the full pipeline.
    ///
    /// Steps:
    /// 1. Apply range filter (remove invalid ranges)
    /// 2. Apply outlier filter (remove noise spikes)
    /// 3. Apply downsampling (reduce point count)
    /// 4. Convert to Cartesian point cloud
    pub fn process(&self, scan: &LaserScan) -> PointCloud2D {
        // Step 1: Range filter
        let filtered = self.range_filter.apply(scan);

        // Step 2: Outlier filter
        let denoised = self.outlier_filter.apply(&filtered);

        // Step 3: Downsample
        let downsampled = self.downsampler.apply(&denoised);

        // Step 4: Convert to point cloud
        ScanConverter::to_point_cloud(&downsampled)
    }

    /// Process scan without downsampling (for debugging/visualization).
    pub fn process_full_resolution(&self, scan: &LaserScan) -> PointCloud2D {
        let filtered = self.range_filter.apply(scan);
        let denoised = self.outlier_filter.apply(&filtered);
        ScanConverter::to_point_cloud(&denoised)
    }

    /// Apply only range filtering and conversion.
    pub fn process_minimal(&self, scan: &LaserScan) -> PointCloud2D {
        let filtered = self.range_filter.apply(scan);
        ScanConverter::to_point_cloud(&filtered)
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
