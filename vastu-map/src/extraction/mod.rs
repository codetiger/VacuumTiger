//! Feature extraction from point clouds.
//!
//! This module provides algorithms for extracting geometric features
//! (lines and corners) from lidar point clouds.
//!
//! # Algorithms
//!
//! - **Line Fitting**: Total Least Squares (orthogonal regression), with weighted variant
//! - **Line Extraction**: Split-and-Merge algorithm for ordered point sequences
//! - **RANSAC Lines**: Random Sample Consensus for robust line extraction
//! - **Hybrid Extraction**: RANSAC for dominant lines, then split-merge for remaining
//! - **Corner Detection**: Find corners at line intersections
//!
//! # Weighted Line Fitting
//!
//! For lidar data, measurement uncertainty increases with range. The weighted
//! TLS functions allow incorporating this range-based uncertainty:
//!
//! ```rust,ignore
//! use vastu_map::extraction::fit_line_from_sensor;
//! use vastu_map::core::Point2D;
//!
//! let sensor_pos = Point2D::new(0.0, 0.0);
//! let points = vec![/* lidar hits */];
//! let line = fit_line_from_sensor(&points, sensor_pos, None);
//! ```
//!
//! # Hybrid Extraction
//!
//! The hybrid approach combines RANSAC's robustness with split-merge's
//! ability to handle structured point sequences:
//!
//! ```rust,ignore
//! use vastu_map::extraction::{extract_lines_hybrid, RansacLineConfig, SplitMergeConfig};
//!
//! let ransac_config = RansacLineConfig::default();
//! let split_merge_config = SplitMergeConfig::default();
//! let lines = extract_lines_hybrid(&points, &ransac_config, &split_merge_config);
//! ```

use crate::core::Point2D;
use crate::features::Line2D;

pub mod corner_detection;
pub mod line_fitting;
pub mod ransac_lines;
pub mod split_merge;
pub mod traits;

pub use corner_detection::{
    CornerConfig, deduplicate_corners, detect_all_corners, detect_corners,
    detect_curvature_corners, detect_hybrid_corners,
};
pub use line_fitting::{
    compute_range_weights, fit_line, fit_line_from_sensor, fit_line_segment, fit_line_weighted,
    fitting_error, fitting_error_weighted, max_distance_point,
};
// Re-export from ransac_lines module
pub use ransac_lines::{
    RansacLineConfig, RansacScratchSpace, extract_lines_ransac, extract_lines_ransac_with_scratch,
};
// Re-export from split_merge submodule
pub use split_merge::{
    SplitMergeConfig, adaptive_split_threshold, extract_lines, extract_lines_from_sensor,
};
// Re-export traits
pub use traits::{LineExtractor, SplitMergeExtractor};

/// Extract lines using hybrid RANSAC + split-merge approach.
///
/// This method combines RANSAC's robustness to outliers with split-merge's
/// ability to handle structured point sequences:
///
/// 1. **RANSAC first**: Extracts dominant lines that may span across gaps
/// 2. **Split-merge second**: Processes remaining points for additional structure
///
/// This is useful when:
/// - Point cloud has significant outliers (RANSAC handles these)
/// - Some features are weak/fragmented (split-merge catches these)
/// - You want the best of both algorithms
///
/// # Arguments
/// * `points` - Input point cloud
/// * `ransac_config` - Configuration for RANSAC extraction
/// * `split_merge_config` - Configuration for split-merge extraction
///
/// # Returns
/// Combined list of extracted lines from both algorithms.
///
/// # Example
/// ```rust,ignore
/// use vastu_map::extraction::{extract_lines_hybrid, RansacLineConfig, SplitMergeConfig};
///
/// let ransac_config = RansacLineConfig::default()
///     .with_max_lines(5)  // Limit RANSAC to 5 dominant lines
///     .with_inlier_threshold(0.02);
///
/// let split_merge_config = SplitMergeConfig::default();
///
/// let all_lines = extract_lines_hybrid(&points, &ransac_config, &split_merge_config);
/// ```
pub fn extract_lines_hybrid(
    points: &[Point2D],
    ransac_config: &RansacLineConfig,
    split_merge_config: &SplitMergeConfig,
) -> Vec<Line2D> {
    if points.is_empty() {
        return Vec::new();
    }

    // Step 1: RANSAC for dominant lines
    let (mut lines, remaining_indices) = extract_lines_ransac(points, ransac_config);

    // Step 2: Split-merge on remaining points
    if remaining_indices.len() >= 3 {
        let remaining_points: Vec<Point2D> = remaining_indices.iter().map(|&i| points[i]).collect();

        let additional_lines = extract_lines(&remaining_points, split_merge_config);
        lines.extend(additional_lines);
    }

    lines
}

/// Extract lines using hybrid RANSAC + split-merge with pre-allocated scratch space.
///
/// This is the zero-allocation version for real-time applications.
/// Reuse the scratch space across multiple calls to eliminate allocations.
///
/// # Arguments
/// * `points` - Input point cloud
/// * `ransac_config` - Configuration for RANSAC extraction
/// * `split_merge_config` - Configuration for split-merge extraction
/// * `scratch` - Pre-allocated RANSAC scratch space
/// * `output_lines` - Buffer to store output lines (cleared and filled)
///
/// # Example
/// ```rust,ignore
/// let mut scratch = RansacScratchSpace::with_capacity(400);
/// let mut lines = Vec::new();
///
/// for scan in scans {
///     extract_lines_hybrid_with_scratch(&scan, &ransac_cfg, &sm_cfg, &mut scratch, &mut lines);
///     // lines contains extracted features, scratch is reused
/// }
/// ```
pub fn extract_lines_hybrid_with_scratch(
    points: &[Point2D],
    ransac_config: &RansacLineConfig,
    split_merge_config: &SplitMergeConfig,
    scratch: &mut RansacScratchSpace,
    output_lines: &mut Vec<Line2D>,
) {
    output_lines.clear();

    if points.is_empty() {
        return;
    }

    // Step 1: RANSAC for dominant lines (using scratch space)
    let (ransac_lines, remaining_indices) =
        extract_lines_ransac_with_scratch(points, ransac_config, scratch);
    output_lines.extend_from_slice(ransac_lines);

    // Step 2: Split-merge on remaining points
    // Note: We still need to collect remaining points for split-merge,
    // but at least the expensive RANSAC iterations are allocation-free
    if remaining_indices.len() >= 3 {
        let remaining_points: Vec<Point2D> = remaining_indices.iter().map(|&i| points[i]).collect();

        let additional_lines = extract_lines(&remaining_points, split_merge_config);
        output_lines.extend(additional_lines);
    }
}
