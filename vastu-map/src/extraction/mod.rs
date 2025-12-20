//! Feature extraction from point clouds.
//!
//! This module provides algorithms for extracting geometric features
//! (lines and corners) from lidar point clouds.
//!
//! # Algorithms
//!
//! - **Line Fitting**: Total Least Squares (orthogonal regression), with weighted variant
//! - **Line Extraction**: Split-and-Merge algorithm for ordered point sequences
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

pub mod corner_detection;
pub mod line_fitting;
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
// Re-export from split_merge submodule
pub use split_merge::{
    SplitMergeConfig, adaptive_split_threshold, extract_lines, extract_lines_from_sensor,
};
// Re-export traits
pub use traits::{LineExtractor, SplitMergeExtractor};
