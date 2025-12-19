//! Feature extraction from point clouds.
//!
//! This module provides algorithms for extracting geometric features
//! (lines and corners) from lidar point clouds.
//!
//! # Algorithms
//!
//! - **Line Fitting**: Total Least Squares (orthogonal regression)
//! - **Line Extraction**: Split-and-Merge algorithm for ordered point sequences
//! - **Corner Detection**: Find corners at line intersections

pub mod corner_detection;
pub mod line_fitting;
pub mod split_merge;

pub use corner_detection::{CornerConfig, deduplicate_corners, detect_all_corners, detect_corners};
pub use line_fitting::{fit_line, fit_line_segment, fitting_error, max_distance_point};
pub use split_merge::{SplitMergeConfig, extract_lines};
