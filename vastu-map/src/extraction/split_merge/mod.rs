//! Split-and-Merge line extraction algorithm.
//!
//! This algorithm extracts line segments from an ordered sequence of points
//! (e.g., from a lidar scan). It works by:
//!
//! 1. **Split Phase**: Recursively split point sequences where the maximum
//!    deviation from the fitted line exceeds a threshold.
//!
//! 2. **Merge Phase**: Adjacent segments that can be represented by a single
//!    line within tolerance are merged.
//!
//! Requirements:
//! - Points must be ordered (sequential around scan)
//! - Works best with relatively uniform point density
//!
//! # Adaptive Thresholds
//!
//! When sensor position is known, the split threshold can adapt to range:
//! farther points have higher measurement uncertainty, so a larger threshold
//! is used. This prevents over-splitting distant segments while maintaining
//! accuracy for close-range features.
//!
//! ```rust,ignore
//! use vastu_map::extraction::{SplitMergeConfig, extract_lines_from_sensor};
//! use vastu_map::core::Point2D;
//!
//! let sensor_pos = Point2D::new(0.0, 0.0);
//! let lines = extract_lines_from_sensor(&points, &config, sensor_pos);
//! ```

mod algorithm;
mod config;
mod helpers;

// Re-export public API
pub use algorithm::{extract_lines, extract_lines_from_sensor};
pub use config::SplitMergeConfig;
pub use helpers::adaptive_split_threshold;
