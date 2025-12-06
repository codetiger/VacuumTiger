//! Metrics module for evaluating SLAM algorithm performance.
//!
//! Provides tools for measuring:
//! - Map quality (noise, coherence, isolated cells)
//! - Scan matching accuracy (transform error vs ground truth)
//! - Trajectory accuracy (for future ground truth comparison)

pub mod map_noise;
pub mod scan_match_error;

pub use map_noise::{MapNoiseMetrics, analyze_map_noise};
pub use scan_match_error::{TransformError, TransformErrorStats, compute_transform_error};
