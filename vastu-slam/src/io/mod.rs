//! Map persistence and export.
//!
//! This module provides save/load functionality:
//!
//! - **Native .vastu format**: Binary format preserving all cell data
//! - **PGM export**: ROS-compatible occupancy grid images
//! - **SVG export**: Visualization with trajectories for auditing
//! - **Scenario parsing**: YAML-based test scenario definitions (requires `scenario` feature)
//!
//! ## Saving and Loading Maps
//!
//! ```rust,ignore
//! use vastu_slam::io::{save_vastu, load_vastu};
//! use std::path::Path;
//!
//! // Save map
//! save_vastu(map.storage(), Path::new("map.vastu"))?;
//!
//! // Load map
//! let storage = load_vastu(Path::new("map.vastu"))?;
//! ```
//!
//! ## ROS Export
//!
//! Export maps in ROS map_server format:
//!
//! ```rust,ignore
//! use vastu_slam::io::export_ros_map;
//! use std::path::Path;
//!
//! // Creates map.pgm and map.yaml
//! export_ros_map(map.storage(), Path::new("map"))?;
//! ```
//!
//! ## SVG Visualization
//!
//! Generate SVG audit files with map and trajectory visualization:
//!
//! ```rust,ignore
//! use vastu_slam::io::{SvgVisualizer, SvgConfig, markers_by_distance};
//!
//! let visualizer = SvgVisualizer::new(storage, SvgConfig::default())
//!     .with_title("Test Run")
//!     .with_ground_truth(gt_poses, markers_by_distance(&gt_poses, 0.5))
//!     .with_estimated(est_poses, markers);
//! visualizer.save(Path::new("output.svg"))?;
//! ```

pub mod pgm;
#[cfg(any(test, feature = "scenario"))]
pub mod scenario;
pub mod svg;
pub mod vastu_format;

pub use vastu_format::{IoError, load_vastu, read_vastu, save_vastu, write_vastu};

pub use pgm::{export_pgm, export_ros_map, export_yaml, write_pgm};

#[cfg(any(test, feature = "scenario"))]
pub use scenario::{Command, Scenario, ScenarioError, SeedConfig, StartPose, SvgOutputConfig};

pub use svg::{SvgConfig, SvgVisualizer, markers_by_distance, markers_by_time};
