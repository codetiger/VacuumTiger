//! Unified configuration loading for VastuSLAM.
//!
//! Loads all configuration from a single YAML file with sensible defaults.
//!
//! ## Quick Start
//!
//! ```rust,ignore
//! use vastu_slam::config::VastuConfig;
//!
//! // Load from default path (configs/config.yaml)
//! let config = VastuConfig::load_default()?;
//!
//! // Or use built-in defaults (no file needed)
//! let config = VastuConfig::default();
//!
//! // Convert to runtime configs
//! let map_config = config.to_map_config();
//! let matcher_config = config.correlative_matcher_config();
//! ```
//!
//! ## Configuration Sections
//!
//! | Section | Description |
//! |---------|-------------|
//! | [`GridSection`] | Grid dimensions, resolution, origin |
//! | [`SensorSection`] | Robot geometry, LiDAR, cliff, bumper |
//! | [`SlamSection`] | Matcher, loop closure, pose graph settings |
//! | [`PersistenceSection`] | Output format and directory |
//!
//! ## Example YAML
//!
//! ```yaml
//! grid:
//!   resolution: 0.025      # 2.5cm cells
//!   initial_width: 800     # 20m
//!
//! sensor:
//!   robot:
//!     radius: 0.17
//!   lidar:
//!     offset_x: -0.110
//!     max_range: 8.0
//!
//! slam:
//!   correlative:
//!     enabled: true
//!     multi_resolution: true
//! ```

mod defaults;
mod error;
mod grid;
mod persistence;
mod sensor;
mod slam;
mod vastu;

// Re-export main types
pub use error::ConfigLoadError;
pub use vastu::VastuConfig;

// Re-export section types
pub use grid::GridSection;
pub use persistence::PersistenceSection;
pub use sensor::{
    BumperSettings, CliffSensorSettings, LidarSettings, RobotSettings, SensorSection,
};
pub use slam::SlamSection;
