//! Unified configuration loading for VastuSLAM.
//!
//! Loads all configuration from a single TOML file with sensible defaults.
//!
//! ## Quick Start
//!
//! ```rust,ignore
//! use vastu_slam::config::VastuConfig;
//!
//! // Load from default path (configs/config.toml)
//! let config = VastuConfig::load_default()?;
//!
//! // Or use built-in defaults (no file needed)
//! let config = VastuConfig::default();
//!
//! // Convert to runtime configs
//! let map_config = config.to_map_config();
//! let matcher_config = config.correlative_matcher_config();
//!
//! // Create localizer config with motion filtering (if configured)
//! let localizer_config = config.to_localizer_config();
//! ```
//!
//! ## Configuration Sections
//!
//! | Section | Description |
//! |---------|-------------|
//! | [`GridSection`] | Grid dimensions, resolution, origin |
//! | [`SensorSection`] | Robot geometry, LiDAR, cliff, bumper |
//! | [`SlamSection`] | Matcher, loop closure, pose graph, motion filtering |
//! | [`PersistenceSection`] | Output format and directory |
//!
//! ## Example TOML
//!
//! ```toml
//! [grid]
//! resolution = 0.025      # 2.5cm cells
//! initial_width = 800     # 20m
//!
//! [sensor.robot]
//! radius = 0.17
//!
//! [sensor.lidar]
//! offset_x = -0.110
//! max_range = 8.0
//!
//! [slam.correlative]
//! enabled = true
//! multi_resolution = true
//!
//! # Optional: IMU fusion for improved pose estimation
//! [slam.pose_extrapolator]
//! pose_queue_duration = 0.5        # seconds
//! odom_queue_duration = 0.5        # seconds
//! imu_gravity_time_constant = 10.0 # seconds
//! imu_rotation_weight = 0.3        # 0.0=odom only, 1.0=IMU only
//!
//! # Optional: Scan insertion throttling
//! [slam.motion_filter]
//! max_time_seconds = 5.0           # insert at least every 5s
//! max_distance_meters = 0.2        # insert if moved 20cm
//! max_angle_radians = 0.035        # insert if rotated ~2deg
//! ```
//!
//! ## Motion Filtering
//!
//! Enable Cartographer-style motion filtering for improved accuracy:
//!
//! ```rust,ignore
//! use vastu_slam::config::VastuConfig;
//!
//! let config = VastuConfig::load_default()?;
//!
//! // Check if motion filtering is enabled
//! if config.has_motion_filtering() {
//!     let extrapolator = config.pose_extrapolator_config();
//!     let filter = config.motion_filter_config();
//! }
//!
//! // Create complete localizer config
//! let localizer_config = config.to_localizer_config();
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
