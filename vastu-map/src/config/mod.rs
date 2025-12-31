//! Unified configuration loading for VastuMap.
//!
//! Loads all configuration from a single YAML file.

mod defaults;
mod error;
mod exploration;
mod grid;
mod motion_filter;
mod pathfinding;
mod persistence;
mod query;
mod sensor;
mod slam;
mod vastu;

// Re-export main types
pub use error::ConfigLoadError;
pub use vastu::VastuConfig;

// Re-export section types
pub use exploration::{ExplorationSection, RecoverySettings};
pub use grid::GridSection;
pub use motion_filter::MotionFilterSection;
pub use pathfinding::{AStarSettings, PathfindingSection, SmoothingSettings};
pub use persistence::PersistenceSection;
pub use query::{FrontierSettings, QuerySection, TraversabilitySettings};
pub use sensor::{
    BumperSettings, CliffSensorSettings, LidarSettings, RobotSettings, SensorSection,
};
pub use slam::SlamSection;
