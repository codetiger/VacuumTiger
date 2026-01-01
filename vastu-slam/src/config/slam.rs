//! SLAM configuration section.

use serde::{Deserialize, Serialize};

use crate::core::{MotionFilterConfig, PoseExtrapolatorConfig};
use crate::grid::LogOddsConfig;
use crate::matching::{
    BackgroundOptimizerConfig, CorrelativeMatcherConfig, LoopClosureConfig, PoseGraphConfig,
};

/// SLAM configuration section (Cartographer-style)
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct SlamSection {
    /// Log-odds occupancy model configuration
    #[serde(default)]
    pub log_odds: LogOddsConfig,

    /// Correlative scan matcher configuration
    #[serde(default)]
    pub correlative: CorrelativeMatcherConfig,

    /// Loop closure detection configuration
    #[serde(default)]
    pub loop_closure: LoopClosureConfig,

    /// Pose graph optimization configuration
    #[serde(default)]
    pub pose_graph: PoseGraphConfig,

    /// Background optimizer configuration
    #[serde(default)]
    pub background_optimizer: BackgroundOptimizerConfig,

    /// Pose extrapolator configuration for IMU fusion.
    ///
    /// When set, enables Cartographer-style pose extrapolation with
    /// odometry (primary) and IMU (secondary) fusion.
    #[serde(default)]
    pub pose_extrapolator: Option<PoseExtrapolatorConfig>,

    /// Motion filter configuration for scan insertion throttling.
    ///
    /// When set, only processes scans when the robot has moved enough
    /// (distance, rotation, or time since last scan).
    #[serde(default)]
    pub motion_filter: Option<MotionFilterConfig>,
}
