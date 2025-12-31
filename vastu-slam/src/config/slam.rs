//! SLAM configuration section.

use serde::{Deserialize, Serialize};

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
}
