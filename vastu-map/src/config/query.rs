//! Query configuration section.

use serde::{Deserialize, Serialize};

use super::defaults;

/// Query settings section
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct QuerySection {
    /// Frontier detection settings
    #[serde(default)]
    pub frontier: FrontierSettings,

    /// Traversability checking settings
    #[serde(default)]
    pub traversability: TraversabilitySettings,
}

/// Frontier detection settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FrontierSettings {
    /// Minimum cells for valid frontier
    #[serde(default = "defaults::min_frontier_size")]
    pub min_frontier_size: usize,

    /// Use 8-connected grid (vs 4-connected)
    #[serde(default = "defaults::enabled")]
    pub use_8_connectivity: bool,
}

impl Default for FrontierSettings {
    fn default() -> Self {
        Self {
            min_frontier_size: 5,
            use_8_connectivity: true,
        }
    }
}

/// Traversability settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TraversabilitySettings {
    /// Extra clearance around robot (meters)
    #[serde(default = "defaults::safety_margin")]
    pub safety_margin: f32,
}

impl Default for TraversabilitySettings {
    fn default() -> Self {
        Self {
            safety_margin: 0.05,
        }
    }
}
