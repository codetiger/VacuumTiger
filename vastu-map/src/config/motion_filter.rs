//! Motion filter configuration section.

use serde::{Deserialize, Serialize};

use crate::explore::MotionFilterConfig;

use super::defaults;

/// Motion filter settings section
///
/// Gates scan processing based on robot movement to reduce redundant observations.
/// A scan is processed if ANY threshold is exceeded.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MotionFilterSection {
    /// Whether motion filtering is enabled
    #[serde(default = "defaults::enabled")]
    pub enabled: bool,

    /// Maximum time between scan insertions (seconds)
    #[serde(default = "defaults::motion_max_time")]
    pub max_time_secs: f32,

    /// Maximum distance before forcing scan insertion (meters)
    #[serde(default = "defaults::motion_max_distance")]
    pub max_distance: f32,

    /// Maximum rotation before forcing scan insertion (radians)
    #[serde(default = "defaults::motion_max_angle")]
    pub max_angle: f32,
}

impl Default for MotionFilterSection {
    fn default() -> Self {
        Self {
            enabled: true,
            max_time_secs: defaults::motion_max_time(),
            max_distance: defaults::motion_max_distance(),
            max_angle: defaults::motion_max_angle(),
        }
    }
}

impl MotionFilterSection {
    /// Convert to MotionFilterConfig
    pub fn to_motion_filter_config(&self) -> MotionFilterConfig {
        MotionFilterConfig {
            enabled: self.enabled,
            max_time_secs: self.max_time_secs,
            max_distance: self.max_distance,
            max_angle: self.max_angle,
        }
    }
}
