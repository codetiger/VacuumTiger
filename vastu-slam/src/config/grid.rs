//! Grid configuration section.

use serde::{Deserialize, Serialize};

use crate::core::WorldPoint;
use crate::grid::GridConfig;

use super::defaults;

/// Grid configuration section
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GridSection {
    /// Cell resolution (meters)
    #[serde(default = "defaults::resolution")]
    pub resolution: f32,

    /// Initial grid width (cells)
    #[serde(default = "defaults::grid_size")]
    pub initial_width: usize,

    /// Initial grid height (cells)
    #[serde(default = "defaults::grid_size")]
    pub initial_height: usize,

    /// Origin mode: "center" or "corner"
    #[serde(default = "defaults::origin_mode")]
    pub origin_mode: String,

    /// Origin X (for custom mode)
    #[serde(default)]
    pub origin_x: f32,

    /// Origin Y (for custom mode)
    #[serde(default)]
    pub origin_y: f32,

    /// Auto expand grid
    #[serde(default = "defaults::enabled")]
    pub auto_expand: bool,
}

impl Default for GridSection {
    fn default() -> Self {
        Self {
            resolution: 0.025,
            initial_width: 800,
            initial_height: 800,
            origin_mode: "center".to_string(),
            origin_x: 0.0,
            origin_y: 0.0,
            auto_expand: true,
        }
    }
}

impl GridSection {
    /// Convert to GridConfig
    pub fn to_grid_config(&self) -> GridConfig {
        let origin = if self.origin_mode == "corner" {
            Some(WorldPoint::new(self.origin_x, self.origin_y))
        } else {
            None // Centered
        };

        GridConfig {
            resolution: self.resolution,
            initial_width: self.initial_width,
            initial_height: self.initial_height,
            origin,
            auto_expand: self.auto_expand,
            max_width: self.initial_width * 2,
            max_height: self.initial_height * 2,
        }
    }
}
