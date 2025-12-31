//! Persistence configuration section.

use serde::{Deserialize, Serialize};

use super::defaults;

/// Persistence settings section
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PersistenceSection {
    /// Output format: "vastu", "pgm", or "both"
    #[serde(default = "defaults::output_format")]
    pub output_format: String,

    /// Output directory path
    #[serde(default = "defaults::output_dir")]
    pub output_dir: String,

    /// Auto-save interval (seconds, 0=disabled)
    #[serde(default)]
    pub auto_save_interval: u32,
}

impl Default for PersistenceSection {
    fn default() -> Self {
        Self {
            output_format: "both".to_string(),
            output_dir: "./output".to_string(),
            auto_save_interval: 30,
        }
    }
}
