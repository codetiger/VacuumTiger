//! Configuration loading from JSON

use crate::core::types::{ActuatorSpec, SensorGroupSpec};
use crate::error::{Error, Result};
use crate::streaming::WireFormat;
use serde::Deserialize;
use std::fs;
use std::path::Path;

/// Hardware configuration
#[derive(Debug, Clone, Deserialize)]
pub struct HardwareConfig {
    pub gd32_port: String,
    pub lidar_port: String,
    pub heartbeat_interval_ms: u64,
}

/// Device configuration
#[derive(Debug, Clone, Deserialize)]
pub struct DeviceConfig {
    #[serde(rename = "type")]
    pub device_type: String,
    pub name: String,
    pub hardware: HardwareConfig,
    pub sensor_groups: Vec<SensorGroupSpec>,
    pub actuators: Vec<ActuatorSpec>,
}

/// Network configuration
#[derive(Debug, Clone, Deserialize)]
pub struct NetworkConfig {
    pub bind_address: String,
    #[serde(default)]
    pub wire_format: WireFormatConfig,
}

/// Wire format configuration (deserializable from string)
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum WireFormatConfig {
    Postcard,
    #[default]
    Json,
}


impl From<WireFormatConfig> for WireFormat {
    fn from(config: WireFormatConfig) -> Self {
        match config {
            WireFormatConfig::Postcard => WireFormat::Postcard,
            WireFormatConfig::Json => WireFormat::Json,
        }
    }
}

/// Root configuration
#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub device: DeviceConfig,
    pub network: NetworkConfig,
}

impl Config {
    /// Load configuration from JSON file
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = fs::read_to_string(path)
            .map_err(|e| Error::Config(format!("Failed to read config: {}", e)))?;

        let config: Config = serde_json::from_str(&content)
            .map_err(|e| Error::Config(format!("Failed to parse config: {}", e)))?;

        Ok(config)
    }
}
