//! Configuration loading from TOML
//!
//! # Configuration File Format
//!
//! The configuration file is TOML-formatted with the following structure:
//!
//! ```toml
//! [device]
//! type = "crl200s"
//! name = "CRL-200S Vacuum Robot"
//!
//! [device.hardware]
//! gd32_port = "/dev/ttyS3"
//! lidar_port = "/dev/ttyS1"
//! heartbeat_interval_ms = 20
//!
//! [network]
//! bind_address = "0.0.0.0:5555"
//! ```
//!
//! See `sangamio.toml` for complete example.

use crate::error::{Error, Result};
use serde::Deserialize;
use std::fs;
use std::path::Path;

/// Hardware configuration for device drivers
///
/// Specifies serial port paths and timing parameters for hardware communication.
#[derive(Debug, Clone, Deserialize)]
pub struct HardwareConfig {
    /// Serial port for GD32 motor controller
    ///
    /// **Format**: Device path (e.g., "/dev/ttyS3", "COM3")
    /// **Baud rate**: 115200 (hardcoded in driver)
    /// **Required**: Yes
    pub gd32_port: String,

    /// Serial port for Delta-2D lidar
    ///
    /// **Format**: Device path (e.g., "/dev/ttyS1", "COM4")
    /// **Baud rate**: 115200 (hardcoded in driver)
    /// **Required**: Yes
    pub lidar_port: String,

    /// Heartbeat interval for GD32 watchdog timer
    ///
    /// **Units**: Milliseconds
    /// **Valid range**: 20-50ms (hardware requirement)
    /// **Recommended**: 20ms for safety margin
    /// **Default**: None (must be specified)
    ///
    /// **CRITICAL**: Values outside 20-50ms will cause motors to stop!
    pub heartbeat_interval_ms: u64,
}

/// Device configuration
///
/// Describes the robot hardware and sensor specifications.
#[derive(Debug, Clone, Deserialize)]
pub struct DeviceConfig {
    /// Device type identifier
    ///
    /// **Valid values**: "crl-200s"
    /// **Required**: Yes
    #[serde(rename = "type")]
    pub device_type: String,

    /// Human-readable device name
    ///
    /// **Format**: Any string (used for logging only)
    /// **Required**: Yes
    pub name: String,

    /// Hardware-specific configuration
    ///
    /// **Required**: Yes
    pub hardware: HardwareConfig,
}

/// Network configuration for TCP server
///
/// Controls how the daemon listens for client connections.
#[derive(Debug, Clone, Deserialize)]
pub struct NetworkConfig {
    /// TCP bind address and port
    ///
    /// **Format**: "host:port"
    /// **Examples**:
    /// - "0.0.0.0:5555" (listen on all interfaces)
    /// - "127.0.0.1:5555" (localhost only)
    /// - "192.168.1.100:5555" (specific interface)
    ///
    /// **Required**: Yes
    pub bind_address: String,
}

/// Root configuration
#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub device: DeviceConfig,
    pub network: NetworkConfig,
}

impl Config {
    /// Load configuration from TOML file
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = fs::read_to_string(&path)
            .map_err(|e| Error::Config(format!("Failed to read config: {}", e)))?;

        let config: Config = basic_toml::from_str(&content)
            .map_err(|e| Error::Config(format!("Failed to parse config: {}", e)))?;

        Ok(config)
    }
}
