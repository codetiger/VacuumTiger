//! Configuration for SangamIO application
//!
//! Loads configuration from TOML file with minimal parameters needed
//! for pure hardware abstraction.

use crate::error::Result;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

/// Top-level application configuration
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct AppConfig {
    pub hardware: HardwareConfig,
    pub robot: RobotConfig,
    pub streaming: StreamingConfig,
    pub logging: LoggingConfig,
}

/// Hardware configuration (serial ports)
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct HardwareConfig {
    /// GD32F103 motor controller serial port
    pub gd32_port: String,
    /// Delta-2D lidar serial port
    pub lidar_port: String,
}

/// Robot configuration (minimal - only calibration constant)
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RobotConfig {
    /// Calibration constant: encoder ticks per second at full motor speed (2000)
    ///
    /// This value is measured empirically by running motors at full speed
    /// and measuring the encoder tick rate. Used for converting ticks/sec
    /// to GD32 motor units (-2000 to 2000).
    pub max_ticks_per_sec: f64,
}

/// TCP streaming configuration
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct StreamingConfig {
    /// TCP bind address for outbound telemetry and lidar data
    ///
    /// Examples:
    /// - `0.0.0.0:5555` - Bind to all interfaces on port 5555
    /// - `127.0.0.1:5555` - Localhost only
    #[serde(alias = "zmq_pub_endpoint")] // Backward compatibility
    pub tcp_pub_address: String,

    /// TCP bind address for inbound commands
    ///
    /// Examples:
    /// - `0.0.0.0:5556` - Bind to all interfaces on port 5556
    /// - `127.0.0.1:5556` - Localhost only
    #[serde(alias = "zmq_sub_endpoint")] // Backward compatibility
    pub tcp_cmd_address: String,
}

/// Logging configuration
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct LoggingConfig {
    /// Log level (trace, debug, info, warn, error)
    pub level: String,
    /// Log output (stdout, stderr, or file path)
    pub output: String,
}

impl AppConfig {
    /// Load configuration from TOML file
    ///
    /// # Arguments
    /// - `path`: Path to TOML configuration file
    ///
    /// # Returns
    /// Parsed configuration or error
    ///
    /// # Example
    /// ```no_run
    /// use sangam_io::config::AppConfig;
    ///
    /// let config = AppConfig::from_file("sangamio.toml")?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let contents = fs::read_to_string(path)?;
        let config: AppConfig = toml::from_str(&contents)?;
        Ok(config)
    }

    /// Default configuration for CRL-200S robot
    ///
    /// Suitable for testing and development. Production deployments
    /// should use a proper TOML configuration file.
    pub fn crl200s_defaults() -> Self {
        Self {
            hardware: HardwareConfig {
                gd32_port: "/dev/ttyS3".to_string(),
                lidar_port: "/dev/ttyS1".to_string(),
            },
            robot: RobotConfig {
                max_ticks_per_sec: 3000.0, // Calibrated value for CRL-200S
            },
            streaming: StreamingConfig {
                tcp_pub_address: "0.0.0.0:5555".to_string(),
                tcp_cmd_address: "0.0.0.0:5556".to_string(),
            },
            logging: LoggingConfig {
                level: "info".to_string(),
                output: "stdout".to_string(),
            },
        }
    }

    /// Save configuration to TOML file
    ///
    /// # Arguments
    /// - `path`: Path to save TOML configuration file
    ///
    /// # Returns
    /// Success or error
    pub fn to_file<P: AsRef<Path>>(&self, path: P) -> Result<()> {
        let contents = toml::to_string_pretty(self)?;
        fs::write(path, contents)?;
        Ok(())
    }
}

impl Default for AppConfig {
    fn default() -> Self {
        Self::crl200s_defaults()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = AppConfig::crl200s_defaults();
        assert_eq!(config.hardware.gd32_port, "/dev/ttyS3");
        assert_eq!(config.hardware.lidar_port, "/dev/ttyS1");
        assert_eq!(config.robot.max_ticks_per_sec, 3000.0);
        assert_eq!(config.streaming.tcp_pub_address, "0.0.0.0:5555");
        assert_eq!(config.streaming.tcp_cmd_address, "0.0.0.0:5556");
    }

    #[test]
    fn test_toml_serialization() {
        let config = AppConfig::crl200s_defaults();
        let toml_string = toml::to_string_pretty(&config).unwrap();

        // Should contain all sections
        assert!(toml_string.contains("[hardware]"));
        assert!(toml_string.contains("[robot]"));
        assert!(toml_string.contains("[streaming]"));
        assert!(toml_string.contains("[logging]"));

        // Should contain key values
        assert!(toml_string.contains("max_ticks_per_sec = 3000.0"));
        assert!(toml_string.contains("gd32_port = \"/dev/ttyS3\""));
    }

    #[test]
    fn test_toml_deserialization() {
        let toml_content = r#"
[hardware]
gd32_port = "/dev/ttyUSB0"
lidar_port = "/dev/ttyUSB1"

[robot]
max_ticks_per_sec = 2500.0

[streaming]
tcp_pub_address = "127.0.0.1:5555"
tcp_cmd_address = "127.0.0.1:5556"

[logging]
level = "debug"
output = "stdout"
"#;

        let config: AppConfig = toml::from_str(toml_content).unwrap();
        assert_eq!(config.hardware.gd32_port, "/dev/ttyUSB0");
        assert_eq!(config.robot.max_ticks_per_sec, 2500.0);
        assert_eq!(config.logging.level, "debug");
    }
}
