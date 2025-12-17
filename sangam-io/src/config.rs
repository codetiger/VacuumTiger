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
//! # Coordinate frame transforms (optional, defaults to identity)
//! [device.hardware.frame_transforms.lidar]
//! scale = -1.0      # -1 converts CW to CCW
//! offset = 3.14159  # π radians = 180° rotation
//!
//! [device.hardware.frame_transforms.imu_gyro]
//! x = [2, 1]   # output_x = input_z * 1
//! y = [1, 1]   # output_y = input_y * 1
//! z = [0, -1]  # output_z = input_x * -1
//!
//! [network]
//! bind_address = "0.0.0.0:5555"
//! ```
//!
//! See `sangamio.toml` for complete example.
//!
//! # Coordinate Frame Transforms
//!
//! SangamIO transforms raw sensor data to **ROS REP-103** convention:
//! - **X = forward** (direction robot drives)
//! - **Y = left** (port side)
//! - **Z = up**
//! - **Angles = counter-clockwise (CCW) positive**
//!
//! Transform types:
//! - [`AffineTransform1D`]: For lidar angles (`output = scale * input + offset`)
//! - [`AxisTransform3D`]: For IMU axis remapping (`[source_index, sign]` per axis)
//! - [`FrameTransforms`]: Container for all sensor transforms
//!
//! All transforms default to identity (no change) if not specified in config.

use crate::error::{Error, Result};
use serde::Deserialize;
use std::fs;
use std::path::Path;

#[cfg(feature = "mock")]
use crate::devices::mock::config::SimulationConfig;

// ============================================================================
// Coordinate Frame Transforms (ROS REP-103)
// ============================================================================

/// 1D affine transform for scalar values (e.g., angles).
///
/// Applies: `output = scale * input + offset`
///
/// # Examples
///
/// - Identity (no change): `scale = 1.0, offset = 0.0`
/// - Flip direction: `scale = -1.0, offset = 0.0`
/// - Rotate 180°: `scale = 1.0, offset = π`
/// - CRL-200S lidar (CW→CCW + 180°): `scale = -1.0, offset = π`
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct AffineTransform1D {
    /// Scale factor (use -1.0 to flip direction)
    #[serde(default = "default_scale")]
    pub scale: f32,

    /// Offset to add after scaling (radians for angles)
    #[serde(default)]
    pub offset: f32,
}

fn default_scale() -> f32 {
    1.0
}

impl Default for AffineTransform1D {
    fn default() -> Self {
        Self::identity()
    }
}

impl AffineTransform1D {
    /// Identity transform (no change)
    pub fn identity() -> Self {
        Self {
            scale: 1.0,
            offset: 0.0,
        }
    }

    /// Apply the transform to an input value
    #[inline]
    pub fn apply(&self, input: f32) -> f32 {
        self.scale * input + self.offset
    }
}

/// 3D axis remapping transform for IMU data.
///
/// Each axis specifies: `[source_index, sign]`
/// - `source_index`: 0=x, 1=y, 2=z (which input axis to read)
/// - `sign`: 1 or -1 (multiply by this value)
///
/// # Examples
///
/// - Identity: `x=[0,1], y=[1,1], z=[2,1]` (no remapping)
/// - Swap X and Z: `x=[2,1], y=[1,1], z=[0,1]`
/// - Flip Z sign: `x=[0,1], y=[1,1], z=[2,-1]`
/// - CRL-200S IMU: `x=[2,1], y=[1,1], z=[0,-1]` (remap + flip yaw)
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct AxisTransform3D {
    /// Transform for X output: [source_index (0=x,1=y,2=z), sign]
    #[serde(default = "default_x_axis")]
    pub x: [i8; 2],

    /// Transform for Y output: [source_index, sign]
    #[serde(default = "default_y_axis")]
    pub y: [i8; 2],

    /// Transform for Z output: [source_index, sign]
    #[serde(default = "default_z_axis")]
    pub z: [i8; 2],
}

fn default_x_axis() -> [i8; 2] {
    [0, 1]
}
fn default_y_axis() -> [i8; 2] {
    [1, 1]
}
fn default_z_axis() -> [i8; 2] {
    [2, 1]
}

impl Default for AxisTransform3D {
    fn default() -> Self {
        Self::identity()
    }
}

impl AxisTransform3D {
    /// Identity transform (no remapping)
    pub fn identity() -> Self {
        Self {
            x: [0, 1],
            y: [1, 1],
            z: [2, 1],
        }
    }

    /// Apply transform to 3 i16 values
    #[inline]
    pub fn apply(&self, input: [i16; 3]) -> [i16; 3] {
        [
            input[self.x[0] as usize] * self.x[1] as i16,
            input[self.y[0] as usize] * self.y[1] as i16,
            input[self.z[0] as usize] * self.z[1] as i16,
        ]
    }
}

/// Coordinate frame transforms for all sensors.
///
/// Transforms raw sensor data to ROS REP-103 robot frame:
/// - X = forward (direction robot drives)
/// - Y = left (port side)
/// - Z = up
/// - Angles = counter-clockwise (CCW) positive
///
/// All transforms default to identity (no change) if not specified.
#[derive(Debug, Clone, Deserialize, Default)]
pub struct FrameTransforms {
    /// Lidar angle transform (identity = no change)
    #[serde(default)]
    pub lidar: AffineTransform1D,

    /// IMU gyroscope axis transform (identity = no remapping)
    #[serde(default)]
    pub imu_gyro: AxisTransform3D,

    /// IMU accelerometer axis transform (identity = no remapping)
    #[serde(default)]
    pub imu_accel: AxisTransform3D,
}

// ============================================================================
// Hardware Configuration
// ============================================================================

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

    /// Coordinate frame transforms for sensor data
    ///
    /// Transforms raw sensor data to ROS REP-103 robot frame.
    /// Defaults to identity (no transformation) if not specified.
    ///
    /// **Optional**: Yes (defaults to identity transforms)
    #[serde(default)]
    pub frame_transforms: FrameTransforms,

    /// Lidar motor PWM duty cycle
    ///
    /// **Units**: Percentage (0-100)
    /// **Default**: 60 (provides ~330 RPM / 5.5 Hz scan rate)
    /// **Recommended**: 60 for stable scanning
    ///
    /// Higher values = faster motor = higher scan rate
    /// Lower values = slower motor = lower scan rate
    #[serde(default = "default_lidar_pwm")]
    pub lidar_pwm: u8,
}

fn default_lidar_pwm() -> u8 {
    60
}

/// Device configuration
///
/// Describes the robot hardware and sensor specifications.
#[derive(Debug, Clone, Deserialize)]
pub struct DeviceConfig {
    /// Device type identifier
    ///
    /// **Valid values**: "crl200s", "mock" (mock requires --features mock)
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
    /// **Required**: For "crl200s" device type
    /// **Optional**: For "mock" device type
    #[serde(default)]
    pub hardware: Option<HardwareConfig>,

    /// Simulation configuration
    ///
    /// **Required**: For "mock" device type
    /// **Optional**: For "crl200s" device type (ignored)
    #[cfg(feature = "mock")]
    #[serde(default)]
    pub simulation: Option<SimulationConfig>,
}

/// Network configuration for TCP/UDP server
///
/// Controls how the daemon listens for client connections and streams sensor data.
/// Both TCP (commands) and UDP (sensor streaming) use the same port from `bind_address`.
#[derive(Debug, Clone, Deserialize)]
pub struct NetworkConfig {
    /// TCP/UDP bind address and port
    ///
    /// **Format**: "host:port"
    /// **Examples**:
    /// - "0.0.0.0:5555" (listen on all interfaces)
    /// - "127.0.0.1:5555" (localhost only)
    /// - "192.168.1.100:5555" (specific interface)
    ///
    /// TCP is used for commands, UDP for sensor streaming.
    /// Both use the same port for simpler client configuration.
    ///
    /// **Required**: Yes
    pub bind_address: String,

    /// UDP client port override (optional).
    ///
    /// If specified, UDP streaming will use this port instead of the TCP port.
    /// Useful for localhost testing where TCP and UDP need different ports.
    ///
    /// **Default**: Same as TCP port from `bind_address`
    #[serde(default)]
    pub udp_client_port: Option<u16>,
}

impl NetworkConfig {
    /// Extract the port number from bind_address.
    /// Used for TCP listening.
    pub fn port(&self) -> u16 {
        self.bind_address
            .rsplit(':')
            .next()
            .and_then(|p| p.parse().ok())
            .unwrap_or(5555)
    }

    /// Get the UDP streaming port.
    /// Uses `udp_client_port` if set, otherwise same as TCP port.
    pub fn udp_port(&self) -> u16 {
        self.udp_client_port.unwrap_or_else(|| self.port())
    }
}

/// Root configuration
#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub device: DeviceConfig,
    pub network: NetworkConfig,
}

/// Minimum heartbeat interval (hardware requirement)
const MIN_HEARTBEAT_INTERVAL_MS: u64 = 20;
/// Maximum heartbeat interval (hardware requirement)
const MAX_HEARTBEAT_INTERVAL_MS: u64 = 50;

impl Config {
    /// Load configuration from TOML file
    ///
    /// # Validation
    ///
    /// For "crl200s" device:
    /// - `hardware` section is required
    /// - `heartbeat_interval_ms` must be between 20-50ms (hardware requirement)
    ///
    /// For "mock" device:
    /// - `simulation` section is required
    /// - `map_file` must be specified
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = fs::read_to_string(&path)
            .map_err(|e| Error::Config(format!("Failed to read config: {}", e)))?;

        let config: Config = basic_toml::from_str(&content)
            .map_err(|e| Error::Config(format!("Failed to parse config: {}", e)))?;

        // Device-specific validation
        match config.device.device_type.as_str() {
            "crl200s" => {
                // Hardware config is required for CRL-200S
                let hardware = config.device.hardware.as_ref().ok_or_else(|| {
                    Error::Config("crl200s device requires [device.hardware] section".to_string())
                })?;

                // Validate heartbeat interval is within hardware-required range
                let interval = hardware.heartbeat_interval_ms;
                if !(MIN_HEARTBEAT_INTERVAL_MS..=MAX_HEARTBEAT_INTERVAL_MS).contains(&interval) {
                    return Err(Error::Config(format!(
                        "heartbeat_interval_ms must be between {}ms and {}ms (got {}ms). \
                        Values outside this range will cause motor watchdog timeout.",
                        MIN_HEARTBEAT_INTERVAL_MS, MAX_HEARTBEAT_INTERVAL_MS, interval
                    )));
                }
            }
            #[cfg(feature = "mock")]
            "mock" => {
                // Simulation config is required for mock device
                let simulation = config.device.simulation.as_ref().ok_or_else(|| {
                    Error::Config("mock device requires [device.simulation] section".to_string())
                })?;

                // Map file is required
                if simulation.map_file.is_empty() {
                    return Err(Error::Config(
                        "mock device requires map_file in [device.simulation]".to_string(),
                    ));
                }

                // Validate speed factor is positive
                if simulation.speed_factor <= 0.0 {
                    return Err(Error::Config("speed_factor must be positive".to_string()));
                }
            }
            #[cfg(not(feature = "mock"))]
            "mock" => {
                return Err(Error::Config(
                    "Mock device not available: rebuild with --features mock".to_string(),
                ));
            }
            other => {
                return Err(Error::UnknownDevice(other.to_string()));
            }
        }

        Ok(config)
    }
}
