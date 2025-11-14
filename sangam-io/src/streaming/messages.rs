//! Message types for ZeroMQ streaming.
//!
//! This module defines the data structures used for bidirectional communication:
//! - Telemetry messages (outbound): Raw sensor data
//! - Lidar messages (outbound): Point cloud scans
//! - Robot commands (inbound): Motion and actuator control

use serde::{Deserialize, Serialize};

/// Top-level telemetry message enum published on "telemetry" topic
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum TelemetryMessage {
    /// Raw sensor data from GD32 STATUS_DATA packet
    SensorUpdate(SensorUpdate),
    /// Connection quality statistics
    ConnectionQuality(ConnectionQuality),
}

/// Raw sensor data from GD32 motor controller
///
/// Published at ~500 Hz (every STATUS_DATA packet from 2ms READ loop)
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SensorUpdate {
    /// Timestamp in microseconds since epoch
    pub timestamp: u64,

    // Battery (raw values from GD32)
    pub battery_level: Option<u8>, // 0-100%
    pub is_charging: Option<bool>, // Charging flag

    // Proximity sensors (raw ADC values)
    pub ir_sensor_1: Option<u16>,     // Front IR sensor
    pub start_button_ir: Option<u16>, // Start button IR
    pub dock_button_ir: Option<u16>,  // Dock button IR

    // Bumper (raw boolean)
    pub bumper_pressed: Option<bool>,

    // Cliff sensors (raw ADC values ×4)
    pub cliff_sensors: Option<[u16; 4]>,

    // Error code from GD32
    pub error_code: Option<u8>,

    // Encoders (RAW TICK COUNTS - key data for odometry)
    pub encoder_left: Option<i32>, // Cumulative ticks (with wraparound)
    pub encoder_right: Option<i32>, // Cumulative ticks (with wraparound)
}

/// Communication statistics for monitoring GD32 connection health
///
/// Published at ~1 Hz
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ConnectionQuality {
    /// Timestamp in microseconds since epoch
    pub timestamp: u64,
    /// Total packets received from GD32
    pub rx_packets: u64,
    /// Total packets sent to GD32
    pub tx_packets: u64,
    /// Success rate (0.0 - 1.0)
    pub success_rate: f32,
    /// Data received in last 100ms
    pub telemetry_fresh: bool,
}

/// Lidar scan data published on "lidar" topic
///
/// Published at ~5 Hz (Delta-2D native scan rate)
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LidarScan {
    /// Timestamp in microseconds since epoch
    pub timestamp: u64,
    /// Monotonically increasing scan number
    pub scan_number: u64,
    /// Point cloud data
    pub points: Vec<LidarPoint>,
}

/// Single lidar measurement point
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LidarPoint {
    /// Angle in radians (0 to 2π)
    pub angle: f64,
    /// Distance in meters
    pub distance: f64,
    /// Signal strength (0-255)
    pub quality: u8,
}

/// Robot commands received on "command" topic
///
/// Commands sent from external processes (SLAM, Navigation, or manual control)
/// to SangamIO for execution on hardware.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum RobotCommand {
    /// Set wheel velocities directly (ticks per second)
    ///
    /// - `left`: Left wheel velocity (ticks/sec, positive = forward)
    /// - `right`: Right wheel velocity (ticks/sec, positive = forward)
    ///
    /// Consumer is responsible for:
    /// - Differential drive kinematics (linear/angular → left/right)
    /// - Converting m/s to ticks/sec
    /// - Acceleration ramping
    ///
    /// To stop, send `SetWheelVelocity { left: 0.0, right: 0.0 }`
    SetWheelVelocity { left: f64, right: f64 },

    /// Set air pump speed (percentage-based)
    ///
    /// - `speed`: 0-100% (0 = stop, 100 = full speed)
    SetAirPumpSpeed { speed: u8 },

    /// Set main roller brush speed (percentage-based)
    ///
    /// - `speed`: 0-100% (0 = stop, 100 = full speed)
    SetMainBrushSpeed { speed: u8 },

    /// Set side brush speed (percentage-based)
    ///
    /// - `speed`: 0-100% (0 = stop, 100 = full speed)
    SetSideBrushSpeed { speed: u8 },

    /// Enable lidar with specified PWM speed
    ///
    /// - `pwm`: PWM duty cycle 0-100% (lidar motor speed)
    ///
    /// This command performs full initialization sequence:
    /// 1. Send lidar preparation command (CMD=0xA2)
    /// 2. Enable lidar power (CMD=0x97)
    /// 3. Set lidar PWM speed (CMD=0x71)
    ///
    /// Managed by GD32 motor controller.
    EnableLidar { pwm: u8 },

    /// Disable lidar
    ///
    /// Powers off the lidar motor (CMD=0x97).
    /// GD32 motor controller manages the shutdown.
    DisableLidar,

    /// Set lidar PWM speed
    ///
    /// - `pwm`: PWM duty cycle 0-100%
    ///
    /// Adjusts lidar motor speed without power cycling.
    /// Lidar must already be enabled.
    SetLidarPWM { pwm: u8 },

    /// Emergency stop all motors and actuators
    ///
    /// Immediately stops:
    /// - Both wheel motors
    /// - Air pump
    /// - Main roller brush
    /// - Side brush
    /// - Lidar motor
    EmergencyStopAll,
}
