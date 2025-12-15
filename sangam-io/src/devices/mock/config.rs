//! Mock device simulation configuration
//!
//! This module defines all configuration structures for the mock device driver.
//! Every parameter has sensible defaults matching CRL-200S hardware specifications,
//! allowing minimal configuration for basic usage while enabling deep customization
//! for advanced testing scenarios.
//!
//! # Configuration Hierarchy
//!
//! ```text
//! SimulationConfig
//! ├── map_file, start_x/y/theta     # Environment setup
//! ├── speed_factor, random_seed      # Simulation control
//! ├── RobotConfig                    # Physical parameters
//! │   ├── wheel_base, ticks_per_meter
//! │   ├── max_linear/angular_speed
//! │   └── collision_mode, robot_radius
//! ├── LidarConfig                    # Lidar simulation
//! │   ├── num_rays, scan_rate_hz
//! │   ├── min/max_range, mounting offsets
//! │   └── LidarNoiseConfig
//! ├── ImuConfig                      # IMU simulation
//! │   ├── GyroNoiseConfig
//! │   ├── Noise3DConfig (accel)
//! │   └── Noise3DConfig (tilt)
//! ├── EncoderConfig                  # Encoder simulation
//! │   └── EncoderNoiseConfig
//! ├── BumpersConfig                  # Bumper zones
//! ├── CliffsConfig                   # Cliff sensor positions
//! ├── SensorsConfig                  # Battery, buttons, etc.
//! └── ActuatorsConfig                # Logging options
//! ```
//!
//! # Default Values
//!
//! All defaults match CRL-200S hardware specifications:
//!
//! | Parameter | Default | Source |
//! |-----------|---------|--------|
//! | wheel_base | 0.233 m | Measured |
//! | ticks_per_meter | 4464 | Calibrated |
//! | max_linear_speed | 0.3 m/s | Hardware limit |
//! | max_angular_speed | 1.0 rad/s | Hardware limit |
//! | robot_radius | 0.17 m | Physical size |
//! | lidar_num_rays | 360 | Delta-2D spec |
//! | lidar_scan_rate | 5 Hz | Delta-2D spec |
//! | lidar_max_range | 8 m | Delta-2D spec |
//!
//! # Noise Configuration
//!
//! Noise parameters are initial estimates and should be calibrated based on
//! SLAM performance with real hardware data. Start with defaults, then adjust
//! if simulated performance differs significantly from real-world results.

use serde::Deserialize;
use std::f32::consts::PI;

// ============================================================================
// Noise Configurations
// ============================================================================

/// Noise configuration for lidar measurements
#[derive(Debug, Clone, Deserialize)]
pub struct LidarNoiseConfig {
    /// Range measurement noise standard deviation (meters)
    #[serde(default = "default_lidar_range_stddev")]
    pub range_stddev: f32,

    /// Systematic range bias (meters)
    #[serde(default)]
    pub range_bias: f32,

    /// Angular noise standard deviation (radians)
    #[serde(default = "default_lidar_angle_stddev")]
    pub angle_stddev: f32,

    /// Base quality value (0-255)
    #[serde(default = "default_quality_base")]
    pub quality_base: u8,

    /// Quality reduction per meter of distance
    #[serde(default = "default_quality_distance_decay")]
    pub quality_distance_decay: u8,

    /// Probability of invalid reading (0.0-1.0)
    #[serde(default = "default_miss_rate")]
    pub miss_rate: f32,
}

fn default_lidar_range_stddev() -> f32 {
    0.005
}
fn default_lidar_angle_stddev() -> f32 {
    0.001
}
fn default_quality_base() -> u8 {
    200
}
fn default_quality_distance_decay() -> u8 {
    10
}
fn default_miss_rate() -> f32 {
    0.01
}

impl Default for LidarNoiseConfig {
    fn default() -> Self {
        Self {
            range_stddev: default_lidar_range_stddev(),
            range_bias: 0.0,
            angle_stddev: default_lidar_angle_stddev(),
            quality_base: default_quality_base(),
            quality_distance_decay: default_quality_distance_decay(),
            miss_rate: default_miss_rate(),
        }
    }
}

/// 3D noise configuration (for IMU axes)
#[derive(Debug, Clone, Deserialize)]
pub struct Noise3DConfig {
    /// Standard deviation per axis [x, y, z]
    #[serde(default = "default_noise_stddev")]
    pub stddev: [f32; 3],

    /// Constant bias per axis [x, y, z]
    #[serde(default)]
    pub bias: [f32; 3],
}

fn default_noise_stddev() -> [f32; 3] {
    [5.0, 5.0, 10.0]
}

impl Default for Noise3DConfig {
    fn default() -> Self {
        Self {
            stddev: default_noise_stddev(),
            bias: [0.0, 0.0, 0.0],
        }
    }
}

/// Gyroscope noise configuration with drift
#[derive(Debug, Clone, Deserialize)]
pub struct GyroNoiseConfig {
    /// Standard deviation per axis [x, y, z] (raw i16 units)
    #[serde(default = "default_gyro_stddev")]
    pub stddev: [f32; 3],

    /// Constant bias per axis [x, y, z]
    #[serde(default)]
    pub bias: [f32; 3],

    /// Bias drift rate (units per second)
    #[serde(default)]
    pub drift_rate: f32,
}

fn default_gyro_stddev() -> [f32; 3] {
    [5.0, 5.0, 10.0]
}

impl Default for GyroNoiseConfig {
    fn default() -> Self {
        Self {
            stddev: default_gyro_stddev(),
            bias: [0.0, 0.0, 0.0],
            drift_rate: 0.0,
        }
    }
}

/// Encoder noise configuration
#[derive(Debug, Clone, Deserialize)]
pub struct EncoderNoiseConfig {
    /// Wheel slip noise standard deviation (multiplicative, 0.0-1.0)
    #[serde(default = "default_slip_stddev")]
    pub slip_stddev: f32,

    /// Systematic slip bias (multiplicative)
    #[serde(default)]
    pub slip_bias: f32,

    /// Enable quantization noise (±0.5 tick jitter)
    #[serde(default = "default_true")]
    pub quantization_noise: bool,
}

fn default_slip_stddev() -> f32 {
    0.002
}
fn default_true() -> bool {
    true
}

impl Default for EncoderNoiseConfig {
    fn default() -> Self {
        Self {
            slip_stddev: default_slip_stddev(),
            slip_bias: 0.0,
            quantization_noise: true,
        }
    }
}

// ============================================================================
// Sensor Configurations
// ============================================================================

/// Robot physical parameters (CRL-200S specifications)
#[derive(Debug, Clone, Deserialize)]
pub struct RobotConfig {
    /// Distance between wheel centers (meters)
    #[serde(default = "default_wheel_base")]
    pub wheel_base: f32,

    /// Encoder ticks per meter of travel
    #[serde(default = "default_ticks_per_meter")]
    pub ticks_per_meter: f32,

    /// Maximum linear velocity (m/s)
    #[serde(default = "default_max_linear_speed")]
    pub max_linear_speed: f32,

    /// Maximum angular velocity (rad/s)
    #[serde(default = "default_max_angular_speed")]
    pub max_angular_speed: f32,

    /// Robot collision radius (meters)
    #[serde(default = "default_robot_radius")]
    pub robot_radius: f32,

    /// Collision behavior: "stop", "slide", "passthrough"
    #[serde(default = "default_collision_mode")]
    pub collision_mode: String,

    /// Slide friction coefficient (0.0-1.0, used when collision_mode="slide")
    #[serde(default = "default_slide_friction")]
    pub slide_friction: f32,
}

fn default_wheel_base() -> f32 {
    0.233
}
fn default_ticks_per_meter() -> f32 {
    4464.0
}
fn default_max_linear_speed() -> f32 {
    0.3
}
fn default_max_angular_speed() -> f32 {
    1.0
}
fn default_robot_radius() -> f32 {
    0.17
}
fn default_collision_mode() -> String {
    "stop".to_string()
}
fn default_slide_friction() -> f32 {
    0.3
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            wheel_base: default_wheel_base(),
            ticks_per_meter: default_ticks_per_meter(),
            max_linear_speed: default_max_linear_speed(),
            max_angular_speed: default_max_angular_speed(),
            robot_radius: default_robot_radius(),
            collision_mode: default_collision_mode(),
            slide_friction: default_slide_friction(),
        }
    }
}

/// Lidar sensor configuration (Delta-2D specifications)
#[derive(Debug, Clone, Deserialize)]
pub struct LidarConfig {
    /// Number of rays per 360° scan
    #[serde(default = "default_num_rays")]
    pub num_rays: usize,

    /// Scan rate in Hz
    #[serde(default = "default_scan_rate_hz")]
    pub scan_rate_hz: f32,

    /// Minimum detection range (meters)
    #[serde(default = "default_min_range")]
    pub min_range: f32,

    /// Maximum detection range (meters)
    #[serde(default = "default_max_range")]
    pub max_range: f32,

    /// Lidar mounting X offset from robot center (meters, negative = behind)
    #[serde(default = "default_mounting_x")]
    pub mounting_x: f32,

    /// Lidar mounting Y offset from robot center (meters)
    #[serde(default)]
    pub mounting_y: f32,

    /// Optical offset from lidar center (meters)
    #[serde(default = "default_optical_offset")]
    pub optical_offset: f32,

    /// Angle offset from robot forward (radians)
    #[serde(default = "default_angle_offset")]
    pub angle_offset: f32,

    /// Noise configuration
    #[serde(default)]
    pub noise: LidarNoiseConfig,
}

fn default_num_rays() -> usize {
    360
}
fn default_scan_rate_hz() -> f32 {
    5.0
}
fn default_min_range() -> f32 {
    0.15
}
fn default_max_range() -> f32 {
    8.0
}
fn default_mounting_x() -> f32 {
    -0.110
}
fn default_optical_offset() -> f32 {
    0.025
}
fn default_angle_offset() -> f32 {
    0.2182
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            num_rays: default_num_rays(),
            scan_rate_hz: default_scan_rate_hz(),
            min_range: default_min_range(),
            max_range: default_max_range(),
            mounting_x: default_mounting_x(),
            mounting_y: 0.0,
            optical_offset: default_optical_offset(),
            angle_offset: default_angle_offset(),
            noise: LidarNoiseConfig::default(),
        }
    }
}

/// IMU sensor configuration
#[derive(Debug, Clone, Deserialize)]
pub struct ImuConfig {
    /// Gyroscope noise configuration
    #[serde(default)]
    pub gyro_noise: GyroNoiseConfig,

    /// Accelerometer noise configuration
    #[serde(default)]
    pub accel_noise: Noise3DConfig,

    /// Tilt sensor noise configuration
    #[serde(default)]
    pub tilt_noise: Noise3DConfig,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            gyro_noise: GyroNoiseConfig::default(),
            accel_noise: Noise3DConfig {
                stddev: [10.0, 10.0, 20.0],
                bias: [0.0, 0.0, 0.0],
            },
            tilt_noise: Noise3DConfig {
                stddev: [5.0, 5.0, 5.0],
                bias: [0.0, 0.0, 0.0],
            },
        }
    }
}

/// Encoder configuration
#[derive(Debug, Clone, Deserialize, Default)]
pub struct EncoderConfig {
    /// Noise configuration
    #[serde(default)]
    pub noise: EncoderNoiseConfig,
}

/// Single bumper zone configuration
#[derive(Debug, Clone, Deserialize)]
pub struct BumperZoneConfig {
    /// Start angle relative to robot forward (radians, CCW positive)
    pub start_angle: f32,

    /// End angle relative to robot forward (radians, CCW positive)
    pub end_angle: f32,
}

/// Bumper sensor configuration
#[derive(Debug, Clone, Deserialize)]
pub struct BumpersConfig {
    /// Bumper trigger distance from robot edge (meters)
    #[serde(default = "default_bumper_trigger_distance")]
    pub trigger_distance: f32,

    /// Left bumper zone
    #[serde(default = "default_left_bumper")]
    pub left: BumperZoneConfig,

    /// Right bumper zone
    #[serde(default = "default_right_bumper")]
    pub right: BumperZoneConfig,
}

fn default_bumper_trigger_distance() -> f32 {
    0.01
}
fn default_left_bumper() -> BumperZoneConfig {
    BumperZoneConfig {
        start_angle: 0.3,
        end_angle: PI / 2.0,
    }
}
fn default_right_bumper() -> BumperZoneConfig {
    BumperZoneConfig {
        start_angle: -PI / 2.0,
        end_angle: -0.3,
    }
}

impl Default for BumpersConfig {
    fn default() -> Self {
        Self {
            trigger_distance: default_bumper_trigger_distance(),
            left: default_left_bumper(),
            right: default_right_bumper(),
        }
    }
}

/// Single cliff sensor position
#[derive(Debug, Clone, Deserialize)]
pub struct CliffSensorConfig {
    /// Sensor name (for output mapping)
    pub name: String,

    /// X position relative to robot center (meters, positive = forward)
    pub x: f32,

    /// Y position relative to robot center (meters, positive = left)
    pub y: f32,
}

/// Cliff sensors configuration
#[derive(Debug, Clone, Deserialize)]
pub struct CliffsConfig {
    /// Enable cliff detection
    #[serde(default = "default_true")]
    pub enabled: bool,

    /// Cliff sensor positions
    #[serde(default = "default_cliff_sensors")]
    pub sensors: Vec<CliffSensorConfig>,
}

fn default_cliff_sensors() -> Vec<CliffSensorConfig> {
    vec![
        CliffSensorConfig {
            name: "left_side".to_string(),
            x: 0.12,
            y: 0.10,
        },
        CliffSensorConfig {
            name: "left_front".to_string(),
            x: 0.15,
            y: 0.05,
        },
        CliffSensorConfig {
            name: "right_front".to_string(),
            x: 0.15,
            y: -0.05,
        },
        CliffSensorConfig {
            name: "right_side".to_string(),
            x: 0.12,
            y: -0.10,
        },
    ]
}

impl Default for CliffsConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            sensors: default_cliff_sensors(),
        }
    }
}

/// Battery simulation configuration
#[derive(Debug, Clone, Deserialize)]
pub struct BatteryConfig {
    /// Battery voltage (volts)
    #[serde(default = "default_battery_voltage")]
    pub voltage: f32,

    /// Battery level (0-100%)
    #[serde(default = "default_battery_level")]
    pub level: u8,

    /// Charging state
    #[serde(default)]
    pub is_charging: bool,
}

fn default_battery_voltage() -> f32 {
    14.8
}
fn default_battery_level() -> u8 {
    85
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            voltage: default_battery_voltage(),
            level: default_battery_level(),
            is_charging: false,
        }
    }
}

/// Binary sensors configuration
#[derive(Debug, Clone, Deserialize)]
pub struct SensorsConfig {
    /// Battery configuration
    #[serde(default)]
    pub battery: BatteryConfig,

    /// Dustbox attached state
    #[serde(default = "default_true")]
    pub dustbox_attached: bool,

    /// Dock connected state
    #[serde(default)]
    pub dock_connected: bool,

    /// Start button state (fixed)
    #[serde(default)]
    pub start_button: u16,

    /// Dock button state (fixed)
    #[serde(default)]
    pub dock_button: u16,
}

impl Default for SensorsConfig {
    fn default() -> Self {
        Self {
            battery: BatteryConfig::default(),
            dustbox_attached: true,
            dock_connected: false,
            start_button: 0,
            dock_button: 0,
        }
    }
}

/// Actuator logging configuration
#[derive(Debug, Clone, Deserialize)]
pub struct ActuatorsConfig {
    /// Log actuator state changes
    #[serde(default = "default_true")]
    pub log_state_changes: bool,
}

impl Default for ActuatorsConfig {
    fn default() -> Self {
        Self {
            log_state_changes: true,
        }
    }
}

// ============================================================================
// Root Simulation Configuration
// ============================================================================

/// Root simulation configuration
///
/// Contains all parameters needed to simulate CRL-200S hardware.
#[derive(Debug, Clone, Deserialize)]
pub struct SimulationConfig {
    /// Map file path (YAML file referencing PGM)
    pub map_file: String,

    /// Initial robot X position (meters)
    #[serde(default)]
    pub start_x: f32,

    /// Initial robot Y position (meters)
    #[serde(default)]
    pub start_y: f32,

    /// Initial robot orientation (radians, CCW from +X)
    #[serde(default)]
    pub start_theta: f32,

    /// Simulation speed multiplier (1.0 = real-time)
    #[serde(default = "default_speed_factor")]
    pub speed_factor: f32,

    /// Random seed for reproducible noise (0 = random each run)
    #[serde(default)]
    pub random_seed: u64,

    /// Logging level: "minimal", "standard", "verbose"
    #[serde(default = "default_log_level")]
    pub log_level: String,

    /// Robot physical parameters
    #[serde(default)]
    pub robot: RobotConfig,

    /// Lidar configuration
    #[serde(default)]
    pub lidar: LidarConfig,

    /// IMU configuration
    #[serde(default)]
    pub imu: ImuConfig,

    /// Encoder configuration
    #[serde(default)]
    pub encoder: EncoderConfig,

    /// Bumper configuration
    #[serde(default)]
    pub bumpers: BumpersConfig,

    /// Cliff sensors configuration
    #[serde(default)]
    pub cliffs: CliffsConfig,

    /// Binary sensors configuration
    #[serde(default)]
    pub sensors: SensorsConfig,

    /// Actuator configuration
    #[serde(default)]
    pub actuators: ActuatorsConfig,
}

fn default_speed_factor() -> f32 {
    1.0
}
fn default_log_level() -> String {
    "standard".to_string()
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            map_file: String::new(),
            start_x: 0.0,
            start_y: 0.0,
            start_theta: 0.0,
            speed_factor: default_speed_factor(),
            random_seed: 0,
            log_level: default_log_level(),
            robot: RobotConfig::default(),
            lidar: LidarConfig::default(),
            imu: ImuConfig::default(),
            encoder: EncoderConfig::default(),
            bumpers: BumpersConfig::default(),
            cliffs: CliffsConfig::default(),
            sensors: SensorsConfig::default(),
            actuators: ActuatorsConfig::default(),
        }
    }
}
