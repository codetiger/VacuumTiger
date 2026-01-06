//! Sensor configuration section.

use serde::{Deserialize, Serialize};

use crate::grid::SensorConfig;

use super::defaults;

/// Sensor configuration section
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct SensorSection {
    /// LiDAR settings
    #[serde(default)]
    pub lidar: LidarSettings,

    /// Robot geometry
    #[serde(default)]
    pub robot: RobotSettings,

    /// Cliff sensor positions
    #[serde(default)]
    pub cliff_sensors: CliffSensorSettings,

    /// Bumper settings
    #[serde(default)]
    pub bumper: BumperSettings,
}

impl SensorSection {
    /// Convert to SensorConfig
    pub fn to_sensor_config(&self) -> SensorConfig {
        SensorConfig {
            robot_radius: self.robot.radius,
            confidence_threshold: 3,
            max_lidar_range: self.lidar.max_range,
            min_lidar_range: self.lidar.min_range,
        }
    }
}

/// LiDAR settings
///
/// Note: Lidar mounting offset is now handled by SangamIO.
/// VastuSLAM receives robot-centered lidar data directly.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LidarSettings {
    /// Minimum valid range
    #[serde(default = "defaults::min_range")]
    pub min_range: f32,

    /// Maximum valid range
    #[serde(default = "defaults::max_range")]
    pub max_range: f32,
}

impl Default for LidarSettings {
    fn default() -> Self {
        Self {
            min_range: 0.15,
            max_range: 8.0,
        }
    }
}

/// Robot geometry settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotSettings {
    /// Robot radius (meters)
    #[serde(default = "defaults::robot_radius")]
    pub radius: f32,
}

impl Default for RobotSettings {
    fn default() -> Self {
        Self { radius: 0.17 }
    }
}

/// Cliff sensor positions
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CliffSensorSettings {
    /// Left side sensor position [x, y]
    pub left_side: [f32; 2],
    /// Left front sensor position [x, y]
    pub left_front: [f32; 2],
    /// Right front sensor position [x, y]
    pub right_front: [f32; 2],
    /// Right side sensor position [x, y]
    pub right_side: [f32; 2],
}

impl Default for CliffSensorSettings {
    fn default() -> Self {
        Self {
            left_side: [0.12, 0.10],
            left_front: [0.15, 0.05],
            right_front: [0.15, -0.05],
            right_side: [0.12, -0.10],
        }
    }
}

/// Bumper settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BumperSettings {
    /// Left bumper angle (radians from forward)
    #[serde(default = "defaults::left_bumper_angle")]
    pub left_angle: f32,

    /// Right bumper angle (radians from forward)
    #[serde(default = "defaults::right_bumper_angle")]
    pub right_angle: f32,
}

impl Default for BumperSettings {
    fn default() -> Self {
        Self {
            left_angle: 0.5,
            right_angle: -0.5,
        }
    }
}
