//! Core data types for SangamIO

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Sensor data types matching hardware precision
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorType {
    Bool,
    U8,
    U16,
    U32,
    U64,
    I8,
    I16,
    I32,
    I64,
    F32,
    F64,
    String,
    Vector3,
    PointCloud2D,
}

/// Runtime sensor values
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorValue {
    Bool(bool),
    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),
    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),
    F32(f32),
    F64(f64),
    String(String),
    Vector3([f32; 3]),
    PointCloud2D(Vec<(f32, f32, u8)>), // (angle_rad, distance_m, quality)
}

/// Sensor specification from config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorSpec {
    pub id: String,
    #[serde(rename = "type")]
    pub sensor_type: SensorType,
}

/// Sensor group specification from config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorGroupSpec {
    pub id: String,
    pub sensors: Vec<SensorSpec>,
}

/// Runtime sensor group data (shared between threads)
#[derive(Debug, Clone)]
pub struct SensorGroupData {
    pub group_id: String,
    pub timestamp_us: u64,
    pub values: HashMap<String, SensorValue>,
}

impl SensorGroupData {
    /// Create a new SensorGroupData with pre-allocated keys
    ///
    /// This avoids HashMap allocations in the hot loop - values are updated in-place
    pub fn new_with_keys(group_id: &str, keys: &[(&str, SensorValue)]) -> Self {
        let mut values = HashMap::with_capacity(keys.len());
        for (key, default_value) in keys {
            values.insert((*key).to_string(), default_value.clone());
        }

        Self {
            group_id: group_id.to_string(),
            timestamp_us: 0,
            values,
        }
    }

    /// Update a value in-place (no allocation if key exists)
    #[inline]
    pub fn update(&mut self, key: &str, value: SensorValue) {
        if let Some(v) = self.values.get_mut(key) {
            *v = value;
        }
    }

    /// Update timestamp to current time
    #[inline]
    pub fn touch(&mut self) {
        self.timestamp_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
    }
}

/// Actions that can be performed on components (sensors and actuators)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ComponentAction {
    /// Turn on/activate the component with optional config (e.g., mode)
    Enable {
        #[serde(default)]
        config: Option<HashMap<String, SensorValue>>,
    },
    /// Turn off/deactivate the component with optional config
    Disable {
        #[serde(default)]
        config: Option<HashMap<String, SensorValue>>,
    },
    /// Reset to factory defaults / trigger calibration with optional config
    Reset {
        #[serde(default)]
        config: Option<HashMap<String, SensorValue>>,
    },
    /// Configure component parameters (speed, velocity - continuous updates)
    Configure {
        config: HashMap<String, SensorValue>,
    },
}

/// Commands to device
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum Command {
    // Unified Component Control
    /// Control any component (sensor or actuator) with standard actions
    ///
    /// # Motion Control (id: "drive")
    /// - `Configure { linear: F32, angular: F32 }` - velocity mode (m/s, rad/s)
    /// - `Configure { left: F32, right: F32 }` - tank drive mode (m/s)
    /// - `Disable` - stop (zero velocity)
    /// - `Reset` - emergency stop (immediate halt, all actuators off)
    ///
    /// # Actuators
    /// - `wheel_motor`: Enable/Disable motor mode
    /// - `vacuum`, `main_brush`, `side_brush`: Enable/Disable/Configure(speed)
    /// - `led`: Configure(state)
    /// - `lidar`: Enable/Disable
    ///
    /// # Sensors
    /// - `imu`: Enable (query state), Reset (factory calibrate)
    /// - `compass`: Enable (query state), Reset (start calibration)
    /// - `cliff_ir`: Enable/Disable/Configure(direction)
    ComponentControl {
        /// Component identifier (e.g., "drive", "vacuum", "imu", "cliff_ir")
        id: String,
        /// Action to perform
        action: ComponentAction,
    },

    // System Lifecycle
    /// Graceful daemon shutdown
    Shutdown,
}
