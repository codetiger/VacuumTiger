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
    PointCloud2D(Vec<(f32, f32)>), // (angle_rad, distance_m)
}

/// Actuator types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ActuatorType {
    DifferentialDrive {
        modes: Vec<String>,
        default_mode: String,
    },
    Pwm {
        min: i32,
        max: i32,
    },
}

/// Sensor specification from config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorSpec {
    pub id: String,
    #[serde(rename = "type")]
    pub sensor_type: SensorType,
    pub unit: Option<String>,
}

/// Sensor group specification from config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorGroupSpec {
    pub id: String,
    pub source: String,
    #[serde(default = "default_poll_interval")]
    pub poll_interval_ms: u64,
    pub sensors: Vec<SensorSpec>,
}

fn default_poll_interval() -> u64 {
    10 // 100Hz default
}

/// Actuator specification from config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorSpec {
    pub id: String,
    #[serde(flatten)]
    pub config: ActuatorType,
}

/// Device capabilities from config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceCapabilities {
    pub sensor_groups: Vec<SensorGroupSpec>,
    pub actuators: Vec<ActuatorSpec>,
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

/// Commands to device
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum Command {
    // Motion Control
    /// Velocity mode: linear (m/s) + angular (rad/s)
    SetVelocity { linear: f32, angular: f32 },
    /// Tank mode: left/right wheel speeds (m/s)
    SetTankDrive { left: f32, right: f32 },
    /// Graceful deceleration to stop
    Stop,
    /// Immediate halt, no deceleration
    EmergencyStop,

    // Actuator Control
    /// Single actuator control (0-100%)
    SetActuator { id: String, value: f32 },
    /// Atomic multi-actuator control
    SetActuatorMultiple { actuators: Vec<(String, f32)> },

    // Sensor Configuration
    /// Configure sensor parameters at runtime
    SetSensorConfig {
        sensor_id: String,
        config: std::collections::HashMap<String, SensorValue>,
    },
    /// Reset sensor to factory defaults
    ResetSensor { sensor_id: String },
    /// Enable a sensor
    EnableSensor { sensor_id: String },
    /// Disable a sensor
    DisableSensor { sensor_id: String },

    // Safety Configuration
    /// Set velocity limits
    SetSafetyLimits {
        max_linear: Option<f32>,
        max_angular: Option<f32>,
    },
    /// Clear emergency stop state
    ClearEmergencyStop,

    // System Lifecycle
    /// Enter low-power mode
    Sleep,
    /// Exit low-power mode
    Wake,
    /// Graceful daemon shutdown
    Shutdown,
    /// Restart device driver
    Restart,
}
