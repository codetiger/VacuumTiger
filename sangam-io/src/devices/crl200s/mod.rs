//! CRL-200S Vacuum Robot driver - reference implementation for new devices.
//!
//! This driver manages two subcomponents:
//! - **GD32**: Motor controller (wheels, brushes, vacuum) via `/dev/ttyS3`
//! - **Delta2D**: Lidar sensor via `/dev/ttyS1`
//!
//! # Sensor Groups Published
//!
//! ## `sensor_status` (500Hz from GD32)
//! Published on topic `sensors/sensor_status`. Contains all real-time sensor data:
//! - **Bumpers**: `bumper_left`, `bumper_right` (Bool)
//! - **Cliffs**: `cliff_left_side`, `cliff_left_front`, `cliff_right_front`, `cliff_right_side` (Bool)
//! - **Battery**: `battery_voltage` (F32 volts), `battery_level` (U8 %), `is_charging` (Bool)
//! - **Encoders**: `wheel_left`, `wheel_right` (U16 ticks)
//! - **IMU**: `gyro_x/y/z`, `accel_x/y/z`, `tilt_x/y/z` (I16 raw)
//! - **Buttons**: `start_button`, `dock_button` (U16)
//! - **Misc**: `dustbox_attached`, `is_dock_connected` (Bool)
//!
//! ## `device_version` (one-time from GD32)
//! Published on topic `sensors/device_version` after first GD32 packet:
//! - `version_string`: GD32 firmware version (String)
//! - `version_code`: Numeric version code (I32)
//!
//! ## `lidar` (5Hz from Delta2D)
//! Published on topic `sensors/lidar`. Contains 360-degree scan:
//! - `scan`: PointCloud2D with (angle_rad, distance_m, quality) tuples

pub mod constants;
pub mod delta2d;
pub mod gd32;

use crate::config::DeviceConfig;
use crate::core::driver::DeviceDriver;
use crate::core::types::{Command, SensorGroupData};
use crate::error::Result;
use delta2d::Delta2DDriver;
use gd32::GD32Driver;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// CRL-200S device driver coordinating GD32 motor controller and Delta2D lidar.
pub struct CRL200SDriver {
    config: DeviceConfig,
    /// Motor controller - `None` before `initialize()`, `Some` after
    gd32: Option<GD32Driver>,
    /// Lidar driver - `None` before `initialize()`, `Some` after
    lidar: Option<Delta2DDriver>,
}

impl CRL200SDriver {
    /// Create a new CRL-200S driver
    pub fn new(config: DeviceConfig) -> Result<Self> {
        Ok(Self {
            config,
            gd32: None,
            lidar: None,
        })
    }

    /// Shutdown all subsystems (lidar driver, GD32)
    fn shutdown_all(&mut self) {
        // Shutdown lidar driver first
        if let Some(ref mut lidar) = self.lidar {
            let _ = lidar.shutdown();
        }
        // Shutdown GD32 (handles lidar motor shutdown via Shutdown command)
        if let Some(ref mut gd32) = self.gd32 {
            let _ = gd32.shutdown();
        }
    }
}

impl DeviceDriver for CRL200SDriver {
    fn initialize(&mut self) -> Result<HashMap<String, Arc<Mutex<SensorGroupData>>>> {
        log::info!("Initializing CRL-200S device: {}", self.config.name);

        let mut sensor_data = HashMap::new();

        // Create GD32 status sensor group (500Hz telemetry)
        // Contains: bumpers, cliffs, battery, encoders, IMU, buttons
        // See module docs for complete field list
        let sensor_status = SensorGroupData::new("sensor_status");
        let gd32_data = Arc::new(Mutex::new(sensor_status));
        sensor_data.insert("sensor_status".to_string(), gd32_data.clone());
        log::info!("Created sensor group 'sensor_status' (bumpers, cliffs, IMU @ 500Hz)");

        // Create GD32 version sensor group (one-time after boot)
        // Contains: version_string, version_code
        let device_version = SensorGroupData::new("device_version");
        let version_data = Arc::new(Mutex::new(device_version));
        sensor_data.insert("device_version".to_string(), version_data.clone());
        log::info!("Created sensor group 'device_version' (GD32 firmware version)");

        // Create lidar sensor group (5Hz scan data)
        // Contains: scan (PointCloud2D)
        let lidar_group = SensorGroupData::new("lidar");
        let lidar_data = Arc::new(Mutex::new(lidar_group));
        sensor_data.insert("lidar".to_string(), lidar_data.clone());
        log::info!("Created sensor group 'lidar' (360° point cloud @ 5Hz)");

        // Initialize GD32 motor controller
        let mut gd32 = GD32Driver::new(
            &self.config.hardware.gd32_port,
            self.config.hardware.heartbeat_interval_ms,
        )?;

        // Send initialization sequence
        gd32.initialize()?;

        // Start reader and heartbeat threads
        gd32.start(gd32_data, Some(version_data))?;

        self.gd32 = Some(gd32);

        // Initialize lidar driver
        // Driver starts but lidar motor is OFF - will be enabled via command
        let mut lidar = Delta2DDriver::new(&self.config.hardware.lidar_port);
        lidar.start(lidar_data)?;
        self.lidar = Some(lidar);
        log::info!("Lidar driver started (motor OFF - use ComponentControl to enable)");

        log::info!("CRL-200S device initialized");
        Ok(sensor_data)
    }

    fn send_command(&mut self, cmd: Command) -> Result<()> {
        // Handle shutdown command specially
        if matches!(cmd, Command::Shutdown) {
            self.shutdown_all();
            return Ok(());
        }

        // Forward all commands to GD32 (including lidar)
        if let Some(ref gd32) = self.gd32 {
            gd32.send_command(cmd)?;
        }

        Ok(())
    }
}

impl Drop for CRL200SDriver {
    fn drop(&mut self) {
        self.shutdown_all();
    }
}
