//! CRL-200S Vacuum Robot device driver

pub mod constants;
pub mod delta2d;
pub mod gd32;

use crate::config::DeviceConfig;
use crate::core::driver::DeviceDriver;
use crate::core::types::{Command, SensorGroupData};
use crate::error::{Error, Result};
use delta2d::Delta2DDriver;
use gd32::GD32Driver;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// CRL-200S device driver
pub struct CRL200SDriver {
    config: DeviceConfig,
    gd32: Option<GD32Driver>,
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
    fn initialize(
        &mut self,
        sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
    ) -> Result<()> {
        log::info!("Initializing CRL-200S device: {}", self.config.name);

        // Initialize GD32 motor controller
        let mut gd32 = GD32Driver::new(
            &self.config.hardware.gd32_port,
            self.config.hardware.heartbeat_interval_ms,
        )?;

        // Send initialization sequence
        gd32.initialize()?;

        // Get shared data for gd32_status sensor group
        let gd32_data = sensor_data
            .get("gd32_status")
            .ok_or_else(|| Error::Config("Missing sensor data for gd32_status".to_string()))?
            .clone();

        // Get version data if configured (for one-time read)
        let version_data = sensor_data.get("gd32_version").cloned();

        // Start reader and heartbeat threads
        gd32.start(gd32_data, version_data)?;

        self.gd32 = Some(gd32);

        // Initialize lidar driver if sensor data is configured
        // Driver starts but lidar motor is OFF - will be enabled via command
        if let Some(lidar_data) = sensor_data.get("lidar").cloned() {
            let mut lidar = Delta2DDriver::new(&self.config.hardware.lidar_port);
            lidar.start(lidar_data)?;
            self.lidar = Some(lidar);
            log::info!("Lidar driver started (motor OFF - use ComponentControl to enable)");
        } else {
            log::warn!("No lidar sensor data configured, skipping lidar initialization");
        }

        log::info!("CRL-200S device initialized");
        Ok(())
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
