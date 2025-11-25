//! Command handling for GD32 driver
//!
//! This module processes high-level Command enums and translates them to GD32 protocol packets.
//!
//! # Unit Conversions
//!
//! The GD32 protocol expects specific units for motion commands:
//!
//! - **Linear velocity**: mm/s (millimeters per second)
//!   - Input: m/s (meters per second)
//!   - Conversion: multiply by 1000
//!   - Range: -32768 to 32767 mm/s (±32.7 m/s max)
//!
//! - **Angular velocity**: mrad/s (milliradians per second)
//!   - Input: rad/s (radians per second)
//!   - Conversion: multiply by 1000
//!   - Range: -32768 to 32767 mrad/s (±32.7 rad/s max)
//!
//! - **Tank drive speeds**: Direct motor units (mm/s equivalent)
//!   - Input: m/s (meters per second)
//!   - Conversion: multiply by 1000
//!
//! These conversions are defined by the GD32F103 firmware protocol.

/// Conversion factor from m/s to GD32 velocity units (mm/s)
const VELOCITY_TO_DEVICE_UNITS: f32 = 1000.0;

use super::actuators;
use super::protocol::{cmd_motor_mode, cmd_motor_speed, cmd_motor_velocity};
use super::state::ActuatorState;
use crate::core::types::Command;
use crate::error::{Error, Result};
use serialport::SerialPort;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// Send a command to the GD32
///
/// This method processes high-level `Command` enums and translates them to
/// GD32 protocol packets. Commands fall into several categories:
///
/// # Motion Commands
///
/// - `SetVelocity`: Stores velocity in atomic state for heartbeat to send continuously
/// - `SetTankDrive`: Direct left/right wheel speeds (bypasses velocity control)
/// - `Stop`: Zeroes velocity (heartbeat continues sending 0,0)
/// - `EmergencyStop`: Immediately stops all actuators and motors, clears all state
///
/// # Actuator Commands
///
/// - `SetActuator`: Single actuator (vacuum, brush, side_brush)
/// - `SetActuatorMultiple`: Batch actuator update (more efficient)
///
/// Both store state atomically and send commands immediately. The heartbeat thread
/// will continue refreshing these commands every 20ms.
///
/// # Sensor Commands
///
/// - `EnableSensor`/`DisableSensor` for "wheel_motor": Controls motor mode 0x02
/// - Other sensors: NotImplemented (GD32 doesn't support runtime sensor config)
///
/// # Lifecycle Commands
///
/// - `Shutdown`: Sets shutdown flag to stop threads gracefully
/// - Others: NotImplemented (Sleep/Wake/Restart not supported by GD32 protocol)
pub(super) fn send_command(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    actuator_state: &Arc<ActuatorState>,
    shutdown: &Arc<AtomicBool>,
    cmd: Command,
) -> Result<()> {
    let packet = match cmd {
        // Motion Control
        Command::SetVelocity { linear, angular } => {
            // Convert m/s → mm/s and rad/s → mrad/s for GD32 protocol
            let linear_units = (linear * VELOCITY_TO_DEVICE_UNITS) as i16;
            let angular_units = (angular * VELOCITY_TO_DEVICE_UNITS) as i16;
            // Store velocity for heartbeat to send continuously
            actuator_state
                .linear_velocity
                .store(linear_units, Ordering::Relaxed);
            actuator_state
                .angular_velocity
                .store(angular_units, Ordering::Relaxed);
            log::info!(
                "SetVelocity: linear={:.3} m/s ({} units), angular={:.3} rad/s ({} units)",
                linear,
                linear_units,
                angular,
                angular_units
            );
            cmd_motor_velocity(linear_units, angular_units)
        }
        Command::SetTankDrive { left, right } => {
            // Convert left/right wheel speeds from m/s to mm/s
            let left_units = (left * VELOCITY_TO_DEVICE_UNITS) as i16;
            let right_units = (right * VELOCITY_TO_DEVICE_UNITS) as i16;
            cmd_motor_speed(left_units, right_units)
        }
        Command::Stop => cmd_motor_velocity(0, 0),
        Command::EmergencyStop => {
            actuators::emergency_stop(port, actuator_state)?;
            return Ok(());
        }

        // Actuator Control
        Command::SetActuator { ref id, value } => {
            let speed = (value.clamp(0.0, 100.0)) as u8;
            let Some(pkt) = actuators::actuator_command(actuator_state, id, speed) else {
                return Ok(()); // Unknown actuator already logged
            };
            let bytes = pkt.to_bytes();
            log::info!("SetActuator: {} = {}, bytes: {:02X?}", id, speed, bytes);
            let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;
            port_guard.write_all(&bytes).map_err(Error::Io)?;
            return Ok(());
        }
        Command::SetActuatorMultiple { ref actuators } => {
            // Send multiple actuator commands
            for (id, value) in actuators {
                let speed = (value.clamp(0.0, 100.0)) as u8;
                let Some(pkt) = actuators::actuator_command(actuator_state, id, speed) else {
                    continue; // Unknown actuator already logged
                };
                let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;
                port_guard.write_all(&pkt.to_bytes()).map_err(Error::Io)?;
            }
            return Ok(());
        }

        // Sensor Configuration
        Command::SetSensorConfig { ref sensor_id, .. } => {
            return Err(Error::NotImplemented(format!(
                "SetSensorConfig for {}",
                sensor_id
            )));
        }
        Command::ResetSensor { ref sensor_id } => {
            return Err(Error::NotImplemented(format!("ResetSensor {}", sensor_id)));
        }
        Command::EnableSensor { ref sensor_id } => {
            if sensor_id == "wheel_motor" {
                log::info!("Enabling wheel motor");
                actuator_state
                    .wheel_motor_enabled
                    .store(true, Ordering::Relaxed);
                return Ok(());
            }
            return Err(Error::NotImplemented(format!("EnableSensor {}", sensor_id)));
        }
        Command::DisableSensor { ref sensor_id } => {
            if sensor_id == "wheel_motor" {
                log::info!("Disabling wheel motor");
                actuator_state
                    .wheel_motor_enabled
                    .store(false, Ordering::Relaxed);
                // Also stop any motion
                actuator_state.linear_velocity.store(0, Ordering::Relaxed);
                actuator_state.angular_velocity.store(0, Ordering::Relaxed);

                // Send motor mode 0x00 to exit navigation mode
                let mode_pkt = cmd_motor_mode(0x00);
                let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;
                port_guard
                    .write_all(&mode_pkt.to_bytes())
                    .map_err(Error::Io)?;
                log::info!("Motor mode set to idle (0x00)");

                return Ok(());
            }
            return Err(Error::NotImplemented(format!(
                "DisableSensor {}",
                sensor_id
            )));
        }

        // Safety Configuration
        Command::SetSafetyLimits { .. } => {
            return Err(Error::NotImplemented("SetSafetyLimits".to_string()));
        }
        Command::ClearEmergencyStop => {
            return Err(Error::NotImplemented("ClearEmergencyStop".to_string()));
        }

        // System Lifecycle
        Command::Sleep => {
            return Err(Error::NotImplemented("Sleep".to_string()));
        }
        Command::Wake => {
            return Err(Error::NotImplemented("Wake".to_string()));
        }
        Command::Shutdown => {
            shutdown.store(true, Ordering::Relaxed);
            return Ok(());
        }
        Command::Restart => {
            return Err(Error::NotImplemented("Restart".to_string()));
        }
    };

    let bytes = packet.to_bytes();
    let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;
    port_guard.write_all(&bytes).map_err(Error::Io)?;

    Ok(())
}
