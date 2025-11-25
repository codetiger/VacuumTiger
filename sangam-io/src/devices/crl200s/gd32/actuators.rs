//! Actuator control operations for GD32 driver
//!
//! This module contains functions for controlling actuators (vacuum, brushes, lidar).

use super::protocol::{
    cmd_air_pump, cmd_lidar_power, cmd_lidar_pwm, cmd_main_brush, cmd_motor_mode,
    cmd_motor_velocity, cmd_side_brush, Packet,
};
use super::state::ActuatorState;
use crate::error::{Error, Result};
use serialport::SerialPort;
use std::io::Write;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};

/// Helper method to send actuator command and update state
///
/// This consolidates the repeated actuator handling logic used in:
/// - Heartbeat loop (periodic refresh)
/// - SetActuator command handler
/// - SetActuatorMultiple command handler
///
/// Returns `None` if the actuator ID is unknown.
pub(super) fn actuator_command(
    actuator_state: &Arc<ActuatorState>,
    id: &str,
    speed: u8,
) -> Option<Packet> {
    match id {
        "vacuum" => {
            actuator_state.vacuum.store(speed, Ordering::Relaxed);
            Some(cmd_air_pump(speed))
        }
        "brush" => {
            actuator_state.main_brush.store(speed, Ordering::Relaxed);
            Some(cmd_main_brush(speed))
        }
        "side_brush" => {
            actuator_state.side_brush.store(speed, Ordering::Relaxed);
            Some(cmd_side_brush(speed))
        }
        _ => {
            log::warn!("Unknown actuator: {}", id);
            None
        }
    }
}

/// Enable or disable the lidar motor via GD32
pub(super) fn enable_lidar(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    actuator_state: &Arc<ActuatorState>,
    enable: bool,
    pwm_speed: i32,
) -> Result<()> {
    let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;

    if enable {
        log::info!("Enabling lidar (PWM: {}%)", pwm_speed);

        // Send power on
        let power_bytes = cmd_lidar_power(true).to_bytes();
        log::debug!("Sending lidar power on: {:02X?}", power_bytes);
        port_guard.write_all(&power_bytes).map_err(Error::Io)?;
        drop(port_guard);

        // Set PWM speed
        let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;
        let pwm_bytes = cmd_lidar_pwm(pwm_speed).to_bytes();
        log::debug!("Sending lidar PWM: {:02X?}", pwm_bytes);
        port_guard.write_all(&pwm_bytes).map_err(Error::Io)?;

        // Store lidar state for heartbeat to send continuously
        actuator_state.lidar_enabled.store(true, Ordering::Relaxed);
        actuator_state
            .lidar_pwm
            .store(pwm_speed as u8, Ordering::Relaxed);

        log::info!("Lidar enabled");
    } else {
        log::info!("Disabling lidar");

        // Clear lidar state
        actuator_state.lidar_enabled.store(false, Ordering::Relaxed);
        actuator_state.lidar_pwm.store(0, Ordering::Relaxed);

        // Set PWM to 0
        let pwm_bytes = cmd_lidar_pwm(0).to_bytes();
        port_guard.write_all(&pwm_bytes).map_err(Error::Io)?;
        drop(port_guard);

        // Power off
        let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;
        let power_bytes = cmd_lidar_power(false).to_bytes();
        port_guard.write_all(&power_bytes).map_err(Error::Io)?;

        log::info!("Lidar disabled");
    }

    Ok(())
}

/// Execute emergency stop sequence
///
/// Clears all actuator states and sends stop commands in the correct sequence:
/// 1. Clear all atomic state
/// 2. Stop all actuators (vacuum, brushes, lidar)
/// 3. Stop motors
/// 4. Exit navigation mode
pub(super) fn emergency_stop(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    actuator_state: &Arc<ActuatorState>,
) -> Result<()> {
    log::warn!("EMERGENCY STOP initiated!");

    // Clear all actuator states first (per user requirement)
    actuator_state.clear_all();

    // Send all actuator stop commands BEFORE motor velocity
    let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;

    // Stop all actuators
    let _ = port_guard.write_all(&cmd_air_pump(0).to_bytes());
    let _ = port_guard.write_all(&cmd_main_brush(0).to_bytes());
    let _ = port_guard.write_all(&cmd_side_brush(0).to_bytes());
    let _ = port_guard.write_all(&cmd_lidar_pwm(0).to_bytes());
    let _ = port_guard.write_all(&cmd_lidar_power(false).to_bytes());

    // Stop motors
    let _ = port_guard.write_all(&cmd_motor_velocity(0, 0).to_bytes());

    // Exit navigation mode
    let _ = port_guard.write_all(&cmd_motor_mode(0x00).to_bytes());

    log::warn!("Emergency stop complete - all actuators and motors stopped");
    Ok(())
}
