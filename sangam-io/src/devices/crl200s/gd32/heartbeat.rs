//! Heartbeat thread for GD32 driver
//!
//! This module contains the heartbeat loop that maintains the GD32 watchdog timer.
//!
//! # Safety Requirements
//!
//! The GD32F103 microcontroller implements a watchdog timer that requires commands
//! to be sent at regular intervals:
//!
//! - **Minimum interval**: 20ms (50Hz)
//! - **Maximum interval**: 50ms (20Hz)
//! - **Recommended**: 20ms for safety margin
//! - **Consequence of violation**: Motors immediately stop (hardware safety feature)
//!
//! This thread runs with blocking mutex locks (not async) to guarantee timing under load.
//!
//! # Motor Mode Timing
//!
//! When switching from idle (0x00) to navigation mode (0x02):
//! - GD32 requires **100ms processing time** before accepting component commands
//! - This thread waits 100ms after mode switch, then resumes normal heartbeat
//! - Mode switches happen automatically when any component is activated
//!
//! # Known Limitations
//!
//! ## Wheel Motors Require Other Components
//!
//! The GD32 firmware appears to stop wheel motors after ~1-2 seconds if no other
//! component (vacuum, brushes, or lidar) is active. This is likely a safety feature
//! in the stock firmware - R2D always runs lidar during navigation.
//!
//! **Workaround**: Enable lidar (even at low PWM) before enabling wheel motors
//! for sustained operation.

use super::protocol::{
    cmd_air_pump, cmd_heartbeat, cmd_lidar_pwm, cmd_main_brush, cmd_motor_mode, cmd_motor_velocity,
    cmd_request_stm32_data, cmd_side_brush,
};
use super::state::ComponentState;
use serialport::SerialPort;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

/// Heartbeat loop - sends appropriate commands at configured interval
///
/// This loop runs continuously on a dedicated OS thread to maintain the GD32 watchdog timer.
/// The GD32 requires a command every 20-50ms or it will disable motors as a safety feature.
///
/// # Behavior
///
/// **When any component is active (vacuum, brushes, lidar, or wheel_motor_enabled):**
/// 1. Set motor mode to 0x02 (navigation mode) if not already set
/// 2. Send motor mode 0x02 command periodically to maintain state
/// 3. Send velocity command (0x66) with current linear/angular values
/// 4. Send component commands for all active components (speed > 0)
/// 5. Send lidar PWM if lidar is enabled
///
/// **When all components are off:**
/// 1. Clear motor_mode_set flag (allows re-entry to mode 0x02 later)
/// 2. Send regular heartbeat (0x06)
///
/// # Timing
///
/// - Acquires port mutex (blocking)
/// - Sends all commands (<10ms typical)
/// - Releases mutex explicitly before sleeping
/// - Sleeps for `interval_ms` (typically 20ms)
///
/// The blocking mutex acquisition is intentional: we prioritize heartbeat delivery
/// over other operations to maintain safety guarantees.
pub(super) fn heartbeat_loop(
    port: Arc<Mutex<Box<dyn SerialPort>>>,
    shutdown: Arc<AtomicBool>,
    interval_ms: u64,
    component_state: Arc<ComponentState>,
) {
    let heartbeat_pkt = cmd_heartbeat();
    let heartbeat_bytes = heartbeat_pkt.to_bytes();

    // Counter for periodic STM32 data request (0x0D)
    // At 20ms interval, 75 cycles = 1.5 seconds (matching R2D MITM log frequency)
    let stm32_request_interval = 1500 / interval_ms;
    let mut stm32_request_counter: u64 = 0;

    while !shutdown.load(Ordering::Relaxed) {
        // Use blocking lock to ensure commands are always sent
        let Ok(mut port) = port.lock() else {
            log::error!("Heartbeat: mutex poisoned, exiting");
            break;
        };

        // Check component states
        let (vacuum, main_brush, side_brush) = component_state.get_component_speeds();
        let lidar_enabled = component_state.lidar_enabled.load(Ordering::Relaxed);
        let (linear, angular) = component_state.get_velocities();

        // Motor mode is needed if any component is active OR wheel motor is explicitly enabled
        let any_component_active = component_state.any_active();

        // Send motor mode 0x02 when first component is enabled
        if any_component_active && !component_state.motor_mode_set.load(Ordering::Relaxed) {
            let pkt = cmd_motor_mode(0x02);
            if port.write_all(&pkt.to_bytes()).is_ok() {
                component_state
                    .motor_mode_set
                    .store(true, Ordering::Relaxed);
                log::info!("Motor mode set to navigation (0x02)");
                // Wait 100ms for GD32 firmware to process mode switch
                // This is a hardware requirement: the GD32F103 needs this time to
                // reconfigure internal state before it can accept component commands.
                // Releasing the port lock during sleep allows other operations to proceed.
                drop(port);
                thread::sleep(Duration::from_millis(100));
                continue; // Skip commands this cycle, send them next cycle
            }
        } else if !any_component_active && component_state.motor_mode_set.load(Ordering::Relaxed) {
            // Reset flag when all components are off
            component_state
                .motor_mode_set
                .store(false, Ordering::Relaxed);
        }

        if component_state.motor_mode_set.load(Ordering::Relaxed) {
            // Send motor mode 0x02 periodically to keep GD32 in navigation mode
            let mode_pkt = cmd_motor_mode(0x02);
            let _ = port.write_all(&mode_pkt.to_bytes());

            // Motor mode 0x02 active - send velocity command as heartbeat
            let pkt = cmd_motor_velocity(linear, angular);
            if let Err(e) = port.write_all(&pkt.to_bytes()) {
                log::error!("Velocity heartbeat send failed: {}", e);
            } else {
                log::trace!(
                    "Velocity heartbeat sent: linear={}, angular={}",
                    linear,
                    angular
                );
            }

            // Send component commands every cycle
            if vacuum > 0 {
                let pkt = cmd_air_pump(vacuum);
                let _ = port.write_all(&pkt.to_bytes());
            }

            if main_brush > 0 {
                let pkt = cmd_main_brush(main_brush);
                let _ = port.write_all(&pkt.to_bytes());
            }

            if side_brush > 0 {
                let pkt = cmd_side_brush(side_brush);
                let _ = port.write_all(&pkt.to_bytes());
            }

            // Send lidar PWM if enabled
            if lidar_enabled {
                let lidar_pwm = component_state.lidar_pwm.load(Ordering::Relaxed);
                let pkt = cmd_lidar_pwm(lidar_pwm as i32);
                let _ = port.write_all(&pkt.to_bytes());
            }
        } else {
            // No components active - send regular heartbeat
            if let Err(e) = port.write_all(&heartbeat_bytes) {
                log::error!("Heartbeat send failed: {}", e);
            } else {
                log::trace!("Heartbeat sent");
            }
        }

        // Send STM32 data request (0x0D) every ~1.5 seconds
        stm32_request_counter += 1;
        if stm32_request_counter >= stm32_request_interval {
            stm32_request_counter = 0;
            let pkt = cmd_request_stm32_data();
            if let Err(e) = port.write_all(&pkt.to_bytes()) {
                log::warn!("STM32 data request (0x0D) send failed: {}", e);
            } else {
                log::trace!("STM32 data request sent");
            }
        }

        // Explicitly release port mutex before sleeping to allow other threads
        // (reader, command handler) to access the serial port during our sleep period
        drop(port);
        thread::sleep(Duration::from_millis(interval_ms));
    }

    log::info!("Heartbeat thread exiting");
}
