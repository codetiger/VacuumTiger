//! Command handling for GD32 driver
//!
//! This module processes high-level Command enums and translates them to GD32 protocol packets.
//! All commands use the unified `ComponentControl` pattern.
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

use super::packet::{protocol_sync_packet, TxPacket};
use super::state::ComponentState;
use crate::core::types::{Command, ComponentAction, SensorValue};
use crate::error::{Error, Result};
use serialport::SerialPort;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};

// ============================================================================
// Constants
// ============================================================================

/// Conversion factor from m/s to GD32 velocity units (mm/s)
const VELOCITY_TO_DEVICE_UNITS: f32 = 1000.0;

/// Default IMU calibration payload observed in R2D logs
const IMU_DEFAULT_PAYLOAD: [u8; 4] = [0x10, 0x0E, 0x00, 0x00];

// Component IDs - use these instead of string literals to catch typos at compile time
const ID_DRIVE: &str = "drive";
const ID_VACUUM: &str = "vacuum";
const ID_MAIN_BRUSH: &str = "main_brush";
const ID_SIDE_BRUSH: &str = "side_brush";
const ID_WATER_PUMP: &str = "water_pump";
const ID_LED: &str = "led";
const ID_LIDAR: &str = "lidar";
const ID_IMU: &str = "imu";
const ID_COMPASS: &str = "compass";
const ID_CLIFF_IR: &str = "cliff_ir";
const ID_MAIN_BOARD: &str = "main_board";
const ID_CHARGER: &str = "charger";
const ID_MCU: &str = "mcu";

// ============================================================================
// Helpers
// ============================================================================

/// Helper to send a TxPacket over the serial port
fn send_packet(port: &Arc<Mutex<Box<dyn SerialPort>>>, pkt: &TxPacket) -> Result<()> {
    let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;
    pkt.send_to(&mut *port_guard).map_err(Error::Io)?;
    Ok(())
}

/// Handle Enable/Disable/Configure for speed-based components (vacuum, main_brush, side_brush, water_pump)
///
/// These components share identical behavior:
/// - Enable: Set to 100%
/// - Disable: Set to 0%
/// - Configure: Set to specified speed (0-100)
fn handle_speed_component(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    state_field: &AtomicU8,
    pkt: &mut TxPacket,
    set_fn: fn(&mut TxPacket, u8),
    name: &str,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Enable { .. } => {
            log::info!("{} enable (100%)", name);
            state_field.store(100, Ordering::Relaxed);
            set_fn(pkt, 100);
            send_packet(port, pkt)
        }
        ComponentAction::Disable { .. } => {
            log::info!("{} disable", name);
            state_field.store(0, Ordering::Relaxed);
            set_fn(pkt, 0);
            send_packet(port, pkt)
        }
        ComponentAction::Configure { ref config } => {
            if let Some(SensorValue::U8(speed)) = config.get("speed") {
                log::info!("{} speed={}", name, speed);
                state_field.store(*speed, Ordering::Relaxed);
                set_fn(pkt, *speed);
                send_packet(port, pkt)?;
            }
            Ok(())
        }
        _ => Err(Error::NotImplemented(format!(
            "{} does not support {:?}",
            name, action
        ))),
    }
}

/// Execute emergency stop sequence
///
/// Clears all component states and sends stop commands in the correct sequence:
/// 1. Clear all atomic state
/// 2. Stop all components (vacuum, brushes, lidar)
/// 3. Stop motors
/// 4. Exit navigation mode
fn emergency_stop(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    component_state: &Arc<ComponentState>,
    pkt: &mut TxPacket,
) -> Result<()> {
    log::warn!("EMERGENCY STOP initiated!");

    // Clear all component states first
    component_state.clear_all();

    // Send all component stop commands BEFORE motor velocity
    let mut port_guard = port.lock().map_err(|_| Error::MutexPoisoned)?;

    // Stop all components
    pkt.set_air_pump(0);
    let _ = pkt.send_to(&mut *port_guard);

    pkt.set_main_brush(0);
    let _ = pkt.send_to(&mut *port_guard);

    pkt.set_side_brush(0);
    let _ = pkt.send_to(&mut *port_guard);

    pkt.set_water_pump(0);
    let _ = pkt.send_to(&mut *port_guard);

    pkt.set_lidar_pwm(0);
    let _ = pkt.send_to(&mut *port_guard);

    pkt.set_lidar_power(false);
    let _ = pkt.send_to(&mut *port_guard);

    // Stop motors
    pkt.set_velocity(0, 0);
    let _ = pkt.send_to(&mut *port_guard);

    // Exit navigation mode
    pkt.set_motor_mode(0x00);
    let _ = pkt.send_to(&mut *port_guard);

    log::warn!("Emergency stop complete - all components and motors stopped");
    Ok(())
}

/// Send a command to the GD32
///
/// This method processes high-level `Command` enums and translates them to
/// GD32 protocol packets. All control uses the unified `ComponentControl` pattern.
///
/// # Component Control
///
/// Unified control for all sensors and components via `ComponentControl`:
/// - `drive`: Enable(mode), Disable (stop + mode 0x00), Reset (emergency stop), Configure(velocity/tank)
/// - `vacuum`, `main_brush`, `side_brush`: Enable/Disable/Configure(speed)
/// - `led`: Configure(state)
/// - `lidar`: Enable(pwm)/Disable/Configure(pwm)
/// - `imu`: Enable (query state), Reset (factory calibrate)
/// - `compass`: Enable (query state), Reset (start calibration)
/// - `cliff_ir`: Enable/Disable/Configure(direction)
///
/// # Lifecycle Commands
///
/// - `Shutdown`: Sets shutdown flag to stop threads gracefully
pub(super) fn send_command(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    component_state: &Arc<ComponentState>,
    shutdown: &Arc<AtomicBool>,
    cmd: Command,
) -> Result<()> {
    // Single TxPacket reused for all commands in this call
    let mut pkt = TxPacket::new();

    match cmd {
        // Unified Component Control
        Command::ComponentControl { ref id, ref action } => {
            handle_component_control(port, component_state, &mut pkt, id, action)
        }

        // Protocol Commands
        Command::ProtocolSync => {
            log::info!("Protocol sync (0x0C)");
            let sync_pkt = protocol_sync_packet();
            send_packet(port, &sync_pkt)
        }

        // System Lifecycle
        Command::Shutdown => {
            shutdown.store(true, Ordering::Relaxed);
            Ok(())
        }
    }
}

/// Handle ComponentControl commands for all sensors and components
fn handle_component_control(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    component_state: &Arc<ComponentState>,
    pkt: &mut TxPacket,
    id: &str,
    action: &ComponentAction,
) -> Result<()> {
    match id {
        // === DRIVE (motion control) ===
        ID_DRIVE => handle_drive(port, component_state, pkt, action),

        // === SPEED-BASED COMPONENTS (vacuum, main_brush, side_brush, water_pump) ===
        ID_VACUUM => handle_speed_component(
            port,
            &component_state.vacuum,
            pkt,
            TxPacket::set_air_pump,
            "Vacuum",
            action,
        ),
        ID_MAIN_BRUSH => handle_speed_component(
            port,
            &component_state.main_brush,
            pkt,
            TxPacket::set_main_brush,
            "Main brush",
            action,
        ),
        ID_SIDE_BRUSH => handle_speed_component(
            port,
            &component_state.side_brush,
            pkt,
            TxPacket::set_side_brush,
            "Side brush",
            action,
        ),
        ID_WATER_PUMP => handle_speed_component(
            port,
            &component_state.water_pump,
            pkt,
            TxPacket::set_water_pump,
            "Water pump",
            action,
        ),

        // === LED ===
        ID_LED => handle_led(port, pkt, action),

        // === LIDAR ===
        ID_LIDAR => handle_lidar(port, component_state, pkt, action),

        // === IMU ===
        ID_IMU => handle_imu(port, pkt, action),

        // === COMPASS ===
        ID_COMPASS => handle_compass(port, pkt, action),

        // === CLIFF IR ===
        ID_CLIFF_IR => handle_cliff_ir(port, pkt, action),

        // === POWER MANAGEMENT ===
        ID_MAIN_BOARD => handle_main_board(port, pkt, action),
        ID_CHARGER => handle_charger(port, pkt, action),
        ID_MCU => handle_mcu(port, pkt, action),

        // === UNSUPPORTED ===
        _ => Err(Error::NotImplemented(format!(
            "ComponentControl id='{}' action={:?}",
            id, action
        ))),
    }
}

// ============================================================================
// Component-specific handlers
// ============================================================================

/// Handle drive (motion control) commands
fn handle_drive(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    component_state: &Arc<ComponentState>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Enable { ref config } => {
            // Enable motor with mode (default 0x02 nav mode)
            let mode = config
                .as_ref()
                .and_then(|c| c.get("mode"))
                .and_then(|v| match v {
                    SensorValue::U8(m) => Some(*m),
                    _ => None,
                })
                .unwrap_or(0x02);
            log::info!("Drive enable (mode 0x{:02X})", mode);
            component_state
                .wheel_motor_enabled
                .store(true, Ordering::Relaxed);
            pkt.set_motor_mode(mode);
            send_packet(port, pkt)
        }
        ComponentAction::Disable { .. } => {
            // Stop: zero velocity and set mode 0x00
            log::info!("Drive disable (stop + mode 0x00)");
            component_state
                .wheel_motor_enabled
                .store(false, Ordering::Relaxed);
            component_state.linear_velocity.store(0, Ordering::Relaxed);
            component_state.angular_velocity.store(0, Ordering::Relaxed);
            // Send velocity 0,0 then mode 0x00
            pkt.set_velocity(0, 0);
            send_packet(port, pkt)?;
            pkt.set_motor_mode(0x00);
            send_packet(port, pkt)
        }
        ComponentAction::Reset { .. } => {
            // Emergency stop: immediate halt, all components off
            log::info!("Drive emergency stop");
            emergency_stop(port, component_state, pkt)
        }
        ComponentAction::Configure { ref config } => {
            // Check for velocity mode (linear + angular) - continuous
            if let (Some(SensorValue::F32(linear)), Some(SensorValue::F32(angular))) =
                (config.get("linear"), config.get("angular"))
            {
                let linear_units = (linear * VELOCITY_TO_DEVICE_UNITS) as i16;
                let angular_units = (angular * VELOCITY_TO_DEVICE_UNITS) as i16;
                // Store velocity for heartbeat to send continuously
                component_state
                    .linear_velocity
                    .store(linear_units, Ordering::Relaxed);
                component_state
                    .angular_velocity
                    .store(angular_units, Ordering::Relaxed);
                log::info!(
                    "Drive velocity: linear={:.3} m/s ({} units), angular={:.3} rad/s ({} units)",
                    linear,
                    linear_units,
                    angular,
                    angular_units
                );
                pkt.set_velocity(linear_units, angular_units);
                return send_packet(port, pkt);
            }
            // Check for tank drive mode (left + right) - continuous
            if let (Some(SensorValue::F32(left)), Some(SensorValue::F32(right))) =
                (config.get("left"), config.get("right"))
            {
                let left_units = (left * VELOCITY_TO_DEVICE_UNITS) as i16;
                let right_units = (right * VELOCITY_TO_DEVICE_UNITS) as i16;
                log::info!(
                    "Drive tank: left={:.3} m/s ({} units), right={:.3} m/s ({} units)",
                    left,
                    left_units,
                    right,
                    right_units
                );
                pkt.set_motor_speed(left_units, right_units);
                return send_packet(port, pkt);
            }
            Err(Error::InvalidParameter(
                "drive Configure requires (linear, angular) or (left, right)".into(),
            ))
        }
    }
}

/// Handle LED commands
fn handle_led(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Configure { ref config } => {
            if let Some(SensorValue::U8(state)) = config.get("state") {
                log::info!("LED state={}", state);
                pkt.set_led(*state);
                send_packet(port, pkt)?;
            }
            Ok(())
        }
        _ => Err(Error::NotImplemented(format!(
            "LED only supports Configure, got {:?}",
            action
        ))),
    }
}

/// Handle lidar commands
fn handle_lidar(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    component_state: &Arc<ComponentState>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Enable { ref config } => {
            // Get PWM from config or use default (60%)
            let pwm = config
                .as_ref()
                .and_then(|c| c.get("pwm"))
                .and_then(|v| match v {
                    SensorValue::U8(p) => Some(*p),
                    _ => None,
                })
                .unwrap_or(60);

            log::info!("Lidar enable (PWM: {}%)", pwm);

            // Power on first, then set PWM
            pkt.set_lidar_power(true);
            send_packet(port, pkt)?;
            pkt.set_lidar_pwm(pwm);
            send_packet(port, pkt)?;

            // Update state for heartbeat
            component_state.lidar_enabled.store(true, Ordering::Relaxed);
            component_state.lidar_pwm.store(pwm, Ordering::Relaxed);

            Ok(())
        }
        ComponentAction::Disable { .. } => {
            log::info!("Lidar disable");

            // Clear state first
            component_state
                .lidar_enabled
                .store(false, Ordering::Relaxed);
            component_state.lidar_pwm.store(0, Ordering::Relaxed);

            // PWM to 0 first, then power off
            pkt.set_lidar_pwm(0);
            send_packet(port, pkt)?;
            pkt.set_lidar_power(false);
            send_packet(port, pkt)
        }
        ComponentAction::Configure { ref config } => {
            if let Some(SensorValue::U8(pwm)) = config.get("pwm") {
                log::info!("Lidar PWM: {}%", pwm);
                component_state.lidar_pwm.store(*pwm, Ordering::Relaxed);
                pkt.set_lidar_pwm(*pwm);
                send_packet(port, pkt)?;
            }
            Ok(())
        }
        _ => Err(Error::NotImplemented(format!(
            "Lidar does not support {:?}",
            action
        ))),
    }
}

/// Handle IMU commands
fn handle_imu(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Enable { .. } => {
            pkt.set_imu_calibrate_state(&IMU_DEFAULT_PAYLOAD);
            log::info!(
                "IMU calibration state query (0xA2): payload={:02X?}, bytes={:02X?}",
                IMU_DEFAULT_PAYLOAD,
                pkt.as_bytes()
            );
            send_packet(port, pkt)
        }
        ComponentAction::Reset { .. } => {
            log::info!("IMU factory reset (0xA1)");
            pkt.set_imu_factory_calibrate();
            send_packet(port, pkt)
        }
        _ => Err(Error::NotImplemented(format!(
            "IMU only supports Enable/Reset, got {:?}",
            action
        ))),
    }
}

/// Handle compass commands
fn handle_compass(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Enable { .. } => {
            log::info!("Compass calibration state query (0xA4)");
            pkt.set_compass_calibration_state();
            send_packet(port, pkt)
        }
        ComponentAction::Reset { .. } => {
            log::info!("Compass calibration start (0xA3)");
            pkt.set_compass_calibrate();
            send_packet(port, pkt)
        }
        _ => Err(Error::NotImplemented(format!(
            "Compass only supports Enable/Reset, got {:?}",
            action
        ))),
    }
}

/// Handle cliff IR commands
fn handle_cliff_ir(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Enable { .. } => {
            log::info!("Cliff IR enable (0x78)");
            pkt.set_cliff_ir(true);
            send_packet(port, pkt)
        }
        ComponentAction::Disable { .. } => {
            log::info!("Cliff IR disable (0x78)");
            pkt.set_cliff_ir(false);
            send_packet(port, pkt)
        }
        ComponentAction::Configure { ref config } => {
            if let Some(SensorValue::U8(dir)) = config.get("direction") {
                log::info!("Cliff IR direction (0x79): {}", dir);
                pkt.set_cliff_ir_direction(*dir);
                send_packet(port, pkt)?;
            }
            Ok(())
        }
        _ => Err(Error::NotImplemented(format!(
            "Cliff IR does not support {:?}",
            action
        ))),
    }
}

/// Handle main board (A33) power commands
///
/// Controls power to the A33 main application board running Linux.
/// - Enable: Power on main board
/// - Disable: Power off main board (WARNING: terminates daemon!)
/// - Reset: Restart main board (WARNING: terminates daemon!)
fn handle_main_board(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Enable { .. } => {
            log::info!("Main board power on (0x99)");
            pkt.set_main_board_power(true);
            send_packet(port, pkt)
        }
        ComponentAction::Disable { .. } => {
            log::warn!("Main board power off (0x99) - daemon will terminate!");
            pkt.set_main_board_power(false);
            send_packet(port, pkt)
        }
        ComponentAction::Reset { .. } => {
            log::warn!("Main board restart (0x9A) - daemon will terminate!");
            pkt.set_main_board_restart();
            send_packet(port, pkt)
        }
        _ => Err(Error::NotImplemented(format!(
            "Main board only supports Enable/Disable/Reset, got {:?}",
            action
        ))),
    }
}

/// Handle charger power commands
///
/// Controls the charger power rail.
/// - Enable: Enable charger power
/// - Disable: Disable charger power
fn handle_charger(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Enable { .. } => {
            log::info!("Charger power enable (0x9B)");
            pkt.set_charger_power(true);
            send_packet(port, pkt)
        }
        ComponentAction::Disable { .. } => {
            log::info!("Charger power disable (0x9B)");
            pkt.set_charger_power(false);
            send_packet(port, pkt)
        }
        _ => Err(Error::NotImplemented(format!(
            "Charger only supports Enable/Disable, got {:?}",
            action
        ))),
    }
}

/// Handle MCU control commands
///
/// Controls the GD32 MCU power state and error codes:
/// - Disable: Put MCU to sleep (0x04)
/// - Enable: Acknowledge wakeup from sleep (0x05)
/// - Reset: Clear/reset error codes (0x0A)
fn handle_mcu(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Disable { .. } => {
            log::info!("MCU sleep (0x04)");
            pkt.set_mcu_sleep();
            send_packet(port, pkt)
        }
        ComponentAction::Enable { .. } => {
            log::info!("MCU wakeup ack (0x05)");
            pkt.set_wakeup_ack();
            send_packet(port, pkt)
        }
        ComponentAction::Reset { .. } => {
            log::info!("MCU reset error code (0x0A)");
            pkt.set_reset_error_code();
            send_packet(port, pkt)
        }
        _ => Err(Error::NotImplemented(format!(
            "MCU only supports Enable/Disable/Reset, got {:?}",
            action
        ))),
    }
}
