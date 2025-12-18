//! Command handling for GD32 driver
//!
//! This module processes high-level Command enums and translates them to GD32 protocol packets.
//! All commands use the unified `ComponentControl` pattern.
//!
//! # Unit Conversions
//!
//! The GD32 protocol expects velocity in internal units (empirically calibrated):
//!
//! - **Linear velocity**: Empirical units
//!   - Input: m/s (meters per second)
//!   - Conversion: multiply by 523 (empirically calibrated)
//!
//! - **Angular velocity**: Empirical units
//!   - Input: rad/s (radians per second)
//!   - Conversion: multiply by 523 (empirically calibrated)
//!
//! - **Tank drive speeds**: Same empirical units
//!   - Input: m/s (meters per second)
//!   - Conversion: multiply by 523
//!
//! Note: The conversion factor was calibrated by comparing commanded angular velocity
//! (0.35 rad/s) with encoder-measured actual velocity (0.669 rad/s), giving a
//! correction ratio of 1000/1.91 ≈ 523.
//!
//! # Protobuf Type Handling
//!
//! Protobuf3 does not have native u8/u16/i8/i16 types. All small integers are
//! encoded as u32/i32. This module handles both representations:
//!
//! - **U8 values** (speed, pwm, state): Accept both `SensorValue::U8` and `SensorValue::U32`
//! - Clients may send either depending on their protobuf implementation
//! - Example: `config.get("speed")` checks for both U8 and U32 variants
//!
//! # Component IDs
//!
//! Valid component IDs are defined as constants below. See [`handle_component_control`]
//! for the complete dispatch table.

use super::packet::{TxPacket, protocol_sync_packet};
use super::state::ComponentState;
use crate::core::types::{Command, ComponentAction, SensorValue};
use crate::error::{Error, Result};
use serialport::SerialPort;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};

// ============================================================================
// Constants
// ============================================================================

/// Conversion factor from m/s or rad/s to GD32 velocity units.
///
/// Empirically calibrated: commanded 0.35 rad/s resulted in 0.669 rad/s actual,
/// so the correction factor is 1000 / 1.91 ≈ 523.
const VELOCITY_TO_DEVICE_UNITS: f32 = 523.0;

/// Default IMU calibration payload observed in R2D logs
const IMU_DEFAULT_PAYLOAD: [u8; 4] = [0x10, 0x0E, 0x00, 0x00];

// ============================================================================
// Component IDs
// ============================================================================
// Use these constants instead of string literals to catch typos at compile time.
// These must match the IDs used in the protobuf `ComponentControl.id` field.
// See also: proto/sangamio.proto ComponentControl message documentation.

/// Motion control - velocity mode or tank drive
const ID_DRIVE: &str = "drive";
/// Vacuum suction motor (0-100%)
const ID_VACUUM: &str = "vacuum";
/// Main brush roller (0-100%)
const ID_MAIN_BRUSH: &str = "main_brush";
/// Side brush spinner (0-100%)
const ID_SIDE_BRUSH: &str = "side_brush";
/// Mopping water pump (0-100%)
const ID_WATER_PUMP: &str = "water_pump";
/// Status LED patterns (0-18)
const ID_LED: &str = "led";
/// Lidar motor power and PWM
const ID_LIDAR: &str = "lidar";
/// IMU calibration queries and resets
const ID_IMU: &str = "imu";
/// Compass/magnetometer calibration
const ID_COMPASS: &str = "compass";
/// Cliff IR sensor enable/direction
const ID_CLIFF_IR: &str = "cliff_ir";
/// A33 main board power control (WARNING: affects daemon!)
const ID_MAIN_BOARD: &str = "main_board";
/// Charger power rail control
const ID_CHARGER: &str = "charger";
/// GD32 MCU sleep/wake/error reset
const ID_MCU: &str = "mcu";

// ============================================================================
// Helpers
// ============================================================================

/// Helper to send a TxPacket over the serial port
fn send_packet(port: &Arc<Mutex<Box<dyn SerialPort>>>, pkt: &TxPacket) -> Result<()> {
    let mut port_guard = port
        .lock()
        .map_err(|e| Error::MutexPoisoned(format!("serial port (send_packet): {}", e)))?;
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
            log::debug!("{} enable (100%)", name);
            state_field.store(100, Ordering::Relaxed);
            set_fn(pkt, 100);
            send_packet(port, pkt)
        }
        ComponentAction::Disable { .. } => {
            log::debug!("{} disable", name);
            state_field.store(0, Ordering::Relaxed);
            set_fn(pkt, 0);
            send_packet(port, pkt)
        }
        ComponentAction::Configure { config } => {
            // Handle both U8 and U32 speed values (protobuf sends U8 as U32)
            let speed = match config.get("speed") {
                Some(SensorValue::U8(s)) => Some(*s),
                Some(SensorValue::U32(s)) => Some(*s as u8),
                _ => None,
            };
            if let Some(speed) = speed {
                log::debug!("{} speed={}", name, speed);
                state_field.store(speed, Ordering::Relaxed);
                set_fn(pkt, speed);
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
    let mut port_guard = port
        .lock()
        .map_err(|e| Error::MutexPoisoned(format!("serial port (emergency_stop): {}", e)))?;

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
            log::debug!("Protocol sync (0x0C)");
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
        ComponentAction::Enable { config } => {
            // Enable motor with mode (default 0x02 nav mode)
            // Handle both U8 and U32 (protobuf sends U8 as U32)
            let mode = config
                .as_ref()
                .and_then(|c| c.get("mode"))
                .and_then(|v| match v {
                    SensorValue::U8(m) => Some(*m),
                    SensorValue::U32(m) => Some(*m as u8),
                    _ => None,
                })
                .unwrap_or(0x02);
            log::debug!("Drive enable (mode 0x{:02X})", mode);
            component_state
                .wheel_motor_enabled
                .store(true, Ordering::Relaxed);
            pkt.set_motor_mode(mode);
            send_packet(port, pkt)
        }
        ComponentAction::Disable { .. } => {
            // Stop: zero velocity and set mode 0x00
            log::debug!("Drive disable (stop + mode 0x00)");
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
            log::warn!("Drive emergency stop");
            emergency_stop(port, component_state, pkt)
        }
        ComponentAction::Configure { config } => {
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
                log::debug!(
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
                log::debug!(
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
        ComponentAction::Configure { config } => {
            // Handle both U8 and U32 (protobuf sends U8 as U32)
            let state = match config.get("state") {
                Some(SensorValue::U8(s)) => Some(*s),
                Some(SensorValue::U32(s)) => Some(*s as u8),
                _ => None,
            };
            if let Some(state) = state {
                log::debug!("LED state={}", state);
                pkt.set_led(state);
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
///
/// PWM is controlled exclusively by sangamio.toml configuration.
/// Upstream clients cannot change lidar speed - SangamIO determines
/// optimal speed based on hardware characteristics.
fn handle_lidar(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    component_state: &Arc<ComponentState>,
    pkt: &mut TxPacket,
    action: &ComponentAction,
) -> Result<()> {
    match action {
        ComponentAction::Enable { .. } => {
            // Use PWM from config (set during driver initialization)
            // Upstream clients cannot override this value
            let pwm = component_state.get_lidar_pwm();

            log::debug!("Lidar enable (PWM={}% from config)", pwm);

            // Power on first
            pkt.set_lidar_power(true);
            send_packet(port, pkt)?;

            // Enable lidar
            component_state.lidar_enabled.store(true, Ordering::Relaxed);

            // Send initial PWM command
            pkt.set_lidar_pwm(pwm);
            send_packet(port, pkt)?;

            Ok(())
        }
        ComponentAction::Disable { .. } => {
            log::debug!("Lidar disable");

            // Clear state first
            component_state
                .lidar_enabled
                .store(false, Ordering::Relaxed);

            // PWM to 0 first, then power off
            pkt.set_lidar_pwm(0);
            send_packet(port, pkt)?;
            pkt.set_lidar_power(false);
            send_packet(port, pkt)
        }
        ComponentAction::Configure { .. } => {
            // PWM is controlled by sangamio.toml, not by upstream clients
            log::warn!("Lidar Configure ignored - PWM is set in sangamio.toml");
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
            log::debug!(
                "IMU calibration state query (0xA2): payload={:02X?}, bytes={:02X?}",
                IMU_DEFAULT_PAYLOAD,
                pkt.as_bytes()
            );
            send_packet(port, pkt)
        }
        ComponentAction::Reset { .. } => {
            log::debug!("IMU factory reset (0xA1)");
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
            log::debug!("Compass calibration state query (0xA4)");
            pkt.set_compass_calibration_state();
            send_packet(port, pkt)
        }
        ComponentAction::Reset { .. } => {
            log::debug!("Compass calibration start (0xA3)");
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
            log::debug!("Cliff IR enable (0x78)");
            pkt.set_cliff_ir(true);
            send_packet(port, pkt)
        }
        ComponentAction::Disable { .. } => {
            log::debug!("Cliff IR disable (0x78)");
            pkt.set_cliff_ir(false);
            send_packet(port, pkt)
        }
        ComponentAction::Configure { config } => {
            // Handle both U8 and U32 (protobuf sends U8 as U32)
            let dir = match config.get("direction") {
                Some(SensorValue::U8(d)) => Some(*d),
                Some(SensorValue::U32(d)) => Some(*d as u8),
                _ => None,
            };
            if let Some(dir) = dir {
                log::debug!("Cliff IR direction (0x79): {}", dir);
                pkt.set_cliff_ir_direction(dir);
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
            log::debug!("Main board power on (0x99)");
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
            log::debug!("Charger power enable (0x9B)");
            pkt.set_charger_power(true);
            send_packet(port, pkt)
        }
        ComponentAction::Disable { .. } => {
            log::debug!("Charger power disable (0x9B)");
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
            log::debug!("MCU sleep (0x04)");
            pkt.set_mcu_sleep();
            send_packet(port, pkt)
        }
        ComponentAction::Enable { .. } => {
            log::debug!("MCU wakeup ack (0x05)");
            pkt.set_wakeup_ack();
            send_packet(port, pkt)
        }
        ComponentAction::Reset { .. } => {
            log::debug!("MCU reset error code (0x0A)");
            pkt.set_reset_error_code();
            send_packet(port, pkt)
        }
        _ => Err(Error::NotImplemented(format!(
            "MCU only supports Enable/Disable/Reset, got {:?}",
            action
        ))),
    }
}
