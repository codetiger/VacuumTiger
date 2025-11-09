//! Background heartbeat thread management
//!
//! The GD32 requires a continuous heartbeat (CMD=0x66) every 20ms.
//! Missing heartbeats will cause the device to enter error state and stop motors.
//!
//! This implementation matches AuxCtrl's architecture:
//! - 2ms receive loop (matches getGDData thread)
//! - CMD=0x66 sent every 10 cycles (10 × 2ms = 20ms)
//! - Lost packet tracking with threshold (150 lost packets)

use crate::transport::Transport;
use parking_lot::Mutex;
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use super::protocol::Gd32Command;
use super::state::Gd32State;

/// Receive interval - matches AuxCtrl's 2ms loop (getGDData thread)
const RECEIVE_INTERVAL: Duration = Duration::from_millis(2);

/// Heartbeat cycles: 10 × 2ms = 20ms (CMD=0x66 interval)
const HEARTBEAT_CYCLES: u64 = 10;

/// Lost packet threshold - matches AuxCtrl (150 lost packets = 300ms timeout)
const LOST_PACKET_THRESHOLD: u32 = 150;

/// Spawn background heartbeat thread
///
/// The thread will:
/// 1. Try to receive packets every 2ms (matches AuxCtrl's getGDData loop)
/// 2. Send CMD=0x66 heartbeat every 20ms (10 cycles) with motor speed commands
/// 3. Track lost packets and log errors if threshold exceeded
/// 4. Update shared state with encoder and battery data
/// 5. Run until shutdown flag is set
///
/// NOTE: CMD=0x06 (Wake) is NOT sent periodically (not found in AuxCtrl's main loop)
pub fn spawn_heartbeat_thread(
    transport: Arc<Mutex<Box<dyn Transport>>>,
    state: Arc<Mutex<Gd32State>>,
    shutdown: Arc<Mutex<bool>>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        log::info!("GD32: Heartbeat thread started (2ms receive loop)");

        let mut cycle_count = 0u64;

        loop {
            let cycle_start = Instant::now();

            // Check shutdown flag
            if *shutdown.lock() {
                log::info!("GD32: Heartbeat thread shutting down");
                break;
            }

            // ===== 1. RECEIVE PACKETS (every 2ms cycle) =====
            // Matches AuxCtrl's getGDData thread receive loop
            match read_and_process_responses(&transport, &state) {
                Ok(_) => {
                    // Reset lost packet counter on successful receive
                    let mut state = state.lock();
                    if state.lost_packet_count > 0 {
                        log::debug!(
                            "GD32: Communication restored (was lost for {}ms)",
                            state.lost_packet_count * 2
                        );
                    }
                    if state.sleep_mode {
                        log::info!("GD32: Device woke up from sleep mode");
                        state.sleep_mode = false;
                    }
                    state.lost_packet_count = 0;
                    state.last_rx_time = Instant::now();
                }
                Err(_) => {
                    // Increment lost packet counter
                    let mut state = state.lock();
                    state.lost_packet_count += 1;

                    // Check threshold (matches AuxCtrl: 150 lost = 300ms)
                    if state.lost_packet_count == LOST_PACKET_THRESHOLD {
                        log::warn!(
                            "GD32: Lost packet threshold exceeded ({} lost packets = {}ms timeout) - entering sleep mode",
                            LOST_PACKET_THRESHOLD,
                            LOST_PACKET_THRESHOLD * 2
                        );
                        state.sleep_mode = true;
                    } else if state.lost_packet_count % 50 == 0 {
                        log::warn!(
                            "GD32: {} consecutive lost packets ({}ms)",
                            state.lost_packet_count,
                            state.lost_packet_count * 2
                        );
                    }
                }
            }

            // ===== 2. SEND MOTOR CONTROL via CMD=0x66 (every 10 cycles = 20ms) =====
            // VERIFIED via MITM: CMD=0x66 payload = [left_speed (i32 LE), right_speed (i32 LE)]
            // Mode 0x02 ONLY supports CMD=0x66 for motor control (CMD=0x67 causes error 0xFF)
            // This matches AuxCtrl behavior which uses CMD=0x66 in mode 0x02
            if cycle_count % HEARTBEAT_CYCLES == 0 {
                let (target_left, target_right) = {
                    let state = state.lock();
                    (state.target_left, state.target_right)
                };

                // CMD=0x66: MotorVelocity with actual wheel speeds
                // Payload: [left_speed (i32 LE), right_speed (i32 LE)]
                let velocity_cmd =
                    Gd32Command::motor_velocity_with_speeds(target_left, target_right);
                let velocity_packet = velocity_cmd.encode();

                // Send CMD=0x66 with motor speeds
                {
                    let mut transport = transport.lock();

                    // Log TX periodically
                    if cycle_count == 0 || cycle_count % 500 == 0 {
                        log::debug!(
                            "GD32: TX CMD=0x66 (cycle #{}) L={}, R={}",
                            cycle_count / HEARTBEAT_CYCLES,
                            target_left,
                            target_right
                        );
                    }

                    // Send CMD=0x66 (MotorVelocity - motor control in mode 0x02)
                    if let Err(e) = transport.write(&velocity_packet) {
                        log::error!("GD32: MotorVelocity send error: {}", e);
                    } else {
                        let mut state = state.lock();
                        state.total_tx_packets += 1;
                    }
                }
            }

            // ===== 3. SEND HEARTBEAT COMMAND (every 100 cycles = 200ms) =====
            // Keeps GD32 from entering sleep mode
            // Pattern observed from AuxCtrl: CMD=0x06 (Heartbeat) sent periodically
            if cycle_count % (HEARTBEAT_CYCLES * 10) == 0 && cycle_count > 0 {
                let heartbeat_cmd = Gd32Command::Heartbeat;
                let packet = heartbeat_cmd.encode();

                let mut transport = transport.lock();
                if let Err(e) = transport.write(&packet) {
                    log::error!("GD32: Heartbeat command send error: {}", e);
                } else {
                    log::debug!("GD32: Sent CMD=0x06 (Heartbeat) to prevent sleep mode");
                }
            }

            // ===== 4. WAKE SEQUENCE FROM SLEEP (every 250 cycles = 500ms) =====
            // When GD32 enters sleep mode (no response for 300ms), send Initialize sequence
            // This matches AuxCtrl's behavior: sleep recovery uses CMD=0x08
            if cycle_count % (HEARTBEAT_CYCLES * 25) == 0 && cycle_count > 0 {
                let is_sleeping = state.lock().sleep_mode;
                if is_sleeping {
                    let init_cmd = Gd32Command::default_initialize();
                    let packet = init_cmd.encode();

                    let mut transport = transport.lock();
                    if let Err(e) = transport.write(&packet) {
                        log::error!("GD32: Sleep recovery CMD=0x08 send error: {}", e);
                    } else {
                        log::warn!(
                            "GD32: Sending CMD=0x08 (Initialize) to recover from sleep mode"
                        );
                    }
                }
            }

            cycle_count += 1;

            // ===== 5. MAINTAIN 2ms TIMING =====
            let elapsed = cycle_start.elapsed();
            if elapsed < RECEIVE_INTERVAL {
                thread::sleep(RECEIVE_INTERVAL - elapsed);
            } else if elapsed.as_millis() > 3 {
                log::warn!("GD32: Cycle overrun: {:?} (target: 2ms)", elapsed);
            }
        }

        log::info!("GD32: Heartbeat thread stopped");
    })
}

/// Read and process any available response packets
fn read_and_process_responses(
    transport: &Arc<Mutex<Box<dyn Transport>>>,
    state: &Arc<Mutex<Gd32State>>,
) -> crate::error::Result<()> {
    use super::protocol::Gd32Response;

    let mut transport = transport.lock();

    // Check if data is available
    let available = transport.available()?;
    if available < 6 {
        // No data available - this counts as a lost packet for sleep detection
        return Err(crate::Error::Io(std::io::Error::new(
            std::io::ErrorKind::WouldBlock,
            "No data available",
        )));
    }

    // Read only the available bytes (don't wait for more)
    // This prevents read() from blocking up to timeout duration
    let buffer_size = available.min(256);
    let mut buffer = vec![0u8; buffer_size];
    let bytes_read = transport.read(&mut buffer)?;

    if bytes_read == 0 {
        // Read returned 0 bytes - count as lost packet
        return Err(crate::Error::Io(std::io::Error::new(
            std::io::ErrorKind::WouldBlock,
            "Read returned 0 bytes",
        )));
    }

    // Log raw bytes received (reduced frequency to avoid slowdown)
    // Only log occasionally to reduce overhead in hot path

    // Try to decode response
    match Gd32Response::decode(&buffer[..bytes_read]) {
        Ok(response) => {
            // Reduced logging frequency for performance
            if log::log_enabled!(log::Level::Trace) {
                log::trace!(
                    "GD32: RX {} bytes, decoded CMD=0x{:02X}, payload_len={}",
                    bytes_read,
                    response.cmd_id,
                    response.payload.len()
                );
            }

            if response.is_status_packet() {
                // Update shared state
                let mut state = state.lock();
                state.battery_voltage = response.battery_voltage;
                state.battery_current = response.battery_current;
                state.battery_level = response.battery_level;
                state.encoder_left = response.encoder_left;
                state.encoder_right = response.encoder_right;
                state.error_code = response.error_code;
                state.status_flag = response.status_flag;
                state.charging_flag = response.charging_flag;
                state.battery_state_flag = response.battery_state_flag;
                state.percent_value = response.percent_value;
                state.ir_sensor_1 = response.ir_sensor_1;
                state.point_button_ir = response.point_button_ir;
                state.dock_button_ir = response.dock_button_ir;
                state.total_rx_packets += 1;

                // Reduced logging for performance (only trace level)
                if log::log_enabled!(log::Level::Trace) {
                    log::trace!(
                        "GD32: Status - Bat: {:.2}V ({}%), Enc: L={}, R={}, Err={}, IR: [{}|{}|{}]",
                        response.battery_voltage,
                        response.battery_level,
                        response.encoder_left,
                        response.encoder_right,
                        response.error_code,
                        response.ir_sensor_1,
                        response.point_button_ir,
                        response.dock_button_ir
                    );
                }
            } else if log::log_enabled!(log::Level::Trace) {
                log::trace!("GD32: Non-status response: CMD=0x{:02X}", response.cmd_id);
            }
        }
        Err(e) => {
            // Only log parse errors at debug level to reduce spam
            if log::log_enabled!(log::Level::Debug) {
                log::debug!("GD32: Parse error: {}", e);
            }
        }
    }

    Ok(())
}
