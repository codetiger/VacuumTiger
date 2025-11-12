//! Background thread management for GD32 communication
//!
//! The GD32 requires continuous heartbeat (CMD=0x66) every 20ms.
//! Missing heartbeats will cause the device to enter error state and stop motors.
//!
//! This implementation uses a dual-thread architecture matching AuxCtrl:
//! - READ thread: 2ms loop for receiving STATUS_DATA packets (never blocks on writes)
//! - WRITE thread: 20ms loop for sending commands (can block without affecting reads)
//!
//! This separation ensures consistent 2ms read timing regardless of write latency.

use crate::transport::Transport;
use parking_lot::Mutex;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc::Receiver;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use super::protocol::Gd32Command;
use super::state::Gd32State;

/// Receive interval - matches AuxCtrl's 2ms loop (getGDData thread)
const RECEIVE_INTERVAL: Duration = Duration::from_millis(2);

/// Write interval - CMD=0x66 heartbeat every 20ms
const WRITE_INTERVAL: Duration = Duration::from_millis(20);

/// Lost packet threshold - matches AuxCtrl (150 lost packets = 300ms timeout)
const LOST_PACKET_THRESHOLD: u32 = 150;

/// Spawn READ thread (fast 2ms loop, never blocks on writes)
///
/// This thread:
/// 1. Checks for available data every 2ms
/// 2. Reads and processes STATUS_DATA packets
/// 3. Updates shared state with sensor/encoder/battery data
/// 4. Tracks lost packets for sleep detection
/// 5. Never performs any write operations (ensures consistent timing)
pub fn spawn_read_thread(
    transport: Arc<Mutex<Box<dyn Transport>>>,
    state: Arc<Mutex<Gd32State>>,
    shutdown: Arc<Mutex<bool>>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        log::info!("GD32: Read thread started (2ms loop, dedicated to receiving STATUS_DATA)");

        // Persistent read buffer to accumulate data across cycles
        // STATUS_DATA packets are 103 bytes, allow 2 full packets + overhead
        let mut read_buffer: Vec<u8> = Vec::with_capacity(256);

        loop {
            let cycle_start = Instant::now();

            // Check shutdown flag
            if *shutdown.lock() {
                log::info!("GD32: Read thread shutting down");
                break;
            }

            // ONLY READ - never write (ensures consistent 2ms timing)
            match read_and_process_responses(&transport, &state, &mut read_buffer) {
                Ok(_) => {
                    // Reset lost packet counter on successful receive
                    let state_lock = state.lock();
                    let lost_count = state_lock
                        .diagnostics
                        .lost_packet_count
                        .load(Ordering::Relaxed);
                    if lost_count > 0 {
                        log::debug!(
                            "GD32: Communication restored (was lost for {}ms)",
                            lost_count * 2
                        );
                    }
                    if state_lock.diagnostics.sleep_mode.load(Ordering::Relaxed) {
                        log::info!("GD32: Device woke up from sleep mode");
                        state_lock
                            .diagnostics
                            .sleep_mode
                            .store(false, Ordering::Relaxed);

                        // Update connection state
                        *state_lock.connection_state.lock() =
                            crate::devices::gd32::state::ConnectionState::Connected;
                    }
                    state_lock
                        .diagnostics
                        .lost_packet_count
                        .store(0, Ordering::Relaxed);
                    *state_lock.diagnostics.last_rx_time.lock() = Instant::now();
                }
                Err(_) => {
                    // Increment lost packet counter
                    let state_lock = state.lock();
                    let new_count = state_lock
                        .diagnostics
                        .lost_packet_count
                        .fetch_add(1, Ordering::Relaxed)
                        + 1;

                    // Check threshold (matches AuxCtrl: 150 lost = 300ms)
                    if new_count == LOST_PACKET_THRESHOLD {
                        log::warn!(
                            "GD32: Lost packet threshold exceeded ({} lost packets = {}ms timeout) - entering sleep mode",
                            LOST_PACKET_THRESHOLD,
                            LOST_PACKET_THRESHOLD * 2
                        );
                        state_lock
                            .diagnostics
                            .sleep_mode
                            .store(true, Ordering::Relaxed);

                        // Update connection state
                        *state_lock.connection_state.lock() =
                            crate::devices::gd32::state::ConnectionState::SleepMode;
                    } else if new_count % 50 == 0 {
                        log::warn!(
                            "GD32: {} consecutive lost packets ({}ms)",
                            new_count,
                            new_count * 2
                        );
                    }
                }
            }

            // Maintain 2ms timing
            let elapsed = cycle_start.elapsed();
            if elapsed < RECEIVE_INTERVAL {
                thread::sleep(RECEIVE_INTERVAL - elapsed);
            } else if elapsed.as_millis() > 3 {
                log::warn!("GD32: Read cycle overrun: {:?} (target: 2ms)", elapsed);
            }
        }

        log::info!("GD32: Read thread stopped");
    })
}

/// Spawn WRITE thread (20ms loop, handles all command transmission)
///
/// This thread:
/// 1. Sends CMD=0x66 (motor control) every 20ms
/// 2. Sends CMD=0x06 (heartbeat) every 200ms
/// 3. Sends CMD=0x08 (wake) when in sleep mode
/// 4. Processes ad-hoc commands from the command queue
/// 5. Can block on writes without affecting read timing
pub fn spawn_write_thread(
    transport: Arc<Mutex<Box<dyn Transport>>>,
    state: Arc<Mutex<Gd32State>>,
    cmd_rx: Receiver<Gd32Command>,
    shutdown: Arc<Mutex<bool>>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        log::info!("GD32: Write thread started (20ms loop, handles all command transmission)");

        let mut cycle_count = 0u64;

        // Track last sent motor command to avoid redundant sends
        let mut last_motor_cmd: Option<(i32, i32, u8)> = None; // (left, right, mode)
        let mut last_motor_send_time = Instant::now();
        const MOTOR_SEND_TIMEOUT: Duration = Duration::from_millis(100); // Safety: send every 100ms minimum

        loop {
            let cycle_start = Instant::now();

            // Check shutdown flag
            if *shutdown.lock() {
                log::info!("GD32: Write thread shutting down");
                break;
            }

            // ===== 1. PROCESS QUEUED COMMANDS (ad-hoc from main thread) =====
            // Drain all pending commands from the queue
            while let Ok(cmd) = cmd_rx.try_recv() {
                send_command(&transport, &state, &cmd);
            }

            // ===== 2. SEND MOTOR CONTROL (only when changed or timeout) =====
            // Choose command based on motor mode:
            // - Mode 0x01: CMD=0x67 (MotorSpeed) - direct wheel control
            // - Mode 0x02: CMD=0x66 (MotorVelocity) - differential drive
            let (target_left, target_right, motor_mode) = {
                let state = state.lock();
                let cmd_state = state.command.lock();
                (
                    cmd_state.target_left,
                    cmd_state.target_right,
                    cmd_state.motor_mode,
                )
            };

            // Check if motor command changed or timeout expired
            let current_cmd = (target_left, target_right, motor_mode);
            let cmd_changed = last_motor_cmd != Some(current_cmd);
            let timeout_expired = last_motor_send_time.elapsed() >= MOTOR_SEND_TIMEOUT;
            let is_moving = target_left != 0 || target_right != 0;

            // Send motor command if:
            // 1. Command changed (including 0→non-zero or non-zero→0)
            // 2. Timeout expired AND motors are moving (safety refresh)
            // 3. Never send repeated zero commands (motors already stopped)
            let should_send = cmd_changed || (timeout_expired && is_moving);

            if should_send {
                let motor_cmd = if motor_mode == 0x01 {
                    // Mode 0x01: Direct motor control (CMD=0x67)
                    Gd32Command::MotorSpeed {
                        left: target_left,
                        right: target_right,
                    }
                } else {
                    // Mode 0x02: Velocity control (CMD=0x66)
                    Gd32Command::motor_velocity_with_speeds(target_left, target_right)
                };

                // Log reason for send
                if cmd_changed {
                    if is_moving {
                        log::debug!(
                            "GD32: Motor cmd changed - TX CMD=0x{:02X} mode=0x{:02X} L={}, R={}",
                            motor_cmd.cmd_id(),
                            motor_mode,
                            target_left,
                            target_right
                        );
                    } else {
                        log::debug!(
                            "GD32: Motors stopped - TX CMD=0x{:02X} mode=0x{:02X} L=0, R=0 (final stop command)",
                            motor_cmd.cmd_id(),
                            motor_mode
                        );
                    }
                } else if timeout_expired && cycle_count % 50 == 0 {
                    // Log periodic safety sends for moving motors (less frequently)
                    log::debug!(
                        "GD32: Safety refresh (moving) - TX CMD=0x{:02X} mode=0x{:02X} L={}, R={}",
                        motor_cmd.cmd_id(),
                        motor_mode,
                        target_left,
                        target_right
                    );
                }

                send_command(&transport, &state, &motor_cmd);
                last_motor_cmd = Some(current_cmd);
                last_motor_send_time = Instant::now();
            }

            // ===== 3. SEND HEARTBEAT COMMAND (every 10 cycles = 200ms) =====
            // Keeps GD32 from entering sleep mode
            if cycle_count % 10 == 0 && cycle_count > 0 {
                let heartbeat_cmd = Gd32Command::Heartbeat;
                send_command(&transport, &state, &heartbeat_cmd);
                log::debug!("GD32: Sent CMD=0x06 (Heartbeat) to prevent sleep mode");
            }

            // ===== 4. WAKE SEQUENCE FROM SLEEP (every 25 cycles = 500ms) =====
            // When GD32 enters sleep mode (no response for 300ms), send Initialize sequence
            if cycle_count % 25 == 0 && cycle_count > 0 {
                let is_sleeping = state.lock().diagnostics.sleep_mode.load(Ordering::Relaxed);
                if is_sleeping {
                    let init_cmd = Gd32Command::default_initialize();
                    send_command(&transport, &state, &init_cmd);
                    log::warn!("GD32: Sending CMD=0x08 (Initialize) to recover from sleep mode");
                }
            }

            cycle_count += 1;

            // ===== 5. MAINTAIN 20ms TIMING =====
            let elapsed = cycle_start.elapsed();
            if elapsed < WRITE_INTERVAL {
                thread::sleep(WRITE_INTERVAL - elapsed);
            } else if elapsed.as_millis() > 25 {
                log::warn!("GD32: Write cycle overrun: {:?} (target: 20ms)", elapsed);
            }
        }

        log::info!("GD32: Write thread stopped");
    })
}

/// Send a command to GD32 (helper function for write thread)
fn send_command(
    transport: &Arc<Mutex<Box<dyn Transport>>>,
    state: &Arc<Mutex<Gd32State>>,
    cmd: &Gd32Command,
) {
    let packet = cmd.encode();
    let mut transport = transport.lock();

    if let Err(e) = transport.write(&packet) {
        log::error!("GD32: Write error for CMD=0x{:02X}: {}", cmd.cmd_id(), e);
    } else {
        state
            .lock()
            .diagnostics
            .total_tx_packets
            .fetch_add(1, Ordering::Relaxed);
    }
    // Note: No flush() - let kernel handle transmission asynchronously
}

/// Read and process any available response packets
fn read_and_process_responses(
    transport: &Arc<Mutex<Box<dyn Transport>>>,
    state: &Arc<Mutex<Gd32State>>,
    read_buffer: &mut Vec<u8>,
) -> crate::error::Result<()> {
    use super::protocol::Gd32Response;

    let mut transport = transport.lock();

    // Check if data is available
    let available = transport.available()?;
    if available == 0 {
        // No NEW data available
        // But we might have leftover data in buffer from previous cycle
        if read_buffer.is_empty() {
            // No data available - this counts as a lost packet for sleep detection
            return Err(crate::Error::Io(std::io::Error::new(
                std::io::ErrorKind::WouldBlock,
                "No data available",
            )));
        }
        // Fall through to try decoding buffered data
    } else {
        // Read available bytes and append to persistent buffer
        let to_read = available.min(256);
        let mut temp_buffer = vec![0u8; to_read];
        let bytes_read = transport.read(&mut temp_buffer)?;

        if bytes_read == 0 {
            // Read returned 0 bytes - count as lost packet
            return Err(crate::Error::Io(std::io::Error::new(
                std::io::ErrorKind::WouldBlock,
                "Read returned 0 bytes",
            )));
        }

        // Append new data to persistent buffer
        read_buffer.extend_from_slice(&temp_buffer[..bytes_read]);
    }

    // Try to decode response from accumulated buffer
    match Gd32Response::decode_with_sync(read_buffer) {
        Ok((bytes_consumed, response)) => {
            // Remove consumed bytes from buffer
            read_buffer.drain(0..bytes_consumed);

            // Prevent buffer from growing unbounded
            if read_buffer.len() > 512 {
                log::warn!("GD32: Read buffer exceeded 512 bytes, clearing garbage data");
                read_buffer.clear();
            }

            // Reduced logging frequency for performance
            if log::log_enabled!(log::Level::Trace) {
                log::trace!(
                    "GD32: Decoded CMD=0x{:02X}, consumed {} bytes, {} bytes remain in buffer",
                    response.cmd_id,
                    bytes_consumed,
                    read_buffer.len()
                );
            }

            if response.is_status_packet() {
                // Log complete STATUS_DATA packet at debug level
                if log::log_enabled!(log::Level::Debug) {
                    let packet_num = state
                        .lock()
                        .diagnostics
                        .total_rx_packets
                        .load(Ordering::Relaxed);

                    // Format: packet_num | enc_L | enc_R | error | complete hex payload
                    let hex_dump: Vec<String> = response
                        .payload
                        .iter()
                        .map(|b| format!("{:02X}", b))
                        .collect();

                    log::debug!(
                        "STATUS_DATA | pkt={:06} | enc_L={:08} enc_R={:08} err=0x{:02X} | RAW: {}",
                        packet_num,
                        response.encoder_left,
                        response.encoder_right,
                        response.error_code,
                        hex_dump.join(" ")
                    );
                }

                // Update shared state
                // Convert parsed response to TelemetryState
                let telemetry = crate::devices::gd32::state::TelemetryState {
                    battery_voltage: Some(response.battery_voltage),
                    battery_current: Some(response.battery_current),
                    battery_level: Some(response.battery_level),
                    encoder_left: Some(response.encoder_left),
                    encoder_right: Some(response.encoder_right),
                    error_code: Some(response.error_code),
                    status_flag: Some(response.status_flag),
                    charging_flag: Some(response.charging_flag),
                    battery_state_flag: Some(response.battery_state_flag),
                    percent_value: Some(response.percent_value),
                    ir_sensor_1: Some(response.ir_sensor_1),
                    start_button_ir: Some(response.start_button_ir),
                    dock_button_ir: Some(response.dock_button_ir),
                    bumper_triggered: Some(response.bumper_triggered),
                };

                // Update state using the helper method
                state.lock().update_telemetry(telemetry);

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
                        response.start_button_ir,
                        response.dock_button_ir
                    );
                }
            } else if log::log_enabled!(log::Level::Trace) {
                log::trace!("GD32: Non-status response: CMD=0x{:02X}", response.cmd_id);
            }
        }
        Err(e) => {
            // Incomplete packet errors are expected when accumulating data
            // Only log if we have significant data buffered but still can't decode
            let error_msg = format!("{}", e);
            if error_msg.contains("Incomplete packet") {
                // Expected - waiting for more data
                if read_buffer.len() > 150 {
                    log::debug!(
                        "GD32: Incomplete packet with {} bytes buffered",
                        read_buffer.len()
                    );
                }
            } else if error_msg.contains("No sync bytes found") {
                // Garbage data in buffer - consume it
                if !read_buffer.is_empty() {
                    log::debug!(
                        "GD32: No sync bytes in {} byte buffer, clearing",
                        read_buffer.len()
                    );
                    read_buffer.clear();
                }
            } else {
                // Other parse errors - log at trace level
                if log::log_enabled!(log::Level::Trace) {
                    log::trace!("GD32: Parse error: {}", e);
                }
            }

            // If buffer has incomplete data, don't count as a lost packet
            // Only count as lost if we had no buffered data and no new data
            if read_buffer.is_empty() {
                return Err(e);
            }
        }
    }

    Ok(())
}
