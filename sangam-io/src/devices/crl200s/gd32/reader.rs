//! Reader thread for GD32 driver
//!
//! This module contains the reader loop that parses incoming packets from the GD32
//! and updates sensor data in real-time.
//!
//! # Thread Model
//!
//! The reader runs on a dedicated OS thread, continuously polling the serial port.
//! It shares the port mutex with the heartbeat thread but minimizes lock hold time
//! (~100μs per packet parse).

use super::packet::version_request_packet;
use super::protocol::{PacketReader, RxPacket};
use crate::core::types::{SensorGroupData, SensorValue, StreamSender};
use crate::devices::crl200s::constants::{
    BATTERY_VOLTAGE_MAX, BATTERY_VOLTAGE_MIN, CMD_PROTOCOL_SYNC, CMD_STATUS, CMD_VERSION,
    FLAG_BUMPER_LEFT, FLAG_BUMPER_RIGHT, FLAG_CHARGING, FLAG_CLIFF_LEFT_FRONT,
    FLAG_CLIFF_LEFT_SIDE, FLAG_CLIFF_RIGHT_FRONT, FLAG_CLIFF_RIGHT_SIDE, FLAG_DOCK_CONNECTED,
    FLAG_DUSTBOX_ATTACHED, OFFSET_ACCEL_X, OFFSET_ACCEL_Y, OFFSET_ACCEL_Z,
    OFFSET_BATTERY_VOLTAGE_RAW, OFFSET_BUMPER_FLAGS, OFFSET_CHARGING_FLAGS, OFFSET_CLIFF_FLAGS,
    OFFSET_DOCK_BUTTON, OFFSET_DUSTBOX_FLAGS, OFFSET_GYRO_X, OFFSET_GYRO_Y, OFFSET_GYRO_Z,
    OFFSET_START_BUTTON, OFFSET_TILT_X, OFFSET_TILT_Y, OFFSET_TILT_Z, OFFSET_WHEEL_LEFT_ENCODER,
    OFFSET_WHEEL_RIGHT_ENCODER, STATUS_PAYLOAD_MIN_SIZE,
};
use serialport::SerialPort;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

/// Reader loop - reads packets and updates shared data directly
///
/// This loop runs continuously to parse incoming packets from the GD32.
///
/// # Packet Handling
///
/// - **Status packets (CMD=0x15)**: Arrive at ~500Hz, contain all sensor data (bumpers, encoders, etc.)
/// - **Version response (CMD=0x42)**: Sent once after we request it following first packet
///
/// # Version Request Flow
///
/// 1. Wait for first packet of any type (indicates GD32 is awake and initialized)
/// 2. Send version request (CMD=0x42) - sent exactly once, not retried on failure
/// 3. Parse version response payload (string + i32 code)
/// 4. Set `version_received` flag to prevent repeated requests
///
/// **Why wait for first packet?** The GD32 requires its ~5 second wake/init sequence
/// before it will respond to commands. Sending version request too early results in
/// the command being silently dropped. By waiting for the first status packet (0x15),
/// we know the GD32 is ready.
///
/// **Failure handling:** If the version request fails to send (e.g., serial error),
/// it is NOT retried. The daemon will continue without version info. This is
/// acceptable because version info is diagnostic only.
///
/// # Data Updates
///
/// Sensor data is updated in-place with no allocations. The mutex is held only for
/// the duration of parsing and updating fields (~100μs typical).
pub(super) fn reader_loop(
    port: Arc<Mutex<Box<dyn SerialPort>>>,
    shutdown: Arc<AtomicBool>,
    sensor_data: Arc<Mutex<SensorGroupData>>,
    version_data: Option<Arc<Mutex<SensorGroupData>>>,
    stream_tx: Option<StreamSender>,
) {
    let mut reader = PacketReader::new();
    let mut version_requested = false;
    let mut version_received = false;
    let mut protocol_sync_received = false;

    while !shutdown.load(Ordering::Relaxed) {
        let packet_result = {
            let Ok(mut port) = port.lock() else {
                log::error!("Reader: mutex poisoned, exiting");
                break;
            };
            reader.read_packet(&mut *port)
        };

        match packet_result {
            Ok(Some(packet)) => {
                log::debug!(
                    "Packet received: CMD=0x{:02X}, payload_len={}",
                    packet.cmd,
                    packet.payload_len()
                );

                // Request version after first packet (like sangam-io2-backup)
                if !version_requested && version_data.is_some() {
                    log::info!(
                        "First packet received (CMD=0x{:02X}), requesting version",
                        packet.cmd
                    );
                    version_requested = true;

                    let version_pkt = version_request_packet();
                    if let Ok(mut port) = port.lock() {
                        if let Err(e) = version_pkt.send_to(&mut *port) {
                            log::error!("Failed to send version request: {}", e);
                        }
                    } else {
                        log::error!("Failed to lock port for version request");
                    }
                }

                // Handle version response
                if packet.cmd == CMD_VERSION && !version_received {
                    handle_version_packet(packet, &version_data, &mut version_received);
                }

                // Handle protocol sync ACK (0x0C echoed back from GD32)
                if packet.cmd == CMD_PROTOCOL_SYNC && !protocol_sync_received {
                    protocol_sync_received = true;
                    log::info!(
                        "Protocol sync ACK received (payload: {:02X?})",
                        packet.payload()
                    );
                }

                // Handle sensor status data
                if packet.cmd == CMD_STATUS && packet.payload_len() >= STATUS_PAYLOAD_MIN_SIZE {
                    if let Some(cloned) = handle_status_packet(packet, &sensor_data) {
                        // Push to streaming channel if available (for 500Hz TCP streaming)
                        if let Some(ref tx) = stream_tx {
                            // Use try_send to avoid blocking - drop message if channel full
                            if tx.try_send(cloned).is_err() {
                                log::trace!("Stream channel full, dropping message");
                            }
                        }
                    }
                }
                // Packet received - immediately try to read next one (no sleep)
            }
            Ok(None) => {
                // No packet available - serial port read already blocked for timeout
                // No additional sleep needed
            }
            Err(e) => {
                log::error!("Packet read error: {}", e);
                thread::sleep(Duration::from_millis(10));
            }
        }
        // Note: No sleep here! The serial port read is blocking with timeout.
        // At 500Hz, we need to process every packet immediately.
    }

    log::info!("Reader thread exiting");
}

/// Handle version response packet
fn handle_version_packet(
    packet: &RxPacket,
    version_data: &Option<Arc<Mutex<SensorGroupData>>>,
    version_received: &mut bool,
) {
    if let Some(ref vdata) = version_data {
        let payload = packet.payload();
        if !payload.is_empty() {
            let Ok(mut data) = vdata.lock() else {
                log::error!("Failed to lock version data");
                return;
            };
            data.touch();

            // Parse version string
            let version_string = if payload[0] < 128 {
                let len = payload[0] as usize;
                if payload.len() > len {
                    String::from_utf8_lossy(&payload[1..=len]).to_string()
                } else {
                    String::from_utf8_lossy(payload).to_string()
                }
            } else {
                let null_pos = payload
                    .iter()
                    .position(|&b| b == 0)
                    .unwrap_or(payload.len());
                String::from_utf8_lossy(&payload[..null_pos]).to_string()
            };

            // Parse version code
            let version_code = if payload.len() >= 8 {
                i32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]])
            } else {
                0
            };

            data.set(
                "version_string",
                SensorValue::String(version_string.clone()),
            );
            data.set("version_code", SensorValue::I32(version_code));

            log::info!("GD32 version: {} ({})", version_string, version_code);
            *version_received = true;
        }
    }
}

/// Handle status packet and update sensor data.
/// Returns a clone of the updated data for streaming (if lock succeeds).
fn handle_status_packet(
    packet: &RxPacket,
    sensor_data: &Arc<Mutex<SensorGroupData>>,
) -> Option<SensorGroupData> {
    // Update shared data directly (no allocations)
    let Ok(mut data) = sensor_data.lock() else {
        log::error!("Failed to lock sensor data");
        return None;
    };
    let payload = packet.payload();

    // Update timestamp
    data.touch();

    // Charging/battery status
    data.set(
        "is_charging",
        SensorValue::Bool((payload[OFFSET_CHARGING_FLAGS] & FLAG_CHARGING) != 0),
    );
    data.set(
        "is_dock_connected",
        SensorValue::Bool((payload[OFFSET_CHARGING_FLAGS] & FLAG_DOCK_CONNECTED) != 0),
    );

    // Battery voltage and level
    // Raw byte at offset 0x08 is voltage * 10 (e.g., 155 = 15.5V)
    let voltage_raw = payload[OFFSET_BATTERY_VOLTAGE_RAW];
    let voltage = voltage_raw as f32 / 10.0;

    // Calculate percentage using linear interpolation between min/max voltage
    let battery_level =
        ((voltage - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * 100.0)
            .clamp(0.0, 100.0) as u8;

    data.set("battery_voltage", SensorValue::F32(voltage));
    data.set("battery_level", SensorValue::U8(battery_level));

    // Buttons
    data.set(
        "start_button",
        SensorValue::U16(u16::from_le_bytes([
            payload[OFFSET_START_BUTTON],
            payload[OFFSET_START_BUTTON + 1],
        ])),
    );
    data.set(
        "dock_button",
        SensorValue::U16(u16::from_le_bytes([
            payload[OFFSET_DOCK_BUTTON],
            payload[OFFSET_DOCK_BUTTON + 1],
        ])),
    );

    // Bumpers
    data.set(
        "bumper_left",
        SensorValue::Bool((payload[OFFSET_BUMPER_FLAGS] & FLAG_BUMPER_LEFT) != 0),
    );
    data.set(
        "bumper_right",
        SensorValue::Bool((payload[OFFSET_BUMPER_FLAGS] & FLAG_BUMPER_RIGHT) != 0),
    );

    // Wheel encoders
    data.set(
        "wheel_left",
        SensorValue::U16(u16::from_le_bytes([
            payload[OFFSET_WHEEL_LEFT_ENCODER],
            payload[OFFSET_WHEEL_LEFT_ENCODER + 1],
        ])),
    );
    data.set(
        "wheel_right",
        SensorValue::U16(u16::from_le_bytes([
            payload[OFFSET_WHEEL_RIGHT_ENCODER],
            payload[OFFSET_WHEEL_RIGHT_ENCODER + 1],
        ])),
    );

    // Cliff sensors
    data.set(
        "cliff_left_side",
        SensorValue::Bool((payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_LEFT_SIDE) != 0),
    );
    data.set(
        "cliff_left_front",
        SensorValue::Bool((payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_LEFT_FRONT) != 0),
    );
    data.set(
        "cliff_right_front",
        SensorValue::Bool((payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_RIGHT_FRONT) != 0),
    );
    data.set(
        "cliff_right_side",
        SensorValue::Bool((payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_RIGHT_SIDE) != 0),
    );

    // Dustbox
    data.set(
        "dustbox_attached",
        SensorValue::Bool((payload[OFFSET_DUSTBOX_FLAGS] & FLAG_DUSTBOX_ATTACHED) != 0),
    );

    // IMU: Gyroscope raw values (i16 LE)
    data.set(
        "gyro_x",
        SensorValue::I16(i16::from_le_bytes([
            payload[OFFSET_GYRO_X],
            payload[OFFSET_GYRO_X + 1],
        ])),
    );
    data.set(
        "gyro_y",
        SensorValue::I16(i16::from_le_bytes([
            payload[OFFSET_GYRO_Y],
            payload[OFFSET_GYRO_Y + 1],
        ])),
    );
    data.set(
        "gyro_z",
        SensorValue::I16(i16::from_le_bytes([
            payload[OFFSET_GYRO_Z],
            payload[OFFSET_GYRO_Z + 1],
        ])),
    );

    // IMU: Accelerometer raw values (i16 LE)
    data.set(
        "accel_x",
        SensorValue::I16(i16::from_le_bytes([
            payload[OFFSET_ACCEL_X],
            payload[OFFSET_ACCEL_X + 1],
        ])),
    );
    data.set(
        "accel_y",
        SensorValue::I16(i16::from_le_bytes([
            payload[OFFSET_ACCEL_Y],
            payload[OFFSET_ACCEL_Y + 1],
        ])),
    );
    data.set(
        "accel_z",
        SensorValue::I16(i16::from_le_bytes([
            payload[OFFSET_ACCEL_Z],
            payload[OFFSET_ACCEL_Z + 1],
        ])),
    );

    // IMU: Low-pass filtered tilt vector (gravity direction, i16 LE)
    data.set(
        "tilt_x",
        SensorValue::I16(i16::from_le_bytes([
            payload[OFFSET_TILT_X],
            payload[OFFSET_TILT_X + 1],
        ])),
    );
    data.set(
        "tilt_y",
        SensorValue::I16(i16::from_le_bytes([
            payload[OFFSET_TILT_Y],
            payload[OFFSET_TILT_Y + 1],
        ])),
    );
    data.set(
        "tilt_z",
        SensorValue::I16(i16::from_le_bytes([
            payload[OFFSET_TILT_Z],
            payload[OFFSET_TILT_Z + 1],
        ])),
    );

    // Clone data for streaming before releasing lock
    Some(data.clone())
}
