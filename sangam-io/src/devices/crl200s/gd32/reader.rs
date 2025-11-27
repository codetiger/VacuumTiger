//! Reader thread for GD32 driver
//!
//! This module contains the reader loop that parses incoming packets from the GD32
//! and updates sensor data in real-time.

use super::protocol::{cmd_version_request, PacketReader};
use crate::core::types::{SensorGroupData, SensorValue};
use crate::devices::crl200s::constants::{
    CMD_STATUS, CMD_VERSION, FLAG_BUMPER_LEFT, FLAG_BUMPER_RIGHT, FLAG_CHARGING,
    FLAG_CLIFF_LEFT_FRONT, FLAG_CLIFF_LEFT_SIDE, FLAG_CLIFF_RIGHT_FRONT, FLAG_CLIFF_RIGHT_SIDE,
    FLAG_DOCK_CONNECTED, FLAG_DUSTBOX_ATTACHED, OFFSET_BUMPER_FLAGS, OFFSET_CHARGING_FLAGS,
    OFFSET_CLIFF_FLAGS, OFFSET_DOCK_BUTTON, OFFSET_DUSTBOX_FLAGS, OFFSET_START_BUTTON,
    OFFSET_WHEEL_LEFT_ENCODER, OFFSET_WHEEL_RIGHT_ENCODER, STATUS_PAYLOAD_MIN_SIZE,
};
use serialport::SerialPort;
use std::io::Write;
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
/// 1. Wait for first packet of any type (indicates GD32 is awake)
/// 2. Send version request (CMD=0x42)
/// 3. Parse version response payload (string + i32 code)
/// 4. Set `version_received` flag to prevent repeated requests
///
/// This approach (from sangam-io2-backup) ensures we don't request version before
/// the GD32 is ready, which would result in lost commands.
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
) {
    let mut reader = PacketReader::new();
    let mut version_requested = false;
    let mut version_received = false;

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
                    packet.payload.len()
                );

                // Request version after first packet (like sangam-io2-backup)
                if !version_requested && version_data.is_some() {
                    log::info!(
                        "First packet received (CMD=0x{:02X}), requesting version",
                        packet.cmd
                    );
                    version_requested = true;

                    let version_pkt = cmd_version_request();
                    if let Ok(mut port) = port.lock() {
                        if let Err(e) = port.write_all(&version_pkt.to_bytes()) {
                            log::error!("Failed to send version request: {}", e);
                        }
                    } else {
                        log::error!("Failed to lock port for version request");
                    }
                }

                // Handle version response
                if packet.cmd == CMD_VERSION && !version_received {
                    handle_version_packet(&packet, &version_data, &mut version_received);
                }

                // Handle sensor status data
                if packet.cmd == CMD_STATUS && packet.payload.len() >= STATUS_PAYLOAD_MIN_SIZE {
                    handle_status_packet(&packet, &sensor_data);
                }
            }
            Ok(None) => {
                // No packet yet, continue
            }
            Err(e) => {
                log::error!("Packet read error: {}", e);
                thread::sleep(Duration::from_millis(10));
            }
        }

        // Small sleep to prevent busy loop
        thread::sleep(Duration::from_millis(2));
    }

    log::info!("Reader thread exiting");
}

/// Handle version response packet
fn handle_version_packet(
    packet: &super::protocol::Packet,
    version_data: &Option<Arc<Mutex<SensorGroupData>>>,
    version_received: &mut bool,
) {
    if let Some(ref vdata) = version_data {
        if !packet.payload.is_empty() {
            let Ok(mut data) = vdata.lock() else {
                log::error!("Failed to lock version data");
                return;
            };
            data.touch();

            // Parse version string
            let version_string = if packet.payload[0] < 128 {
                let len = packet.payload[0] as usize;
                if packet.payload.len() > len {
                    String::from_utf8_lossy(&packet.payload[1..=len]).to_string()
                } else {
                    String::from_utf8_lossy(&packet.payload).to_string()
                }
            } else {
                let null_pos = packet
                    .payload
                    .iter()
                    .position(|&b| b == 0)
                    .unwrap_or(packet.payload.len());
                String::from_utf8_lossy(&packet.payload[..null_pos]).to_string()
            };

            // Parse version code
            let version_code = if packet.payload.len() >= 8 {
                i32::from_le_bytes([
                    packet.payload[4],
                    packet.payload[5],
                    packet.payload[6],
                    packet.payload[7],
                ])
            } else {
                0
            };

            data.update(
                "version_string",
                SensorValue::String(version_string.clone()),
            );
            data.update("version_code", SensorValue::I32(version_code));

            log::info!("GD32 version: {} ({})", version_string, version_code);
            *version_received = true;
        }
    }
}

/// Handle status packet and update sensor data
fn handle_status_packet(
    packet: &super::protocol::Packet,
    sensor_data: &Arc<Mutex<SensorGroupData>>,
) {
    // Update shared data directly (no allocations)
    let Ok(mut data) = sensor_data.lock() else {
        log::error!("Failed to lock sensor data");
        return;
    };
    let payload = &packet.payload;

    // Update timestamp
    data.touch();

    // Charging/battery status
    data.update(
        "is_charging",
        SensorValue::Bool((payload[OFFSET_CHARGING_FLAGS] & FLAG_CHARGING) != 0),
    );
    data.update(
        "is_dock_connected",
        SensorValue::Bool((payload[OFFSET_CHARGING_FLAGS] & FLAG_DOCK_CONNECTED) != 0),
    );

    // Buttons
    data.update(
        "start_button",
        SensorValue::U16(u16::from_le_bytes([
            payload[OFFSET_START_BUTTON],
            payload[OFFSET_START_BUTTON + 1],
        ])),
    );
    data.update(
        "dock_button",
        SensorValue::U16(u16::from_le_bytes([
            payload[OFFSET_DOCK_BUTTON],
            payload[OFFSET_DOCK_BUTTON + 1],
        ])),
    );

    // Bumpers
    data.update(
        "bumper_left",
        SensorValue::Bool((payload[OFFSET_BUMPER_FLAGS] & FLAG_BUMPER_LEFT) != 0),
    );
    data.update(
        "bumper_right",
        SensorValue::Bool((payload[OFFSET_BUMPER_FLAGS] & FLAG_BUMPER_RIGHT) != 0),
    );

    // Wheel encoders
    data.update(
        "wheel_left",
        SensorValue::U16(u16::from_le_bytes([
            payload[OFFSET_WHEEL_LEFT_ENCODER],
            payload[OFFSET_WHEEL_LEFT_ENCODER + 1],
        ])),
    );
    data.update(
        "wheel_right",
        SensorValue::U16(u16::from_le_bytes([
            payload[OFFSET_WHEEL_RIGHT_ENCODER],
            payload[OFFSET_WHEEL_RIGHT_ENCODER + 1],
        ])),
    );

    // Cliff sensors
    data.update(
        "cliff_left_side",
        SensorValue::Bool((payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_LEFT_SIDE) != 0),
    );
    data.update(
        "cliff_left_front",
        SensorValue::Bool((payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_LEFT_FRONT) != 0),
    );
    data.update(
        "cliff_right_front",
        SensorValue::Bool((payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_RIGHT_FRONT) != 0),
    );
    data.update(
        "cliff_right_side",
        SensorValue::Bool((payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_RIGHT_SIDE) != 0),
    );

    // Dustbox
    data.update(
        "dustbox_attached",
        SensorValue::Bool((payload[OFFSET_DUSTBOX_FLAGS] & FLAG_DUSTBOX_ATTACHED) != 0),
    );

    // Raw packet bytes for debugging/reverse engineering
    data.update("raw_packet", SensorValue::Bytes(payload.to_vec()));
}
