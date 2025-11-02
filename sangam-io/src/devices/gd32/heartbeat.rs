//! Background heartbeat thread management
//!
//! The GD32 requires a continuous heartbeat (CMD=0x66) every 20ms.
//! Missing heartbeats will cause the device to enter error state and stop motors.

use crate::transport::Transport;
use parking_lot::Mutex;
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use super::protocol::Gd32Command;
use super::state::Gd32State;

/// Heartbeat interval (20ms = 50Hz)
const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(20);

/// Spawn background heartbeat thread
///
/// The thread will:
/// 1. Send CMD=0x66 heartbeat every 20ms with motor speed commands
/// 2. Send CMD=0x71 telemetry query every ~27ms (matches AuxCtrl frequency)
/// 3. Send CMD=0x06 wake command every ~250ms (matches AuxCtrl frequency)
/// 4. Read and process CMD=0x15 status packets
/// 5. Update shared state with encoder and battery data
/// 6. Run until shutdown flag is set
pub fn spawn_heartbeat_thread(
    transport: Arc<Mutex<Box<dyn Transport>>>,
    state: Arc<Mutex<Gd32State>>,
    shutdown: Arc<Mutex<bool>>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        log::info!("GD32: Heartbeat thread started");

        let mut last_heartbeat = Instant::now();
        let mut heartbeat_count = 0u64;

        loop {
            // Check shutdown flag
            if *shutdown.lock() {
                log::info!("GD32: Heartbeat thread shutting down");
                break;
            }

            // Wait until next heartbeat interval
            let elapsed = last_heartbeat.elapsed();
            if elapsed < HEARTBEAT_INTERVAL {
                thread::sleep(HEARTBEAT_INTERVAL - elapsed);
            }
            last_heartbeat = Instant::now();
            heartbeat_count += 1;

            // Get current target speeds from shared state
            let (target_left, target_right) = {
                let state = state.lock();
                (state.target_left, state.target_right)
            };

            // Send heartbeat with motor speed command
            // The heartbeat doubles as the motor control command
            let heartbeat_cmd = Gd32Command::MotorSpeed {
                left: target_left,
                right: target_right,
            };

            let packet = heartbeat_cmd.encode();

            // Send packet
            {
                let mut transport = transport.lock();

                // Log TX on first heartbeat and every 50th
                if heartbeat_count == 1 || heartbeat_count % 50 == 0 {
                    log::debug!("GD32: TX {} bytes: {:02X?}", packet.len(), &packet[..packet.len().min(32)]);
                }

                if let Err(e) = transport.write(&packet) {
                    log::error!("GD32: Heartbeat send error: {}", e);
                    continue;
                }
                if let Err(e) = transport.flush() {
                    log::error!("GD32: Heartbeat flush error: {}", e);
                    continue;
                }
            }

            // Send CMD=0x06 wake command periodically (every ~250ms)
            // AuxCtrl sends this 1,421x in 6min = ~4/sec = every ~250ms
            // At 20ms intervals, every 12th heartbeat = 240ms
            if heartbeat_count % 12 == 0 {
                send_command_unlocked(&transport, &Gd32Command::Wake);
            }

            if heartbeat_count % 50 == 0 {
                // Log every second
                log::debug!(
                    "GD32: Heartbeat #{} (L={}, R={})",
                    heartbeat_count,
                    target_left,
                    target_right
                );
            }

            // Try to read any incoming status packets
            if let Err(e) = read_and_process_responses(&transport, &state) {
                log::warn!("GD32: Error processing responses: {}", e);
            }
        }

        log::info!("GD32: Heartbeat thread stopped");
    })
}

/// Send a command without logging (helper for heartbeat loop)
fn send_command_unlocked(transport: &Arc<Mutex<Box<dyn Transport>>>, cmd: &Gd32Command) {
    let packet = cmd.encode();
    let mut transport = transport.lock();
    let _ = transport.write(&packet);
    let _ = transport.flush();
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
        return Ok(());
    }

    // Try to read data
    let mut buffer = vec![0u8; 256];
    let bytes_read = transport.read(&mut buffer)?;

    if bytes_read == 0 {
        return Ok(());
    }

    // Log raw bytes received
    log::debug!(
        "GD32: RX {} bytes: {:02X?}",
        bytes_read,
        &buffer[..bytes_read.min(32)]
    );

    // Try to decode response
    match Gd32Response::decode(&buffer[..bytes_read]) {
        Ok(response) => {
            log::debug!(
                "GD32: RX decoded - CMD=0x{:02X}, payload_len={}",
                response.cmd_id,
                response.payload.len()
            );

            if response.is_status_packet() {
                // Update shared state
                let mut state = state.lock();
                state.battery_voltage = response.battery_voltage;
                state.battery_current = response.battery_current;
                state.battery_level = response.battery_level;
                state.encoder_left = response.encoder_left;
                state.encoder_right = response.encoder_right;
                state.error_code = response.error_code;

                log::debug!(
                    "GD32: Status update - Battery: {:.2}V ({}%), Encoders: L={}, R={}, Error={}",
                    response.battery_voltage,
                    response.battery_level,
                    response.encoder_left,
                    response.encoder_right,
                    response.error_code
                );
            } else {
                log::debug!("GD32: Non-status response: CMD=0x{:02X}", response.cmd_id);
            }
        }
        Err(e) => {
            log::warn!("GD32: Failed to decode packet: {} (bytes: {:02X?})", e, &buffer[..bytes_read.min(16)]);
        }
    }

    Ok(())
}
