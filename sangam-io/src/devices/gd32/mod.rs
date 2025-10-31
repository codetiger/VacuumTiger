//! GD32F103 motor controller driver

mod heartbeat;
mod protocol;
mod state;

use crate::drivers::MotorDriver;
use crate::error::{Error, Result};
use crate::transport::Transport;
use crate::types::Odometry;

use parking_lot::Mutex;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

pub use protocol::{Gd32Command, Gd32Response};
use state::Gd32State;

/// GD32F103 motor controller driver
///
/// Handles initialization, heartbeat management, and communication with GD32 MCU.
pub struct Gd32Driver {
    /// Transport layer for serial I/O
    transport: Arc<Mutex<Box<dyn Transport>>>,
    /// Shared state between main thread and heartbeat thread
    state: Arc<Mutex<Gd32State>>,
    /// Heartbeat thread handle
    heartbeat_thread: Option<thread::JoinHandle<()>>,
    /// Shutdown flag for heartbeat thread
    shutdown: Arc<Mutex<bool>>,
}

impl Gd32Driver {
    /// Create new GD32 driver and initialize device
    ///
    /// This will:
    /// 1. Send CMD=0x08 initialization loop (up to 5 seconds)
    /// 2. Send wake command (CMD=0x06)
    /// 3. Set control mode (CMD=0x8D)
    /// 4. Start heartbeat thread (CMD=0x66 every 20ms)
    pub fn new<T: Transport + 'static>(transport: T) -> Result<Self> {
        let transport = Arc::new(Mutex::new(Box::new(transport) as Box<dyn Transport>));
        let state = Arc::new(Mutex::new(Gd32State::default()));
        let shutdown = Arc::new(Mutex::new(false));

        // Phase 1: Initialization sequence
        log::info!("GD32: Starting initialization sequence");
        Self::initialize_device(&transport)?;

        // Phase 2: Wake and setup
        log::info!("GD32: Sending wake command");
        Self::send_command(&transport, &Gd32Command::Wake)?;
        thread::sleep(Duration::from_millis(50));

        log::info!("GD32: Setting control mode");
        Self::send_command(&transport, &Gd32Command::ButtonLedState(0x01))?;
        thread::sleep(Duration::from_millis(50));

        // Phase 3: Start heartbeat thread
        log::info!("GD32: Starting heartbeat thread");
        let heartbeat_thread = Some(heartbeat::spawn_heartbeat_thread(
            Arc::clone(&transport),
            Arc::clone(&state),
            Arc::clone(&shutdown),
        ));

        log::info!("GD32: Initialization complete");

        Ok(Gd32Driver {
            transport,
            state,
            heartbeat_thread,
            shutdown,
        })
    }

    /// Initialize device with CMD=0x08 loop
    ///
    /// Sends initialization command every 200ms for up to 5 seconds
    /// until GD32 responds with CMD=0x15 status packet.
    fn initialize_device(transport: &Arc<Mutex<Box<dyn Transport>>>) -> Result<()> {
        // STEP 1: Flush any leftover data from previous session
        log::debug!("GD32: Flushing serial buffer...");
        {
            let mut transport = transport.lock();
            let mut discard = vec![0u8; 1024];
            let mut total_flushed = 0;

            // Read until buffer is empty
            while transport.available()? > 0 {
                let read = transport.read(&mut discard)?;
                total_flushed += read;
                if read == 0 {
                    break;
                }
            }

            if total_flushed > 0 {
                log::debug!("GD32: Flushed {} bytes of old data", total_flushed);
            }
        }

        // Small delay to ensure buffer is clear
        thread::sleep(Duration::from_millis(100));

        // STEP 2: Send initialization commands
        let init_cmd = Gd32Command::default_initialize();
        let init_packet = init_cmd.encode();
        let start = Instant::now();
        let timeout = Duration::from_secs(5);
        let retry_interval = Duration::from_millis(200);

        // Packet buffer to accumulate bytes across reads
        let mut packet_buffer: Vec<u8> = Vec::new();

        while start.elapsed() < timeout {
            // Send initialization command
            {
                let mut transport = transport.lock();
                transport.write(&init_packet)?;
                transport.flush()?;
            }

            log::debug!("GD32: Sent initialization command");

            // Try to read response with packet accumulation
            let read_start = Instant::now();
            while read_start.elapsed() < retry_interval {
                if let Some(response) =
                    Self::try_read_response_buffered(transport, &mut packet_buffer)?
                    && response.is_status_packet()
                {
                    log::info!("GD32: Device initialized successfully");
                    return Ok(());
                }
                thread::sleep(Duration::from_millis(10));
            }
        }

        Err(Error::InitializationFailed(
            "GD32 initialization timeout (no response after 5 seconds)".into(),
        ))
    }

    /// Send a command to the GD32
    fn send_command(transport: &Arc<Mutex<Box<dyn Transport>>>, cmd: &Gd32Command) -> Result<()> {
        let packet = cmd.encode();
        let mut transport = transport.lock();
        transport.write(&packet)?;
        transport.flush()?;
        Ok(())
    }

    /// Try to read a response packet with buffering (non-blocking)
    fn try_read_response_buffered(
        transport: &Arc<Mutex<Box<dyn Transport>>>,
        packet_buffer: &mut Vec<u8>,
    ) -> Result<Option<Gd32Response>> {
        let mut transport = transport.lock();

        // Check if data is available
        let available = transport.available()?;
        if available > 0 {
            // Read new data and append to buffer
            let mut temp_buffer = vec![0u8; available.min(256)];
            let bytes_read = transport.read(&mut temp_buffer)?;

            if bytes_read > 0 {
                packet_buffer.extend_from_slice(&temp_buffer[..bytes_read]);
                log::debug!(
                    "GD32: Read {} bytes, buffer now {} bytes",
                    bytes_read,
                    packet_buffer.len()
                );
            }
        }

        // Release lock before processing
        drop(transport);

        // Try to decode a packet from accumulated buffer
        if packet_buffer.len() >= 6 {
            match Gd32Response::decode_with_sync(packet_buffer) {
                Ok((bytes_consumed, response)) => {
                    // Remove consumed bytes from buffer
                    packet_buffer.drain(..bytes_consumed);
                    log::debug!(
                        "GD32: Decoded packet (CMD=0x{:02X}), {} bytes remaining in buffer",
                        response.cmd_id,
                        packet_buffer.len()
                    );
                    return Ok(Some(response));
                }
                Err(e) => {
                    // If buffer is getting too large without valid packets, discard some bytes to make progress
                    log::debug!(
                        "GD32: Decode error: {}, buffer size: {}",
                        e,
                        packet_buffer.len()
                    );
                    if packet_buffer.len() > 512 {
                        // Search for sync bytes manually, discard everything before first sync
                        let mut found_sync = false;
                        for i in 0..packet_buffer.len().saturating_sub(1) {
                            if packet_buffer[i] == 0xFA && packet_buffer[i + 1] == 0xFB {
                                // Found sync, keep from here
                                packet_buffer.drain(..i);
                                found_sync = true;
                                log::debug!("GD32: Discarded {} bytes of garbage, found sync", i);
                                break;
                            }
                        }
                        if !found_sync {
                            // No sync found, keep only last byte (might be first half of sync)
                            let to_discard = packet_buffer.len() - 1;
                            packet_buffer.drain(..to_discard);
                            log::debug!("GD32: No sync found, discarded {} bytes", to_discard);
                        }
                    }
                }
            }
        }

        Ok(None)
    }

    /// Set lidar motor power
    pub fn set_lidar_power(&mut self, on: bool) -> Result<()> {
        log::info!("GD32: Setting lidar power: {}", on);
        Self::send_command(&self.transport, &Gd32Command::LidarPower(on))
    }

    /// Set vacuum blower speed (0-100%)
    pub fn set_blower_speed(&mut self, speed: u8) -> Result<()> {
        let speed_value = (speed as u16) * 100; // Scale to device range
        Self::send_command(&self.transport, &Gd32Command::BlowerSpeed(speed_value))
    }

    /// Read latest state from heartbeat thread
    fn read_state(&self) -> Gd32State {
        self.state.lock().clone()
    }
}

impl MotorDriver for Gd32Driver {
    fn set_velocity(&mut self, linear: f32, angular: f32) -> Result<()> {
        // Convert linear and angular velocity to wheel speeds
        // Assuming differential drive with wheel base of 0.3m and wheel radius of 0.05m
        const WHEEL_BASE: f32 = 0.3;
        const WHEEL_RADIUS: f32 = 0.05;

        let left = (linear - angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS;
        let right = (linear + angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS;

        self.set_wheel_velocity(left, right)
    }

    fn set_wheel_velocity(&mut self, left: f32, right: f32) -> Result<()> {
        // Convert rad/s to motor units (encoder ticks per control cycle)
        // These conversion factors need hardware calibration
        const TICKS_PER_RADIAN: f32 = 100.0; // Placeholder value

        let left_ticks = (left * TICKS_PER_RADIAN) as i32;
        let right_ticks = (right * TICKS_PER_RADIAN) as i32;

        // Update target state for heartbeat thread
        {
            let mut state = self.state.lock();
            state.target_left = left_ticks;
            state.target_right = right_ticks;
        }

        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        self.set_wheel_velocity(0.0, 0.0)
    }

    fn emergency_stop(&mut self) -> Result<()> {
        // Same as normal stop for now
        self.stop()
    }

    fn get_odometry(&mut self) -> Result<Odometry> {
        let state = self.read_state();

        // Convert encoder ticks to position
        // These conversion factors need hardware calibration
        const WHEEL_RADIUS: f32 = 0.05; // meters
        const TICKS_PER_REVOLUTION: f32 = 1000.0; // Placeholder

        let left_distance = (state.encoder_left as f32) / TICKS_PER_REVOLUTION
            * 2.0
            * std::f32::consts::PI
            * WHEEL_RADIUS;
        let right_distance = (state.encoder_right as f32) / TICKS_PER_REVOLUTION
            * 2.0
            * std::f32::consts::PI
            * WHEEL_RADIUS;

        // Simple differential drive odometry
        let distance = (left_distance + right_distance) / 2.0;

        Ok(Odometry {
            x: distance,
            y: 0.0, // Simplified - needs proper pose tracking
            theta: 0.0,
            velocity: crate::types::Velocity::zero(), // Could be calculated from encoder changes
            encoder_left: state.encoder_left,
            encoder_right: state.encoder_right,
        })
    }
}

impl Drop for Gd32Driver {
    fn drop(&mut self) {
        log::info!("GD32: Shutting down driver");

        // Signal shutdown to heartbeat thread
        *self.shutdown.lock() = true;

        // Wait for heartbeat thread to exit
        if let Some(handle) = self.heartbeat_thread.take() {
            let _ = handle.join();
        }

        // Send stop command
        let _ = Self::send_command(
            &self.transport,
            &Gd32Command::MotorSpeed { left: 0, right: 0 },
        );

        log::info!("GD32: Driver shutdown complete");
    }
}
