//! Command receiver for receiving robot commands via TCP sockets.
//!
//! Receives RobotCommand messages from external processes (SLAM, Navigation)
//! and executes them on hardware after simple unit conversion.

use crate::devices::Gd32Driver;
use crate::error::Result;
use crate::streaming::messages::RobotCommand;
use log::{debug, error, info, warn};
use parking_lot::Mutex;
use std::io::Read;
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::time::Duration;

/// Command receiver that receives commands from TCP connections
pub struct TcpCommandReceiver {
    listener: TcpListener,
    gd32_driver: Arc<Mutex<Gd32Driver>>,
    max_ticks_per_sec: f64,
    active_client: Option<TcpStream>,
}

impl TcpCommandReceiver {
    /// Create a new TCP command receiver
    ///
    /// # Arguments
    /// - `bind_address`: TCP bind address (e.g., "0.0.0.0:5556")
    /// - `gd32_driver`: Reference to GD32 driver for command execution
    /// - `max_ticks_per_sec`: Maximum encoder ticks per second (calibration constant)
    ///
    /// # Returns
    /// New TcpCommandReceiver instance
    pub fn new(
        bind_address: String,
        gd32_driver: Arc<Mutex<Gd32Driver>>,
        max_ticks_per_sec: f64,
    ) -> Result<Self> {
        let listener = TcpListener::bind(&bind_address)?;
        listener.set_nonblocking(true)?;

        info!("TCP command receiver bound to {}", bind_address);

        Ok(Self {
            listener,
            gd32_driver,
            max_ticks_per_sec,
            active_client: None,
        })
    }

    /// Process one command with timeout
    ///
    /// # Arguments
    /// - `timeout`: Maximum time to wait for a command
    ///
    /// # Returns
    /// Ok(()) if command processed successfully or timeout occurred
    /// Err() if there was an error processing the command
    pub fn process_command(&mut self, timeout: Duration) -> Result<()> {
        // Accept new client if we don't have one
        if self.active_client.is_none() {
            match self.listener.accept() {
                Ok((stream, addr)) => {
                    info!("Command client connected: {}", addr);
                    if let Err(e) = stream.set_nonblocking(false) {
                        warn!("Failed to set blocking mode for command client: {}", e);
                    } else {
                        self.active_client = Some(stream);
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No new connections, continue
                }
                Err(e) => {
                    error!("Error accepting command client: {}", e);
                }
            }
        }

        // If we have an active client, try to read a command
        if let Some(ref mut client) = self.active_client {
            // Set read timeout
            if let Err(e) = client.set_read_timeout(Some(timeout)) {
                warn!("Failed to set read timeout: {}", e);
            }

            // Read length-prefixed message: [4-byte length][topic (null-terminated)][MessagePack payload]
            let mut length_buf = [0u8; 4];
            match client.read_exact(&mut length_buf) {
                Ok(_) => {
                    let length = u32::from_be_bytes(length_buf) as usize;

                    // Sanity check: limit message size to 1MB
                    if length > 1_000_000 {
                        error!("Message too large: {} bytes", length);
                        self.active_client = None;
                        return Ok(());
                    }

                    // Read the frame
                    let mut frame_buf = vec![0u8; length];
                    if let Err(e) = client.read_exact(&mut frame_buf) {
                        debug!("Failed to read command frame: {}", e);
                        self.active_client = None;
                        return Ok(());
                    }

                    // Parse topic (null-terminated string) and payload
                    if let Some(null_pos) = frame_buf.iter().position(|&b| b == 0) {
                        let topic = &frame_buf[..null_pos];
                        let payload = &frame_buf[null_pos + 1..];

                        // Verify topic is "command"
                        if topic != b"command" {
                            debug!(
                                "Ignoring non-command topic: {:?}",
                                String::from_utf8_lossy(topic)
                            );
                            return Ok(());
                        }

                        // Deserialize command
                        let command: RobotCommand = match rmp_serde::from_slice(payload) {
                            Ok(cmd) => cmd,
                            Err(e) => {
                                error!("Failed to deserialize command: {}", e);
                                return Ok(());
                            }
                        };

                        // Execute command
                        self.execute_command(command)?;
                    } else {
                        warn!("Invalid command frame: no null terminator in topic");
                    }
                }
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    // Timeout occurred, not an error
                    return Ok(());
                }
                Err(e) => {
                    debug!("Client disconnected or error reading: {}", e);
                    self.active_client = None;
                    return Ok(());
                }
            }
        }

        Ok(())
    }

    /// Execute a robot command on hardware
    fn execute_command(&mut self, command: RobotCommand) -> Result<()> {
        debug!("Executing command: {:?}", command);

        let mut driver = self.gd32_driver.lock();

        match command {
            RobotCommand::SetWheelVelocity { left, right } => {
                // Convert ticks/sec to GD32 motor units (-2000 to 2000)
                let left_motor = self.ticks_per_sec_to_motor_value(left);
                let right_motor = self.ticks_per_sec_to_motor_value(right);

                debug!(
                    "Setting wheel velocities: left={:.1} ticks/s ({}) right={:.1} ticks/s ({})",
                    left, left_motor, right, right_motor
                );

                driver.set_wheel_velocity(left_motor as f32, right_motor as f32)?;
            }

            RobotCommand::SetAirPumpSpeed { speed } => {
                debug!("Setting air pump speed: {}%", speed);

                driver.set_air_pump(speed)?;
            }

            RobotCommand::SetMainBrushSpeed { speed } => {
                debug!("Setting main brush speed: {}%", speed);

                driver.set_rolling_brush(speed)?;
            }

            RobotCommand::SetSideBrushSpeed { speed } => {
                debug!("Setting side brush speed: {}%", speed);

                driver.set_side_brush(speed)?;
            }

            RobotCommand::EnableLidar { pwm } => {
                info!("Enabling lidar with PWM: {}%", pwm);

                // Full lidar initialization sequence (verified from MITM logs)
                // This sequence is critical and timing-sensitive!
                //
                // NOTE: Motor mode switching to 0x02 is now automatic!
                // The heartbeat thread detects lidar_powered=true && last_lidar_pwm!=0
                // and auto-switches to mode 0x02 within 1 second

                // Step 1: Send lidar preparation command (CMD=0xA2)
                driver.send_lidar_prep()?;
                std::thread::sleep(std::time::Duration::from_millis(50));

                // Step 2: Power on lidar (CMD=0x97) - updates state.lidar_powered=true
                driver.set_lidar_power(true)?;
                std::thread::sleep(std::time::Duration::from_millis(50));

                // Step 3: Set lidar PWM speed (CMD=0x71) - updates state.last_lidar_pwm
                driver.set_lidar_pwm(pwm as i32)?;

                // Wait for motor stabilization (critical for first scan)
                // Motor mode will auto-switch to 0x02 within 1 second (heartbeat cycle)
                std::thread::sleep(std::time::Duration::from_millis(1100));

                info!(
                    "Lidar enabled successfully (CMD=0xA2/0x97/0x71 sequence complete, mode auto-switching)"
                );
            }

            RobotCommand::DisableLidar => {
                info!("Disabling lidar");

                driver.set_lidar_power(false)?;

                info!("Lidar disabled successfully");
            }

            RobotCommand::SetLidarPWM { pwm } => {
                debug!("Setting lidar PWM: {}%", pwm);

                driver.set_lidar_pwm(pwm as i32)?;
            }

            RobotCommand::EmergencyStopAll => {
                warn!("Emergency stop all requested");

                // Stop wheels
                driver.set_wheel_velocity(0.0, 0.0)?;

                // Stop all actuators
                driver.set_air_pump(0)?;
                driver.set_rolling_brush(0)?;
                driver.set_side_brush(0)?;

                // Stop lidar
                driver.set_lidar_power(false)?;

                info!("Emergency stop completed");
            }
        }

        Ok(())
    }

    /// Convert ticks per second to GD32 motor units (-2000 to 2000)
    fn ticks_per_sec_to_motor_value(&self, ticks_per_sec: f64) -> i16 {
        let normalized = ticks_per_sec / self.max_ticks_per_sec;
        let motor_value = normalized * 2000.0;
        motor_value.clamp(-2000.0, 2000.0) as i16
    }
}

impl Drop for TcpCommandReceiver {
    fn drop(&mut self) {
        info!("TCP command receiver shutting down");
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_ticks_per_sec_conversion() {
        // Test unit conversion logic
        let max_ticks = 3000.0;

        let test_cases = vec![
            (3000.0, 2000),   // Full speed forward
            (1500.0, 1000),   // Half speed forward
            (0.0, 0),         // Stop
            (-3000.0, -2000), // Full speed backward
            (5000.0, 2000),   // Over-limit should clamp to 2000
            (-5000.0, -2000), // Over-limit should clamp to -2000
        ];

        for (ticks, expected_motor) in test_cases {
            let normalized = ticks / max_ticks;
            let motor_value = (normalized * 2000.0_f64).clamp(-2000.0, 2000.0) as i16;
            assert_eq!(motor_value, expected_motor);
        }
    }
}
