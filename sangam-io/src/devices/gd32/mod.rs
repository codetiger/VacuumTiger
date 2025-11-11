//! GD32F103 motor controller driver

mod heartbeat;
mod protocol;
mod state;

use crate::drivers::motor::MotorDriver;
use crate::error::{Error, Result};
use crate::transport::Transport;
use crate::types::Odometry;

use parking_lot::Mutex;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::thread;
use std::time::{Duration, Instant};

pub use protocol::Gd32Command;
use state::Gd32State;

/// GD32F103 motor controller driver
///
/// Handles initialization, heartbeat management, and communication with GD32 MCU.
///
/// Uses a dual-thread architecture:
/// - READ thread: Fast 2ms loop for receiving STATUS_DATA (never blocks on writes)
/// - WRITE thread: 20ms loop for sending commands (can block without affecting reads)
pub struct Gd32Driver {
    /// Transport layer for serial I/O (shared between threads)
    #[allow(dead_code)] // Kept to maintain Arc reference count
    transport: Arc<Mutex<Box<dyn Transport>>>,
    /// Shared state between threads
    state: Arc<Mutex<Gd32State>>,
    /// READ thread handle (2ms loop)
    read_thread: Option<thread::JoinHandle<()>>,
    /// WRITE thread handle (20ms loop)
    write_thread: Option<thread::JoinHandle<()>>,
    /// Command queue sender (for ad-hoc commands to write thread)
    command_tx: std::sync::mpsc::SyncSender<Gd32Command>,
    /// Shutdown flag for both threads
    shutdown: Arc<Mutex<bool>>,
}

impl Gd32Driver {
    /// Create new GD32 driver and initialize device
    ///
    /// This matches AuxCtrl's initialization sequence from reverse engineering and MITM:
    /// 0. CMD=0x08 (Initialize) retry loop - 10-30 attempts with 18ms intervals
    /// 1. CMD=0x8D (LED), 0x07 (Version), 0x65 (Motor type), 0x71 (Telemetry)
    /// 2. CMD=0x8D burst (4x at T+245ms)
    /// 3. CMD=0x06 (Wake at T+253ms)
    /// 4. Brush init: 0x69, 0x6B (at T+280ms)
    /// 5. CMD=0x66 heartbeat starts (at T+289ms)
    pub fn new<T: Transport + 'static>(transport: T) -> Result<Self> {
        let transport = Arc::new(Mutex::new(Box::new(transport) as Box<dyn Transport>));
        let state = Arc::new(Mutex::new(Gd32State::default()));
        let shutdown = Arc::new(Mutex::new(false));

        // Phase 0: CMD=0x08 initialization retry loop (matches AuxCtrl behavior)
        // MITM logs show 20-30 attempts with 18-20ms intervals
        // When in deep sleep, more attempts may be needed to wake the device
        log::info!("GD32: Sending CMD=0x08 initialization sequence");
        let init_cmd = Gd32Command::default_initialize();
        let max_retries = 30;

        for attempt in 1..=max_retries {
            log::debug!(
                "  -> CMD=0x08 (Initialize) attempt {}/{}",
                attempt,
                max_retries
            );
            Self::send_command(&transport, &init_cmd)?;

            // Try to read response (CMD=0x15 status packet expected)
            // TODO: Actually parse response to confirm initialization success
            thread::sleep(Duration::from_millis(18));
        }

        log::info!(
            "GD32: Initialization sequence complete ({} CMD=0x08 attempts)",
            max_retries
        );

        // Phase 1: Initial command burst (T+0ms) - matches AuxCtrl
        log::info!("GD32: Sending initial command burst");

        // CMD=0x8D - Button LED (early)
        log::debug!("  -> CMD=0x8D (ButtonLED)");
        Self::send_command(&transport, &Gd32Command::ButtonLedState(0x01))?;
        thread::sleep(Duration::from_millis(8));

        // CMD=0x07 - System setup/version query
        log::debug!("  -> CMD=0x07 (SystemSetup)");
        Self::send_command(&transport, &Gd32Command::SystemSetup)?;
        thread::sleep(Duration::from_millis(8));

        // CMD=0x65 - Motor control type (mode=0)
        log::debug!("  -> CMD=0x65 (MotorControlType=0)");
        Self::send_command(&transport, &Gd32Command::MotorControlType(0))?;
        thread::sleep(Duration::from_millis(8));

        // CMD=0x71 - Lidar PWM (set to 0 initially, will be set properly when powering lidar)
        log::debug!("  -> CMD=0x71 (LidarPWM=0)");
        Self::send_command(&transport, &Gd32Command::LidarPWM(0))?;

        // Phase 2: Wait ~245ms then send LED burst
        log::info!("GD32: Waiting 245ms before LED burst");
        thread::sleep(Duration::from_millis(200)); // Conservative timing

        log::info!("GD32: Sending button LED burst (4x)");
        for i in 0..4 {
            Self::send_command(&transport, &Gd32Command::ButtonLedState(0x01))?;
            if i < 3 {
                thread::sleep(Duration::from_millis(8));
            }
        }

        // Phase 3: Heartbeat command (T+253ms)
        thread::sleep(Duration::from_millis(8));
        log::info!("GD32: Sending heartbeat command (CMD=0x06)");
        Self::send_command(&transport, &Gd32Command::Heartbeat)?;

        // Phase 4: Brush initialization (T+280ms)
        thread::sleep(Duration::from_millis(27));
        log::info!("GD32: Initializing brushes");
        Self::send_command(&transport, &Gd32Command::SideBrushSpeed(0))?;
        thread::sleep(Duration::from_millis(1));
        Self::send_command(&transport, &Gd32Command::BrushControl(0x64))?; // Value from MITM log

        // Phase 5: Start dual-thread architecture (T+289ms)
        thread::sleep(Duration::from_millis(9));
        log::info!("GD32: Starting dual-thread architecture (READ: 2ms, WRITE: 20ms)");

        // Create bounded command queue for ad-hoc commands (16 commands max)
        // This provides backpressure to prevent memory exhaustion from rapid command submission
        const COMMAND_QUEUE_SIZE: usize = 16;
        let (command_tx, command_rx) = std::sync::mpsc::sync_channel(COMMAND_QUEUE_SIZE);

        // Spawn READ thread (fast 2ms loop, never blocks on writes)
        let read_thread = Some(heartbeat::spawn_read_thread(
            Arc::clone(&transport),
            Arc::clone(&state),
            Arc::clone(&shutdown),
        ));

        // Spawn WRITE thread (20ms loop, handles all command transmission)
        let write_thread = Some(heartbeat::spawn_write_thread(
            Arc::clone(&transport),
            Arc::clone(&state),
            command_rx,
            Arc::clone(&shutdown),
        ));

        // Wait for GD32 to respond with STATUS_DATA
        log::info!("GD32: Waiting for STATUS_DATA (CMD=0x15) from GD32...");
        Self::wait_for_status_packet(&transport, &state)?;
        log::info!("GD32: Received STATUS_DATA - GD32 is ready");

        log::info!("GD32: Initialization complete - ready for operations");

        Ok(Gd32Driver {
            transport,
            state,
            read_thread,
            write_thread,
            command_tx,
            shutdown,
        })
    }

    /// Wait for GD32 to send CMD=0x15 STATUS_DATA packet
    ///
    /// The heartbeat thread receives and processes status packets.
    /// We check the shared state to see if battery voltage has been populated,
    /// which indicates that at least one status packet has been received.
    ///
    /// If the device is in deep sleep mode, continue sending CMD=0x08 to wake it.
    fn wait_for_status_packet(
        transport: &Arc<Mutex<Box<dyn Transport>>>,
        state: &Arc<Mutex<Gd32State>>,
    ) -> Result<()> {
        let start = Instant::now();
        let timeout = Duration::from_secs(5); // Increased timeout for deep sleep wake
        let init_cmd = Gd32Command::default_initialize();
        let mut wake_attempts = 0;
        let mut check_count = 0;

        while start.elapsed() < timeout {
            // Check if we've received status data from heartbeat thread
            let current_state = state.lock();
            if current_state
                .diagnostics
                .total_rx_packets
                .load(Ordering::Relaxed)
                > 0
            {
                // Received at least one STATUS_DATA packet
                drop(current_state);
                log::info!(
                    "GD32: Device responded after {} wake attempts",
                    wake_attempts
                );
                return Ok(());
            }

            // If in sleep mode (detected by heartbeat thread), send additional wake commands
            let is_sleeping = current_state.diagnostics.sleep_mode.load(Ordering::Relaxed);
            drop(current_state);

            // Send CMD=0x08 every 5 cycles (5 * 20ms = 100ms) when in sleep mode
            if is_sleeping && check_count % 5 == 0 {
                let packet = init_cmd.encode();
                let mut t = transport.lock();
                if t.write(&packet).is_ok() {
                    wake_attempts += 1;
                    log::debug!(
                        "GD32: Sending additional CMD=0x08 wake attempt #{}",
                        wake_attempts
                    );
                }
            }

            check_count += 1;
            thread::sleep(Duration::from_millis(20)); // Check every heartbeat interval
        }

        Err(Error::InitializationFailed(format!(
            "GD32 did not send STATUS_DATA (CMD=0x15) after {} seconds ({} wake attempts)",
            timeout.as_secs(),
            wake_attempts
        )))
    }

    /// Send a command to the GD32 (for initialization only, before threads start)
    fn send_command(transport: &Arc<Mutex<Box<dyn Transport>>>, cmd: &Gd32Command) -> Result<()> {
        let packet = cmd.encode();
        log::debug!(
            "GD32: TX CMD=0x{:02X}, {} bytes: {:02X?}",
            cmd.cmd_id(),
            packet.len(),
            &packet
        );
        let mut transport = transport.lock();
        transport.write(&packet)?;
        transport.flush()?; // Required for initialization timing
        Ok(())
    }

    /// Queue a command to the write thread
    fn queue_command(&self, cmd: Gd32Command) -> Result<()> {
        use std::sync::mpsc::TrySendError;

        // Try to send without blocking
        match self.command_tx.try_send(cmd) {
            Ok(()) => Ok(()),
            Err(TrySendError::Full(_)) => {
                // Queue is full - provide backpressure feedback
                Err(Error::Other(
                    "Command queue full (16 pending commands). Please wait for commands to process.".into()
                ))
            }
            Err(TrySendError::Disconnected(_)) => {
                // Write thread has exited
                Err(Error::Io(std::io::Error::new(
                    std::io::ErrorKind::BrokenPipe,
                    "Write thread disconnected",
                )))
            }
        }
    }

    /// Set motor control mode via CMD=0x65
    ///
    /// Mode values (VERIFIED via hardware testing + MITM analysis):
    /// - 0x00: Initialization mode (used during boot sequence)
    /// - 0x01: Direct motor control mode (supports CMD=0x67 MotorSpeed only)
    /// - 0x02: Navigation/cleaning mode (supports CMD=0x66 MotorVelocity + all accessories)
    ///
    /// **CRITICAL MODE RESTRICTIONS** (verified via hardware tests):
    /// - **Mode 0x01**: CMD=0x67 (MotorSpeed) ✅, CMD=0x66 (MotorVelocity) ❓, accessories ❌
    /// - **Mode 0x02**: CMD=0x66 (MotorVelocity) ✅, CMD=0x67 (MotorSpeed) ❌ (error 0xFF), accessories ✅
    ///
    /// **AuxCtrl uses mode 0x02 throughout cleaning with CMD=0x66 for motor control.**
    /// This driver now uses the same approach: mode 0x02 + CMD=0x66 for unified control.
    pub fn set_motor_mode(&mut self, mode: u8) -> Result<()> {
        log::info!("GD32: Setting motor control mode: 0x{:02X}", mode);

        // Update state so write thread knows which command to use
        {
            let state = self.state.lock();
            let mut cmd_state = state.command.lock();
            cmd_state.motor_mode = mode;
        }

        self.queue_command(Gd32Command::MotorControlType(mode))?;
        log::debug!("GD32: Queued CMD=0x65 (MotorControlType=0x{:02X})", mode);
        Ok(())
    }

    /// Send lidar preparation command via CMD=0xA2
    ///
    /// Must be sent before enabling lidar power (CMD=0x97).
    /// Discovered via MITM capture of AuxCtrl cleaning session.
    pub fn send_lidar_prep(&mut self) -> Result<()> {
        log::info!("GD32: Sending lidar preparation command");
        self.queue_command(Gd32Command::LidarPrep)?;
        log::debug!("GD32: Queued CMD=0xA2 (LidarPrep)");
        Ok(())
    }

    /// Set lidar motor power via CMD=0x97
    ///
    /// Controls GPIO 233 through GD32 firmware using CMD=0x97.
    /// Discovered via AuxCtrl decompilation at address 0x00060988.
    /// NOTE: Must send CMD=0xA2 (LidarPrep) before this command!
    pub fn set_lidar_power(&mut self, enable: bool) -> Result<()> {
        log::info!("GD32: Setting lidar power: {}", enable);
        self.queue_command(Gd32Command::LidarPower(enable))?;
        log::debug!("GD32: Queued CMD=0x97 (LidarPower={})", enable);
        Ok(())
    }

    /// Set lidar motor PWM speed via CMD=0x71
    ///
    /// Controls lidar motor speed (0-100%).
    /// Discovered via AuxCtrl decompilation at address 0x0003f604.
    ///
    /// # Arguments
    /// * `pwm_percent` - PWM duty cycle (0-100%). Values outside range will be clamped.
    pub fn set_lidar_pwm(&mut self, pwm_percent: i32) -> Result<()> {
        let clamped = pwm_percent.clamp(0, 100);
        log::info!("GD32: Setting lidar PWM: {}%", clamped);
        self.queue_command(Gd32Command::LidarPWM(pwm_percent))?;
        log::debug!("GD32: Queued CMD=0x71 (LidarPWM={})", clamped);
        Ok(())
    }

    /// Set vacuum blower speed (0-100%)
    pub fn set_blower_speed(&mut self, speed: u8) -> Result<()> {
        let speed_value = (speed as u16) * 100; // Scale to device range
        self.queue_command(Gd32Command::BlowerSpeed(speed_value))
    }

    /// Get battery information
    ///
    /// Returns (voltage in V, current in A, level 0-100%) or None if no telemetry received yet
    pub fn get_battery_info(&self) -> Option<(f32, f32, u8)> {
        let state = self.state.lock();
        let telemetry = state.telemetry.lock();
        telemetry.as_ref().and_then(|t| {
            match (t.battery_voltage, t.battery_current, t.battery_level) {
                (Some(v), Some(c), Some(l)) => Some((v, c, l)),
                _ => None,
            }
        })
    }

    /// Get current error code
    pub fn get_error_code(&self) -> Option<u8> {
        let state = self.state.lock();
        let telemetry = state.telemetry.lock();
        telemetry.as_ref().and_then(|t| t.error_code)
    }

    /// Get status flags
    ///
    /// Returns (status_flag, charging_flag, battery_state_flag, percent_value) or None if no telemetry
    pub fn get_status_flags(&self) -> Option<(u8, bool, bool, f32)> {
        let state = self.state.lock();
        let telemetry = state.telemetry.lock();
        telemetry.as_ref().and_then(|t| {
            match (
                t.status_flag,
                t.charging_flag,
                t.battery_state_flag,
                t.percent_value,
            ) {
                (Some(s), Some(c), Some(b), Some(p)) => Some((s, c, b, p)),
                _ => None,
            }
        })
    }

    /// Get IR proximity sensor values
    ///
    /// All values use ×5 scaling. Button detection: 100-199 = pressed.
    /// Returns (ir_sensor_1, start_button_ir, dock_button_ir) or None if no telemetry
    pub fn get_ir_sensors(&self) -> Option<(u16, u16, u16)> {
        let state = self.state.lock();
        let telemetry = state.telemetry.lock();
        telemetry.as_ref().and_then(|t| {
            match (t.ir_sensor_1, t.start_button_ir, t.dock_button_ir) {
                (Some(i1), Some(sb), Some(db)) => Some((i1, sb, db)),
                _ => None,
            }
        })
    }

    /// Check if start button is pressed (IR value 100-199)
    pub fn is_start_button_pressed(&self) -> Option<bool> {
        let state = self.state.lock();
        let telemetry = state.telemetry.lock();
        telemetry
            .as_ref()
            .and_then(|t| t.start_button_ir.map(|v| (100..200).contains(&v)))
    }

    /// Check if dock button is pressed (IR value 100-199)
    pub fn is_dock_button_pressed(&self) -> Option<bool> {
        let state = self.state.lock();
        let telemetry = state.telemetry.lock();
        telemetry
            .as_ref()
            .and_then(|t| t.dock_button_ir.map(|v| (100..200).contains(&v)))
    }

    /// Get packet statistics
    ///
    /// Returns (total_rx_packets, total_tx_packets, lost_packet_count)
    pub fn get_packet_stats(&self) -> (u64, u64, u32) {
        let state = self.state.lock();
        (
            state.diagnostics.total_rx_packets.load(Ordering::Relaxed),
            state.diagnostics.total_tx_packets.load(Ordering::Relaxed),
            state.diagnostics.lost_packet_count.load(Ordering::Relaxed),
        )
    }

    /// Check if telemetry is fresh (not stale)
    ///
    /// Returns true if telemetry is fresher than max_age, false otherwise
    pub fn is_telemetry_fresh(&self, max_age: Duration) -> bool {
        self.state.lock().is_telemetry_fresh(max_age)
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
        // Motor speeds are already in the correct range (-2000 to 2000)
        // from the motion controller, so just cast to i32
        let left_ticks = left as i32;
        let right_ticks = right as i32;

        // Log motor speeds periodically for debugging
        log::debug!(
            "GD32: Setting motor speeds - left={}, right={}",
            left_ticks,
            right_ticks
        );

        // Update target state for heartbeat thread
        {
            let state = self.state.lock();
            let mut cmd_state = state.command.lock();
            cmd_state.target_left = left_ticks;
            cmd_state.target_right = right_ticks;
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
        // Get encoder values from telemetry
        let state = self.state.lock();
        let telemetry = state.telemetry.lock();

        let (encoder_left, encoder_right) = match telemetry.as_ref() {
            Some(t) => match (t.encoder_left, t.encoder_right) {
                (Some(l), Some(r)) => (l, r),
                _ => return Err(Error::NotInitialized),
            },
            None => return Err(Error::NotInitialized),
        };
        drop(telemetry);
        drop(state);

        // Convert encoder ticks to position
        // These conversion factors need hardware calibration
        const WHEEL_RADIUS: f32 = 0.05; // meters
        const TICKS_PER_REVOLUTION: f32 = 1000.0; // Placeholder

        let left_distance = (encoder_left as f32) / TICKS_PER_REVOLUTION
            * 2.0
            * std::f32::consts::PI
            * WHEEL_RADIUS;
        let right_distance = (encoder_right as f32) / TICKS_PER_REVOLUTION
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
            encoder_left,
            encoder_right,
        })
    }

    fn set_air_pump(&mut self, power: u8) -> Result<()> {
        let clamped = power.min(100);
        log::info!("GD32: Setting air pump power: {}%", clamped);
        self.set_blower_speed(clamped)
    }

    fn set_side_brush(&mut self, speed: u8) -> Result<()> {
        let clamped = speed.min(100);
        log::info!("GD32: Setting side brush speed: {}%", clamped);
        self.queue_command(Gd32Command::SideBrushSpeed(clamped))?;
        log::debug!("GD32: Queued CMD=0x69 (SideBrushSpeed={})", clamped);
        Ok(())
    }

    fn set_rolling_brush(&mut self, speed: u8) -> Result<()> {
        let clamped = speed.min(100);
        log::info!("GD32: Setting rolling brush speed: {}%", clamped);
        self.queue_command(Gd32Command::RollingBrushSpeed(clamped))?;
        log::debug!("GD32: Queued CMD=0x6A (RollingBrushSpeed={})", clamped);
        Ok(())
    }
}

impl Drop for Gd32Driver {
    fn drop(&mut self) {
        log::info!("GD32: Shutting down driver");

        // Constants for shutdown behavior
        const SHUTDOWN_TIMEOUT: Duration = Duration::from_millis(500);

        let shutdown_start = Instant::now();

        // IMPORTANT: Queue stop commands BEFORE signaling shutdown
        // This ensures the write thread can send them before stopping
        log::info!("GD32: Queueing stop commands for all components");

        // Stop all components - use queue_command to ensure they're sent via heartbeat thread
        if let Err(e) = self.queue_command(Gd32Command::BlowerSpeed(0)) {
            log::warn!("GD32: Failed to queue vacuum stop: {}", e);
        } else {
            log::debug!("GD32: Vacuum stop command queued");
        }

        if let Err(e) = self.queue_command(Gd32Command::RollingBrushSpeed(0)) {
            log::warn!("GD32: Failed to queue main brush stop: {}", e);
        } else {
            log::debug!("GD32: Main brush stop command queued");
        }

        if let Err(e) = self.queue_command(Gd32Command::SideBrushSpeed(0)) {
            log::warn!("GD32: Failed to queue side brush stop: {}", e);
        } else {
            log::debug!("GD32: Side brush stop command queued");
        }

        // Stop motors based on current mode
        let motor_mode = self.state.lock().command.lock().motor_mode;
        let stop_cmd = match motor_mode {
            0x01 => Gd32Command::MotorSpeed { left: 0, right: 0 },
            0x02 => Gd32Command::motor_velocity_with_speeds(0, 0),
            _ => Gd32Command::MotorSpeed { left: 0, right: 0 },
        };

        if let Err(e) = self.queue_command(stop_cmd) {
            log::warn!("GD32: Failed to queue motor stop: {}", e);
        } else {
            log::debug!("GD32: Motor stop command queued");
        }

        // Give write thread time to send the stop commands
        log::debug!("GD32: Waiting for stop commands to be sent");
        thread::sleep(Duration::from_millis(100));

        // Now signal shutdown to both threads
        *self.shutdown.lock() = true;

        // Track thread exit status
        let mut read_thread_ok = true;
        let mut write_thread_ok = true;

        // Wait for READ thread to exit with timeout
        if let Some(handle) = self.read_thread.take() {
            log::debug!("GD32: Waiting for READ thread to exit");

            // Simple timeout implementation using a busy wait
            // Note: std::thread::JoinHandle doesn't have join_timeout in stable Rust
            let timeout = SHUTDOWN_TIMEOUT.saturating_sub(shutdown_start.elapsed());
            let deadline = Instant::now() + timeout;

            loop {
                if handle.is_finished() {
                    match handle.join() {
                        Ok(()) => {
                            log::debug!("GD32: READ thread exited cleanly");
                            break;
                        }
                        Err(panic_payload) => {
                            log::error!("GD32: READ thread panicked: {:?}", panic_payload);
                            read_thread_ok = false;
                            break;
                        }
                    }
                }

                if Instant::now() >= deadline {
                    log::warn!(
                        "GD32: READ thread did not exit within timeout, may be blocked on I/O"
                    );
                    read_thread_ok = false;
                    break;
                }

                thread::sleep(Duration::from_millis(10));
            }
        }

        // Wait for WRITE thread to exit with timeout
        if let Some(handle) = self.write_thread.take() {
            log::debug!("GD32: Waiting for WRITE thread to exit");

            let timeout = SHUTDOWN_TIMEOUT.saturating_sub(shutdown_start.elapsed());
            let deadline = Instant::now() + timeout;

            loop {
                if handle.is_finished() {
                    match handle.join() {
                        Ok(()) => {
                            log::debug!("GD32: WRITE thread exited cleanly");
                            break;
                        }
                        Err(panic_payload) => {
                            log::error!("GD32: WRITE thread panicked: {:?}", panic_payload);
                            write_thread_ok = false;
                            break;
                        }
                    }
                }

                if Instant::now() >= deadline {
                    log::warn!(
                        "GD32: WRITE thread did not exit within timeout, may be blocked on I/O"
                    );
                    write_thread_ok = false;
                    break;
                }

                thread::sleep(Duration::from_millis(10));
            }
        }

        // Log thread exit status
        if !read_thread_ok || !write_thread_ok {
            log::warn!("GD32: One or more threads did not exit cleanly");
        } else {
            log::debug!("GD32: All threads exited cleanly");
        }

        let total_shutdown_time = shutdown_start.elapsed();
        log::info!(
            "GD32: Driver shutdown complete (took {:?})",
            total_shutdown_time
        );
    }
}
