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

        // Phase 5: Start heartbeat thread (T+289ms)
        thread::sleep(Duration::from_millis(9));
        log::info!("GD32: Starting heartbeat thread (CMD=0x66)");
        let heartbeat_thread = Some(heartbeat::spawn_heartbeat_thread(
            Arc::clone(&transport),
            Arc::clone(&state),
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
            heartbeat_thread,
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
            if current_state.battery_voltage > 0.0 {
                // Battery voltage populated means we got CMD=0x15
                drop(current_state);
                log::info!(
                    "GD32: Device responded after {} wake attempts",
                    wake_attempts
                );
                return Ok(());
            }

            // If in sleep mode (detected by heartbeat thread), send additional wake commands
            let is_sleeping = current_state.sleep_mode;
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

    /// Send a command to the GD32
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
        Self::send_command(&self.transport, &Gd32Command::MotorControlType(mode))?;
        log::debug!("GD32: Sent CMD=0x65 (MotorControlType=0x{:02X})", mode);
        Ok(())
    }

    /// Send lidar preparation command via CMD=0xA2
    ///
    /// Must be sent before enabling lidar power (CMD=0x97).
    /// Discovered via MITM capture of AuxCtrl cleaning session.
    pub fn send_lidar_prep(&mut self) -> Result<()> {
        log::info!("GD32: Sending lidar preparation command");
        Self::send_command(&self.transport, &Gd32Command::LidarPrep)?;
        log::debug!("GD32: Sent CMD=0xA2 (LidarPrep)");
        Ok(())
    }

    /// Set lidar motor power via CMD=0x97
    ///
    /// Controls GPIO 233 through GD32 firmware using CMD=0x97.
    /// Discovered via AuxCtrl decompilation at address 0x00060988.
    /// NOTE: Must send CMD=0xA2 (LidarPrep) before this command!
    pub fn set_lidar_power(&mut self, enable: bool) -> Result<()> {
        log::info!("GD32: Setting lidar power: {}", enable);
        Self::send_command(&self.transport, &Gd32Command::LidarPower(enable))?;
        log::debug!("GD32: Sent CMD=0x97 (LidarPower={})", enable);
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
        Self::send_command(&self.transport, &Gd32Command::LidarPWM(pwm_percent))?;
        log::debug!("GD32: Sent CMD=0x71 (LidarPWM={})", clamped);
        Ok(())
    }

    /// Set vacuum blower speed (0-100%)
    pub fn set_blower_speed(&mut self, speed: u8) -> Result<()> {
        let speed_value = (speed as u16) * 100; // Scale to device range
        Self::send_command(&self.transport, &Gd32Command::BlowerSpeed(speed_value))
    }

    /// Reset GD32 error codes (CMD=0x0A)
    /// Clears any accumulated error flags in the motor controller
    pub fn reset_error_code(&mut self) -> Result<()> {
        Self::send_command(&self.transport, &Gd32Command::ResetErrorCode)?;
        log::debug!("GD32: Sent CMD=0x0A (ResetErrorCode)");
        Ok(())
    }

    /// Put GD32 into sleep mode (CMD=0x04)
    /// WARNING: Device may require hardware wake (GPIO) or button press to recover
    pub fn sleep(&mut self) -> Result<()> {
        log::warn!("GD32: Putting device into sleep mode (CMD=0x04)");
        Self::send_command(&self.transport, &Gd32Command::Sleep)?;
        log::debug!("GD32: Sent CMD=0x04 (Sleep)");
        Ok(())
    }

    /// Send wakeup acknowledgment (CMD=0x05)
    /// Used to acknowledge wakeup signal or attempt software wake
    pub fn wakeup_ack(&mut self) -> Result<()> {
        log::info!("GD32: Sending wakeup acknowledgment (CMD=0x05)");
        Self::send_command(&self.transport, &Gd32Command::WakeupAck)?;
        log::debug!("GD32: Sent CMD=0x05 (WakeupAck)");
        Ok(())
    }

    /// Control R16 module power (CMD=0x99)
    /// The R16 module handles certain sensor or control functions
    pub fn set_r16_power(&mut self, enable: bool) -> Result<()> {
        Self::send_command(&self.transport, &Gd32Command::R16Power(enable))?;
        log::debug!("GD32: Sent CMD=0x99 (R16Power={})", enable);
        Ok(())
    }

    /// Restart R16 module (CMD=0x9A)
    /// Performs a soft reset of the R16 subsystem
    pub fn restart_r16(&mut self) -> Result<()> {
        Self::send_command(&self.transport, &Gd32Command::RestartR16)?;
        log::debug!("GD32: Sent CMD=0x9A (RestartR16)");
        Ok(())
    }

    /// Read latest state from heartbeat thread
    fn read_state(&self) -> Gd32State {
        self.state.lock().clone()
    }

    /// Get battery information
    ///
    /// Returns (voltage in V, current in A, level 0-100%)
    pub fn get_battery_info(&self) -> (f32, f32, u8) {
        let state = self.read_state();
        (
            state.battery_voltage,
            state.battery_current,
            state.battery_level,
        )
    }

    /// Get encoder counts
    ///
    /// Returns (left_ticks, right_ticks)
    pub fn get_encoder_counts(&self) -> (i32, i32) {
        let state = self.read_state();
        (state.encoder_left, state.encoder_right)
    }

    /// Get current error code
    pub fn get_error_code(&self) -> u8 {
        let state = self.read_state();
        state.error_code
    }

    /// Get status flags
    ///
    /// Returns (status_flag, charging_flag, battery_state_flag, percent_value)
    pub fn get_status_flags(&self) -> (u8, bool, bool, f32) {
        let state = self.read_state();
        (
            state.status_flag,
            state.charging_flag,
            state.battery_state_flag,
            state.percent_value,
        )
    }

    /// Get IR proximity sensor values
    ///
    /// All values use ×5 scaling. Button detection: 100-199 = pressed.
    /// Returns (ir_sensor_1, point_button_ir, dock_button_ir)
    pub fn get_ir_sensors(&self) -> (u16, u16, u16) {
        let state = self.read_state();
        (
            state.ir_sensor_1,
            state.point_button_ir,
            state.dock_button_ir,
        )
    }

    /// Check if point button is pressed (IR value 100-199)
    pub fn is_point_button_pressed(&self) -> bool {
        let state = self.read_state();
        state.point_button_ir >= 100 && state.point_button_ir < 200
    }

    /// Check if dock button is pressed (IR value 100-199)
    pub fn is_dock_button_pressed(&self) -> bool {
        let state = self.read_state();
        state.dock_button_ir >= 100 && state.dock_button_ir < 200
    }

    /// Get packet statistics
    ///
    /// Returns (total_rx_packets, total_tx_packets, lost_packet_count)
    pub fn get_packet_stats(&self) -> (u64, u64, u32) {
        let state = self.read_state();
        (
            state.total_rx_packets,
            state.total_tx_packets,
            state.lost_packet_count,
        )
    }

    /// Set raw motor speeds (for testing/debugging)
    ///
    /// Directly sets motor speed values in encoder ticks.
    /// Use this for testing to find correct motor speed ranges.
    ///
    /// # Arguments
    /// * `left` - Left motor speed in encoder ticks
    /// * `right` - Right motor speed in encoder ticks
    pub fn set_raw_motor_speeds(&mut self, left: i32, right: i32) -> Result<()> {
        log::info!(
            "GD32: Setting raw motor speeds: left={}, right={}",
            left,
            right
        );
        let mut state = self.state.lock();
        state.target_left = left;
        state.target_right = right;
        Ok(())
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

    fn set_vacuum(&mut self, power: u8) -> Result<()> {
        let clamped = power.min(100);
        log::info!("GD32: Setting vacuum power: {}%", clamped);
        self.set_blower_speed(clamped)
    }

    fn set_side_brush(&mut self, speed: u8) -> Result<()> {
        let clamped = speed.min(100);
        log::info!("GD32: Setting side brush speed: {}%", clamped);
        Self::send_command(&self.transport, &Gd32Command::SideBrushSpeed(clamped))?;
        log::debug!("GD32: Sent CMD=0x69 (SideBrushSpeed={})", clamped);
        Ok(())
    }

    fn set_main_brush(&mut self, speed: u8) -> Result<()> {
        let clamped = speed.min(100);
        log::info!("GD32: Setting main/rolling brush speed: {}%", clamped);
        Self::send_command(&self.transport, &Gd32Command::RollingBrushSpeed(clamped))?;
        log::debug!("GD32: Sent CMD=0x6A (RollingBrushSpeed={})", clamped);
        Ok(())
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
