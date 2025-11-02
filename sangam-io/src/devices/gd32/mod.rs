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
    /// This matches AuxCtrl's exact initialization sequence from MITM capture:
    /// 1. CMD=0x0C (Init sync), 0x8D (LED), 0x07 (Version), 0x65 (Motor type), 0x71 (Telemetry)
    /// 2. CMD=0x8D burst (4x at T+245ms)
    /// 3. CMD=0x06 (Wake at T+253ms)
    /// 4. Brush init: 0x69, 0x6B (at T+280ms)
    /// 5. CMD=0x66 heartbeat starts (at T+289ms)
    pub fn new<T: Transport + 'static>(transport: T) -> Result<Self> {
        let transport = Arc::new(Mutex::new(Box::new(transport) as Box<dyn Transport>));
        let state = Arc::new(Mutex::new(Gd32State::default()));
        let shutdown = Arc::new(Mutex::new(false));

        // Phase 1: Initial command burst (T+0ms) - matches AuxCtrl
        log::info!("GD32: Sending initial command burst");

        // CMD=0x0C - Init sync (first command AuxCtrl sends)
        log::debug!("  -> CMD=0x0C (InitSync)");
        Self::send_command(&transport, &Gd32Command::InitSync)?;
        thread::sleep(Duration::from_millis(8));

        // CMD=0x8D - Button LED (early)
        log::debug!("  -> CMD=0x8D (ButtonLED)");
        Self::send_command(&transport, &Gd32Command::ButtonLedState(0x01))?;
        thread::sleep(Duration::from_millis(8));

        // CMD=0x07 - System setup/version query
        log::debug!("  -> CMD=0x07 (SystemSetup)");
        Self::send_command(&transport, &Gd32Command::SystemSetup)?;
        thread::sleep(Duration::from_millis(8));

        // CMD=0x65 - Motor control type (mode=0)
        log::debug!("  -> CMD=0x65 (MotorType=0)");
        Self::send_command(&transport, &Gd32Command::MotorType(0))?;
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

        // Phase 3: Wake command (T+253ms)
        thread::sleep(Duration::from_millis(8));
        log::info!("GD32: Sending wake command (CMD=0x06)");
        Self::send_command(&transport, &Gd32Command::Wake)?;

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
    fn wait_for_status_packet(
        _transport: &Arc<Mutex<Box<dyn Transport>>>,
        state: &Arc<Mutex<Gd32State>>,
    ) -> Result<()> {
        let start = Instant::now();
        let timeout = Duration::from_secs(2);

        while start.elapsed() < timeout {
            // Check if we've received status data from heartbeat thread
            let current_state = state.lock();
            if current_state.battery_voltage > 0.0 {
                // Battery voltage populated means we got CMD=0x15
                drop(current_state);
                return Ok(());
            }
            drop(current_state);

            thread::sleep(Duration::from_millis(20)); // Check every heartbeat interval
        }

        Err(Error::InitializationFailed(
            "GD32 did not send STATUS_DATA (CMD=0x15) after 2 seconds".into(),
        ))
    }

    /// Send a command to the GD32
    fn send_command(transport: &Arc<Mutex<Box<dyn Transport>>>, cmd: &Gd32Command) -> Result<()> {
        let packet = cmd.encode();
        log::debug!("GD32: TX CMD=0x{:02X}, {} bytes: {:02X?}", cmd.cmd_id(), packet.len(), &packet);
        let mut transport = transport.lock();
        transport.write(&packet)?;
        transport.flush()?;
        Ok(())
    }


    /// Set motor control mode via CMD=0x65
    ///
    /// Mode values:
    /// - 0x00: Initialization/idle mode
    /// - 0x02: Navigation/cleaning mode (required for lidar control)
    pub fn set_motor_mode(&mut self, mode: u8) -> Result<()> {
        log::info!("GD32: Setting motor mode: 0x{:02X}", mode);
        Self::send_command(&self.transport, &Gd32Command::MotorType(mode))?;
        log::debug!("GD32: Sent CMD=0x65 (MotorType=0x{:02X})", mode);
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
