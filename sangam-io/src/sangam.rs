//! SangamIO - Hardware abstraction layer for robotic vacuum control

use crate::config::SangamConfig;
use crate::devices::{Delta2DDriver, Gd32Driver};
use crate::drivers::lidar::LidarDriver;
use crate::drivers::motor::MotorDriver;
use crate::error::{Error, Result};
use crate::motion::{MotionCommand, MotionController, MotionStatus, Velocity2D};
use crate::odometry::{OdometryDelta, OdometryTracker};
use crate::transport::SerialTransport;
use crate::types::LidarScan;

use parking_lot::Mutex;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// SangamIO - Hardware abstraction layer for vacuum robot
///
/// Provides mid-level control interface for SLAM and navigation algorithms.
/// Handles motor control, odometry, lidar, and cleaning components.
///
/// # Examples
///
/// ## Basic Movement and Sensor Monitoring
///
/// ```no_run
/// use sangam_io::SangamIO;
///
/// # fn main() -> sangam_io::Result<()> {
/// let sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;
///
/// // Monitor battery level
/// if let Some(level) = sangam.get_battery_level() {
///     println!("Battery: {}%", level);
///     if level < 20 {
///         println!("Low battery - returning to dock");
///     }
/// }
///
/// // Check button presses
/// if sangam.is_start_button_pressed().unwrap_or(false) {
///     println!("Start button pressed!");
/// }
///
/// // Move forward while monitoring sensors
/// sangam.set_velocity(0.2, 0.0)?;  // 0.2 m/s forward
///
/// // Check telemetry freshness
/// if !sangam.is_telemetry_fresh() {
///     println!("Warning: Lost communication with motor controller");
/// }
/// # Ok(())
/// # }
/// ```
///
/// ## Sensor-Based Obstacle Avoidance
///
/// ```no_run
/// use sangam_io::SangamIO;
///
/// # fn main() -> sangam_io::Result<()> {
/// let sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;
///
/// // Read IR proximity sensors
/// if let Some((ir1, start_ir, dock_ir)) = sangam.get_ir_sensors() {
///     // IR values 100-199 indicate button press detection
///     if start_ir >= 100 && start_ir < 200 {
///         println!("Start button detected via IR");
///     }
///
///     // High IR values indicate nearby obstacles
///     if ir1 > 500 {
///         println!("Obstacle detected, stopping");
///         sangam.stop()?;
///     }
/// }
/// # Ok(())
/// # }
/// ```
pub struct SangamIO {
    /// Motor driver (required)
    motor_driver: Arc<Mutex<Box<dyn MotorDriver>>>,

    /// Lidar driver (optional)
    lidar_driver: Option<Arc<Mutex<Delta2DDriver>>>,

    /// Motion controller
    motion: Arc<Mutex<MotionController>>,

    /// Odometry tracker
    odometry: Arc<Mutex<OdometryTracker>>,

    /// Configuration
    config: SangamConfig,

    /// Control thread handle
    control_thread: Option<JoinHandle<()>>,

    /// Shutdown flag
    shutdown: Arc<AtomicBool>,

    /// Telemetry freshness tracking (reserved for future use)
    #[allow(dead_code)]
    last_telemetry: Arc<Mutex<Instant>>,
}

impl SangamIO {
    // === Constructors ===

    /// Create SangamIO for CRL-200S hardware configuration
    ///
    /// # Arguments
    /// * `gd32_port` - Serial port for GD32 motor controller (e.g., "/dev/ttyS3")
    /// * `lidar_port` - Serial port for Delta-2D lidar (e.g., "/dev/ttyS1")
    pub fn crl200s(gd32_port: &str, lidar_port: &str) -> Result<Self> {
        log::info!("SangamIO: Initializing CRL-200S configuration");

        // Initialize GD32 motor controller
        log::info!("SangamIO: Connecting to GD32 on {}", gd32_port);
        let gd32_transport = SerialTransport::open(gd32_port, 115200)?;
        let gd32 = Gd32Driver::new(gd32_transport)?;

        // Wait for stabilization
        log::info!("SangamIO: Waiting for motor controller stabilization");
        thread::sleep(Duration::from_millis(1400));

        // Initialize Delta-2D lidar
        log::info!("SangamIO: Connecting to Delta-2D lidar on {}", lidar_port);
        let lidar_transport = SerialTransport::open(lidar_port, 115200)?;
        let lidar = Delta2DDriver::new(lidar_transport)?;

        // Use default CRL-200S configuration
        let config = SangamConfig::crl200s_defaults();

        Self::new_with_drivers(Box::new(gd32), Some(lidar), config)
    }

    /// Create SangamIO with custom drivers and configuration
    fn new_with_drivers(
        motor_driver: Box<dyn MotorDriver>,
        lidar_driver: Option<Delta2DDriver>,
        config: SangamConfig,
    ) -> Result<Self> {
        let motor_driver = Arc::new(Mutex::new(motor_driver));
        let lidar_driver = lidar_driver.map(|d| Arc::new(Mutex::new(d)));

        let motion = Arc::new(Mutex::new(MotionController::new(config.clone())));
        let odometry = Arc::new(Mutex::new(OdometryTracker::new(config.clone())));
        let shutdown = Arc::new(AtomicBool::new(false));
        let last_telemetry = Arc::new(Mutex::new(Instant::now()));

        // Start control loop thread
        let control_thread = Some(Self::start_control_loop(
            Arc::clone(&motion),
            Arc::clone(&odometry),
            Arc::clone(&motor_driver),
            Arc::clone(&shutdown),
            Arc::clone(&last_telemetry),
            config.clone(),
        ));

        log::info!("SangamIO: Initialization complete");

        Ok(Self {
            motor_driver,
            lidar_driver,
            motion,
            odometry,
            config,
            control_thread,
            shutdown,
            last_telemetry,
        })
    }

    // === Velocity Control ===

    /// Set velocity (non-blocking, continuous motion)
    ///
    /// # Arguments
    /// * `linear` - Linear velocity in m/s (positive = forward)
    /// * `angular` - Angular velocity in rad/s (positive = CCW)
    pub fn set_velocity(&self, linear: f32, angular: f32) -> Result<()> {
        log::debug!(
            "SangamIO: Set velocity - linear={:.3}m/s, angular={:.3}rad/s",
            linear,
            angular
        );

        let mut motion = self.motion.lock();
        motion.execute_command(MotionCommand::Velocity { linear, angular });
        Ok(())
    }

    /// Stop with normal deceleration
    pub fn stop(&self) -> Result<()> {
        log::info!("SangamIO: Stop requested (normal deceleration)");

        let mut motion = self.motion.lock();
        motion.stop(false);
        Ok(())
    }

    /// Emergency stop with maximum deceleration
    pub fn emergency_stop(&self) -> Result<()> {
        log::info!("SangamIO: Emergency stop requested");

        let mut motion = self.motion.lock();
        motion.stop(true);
        Ok(())
    }

    // === Position-based Movements ===

    /// Move forward specified distance (non-blocking)
    pub fn move_forward(&self, distance: f32) -> Result<()> {
        if distance <= 0.0 {
            return Err(Error::InvalidParameter(
                "Distance must be positive".to_string(),
            ));
        }

        log::info!("SangamIO: Move forward {:.3}m", distance);

        let mut motion = self.motion.lock();
        motion.execute_command(MotionCommand::MoveDistance {
            distance,
            max_velocity: self.config.max_linear_velocity,
        });
        Ok(())
    }

    /// Move backward specified distance (non-blocking)
    pub fn move_backward(&self, distance: f32) -> Result<()> {
        if distance <= 0.0 {
            return Err(Error::InvalidParameter(
                "Distance must be positive".to_string(),
            ));
        }

        log::info!("SangamIO: Move backward {:.3}m", distance);

        let mut motion = self.motion.lock();
        motion.execute_command(MotionCommand::MoveDistance {
            distance: -distance,
            max_velocity: self.config.max_linear_velocity,
        });
        Ok(())
    }

    /// Rotate in place by specified angle (non-blocking)
    ///
    /// # Arguments
    /// * `angle` - Angle in radians (positive = CCW)
    pub fn rotate(&self, angle: f32) -> Result<()> {
        log::info!(
            "SangamIO: Rotate {:.3}rad ({:.1}°)",
            angle,
            angle.to_degrees()
        );

        let mut motion = self.motion.lock();
        motion.execute_command(MotionCommand::Rotate {
            angle,
            max_velocity: self.config.max_angular_velocity,
        });
        Ok(())
    }

    // === Odometry ===

    /// Get odometry delta since last call (for SLAM)
    pub fn get_odometry_delta(&self) -> Result<OdometryDelta> {
        let mut odometry = self.odometry.lock();
        Ok(odometry.get_delta_and_reset())
    }

    /// Reset odometry accumulator
    pub fn reset_odometry(&self) -> Result<()> {
        let mut odometry = self.odometry.lock();
        odometry.reset();
        Ok(())
    }

    /// Get raw encoder counts
    pub fn get_encoder_counts(&self) -> Result<(i32, i32)> {
        let mut driver = self.motor_driver.lock();
        let odometry = driver.get_odometry()?;
        Ok((odometry.encoder_left, odometry.encoder_right))
    }

    // === Motion Status ===

    /// Check if currently in motion
    pub fn is_moving(&self) -> bool {
        let motion = self.motion.lock();
        motion.is_moving()
    }

    /// Get current velocity
    pub fn get_velocity(&self) -> Result<Velocity2D> {
        let motion = self.motion.lock();
        Ok(motion.current_velocity)
    }

    /// Get detailed motion status
    pub fn motion_status(&self) -> Result<MotionStatus> {
        let motion = self.motion.lock();
        Ok(motion.status())
    }

    // === Lidar Control ===

    /// Start lidar scanning with callback
    pub fn start_lidar<F>(&mut self, callback: F) -> Result<()>
    where
        F: Fn(&LidarScan) + Send + 'static,
    {
        let lidar_arc = self
            .lidar_driver
            .as_ref()
            .ok_or(Error::ComponentNotAvailable("Lidar not configured"))?;

        // Power on lidar via GD32 (hardware-specific implementation)
        // NOTE: This uses downcast since MotorDriver trait doesn't expose lidar control
        {
            use std::any::Any;
            let mut driver = self.motor_driver.lock();

            if let Some(gd32) = (&mut **driver as &mut dyn Any).downcast_mut::<Gd32Driver>() {
                log::info!("SangamIO: Executing GD32 lidar power-up sequence");

                // Switch to navigation mode (enables lidar GPIO control)
                gd32.set_motor_mode(0x02)?;
                thread::sleep(Duration::from_millis(50));

                // Execute verified 3-step power sequence from MITM logs
                gd32.send_lidar_prep()?;
                thread::sleep(Duration::from_millis(50));

                gd32.set_lidar_power(true)?;
                thread::sleep(Duration::from_millis(50));

                gd32.set_lidar_pwm(100)?;

                log::info!("SangamIO: Lidar motor powered (CMD=0xA2/0x97/0x71 sequence complete)");
            } else {
                log::warn!("SangamIO: Motor driver is not GD32, cannot control lidar power");
                return Err(Error::ComponentNotAvailable(
                    "Lidar power control requires GD32 driver",
                ));
            }
        }

        // Wait for lidar motor to stabilize
        log::info!("SangamIO: Waiting for lidar motor stabilization ({:?})", self.config.lidar_stabilization_delay);
        thread::sleep(self.config.lidar_stabilization_delay);

        // Start lidar scanning with callback
        let mut lidar = lidar_arc.lock();
        lidar.start(callback)?;

        log::info!("SangamIO: Lidar scanning started");
        Ok(())
    }

    /// Stop lidar scanning
    pub fn stop_lidar(&mut self) -> Result<()> {
        let lidar_arc = self
            .lidar_driver
            .as_ref()
            .ok_or(Error::ComponentNotAvailable("Lidar not configured"))?;

        let mut lidar = lidar_arc.lock();
        lidar.stop()?;

        // Power off lidar motor if using GD32 (complete shutdown sequence)
        {
            use std::any::Any;
            let mut driver = self.motor_driver.lock();

            if let Some(gd32) = (&mut **driver as &mut dyn Any).downcast_mut::<Gd32Driver>() {
                log::info!("SangamIO: Powering off lidar motor");

                // Step 1: Stop PWM
                gd32.set_lidar_pwm(0)?;
                thread::sleep(Duration::from_millis(100));

                // Step 2: Disable GPIO 233 power
                gd32.set_lidar_power(false)?;

                log::info!("SangamIO: Lidar motor powered off (CMD=0x71/0x97 shutdown complete)");
            } else {
                log::warn!("SangamIO: Motor driver is not GD32, cannot control lidar power");
            }
        }

        Ok(())
    }

    /// Check if lidar is active
    pub fn is_lidar_active(&self) -> bool {
        self.lidar_driver
            .as_ref()
            .map(|l| l.lock().is_active())
            .unwrap_or(false)
    }

    /// Get lidar statistics
    pub fn lidar_stats(&self) -> Option<(u64, u64)> {
        self.lidar_driver.as_ref().map(|l| l.lock().get_stats())
    }

    // === Cleaning Components ===

    /// Set air pump power (0-100%)
    pub fn set_air_pump(&self, power: u8) -> Result<()> {
        log::debug!("SangamIO: Set air pump power to {}%", power);

        let mut driver = self.motor_driver.lock();
        driver.set_air_pump(power)
    }

    /// Set brush speeds
    pub fn set_brushes(&self, rolling: u8, side: u8) -> Result<()> {
        log::debug!(
            "SangamIO: Set brush speeds - rolling={}%, side={}%",
            rolling,
            side
        );

        let mut driver = self.motor_driver.lock();
        driver.set_rolling_brush(rolling)?;
        driver.set_side_brush(side)
    }

    // === Battery & Sensors ===

    /// Get battery charge level (0-100%)
    ///
    /// Returns `None` if telemetry data is not yet available from GD32.
    ///
    /// # Example
    /// ```no_run
    /// # use sangam_io::SangamIO;
    /// # let sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;
    /// if let Some(level) = sangam.get_battery_level() {
    ///     println!("Battery: {}%", level);
    ///     if level < 20 {
    ///         println!("Warning: Low battery!");
    ///     }
    /// }
    /// # Ok::<(), sangam_io::Error>(())
    /// ```
    pub fn get_battery_level(&self) -> Option<u8> {
        use std::any::Any;
        let driver = self.motor_driver.lock();

        if let Some(gd32) = (&**driver as &dyn Any).downcast_ref::<Gd32Driver>() {
            gd32.get_battery_info().map(|(_, _, level)| level)
        } else {
            None
        }
    }

    /// Get battery voltage (Volts)
    ///
    /// **Note:** Currently returns `None` due to protocol byte position conflicts.
    /// This is a placeholder for future implementation once the correct
    /// STATUS_DATA packet byte positions are identified.
    ///
    /// See: `src/devices/gd32/protocol.rs` lines 408-424 for details.
    pub fn get_battery_voltage(&self) -> Option<f32> {
        // TODO: Investigate correct byte positions (candidates: bytes 16-19)
        None
    }

    /// Get battery current (Amps)
    ///
    /// **Note:** Currently returns `None` due to protocol byte position conflicts.
    /// This is a placeholder for future implementation once the correct
    /// STATUS_DATA packet byte positions are identified.
    ///
    /// See: `src/devices/gd32/protocol.rs` lines 408-424 for details.
    pub fn get_battery_current(&self) -> Option<f32> {
        // TODO: Investigate correct byte positions (candidates: bytes 24-27)
        None
    }

    /// Check if robot is currently charging
    ///
    /// Returns `None` if telemetry data is not yet available from GD32.
    pub fn is_charging(&self) -> Option<bool> {
        use std::any::Any;
        let driver = self.motor_driver.lock();

        if let Some(gd32) = (&**driver as &dyn Any).downcast_ref::<Gd32Driver>() {
            gd32.get_status_flags().map(|(_, charging, _, _)| charging)
        } else {
            None
        }
    }

    /// Check if start button is pressed
    ///
    /// The start button is detected via IR sensor reading in range 100-199 (×5 scaling).
    /// Returns `None` if telemetry data is not yet available from GD32.
    ///
    /// # Example
    /// ```no_run
    /// # use sangam_io::SangamIO;
    /// # let sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;
    /// if sangam.is_start_button_pressed() == Some(true) {
    ///     println!("Start button pressed - starting cleaning!");
    /// }
    /// # Ok::<(), sangam_io::Error>(())
    /// ```
    pub fn is_start_button_pressed(&self) -> Option<bool> {
        use std::any::Any;
        let driver = self.motor_driver.lock();

        if let Some(gd32) = (&**driver as &dyn Any).downcast_ref::<Gd32Driver>() {
            gd32.is_start_button_pressed()
        } else {
            None
        }
    }

    /// Check if dock button is pressed
    ///
    /// The dock button is detected via IR sensor reading in range 100-199 (×5 scaling).
    /// Returns `None` if telemetry data is not yet available from GD32.
    pub fn is_dock_button_pressed(&self) -> Option<bool> {
        use std::any::Any;
        let driver = self.motor_driver.lock();

        if let Some(gd32) = (&**driver as &dyn Any).downcast_ref::<Gd32Driver>() {
            gd32.is_dock_button_pressed()
        } else {
            None
        }
    }

    /// Get IR proximity sensor readings (×5 scaling, raw values)
    ///
    /// Returns `(ir_sensor_1, start_button_ir, dock_button_ir)`.
    /// Values in range 100-199 indicate button press detection.
    ///
    /// Returns `None` if telemetry data is not yet available from GD32.
    pub fn get_ir_sensors(&self) -> Option<(u16, u16, u16)> {
        use std::any::Any;
        let driver = self.motor_driver.lock();

        if let Some(gd32) = (&**driver as &dyn Any).downcast_ref::<Gd32Driver>() {
            gd32.get_ir_sensors()
        } else {
            None
        }
    }

    /// Get GD32 error code from status telemetry
    ///
    /// Returns `None` if telemetry data is not yet available from GD32.
    /// Error code meanings are hardware-specific (see GD32 protocol documentation).
    pub fn get_error_code(&self) -> Option<u8> {
        use std::any::Any;
        let driver = self.motor_driver.lock();

        if let Some(gd32) = (&**driver as &dyn Any).downcast_ref::<Gd32Driver>() {
            gd32.get_error_code()
        } else {
            None
        }
    }

    /// Get communication statistics with GD32
    ///
    /// Returns `(received_packets, transmitted_packets, success_rate)`.
    /// Success rate is `received / (transmitted * 2)` as GD32 sends ~2x packets.
    ///
    /// # Example
    /// ```no_run
    /// # use sangam_io::SangamIO;
    /// # let sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;
    /// let (rx, tx, rate) = sangam.get_connection_quality();
    /// println!("GD32 connection: RX={}, TX={}, rate={:.1}%", rx, tx, rate * 100.0);
    /// # Ok::<(), sangam_io::Error>(())
    /// ```
    pub fn get_connection_quality(&self) -> (u64, u64, f32) {
        use std::any::Any;
        let driver = self.motor_driver.lock();

        if let Some(gd32) = (&**driver as &dyn Any).downcast_ref::<Gd32Driver>() {
            let (rx, tx, _lost) = gd32.get_packet_stats();
            let success_rate = if tx > 0 {
                rx as f32 / (tx as f32 * 2.0)  // GD32 sends ~2x as many packets as we do
            } else {
                0.0
            };
            (rx, tx, success_rate)
        } else {
            (0, 0, 0.0)
        }
    }

    /// Check if GD32 telemetry is fresh (received within last 100ms)
    ///
    /// Returns `false` if no telemetry has been received or if data is stale.
    /// Use this to verify GD32 connection health before issuing commands.
    pub fn is_telemetry_fresh(&self) -> bool {
        use std::any::Any;
        use std::time::Duration;

        let driver = self.motor_driver.lock();

        if let Some(gd32) = (&**driver as &dyn Any).downcast_ref::<Gd32Driver>() {
            gd32.is_telemetry_fresh(Duration::from_millis(100))
        } else {
            false
        }
    }

    // === Control Loop ===

    /// Start the control loop thread
    fn start_control_loop(
        motion: Arc<Mutex<MotionController>>,
        odometry: Arc<Mutex<OdometryTracker>>,
        motor_driver: Arc<Mutex<Box<dyn MotorDriver>>>,
        shutdown: Arc<AtomicBool>,
        _last_telemetry: Arc<Mutex<Instant>>,
        config: SangamConfig,
    ) -> JoinHandle<()> {
        thread::Builder::new()
            .name("sangam-control".to_string())
            .spawn(move || {
                log::info!(
                    "SangamIO: Control loop started at {}Hz",
                    config.control_frequency
                );
                let mut last_update = Instant::now();
                let mut last_log = Instant::now();
                let mut overrun_count = 0;
                let mut lock_fail_count = 0;
                let mut first_iteration = true;

                while !shutdown.load(Ordering::Relaxed) {
                    let loop_start = Instant::now();
                    // Use control period for first iteration, actual elapsed time after
                    let dt = if first_iteration {
                        first_iteration = false;
                        config.control_period.as_secs_f32()
                    } else {
                        last_update.elapsed().as_secs_f32()
                    };

                    // Get odometry for motion controller
                    let odom_delta = {
                        let odom = odometry.lock();
                        Some(odom.peek())
                    };

                    // Update motion controller
                    let (left_speed, right_speed) = {
                        let mut motion = motion.lock();
                        motion.update(dt, odom_delta.as_ref())
                    };

                    // Send to motors and update odometry
                    if let Some(mut driver) = motor_driver.try_lock() {
                        lock_fail_count = 0;  // Reset counter on success

                        // Pass motor speeds directly to driver (already in -2000 to 2000 range)
                        // The GD32 driver expects raw motor values and handles the conversion
                        let left_vel = left_speed as f32;
                        let right_vel = right_speed as f32;

                        if let Err(e) = driver.set_wheel_velocity(left_vel, right_vel) {
                            log::warn!("SangamIO: Failed to set wheel velocities: {}", e);
                        }

                        // Update odometry from encoder counts
                        if let Ok(odom_data) = driver.get_odometry() {
                            let mut odom = odometry.lock();
                            odom.update_encoders(odom_data.encoder_left, odom_data.encoder_right);
                            let last_telemetry = _last_telemetry.lock();
                            let telemetry_age = last_telemetry.elapsed();
                            drop(last_telemetry);
                            *_last_telemetry.lock() = Instant::now();

                            // Warn if telemetry is stale
                            if telemetry_age > Duration::from_millis(500) {
                                log::warn!("SangamIO: Telemetry stale - {:.1}s since last update",
                                    telemetry_age.as_secs_f32());
                            }
                        }

                        // Log control loop stats periodically (throttled to 1Hz)
                        if last_log.elapsed() >= Duration::from_secs(1) {
                            log::debug!("SangamIO: Control loop - dt={:.3}ms, motor=(L:{}, R:{})",
                                dt * 1000.0, left_speed, right_speed);
                            last_log = Instant::now();
                        }
                    } else {
                        lock_fail_count += 1;
                        if lock_fail_count >= 10 {
                            log::warn!("SangamIO: Motor driver lock contention - {} consecutive failures",
                                lock_fail_count);
                            lock_fail_count = 0;  // Reset to avoid spam
                        }
                    }

                    // Check for control loop overruns
                    let loop_duration = loop_start.elapsed();
                    if loop_duration > config.control_period {
                        overrun_count += 1;
                        if overrun_count % 10 == 1 {  // Log every 10th overrun
                            log::warn!("SangamIO: Control loop overrun - {:.1}ms (target: {:.1}ms), {} overruns",
                                loop_duration.as_secs_f32() * 1000.0,
                                config.control_period.as_secs_f32() * 1000.0,
                                overrun_count
                            );
                        }
                    }

                    // Maintain control frequency
                    thread::sleep(config.control_period.saturating_sub(loop_duration));
                    last_update = loop_start;
                }

                log::info!("SangamIO: Control loop stopped");
            })
            .expect("Failed to spawn control thread")
    }
}

impl Drop for SangamIO {
    fn drop(&mut self) {
        log::info!("SangamIO: Shutting down");

        // Signal shutdown
        self.shutdown.store(true, Ordering::Relaxed);

        // Stop lidar if active
        if self.is_lidar_active()
            && let Err(e) = self.stop_lidar()
        {
            log::error!("SangamIO: Failed to stop lidar: {}", e);
        }

        // Stop motors
        if let Err(e) = self.emergency_stop() {
            log::error!("SangamIO: Failed to stop motors: {}", e);
        }

        // Wait for control thread
        if let Some(thread) = self.control_thread.take()
            && let Err(e) = thread.join()
        {
            log::error!("SangamIO: Control thread panicked: {:?}", e);
        }

        log::info!("SangamIO: Shutdown complete");
    }
}
