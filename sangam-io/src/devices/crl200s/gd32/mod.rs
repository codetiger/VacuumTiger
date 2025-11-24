//! GD32 motor controller driver

pub mod protocol;

use crate::core::types::{Command, SensorGroupData, SensorValue};
use crate::devices::crl200s::constants::{
    CMD_STATUS, CMD_VERSION, FLAG_BATTERY_CONNECTED, FLAG_BUMPER_LEFT, FLAG_BUMPER_RIGHT,
    FLAG_CHARGING, FLAG_CLIFF_LEFT_FRONT, FLAG_CLIFF_LEFT_SIDE, FLAG_CLIFF_RIGHT_FRONT,
    FLAG_CLIFF_RIGHT_SIDE, FLAG_DUSTBOX_ATTACHED, INIT_RETRY_DELAY_MS, OFFSET_BUMPER_FLAGS, 
    OFFSET_CHARGING_FLAGS, OFFSET_CLIFF_FLAGS, OFFSET_DOCK_BUTTON, OFFSET_DUSTBOX_FLAGS, 
    OFFSET_START_BUTTON, OFFSET_WHEEL_LEFT_ENCODER, OFFSET_WHEEL_RIGHT_ENCODER, 
    SERIAL_READ_TIMEOUT_MS, STATUS_PAYLOAD_MIN_SIZE,
};
use crate::error::{Error, Result};
use protocol::{
    cmd_air_pump, cmd_heartbeat, cmd_initialize, cmd_lidar_power, cmd_lidar_prep, cmd_lidar_pwm,
    cmd_main_brush, cmd_motor_mode, cmd_motor_speed, cmd_motor_velocity, cmd_side_brush,
    cmd_version_request, PacketReader,
};
use serialport::SerialPort;
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicI16, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// Shared actuator state for periodic refresh
#[derive(Default)]
pub struct ActuatorState {
    pub vacuum: AtomicU8,
    pub main_brush: AtomicU8,
    pub side_brush: AtomicU8,
    pub motor_mode_set: AtomicBool, // Track if motor mode 0x02 has been sent
    pub lidar_enabled: AtomicBool,
    pub lidar_pwm: AtomicU8,
    pub linear_velocity: AtomicI16,
    pub angular_velocity: AtomicI16,
}

/// GD32 driver managing heartbeat and sensor reading
pub struct GD32Driver {
    port: Arc<Mutex<Box<dyn SerialPort>>>,
    heartbeat_interval_ms: u64,
    shutdown: Arc<AtomicBool>,
    heartbeat_handle: Option<JoinHandle<()>>,
    reader_handle: Option<JoinHandle<()>>,
    actuator_state: Arc<ActuatorState>,
}

impl GD32Driver {
    /// Create a new GD32 driver
    pub fn new(port_path: &str, heartbeat_interval_ms: u64) -> Result<Self> {
        let port = serialport::new(port_path, 115200)
            .timeout(Duration::from_millis(SERIAL_READ_TIMEOUT_MS))
            .open()
            .map_err(Error::Serial)?;

        Ok(Self {
            port: Arc::new(Mutex::new(port)),
            heartbeat_interval_ms,
            shutdown: Arc::new(AtomicBool::new(false)),
            heartbeat_handle: None,
            reader_handle: None,
            actuator_state: Arc::new(ActuatorState::default()),
        })
    }

    /// Initialize the GD32 device by sending init commands
    /// Note: Version is requested by reader thread after first packet received
    pub fn initialize(&mut self) -> Result<()> {
        log::info!("Initializing GD32 device...");

        // Send init commands (like sangam-io2-backup approach)
        // Don't wait for response here - reader thread will handle it
        let init_pkt = cmd_initialize();
        let init_bytes = init_pkt.to_bytes();

        for attempt in 1..=5 {
            {
                let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;
                if let Err(e) = port.write_all(&init_bytes) {
                    log::warn!("Init send failed (attempt {}): {}", attempt, e);
                }
            }
            thread::sleep(Duration::from_millis(INIT_RETRY_DELAY_MS));
        }

        // Initialize lidar to off state
        {
            let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;
            let power_bytes = cmd_lidar_power(false).to_bytes();
            let _ = port.write_all(&power_bytes);
            let pwm_bytes = cmd_lidar_pwm(0).to_bytes();
            let _ = port.write_all(&pwm_bytes);
        }

        log::info!("GD32 initialization sequence sent");
        Ok(())
    }

    /// Enable or disable the lidar motor via GD32
    pub fn enable_lidar(&self, enable: bool, pwm_speed: i32) -> Result<()> {
        let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;

        if enable {
            log::info!("Enabling lidar (PWM: {}%)", pwm_speed);

            // Send prep command
            let prep_bytes = cmd_lidar_prep().to_bytes();
            log::debug!("Sending lidar prep: {:02X?}", prep_bytes);
            port.write_all(&prep_bytes).map_err(Error::Io)?;
            drop(port);

            // Send power on
            let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;
            let power_bytes = cmd_lidar_power(true).to_bytes();
            log::debug!("Sending lidar power on: {:02X?}", power_bytes);
            port.write_all(&power_bytes).map_err(Error::Io)?;
            drop(port);

            // Set PWM speed
            let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;
            let pwm_bytes = cmd_lidar_pwm(pwm_speed).to_bytes();
            log::debug!("Sending lidar PWM: {:02X?}", pwm_bytes);
            port.write_all(&pwm_bytes).map_err(Error::Io)?;

            // Store lidar state for heartbeat to send continuously
            self.actuator_state.lidar_enabled.store(true, Ordering::Relaxed);
            self.actuator_state.lidar_pwm.store(pwm_speed as u8, Ordering::Relaxed);

            log::info!("Lidar enabled");
        } else {
            log::info!("Disabling lidar");

            // Clear lidar state
            self.actuator_state.lidar_enabled.store(false, Ordering::Relaxed);
            self.actuator_state.lidar_pwm.store(0, Ordering::Relaxed);

            // Set PWM to 0
            let pwm_bytes = cmd_lidar_pwm(0).to_bytes();
            port.write_all(&pwm_bytes).map_err(Error::Io)?;
            drop(port);

            // Power off
            let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;
            let power_bytes = cmd_lidar_power(false).to_bytes();
            port.write_all(&power_bytes).map_err(Error::Io)?;

            log::info!("Lidar disabled");
        }

        Ok(())
    }

    /// Start heartbeat and reader threads
    pub fn start(
        &mut self,
        sensor_data: Arc<Mutex<SensorGroupData>>,
        version_data: Option<Arc<Mutex<SensorGroupData>>>,
    ) -> Result<()> {
        let shutdown = Arc::clone(&self.shutdown);
        let port = Arc::clone(&self.port);
        let interval_ms = self.heartbeat_interval_ms;
        let actuator_state = Arc::clone(&self.actuator_state);

        // Start heartbeat thread
        let heartbeat_shutdown = Arc::clone(&shutdown);
        let heartbeat_port = Arc::clone(&port);
        let heartbeat_actuators = Arc::clone(&actuator_state);
        self.heartbeat_handle = Some(
            thread::Builder::new()
                .name("gd32-heartbeat".to_string())
                .spawn(move || {
                    Self::heartbeat_loop(heartbeat_port, heartbeat_shutdown, interval_ms, heartbeat_actuators);
                })
                .map_err(|e| Error::Other(format!("Failed to spawn heartbeat thread: {}", e)))?,
        );

        // Start reader thread
        let reader_shutdown = Arc::clone(&shutdown);
        let reader_port = Arc::clone(&port);
        self.reader_handle = Some(
            thread::Builder::new()
                .name("gd32-reader".to_string())
                .spawn(move || {
                    Self::reader_loop(reader_port, reader_shutdown, sensor_data, version_data);
                })
                .map_err(|e| Error::Other(format!("Failed to spawn reader thread: {}", e)))?,
        );

        log::info!("GD32 driver started");
        Ok(())
    }

    /// Heartbeat loop - sends appropriate commands at configured interval
    /// When motor mode 0x02 is active: sends velocity (0x66) + actuator commands
    /// When lidar enabled: sends PWM (0x71)
    /// Otherwise: sends regular heartbeat (0x06)
    fn heartbeat_loop(
        port: Arc<Mutex<Box<dyn SerialPort>>>,
        shutdown: Arc<AtomicBool>,
        interval_ms: u64,
        actuator_state: Arc<ActuatorState>,
    ) {
        let heartbeat_pkt = cmd_heartbeat();
        let heartbeat_bytes = heartbeat_pkt.to_bytes();

        while !shutdown.load(Ordering::Relaxed) {
            // Use blocking lock to ensure commands are always sent
            let Ok(mut port) = port.lock() else {
                log::error!("Heartbeat: mutex poisoned, exiting");
                break;
            };

            // Check actuator states
            let vacuum = actuator_state.vacuum.load(Ordering::Relaxed);
            let main_brush = actuator_state.main_brush.load(Ordering::Relaxed);
            let side_brush = actuator_state.side_brush.load(Ordering::Relaxed);
            let lidar_enabled = actuator_state.lidar_enabled.load(Ordering::Relaxed);

            let any_actuator_active = vacuum > 0 || main_brush > 0 || side_brush > 0 || lidar_enabled;

            // Send motor mode 0x02 when first actuator is enabled
            if any_actuator_active && !actuator_state.motor_mode_set.load(Ordering::Relaxed) {
                let pkt = cmd_motor_mode(0x02);
                if port.write_all(&pkt.to_bytes()).is_ok() {
                    actuator_state.motor_mode_set.store(true, Ordering::Relaxed);
                    log::info!("Motor mode set to navigation (0x02)");
                    // Wait for GD32 to process mode switch before sending actuator commands
                    drop(port);
                    thread::sleep(Duration::from_millis(100));
                    continue; // Skip commands this cycle, send them next cycle
                }
            } else if !any_actuator_active && actuator_state.motor_mode_set.load(Ordering::Relaxed) {
                // Reset flag when all actuators are off
                actuator_state.motor_mode_set.store(false, Ordering::Relaxed);
            }

            if actuator_state.motor_mode_set.load(Ordering::Relaxed) {
                // Motor mode 0x02 active - send velocity command as heartbeat
                let linear = actuator_state.linear_velocity.load(Ordering::Relaxed);
                let angular = actuator_state.angular_velocity.load(Ordering::Relaxed);
                let pkt = cmd_motor_velocity(linear, angular);
                if let Err(e) = port.write_all(&pkt.to_bytes()) {
                    log::error!("Velocity heartbeat send failed: {}", e);
                } else {
                    log::trace!("Velocity heartbeat sent");
                }

                // Send actuator commands every cycle
                if vacuum > 0 {
                    let pkt = cmd_air_pump(vacuum);
                    let _ = port.write_all(&pkt.to_bytes());
                }

                if main_brush > 0 {
                    let pkt = cmd_main_brush(main_brush);
                    let _ = port.write_all(&pkt.to_bytes());
                }

                if side_brush > 0 {
                    let pkt = cmd_side_brush(side_brush);
                    let _ = port.write_all(&pkt.to_bytes());
                }

                // Send lidar PWM if enabled
                if lidar_enabled {
                    let lidar_pwm = actuator_state.lidar_pwm.load(Ordering::Relaxed);
                    let pkt = cmd_lidar_pwm(lidar_pwm as i32);
                    let _ = port.write_all(&pkt.to_bytes());
                }
            } else {
                // No actuators active - send regular heartbeat
                if let Err(e) = port.write_all(&heartbeat_bytes) {
                    log::error!("Heartbeat send failed: {}", e);
                } else {
                    log::trace!("Heartbeat sent");
                }
            }

            drop(port); // Release lock before sleeping
            thread::sleep(Duration::from_millis(interval_ms));
        }

        log::info!("Heartbeat thread exiting");
    }

    /// Reader loop - reads packets and updates shared data directly
    fn reader_loop(
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
                        if let Some(ref vdata) = version_data {
                            if !packet.payload.is_empty() {
                                let Ok(mut data) = vdata.lock() else {
                                    log::error!("Failed to lock version data");
                                    continue;
                                };
                                data.touch();

                                // Parse version string
                                let version_string = if packet.payload[0] < 128 {
                                    let len = packet.payload[0] as usize;
                                    if packet.payload.len() > len {
                                        String::from_utf8_lossy(&packet.payload[1..=len])
                                            .to_string()
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
                                version_received = true;
                            }
                        }
                    }

                    // Handle sensor status data
                    if packet.cmd == CMD_STATUS && packet.payload.len() >= STATUS_PAYLOAD_MIN_SIZE {
                        // Update shared data directly (no allocations)
                        let Ok(mut data) = sensor_data.lock() else {
                            log::error!("Failed to lock sensor data");
                            continue;
                        };
                        let payload = &packet.payload;

                        // Update timestamp
                        data.touch();

                        // Charging/battery status
                        data.update(
                            "is_charging",
                            SensorValue::Bool(
                                (payload[OFFSET_CHARGING_FLAGS] & FLAG_CHARGING) != 0,
                            ),
                        );
                        data.update(
                            "is_battery_connected",
                            SensorValue::Bool(
                                (payload[OFFSET_CHARGING_FLAGS] & FLAG_BATTERY_CONNECTED) != 0,
                            ),
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
                            SensorValue::Bool(
                                (payload[OFFSET_BUMPER_FLAGS] & FLAG_BUMPER_LEFT) != 0,
                            ),
                        );
                        data.update(
                            "bumper_right",
                            SensorValue::Bool(
                                (payload[OFFSET_BUMPER_FLAGS] & FLAG_BUMPER_RIGHT) != 0,
                            ),
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
                            SensorValue::Bool(
                                (payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_LEFT_SIDE) != 0,
                            ),
                        );
                        data.update(
                            "cliff_left_front",
                            SensorValue::Bool(
                                (payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_LEFT_FRONT) != 0,
                            ),
                        );
                        data.update(
                            "cliff_right_front",
                            SensorValue::Bool(
                                (payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_RIGHT_FRONT) != 0,
                            ),
                        );
                        data.update(
                            "cliff_right_side",
                            SensorValue::Bool(
                                (payload[OFFSET_CLIFF_FLAGS] & FLAG_CLIFF_RIGHT_SIDE) != 0,
                            ),
                        );

                        // Dustbox
                        data.update(
                            "dustbox_attached",
                            SensorValue::Bool(
                                (payload[OFFSET_DUSTBOX_FLAGS] & FLAG_DUSTBOX_ATTACHED) != 0,
                            ),
                        );
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

    /// Send a command to the GD32
    pub fn send_command(&self, cmd: Command) -> Result<()> {
        let packet = match cmd {
            // Motion Control
            Command::SetVelocity { linear, angular } => {
                // Convert m/s and rad/s to motor units
                let linear_units = (linear * 1000.0) as i16;
                let angular_units = (angular * 1000.0) as i16;
                // Store velocity for heartbeat to send continuously
                self.actuator_state.linear_velocity.store(linear_units, Ordering::Relaxed);
                self.actuator_state.angular_velocity.store(angular_units, Ordering::Relaxed);
                cmd_motor_velocity(linear_units, angular_units)
            }
            Command::SetTankDrive { left, right } => {
                let left_units = (left * 1000.0) as i16;
                let right_units = (right * 1000.0) as i16;
                cmd_motor_speed(left_units, right_units)
            }
            Command::Stop => cmd_motor_velocity(0, 0),
            Command::EmergencyStop => {
                // TODO: Implement actual emergency stop (may need multiple commands)
                cmd_motor_velocity(0, 0)
            }

            // Actuator Control
            Command::SetActuator { ref id, value } => {
                let speed = (value.clamp(0.0, 100.0)) as u8;
                let pkt = match id.as_str() {
                    "vacuum" => {
                        self.actuator_state.vacuum.store(speed, Ordering::Relaxed);
                        cmd_air_pump(speed)
                    }
                    "brush" => {
                        self.actuator_state.main_brush.store(speed, Ordering::Relaxed);
                        cmd_main_brush(speed)
                    }
                    "side_brush" => {
                        self.actuator_state.side_brush.store(speed, Ordering::Relaxed);
                        cmd_side_brush(speed)
                    }
                    _ => {
                        log::warn!("Unknown actuator: {}", id);
                        return Ok(());
                    }
                };
                let bytes = pkt.to_bytes();
                log::info!("SetActuator: {} = {}, bytes: {:02X?}", id, speed, bytes);
                let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;
                port.write_all(&bytes).map_err(Error::Io)?;
                return Ok(());
            }
            Command::SetActuatorMultiple { ref actuators } => {
                // Send multiple actuator commands
                for (id, value) in actuators {
                    let speed = (value.clamp(0.0, 100.0)) as u8;
                    let pkt = match id.as_str() {
                        "vacuum" => {
                            self.actuator_state.vacuum.store(speed, Ordering::Relaxed);
                            cmd_air_pump(speed)
                        }
                        "brush" => {
                            self.actuator_state.main_brush.store(speed, Ordering::Relaxed);
                            cmd_main_brush(speed)
                        }
                        "side_brush" => {
                            self.actuator_state.side_brush.store(speed, Ordering::Relaxed);
                            cmd_side_brush(speed)
                        }
                        _ => {
                            log::warn!("Unknown actuator: {}", id);
                            continue;
                        }
                    };
                    let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;
                    port.write_all(&pkt.to_bytes()).map_err(Error::Io)?;
                }
                return Ok(());
            }

            // Sensor Configuration
            Command::SetSensorConfig { ref sensor_id, .. } => {
                return Err(Error::NotImplemented(format!("SetSensorConfig for {}", sensor_id)));
            }
            Command::ResetSensor { ref sensor_id } => {
                return Err(Error::NotImplemented(format!("ResetSensor {}", sensor_id)));
            }
            Command::EnableSensor { ref sensor_id } => {
                return Err(Error::NotImplemented(format!("EnableSensor {}", sensor_id)));
            }
            Command::DisableSensor { ref sensor_id } => {
                return Err(Error::NotImplemented(format!("DisableSensor {}", sensor_id)));
            }

            // Safety Configuration
            Command::SetSafetyLimits { .. } => {
                return Err(Error::NotImplemented("SetSafetyLimits".to_string()));
            }
            Command::ClearEmergencyStop => {
                return Err(Error::NotImplemented("ClearEmergencyStop".to_string()));
            }

            // System Lifecycle
            Command::Sleep => {
                return Err(Error::NotImplemented("Sleep".to_string()));
            }
            Command::Wake => {
                return Err(Error::NotImplemented("Wake".to_string()));
            }
            Command::Shutdown => {
                self.shutdown.store(true, Ordering::Relaxed);
                return Ok(());
            }
            Command::Restart => {
                return Err(Error::NotImplemented("Restart".to_string()));
            }
        };

        let bytes = packet.to_bytes();
        let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;
        port.write_all(&bytes).map_err(Error::Io)?;

        Ok(())
    }

    /// Shutdown the driver
    pub fn shutdown(&mut self) -> Result<()> {
        log::info!("Shutting down GD32 driver...");
        self.shutdown.store(true, Ordering::Relaxed);

        // Wait for threads to finish
        if let Some(handle) = self.heartbeat_handle.take() {
            handle.join().map_err(|_| Error::ThreadPanic)?;
        }
        if let Some(handle) = self.reader_handle.take() {
            handle.join().map_err(|_| Error::ThreadPanic)?;
        }

        // Send stop command
        let stop_pkt = cmd_motor_velocity(0, 0);
        if let Ok(mut port) = self.port.lock() {
            let _ = port.write_all(&stop_pkt.to_bytes());
        }

        log::info!("GD32 driver shutdown complete");
        Ok(())
    }
}

impl Drop for GD32Driver {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}
