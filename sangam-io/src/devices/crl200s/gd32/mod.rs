//! GD32 motor controller driver

pub mod protocol;

use crate::core::types::{Command, SensorGroupData, SensorValue};
use crate::devices::crl200s::constants::*;
use crate::error::{Error, Result};
use protocol::{
    cmd_air_pump, cmd_heartbeat, cmd_initialize, cmd_main_brush, cmd_motor_speed,
    cmd_motor_velocity, cmd_side_brush, cmd_version_request, PacketReader,
};
use serialport::SerialPort;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// GD32 driver managing heartbeat and sensor reading
pub struct GD32Driver {
    port: Arc<Mutex<Box<dyn SerialPort>>>,
    heartbeat_interval_ms: u64,
    shutdown: Arc<AtomicBool>,
    heartbeat_handle: Option<JoinHandle<()>>,
    reader_handle: Option<JoinHandle<()>>,
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
                let mut port = self.port.lock().unwrap();
                if let Err(e) = port.write_all(&init_bytes) {
                    log::warn!("Init send failed (attempt {}): {}", attempt, e);
                }
            }
            thread::sleep(Duration::from_millis(INIT_RETRY_DELAY_MS));
        }

        log::info!("GD32 initialization sequence sent");
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

        // Start heartbeat thread
        let heartbeat_shutdown = Arc::clone(&shutdown);
        let heartbeat_port = Arc::clone(&port);
        self.heartbeat_handle = Some(
            thread::Builder::new()
                .name("gd32-heartbeat".to_string())
                .spawn(move || {
                    Self::heartbeat_loop(heartbeat_port, heartbeat_shutdown, interval_ms);
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

    /// Heartbeat loop - sends CMD=0x06 at configured interval
    fn heartbeat_loop(
        port: Arc<Mutex<Box<dyn SerialPort>>>,
        shutdown: Arc<AtomicBool>,
        interval_ms: u64,
    ) {
        let heartbeat_pkt = cmd_heartbeat();
        let heartbeat_bytes = heartbeat_pkt.to_bytes();

        while !shutdown.load(Ordering::Relaxed) {
            // Use try_lock to avoid blocking on reader thread
            if let Ok(mut port) = port.try_lock() {
                if let Err(e) = port.write_all(&heartbeat_bytes) {
                    log::error!("Heartbeat send failed: {}", e);
                } else {
                    log::trace!("Heartbeat sent");
                }
            } else {
                log::trace!("Heartbeat skipped (port busy)");
            }

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
                let mut port = port.lock().unwrap();
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
                        let mut port = port.lock().unwrap();
                        if let Err(e) = port.write_all(&version_pkt.to_bytes()) {
                            log::error!("Failed to send version request: {}", e);
                        }
                    }

                    // Handle version response
                    if packet.cmd == CMD_VERSION && !version_received {
                        if let Some(ref vdata) = version_data {
                            if !packet.payload.is_empty() {
                                let mut data = vdata.lock().unwrap();
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
                        let mut data = sensor_data.lock().unwrap();
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
                match id.as_str() {
                    "vacuum" => cmd_air_pump(speed),
                    "brush" => cmd_main_brush(speed),
                    "side_brush" => cmd_side_brush(speed),
                    _ => {
                        log::warn!("Unknown actuator: {}", id);
                        return Ok(());
                    }
                }
            }
            Command::SetActuatorMultiple { ref actuators } => {
                // Send multiple actuator commands
                for (id, value) in actuators {
                    let speed = (value.clamp(0.0, 100.0)) as u8;
                    let pkt = match id.as_str() {
                        "vacuum" => cmd_air_pump(speed),
                        "brush" => cmd_main_brush(speed),
                        "side_brush" => cmd_side_brush(speed),
                        _ => {
                            log::warn!("Unknown actuator: {}", id);
                            continue;
                        }
                    };
                    let mut port = self.port.lock().unwrap();
                    port.write_all(&pkt.to_bytes()).map_err(Error::Io)?;
                }
                return Ok(());
            }

            // Sensor Configuration
            Command::SetSensorConfig { ref sensor_id, .. } => {
                // TODO: Implement sensor configuration (e.g., IMU calibration)
                log::info!("SetSensorConfig for {} - not yet implemented", sensor_id);
                return Ok(());
            }
            Command::ResetSensor { ref sensor_id } => {
                // TODO: Implement sensor reset
                log::info!("ResetSensor {} - not yet implemented", sensor_id);
                return Ok(());
            }
            Command::EnableSensor { ref sensor_id } => {
                // TODO: Implement sensor enable
                log::info!("EnableSensor {} - not yet implemented", sensor_id);
                return Ok(());
            }
            Command::DisableSensor { ref sensor_id } => {
                // TODO: Implement sensor disable
                log::info!("DisableSensor {} - not yet implemented", sensor_id);
                return Ok(());
            }

            // Safety Configuration
            Command::SetSafetyLimits {
                max_linear,
                max_angular,
            } => {
                // TODO: Implement safety limits (may need GD32 firmware support)
                log::info!(
                    "SetSafetyLimits linear={:?}, angular={:?} - not yet implemented",
                    max_linear,
                    max_angular
                );
                return Ok(());
            }
            Command::ClearEmergencyStop => {
                // TODO: Implement clear emergency stop
                log::info!("ClearEmergencyStop - not yet implemented");
                return Ok(());
            }

            // System Lifecycle
            Command::Sleep => {
                // TODO: Implement sleep mode (stop lidar, reduce polling)
                log::info!("Sleep mode - not yet implemented");
                return Ok(());
            }
            Command::Wake => {
                // TODO: Implement wake mode
                log::info!("Wake mode - not yet implemented");
                return Ok(());
            }
            Command::Shutdown => {
                self.shutdown.store(true, Ordering::Relaxed);
                return Ok(());
            }
            Command::Restart => {
                // TODO: Implement restart (re-initialization)
                log::info!("Restart - not yet implemented");
                return Ok(());
            }
        };

        let bytes = packet.to_bytes();
        let mut port = self.port.lock().unwrap();
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
        let mut port = self.port.lock().unwrap();
        let _ = port.write_all(&stop_pkt.to_bytes());

        log::info!("GD32 driver shutdown complete");
        Ok(())
    }
}

impl Drop for GD32Driver {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}
