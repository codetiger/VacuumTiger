//! UDP receiver for SangamIO sensor data.
//!
//! Receives sensor data from SangamIO via UDP unicast and distributes it
//! to processing threads via crossbeam channels.
//!
//! # Wire Protocol
//!
//! Same as TCP for client compatibility:
//! ```text
//! ┌──────────────────┬──────────────────────────┐
//! │ Length (4 bytes) │ Payload (variable)       │
//! │ Big-endian u32   │ Protobuf binary          │
//! └──────────────────┴──────────────────────────┘
//! ```
//!
//! Note: Some fields are defined for planned IMU integration.
//!
//! # Message Types
//!
//! - `sensor_status` @ 110Hz: Encoder ticks, gyro, bumpers, cliffs
//! - `lidar` @ 5Hz: Point cloud scans
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::io::sangam_udp_receiver::{SangamUdpReceiver, ReceiverConfig};
//! use std::sync::Arc;
//! use std::sync::atomic::AtomicBool;
//!
//! let config = ReceiverConfig {
//!     bind_addr: "0.0.0.0:5555".to_string(),
//! };
//! let running = Arc::new(AtomicBool::new(true));
//! let (receiver, sensor_rx, lidar_rx) = SangamUdpReceiver::new(config, running)?;
//!
//! // Spawn receiver thread
//! std::thread::spawn(move || receiver.run());
//!
//! // Process sensor data in another thread
//! while let Ok(msg) = sensor_rx.recv() {
//!     println!("Got sensor data: {:?}", msg.timestamp_us);
//! }
//! ```

use crossbeam_channel::{Receiver, Sender, bounded};
use prost::Message as ProstMessage;
use std::collections::HashMap;
use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use thiserror::Error;

// Re-use proto types from sangam_client
use crate::io::sangam_client::proto::sangamio;

/// Receiver errors
#[derive(Error, Debug)]
pub enum ReceiverError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Decode error: {0}")]
    Decode(String),
}

impl From<prost::DecodeError> for ReceiverError {
    fn from(e: prost::DecodeError) -> Self {
        ReceiverError::Decode(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, ReceiverError>;

/// Configuration for UDP receiver.
#[derive(Debug, Clone)]
pub struct ReceiverConfig {
    /// Address to bind UDP socket (e.g., "0.0.0.0:5555").
    pub bind_addr: String,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            bind_addr: "0.0.0.0:5555".to_string(),
        }
    }
}

/// Sensor status message extracted from protobuf.
#[derive(Debug, Clone)]
pub struct SensorStatusData {
    /// Timestamp in microseconds.
    pub timestamp_us: u64,
    /// Left wheel encoder ticks.
    pub left_encoder: u16,
    /// Right wheel encoder ticks.
    pub right_encoder: u16,
    /// Gyro Z (yaw rate) raw value in 0.1 deg/s.
    pub gyro_yaw_raw: i16,
    /// Left bumper pressed.
    pub bumper_left: bool,
    /// Right bumper pressed.
    pub bumper_right: bool,
    /// Cliff sensor: left side.
    pub cliff_left_side: bool,
    /// Cliff sensor: left front.
    pub cliff_left_front: bool,
    /// Cliff sensor: right front.
    pub cliff_right_front: bool,
    /// Cliff sensor: right side.
    pub cliff_right_side: bool,
}

/// A single LiDAR point: (angle_rad, distance_m, quality).
pub type LidarPoint = (f32, f32, u8);

/// LiDAR scan message.
#[derive(Debug, Clone)]
pub struct LidarData {
    /// Timestamp in microseconds.
    pub timestamp_us: u64,
    /// Point cloud: Vec<(angle_rad, distance_m, quality)>.
    pub points: Vec<LidarPoint>,
}

/// Channel capacity for sensor data (small to avoid buffering old data).
const SENSOR_CHANNEL_CAPACITY: usize = 8;

/// Channel capacity for lidar data.
const LIDAR_CHANNEL_CAPACITY: usize = 4;

/// Maximum UDP datagram size (64KB for fragmented packets).
const MAX_DATAGRAM_SIZE: usize = 65536;

/// UDP receiver for SangamIO sensor data.
pub struct SangamUdpReceiver {
    socket: UdpSocket,
    running: Arc<AtomicBool>,
    sensor_tx: Sender<SensorStatusData>,
    lidar_tx: Sender<LidarData>,
}

impl SangamUdpReceiver {
    /// Create a new UDP receiver.
    ///
    /// Returns the receiver and channels for sensor/lidar data.
    pub fn new(
        config: ReceiverConfig,
        running: Arc<AtomicBool>,
    ) -> Result<(Self, Receiver<SensorStatusData>, Receiver<LidarData>)> {
        let socket = UdpSocket::bind(&config.bind_addr)?;

        // Set non-blocking with short timeout for shutdown checks
        socket.set_read_timeout(Some(Duration::from_millis(100)))?;

        // Create channels for sensor data
        let (sensor_tx, sensor_rx) = bounded(SENSOR_CHANNEL_CAPACITY);
        let (lidar_tx, lidar_rx) = bounded(LIDAR_CHANNEL_CAPACITY);

        log::info!("UDP receiver bound to {}", config.bind_addr);

        Ok((
            Self {
                socket,
                running,
                sensor_tx,
                lidar_tx,
            },
            sensor_rx,
            lidar_rx,
        ))
    }

    /// Run the receiver loop (blocking).
    ///
    /// Receives UDP datagrams, parses them, and sends to appropriate channels.
    pub fn run(self) {
        log::info!("UDP receiver started");

        let mut buffer = vec![0u8; MAX_DATAGRAM_SIZE];

        while self.running.load(Ordering::Relaxed) {
            // Receive UDP datagram
            let (len, _src) = match self.socket.recv_from(&mut buffer) {
                Ok(result) => result,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => {
                    log::error!("UDP recv error: {}", e);
                    continue;
                }
            };

            // Parse length-prefixed message
            if len < 4 {
                log::warn!("UDP datagram too short: {} bytes", len);
                continue;
            }

            let msg_len = u32::from_be_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]) as usize;
            if msg_len + 4 > len {
                log::warn!(
                    "UDP message incomplete: expected {} + 4 bytes, got {}",
                    msg_len,
                    len
                );
                continue;
            }

            // Decode protobuf
            let proto_msg = match sangamio::Message::decode(&buffer[4..4 + msg_len]) {
                Ok(m) => m,
                Err(e) => {
                    log::warn!("Failed to decode protobuf: {}", e);
                    continue;
                }
            };

            // Extract sensor group
            let sensor_group = match proto_msg.payload {
                Some(sangamio::message::Payload::SensorGroup(sg)) => sg,
                _ => continue,
            };

            // Route based on group_id
            match sensor_group.group_id.as_str() {
                "sensor_status" => {
                    if let Some(data) = Self::parse_sensor_status(&sensor_group) {
                        // Non-blocking send - drop if channel full (old data)
                        self.sensor_tx.try_send(data).ok();
                    }
                }
                "lidar" => {
                    if let Some(data) = Self::parse_lidar(&sensor_group) {
                        // Non-blocking send - drop if channel full
                        self.lidar_tx.try_send(data).ok();
                    }
                }
                other => {
                    log::trace!("Ignoring unknown group_id: {}", other);
                }
            }
        }

        log::info!("UDP receiver stopped");
    }

    /// Parse sensor_status group into SensorStatusData.
    fn parse_sensor_status(sg: &sangamio::SensorGroup) -> Option<SensorStatusData> {
        let values = &sg.values;

        let left_encoder = Self::get_u32(values, "wheel_left")? as u16;
        let right_encoder = Self::get_u32(values, "wheel_right")? as u16;
        let gyro_yaw_raw = Self::get_i32(values, "gyro_z").unwrap_or(0) as i16;

        Some(SensorStatusData {
            timestamp_us: sg.timestamp_us,
            left_encoder,
            right_encoder,
            gyro_yaw_raw,
            bumper_left: Self::get_bool(values, "bumper_left").unwrap_or(false),
            bumper_right: Self::get_bool(values, "bumper_right").unwrap_or(false),
            cliff_left_side: Self::get_bool(values, "cliff_left_side").unwrap_or(false),
            cliff_left_front: Self::get_bool(values, "cliff_left_front").unwrap_or(false),
            cliff_right_front: Self::get_bool(values, "cliff_right_front").unwrap_or(false),
            cliff_right_side: Self::get_bool(values, "cliff_right_side").unwrap_or(false),
        })
    }

    /// Parse lidar group into LidarData.
    fn parse_lidar(sg: &sangamio::SensorGroup) -> Option<LidarData> {
        let scan = sg.values.get("scan")?;
        let pointcloud = match &scan.value {
            Some(sangamio::sensor_value::Value::PointcloudVal(pc)) => pc,
            _ => return None,
        };

        let points: Vec<LidarPoint> = pointcloud
            .points
            .iter()
            .map(|p| (p.angle_rad, p.distance_m, p.quality as u8))
            .collect();

        Some(LidarData {
            timestamp_us: sg.timestamp_us,
            points,
        })
    }

    /// Helper to extract u32 from sensor value map.
    fn get_u32(values: &HashMap<String, sangamio::SensorValue>, key: &str) -> Option<u32> {
        match values.get(key)?.value.as_ref()? {
            sangamio::sensor_value::Value::U32Val(v) => Some(*v),
            _ => None,
        }
    }

    /// Helper to extract i32 from sensor value map.
    fn get_i32(values: &HashMap<String, sangamio::SensorValue>, key: &str) -> Option<i32> {
        match values.get(key)?.value.as_ref()? {
            sangamio::sensor_value::Value::I32Val(v) => Some(*v),
            _ => None,
        }
    }

    /// Helper to extract bool from sensor value map.
    fn get_bool(values: &HashMap<String, sangamio::SensorValue>, key: &str) -> Option<bool> {
        match values.get(key)?.value.as_ref()? {
            sangamio::sensor_value::Value::BoolVal(v) => Some(*v),
            _ => None,
        }
    }
}
