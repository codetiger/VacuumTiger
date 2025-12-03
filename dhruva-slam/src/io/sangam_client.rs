//! SangamIO TCP client for receiving sensor data.
//!
//! Connects to SangamIO daemon and receives sensor messages.
//! Supports both JSON and Postcard wire formats.
//!
//! # Wire Protocol
//!
//! ```text
//! ┌──────────────────┬──────────────────────────┐
//! │ Length (4 bytes) │ Payload (variable)       │
//! │ Big-endian u32   │ JSON or Postcard binary  │
//! └──────────────────┴──────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::sangam_client::{SangamClient, WireFormat};
//! use std::time::Duration;
//!
//! // Connect with JSON format (default for SangamIO)
//! let mut client = SangamClient::connect_with_format(
//!     "192.168.68.101:5555",
//!     WireFormat::Json
//! )?;
//! client.set_timeout(Some(Duration::from_secs(5)))?;
//!
//! loop {
//!     let msg = client.recv()?;
//!     if let Some(lidar) = msg.as_lidar() {
//!         println!("Got {} lidar points", lidar.data.len());
//!     }
//! }
//! ```

use crate::core::types::Timestamped;
use serde::Deserialize;
use std::collections::HashMap;
use std::io::Read;
use std::net::TcpStream;
use std::time::Duration;
use thiserror::Error;

/// Client errors
#[derive(Error, Debug)]
pub enum ClientError {
    #[error("Connection failed: {0}")]
    Connection(#[from] std::io::Error),

    #[error("Deserialization failed: {0}")]
    Deserialize(String),

    #[error("Connection closed")]
    Disconnected,

    #[error("Buffer too small for message (need {0} bytes)")]
    BufferTooSmall(usize),
}

impl From<postcard::Error> for ClientError {
    fn from(e: postcard::Error) -> Self {
        ClientError::Deserialize(e.to_string())
    }
}

impl From<serde_json::Error> for ClientError {
    fn from(e: serde_json::Error) -> Self {
        ClientError::Deserialize(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, ClientError>;

/// Supported wire formats for SangamIO communication.
///
/// Must match the `wire_format` setting in SangamIO's hardware.json.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WireFormat {
    /// JSON format - human-readable, default for SangamIO
    ///
    /// Pros: Easy to debug, widely supported
    /// Cons: Larger messages, slower serialization
    #[default]
    Json,

    /// Postcard binary format - compact and fast
    ///
    /// Pros: Small messages, fast serialization
    /// Cons: Binary format, harder to debug
    Postcard,
}

/// SangamIO message (mirrors sangam-io/src/streaming/messages.rs)
#[derive(Debug, Deserialize)]
pub struct Message {
    pub topic: String,
    pub payload: Payload,
}

/// Message payload (only SensorGroup for now)
#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
pub enum Payload {
    SensorGroup {
        group_id: String,
        timestamp_us: u64,
        values: HashMap<String, SensorValue>,
    },
}

/// Sensor value types (subset of sangam-io types we need)
#[derive(Debug, Clone, Deserialize)]
pub enum SensorValue {
    Bool(bool),
    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),
    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),
    F32(f32),
    F64(f64),
    String(String),
    Bytes(Vec<u8>),
    Vector3([f32; 3]),
    /// LiDAR point cloud: Vec<(angle_rad, distance_m, quality)>
    PointCloud2D(Vec<LidarPoint>),
}

/// A single LiDAR point: (angle_rad, distance_m, quality)
pub type LidarPoint = (f32, f32, u8);

/// A complete LiDAR scan as a vector of points.
pub type LidarScan = Vec<LidarPoint>;

impl Payload {
    /// Get group_id from payload.
    #[inline]
    fn group_id(&self) -> &str {
        let Payload::SensorGroup { group_id, .. } = self;
        group_id
    }

    /// Get timestamp from payload.
    #[inline]
    fn timestamp_us(&self) -> u64 {
        let Payload::SensorGroup { timestamp_us, .. } = self;
        *timestamp_us
    }

    /// Get values from payload.
    #[inline]
    fn values(&self) -> &HashMap<String, SensorValue> {
        let Payload::SensorGroup { values, .. } = self;
        values
    }
}

impl Message {
    /// Extract lidar point cloud if this is a lidar message.
    ///
    /// Returns timestamped reference to LidarScan (Vec of (angle_rad, distance_m, quality)).
    pub fn as_lidar(&self) -> Option<Timestamped<&LidarScan>> {
        if self.payload.group_id() != "lidar" {
            return None;
        }
        if let Some(SensorValue::PointCloud2D(points)) = self.payload.values().get("scan") {
            Some(Timestamped::new(points, self.payload.timestamp_us()))
        } else {
            None
        }
    }

    /// Extract wheel encoder ticks (left, right) if this is a sensor_status message.
    ///
    /// Returns raw U16 tick values.
    pub fn encoder_ticks(&self) -> Option<(u16, u16)> {
        if self.payload.group_id() != "sensor_status" {
            return None;
        }
        let values = self.payload.values();
        let left = match values.get("wheel_left") {
            Some(SensorValue::U16(v)) => *v,
            _ => return None,
        };
        let right = match values.get("wheel_right") {
            Some(SensorValue::U16(v)) => *v,
            _ => return None,
        };
        Some((left, right))
    }

    /// Extract gyroscope Z raw value if this is a sensor_status message.
    ///
    /// **Note**: This returns `gyro_z` which is actually the Roll axis on CRL-200S.
    /// For yaw (heading), use [`gyro_yaw_raw`] instead.
    ///
    /// Returns raw I16 value from IMU. Needs calibration/conversion to rad/s.
    pub fn gyro_z_raw(&self) -> Option<i16> {
        if self.payload.group_id() != "sensor_status" {
            return None;
        }
        if let Some(SensorValue::I16(v)) = self.payload.values().get("gyro_z") {
            Some(*v)
        } else {
            None
        }
    }

    /// Extract gyroscope Yaw rate (for 2D heading) from sensor_status message.
    ///
    /// On CRL-200S, the yaw rate is in `gyro_x` due to axis mapping:
    /// - gyro_x (B40-41) = Yaw rate (Z axis rotation on floor)
    /// - gyro_y (B44-45) = Pitch rate (nose up/down)
    /// - gyro_z (B48-49) = Roll rate (left/right tilt)
    ///
    /// Returns raw I16 value in 0.1 deg/s units.
    pub fn gyro_yaw_raw(&self) -> Option<i16> {
        if self.payload.group_id() != "sensor_status" {
            return None;
        }
        // Yaw rate is in gyro_x on CRL-200S
        if let Some(SensorValue::I16(v)) = self.payload.values().get("gyro_x") {
            Some(*v)
        } else {
            None
        }
    }

    /// Extract full gyroscope raw reading [x, y, z].
    ///
    /// Returns raw I16 values from IMU. Needs calibration/conversion to rad/s.
    pub fn gyro_raw(&self) -> Option<[i16; 3]> {
        if self.payload.group_id() != "sensor_status" {
            return None;
        }
        let values = self.payload.values();
        let x = match values.get("gyro_x") {
            Some(SensorValue::I16(v)) => *v,
            _ => return None,
        };
        let y = match values.get("gyro_y") {
            Some(SensorValue::I16(v)) => *v,
            _ => return None,
        };
        let z = match values.get("gyro_z") {
            Some(SensorValue::I16(v)) => *v,
            _ => return None,
        };
        Some([x, y, z])
    }

    /// Extract accelerometer raw reading [x, y, z].
    ///
    /// Returns raw I16 values from IMU. Needs calibration/conversion to m/s².
    pub fn accel_raw(&self) -> Option<[i16; 3]> {
        if self.payload.group_id() != "sensor_status" {
            return None;
        }
        let values = self.payload.values();
        let x = match values.get("accel_x") {
            Some(SensorValue::I16(v)) => *v,
            _ => return None,
        };
        let y = match values.get("accel_y") {
            Some(SensorValue::I16(v)) => *v,
            _ => return None,
        };
        let z = match values.get("accel_z") {
            Some(SensorValue::I16(v)) => *v,
            _ => return None,
        };
        Some([x, y, z])
    }

    /// Get the group_id of this message.
    #[inline]
    pub fn group_id(&self) -> &str {
        self.payload.group_id()
    }

    /// Get timestamp in microseconds since epoch.
    #[inline]
    pub fn timestamp_us(&self) -> u64 {
        self.payload.timestamp_us()
    }

    /// Get raw access to sensor values.
    #[inline]
    pub fn values(&self) -> &HashMap<String, SensorValue> {
        self.payload.values()
    }
}

/// Default buffer size (64KB should handle most messages)
const DEFAULT_BUFFER_SIZE: usize = 65536;

/// Maximum buffer size (1MB)
const MAX_BUFFER_SIZE: usize = 1024 * 1024;

/// TCP client for SangamIO daemon.
///
/// Supports both JSON and Postcard wire formats. The format must match
/// the `wire_format` setting in SangamIO's hardware.json configuration.
pub struct SangamClient {
    stream: TcpStream,
    buffer: Vec<u8>,
    format: WireFormat,
}

impl SangamClient {
    /// Connect to SangamIO daemon using JSON wire format (default).
    ///
    /// This is a convenience method equivalent to:
    /// ```ignore
    /// SangamClient::connect_with_format(addr, WireFormat::Json)
    /// ```
    ///
    /// # Example
    /// ```ignore
    /// let client = SangamClient::connect("192.168.68.101:5555")?;
    /// ```
    pub fn connect(addr: &str) -> Result<Self> {
        Self::connect_with_format(addr, WireFormat::Json)
    }

    /// Connect to SangamIO daemon with specified wire format.
    ///
    /// The format must match the `wire_format` setting in SangamIO's
    /// hardware.json configuration file.
    ///
    /// # Example
    /// ```ignore
    /// use dhruva_slam::sangam_client::{SangamClient, WireFormat};
    ///
    /// // For hardware.json with "wire_format": "json"
    /// let client = SangamClient::connect_with_format(
    ///     "192.168.68.101:5555",
    ///     WireFormat::Json
    /// )?;
    ///
    /// // For hardware.json with "wire_format": "postcard"
    /// let client = SangamClient::connect_with_format(
    ///     "192.168.68.101:5555",
    ///     WireFormat::Postcard
    /// )?;
    /// ```
    pub fn connect_with_format(addr: &str, format: WireFormat) -> Result<Self> {
        let stream = TcpStream::connect(addr)?;
        Ok(Self {
            stream,
            buffer: vec![0u8; DEFAULT_BUFFER_SIZE],
            format,
        })
    }

    /// Get the current wire format.
    pub fn wire_format(&self) -> WireFormat {
        self.format
    }

    /// Set read timeout for the connection.
    ///
    /// Pass `None` to disable timeout (blocking reads).
    pub fn set_timeout(&mut self, timeout: Option<Duration>) -> Result<()> {
        self.stream.set_read_timeout(timeout)?;
        Ok(())
    }

    /// Get the local address of this connection.
    pub fn local_addr(&self) -> Result<std::net::SocketAddr> {
        Ok(self.stream.local_addr()?)
    }

    /// Get the peer address of this connection.
    pub fn peer_addr(&self) -> Result<std::net::SocketAddr> {
        Ok(self.stream.peer_addr()?)
    }

    /// Deserialize message bytes using the configured wire format.
    fn deserialize(&self, bytes: &[u8]) -> Result<Message> {
        match self.format {
            WireFormat::Json => Ok(serde_json::from_slice(bytes)?),
            WireFormat::Postcard => Ok(postcard::from_bytes(bytes)?),
        }
    }

    /// Receive the next message (blocking).
    ///
    /// Returns deserialized message from SangamIO.
    pub fn recv(&mut self) -> Result<Message> {
        // Read length prefix (4 bytes, big-endian)
        let mut len_buf = [0u8; 4];
        self.stream.read_exact(&mut len_buf)?;
        let len = u32::from_be_bytes(len_buf) as usize;

        // Grow buffer if needed (up to max)
        if len > self.buffer.len() {
            if len > MAX_BUFFER_SIZE {
                return Err(ClientError::BufferTooSmall(len));
            }
            self.buffer.resize(len, 0);
        }

        // Read payload
        self.stream.read_exact(&mut self.buffer[..len])?;

        // Deserialize using configured format
        self.deserialize(&self.buffer[..len])
    }

    /// Receive with timeout, returns None on timeout.
    ///
    /// Temporarily sets timeout, attempts to receive, then restores
    /// previous timeout setting.
    pub fn recv_timeout(&mut self, timeout: Duration) -> Result<Option<Message>> {
        let old_timeout = self.stream.read_timeout()?;
        self.stream.set_read_timeout(Some(timeout))?;

        let result = match self.recv() {
            Ok(msg) => Ok(Some(msg)),
            Err(ClientError::Connection(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {
                Ok(None)
            }
            Err(ClientError::Connection(e)) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(e),
        };

        self.stream.set_read_timeout(old_timeout)?;
        result
    }

    /// Try to receive a message without blocking.
    ///
    /// Returns None if no data is immediately available.
    pub fn try_recv(&mut self) -> Result<Option<Message>> {
        // Use 1ms timeout since 0 duration is not allowed on some platforms
        self.recv_timeout(Duration::from_millis(1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_value_sizes() {
        // Ensure SensorValue variants compile and have expected structure
        let _bool = SensorValue::Bool(true);
        let _u16 = SensorValue::U16(1000);
        let _vec3 = SensorValue::Vector3([1.0, 2.0, 3.0]);
        let _points = SensorValue::PointCloud2D(vec![(0.0, 1.0, 100)]);
    }

    #[test]
    fn test_wire_format_default() {
        assert_eq!(WireFormat::default(), WireFormat::Json);
    }
}
