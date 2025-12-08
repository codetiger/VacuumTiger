//! SangamIO TCP client for receiving sensor data.
//!
//! Connects to SangamIO daemon and receives sensor messages using Protobuf.
//!
//! # Wire Protocol
//!
//! ```text
//! ┌──────────────────┬──────────────────────────┐
//! │ Length (4 bytes) │ Payload (variable)       │
//! │ Big-endian u32   │ Protobuf binary          │
//! └──────────────────┴──────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::sangam_client::SangamClient;
//! use std::time::Duration;
//!
//! let mut client = SangamClient::connect("192.168.68.101:5555")?;
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
use prost::Message as ProstMessage;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::Duration;
use thiserror::Error;

// Include generated protobuf types
pub mod proto {
    pub mod sangamio {
        include!(concat!(env!("OUT_DIR"), "/sangamio.rs"));
    }
}

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

impl From<prost::DecodeError> for ClientError {
    fn from(e: prost::DecodeError) -> Self {
        ClientError::Deserialize(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, ClientError>;

/// Sensor value types (converted from protobuf)
#[derive(Debug, Clone)]
pub enum SensorValue {
    Bool(bool),
    U32(u32),
    U64(u64),
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

/// Parsed message from SangamIO
#[derive(Debug)]
pub struct Message {
    topic: String,
    group_id: String,
    timestamp_us: u64,
    values: HashMap<String, SensorValue>,
}

impl Message {
    /// Create a new message (used by SimulatedClient)
    pub fn new(
        topic: &str,
        group_id: &str,
        timestamp_us: u64,
        values: HashMap<String, SensorValue>,
    ) -> Self {
        Self {
            topic: topic.to_string(),
            group_id: group_id.to_string(),
            timestamp_us,
            values,
        }
    }

    /// Create from protobuf message
    fn from_proto(msg: proto::sangamio::Message) -> Result<Self> {
        match msg.payload {
            Some(proto::sangamio::message::Payload::SensorGroup(sg)) => {
                let values: HashMap<String, SensorValue> = sg
                    .values
                    .into_iter()
                    .map(|(k, v)| (k, SensorValue::from_proto(v)))
                    .collect();

                Ok(Self {
                    topic: msg.topic,
                    group_id: sg.group_id,
                    timestamp_us: sg.timestamp_us,
                    values,
                })
            }
            Some(proto::sangamio::message::Payload::Command(_)) => Err(ClientError::Deserialize(
                "Received command message from server".to_string(),
            )),
            None => Err(ClientError::Deserialize("Empty payload".to_string())),
        }
    }

    /// Extract lidar point cloud if this is a lidar message.
    ///
    /// Returns timestamped reference to LidarScan (Vec of (angle_rad, distance_m, quality)).
    pub fn as_lidar(&self) -> Option<Timestamped<&LidarScan>> {
        if self.group_id != "lidar" {
            return None;
        }
        if let Some(SensorValue::PointCloud2D(points)) = self.values.get("scan") {
            Some(Timestamped::new(points, self.timestamp_us))
        } else {
            None
        }
    }

    /// Extract wheel encoder ticks (left, right) if this is a sensor_status message.
    ///
    /// Returns raw U16 tick values (stored as U32 in protobuf).
    pub fn encoder_ticks(&self) -> Option<(u16, u16)> {
        if self.group_id != "sensor_status" {
            return None;
        }
        let left = match self.values.get("wheel_left") {
            Some(SensorValue::U32(v)) => *v as u16,
            _ => return None,
        };
        let right = match self.values.get("wheel_right") {
            Some(SensorValue::U32(v)) => *v as u16,
            _ => return None,
        };
        Some((left, right))
    }

    /// Extract gyroscope Z (Yaw) raw value from sensor_status message.
    ///
    /// SangamIO transforms gyro data to ROS REP-103 frame:
    /// - gyro_x = Roll rate (X axis rotation, left/right tilt)
    /// - gyro_y = Pitch rate (Y axis rotation, nose up/down)
    /// - gyro_z = Yaw rate (Z axis rotation, heading change)
    ///
    /// Returns raw I16 value in 0.1 deg/s units (stored as I32 in protobuf).
    pub fn gyro_z_raw(&self) -> Option<i16> {
        if self.group_id != "sensor_status" {
            return None;
        }
        if let Some(SensorValue::I32(v)) = self.values.get("gyro_z") {
            Some(*v as i16)
        } else {
            None
        }
    }

    /// Extract gyroscope Yaw rate (for 2D heading) from sensor_status message.
    ///
    /// SangamIO transforms gyro data to ROS REP-103 frame, so yaw is in `gyro_z`:
    /// - gyro_x = Roll rate (X axis rotation)
    /// - gyro_y = Pitch rate (Y axis rotation)
    /// - gyro_z = Yaw rate (Z axis rotation, CCW positive)
    ///
    /// Returns raw I16 value in 0.1 deg/s units.
    pub fn gyro_yaw_raw(&self) -> Option<i16> {
        if self.group_id != "sensor_status" {
            return None;
        }
        // Yaw rate is now correctly in gyro_z after SangamIO transform
        if let Some(SensorValue::I32(v)) = self.values.get("gyro_z") {
            Some(*v as i16)
        } else {
            None
        }
    }

    /// Extract full gyroscope raw reading [x, y, z] in ROS REP-103 frame.
    ///
    /// Returns raw I16 values from IMU: [roll_rate, pitch_rate, yaw_rate]
    pub fn gyro_raw(&self) -> Option<[i16; 3]> {
        if self.group_id != "sensor_status" {
            return None;
        }
        let x = match self.values.get("gyro_x") {
            Some(SensorValue::I32(v)) => *v as i16,
            _ => return None,
        };
        let y = match self.values.get("gyro_y") {
            Some(SensorValue::I32(v)) => *v as i16,
            _ => return None,
        };
        let z = match self.values.get("gyro_z") {
            Some(SensorValue::I32(v)) => *v as i16,
            _ => return None,
        };
        Some([x, y, z])
    }

    /// Extract accelerometer raw reading [x, y, z].
    ///
    /// Returns raw I16 values from IMU.
    pub fn accel_raw(&self) -> Option<[i16; 3]> {
        if self.group_id != "sensor_status" {
            return None;
        }
        let x = match self.values.get("accel_x") {
            Some(SensorValue::I32(v)) => *v as i16,
            _ => return None,
        };
        let y = match self.values.get("accel_y") {
            Some(SensorValue::I32(v)) => *v as i16,
            _ => return None,
        };
        let z = match self.values.get("accel_z") {
            Some(SensorValue::I32(v)) => *v as i16,
            _ => return None,
        };
        Some([x, y, z])
    }

    /// Extract LP-filtered tilt/gravity vector [x, y, z].
    ///
    /// Returns raw I16 values from IMU. Used for Mahony AHRS gravity correction.
    pub fn tilt_raw(&self) -> Option<[i16; 3]> {
        if self.group_id != "sensor_status" {
            return None;
        }
        let x = match self.values.get("tilt_x") {
            Some(SensorValue::I32(v)) => *v as i16,
            _ => return None,
        };
        let y = match self.values.get("tilt_y") {
            Some(SensorValue::I32(v)) => *v as i16,
            _ => return None,
        };
        let z = match self.values.get("tilt_z") {
            Some(SensorValue::I32(v)) => *v as i16,
            _ => return None,
        };
        Some([x, y, z])
    }

    /// Get the topic of this message (e.g., "sensors/sensor_status").
    #[inline]
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Get the group_id of this message.
    #[inline]
    pub fn group_id(&self) -> &str {
        &self.group_id
    }

    /// Get timestamp in microseconds since epoch.
    #[inline]
    pub fn timestamp_us(&self) -> u64 {
        self.timestamp_us
    }

    /// Get raw access to sensor values.
    #[inline]
    pub fn values(&self) -> &HashMap<String, SensorValue> {
        &self.values
    }
}

impl SensorValue {
    /// Convert from protobuf sensor value
    fn from_proto(value: proto::sangamio::SensorValue) -> Self {
        match value.value {
            Some(proto::sangamio::sensor_value::Value::BoolVal(v)) => SensorValue::Bool(v),
            Some(proto::sangamio::sensor_value::Value::U32Val(v)) => SensorValue::U32(v),
            Some(proto::sangamio::sensor_value::Value::U64Val(v)) => SensorValue::U64(v),
            Some(proto::sangamio::sensor_value::Value::I32Val(v)) => SensorValue::I32(v),
            Some(proto::sangamio::sensor_value::Value::I64Val(v)) => SensorValue::I64(v),
            Some(proto::sangamio::sensor_value::Value::F32Val(v)) => SensorValue::F32(v),
            Some(proto::sangamio::sensor_value::Value::F64Val(v)) => SensorValue::F64(v),
            Some(proto::sangamio::sensor_value::Value::StringVal(v)) => SensorValue::String(v),
            Some(proto::sangamio::sensor_value::Value::BytesVal(v)) => SensorValue::Bytes(v),
            Some(proto::sangamio::sensor_value::Value::Vector3Val(v)) => {
                SensorValue::Vector3([v.x, v.y, v.z])
            }
            Some(proto::sangamio::sensor_value::Value::PointcloudVal(v)) => {
                SensorValue::PointCloud2D(
                    v.points
                        .into_iter()
                        .map(|p| (p.angle_rad, p.distance_m, p.quality as u8))
                        .collect(),
                )
            }
            None => SensorValue::Bool(false), // Default fallback
        }
    }

    /// Convert to protobuf sensor value
    fn to_proto(&self) -> proto::sangamio::SensorValue {
        use proto::sangamio::sensor_value::Value;
        proto::sangamio::SensorValue {
            value: Some(match self {
                SensorValue::Bool(v) => Value::BoolVal(*v),
                SensorValue::U32(v) => Value::U32Val(*v),
                SensorValue::U64(v) => Value::U64Val(*v),
                SensorValue::I32(v) => Value::I32Val(*v),
                SensorValue::I64(v) => Value::I64Val(*v),
                SensorValue::F32(v) => Value::F32Val(*v),
                SensorValue::F64(v) => Value::F64Val(*v),
                SensorValue::String(v) => Value::StringVal(v.clone()),
                SensorValue::Bytes(v) => Value::BytesVal(v.clone()),
                SensorValue::Vector3(v) => Value::Vector3Val(proto::sangamio::Vector3 {
                    x: v[0],
                    y: v[1],
                    z: v[2],
                }),
                SensorValue::PointCloud2D(points) => {
                    Value::PointcloudVal(proto::sangamio::PointCloud2D {
                        points: points
                            .iter()
                            .map(|(angle, dist, quality)| proto::sangamio::LidarPoint {
                                angle_rad: *angle,
                                distance_m: *dist,
                                quality: *quality as u32,
                            })
                            .collect(),
                    })
                }
            }),
        }
    }
}

/// Default buffer size (64KB should handle most messages)
const DEFAULT_BUFFER_SIZE: usize = 65536;

/// Maximum buffer size (1MB)
const MAX_BUFFER_SIZE: usize = 1024 * 1024;

/// TCP client for SangamIO daemon.
///
/// Uses Protobuf wire format for efficient binary communication.
pub struct SangamClient {
    stream: TcpStream,
    buffer: Vec<u8>,
}

impl SangamClient {
    /// Connect to SangamIO daemon.
    ///
    /// # Example
    /// ```ignore
    /// let client = SangamClient::connect("192.168.68.101:5555")?;
    /// ```
    pub fn connect(addr: &str) -> Result<Self> {
        let stream = TcpStream::connect(addr)?;
        Ok(Self {
            stream,
            buffer: vec![0u8; DEFAULT_BUFFER_SIZE],
        })
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

        // Decode protobuf message
        let proto_msg = proto::sangamio::Message::decode(&self.buffer[..len])?;
        Message::from_proto(proto_msg)
    }

    /// Receive with timeout, returns None on timeout.
    ///
    /// Uses timeout only for the initial length read. Once we start reading
    /// a message, we use a longer timeout to ensure we don't leave the stream
    /// in a corrupted state from partial reads.
    pub fn recv_timeout(&mut self, timeout: Duration) -> Result<Option<Message>> {
        let old_timeout = self.stream.read_timeout()?;
        self.stream.set_read_timeout(Some(timeout))?;

        // Try to read length prefix - timeout OK here
        let mut len_buf = [0u8; 4];
        match self.stream.read_exact(&mut len_buf) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                self.stream.set_read_timeout(old_timeout)?;
                return Ok(None);
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                self.stream.set_read_timeout(old_timeout)?;
                return Ok(None);
            }
            Err(e) => {
                self.stream.set_read_timeout(old_timeout)?;
                return Err(ClientError::Connection(e));
            }
        }

        let len = u32::from_be_bytes(len_buf) as usize;

        // Now we've committed to reading a message - use longer timeout for payload
        // to avoid corrupting the stream
        self.stream.set_read_timeout(Some(Duration::from_secs(5)))?;

        // Grow buffer if needed (up to max)
        if len > self.buffer.len() {
            if len > MAX_BUFFER_SIZE {
                self.stream.set_read_timeout(old_timeout)?;
                return Err(ClientError::BufferTooSmall(len));
            }
            self.buffer.resize(len, 0);
        }

        // Read payload - must succeed now that we've started
        match self.stream.read_exact(&mut self.buffer[..len]) {
            Ok(()) => {}
            Err(e) => {
                self.stream.set_read_timeout(old_timeout)?;
                return Err(ClientError::Connection(e));
            }
        }

        self.stream.set_read_timeout(old_timeout)?;

        // Decode protobuf message
        let proto_msg = proto::sangamio::Message::decode(&self.buffer[..len])?;
        Message::from_proto(proto_msg).map(Some)
    }

    /// Try to receive a message without blocking.
    ///
    /// Returns None if no data is immediately available.
    pub fn try_recv(&mut self) -> Result<Option<Message>> {
        // Use 1ms timeout since 0 duration is not allowed on some platforms
        self.recv_timeout(Duration::from_millis(1))
    }

    /// Send a component control command to the robot.
    ///
    /// # Arguments
    /// * `component_id` - Component to control (e.g., "imu", "drive", "vacuum")
    /// * `action` - Action type (Enable, Disable, Reset, Configure)
    ///
    /// # Example
    /// ```ignore
    /// // Enable IMU calibration
    /// client.send_component_command("imu", ComponentActionType::Enable)?;
    /// ```
    pub fn send_component_command(
        &mut self,
        component_id: &str,
        action: ComponentActionType,
    ) -> Result<()> {
        self.send_component_command_with_config(component_id, action, HashMap::new())
    }

    /// Send a component control command with configuration parameters.
    ///
    /// # Arguments
    /// * `component_id` - Component to control (e.g., "lidar", "drive", "vacuum")
    /// * `action` - Action type (Enable, Disable, Reset, Configure)
    /// * `config` - Configuration parameters (e.g., velocity for drive)
    ///
    /// # Example
    /// ```ignore
    /// // Set drive velocity
    /// let mut config = HashMap::new();
    /// config.insert("linear".to_string(), SensorValue::F32(0.1));
    /// config.insert("angular".to_string(), SensorValue::F32(0.0));
    /// client.send_component_command_with_config("drive", ComponentActionType::Configure, config)?;
    /// ```
    pub fn send_component_command_with_config(
        &mut self,
        component_id: &str,
        action: ComponentActionType,
        config: HashMap<String, SensorValue>,
    ) -> Result<()> {
        use proto::sangamio::{
            ComponentAction, ComponentControl, Message as ProtoMessage,
            command::Command as CommandPayload, component_action::ActionType,
        };

        let action_type = match action {
            ComponentActionType::Enable => ActionType::Enable,
            ComponentActionType::Disable => ActionType::Disable,
            ComponentActionType::Reset => ActionType::Reset,
            ComponentActionType::Configure => ActionType::Configure,
        };

        // Convert SensorValue to proto SensorValue
        let proto_config: HashMap<String, proto::sangamio::SensorValue> =
            config.into_iter().map(|(k, v)| (k, v.to_proto())).collect();

        let command = ProtoMessage {
            topic: "command".to_string(),
            payload: Some(proto::sangamio::message::Payload::Command(
                proto::sangamio::Command {
                    command: Some(CommandPayload::ComponentControl(ComponentControl {
                        id: component_id.to_string(),
                        action: Some(ComponentAction {
                            r#type: action_type as i32,
                            config: proto_config,
                        }),
                    })),
                },
            )),
        };

        self.send_proto(&command)
    }

    /// Send a raw protobuf message.
    fn send_proto(&mut self, msg: &proto::sangamio::Message) -> Result<()> {
        let encoded = msg.encode_to_vec();
        let len = encoded.len() as u32;

        // Write length prefix (big-endian)
        self.stream.write_all(&len.to_be_bytes())?;
        // Write payload
        self.stream.write_all(&encoded)?;
        self.stream.flush()?;

        Ok(())
    }
}

/// Action types for component control commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComponentActionType {
    /// Enable component (for IMU: triggers calibration state query)
    Enable,
    /// Disable component
    Disable,
    /// Reset component (for IMU: triggers factory calibration)
    Reset,
    /// Configure component with parameters
    Configure,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_value_sizes() {
        // Ensure SensorValue variants compile and have expected structure
        let _bool = SensorValue::Bool(true);
        let _u32 = SensorValue::U32(1000);
        let _vec3 = SensorValue::Vector3([1.0, 2.0, 3.0]);
        let _points = SensorValue::PointCloud2D(vec![(0.0, 1.0, 100)]);
    }
}
