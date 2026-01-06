//! SangamIO TCP client for receiving sensor data and sending commands.
//!
//! Adapted from dhruva-slam/src/io/sangam_client.rs

use crate::error::{DhruvaError, Result};
use prost::Message as ProstMessage;
use std::collections::HashMap;
use std::io::Write;
use std::net::{TcpStream, UdpSocket};
use std::time::Duration;

// Include generated protobuf types
pub mod proto {
    #[allow(clippy::enum_variant_names)]
    pub mod sangamio {
        include!(concat!(env!("OUT_DIR"), "/sangamio.rs"));
    }
}

/// Sensor value types (converted from protobuf)
#[derive(Debug, Clone)]
pub enum SensorValue {
    Bool(bool),
    U32(u32),
    PointCloud2D(Vec<LidarPoint>),
}

/// A single LiDAR point: (angle_rad, distance_m, quality)
pub type LidarPoint = (f32, f32, u8);

/// Parsed message from SangamIO
#[derive(Debug)]
pub struct SensorMessage {
    pub group_id: String,
    pub values: HashMap<String, SensorValue>,
}

impl SensorMessage {
    /// Create from protobuf message
    fn from_proto(msg: proto::sangamio::Message) -> Result<Self> {
        match msg.payload {
            Some(proto::sangamio::message::Payload::SensorGroup(sg)) => {
                let values: HashMap<String, SensorValue> = sg
                    .values
                    .into_iter()
                    .filter_map(|(k, v)| {
                        let val = match v.value {
                            Some(proto::sangamio::sensor_value::Value::BoolVal(b)) => {
                                SensorValue::Bool(b)
                            }
                            Some(proto::sangamio::sensor_value::Value::U32Val(u)) => {
                                SensorValue::U32(u)
                            }
                            Some(proto::sangamio::sensor_value::Value::PointcloudVal(pc)) => {
                                SensorValue::PointCloud2D(
                                    pc.points
                                        .into_iter()
                                        .map(|p| (p.angle_rad, p.distance_m, p.quality as u8))
                                        .collect(),
                                )
                            }
                            _ => return None,
                        };
                        Some((k, val))
                    })
                    .collect();

                Ok(Self {
                    group_id: sg.group_id,
                    values,
                })
            }
            Some(proto::sangamio::message::Payload::Command(_)) => {
                Err(DhruvaError::Protocol("Received command from server".into()))
            }
            None => Err(DhruvaError::Protocol("Empty payload".into())),
        }
    }

    /// Check if this is a sensor status message
    pub fn is_sensor_status(&self) -> bool {
        self.group_id == "sensor_status"
    }

    /// Check if this is a lidar message
    pub fn is_lidar(&self) -> bool {
        self.group_id == "lidar"
    }

    /// Extract lidar point cloud if this is a lidar message
    pub fn as_lidar(&self) -> Option<&Vec<LidarPoint>> {
        if !self.is_lidar() {
            return None;
        }
        if let Some(SensorValue::PointCloud2D(points)) = self.values.get("scan") {
            Some(points)
        } else {
            None
        }
    }

    /// Extract wheel encoder ticks (left, right)
    pub fn encoder_ticks(&self) -> Option<(u16, u16)> {
        if !self.is_sensor_status() {
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

    /// Extract bumper state (left, right)
    pub fn bumper_state(&self) -> Option<(bool, bool)> {
        if !self.is_sensor_status() {
            return None;
        }
        let left = match self.values.get("bumper_left") {
            Some(SensorValue::Bool(v)) => *v,
            _ => false,
        };
        let right = match self.values.get("bumper_right") {
            Some(SensorValue::Bool(v)) => *v,
            _ => false,
        };
        Some((left, right))
    }

    /// Extract cliff sensor states (left_side, left_front, right_front, right_side)
    pub fn cliff_state(&self) -> Option<(bool, bool, bool, bool)> {
        if !self.is_sensor_status() {
            return None;
        }
        let left_side = match self.values.get("cliff_left_side") {
            Some(SensorValue::Bool(v)) => *v,
            _ => false,
        };
        let left_front = match self.values.get("cliff_left_front") {
            Some(SensorValue::Bool(v)) => *v,
            _ => false,
        };
        let right_front = match self.values.get("cliff_right_front") {
            Some(SensorValue::Bool(v)) => *v,
            _ => false,
        };
        let right_side = match self.values.get("cliff_right_side") {
            Some(SensorValue::Bool(v)) => *v,
            _ => false,
        };
        Some((left_side, left_front, right_front, right_side))
    }
}

/// Default buffer size (64KB)
const DEFAULT_BUFFER_SIZE: usize = 65536;

/// TCP/UDP client for SangamIO daemon
pub struct SangamClient {
    stream: TcpStream,
    udp_socket: UdpSocket,
    buffer: Vec<u8>,
}

impl SangamClient {
    /// Connect with timeout (uses same port for TCP commands and UDP sensor receiving)
    pub fn connect_timeout(addr: &str, timeout: Duration) -> Result<Self> {
        let sock_addr: std::net::SocketAddr = addr
            .parse()
            .map_err(|e| DhruvaError::Config(format!("Invalid address: {}", e)))?;

        // Use the same port for UDP as TCP (they're different protocols, can coexist)
        Self::connect_with_udp_port(addr, timeout, sock_addr.port())
    }

    /// Connect with timeout and custom UDP port
    pub fn connect_with_udp_port(addr: &str, timeout: Duration, udp_port: u16) -> Result<Self> {
        let addr: std::net::SocketAddr = addr
            .parse()
            .map_err(|e| DhruvaError::Config(format!("Invalid address: {}", e)))?;
        let stream = TcpStream::connect_timeout(&addr, timeout)?;

        // Bind UDP socket to receive sensor data from SangamIO
        // TCP and UDP can use the same port number (different protocols)
        let udp_bind_addr = format!("0.0.0.0:{}", udp_port);
        let udp_socket = UdpSocket::bind(&udp_bind_addr).map_err(|e| {
            DhruvaError::Config(format!("Failed to bind UDP to {}: {}", udp_bind_addr, e))
        })?;
        udp_socket.set_nonblocking(true)?;

        tracing::info!("UDP socket bound to port {} for sensor data", udp_port);

        Ok(Self {
            stream,
            udp_socket,
            buffer: vec![0u8; DEFAULT_BUFFER_SIZE],
        })
    }

    /// Set read timeout for TCP (UDP remains non-blocking for buffer draining)
    pub fn set_timeout(&mut self, timeout: Option<Duration>) -> Result<()> {
        self.stream.set_read_timeout(timeout)?;
        // Keep UDP non-blocking to allow draining the buffer efficiently
        // The main loop will drain all available messages before processing
        self.udp_socket.set_nonblocking(true)?;
        Ok(())
    }

    /// Receive sensor message from UDP (non-blocking)
    pub fn recv_udp(&mut self) -> Result<Option<SensorMessage>> {
        match self.udp_socket.recv(&mut self.buffer) {
            Ok(len) => {
                if len < 4 {
                    return Ok(None);
                }
                // Read length prefix
                let msg_len = u32::from_be_bytes([
                    self.buffer[0],
                    self.buffer[1],
                    self.buffer[2],
                    self.buffer[3],
                ]) as usize;

                if len < 4 + msg_len {
                    return Err(DhruvaError::Protocol("Incomplete UDP message".into()));
                }

                // Decode protobuf
                let proto_msg = proto::sangamio::Message::decode(&self.buffer[4..4 + msg_len])?;
                SensorMessage::from_proto(proto_msg).map(Some)
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(DhruvaError::Connection(e)),
        }
    }

    /// Send drive velocity command
    pub fn send_drive_command(&mut self, linear: f32, angular: f32) -> Result<()> {
        use proto::sangamio::{
            ComponentAction, ComponentControl, Message as ProtoMessage,
            command::Command as CommandPayload, component_action::ActionType,
        };

        let mut config = HashMap::new();
        config.insert(
            "linear".to_string(),
            proto::sangamio::SensorValue {
                value: Some(proto::sangamio::sensor_value::Value::F32Val(linear)),
            },
        );
        config.insert(
            "angular".to_string(),
            proto::sangamio::SensorValue {
                value: Some(proto::sangamio::sensor_value::Value::F32Val(angular)),
            },
        );

        let command = ProtoMessage {
            topic: "command".to_string(),
            payload: Some(proto::sangamio::message::Payload::Command(
                proto::sangamio::Command {
                    command: Some(CommandPayload::ComponentControl(ComponentControl {
                        id: "drive".to_string(),
                        action: Some(ComponentAction {
                            r#type: ActionType::Configure as i32,
                            config,
                        }),
                    })),
                },
            )),
        };

        self.send_proto(&command)
    }

    /// Send stop command (set velocity to zero)
    pub fn send_stop(&mut self) -> Result<()> {
        self.send_drive_command(0.0, 0.0)
    }

    /// Enable the drive component (must call before sending velocity commands)
    pub fn enable_drive(&mut self) -> Result<()> {
        use proto::sangamio::{
            ComponentAction, ComponentControl, Message as ProtoMessage,
            command::Command as CommandPayload, component_action::ActionType,
        };

        let command = ProtoMessage {
            topic: "command".to_string(),
            payload: Some(proto::sangamio::message::Payload::Command(
                proto::sangamio::Command {
                    command: Some(CommandPayload::ComponentControl(ComponentControl {
                        id: "drive".to_string(),
                        action: Some(ComponentAction {
                            r#type: ActionType::Enable as i32,
                            config: HashMap::new(),
                        }),
                    })),
                },
            )),
        };

        self.send_proto(&command)
    }

    /// Enable the lidar component
    pub fn enable_lidar(&mut self) -> Result<()> {
        use proto::sangamio::{
            ComponentAction, ComponentControl, Message as ProtoMessage,
            command::Command as CommandPayload, component_action::ActionType,
        };

        let command = ProtoMessage {
            topic: "command".to_string(),
            payload: Some(proto::sangamio::message::Payload::Command(
                proto::sangamio::Command {
                    command: Some(CommandPayload::ComponentControl(ComponentControl {
                        id: "lidar".to_string(),
                        action: Some(ComponentAction {
                            r#type: ActionType::Enable as i32,
                            config: HashMap::new(),
                        }),
                    })),
                },
            )),
        };

        self.send_proto(&command)
    }

    /// Send raw protobuf message
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
