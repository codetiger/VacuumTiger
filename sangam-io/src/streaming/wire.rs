//! Wire format serialization using Protobuf
//!
//! # TCP Protocol Specification
//!
//! SangamIO uses a length-prefixed framing protocol for all TCP communication:
//!
//! ```text
//! ┌──────────────────┬──────────────────────────┐
//! │ Length (4 bytes) │ Payload (variable)       │
//! │ Big-endian u32   │ Protobuf binary          │
//! └──────────────────┴──────────────────────────┘
//! ```
//!
//! ## Framing
//!
//! - **Length field**: 4-byte big-endian unsigned integer
//! - **Payload**: Protobuf-encoded message
//! - **Maximum message size**: 1MB (1,048,576 bytes)
//! - **Byte order**: Network byte order (big-endian) for length prefix
//!
//! ## Performance Characteristics
//!
//! - **Latency**: <1ms serialization time (typical)
//! - **Throughput**: ~50,000 messages/sec
//! - **Bandwidth**: Sensor stream @ 110Hz ≈ 13KB/s

use crate::core::types::{Command, SensorGroupData};
use crate::error::{Error, Result};
use prost::Message;

// Include generated protobuf types
pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/sangamio.rs"));
}

/// Serializer for protobuf wire format
#[derive(Clone, Default)]
pub struct Serializer;

impl Serializer {
    /// Create a new serializer
    pub fn new() -> Self {
        Self
    }

    /// Serialize a sensor group message to bytes
    pub fn serialize_sensor_group(&self, data: &SensorGroupData) -> Result<Vec<u8>> {
        let msg = proto::Message::from_sensor_group(data);
        let mut buf = Vec::with_capacity(msg.encoded_len());
        msg.encode(&mut buf)
            .map_err(|e| Error::Serialization(e.to_string()))?;
        Ok(buf)
    }

    /// Deserialize bytes to a command
    pub fn deserialize_command(&self, bytes: &[u8]) -> Result<Option<Command>> {
        let msg = proto::Message::decode(bytes).map_err(|e| Error::Serialization(e.to_string()))?;

        match msg.payload {
            Some(proto::message::Payload::Command(cmd)) => Ok(Some(Command::from_proto(cmd)?)),
            Some(proto::message::Payload::SensorGroup(_)) => {
                log::warn!("Received unexpected SensorGroup message from client");
                Ok(None)
            }
            None => {
                log::warn!("Received message with no payload");
                Ok(None)
            }
        }
    }
}

/// Create a serializer
pub fn create_serializer() -> Serializer {
    Serializer::new()
}

// Conversion implementations for proto types
impl proto::Message {
    /// Create a sensor group message from internal types
    ///
    /// Uses references where possible to minimize allocations in hot path (~110Hz).
    pub fn from_sensor_group(data: &SensorGroupData) -> Self {
        use std::collections::HashMap;

        // Pre-allocate HashMap with exact capacity to avoid reallocation
        let mut values: HashMap<String, proto::SensorValue> =
            HashMap::with_capacity(data.values.len());

        // Convert values using references - only clone the key string,
        // SensorValue conversion uses references where possible
        for (k, v) in &data.values {
            values.insert(k.clone(), proto::SensorValue::from_ref(v));
        }

        Self {
            topic: format!("sensors/{}", data.group_id),
            payload: Some(proto::message::Payload::SensorGroup(proto::SensorGroup {
                group_id: data.group_id.clone(),
                timestamp_us: data.timestamp_us,
                values,
            })),
        }
    }
}

impl proto::SensorValue {
    /// Convert from a reference to SensorValue - avoids cloning for primitive types
    ///
    /// This is the preferred method in hot paths (~110Hz) as it only clones
    /// heap-allocated data (String, Bytes, PointCloud2D) when necessary.
    pub fn from_ref(value: &crate::core::types::SensorValue) -> Self {
        use crate::core::types::SensorValue as SV;

        let value = match value {
            // Primitive types - zero-cost copy
            SV::Bool(v) => proto::sensor_value::Value::BoolVal(*v),
            SV::U8(v) => proto::sensor_value::Value::U32Val(*v as u32),
            SV::U16(v) => proto::sensor_value::Value::U32Val(*v as u32),
            SV::U32(v) => proto::sensor_value::Value::U32Val(*v),
            SV::U64(v) => proto::sensor_value::Value::U64Val(*v),
            SV::I8(v) => proto::sensor_value::Value::I32Val(*v as i32),
            SV::I16(v) => proto::sensor_value::Value::I32Val(*v as i32),
            SV::I32(v) => proto::sensor_value::Value::I32Val(*v),
            SV::I64(v) => proto::sensor_value::Value::I64Val(*v),
            SV::F32(v) => proto::sensor_value::Value::F32Val(*v),
            SV::F64(v) => proto::sensor_value::Value::F64Val(*v),
            SV::Vector3(arr) => proto::sensor_value::Value::Vector3Val(proto::Vector3 {
                x: arr[0],
                y: arr[1],
                z: arr[2],
            }),
            // Heap types - must clone
            SV::String(v) => proto::sensor_value::Value::StringVal(v.clone()),
            SV::Bytes(v) => proto::sensor_value::Value::BytesVal(v.clone()),
            SV::PointCloud2D(points) => {
                proto::sensor_value::Value::PointcloudVal(proto::PointCloud2D {
                    points: points
                        .iter()
                        .map(|(angle, dist, quality)| proto::LidarPoint {
                            angle_rad: *angle,
                            distance_m: *dist,
                            quality: *quality as u32,
                        })
                        .collect(),
                })
            }
        };

        proto::SensorValue { value: Some(value) }
    }
}

impl From<crate::core::types::SensorValue> for proto::SensorValue {
    fn from(value: crate::core::types::SensorValue) -> Self {
        use crate::core::types::SensorValue as SV;

        let value = match value {
            SV::Bool(v) => proto::sensor_value::Value::BoolVal(v),
            SV::U8(v) => proto::sensor_value::Value::U32Val(v as u32),
            SV::U16(v) => proto::sensor_value::Value::U32Val(v as u32),
            SV::U32(v) => proto::sensor_value::Value::U32Val(v),
            SV::U64(v) => proto::sensor_value::Value::U64Val(v),
            SV::I8(v) => proto::sensor_value::Value::I32Val(v as i32),
            SV::I16(v) => proto::sensor_value::Value::I32Val(v as i32),
            SV::I32(v) => proto::sensor_value::Value::I32Val(v),
            SV::I64(v) => proto::sensor_value::Value::I64Val(v),
            SV::F32(v) => proto::sensor_value::Value::F32Val(v),
            SV::F64(v) => proto::sensor_value::Value::F64Val(v),
            SV::String(v) => proto::sensor_value::Value::StringVal(v),
            SV::Bytes(v) => proto::sensor_value::Value::BytesVal(v),
            SV::Vector3(arr) => proto::sensor_value::Value::Vector3Val(proto::Vector3 {
                x: arr[0],
                y: arr[1],
                z: arr[2],
            }),
            SV::PointCloud2D(points) => {
                proto::sensor_value::Value::PointcloudVal(proto::PointCloud2D {
                    points: points
                        .into_iter()
                        .map(|(angle, dist, quality)| proto::LidarPoint {
                            angle_rad: angle,
                            distance_m: dist,
                            quality: quality as u32,
                        })
                        .collect(),
                })
            }
        };

        proto::SensorValue { value: Some(value) }
    }
}

impl Command {
    /// Convert from proto command
    pub fn from_proto(cmd: proto::Command) -> Result<Self> {
        use crate::core::types::ComponentAction;

        match cmd.command {
            Some(proto::command::Command::ComponentControl(ctrl)) => {
                let action = match ctrl.action {
                    Some(action) => {
                        let config = if action.config.is_empty() {
                            None
                        } else {
                            Some(
                                action
                                    .config
                                    .into_iter()
                                    .map(|(k, v)| {
                                        (k, crate::core::types::SensorValue::from_proto(v))
                                    })
                                    .collect(),
                            )
                        };

                        match proto::component_action::ActionType::try_from(action.r#type) {
                            Ok(proto::component_action::ActionType::Enable) => {
                                ComponentAction::Enable { config }
                            }
                            Ok(proto::component_action::ActionType::Disable) => {
                                ComponentAction::Disable { config }
                            }
                            Ok(proto::component_action::ActionType::Reset) => {
                                ComponentAction::Reset { config }
                            }
                            Ok(proto::component_action::ActionType::Configure) => {
                                ComponentAction::Configure {
                                    config: config.unwrap_or_default(),
                                }
                            }
                            Err(_) => {
                                return Err(Error::Serialization(
                                    "Unknown action type".to_string(),
                                ));
                            }
                        }
                    }
                    None => return Err(Error::Serialization("Missing action".to_string())),
                };

                Ok(Command::ComponentControl {
                    id: ctrl.id,
                    action,
                })
            }
            Some(proto::command::Command::ProtocolSync(_)) => Ok(Command::ProtocolSync),
            Some(proto::command::Command::Shutdown(_)) => Ok(Command::Shutdown),
            None => Err(Error::Serialization("Missing command".to_string())),
        }
    }
}

impl crate::core::types::SensorValue {
    /// Convert from proto sensor value
    pub fn from_proto(value: proto::SensorValue) -> Self {
        use crate::core::types::SensorValue as SV;

        match value.value {
            Some(proto::sensor_value::Value::BoolVal(v)) => SV::Bool(v),
            Some(proto::sensor_value::Value::U32Val(v)) => SV::U32(v),
            Some(proto::sensor_value::Value::U64Val(v)) => SV::U64(v),
            Some(proto::sensor_value::Value::I32Val(v)) => SV::I32(v),
            Some(proto::sensor_value::Value::I64Val(v)) => SV::I64(v),
            Some(proto::sensor_value::Value::F32Val(v)) => SV::F32(v),
            Some(proto::sensor_value::Value::F64Val(v)) => SV::F64(v),
            Some(proto::sensor_value::Value::StringVal(v)) => SV::String(v),
            Some(proto::sensor_value::Value::BytesVal(v)) => SV::Bytes(v),
            Some(proto::sensor_value::Value::Vector3Val(v)) => SV::Vector3([v.x, v.y, v.z]),
            Some(proto::sensor_value::Value::PointcloudVal(v)) => SV::PointCloud2D(
                v.points
                    .into_iter()
                    .map(|p| (p.angle_rad, p.distance_m, p.quality as u8))
                    .collect(),
            ),
            None => SV::Bool(false), // Default fallback
        }
    }
}
