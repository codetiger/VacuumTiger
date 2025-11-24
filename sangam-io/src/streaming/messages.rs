//! TCP protocol message types

use crate::core::types::{Command, SensorGroupData, SensorValue};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Top-level message wrapper
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Message {
    pub topic: String,
    pub payload: Payload,
}

/// Message payload types
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type")]
pub enum Payload {
    /// Sensor group data (outbound)
    SensorGroup {
        group_id: String,
        timestamp_us: u64,
        values: HashMap<String, SensorValue>,
    },
    /// Command from client (inbound)
    Command { command: Command },
}

impl Message {
    /// Create a sensor group message
    pub fn sensor_group(data: &SensorGroupData) -> Self {
        Self {
            topic: format!("sensors/{}", data.group_id),
            payload: Payload::SensorGroup {
                group_id: data.group_id.clone(),
                timestamp_us: data.timestamp_us,
                values: data.values.clone(),
            },
        }
    }
}
