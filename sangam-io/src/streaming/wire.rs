//! Wire format serialization abstraction

use crate::error::{Error, Result};
use crate::streaming::messages::Message;

/// Supported wire formats
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WireFormat {
    /// Binary format using postcard - fast and compact
    Postcard,
    /// JSON format - human-readable for debugging
    Json,
}

impl Default for WireFormat {
    fn default() -> Self {
        WireFormat::Json
    }
}

/// Serializer that can handle both formats
#[derive(Clone)]
pub struct Serializer {
    format: WireFormat,
}

impl Serializer {
    /// Create a new serializer for the given format
    pub fn new(format: WireFormat) -> Self {
        Self { format }
    }

    /// Serialize a message to bytes
    pub fn serialize(&self, msg: &Message) -> Result<Vec<u8>> {
        match self.format {
            WireFormat::Postcard => {
                postcard::to_allocvec(msg).map_err(|e| Error::Serialization(e.to_string()))
            }
            WireFormat::Json => {
                serde_json::to_vec(msg).map_err(|e| Error::Serialization(e.to_string()))
            }
        }
    }

    /// Deserialize bytes to a message
    pub fn deserialize(&self, bytes: &[u8]) -> Result<Message> {
        match self.format {
            WireFormat::Postcard => {
                postcard::from_bytes(bytes).map_err(|e| Error::Serialization(e.to_string()))
            }
            WireFormat::Json => {
                serde_json::from_slice(bytes).map_err(|e| Error::Serialization(e.to_string()))
            }
        }
    }
}

/// Create a serializer for the given wire format
pub fn create_serializer(format: WireFormat) -> Serializer {
    Serializer::new(format)
}
