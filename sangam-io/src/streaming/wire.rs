//! Wire format serialization abstraction
//!
//! # TCP Protocol Specification
//!
//! SangamIO uses a length-prefixed framing protocol for all TCP communication:
//!
//! ```text
//! ┌──────────────────┬──────────────────────────┐
//! │ Length (4 bytes) │ Payload (variable)       │
//! │ Big-endian u32   │ JSON or Postcard binary  │
//! └──────────────────┴──────────────────────────┘
//! ```
//!
//! ## Framing
//!
//! - **Length field**: 4-byte big-endian unsigned integer
//! - **Payload**: Serialized message in configured wire format
//! - **Maximum message size**: 1MB (1,048,576 bytes)
//! - **Byte order**: Network byte order (big-endian) for length prefix
//!
//! ## Wire Formats
//!
//! Two wire formats are supported:
//!
//! ### JSON (Default)
//! - **Pros**: Human-readable, easy to debug, widely supported
//! - **Cons**: Larger message size, slower serialization
//! - **Use case**: Development, debugging, cross-language clients
//!
//! ### Postcard (Binary)
//! - **Pros**: Compact, fast serialization, type-safe
//! - **Cons**: Binary format, requires schema knowledge
//! - **Use case**: Production, high-frequency sensor streaming
//!
//! ## Message Flow
//!
//! **Sensor streaming (daemon → client):**
//! 1. Sensor data updates in driver threads
//! 2. Publisher checks if data changed (timestamp comparison)
//! 3. Serialize to wire format
//! 4. Send length prefix + payload
//! 5. Client receives and deserializes
//!
//! **Command handling (client → daemon):**
//! 1. Client serializes command
//! 2. Send length prefix + payload
//! 3. Daemon receives and deserializes
//! 4. Dispatch to device driver
//! 5. No response (fire-and-forget for low latency)
//!
//! ## Error Handling
//!
//! - **Malformed length**: Connection closed
//! - **Oversized message**: Connection closed (security)
//! - **Deserialization failure**: Message logged and discarded, connection remains open
//! - **Serialization failure**: Message skipped, error logged
//!
//! ## Performance Characteristics
//!
//! - **Latency**: <1ms serialization time (typical)
//! - **Throughput**: ~10,000 messages/sec (JSON), ~50,000 messages/sec (Postcard)
//! - **Bandwidth**: Sensor stream @ 500Hz ≈ 200KB/s (JSON), ≈ 80KB/s (Postcard)

use crate::error::{Error, Result};
use crate::streaming::messages::Message;

/// Supported wire formats
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum WireFormat {
    /// Binary format using postcard - fast and compact
    Postcard,
    /// JSON format - human-readable for debugging
    #[default]
    Json,
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
