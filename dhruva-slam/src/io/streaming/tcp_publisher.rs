//! Protobuf types for Dhruva SLAM streaming.
//!
//! This module provides the generated protobuf types used for TCP/UDP
//! communication with visualization clients.

// Include generated protobuf types
pub mod proto {
    pub mod dhruva {
        include!(concat!(env!("OUT_DIR"), "/dhruva.rs"));
    }
}
