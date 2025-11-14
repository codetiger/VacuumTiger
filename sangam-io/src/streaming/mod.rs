//! TCP streaming for bidirectional robot communication.
//!
//! This module provides publish/receive capabilities for:
//! - **Outbound**: Raw sensor data (telemetry) and lidar scans
//! - **Inbound**: Robot commands (motion and actuators)

pub mod messages;
pub mod tcp_publisher;
pub mod tcp_receiver;

// Re-export main types for external use
pub use tcp_publisher::TcpPublisher;
pub use tcp_receiver::TcpCommandReceiver;
