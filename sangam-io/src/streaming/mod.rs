//! Streaming module for SangamIO
//!
//! - **UDP**: Unicast sensor data to registered clients (fire-and-forget)
//! - **TCP**: Handles commands from clients (reliable, bidirectional)

pub mod tcp_receiver;
pub mod udp_publisher;
pub mod wire;

pub use tcp_receiver::TcpReceiver;
pub use udp_publisher::{UdpClientRegistry, UdpPublisher};
pub use wire::create_serializer;
