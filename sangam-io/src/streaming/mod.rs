//! Network streaming module for SangamIO
//!
//! This module provides the communication layer between SangamIO and client
//! applications (SLAM, visualization, etc.) using a hybrid UDP/TCP architecture.
//!
//! # Architecture Overview
//!
//! ```text
//! ┌───────────────────────────────────────────────────────────┐
//! │                     SangamIO Daemon                       │
//! │                                                           │
//! │      ┌─────────────────┐        ┌─────────────────┐       │
//! │      │  UDP Publisher  │        │  TCP Receiver   │       │
//! │      │  (port 5555)    │        │  (port 5555)    │       │
//! │      │                 │        │                 │       │
//! │      │  • sensor_status│        │  • Commands     │       │
//! │      │    @ 110Hz      │        │  • Client       │       │
//! │      │  • lidar @ 5Hz  │        │    registration │       │
//! │      └────────┬────────┘        └────────┬────────┘       │
//! │               │                          │                │
//! └───────────────┼──────────────────────────┼────────────────┘
//!                 │ UDP unicast              │ TCP stream
//!                 ▼                          ▼
//! ┌───────────────────────────────────────────────────────────┐
//! │                     Client Application                    │
//! │  (receives sensor data via UDP, sends commands via TCP)   │
//! └───────────────────────────────────────────────────────────┘
//! ```
//!
//! # Protocol Design
//!
//! ## Why UDP for Sensor Data?
//!
//! - **Lower latency**: No TCP handshake overhead, no head-of-line blocking
//! - **Better for real-time**: Dropped packets are preferable to delayed ones
//! - **Reduced buffering**: No kernel TCP buffer delays
//! - **Fire-and-forget**: Server doesn't wait for ACKs
//!
//! ## Why TCP for Commands?
//!
//! - **Reliability**: Commands must not be lost or duplicated
//! - **Ordering**: Commands must execute in order
//! - **Acknowledgment**: Caller knows command was received
//!
//! # Wire Format
//!
//! Both UDP and TCP use the same length-prefixed Protobuf format for
//! client compatibility:
//!
//! ```text
//! ┌──────────────────┬─────────────────────┐
//! │ Length (4 bytes) │ Protobuf Payload    │
//! │ Big-endian u32   │ (variable size)     │
//! └──────────────────┴─────────────────────┘
//! ```
//!
//! # Client Registration
//!
//! UDP streaming uses unicast (not broadcast) for security and efficiency.
//! Clients must first connect via TCP to register their UDP endpoint:
//!
//! 1. Client connects to TCP port 5555
//! 2. Server records client's IP address
//! 3. UDP packets are sent to client's IP on same port
//! 4. When TCP disconnects, UDP streaming to that client stops
//!
//! # Module Contents
//!
//! - [`UdpPublisher`]: Streams sensor data to registered clients
//! - [`TcpReceiver`]: Handles incoming commands and client registration
//! - [`wire`]: Protobuf serialization/deserialization

pub mod tcp_receiver;
pub mod udp_publisher;
pub mod wire;

pub use tcp_receiver::TcpReceiver;
pub use udp_publisher::{UdpClientRegistry, UdpPublisher};
pub use wire::create_serializer;
