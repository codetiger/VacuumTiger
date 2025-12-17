//! Streaming module for publishing SLAM data to visualization clients.
//!
//! This module provides:
//! - TCP server for large/critical data (maps, map list)
//! - UDP unicast for high-frequency streams (robot status, sensors, navigation)
//! - Message types for streaming data
//!
//! # Communication Architecture
//!
//! | Stream | Protocol | Rate | Size |
//! |--------|----------|------|------|
//! | RobotStatus | UDP | 10Hz | ~100B |
//! | SensorStatus | UDP | 10Hz | ~200-2KB |
//! | NavigationStatus | UDP | 5Hz | ~500B |
//! | CurrentMap | TCP | 1Hz | 10-100KB |
//! | MapList | TCP | on-change | ~100-500B |

mod tcp_publisher;
pub mod udp_publisher;

// UDP publisher
pub use udp_publisher::{DhruvaUdpPublisher, create_client_registry};

// Proto types used by publisher, tests, and other modules
pub use tcp_publisher::proto::dhruva::{
    CurrentMap, DhruvaCommand, DhruvaResponse, DhruvaStream, EmergencyStopResponse,
    GoalAcceptedResponse, GoalCancelledResponse, LidarScan, MapList, MapSummary, MappingProgress,
    MappingStartedResponse, MappingState as ProtoMappingState, MappingStoppedResponse, NavState,
    NavigationStatus, Pose2D, RobotState, RobotStatus, SensorStatus, dhruva_command,
    dhruva_response, dhruva_stream,
};
