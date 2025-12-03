//! TCP streaming module for publishing odometry and SLAM data to visualization clients.
//!
//! This module provides a TCP server that broadcasts odometry poses, diagnostics,
//! and SLAM data (status, map, scan, diagnostics) to connected clients (like Drishti).

pub mod odometry_pipeline;
mod slam_messages;
mod tcp_publisher;

pub use odometry_pipeline::{OdometryPipeline, OdometryPipelineConfig};
pub use slam_messages::{
    LoopClosureStats,
    MappingStats,
    ParticleFilterStats,
    ScanMatchStats,
    SlamCommand,
    // Diagnostics types
    SlamDiagnosticsMessage,
    SlamMapMessage,
    SlamScanMessage,
    SlamStatusMessage,
    TimingBreakdown,
};
pub use tcp_publisher::{OdometryDiagnostics, OdometryMessage, OdometryPublisher, SlamMessage};
