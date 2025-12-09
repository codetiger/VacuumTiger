//! TCP streaming module for publishing odometry and SLAM data to visualization clients.
//!
//! This module provides:
//! - TCP server for broadcasting odometry and SLAM data to clients (like Drishti)
//! - Message types for streaming data
//!
//! Note: `PoseTracker` has been moved to `core::types` for proper layer separation.
//! `FusedPoseTracker` has been moved to `sensors::odometry` as it's a sensor fusion concept.

pub mod odometry_pipeline;
mod slam_messages;
mod tcp_publisher;

pub use odometry_pipeline::{OdometryPipeline, OdometryPipelineConfig};

// Re-export from new locations for backward compatibility
pub use crate::core::types::PoseTracker;
pub use crate::sensors::odometry::FusedPoseTracker;
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
