//! UDP publisher for real-time DhruvaSLAM data streaming.
//!
//! This module implements UDP unicast publisher for streaming high-frequency
//! data (RobotStatus, SensorStatus, NavigationStatus) to registered clients.
//!
//! # Design Principles
//!
//! Following the SangamIO UDP streaming pattern:
//! - **UDP unicast** to registered client (not broadcast)
//! - **Client registration** via TCP connection
//! - **Single-client** enforcement
//! - **Length-prefixed Protobuf** framing (same as TCP)
//!
//! # Wire Format
//!
//! Same as TCP for client compatibility:
//! ```text
//! ┌──────────────────┬──────────────────────────┐
//! │ Length (4 bytes) │ Protobuf DhruvaStream    │
//! │ Big-endian u32   │ (variable size)          │
//! └──────────────────┴──────────────────────────┘
//! ```
//!
//! # Data Streams via UDP
//!
//! - RobotStatus @ 10Hz (~100 bytes)
//! - SensorStatus @ 10Hz (~200-2KB depending on lidar)
//! - NavigationStatus @ 5Hz (~500 bytes)
//!
//! Maps (CurrentMap, MapList) remain on TCP due to size.

use prost::Message;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use thiserror::Error;

/// Publisher errors
#[derive(Error, Debug)]
pub enum PublisherError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Encode error: {0}")]
    Encode(String),
}

impl From<prost::EncodeError> for PublisherError {
    fn from(e: prost::EncodeError) -> Self {
        PublisherError::Encode(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, PublisherError>;

/// Type alias for UDP client registry (single client at a time).
pub type UdpClientRegistry = Arc<Mutex<Option<SocketAddr>>>;

/// Create a new client registry.
pub fn create_client_registry() -> UdpClientRegistry {
    Arc::new(Mutex::new(None))
}

/// Maximum UDP datagram size (should fit most DhruvaStream messages).
const MAX_UDP_BUFFER_SIZE: usize = 4096;

/// UDP publisher for DhruvaSLAM data streams.
///
/// Streams high-frequency data to registered UDP clients using unicast.
/// Client registration is managed externally (typically from TCP accept).
pub struct DhruvaUdpPublisher {
    socket: UdpSocket,
    client_registry: UdpClientRegistry,
    running: Arc<AtomicBool>,
    send_buffer: Vec<u8>,
}

impl DhruvaUdpPublisher {
    /// Create a new UDP publisher.
    ///
    /// # Arguments
    /// * `port` - UDP port to bind (same as TCP for simplicity)
    /// * `client_registry` - Shared registry for client address
    /// * `running` - Shared flag for shutdown coordination
    pub fn new(
        port: u16,
        client_registry: UdpClientRegistry,
        running: Arc<AtomicBool>,
    ) -> Result<Self> {
        let socket = UdpSocket::bind(format!("0.0.0.0:{}", port))?;

        // Set non-blocking for sends
        socket.set_nonblocking(true)?;

        log::info!("DhruvaUdpPublisher bound to port {}", port);

        Ok(Self {
            socket,
            client_registry,
            running,
            send_buffer: Vec::with_capacity(MAX_UDP_BUFFER_SIZE),
        })
    }

    /// Register a client for UDP streaming.
    ///
    /// Called when a TCP client connects. The client IP from TCP is used
    /// for UDP unicast on the same port.
    pub fn register_client(registry: &UdpClientRegistry, client_addr: SocketAddr) {
        let mut guard = registry.lock().unwrap_or_else(|e| e.into_inner());
        *guard = Some(client_addr);
        log::info!("UDP client registered: {}", client_addr);
    }

    /// Unregister the current client.
    ///
    /// Called when TCP client disconnects.
    pub fn unregister_client(registry: &UdpClientRegistry) {
        let mut guard = registry.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(addr) = guard.take() {
            log::info!("UDP client unregistered: {}", addr);
        }
    }

    /// Get current registered client address.
    fn get_client(&self) -> Option<SocketAddr> {
        *self
            .client_registry
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    /// Publish a DhruvaStream message to the registered client.
    ///
    /// Non-blocking: drops message if no client or send fails.
    pub fn publish(&mut self, msg: &super::DhruvaStream) {
        let target = match self.get_client() {
            Some(addr) => addr,
            None => return, // No client registered
        };

        // Encode message
        let payload = msg.encode_to_vec();

        // Build length-prefixed frame
        let len = payload.len() as u32;
        self.send_buffer.clear();
        self.send_buffer.extend_from_slice(&len.to_be_bytes());
        self.send_buffer.extend_from_slice(&payload);

        // Send (non-blocking, ignore errors - UDP is best-effort)
        if let Err(e) = self.socket.send_to(&self.send_buffer, target) {
            // Only log if not WouldBlock (expected for non-blocking)
            if e.kind() != std::io::ErrorKind::WouldBlock {
                log::trace!("UDP send failed: {}", e);
            }
        }
    }

    /// Publish RobotStatus.
    pub fn publish_robot_status(&mut self, status: &crate::state::RobotStatus, timestamp_us: u64) {
        let msg = build_robot_status_message(status, timestamp_us);
        self.publish(&msg);
    }

    /// Publish SensorStatus.
    pub fn publish_sensor_status(
        &mut self,
        status: &crate::state::SensorStatus,
        timestamp_us: u64,
    ) {
        let msg = build_sensor_status_message(status, timestamp_us);
        self.publish(&msg);
    }

    /// Publish NavigationStatus.
    pub fn publish_navigation_status(
        &mut self,
        nav_state: &crate::navigation::NavigationState,
        robot_pose: &crate::core::types::Pose2D,
        timestamp_us: u64,
    ) {
        let msg = build_navigation_status_message(nav_state, robot_pose, timestamp_us);
        self.publish(&msg);
    }

    /// Publish MappingProgress.
    pub fn publish_mapping_progress(
        &mut self,
        progress: &crate::state::MappingProgressState,
        timestamp_us: u64,
    ) {
        let msg = build_mapping_progress_message(progress, timestamp_us);
        self.publish(&msg);
    }
}

/// Build RobotStatus stream message.
fn build_robot_status_message(
    status: &crate::state::RobotStatus,
    timestamp_us: u64,
) -> super::DhruvaStream {
    use super::{Pose2D, RobotState, RobotStatus as ProtoRobotStatus, dhruva_stream};

    let state = match status.state {
        crate::state::RobotState::Idle => RobotState::Idle,
        crate::state::RobotState::Mapping => RobotState::Mapping,
        crate::state::RobotState::Localizing => RobotState::Localizing,
        crate::state::RobotState::Lost => RobotState::Lost,
    };

    super::DhruvaStream {
        timestamp_us,
        data: Some(dhruva_stream::Data::RobotStatus(ProtoRobotStatus {
            pose: Some(Pose2D {
                x: status.pose.x,
                y: status.pose.y,
                theta: status.pose.theta,
            }),
            state: state as i32,
            active_map_id: status.active_map_id.clone().unwrap_or_default(),
            battery_percent: status.battery_percent,
            is_charging: status.is_charging,
            is_docked: status.is_docked,
            distance_traveled_m: status.distance_traveled_m,
            keyframe_count: status.keyframe_count,
            loop_closures: status.loop_closures,
            map_area_m2: status.map_area_m2,
            localization_confidence: status.localization_confidence,
        })),
    }
}

/// Build SensorStatus stream message.
fn build_sensor_status_message(
    status: &crate::state::SensorStatus,
    timestamp_us: u64,
) -> super::DhruvaStream {
    use super::{LidarScan, Pose2D, SensorStatus as ProtoSensorStatus, dhruva_stream};

    let lidar = if !status.lidar_ranges.is_empty() && !status.lidar_angles.is_empty() {
        // Use actual angles from the lidar scan (includes lidar offset correction)
        let angle_min = status
            .lidar_angles
            .first()
            .copied()
            .unwrap_or(-std::f32::consts::PI);
        let angle_max = status
            .lidar_angles
            .last()
            .copied()
            .unwrap_or(std::f32::consts::PI);
        let num_rays = status.lidar_ranges.len();
        let angle_increment = if num_rays > 1 {
            (angle_max - angle_min) / (num_rays - 1) as f32
        } else {
            0.0
        };

        Some(LidarScan {
            angle_min,
            angle_max,
            angle_increment,
            ranges: status.lidar_ranges.clone(),
            intensities: Vec::new(),
        })
    } else {
        None
    };

    super::DhruvaStream {
        timestamp_us,
        data: Some(dhruva_stream::Data::SensorStatus(ProtoSensorStatus {
            lidar,
            left_encoder_ticks: status.left_encoder_ticks,
            right_encoder_ticks: status.right_encoder_ticks,
            left_velocity_mps: status.left_velocity_mps,
            right_velocity_mps: status.right_velocity_mps,
            gyro_z_radps: status.gyro_z_radps,
            accel_x_mps2: status.accel_x_mps2,
            accel_y_mps2: status.accel_y_mps2,
            raw_odometry: Some(Pose2D {
                x: status.raw_odometry.x,
                y: status.raw_odometry.y,
                theta: status.raw_odometry.theta,
            }),
        })),
    }
}

/// Build NavigationStatus stream message.
fn build_navigation_status_message(
    nav_state: &crate::navigation::NavigationState,
    _robot_pose: &crate::core::types::Pose2D,
    timestamp_us: u64,
) -> super::DhruvaStream {
    use super::{NavState as ProtoNavState, NavigationStatus, Pose2D, dhruva_stream};
    use crate::navigation::NavState;

    // Convert internal NavState to proto NavState
    let proto_state = match nav_state.nav_state {
        NavState::Idle => ProtoNavState::Idle,
        NavState::Planning => ProtoNavState::Planning,
        NavState::Navigating => ProtoNavState::Navigating,
        NavState::RotatingToHeading => ProtoNavState::RotatingToHeading,
        NavState::TargetReached => ProtoNavState::Reached,
        NavState::Failed => ProtoNavState::Failed,
        NavState::Cancelled => ProtoNavState::Cancelled,
    };

    // Build path waypoints (Waypoint only has x, y - theta is 0)
    let path: Vec<Pose2D> = nav_state
        .current_path()
        .map(|p| {
            p.waypoints
                .iter()
                .map(|wp| Pose2D {
                    x: wp.x,
                    y: wp.y,
                    theta: 0.0,
                })
                .collect()
        })
        .unwrap_or_default();

    // Get goal from current target
    let goal = nav_state.current_target().map(|t| Pose2D {
        x: t.position.x,
        y: t.position.y,
        theta: t.heading.unwrap_or(0.0),
    });

    // Get goal description
    let goal_description = nav_state
        .current_target()
        .map(|t| t.description.clone())
        .unwrap_or_default();

    // Get target ID
    let target_id = nav_state.current_target().map(|t| t.id).unwrap_or(0);

    // Calculate distance remaining
    let distance_remaining_m = nav_state.path_remaining();
    let path_length_m = nav_state.path_total_length();

    // Estimate time remaining (assume 0.2 m/s average speed)
    let estimated_time_remaining_s = distance_remaining_m / 0.2;

    // Get failure reason
    let failure_reason = nav_state
        .failure_reason
        .as_ref()
        .map(|r| r.description())
        .unwrap_or_default();

    super::DhruvaStream {
        timestamp_us,
        data: Some(dhruva_stream::Data::NavigationStatus(NavigationStatus {
            state: proto_state as i32,
            path,
            current_waypoint_index: nav_state.current_waypoint_index as u32,
            path_length_m,
            distance_remaining_m,
            estimated_time_remaining_s,
            goal,
            goal_description,
            target_id,
            targets_pending: nav_state.target_count() as u32,
            targets_completed: nav_state.targets_completed,
            status_message: nav_state.status_message.clone(),
            failure_reason,
        })),
    }
}

/// Build MappingProgress stream message.
fn build_mapping_progress_message(
    progress: &crate::state::MappingProgressState,
    timestamp_us: u64,
) -> super::DhruvaStream {
    use super::{MappingProgress, Pose2D, ProtoMappingState, dhruva_stream};
    use crate::state::MappingProgressStateEnum;

    let proto_state = match progress.state {
        MappingProgressStateEnum::Idle => ProtoMappingState::Idle,
        MappingProgressStateEnum::Initializing => ProtoMappingState::Initializing,
        MappingProgressStateEnum::Analyzing => ProtoMappingState::Analyzing,
        MappingProgressStateEnum::Escaping => ProtoMappingState::Escaping,
        MappingProgressStateEnum::Scanning => ProtoMappingState::Scanning,
        MappingProgressStateEnum::FindingFrontier => ProtoMappingState::FindingFrontier,
        MappingProgressStateEnum::Navigating => ProtoMappingState::Navigating,
        MappingProgressStateEnum::Complete => ProtoMappingState::Complete,
        MappingProgressStateEnum::Paused => ProtoMappingState::Paused,
        MappingProgressStateEnum::Failed => ProtoMappingState::Failed,
    };

    super::DhruvaStream {
        timestamp_us,
        data: Some(dhruva_stream::Data::MappingProgress(MappingProgress {
            state: proto_state as i32,
            frontiers_found: progress.frontiers_found,
            frontiers_visited: progress.frontiers_visited,
            frontiers_failed: progress.frontiers_failed,
            bumper_obstacles_marked: progress.bumper_obstacles_marked,
            elapsed_time_ms: progress.elapsed_time_ms,
            scan_count: progress.scan_count,
            current_frontier: progress.current_frontier.map(|p| Pose2D {
                x: p.x,
                y: p.y,
                theta: 0.0,
            }),
            frontier_distance: progress.frontier_distance,
            area_explored_m2: progress.area_explored_m2,
            exploration_percent: progress.exploration_percent,
            status_message: progress.status_message.clone(),
            failure_reason: progress.failure_reason.clone(),
        })),
    }
}
