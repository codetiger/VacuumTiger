//! TCP publisher for streaming odometry data to visualization clients.
//!
//! Broadcasts pose and diagnostics messages to connected TCP clients using
//! length-prefixed Protobuf framing.
//!
//! # Wire Protocol
//!
//! ```text
//! ┌──────────────────┬──────────────────────────┐
//! │ Length (4 bytes) │ Protobuf Payload         │
//! │ Big-endian u32   │                          │
//! └──────────────────┴──────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::streaming::OdometryPublisher;
//! use dhruva_slam::Pose2D;
//!
//! let publisher = OdometryPublisher::new(5557)?;
//!
//! // In main loop
//! let pose = Pose2D::new(1.0, 0.5, 0.1);
//! publisher.publish_pose(&pose, timestamp_us);
//! ```

use crate::algorithms::mapping::OccupancyGrid;
use crate::core::types::{PointCloud2D, Pose2D};
use crate::engine::slam::SlamStatus;
use prost::Message;
use serde::Serialize;
use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use thiserror::Error;

use super::slam_messages::SlamDiagnosticsMessage;

// Include generated protobuf types
pub mod proto {
    pub mod dhruva {
        include!(concat!(env!("OUT_DIR"), "/dhruva.rs"));
    }
}

/// Publisher errors
#[derive(Error, Debug)]
pub enum PublisherError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Serialization error: {0}")]
    Serialization(String),
}

impl From<prost::EncodeError> for PublisherError {
    fn from(e: prost::EncodeError) -> Self {
        PublisherError::Serialization(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, PublisherError>;

/// Odometry pose message payload (for internal use)
#[derive(Debug, Clone, Serialize)]
pub struct OdometryPose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub timestamp_us: u64,
}

impl OdometryPose {
    pub fn from_pose(pose: &Pose2D, timestamp_us: u64) -> Self {
        Self {
            x: pose.x,
            y: pose.y,
            theta: pose.theta,
            timestamp_us,
        }
    }
}

/// Odometry diagnostics message payload
#[derive(Debug, Clone, Serialize, Default)]
pub struct OdometryDiagnostics {
    /// Estimated drift rate in rad/s
    pub drift_rate: f32,
    /// Left wheel tick rate in ticks/s
    pub tick_rate_left: f32,
    /// Right wheel tick rate in ticks/s
    pub tick_rate_right: f32,
    /// Total distance traveled in meters
    pub distance_traveled: f32,
    /// Estimated gyro bias in rad/s
    pub gyro_bias: f32,
}

/// Message types for odometry streaming
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "topic", content = "payload")]
pub enum OdometryMessage {
    #[serde(rename = "odometry/pose")]
    Pose(OdometryPose),

    #[serde(rename = "odometry/diagnostics")]
    Diagnostics(OdometryDiagnostics),
}

/// Message types for SLAM streaming (for serde compatibility in tests)
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "topic", content = "payload")]
pub enum SlamMessage {
    #[serde(rename = "slam/status")]
    Status(super::slam_messages::SlamStatusMessage),

    #[serde(rename = "slam/map")]
    Map(super::slam_messages::SlamMapMessage),

    #[serde(rename = "slam/scan")]
    Scan(super::slam_messages::SlamScanMessage),

    #[serde(rename = "slam/diagnostics")]
    Diagnostics(SlamDiagnosticsMessage),
}

/// Client connection state
struct Client {
    stream: TcpStream,
    addr: std::net::SocketAddr,
}

/// TCP publisher for broadcasting odometry data to visualization clients.
///
/// Maintains a list of connected clients and broadcasts messages to all of them.
/// Automatically removes disconnected clients.
pub struct OdometryPublisher {
    clients: Arc<Mutex<Vec<Client>>>,
    running: Arc<AtomicBool>,
    listener_handle: Option<thread::JoinHandle<()>>,
}

impl OdometryPublisher {
    /// Create a new odometry publisher listening on the specified port.
    ///
    /// Starts a background thread to accept new client connections.
    pub fn new(port: u16) -> Result<Self> {
        let listener = TcpListener::bind(format!("0.0.0.0:{}", port))?;
        listener.set_nonblocking(true)?;

        let clients = Arc::new(Mutex::new(Vec::new()));
        let running = Arc::new(AtomicBool::new(true));

        let clients_clone = clients.clone();
        let running_clone = running.clone();

        let listener_handle = thread::spawn(move || {
            Self::accept_loop(listener, clients_clone, running_clone);
        });

        log::info!("OdometryPublisher listening on port {}", port);

        Ok(Self {
            clients,
            running,
            listener_handle: Some(listener_handle),
        })
    }

    /// Accept loop for new client connections
    fn accept_loop(
        listener: TcpListener,
        clients: Arc<Mutex<Vec<Client>>>,
        running: Arc<AtomicBool>,
    ) {
        while running.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, addr)) => {
                    log::info!("New odometry client connected: {}", addr);

                    // Set TCP_NODELAY for low latency
                    if let Err(e) = stream.set_nodelay(true) {
                        log::warn!("Failed to set TCP_NODELAY: {}", e);
                    }

                    // Set write timeout
                    if let Err(e) = stream.set_write_timeout(Some(Duration::from_secs(1))) {
                        log::warn!("Failed to set write timeout: {}", e);
                    }

                    if let Ok(mut clients) = clients.lock() {
                        clients.push(Client { stream, addr });
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No pending connections, sleep briefly
                    thread::sleep(Duration::from_millis(100));
                }
                Err(e) => {
                    log::error!("Failed to accept connection: {}", e);
                    thread::sleep(Duration::from_millis(100));
                }
            }
        }
    }

    /// Get number of connected clients
    pub fn client_count(&self) -> usize {
        self.clients.lock().map(|c| c.len()).unwrap_or(0)
    }

    /// Publish a pose message to all connected clients.
    pub fn publish_pose(&self, pose: &Pose2D, timestamp_us: u64) {
        let msg = proto::dhruva::DhruvaMessage {
            topic: "odometry/pose".to_string(),
            payload: Some(proto::dhruva::dhruva_message::Payload::OdometryPose(
                proto::dhruva::OdometryPose {
                    x: pose.x,
                    y: pose.y,
                    theta: pose.theta,
                    timestamp_us,
                },
            )),
        };
        self.broadcast_proto(&msg);
    }

    /// Publish diagnostics to all connected clients.
    pub fn publish_diagnostics(&self, diagnostics: &OdometryDiagnostics) {
        let msg = proto::dhruva::DhruvaMessage {
            topic: "odometry/diagnostics".to_string(),
            payload: Some(proto::dhruva::dhruva_message::Payload::OdometryDiagnostics(
                proto::dhruva::OdometryDiagnostics {
                    drift_rate: diagnostics.drift_rate,
                    tick_rate_left: diagnostics.tick_rate_left,
                    tick_rate_right: diagnostics.tick_rate_right,
                    distance_traveled: diagnostics.distance_traveled,
                    gyro_bias: diagnostics.gyro_bias,
                },
            )),
        };
        self.broadcast_proto(&msg);
    }

    /// Publish SLAM status to all connected clients.
    pub fn publish_slam_status(&self, status: &SlamStatus) {
        let mode = match status.mode {
            crate::engine::slam::SlamMode::Mapping => "Mapping",
            crate::engine::slam::SlamMode::Localization => "Localization",
            crate::engine::slam::SlamMode::Idle => "Idle",
        };

        let msg = proto::dhruva::DhruvaMessage {
            topic: "slam/status".to_string(),
            payload: Some(proto::dhruva::dhruva_message::Payload::SlamStatus(
                proto::dhruva::SlamStatus {
                    mode: mode.to_string(),
                    num_scans: status.num_scans,
                    num_keyframes: status.num_keyframes as u32,
                    num_submaps: status.num_submaps as u32,
                    num_finished_submaps: status.num_finished_submaps as u32,
                    match_score: status.last_match_score,
                    is_lost: status.is_lost,
                    memory_usage_bytes: status.memory_usage as u64,
                },
            )),
        };
        self.broadcast_proto(&msg);
    }

    /// Publish SLAM map to all connected clients.
    pub fn publish_slam_map(&self, grid: &OccupancyGrid, timestamp_us: u64) {
        let (width, height) = grid.dimensions();
        let (origin_x, origin_y) = grid.origin();
        let config = grid.config();

        // Convert log-odds to u8 encoding
        let mut cells_u8 = Vec::with_capacity(width * height);
        for y in 0..height {
            for x in 0..width {
                let log_odds = grid.get_log_odds(x, y);
                let cell_value = Self::log_odds_to_u8(log_odds, config);
                cells_u8.push(cell_value);
            }
        }

        let msg = proto::dhruva::DhruvaMessage {
            topic: "slam/map".to_string(),
            payload: Some(proto::dhruva::dhruva_message::Payload::SlamMap(
                proto::dhruva::SlamMap {
                    resolution: config.resolution,
                    width: width as u32,
                    height: height as u32,
                    origin_x,
                    origin_y,
                    cells: cells_u8,
                    timestamp_us,
                },
            )),
        };
        self.broadcast_proto(&msg);
    }

    /// Convert log-odds value to u8 cell encoding.
    fn log_odds_to_u8(
        log_odds: f32,
        config: &crate::algorithms::mapping::OccupancyGridConfig,
    ) -> u8 {
        if log_odds.abs() < 0.01 {
            0
        } else if log_odds <= config.free_threshold {
            let confidence = (-log_odds / config.log_odds_min.abs()).clamp(0.0, 1.0);
            (1.0 + confidence * 126.0) as u8
        } else if log_odds >= config.occupied_threshold {
            let confidence = (log_odds / config.log_odds_max).clamp(0.0, 1.0);
            (128.0 + confidence * 127.0) as u8
        } else {
            0
        }
    }

    /// Publish SLAM scan to all connected clients.
    pub fn publish_slam_scan(&self, scan: &PointCloud2D, pose: &Pose2D, timestamp_us: u64) {
        let cos_theta = pose.theta.cos();
        let sin_theta = pose.theta.sin();

        // Transform points to global frame
        let points: Vec<proto::dhruva::Point2D> = scan
            .points
            .iter()
            .map(|p| {
                let global_x = pose.x + p.x * cos_theta - p.y * sin_theta;
                let global_y = pose.y + p.x * sin_theta + p.y * cos_theta;
                proto::dhruva::Point2D {
                    x: global_x,
                    y: global_y,
                }
            })
            .collect();

        let msg = proto::dhruva::DhruvaMessage {
            topic: "slam/scan".to_string(),
            payload: Some(proto::dhruva::dhruva_message::Payload::SlamScan(
                proto::dhruva::SlamScan {
                    points,
                    pose: Some(proto::dhruva::Pose2D {
                        x: pose.x,
                        y: pose.y,
                        theta: pose.theta,
                    }),
                    timestamp_us,
                },
            )),
        };
        self.broadcast_proto(&msg);
    }

    /// Publish SLAM diagnostics to all connected clients.
    pub fn publish_slam_diagnostics(&self, diagnostics: &SlamDiagnosticsMessage) {
        let msg = proto::dhruva::DhruvaMessage {
            topic: "slam/diagnostics".to_string(),
            payload: Some(proto::dhruva::dhruva_message::Payload::SlamDiagnostics(
                proto::dhruva::SlamDiagnostics {
                    timestamp_us: diagnostics.timestamp_us,
                    timing: Some(proto::dhruva::TimingBreakdown {
                        total_us: diagnostics.timing.total_us,
                        preprocessing_us: diagnostics.timing.preprocessing_us,
                        scan_matching_us: diagnostics.timing.scan_matching_us,
                        map_update_us: diagnostics.timing.map_update_us,
                        particle_filter_us: diagnostics.timing.particle_filter_us,
                        keyframe_check_us: diagnostics.timing.keyframe_check_us,
                        avg_total_us: diagnostics.timing.avg_total_us,
                    }),
                    scan_match: Some(proto::dhruva::ScanMatchStats {
                        method: diagnostics.scan_match.method.clone(),
                        iterations: diagnostics.scan_match.iterations,
                        score: diagnostics.scan_match.score,
                        mse: diagnostics.scan_match.mse,
                        converged: diagnostics.scan_match.converged,
                        correspondences: diagnostics.scan_match.correspondences as u32,
                        inlier_ratio: diagnostics.scan_match.inlier_ratio,
                    }),
                    mapping: Some(proto::dhruva::MappingStats {
                        cells_updated: diagnostics.mapping.cells_updated,
                        map_size_bytes: diagnostics.mapping.map_size_bytes as u64,
                        occupied_cells: diagnostics.mapping.occupied_cells,
                        free_cells: diagnostics.mapping.free_cells,
                        active_submap_id: diagnostics.mapping.active_submap_id,
                        num_submaps: diagnostics.mapping.num_submaps as u32,
                    }),
                    loop_closure: Some(proto::dhruva::LoopClosureStats {
                        candidates_evaluated: diagnostics.loop_closure.candidates_evaluated,
                        closures_accepted: diagnostics.loop_closure.closures_accepted,
                        last_closure_score: diagnostics.loop_closure.last_closure_score,
                        pose_graph_nodes: diagnostics.loop_closure.pose_graph_nodes as u32,
                        pose_graph_edges: diagnostics.loop_closure.pose_graph_edges as u32,
                    }),
                },
            )),
        };
        self.broadcast_proto(&msg);
    }

    /// Broadcast a protobuf message to all connected clients.
    fn broadcast_proto(&self, msg: &proto::dhruva::DhruvaMessage) {
        let mut buf = Vec::with_capacity(msg.encoded_len());
        if let Err(e) = msg.encode(&mut buf) {
            log::error!("Failed to encode protobuf message: {}", e);
            return;
        }
        self.broadcast_bytes(&buf);
    }

    /// Broadcast raw bytes to all connected clients.
    ///
    /// Removes clients that fail to receive the message.
    fn broadcast_bytes(&self, bytes: &[u8]) {
        let len_bytes = (bytes.len() as u32).to_be_bytes();

        let mut clients = match self.clients.lock() {
            Ok(c) => c,
            Err(e) => {
                log::error!("Failed to lock clients: {}", e);
                return;
            }
        };

        // Track indices of failed clients
        let mut failed_indices = Vec::new();

        for (i, client) in clients.iter_mut().enumerate() {
            if let Err(e) = client.stream.write_all(&len_bytes) {
                log::debug!("Client {} write failed (len): {}", client.addr, e);
                failed_indices.push(i);
                continue;
            }

            if let Err(e) = client.stream.write_all(bytes) {
                log::debug!("Client {} write failed (payload): {}", client.addr, e);
                failed_indices.push(i);
            }
        }

        // Remove failed clients (in reverse order to preserve indices)
        for i in failed_indices.into_iter().rev() {
            let client = clients.remove(i);
            log::info!("Removed disconnected client: {}", client.addr);
        }
    }

    /// Stop the publisher and close all connections.
    pub fn stop(&mut self) {
        self.running.store(false, Ordering::Relaxed);

        // Clear all clients
        if let Ok(mut clients) = self.clients.lock() {
            clients.clear();
        }

        // Wait for listener thread to finish
        if let Some(handle) = self.listener_handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for OdometryPublisher {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_odometry_pose_creation() {
        let pose = Pose2D::new(1.234, 0.567, 0.785);
        let odom_pose = OdometryPose::from_pose(&pose, 1234567890);
        assert_eq!(odom_pose.x, 1.234);
        assert_eq!(odom_pose.y, 0.567);
        assert_eq!(odom_pose.theta, 0.785);
        assert_eq!(odom_pose.timestamp_us, 1234567890);
    }

    #[test]
    fn test_diagnostics_default() {
        let diag = OdometryDiagnostics::default();
        assert_eq!(diag.drift_rate, 0.0);
        assert_eq!(diag.distance_traveled, 0.0);
    }
}
