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
use std::sync::mpsc::{self, SyncSender, TrySendError};
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

/// Maximum number of pending messages per client before dropping
const CLIENT_QUEUE_SIZE: usize = 16;

/// Client connection state with async write channel
struct Client {
    /// Sender channel for queueing messages to write
    sender: SyncSender<Arc<Vec<u8>>>,
    /// Client address for logging
    addr: std::net::SocketAddr,
    /// Flag to indicate if the client writer thread is still alive
    alive: Arc<AtomicBool>,
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

        log::info!("Spawning accept thread for port {}...", port);
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
        log::info!("Accept loop started, waiting for connections...");
        while running.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, addr)) => {
                    log::info!("New odometry client connected: {}", addr);

                    // IMPORTANT: Set client socket to blocking mode
                    // (listener is non-blocking, but accepted sockets inherit this)
                    if let Err(e) = stream.set_nonblocking(false) {
                        log::warn!("Failed to set blocking mode: {}", e);
                    }

                    // Set TCP_NODELAY for low latency
                    if let Err(e) = stream.set_nodelay(true) {
                        log::warn!("Failed to set TCP_NODELAY: {}", e);
                    }

                    // Set write timeout (10s to handle large map messages ~1MB)
                    if let Err(e) = stream.set_write_timeout(Some(Duration::from_secs(10))) {
                        log::warn!("Failed to set write timeout: {}", e);
                    }

                    // Create channel for async writes
                    let (sender, receiver) = mpsc::sync_channel::<Arc<Vec<u8>>>(CLIENT_QUEUE_SIZE);
                    let alive = Arc::new(AtomicBool::new(true));
                    let alive_clone = alive.clone();
                    let addr_clone = addr;

                    // Spawn writer thread for this client
                    thread::spawn(move || {
                        Self::client_writer_loop(stream, receiver, alive_clone, addr_clone);
                    });

                    match clients.lock() {
                        Ok(mut c) => {
                            c.push(Client {
                                sender,
                                addr,
                                alive,
                            });
                            log::info!("Client added, total clients: {}", c.len());
                        }
                        Err(e) => {
                            log::error!("Failed to lock clients mutex: {}", e);
                        }
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
        log::info!("Accept loop exiting");
    }

    /// Writer loop for a single client - runs in its own thread
    fn client_writer_loop(
        mut stream: TcpStream,
        receiver: mpsc::Receiver<Arc<Vec<u8>>>,
        alive: Arc<AtomicBool>,
        addr: std::net::SocketAddr,
    ) {
        log::debug!("Writer thread started for client {}", addr);

        while let Ok(data) = receiver.recv() {
            let len_bytes = (data.len() as u32).to_be_bytes();

            // Write length prefix
            if let Err(e) = stream.write_all(&len_bytes) {
                log::warn!("Client {} write failed (len): {}", addr, e);
                break;
            }

            // Write payload
            if let Err(e) = stream.write_all(&data) {
                log::warn!(
                    "Client {} write failed (payload, {} bytes): {}",
                    addr,
                    data.len(),
                    e
                );
                break;
            }
        }

        // Mark client as dead so it gets cleaned up
        alive.store(false, Ordering::Relaxed);
        log::info!("Writer thread exiting for client {}", addr);
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
        let points: Vec<proto::dhruva::Point2D> = (0..scan.len())
            .map(|i| {
                let global_x = pose.x + scan.xs[i] * cos_theta - scan.ys[i] * sin_theta;
                let global_y = pose.y + scan.xs[i] * sin_theta + scan.ys[i] * cos_theta;
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

    /// Publish map features (lines and corners) to all connected clients.
    pub fn publish_features(
        &self,
        features: &crate::algorithms::mapping::MapFeatures,
        timestamp_us: u64,
    ) {
        let lines: Vec<proto::dhruva::LineSegment> = features
            .lines
            .iter()
            .map(|line| proto::dhruva::LineSegment {
                start_x: line.start.x,
                start_y: line.start.y,
                end_x: line.end.x,
                end_y: line.end.y,
                length: line.length,
                quality: line.quality,
                point_count: line.point_count as u32,
            })
            .collect();

        let corners: Vec<proto::dhruva::Corner> = features
            .corners
            .iter()
            .map(|corner| proto::dhruva::Corner {
                x: corner.position.x,
                y: corner.position.y,
                angle: corner.angle,
                line1_idx: corner.line1_idx as u32,
                line2_idx: corner.line2_idx as u32,
                corner_type: format!("{:?}", corner.corner_type),
            })
            .collect();

        let msg = proto::dhruva::DhruvaMessage {
            topic: "slam/features".to_string(),
            payload: Some(proto::dhruva::dhruva_message::Payload::MapFeatures(
                proto::dhruva::MapFeatures {
                    lines,
                    corners,
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

    /// Broadcast raw bytes to all connected clients (non-blocking).
    ///
    /// Messages are queued for each client's writer thread. If a client's
    /// queue is full, the message is dropped for that client (backpressure).
    /// Dead clients are removed automatically.
    fn broadcast_bytes(&self, bytes: &[u8]) {
        // Wrap bytes in Arc so we can share across clients without copying
        let data = Arc::new(bytes.to_vec());

        let mut clients = match self.clients.lock() {
            Ok(c) => c,
            Err(e) => {
                log::error!("Failed to lock clients: {}", e);
                return;
            }
        };

        // Track indices of dead clients to remove
        let mut dead_indices = Vec::new();

        for (i, client) in clients.iter().enumerate() {
            // Check if writer thread is still alive
            if !client.alive.load(Ordering::Relaxed) {
                dead_indices.push(i);
                continue;
            }

            // Try to send to client's queue (non-blocking)
            match client.sender.try_send(data.clone()) {
                Ok(()) => {}
                Err(TrySendError::Full(_)) => {
                    // Queue is full - drop this message for this client (backpressure)
                    log::debug!(
                        "Client {} queue full, dropping {} byte message",
                        client.addr,
                        bytes.len()
                    );
                }
                Err(TrySendError::Disconnected(_)) => {
                    // Writer thread died
                    dead_indices.push(i);
                }
            }
        }

        // Remove dead clients (in reverse order to preserve indices)
        for i in dead_indices.into_iter().rev() {
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
