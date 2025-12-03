//! TCP publisher for streaming odometry data to visualization clients.
//!
//! Broadcasts pose and diagnostics messages to connected TCP clients using
//! length-prefixed JSON framing (compatible with SangamIO protocol).
//!
//! # Wire Protocol
//!
//! ```text
//! ┌──────────────────┬──────────────────────────┐
//! │ Length (4 bytes) │ JSON Payload (variable)  │
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
use serde::Serialize;
use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use thiserror::Error;

use super::slam_messages::{
    SlamDiagnosticsMessage, SlamMapMessage, SlamScanMessage, SlamStatusMessage,
};

/// Publisher errors
#[derive(Error, Debug)]
pub enum PublisherError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),
}

pub type Result<T> = std::result::Result<T, PublisherError>;

/// Odometry pose message payload
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

/// Message types for SLAM streaming
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "topic", content = "payload")]
pub enum SlamMessage {
    #[serde(rename = "slam/status")]
    Status(SlamStatusMessage),

    #[serde(rename = "slam/map")]
    Map(SlamMapMessage),

    #[serde(rename = "slam/scan")]
    Scan(SlamScanMessage),

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
        let msg = OdometryMessage::Pose(OdometryPose::from_pose(pose, timestamp_us));
        self.broadcast_odometry(&msg);
    }

    /// Publish diagnostics to all connected clients.
    pub fn publish_diagnostics(&self, diagnostics: &OdometryDiagnostics) {
        let msg = OdometryMessage::Diagnostics(diagnostics.clone());
        self.broadcast_odometry(&msg);
    }

    /// Publish SLAM status to all connected clients.
    pub fn publish_slam_status(&self, status: &SlamStatus) {
        let msg = SlamMessage::Status(SlamStatusMessage::from_status(status));
        self.broadcast_slam(&msg);
    }

    /// Publish SLAM map to all connected clients.
    pub fn publish_slam_map(&self, grid: &OccupancyGrid, timestamp_us: u64) {
        let msg = SlamMessage::Map(SlamMapMessage::from_grid(grid, timestamp_us));
        self.broadcast_slam(&msg);
    }

    /// Publish SLAM scan to all connected clients.
    pub fn publish_slam_scan(&self, scan: &PointCloud2D, pose: &Pose2D, timestamp_us: u64) {
        let msg = SlamMessage::Scan(SlamScanMessage::from_scan(scan, pose, timestamp_us));
        self.broadcast_slam(&msg);
    }

    /// Publish SLAM diagnostics to all connected clients.
    pub fn publish_slam_diagnostics(&self, diagnostics: &SlamDiagnosticsMessage) {
        let msg = SlamMessage::Diagnostics(diagnostics.clone());
        self.broadcast_slam(&msg);
    }

    /// Broadcast an odometry message to all connected clients.
    fn broadcast_odometry(&self, msg: &OdometryMessage) {
        let bytes = match serde_json::to_vec(msg) {
            Ok(b) => b,
            Err(e) => {
                log::error!("Failed to serialize message: {}", e);
                return;
            }
        };
        self.broadcast_bytes(&bytes);
    }

    /// Broadcast a SLAM message to all connected clients.
    fn broadcast_slam(&self, msg: &SlamMessage) {
        let bytes = match serde_json::to_vec(msg) {
            Ok(b) => b,
            Err(e) => {
                log::error!("Failed to serialize message: {}", e);
                return;
            }
        };
        self.broadcast_bytes(&bytes);
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
    fn test_odometry_pose_serialization() {
        let msg = OdometryMessage::Pose(OdometryPose {
            x: 1.234,
            y: 0.567,
            theta: 0.785,
            timestamp_us: 1234567890,
        });

        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("odometry/pose"));
        assert!(json.contains("1.234"));
    }

    #[test]
    fn test_diagnostics_serialization() {
        let msg = OdometryMessage::Diagnostics(OdometryDiagnostics {
            drift_rate: 0.02,
            tick_rate_left: 450.0,
            tick_rate_right: 448.0,
            distance_traveled: 5.67,
            gyro_bias: 0.001,
        });

        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("odometry/diagnostics"));
        assert!(json.contains("drift_rate"));
    }

    #[test]
    fn test_slam_status_serialization() {
        let msg = SlamMessage::Status(SlamStatusMessage {
            mode: "Mapping".to_string(),
            num_scans: 1234,
            num_keyframes: 45,
            num_submaps: 3,
            num_finished_submaps: 2,
            match_score: 0.78,
            is_lost: false,
            memory_usage_bytes: 5242880,
        });

        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("slam/status"));
        assert!(json.contains("\"mode\":\"Mapping\""));
        assert!(json.contains("\"num_scans\":1234"));
    }

    #[test]
    fn test_slam_scan_serialization() {
        let msg = SlamMessage::Scan(SlamScanMessage {
            points: vec![[1.0, 2.0], [3.0, 4.0]],
            pose: [1.234, 0.567, 0.785],
            timestamp_us: 1234567890,
        });

        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("slam/scan"));
        assert!(json.contains("points"));
        assert!(json.contains("pose"));
    }
}
