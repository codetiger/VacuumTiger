//! SLAM message types for TCP streaming.
//!
//! Defines message structures for streaming SLAM data (status, map, scan, diagnostics)
//! and receiving SLAM commands from visualization clients.

use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use serde::{Deserialize, Serialize};

use crate::algorithms::mapping::OccupancyGrid;
use crate::core::types::{PointCloud2D, Pose2D};
use crate::engine::slam::{SlamMode, SlamStatus};

// ============================================================================
// SLAM Diagnostics Messages
// ============================================================================

/// SLAM diagnostics message payload (5 Hz).
///
/// Provides detailed timing and statistics for algorithm debugging and tuning.
#[derive(Debug, Clone, Serialize, Default)]
pub struct SlamDiagnosticsMessage {
    /// Timestamp in microseconds.
    pub timestamp_us: u64,
    /// Timing breakdown for each component.
    pub timing: TimingBreakdown,
    /// Scan matching statistics.
    pub scan_match: ScanMatchStats,
    /// Particle filter statistics (if applicable).
    pub particle_filter: Option<ParticleFilterStats>,
    /// Mapping statistics.
    pub mapping: MappingStats,
    /// Loop closure statistics.
    pub loop_closure: LoopClosureStats,
}

/// Timing breakdown for SLAM components.
#[derive(Debug, Clone, Serialize, Default)]
pub struct TimingBreakdown {
    /// Total SLAM cycle time in microseconds.
    pub total_us: u64,
    /// Scan preprocessing time in microseconds.
    pub preprocessing_us: u64,
    /// Scan matching time in microseconds.
    pub scan_matching_us: u64,
    /// Map update time in microseconds.
    pub map_update_us: u64,
    /// Particle filter time in microseconds (if applicable).
    pub particle_filter_us: u64,
    /// Keyframe check time in microseconds.
    pub keyframe_check_us: u64,
    /// Running average of total time in microseconds.
    pub avg_total_us: f32,
}

/// Scan matching statistics.
#[derive(Debug, Clone, Serialize, Default)]
pub struct ScanMatchStats {
    /// Matcher type: "icp", "correlative", "hybrid", or "none".
    pub method: String,
    /// Number of iterations (for ICP).
    pub iterations: u32,
    /// Match quality score (0.0 - 1.0).
    pub score: f32,
    /// Mean squared error of final alignment.
    pub mse: f32,
    /// Whether the matcher converged.
    pub converged: bool,
    /// Number of point correspondences found.
    pub correspondences: usize,
    /// Ratio of inliers to total correspondences.
    pub inlier_ratio: f32,
}

/// Particle filter statistics for MCL localization.
#[derive(Debug, Clone, Serialize, Default)]
pub struct ParticleFilterStats {
    /// Number of particles.
    pub num_particles: u32,
    /// Effective sample size (measure of particle diversity).
    pub neff: f32,
    /// Maximum particle weight.
    pub max_weight: f32,
    /// Whether resampling occurred this step.
    pub resampled: bool,
    /// Particle spread in XY plane (standard deviation).
    pub spread_xy: f32,
    /// Particle spread in theta (standard deviation).
    pub spread_theta: f32,
}

/// Mapping statistics.
#[derive(Debug, Clone, Serialize, Default)]
pub struct MappingStats {
    /// Number of cells updated this cycle.
    pub cells_updated: u32,
    /// Total map memory usage in bytes.
    pub map_size_bytes: usize,
    /// Number of occupied cells.
    pub occupied_cells: u32,
    /// Number of free cells.
    pub free_cells: u32,
    /// Active submap ID (if any).
    pub active_submap_id: Option<u64>,
    /// Total number of submaps.
    pub num_submaps: usize,
}

/// Loop closure statistics.
#[derive(Debug, Clone, Serialize, Default)]
pub struct LoopClosureStats {
    /// Number of loop closure candidates evaluated.
    pub candidates_evaluated: u32,
    /// Number of loop closures accepted.
    pub closures_accepted: u32,
    /// Score of the last accepted loop closure.
    pub last_closure_score: f32,
    /// Number of nodes in the pose graph.
    pub pose_graph_nodes: usize,
    /// Number of edges in the pose graph.
    pub pose_graph_edges: usize,
}

// ============================================================================
// Original SLAM Messages
// ============================================================================

/// SLAM status message payload (5 Hz).
#[derive(Debug, Clone, Serialize)]
pub struct SlamStatusMessage {
    /// Operating mode: "Mapping", "Localization", or "Idle"
    pub mode: String,
    /// Total scans processed
    pub num_scans: u64,
    /// Number of keyframes created
    pub num_keyframes: usize,
    /// Number of submaps
    pub num_submaps: usize,
    /// Number of finished submaps
    pub num_finished_submaps: usize,
    /// Last scan match quality score (0.0 - 1.0)
    pub match_score: f32,
    /// Whether the robot is considered "lost"
    pub is_lost: bool,
    /// Memory usage in bytes
    pub memory_usage_bytes: usize,
}

impl SlamStatusMessage {
    /// Create a status message from SlamStatus.
    pub fn from_status(status: &SlamStatus) -> Self {
        let mode = match status.mode {
            SlamMode::Mapping => "Mapping",
            SlamMode::Localization => "Localization",
            SlamMode::Idle => "Idle",
        };

        Self {
            mode: mode.to_string(),
            num_scans: status.num_scans,
            num_keyframes: status.num_keyframes,
            num_submaps: status.num_submaps,
            num_finished_submaps: status.num_finished_submaps,
            match_score: status.last_match_score,
            is_lost: status.is_lost,
            memory_usage_bytes: status.memory_usage,
        }
    }
}

/// SLAM occupancy grid map message payload (1 Hz or on-demand).
#[derive(Debug, Clone, Serialize)]
pub struct SlamMapMessage {
    /// Cell resolution in meters per cell
    pub resolution: f32,
    /// Grid width in cells
    pub width: usize,
    /// Grid height in cells
    pub height: usize,
    /// World X coordinate of cell (0, 0)
    pub origin_x: f32,
    /// World Y coordinate of cell (0, 0)
    pub origin_y: f32,
    /// Base64-encoded cell data (u8 array)
    ///
    /// Cell encoding:
    /// - 0 = unknown
    /// - 1-127 = free (higher = more confident)
    /// - 128-255 = occupied (higher = more confident)
    pub cells: String,
    /// Timestamp in microseconds
    pub timestamp_us: u64,
}

impl SlamMapMessage {
    /// Create a map message from OccupancyGrid.
    ///
    /// Converts log-odds values to u8 confidence encoding for compact transmission.
    pub fn from_grid(grid: &OccupancyGrid, timestamp_us: u64) -> Self {
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

        // Base64 encode
        let cells = BASE64.encode(&cells_u8);

        Self {
            resolution: config.resolution,
            width,
            height,
            origin_x,
            origin_y,
            cells,
            timestamp_us,
        }
    }

    /// Convert log-odds value to u8 cell encoding.
    fn log_odds_to_u8(
        log_odds: f32,
        config: &crate::algorithms::mapping::OccupancyGridConfig,
    ) -> u8 {
        if log_odds.abs() < 0.01 {
            // Unknown (near zero log-odds)
            0
        } else if log_odds <= config.free_threshold {
            // Free: map to 1-127 based on confidence
            let confidence = (-log_odds / config.log_odds_min.abs()).clamp(0.0, 1.0);
            (1.0 + confidence * 126.0) as u8
        } else if log_odds >= config.occupied_threshold {
            // Occupied: map to 128-255 based on confidence
            let confidence = (log_odds / config.log_odds_max).clamp(0.0, 1.0);
            (128.0 + confidence * 127.0) as u8
        } else {
            // Unknown (between thresholds)
            0
        }
    }
}

/// SLAM lidar scan message payload (5 Hz).
#[derive(Debug, Clone, Serialize)]
pub struct SlamScanMessage {
    /// Scan points in global frame: [[x1, y1], [x2, y2], ...]
    pub points: Vec<[f32; 2]>,
    /// Robot pose when scan was taken: [x, y, theta]
    pub pose: [f32; 3],
    /// Timestamp in microseconds
    pub timestamp_us: u64,
}

impl SlamScanMessage {
    /// Create a scan message from PointCloud2D and Pose2D.
    ///
    /// Transforms scan points to global frame using the provided pose.
    pub fn from_scan(scan: &PointCloud2D, pose: &Pose2D, timestamp_us: u64) -> Self {
        let cos_theta = pose.theta.cos();
        let sin_theta = pose.theta.sin();

        // Transform points to global frame
        let points: Vec<[f32; 2]> = scan
            .points
            .iter()
            .map(|p| {
                let global_x = pose.x + p.x * cos_theta - p.y * sin_theta;
                let global_y = pose.y + p.x * sin_theta + p.y * cos_theta;
                [global_x, global_y]
            })
            .collect();

        Self {
            points,
            pose: [pose.x, pose.y, pose.theta],
            timestamp_us,
        }
    }

    /// Create a scan message with points already in global frame.
    pub fn from_global_points(points: Vec<[f32; 2]>, pose: &Pose2D, timestamp_us: u64) -> Self {
        Self {
            points,
            pose: [pose.x, pose.y, pose.theta],
            timestamp_us,
        }
    }
}

/// SLAM command received from clients.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "command")]
pub enum SlamCommand {
    /// Set SLAM operating mode
    #[serde(rename = "set_mode")]
    SetMode {
        /// Target mode: "Mapping", "Localization", or "Idle"
        mode: String,
    },

    /// Reset SLAM (clear map, reset pose to origin)
    #[serde(rename = "reset")]
    Reset,

    /// Reset pose to specific location
    #[serde(rename = "reset_pose")]
    ResetPose {
        /// X position in meters
        x: f32,
        /// Y position in meters
        y: f32,
        /// Heading in radians
        theta: f32,
    },

    /// Save map to file
    #[serde(rename = "save_map")]
    SaveMap {
        /// File path to save map
        path: String,
    },
}

impl SlamCommand {
    /// Parse SLAM mode string to SlamMode enum.
    pub fn parse_mode(mode: &str) -> Option<SlamMode> {
        match mode.to_lowercase().as_str() {
            "mapping" => Some(SlamMode::Mapping),
            "localization" => Some(SlamMode::Localization),
            "idle" => Some(SlamMode::Idle),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::mapping::OccupancyGridConfig;

    #[test]
    fn test_status_message_serialization() {
        let status = SlamStatus {
            mode: SlamMode::Mapping,
            num_scans: 1234,
            num_keyframes: 45,
            num_submaps: 3,
            num_finished_submaps: 2,
            last_match_score: 0.78,
            is_lost: false,
            memory_usage: 5242880,
        };

        let msg = SlamStatusMessage::from_status(&status);
        let json = serde_json::to_string(&msg).unwrap();

        assert!(json.contains("\"mode\":\"Mapping\""));
        assert!(json.contains("\"num_scans\":1234"));
        assert!(json.contains("\"match_score\":0.78"));
    }

    #[test]
    fn test_map_message_encoding() {
        let config = OccupancyGridConfig {
            resolution: 0.05,
            initial_width: 1.0,
            initial_height: 1.0,
            ..Default::default()
        };
        let grid = OccupancyGrid::new(config);

        let msg = SlamMapMessage::from_grid(&grid, 1234567890);

        assert_eq!(msg.resolution, 0.05);
        assert_eq!(msg.width, 20); // 1.0m / 0.05m = 20 cells
        assert_eq!(msg.height, 20);
        assert!(!msg.cells.is_empty());

        // Verify base64 decoding works
        let decoded = BASE64.decode(&msg.cells).unwrap();
        assert_eq!(decoded.len(), 20 * 20);
    }

    #[test]
    fn test_scan_message_transform() {
        use crate::core::types::Point2D;

        let mut scan = PointCloud2D::new();
        scan.push(Point2D::new(1.0, 0.0)); // Point at (1, 0) in robot frame

        let pose = Pose2D::new(2.0, 3.0, std::f32::consts::FRAC_PI_2); // 90 degrees

        let msg = SlamScanMessage::from_scan(&scan, &pose, 1234567890);

        // Point (1, 0) rotated 90 degrees becomes (0, 1), then translated by (2, 3)
        assert_eq!(msg.points.len(), 1);
        assert!((msg.points[0][0] - 2.0).abs() < 0.01);
        assert!((msg.points[0][1] - 4.0).abs() < 0.01);
    }

    #[test]
    fn test_command_parsing() {
        let json = r#"{"command": "set_mode", "mode": "Localization"}"#;
        let cmd: SlamCommand = serde_json::from_str(json).unwrap();

        match cmd {
            SlamCommand::SetMode { mode } => {
                assert_eq!(mode, "Localization");
                assert_eq!(SlamCommand::parse_mode(&mode), Some(SlamMode::Localization));
            }
            _ => panic!("Expected SetMode command"),
        }
    }

    #[test]
    fn test_reset_command_parsing() {
        let json = r#"{"command": "reset"}"#;
        let cmd: SlamCommand = serde_json::from_str(json).unwrap();

        assert!(matches!(cmd, SlamCommand::Reset));
    }

    #[test]
    fn test_save_map_command_parsing() {
        let json = r#"{"command": "save_map", "path": "/tmp/map.bin"}"#;
        let cmd: SlamCommand = serde_json::from_str(json).unwrap();

        match cmd {
            SlamCommand::SaveMap { path } => {
                assert_eq!(path, "/tmp/map.bin");
            }
            _ => panic!("Expected SaveMap command"),
        }
    }
}
