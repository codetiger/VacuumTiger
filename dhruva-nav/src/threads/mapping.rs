//! Mapping thread: Scan matching and map updates.
//!
//! This thread receives lidar scans from the sensor thread and:
//! - Performs scan matching for pose correction
//! - Updates the occupancy grid map
//! - Copies the grid to shared storage for exploration

use std::sync::Arc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use vastu_slam::matching::CorrelativeMatcherConfig;
use vastu_slam::{LidarScan, MapConfig, OccupancyGridMap, Pose2D};

use crate::shared::{SharedGrid, SharedState, SharedTrajectory, messages::LidarScanMsg};

/// Mapping thread state and logic.
pub struct MappingThread {
    shared_state: Arc<SharedState>,
    shared_grid: SharedGrid,
    shared_trajectory: SharedTrajectory,
    lidar_rx: Receiver<LidarScanMsg>,
    map: OccupancyGridMap,
    matcher_config: CorrelativeMatcherConfig,
    trajectory: Vec<Pose2D>,
    scan_count: usize,
    min_match_score: f32,
    last_grid_sync: Instant,
    grid_sync_interval: Duration,
}

impl MappingThread {
    /// Create a new mapping thread.
    pub fn new(
        shared_state: Arc<SharedState>,
        shared_grid: SharedGrid,
        shared_trajectory: SharedTrajectory,
        lidar_rx: Receiver<LidarScanMsg>,
    ) -> Self {
        // Initialize map with default configuration
        let map_config = MapConfig::default();

        // Use a balanced matcher config: better accuracy than fast() but still real-time
        // The fast() config (3cm resolution, 180 points) causes map smearing
        let matcher_config = CorrelativeMatcherConfig {
            search_x: 0.20,           // 20cm search window
            search_y: 0.20,           // 20cm search window
            search_theta: 0.20,       // ~11 degrees
            linear_resolution: 0.02,  // 2cm steps (finer than fast's 3cm)
            angular_resolution: 0.02, // ~1.1 degrees (finer than fast's 3cm)
            max_points: 0,            // Use all points for better accuracy
            // Note: sensor_offset is (0,0) - SangamIO now sends lidar data
            // already transformed to robot center coordinates
            sensor_offset: (0.0, 0.0),
            ..CorrelativeMatcherConfig::default()
        };

        Self {
            shared_state,
            shared_grid,
            shared_trajectory,
            lidar_rx,
            map: OccupancyGridMap::new(map_config),
            matcher_config,
            trajectory: Vec::new(),
            scan_count: 0,
            min_match_score: 0.55, // Slightly higher threshold
            last_grid_sync: Instant::now(),
            grid_sync_interval: Duration::from_millis(100), // Sync grid every 100ms
        }
    }

    /// Run the mapping thread main loop.
    pub fn run(&mut self) {
        tracing::info!("Mapping thread started");

        loop {
            // Check for shutdown
            if self.shared_state.should_shutdown() {
                tracing::info!("Mapping thread shutting down");
                // Final grid sync before shutdown
                self.sync_grid();
                break;
            }

            // Receive lidar scan with timeout
            match self.lidar_rx.recv_timeout(Duration::from_millis(100)) {
                Ok(scan_msg) => {
                    self.process_scan(scan_msg);
                }
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    // No scan available, sync grid if needed
                    if self.last_grid_sync.elapsed() >= self.grid_sync_interval {
                        self.sync_grid();
                    }
                    continue;
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                    tracing::warn!("Lidar channel disconnected, mapping thread exiting");
                    break;
                }
            }

            // Sync grid periodically
            if self.last_grid_sync.elapsed() >= self.grid_sync_interval {
                self.sync_grid();
            }
        }
    }

    /// Process a lidar scan.
    fn process_scan(&mut self, scan_msg: LidarScanMsg) {
        let scan = to_lidar_scan(&scan_msg.points);
        let encoder_pose = scan_msg.encoder_pose;

        self.scan_count += 1;
        self.shared_state.increment_scan_count();

        // For the first few scans, seed the map without matching
        let corrected_pose = if self.scan_count <= 5 {
            self.map.observe_lidar(&scan, encoder_pose);
            encoder_pose
        } else {
            // Use scan matching for pose correction
            let (_result, corrected, match_result) =
                self.map
                    .observe_lidar_with_matching(&scan, encoder_pose, &self.matcher_config);

            if match_result.converged && match_result.score >= self.min_match_score {
                tracing::trace!(
                    "Scan match accepted: score={:.3}, pose=({:.3}, {:.3})",
                    match_result.score,
                    corrected.x,
                    corrected.y
                );
                corrected
            } else {
                tracing::debug!(
                    "Scan match rejected: score={:.3}, converged={}",
                    match_result.score,
                    match_result.converged
                );
                encoder_pose
            }
        };

        // Apply correction: store corrected pose and the odometry at correction time
        // This allows pose() to apply odometry deltas between corrections
        self.shared_state
            .apply_correction(corrected_pose, encoder_pose);
        self.trajectory.push(corrected_pose);

        // Push to shared trajectory for SVG visualization
        if let Ok(mut traj) = self.shared_trajectory.write() {
            traj.push(corrected_pose);
        }
    }

    /// Sync the internal grid to shared storage.
    fn sync_grid(&mut self) {
        // Clone the grid storage and update shared state
        if let Ok(mut grid) = self.shared_grid.write() {
            *grid = self.map.storage().clone();
        }
        self.last_grid_sync = Instant::now();
    }
}

/// Convert lidar points to VastuSLAM LidarScan.
fn to_lidar_scan(points: &[(f32, f32, u8)]) -> LidarScan {
    let angles: Vec<f32> = points.iter().map(|(a, _, _)| *a).collect();
    let ranges: Vec<f32> = points.iter().map(|(_, d, _)| *d).collect();
    LidarScan::new(ranges, angles, 0.15, 8.0)
}
