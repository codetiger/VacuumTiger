//! Multi-threaded architecture for DhruvaNav.
//!
//! Separates concerns into three threads:
//! - Sensor thread: High-frequency UDP reading, odometry, safety, commands
//! - Mapping thread: Scan matching and map updates
//! - Exploration thread: Frontier detection and path planning

mod exploration;
mod mapping;
mod sensor;

pub use exploration::ExplorationThread;
pub use mapping::MappingThread;
pub use sensor::SensorThread;

use std::sync::Arc;
use std::sync::mpsc;
use std::thread::{self, JoinHandle};

use crate::config::DhruvaConfig;
use crate::error::Result;
use crate::shared::{
    DebugVisualization, SharedDebugViz, SharedGrid, SharedState, SharedTrajectory,
    messages::LidarScanMsg,
};

/// Thread handles for the multi-threaded system.
pub struct ThreadHandles {
    pub sensor: JoinHandle<()>,
    pub mapping: JoinHandle<()>,
    pub exploration: JoinHandle<()>,
    /// Shared trajectory for SVG visualization (accumulated by mapping thread)
    pub trajectory: SharedTrajectory,
    /// Shared debug visualization data (accumulated by exploration thread)
    pub debug_viz: SharedDebugViz,
}

/// Spawn all threads and return handles.
pub fn spawn_threads(
    config: DhruvaConfig,
    shared_state: Arc<SharedState>,
    shared_grid: SharedGrid,
) -> Result<ThreadHandles> {
    // Create channel for lidar scans (bounded to prevent memory growth)
    let (lidar_tx, lidar_rx) = mpsc::sync_channel::<LidarScanMsg>(10);

    // Create shared trajectory for SVG visualization
    let shared_trajectory: SharedTrajectory = Arc::new(std::sync::RwLock::new(Vec::new()));
    let mapping_trajectory = Arc::clone(&shared_trajectory);

    // Create shared debug visualization data
    let shared_debug_viz: SharedDebugViz =
        Arc::new(std::sync::RwLock::new(DebugVisualization::default()));
    let exploration_debug_viz = Arc::clone(&shared_debug_viz);

    // Clone shared state for each thread
    let sensor_state = Arc::clone(&shared_state);
    let mapping_state = Arc::clone(&shared_state);
    let exploration_state = Arc::clone(&shared_state);

    // Clone shared grid for exploration
    let exploration_grid = Arc::clone(&shared_grid);

    // Clone config parts for each thread
    let sensor_config = config.clone();
    let exploration_config = config.clone();

    // Spawn sensor thread (highest priority - real-time constraints)
    let sensor_handle = thread::Builder::new()
        .name("sensor".into())
        .spawn(move || {
            let mut sensor_thread = SensorThread::new(sensor_config, sensor_state, lidar_tx);
            if let Err(e) = sensor_thread.run() {
                tracing::error!("Sensor thread error: {}", e);
            }
        })
        .expect("Failed to spawn sensor thread");

    // Spawn mapping thread
    let mapping_handle = thread::Builder::new()
        .name("mapping".into())
        .spawn(move || {
            let mut mapping_thread =
                MappingThread::new(mapping_state, shared_grid, mapping_trajectory, lidar_rx);
            mapping_thread.run();
        })
        .expect("Failed to spawn mapping thread");

    // Spawn exploration thread
    let exploration_handle = thread::Builder::new()
        .name("exploration".into())
        .spawn(move || {
            let mut exploration_thread = ExplorationThread::new(
                exploration_config,
                exploration_state,
                exploration_grid,
                exploration_debug_viz,
            );
            exploration_thread.run();
        })
        .expect("Failed to spawn exploration thread");

    Ok(ThreadHandles {
        sensor: sensor_handle,
        mapping: mapping_handle,
        exploration: exploration_handle,
        trajectory: shared_trajectory,
        debug_viz: shared_debug_viz,
    })
}
