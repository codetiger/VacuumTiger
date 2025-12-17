//! Thread-safe shared state for multi-threaded SLAM daemon.
//!
//! This module provides `SharedState` which is shared between:
//! - SLAM Thread: Primary writer (updates pose, map, sensor status)
//! - Command Thread: Reads for responses, updates map list on save/delete
//! - Publisher Thread: Reads for streaming to clients
//!
//! Note: Some fields are defined for planned exploration features.

use std::sync::{Arc, RwLock};

use crate::algorithms::mapping::OccupancyGrid;
use crate::core::types::Pose2D;
use crate::navigation::NavigationState;

/// Robot operating state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RobotState {
    /// No active SLAM operation.
    #[default]
    Idle,
    /// Building a new map.
    Mapping,
    /// Localizing in an existing map.
    Localizing,
    /// Lost localization.
    Lost,
}

/// Summary information about a saved map.
#[derive(Debug, Clone)]
pub struct MapSummary {
    /// Unique map identifier.
    pub map_id: String,
    /// User-friendly map name.
    pub name: String,
    /// Creation timestamp in microseconds.
    pub created_at_us: u64,
    /// Last update timestamp in microseconds.
    pub updated_at_us: u64,
    /// Map area in square meters.
    pub area_m2: f32,
    /// Number of detected rooms (if segmented).
    pub room_count: u32,
    /// Whether mapping was completed.
    pub is_complete: bool,
}

/// Robot status including pose, state, and progress metrics.
#[derive(Debug, Clone)]
pub struct RobotStatus {
    /// Current pose in map frame.
    pub pose: Pose2D,
    /// Current SLAM state.
    pub state: RobotState,
    /// Active map ID (if localizing).
    pub active_map_id: Option<String>,
    /// Battery percentage (0-100).
    pub battery_percent: f32,
    /// Whether robot is charging.
    pub is_charging: bool,
    /// Whether robot is docked.
    pub is_docked: bool,
    /// Distance traveled during mapping (meters).
    pub distance_traveled_m: f32,
    /// Number of keyframes created.
    pub keyframe_count: u32,
    /// Number of loop closures detected.
    pub loop_closures: u32,
    /// Estimated map area (square meters).
    pub map_area_m2: f32,
    /// Localization confidence (0.0-1.0).
    pub localization_confidence: f32,
    /// Timestamp of last update (microseconds).
    pub timestamp_us: u64,
}

impl Default for RobotStatus {
    fn default() -> Self {
        Self {
            pose: Pose2D::identity(),
            state: RobotState::Idle,
            active_map_id: None,
            battery_percent: 100.0,
            is_charging: false,
            is_docked: false,
            distance_traveled_m: 0.0,
            keyframe_count: 0,
            loop_closures: 0,
            map_area_m2: 0.0,
            localization_confidence: 1.0,
            timestamp_us: 0,
        }
    }
}

/// Bumper state for obstacle detection.
#[derive(Debug, Clone, Default)]
pub struct BumperState {
    /// Left bumper currently pressed.
    pub left_pressed: bool,
    /// Right bumper currently pressed.
    pub right_pressed: bool,
    /// Timestamp of last bumper trigger (microseconds).
    pub last_trigger_us: Option<u64>,
}

/// Cliff sensor state for drop-off hazard detection.
#[derive(Debug, Clone, Default)]
pub struct CliffState {
    /// Left side cliff sensor triggered.
    pub left_side: bool,
    /// Left front cliff sensor triggered.
    pub left_front: bool,
    /// Right front cliff sensor triggered.
    pub right_front: bool,
    /// Right side cliff sensor triggered.
    pub right_side: bool,
    /// Timestamp of last cliff trigger (microseconds).
    pub last_trigger_us: Option<u64>,
}

impl CliffState {
    /// Check if any cliff sensor is triggered.
    pub fn any_triggered(&self) -> bool {
        self.left_side || self.left_front || self.right_front || self.right_side
    }
}

/// Sensor status for visualization.
#[derive(Debug, Clone, Default)]
pub struct SensorStatus {
    /// Left encoder ticks.
    pub left_encoder_ticks: i32,
    /// Right encoder ticks.
    pub right_encoder_ticks: i32,
    /// Left wheel velocity (m/s).
    pub left_velocity_mps: f32,
    /// Right wheel velocity (m/s).
    pub right_velocity_mps: f32,
    /// Gyro Z angular velocity (rad/s).
    pub gyro_z_radps: f32,
    /// Accelerometer X (m/s²).
    pub accel_x_mps2: f32,
    /// Accelerometer Y (m/s²).
    pub accel_y_mps2: f32,
    /// Raw odometry pose (before SLAM correction).
    pub raw_odometry: Pose2D,
    /// Latest lidar scan angles (radians).
    pub lidar_angles: Vec<f32>,
    /// Latest lidar scan ranges (meters).
    pub lidar_ranges: Vec<f32>,
    /// Timestamp of last update (microseconds).
    pub timestamp_us: u64,
    /// Bumper state.
    pub bumper: BumperState,
    /// Cliff sensor state.
    pub cliff: CliffState,
}

/// Current map data for visualization.
#[derive(Debug, Clone)]
pub struct CurrentMapData {
    /// Map identifier.
    pub map_id: String,
    /// Map name.
    pub name: String,
    /// Grid resolution (meters per cell).
    pub resolution: f32,
    /// Grid width in cells.
    pub width: u32,
    /// Grid height in cells.
    pub height: u32,
    /// World X coordinate of cell (0,0).
    pub origin_x: f32,
    /// World Y coordinate of cell (0,0).
    pub origin_y: f32,
    /// Occupancy data (0=free, 100=occupied, 255=unknown).
    pub cells: Vec<u8>,
    /// Explored area in square meters.
    pub explored_area_m2: f32,
}

impl Default for CurrentMapData {
    fn default() -> Self {
        Self {
            map_id: String::new(),
            name: "Untitled".to_string(),
            resolution: 0.05,
            width: 0,
            height: 0,
            origin_x: 0.0,
            origin_y: 0.0,
            cells: Vec::new(),
            explored_area_m2: 0.0,
        }
    }
}

impl CurrentMapData {
    /// Create from an OccupancyGrid.
    pub fn from_occupancy_grid(grid: &OccupancyGrid, map_id: &str, name: &str) -> Self {
        let (width, height) = grid.dimensions();
        let (origin_x, origin_y) = grid.origin();
        let resolution = grid.resolution();

        // Convert grid cells to visualization format
        let mut cells = Vec::with_capacity(width * height);
        let mut explored_count = 0usize;

        for y in 0..height {
            for x in 0..width {
                let state = grid.get_state(x, y);
                let value = match state {
                    crate::algorithms::mapping::CellState::Free => {
                        explored_count += 1;
                        0u8
                    }
                    crate::algorithms::mapping::CellState::Occupied => {
                        explored_count += 1;
                        100u8
                    }
                    crate::algorithms::mapping::CellState::Unknown => 255u8,
                };
                cells.push(value);
            }
        }

        let cell_area = resolution * resolution;
        let explored_area_m2 = explored_count as f32 * cell_area;

        Self {
            map_id: map_id.to_string(),
            name: name.to_string(),
            resolution,
            width: width as u32,
            height: height as u32,
            origin_x,
            origin_y,
            cells,
            explored_area_m2,
        }
    }
}

/// Map list with all saved maps.
#[derive(Debug, Clone, Default)]
pub struct MapList {
    /// All available maps.
    pub maps: Vec<MapSummary>,
    /// Currently active map ID.
    pub active_map_id: Option<String>,
}

/// Autonomous exploration operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ExplorationMode {
    /// No autonomous exploration active.
    #[default]
    Disabled,
    /// Actively exploring and mapping.
    Exploring,
    /// Exploration complete (all reachable areas mapped).
    Complete,
}

/// Current exploration state for progress tracking.
#[derive(Debug, Clone, Default)]
pub struct ExplorationState {
    /// Current exploration mode.
    pub mode: ExplorationMode,
    /// Current target pose (if navigating).
    pub current_target: Option<Pose2D>,
    /// Human-readable status message.
    pub status: String,
    /// Number of frontiers remaining to explore.
    pub frontiers_remaining: usize,
    /// Whether a bumper obstacle was detected (triggers re-route).
    pub bumper_obstacle_detected: bool,
    /// Total area explored so far (square meters).
    pub explored_area_m2: f32,
    /// Estimated completion percentage (0-100).
    pub completion_percent: f32,
}

/// Mapping progress state for UI streaming (matches proto MappingProgress).
#[derive(Debug, Clone, Default)]
pub struct MappingProgressState {
    /// Current mapping state.
    pub state: MappingProgressStateEnum,
    /// Number of frontiers found.
    pub frontiers_found: u32,
    /// Number of frontiers visited successfully.
    pub frontiers_visited: u32,
    /// Number of frontiers that couldn't be reached.
    pub frontiers_failed: u32,
    /// Obstacles marked from bumper hits.
    pub bumper_obstacles_marked: u32,
    /// Elapsed time in milliseconds.
    pub elapsed_time_ms: u64,
    /// Number of 360° scans performed.
    pub scan_count: u32,
    /// Current frontier position (if navigating).
    pub current_frontier: Option<Pose2D>,
    /// Distance to current frontier.
    pub frontier_distance: Option<f32>,
    /// Total area explored (square meters).
    pub area_explored_m2: f32,
    /// Estimated completion percentage (0-100).
    pub exploration_percent: f32,
    /// Human-readable status message.
    pub status_message: String,
    /// Failure reason (if failed).
    pub failure_reason: String,
}

/// Mapping progress state enum (matches proto MappingState).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum MappingProgressStateEnum {
    #[default]
    Idle,
    Initializing,
    Analyzing,
    Escaping,
    Scanning,
    FindingFrontier,
    Navigating,
    Complete,
    #[allow(dead_code)] // Proto compatibility
    Paused,
    Failed,
}

/// Thread-safe shared state between all threads.
///
/// Access pattern:
/// - SLAM Thread: Writes robot_status, current_map, sensor_status frequently
/// - Command Thread: Reads for responses, writes map_list on save/delete
/// - Publisher Thread: Reads all fields for streaming
/// - Exploration Thread: Reads map/pose, writes exploration_state
/// - Navigation Thread: Reads pose/map, writes navigation_state
#[derive(Debug)]
pub struct SharedState {
    /// Current robot status (pose, state, battery, progress).
    pub robot_status: RobotStatus,

    /// Current occupancy grid map data for visualization.
    pub current_map: Option<CurrentMapData>,

    /// Latest sensor readings.
    pub sensor_status: SensorStatus,

    /// List of saved maps.
    pub map_list: MapList,

    /// Flag indicating map list has changed (for publisher).
    pub map_list_dirty: bool,

    /// Current mapping session name (when mapping).
    pub current_map_name: String,

    /// Current mapping session ID (when mapping).
    pub current_map_id: String,

    /// Current autonomous exploration state.
    pub exploration_state: ExplorationState,

    /// Current navigation state (target stack, path, nav state).
    pub navigation_state: NavigationState,

    /// Mapping feature progress (for UI streaming).
    pub mapping_progress: MappingProgressState,
}

impl Default for SharedState {
    fn default() -> Self {
        Self::new()
    }
}

impl SharedState {
    /// Create a new shared state with default values.
    pub fn new() -> Self {
        Self {
            robot_status: RobotStatus::default(),
            current_map: None,
            sensor_status: SensorStatus::default(),
            map_list: MapList::default(),
            map_list_dirty: false,
            current_map_name: "Untitled".to_string(),
            current_map_id: String::new(),
            exploration_state: ExplorationState::default(),
            navigation_state: NavigationState::default(),
            mapping_progress: MappingProgressState::default(),
        }
    }

    /// Update robot status from SLAM engine.
    pub fn update_from_slam(
        &mut self,
        pose: Pose2D,
        state: RobotState,
        keyframe_count: u32,
        loop_closures: u32,
        match_score: f32,
        timestamp_us: u64,
    ) {
        self.robot_status.pose = pose;
        self.robot_status.state = state;
        self.robot_status.keyframe_count = keyframe_count;
        self.robot_status.loop_closures = loop_closures;
        self.robot_status.localization_confidence = match_score;
        self.robot_status.timestamp_us = timestamp_us;
    }

    /// Update map data from SLAM engine.
    pub fn update_map(&mut self, grid: &OccupancyGrid) {
        self.current_map = Some(CurrentMapData::from_occupancy_grid(
            grid,
            &self.current_map_id,
            &self.current_map_name,
        ));

        // Update map area in robot status
        if let Some(ref map) = self.current_map {
            self.robot_status.map_area_m2 = map.explored_area_m2;
        }
    }

    /// Start a new mapping session.
    pub fn start_mapping(&mut self, map_id: String, map_name: String) {
        self.robot_status.state = RobotState::Mapping;
        self.robot_status.distance_traveled_m = 0.0;
        self.robot_status.keyframe_count = 0;
        self.robot_status.loop_closures = 0;
        self.robot_status.map_area_m2 = 0.0;
        self.current_map_id = map_id;
        self.current_map_name = map_name;
        self.current_map = None;
    }

    /// Stop mapping session.
    pub fn stop_mapping(&mut self) {
        self.robot_status.state = RobotState::Idle;
        self.robot_status.active_map_id = None;
    }

    /// Enable localization on a map.
    pub fn enable_localization(&mut self, map_id: String) {
        self.robot_status.state = RobotState::Localizing;
        self.robot_status.active_map_id = Some(map_id.clone());
        self.current_map_id = map_id;
    }
}

/// Handle type for shared state (Arc<RwLock<SharedState>>).
pub type SharedStateHandle = Arc<RwLock<SharedState>>;

/// Create a new shared state wrapped in Arc<RwLock>.
pub fn create_shared_state() -> SharedStateHandle {
    Arc::new(RwLock::new(SharedState::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shared_state_creation() {
        let state = SharedState::new();
        assert_eq!(state.robot_status.state, RobotState::Idle);
        assert!(state.current_map.is_none());
    }

    #[test]
    fn test_start_mapping() {
        let mut state = SharedState::new();
        state.start_mapping("map_001".to_string(), "Living Room".to_string());

        assert_eq!(state.robot_status.state, RobotState::Mapping);
        assert_eq!(state.current_map_id, "map_001");
        assert_eq!(state.current_map_name, "Living Room");
        assert_eq!(state.robot_status.distance_traveled_m, 0.0);
    }

    #[test]
    fn test_shared_state_handle() {
        let handle = create_shared_state();

        // Test write access
        {
            let mut state = handle.write().unwrap();
            state.start_mapping("test".to_string(), "Test Map".to_string());
        }

        // Test read access
        {
            let state = handle.read().unwrap();
            assert_eq!(state.robot_status.state, RobotState::Mapping);
        }
    }
}
