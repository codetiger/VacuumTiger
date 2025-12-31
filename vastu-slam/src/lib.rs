//! # VastuSLAM
//!
//! 2D SLAM library with occupancy grid mapping, scan matching, and loop closure.
//!
//! ## Overview
//!
//! VastuSLAM provides a multi-layer occupancy grid with semantic cell types:
//!
//! - **Unknown** - Not yet observed by any sensor
//! - **Floor** - Traversable surface
//! - **Wall** - Lidar-detected obstacles
//! - **Cliff** - Floor drop-offs (stairs, ledges)
//! - **Bump** - Invisible obstacles (glass, mirrors, thin legs)
//!
//! ## Features
//!
//! - **Occupancy Grid Mapping**: Multi-sensor integration with priority-based updates
//! - **Scan Matching**: Correlative and branch-and-bound scan-to-map alignment
//! - **Loop Closure**: LiDAR-IRIS descriptors for place recognition
//! - **Submap Architecture**: Cartographer-style reversible map updates
//!
//! ## Quick Start
//!
//! ```rust,ignore
//! use vastu_slam::{OccupancyGridMap, MapConfig, Pose2D, LidarScan};
//!
//! // Create map with default configuration
//! let config = MapConfig::default();
//! let mut map = OccupancyGridMap::new(config);
//!
//! // Process lidar scan
//! let pose = Pose2D::new(0.0, 0.0, 0.0);
//! let scan = LidarScan::new(ranges, angles, 0.15, 8.0);
//! let result = map.observe_lidar(&scan, pose);
//!
//! println!("Updated {} cells", result.cells_updated);
//! ```
//!
//! ## Coordinate System
//!
//! Uses ROS REP-103 convention:
//! - X: Forward (positive ahead of robot)
//! - Y: Left (positive to robot's left)
//! - Theta: Rotation in radians, CCW positive from +X axis

#![warn(missing_docs)]
#![feature(portable_simd)]

// Core types
pub mod core;

// Grid storage and configuration
pub mod grid;

// Unified configuration
pub mod config;

// Scan matching and loop closure
pub mod matching;

// Submap architecture for reversible map updates
pub mod submap;

// SLAM operation modes (localization, etc.)
pub mod modes;

// Persistence (save/load)
pub mod io;

// Evaluation framework (Cartographer-style)
pub mod evaluation;

// Re-export commonly used types
pub use core::{
    BumperSensors, Cell, CellType, CliffSensors, GridCoord, LidarScan, MotionModel, Odometry,
    Pose2D, SensorObservation, WorldPoint,
};

pub use grid::{CellCounts, ConfigError, GridConfig, GridStorage, MapConfig, SensorConfig};

pub use config::{ConfigLoadError, VastuConfig};

pub use matching::{
    CorrelativeMatcher, CorrelativeMatcherConfig, LoopClosureDetector, LoopValidator, RobustKernel,
    ScanMatchResult, ScanMatcher,
};

pub use modes::{LocalizationResult, Localizer, LocalizerConfig};

pub use submap::{
    InsertResult, MultiSubmapMatchConfig, MultiSubmapMatchResult, MultiSubmapMatcher, StoredScan,
    Submap, SubmapConfig, SubmapCorrection, SubmapGraphConfig, SubmapId, SubmapLoopClosure,
    SubmapManager, SubmapPoseGraph, SubmapState,
};

/// Result of processing sensor observations
#[derive(Clone, Debug, Default)]
pub struct ObserveResult {
    /// Total cells updated
    pub cells_updated: usize,
    /// Cells newly marked as Floor
    pub cells_floor: usize,
    /// Cells marked as Wall (lidar hit)
    pub cells_wall: usize,
    /// Cells marked as Cliff
    pub cells_cliff: usize,
    /// Cells marked as Bump (invisible obstacle)
    pub cells_bump: usize,
}

impl ObserveResult {
    /// Merge another result into this one
    pub fn merge(&mut self, other: &ObserveResult) {
        self.cells_updated += other.cells_updated;
        self.cells_floor += other.cells_floor;
        self.cells_wall += other.cells_wall;
        self.cells_cliff += other.cells_cliff;
        self.cells_bump += other.cells_bump;
    }
}

/// Map coverage statistics (cell type breakdown)
#[derive(Clone, Debug, Default)]
pub struct MapCoverageStats {
    /// Total known cells (Floor + Wall + Cliff + Bump)
    pub known_cells: usize,
    /// Floor cells (traversable)
    pub floor_cells: usize,
    /// Wall cells
    pub wall_cells: usize,
    /// Cliff cells
    pub cliff_cells: usize,
    /// Bump cells (invisible obstacles)
    pub bump_cells: usize,
    /// Unknown cells still in grid
    pub unknown_cells: usize,
    /// Estimated explored area (m²)
    pub explored_area_m2: f32,
}

impl From<CellCounts> for MapCoverageStats {
    fn from(counts: CellCounts) -> Self {
        Self {
            known_cells: counts.known(),
            floor_cells: counts.floor,
            wall_cells: counts.wall,
            cliff_cells: counts.cliff,
            bump_cells: counts.bump,
            unknown_cells: counts.unknown,
            explored_area_m2: 0.0,
        }
    }
}

/// The main occupancy grid map
///
/// This is the primary type for interacting with the map.
pub struct OccupancyGridMap {
    /// Grid storage
    storage: GridStorage,
    /// Configuration
    config: MapConfig,
}

impl OccupancyGridMap {
    /// Create a new occupancy grid map
    pub fn new(config: MapConfig) -> Self {
        let origin = config.grid.effective_origin();
        let storage = GridStorage::new(
            config.grid.initial_width,
            config.grid.initial_height,
            config.grid.resolution,
            origin,
        );

        Self { storage, config }
    }

    /// Get the grid storage
    pub fn storage(&self) -> &GridStorage {
        &self.storage
    }

    /// Get mutable grid storage
    pub fn storage_mut(&mut self) -> &mut GridStorage {
        &mut self.storage
    }

    /// Get the configuration
    pub fn config(&self) -> &MapConfig {
        &self.config
    }

    /// Grid resolution in meters per cell
    pub fn resolution(&self) -> f32 {
        self.storage.resolution()
    }

    /// Grid dimensions (width, height) in cells
    pub fn dimensions(&self) -> (usize, usize) {
        (self.storage.width(), self.storage.height())
    }

    /// World bounds: (min_point, max_point)
    pub fn bounds(&self) -> (WorldPoint, WorldPoint) {
        self.storage.bounds()
    }

    /// Convert world coordinates to grid coordinates
    pub fn world_to_grid(&self, point: WorldPoint) -> GridCoord {
        self.storage.world_to_grid(point)
    }

    /// Convert grid coordinates to world coordinates (cell center)
    pub fn grid_to_world(&self, coord: GridCoord) -> WorldPoint {
        self.storage.grid_to_world(coord)
    }

    /// Get cell at world coordinates
    pub fn get_cell(&self, point: WorldPoint) -> Cell {
        self.storage.get_world(point).unwrap_or_default()
    }

    /// Get cell at grid coordinates
    pub fn get_cell_grid(&self, coord: GridCoord) -> Cell {
        self.storage.get(coord).unwrap_or_default()
    }

    /// Get cell type at world coordinates
    pub fn get_type(&self, point: WorldPoint) -> CellType {
        self.storage.get_type_world(point)
    }

    /// Is this point traversable?
    pub fn is_traversable(&self, point: WorldPoint) -> bool {
        self.get_type(point).is_traversable()
    }

    /// Is this point known (not Unknown)?
    pub fn is_known(&self, point: WorldPoint) -> bool {
        self.get_type(point).is_known()
    }

    /// Get map coverage statistics
    pub fn coverage_stats(&self) -> MapCoverageStats {
        let counts = self.storage.count_by_type();
        let resolution = self.resolution();
        let cell_area = resolution * resolution;

        let mut stats = MapCoverageStats::from(counts);
        stats.explored_area_m2 = counts.known() as f32 * cell_area;
        stats
    }

    /// Clear all cells to Unknown
    pub fn clear(&mut self) {
        self.storage.clear();
    }

    // =========================================================================
    // SENSOR OBSERVATION METHODS
    // =========================================================================

    /// Process a complete sensor observation.
    ///
    /// This is the main entry point for updating the map with sensor data.
    /// It processes lidar, cliff, and bumper sensors in the correct order
    /// (priority: bumper > cliff > lidar).
    pub fn observe(&mut self, observation: &SensorObservation) -> ObserveResult {
        let mut result = ObserveResult::default();

        // Process lidar first (lowest priority)
        if let Some(ref scan) = observation.lidar {
            let lidar_result = self.observe_lidar(scan, observation.pose);
            result.merge(&lidar_result);
        }

        // Process cliff sensors (higher priority than lidar)
        if observation.cliffs.any_triggered() {
            let cliff_result = self.observe_cliff(&observation.cliffs, observation.pose);
            result.merge(&cliff_result);
        }

        // Process bumper sensors (highest priority)
        if observation.bumpers.any_triggered() {
            let bumper_result = self.observe_bumper(&observation.bumpers, observation.pose);
            result.merge(&bumper_result);
        }

        result
    }

    /// Update the map with a lidar scan.
    ///
    /// For each valid ray in the scan:
    /// - Marks all cells along the ray as Floor (free space)
    /// - Marks the endpoint cell as Wall (if range < max_range)
    ///
    /// # Arguments
    /// * `scan` - The lidar scan data
    /// * `pose` - Robot pose when the scan was taken
    ///
    /// # Returns
    /// Statistics about the cells that were updated
    pub fn observe_lidar(&mut self, scan: &LidarScan, pose: Pose2D) -> ObserveResult {
        grid::lidar_update::update_from_lidar(
            &mut self.storage,
            scan,
            pose,
            &self.config.sensor,
            &self.config.grid,
        )
    }

    /// Update the map with a lidar scan, using scan-to-map matching.
    ///
    /// This method first aligns the scan to the current map using
    /// correlative matching, then integrates the scan at the refined pose.
    /// This helps correct for encoder drift.
    ///
    /// # Arguments
    /// * `scan` - The lidar scan data
    /// * `encoder_pose` - Initial pose estimate from encoders
    /// * `matcher_config` - Configuration for the scan matcher
    ///
    /// # Returns
    /// Tuple of (observe_result, refined_pose, match_result)
    pub fn observe_lidar_with_matching(
        &mut self,
        scan: &LidarScan,
        encoder_pose: Pose2D,
        matcher_config: &matching::CorrelativeMatcherConfig,
    ) -> (ObserveResult, Pose2D, matching::ScanMatchResult) {
        let matcher = matching::CorrelativeMatcher::new(matcher_config.clone());
        let match_result = matcher.match_scan(scan, encoder_pose, &self.storage);

        // Use matched pose if converged, otherwise fall back to encoder pose
        let final_pose = if match_result.converged && match_result.score >= matcher_config.min_score
        {
            match_result.pose
        } else {
            encoder_pose
        };

        // Integrate scan at the refined pose
        let observe_result = self.observe_lidar(scan, final_pose);

        (observe_result, final_pose, match_result)
    }

    /// Update the map with cliff sensor readings.
    ///
    /// For each triggered cliff sensor, marks the cell at the sensor
    /// position as Cliff. Cliff has higher priority than Floor/Wall.
    ///
    /// # Arguments
    /// * `cliffs` - Cliff sensor states
    /// * `pose` - Robot pose when sensors were read
    ///
    /// # Returns
    /// Statistics about the cells that were updated
    pub fn observe_cliff(&mut self, cliffs: &CliffSensors, pose: Pose2D) -> ObserveResult {
        grid::cliff_update::update_from_cliff(&mut self.storage, cliffs, pose)
    }

    /// Update the map with bumper collision readings.
    ///
    /// For each triggered bumper, estimates the collision position and
    /// marks those cells as Bump. Bump has the highest priority.
    ///
    /// # Arguments
    /// * `bumpers` - Bumper sensor states
    /// * `pose` - Robot pose when collision occurred
    ///
    /// # Returns
    /// Statistics about the cells that were updated
    pub fn observe_bumper(&mut self, bumpers: &BumperSensors, pose: Pose2D) -> ObserveResult {
        grid::bumper_update::update_from_bumper(
            &mut self.storage,
            bumpers,
            pose,
            self.config.sensor.robot_radius,
        )
    }

    /// Mark cells as swept by the robot at its current position.
    ///
    /// This is used for coverage tracking during cleaning.
    pub fn mark_swept(&mut self, pose: Pose2D, radius: f32) {
        let center = pose.position();
        let resolution = self.storage.resolution();
        let cells_radius = (radius / resolution).ceil() as i32;
        let center_coord = self.storage.world_to_grid(center);

        for dy in -cells_radius..=cells_radius {
            for dx in -cells_radius..=cells_radius {
                let coord = center_coord + GridCoord::new(dx, dy);

                if !self.storage.is_valid_coord(coord) {
                    continue;
                }

                // Check if within circular radius
                let cell_center = self.storage.grid_to_world(coord);
                if cell_center.distance(&center) <= radius
                    && let Some(cell) = self.storage.get_mut(coord)
                {
                    *cell.swept = 1;
                }
            }
        }
    }

    /// Reset swept status for all cells.
    pub fn reset_swept(&mut self) {
        for (_, cell) in self.storage.iter_mut() {
            *cell.swept = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_map_creation() {
        let config = MapConfig::default();
        let map = OccupancyGridMap::new(config);

        assert_eq!(map.resolution(), 0.025);
        assert_eq!(map.dimensions(), (800, 800));
    }

    #[test]
    fn test_coordinate_conversion() {
        let config = MapConfig::default();
        let map = OccupancyGridMap::new(config);

        // Origin should map to a grid coordinate near center
        let origin_coord = map.world_to_grid(WorldPoint::ZERO);
        assert!(origin_coord.x > 0);
        assert!(origin_coord.y > 0);

        // Round-trip should be close to original
        let back_to_world = map.grid_to_world(origin_coord);
        assert!((back_to_world.x - 0.0).abs() < map.resolution());
        assert!((back_to_world.y - 0.0).abs() < map.resolution());
    }

    #[test]
    fn test_cell_access() {
        let config = MapConfig::default();
        let map = OccupancyGridMap::new(config);

        // All cells should initially be Unknown
        let cell = map.get_cell(WorldPoint::ZERO);
        assert_eq!(cell.cell_type, CellType::Unknown);
        assert!(!map.is_traversable(WorldPoint::ZERO));
        assert!(!map.is_known(WorldPoint::ZERO));
    }

    #[test]
    fn test_coverage_stats() {
        let config = MapConfig::default();
        let map = OccupancyGridMap::new(config);

        let stats = map.coverage_stats();
        assert_eq!(stats.known_cells, 0);
        assert_eq!(stats.unknown_cells, 800 * 800);
        assert_eq!(stats.explored_area_m2, 0.0);
    }
}
