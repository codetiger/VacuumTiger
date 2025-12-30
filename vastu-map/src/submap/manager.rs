//! Submap manager - orchestrates submap lifecycle and global grid composition.

use super::config::SubmapConfig;
use super::types::{Submap, SubmapId};
use crate::core::{LidarScan, Pose2D, WorldPoint};
use crate::grid::{GridConfig, GridStorage, MapConfig, SensorConfig};

/// Correction to apply to a submap's origin after pose graph optimization.
#[derive(Clone, Debug)]
pub struct SubmapCorrection {
    /// ID of the submap to correct.
    pub submap_id: SubmapId,
    /// New origin pose in world frame.
    pub new_origin: Pose2D,
}

/// Cached global grid with version tracking.
pub struct CachedGlobalGrid {
    /// The composed global grid.
    pub grid: GridStorage,
    /// Submap versions used to create this grid (id -> origin).
    /// Used to detect if regeneration is needed.
    _submap_versions: Vec<(SubmapId, Pose2D)>,
}

/// Result of inserting a scan into the submap manager.
#[derive(Clone, Debug)]
pub struct InsertResult {
    /// The submap(s) the scan was inserted into.
    pub submaps: Vec<SubmapId>,
    /// The local pose in the active submap.
    pub local_pose: Pose2D,
    /// Whether a new submap was created.
    pub new_submap_created: bool,
    /// Whether any submap was finalized.
    pub submap_finalized: bool,
}

/// Manages submaps and global grid composition.
///
/// The SubmapManager is the central orchestrator for the submap architecture.
/// It handles:
/// - Submap lifecycle (Active → Filling → Finalized)
/// - Scan insertion with automatic transitions
/// - Global grid composition with lazy regeneration
/// - Applying pose corrections from optimizer
pub struct SubmapManager {
    /// All submaps (completed + active).
    submaps: Vec<Submap>,

    /// Next submap ID to assign.
    next_id: u32,

    /// Index of the currently active submap.
    active_idx: Option<usize>,

    /// Index of the filling submap (if any).
    filling_idx: Option<usize>,

    /// Configuration.
    config: SubmapConfig,

    /// Sensor configuration for grid updates.
    sensor_config: SensorConfig,

    /// Grid configuration.
    grid_config: GridConfig,

    /// Cached global grid (regenerated on demand).
    global_grid: Option<CachedGlobalGrid>,

    /// Whether the global grid needs regeneration.
    global_dirty: bool,
}

impl SubmapManager {
    /// Create a new submap manager.
    pub fn new(config: SubmapConfig, map_config: &MapConfig) -> Self {
        Self {
            submaps: Vec::new(),
            next_id: 0,
            active_idx: None,
            filling_idx: None,
            config,
            sensor_config: map_config.sensor.clone(),
            grid_config: map_config.grid.clone(),
            global_grid: None,
            global_dirty: true,
        }
    }

    /// Get the configuration.
    pub fn config(&self) -> &SubmapConfig {
        &self.config
    }

    /// Get all submaps.
    pub fn submaps(&self) -> &[Submap] {
        &self.submaps
    }

    /// Get the active submap (if any).
    pub fn active_submap(&self) -> Option<&Submap> {
        self.active_idx.map(|i| &self.submaps[i])
    }

    /// Get the active submap mutably.
    pub fn active_submap_mut(&mut self) -> Option<&mut Submap> {
        self.active_idx.map(|i| &mut self.submaps[i])
    }

    /// Get a submap by ID.
    pub fn get_submap(&self, id: SubmapId) -> Option<&Submap> {
        self.submaps.iter().find(|s| s.id == id)
    }

    /// Get a submap by ID mutably.
    pub fn get_submap_mut(&mut self, id: SubmapId) -> Option<&mut Submap> {
        self.submaps.iter_mut().find(|s| s.id == id)
    }

    /// Get all finalized submaps (eligible for loop closure).
    pub fn finalized_submaps(&self) -> impl Iterator<Item = &Submap> {
        self.submaps.iter().filter(|s| s.is_finalized())
    }

    /// Number of submaps.
    pub fn submap_count(&self) -> usize {
        self.submaps.len()
    }

    /// Total memory usage in bytes.
    pub fn memory_bytes(&self) -> usize {
        let submaps_bytes: usize = self.submaps.iter().map(|s| s.memory_bytes()).sum();
        let global_bytes = self
            .global_grid
            .as_ref()
            .map(|g| g.grid.width() * g.grid.height() * 6)
            .unwrap_or(0);
        submaps_bytes + global_bytes
    }

    /// Insert a scan at the given world pose.
    ///
    /// This method handles:
    /// - Creating the first submap if none exists
    /// - Inserting into active (and filling) submaps
    /// - Triggering state transitions based on scan count and distance
    /// - Creating new submaps when thresholds are exceeded
    pub fn insert_scan(
        &mut self,
        scan: &LidarScan,
        world_pose: Pose2D,
        timestamp_us: u64,
    ) -> InsertResult {
        let mut result = InsertResult {
            submaps: Vec::new(),
            local_pose: Pose2D::default(),
            new_submap_created: false,
            submap_finalized: false,
        };

        // Create first submap if needed
        if self.active_idx.is_none() {
            self.create_new_submap(world_pose);
            result.new_submap_created = true;
        }

        // Check if we need to create a new submap before inserting
        let should_transition = self.should_create_new_submap(world_pose);

        if should_transition && self.filling_idx.is_none() {
            // Start transition: active -> filling, create new active
            self.start_submap_transition(world_pose);
            result.new_submap_created = true;
        }

        // Insert into active submap
        let active_submap = &mut self.submaps[self.active_idx.unwrap()];
        active_submap.insert_scan(
            scan,
            world_pose,
            timestamp_us,
            &self.sensor_config,
            &self.grid_config,
        );
        result.submaps.push(active_submap.id);
        result.local_pose = active_submap.origin.inverse().compose(&world_pose);

        // Insert into filling submap if exists (overlap period)
        if let Some(filling_idx) = self.filling_idx {
            let filling_submap = &mut self.submaps[filling_idx];
            filling_submap.insert_scan(
                scan,
                world_pose,
                timestamp_us,
                &self.sensor_config,
                &self.grid_config,
            );
            result.submaps.push(filling_submap.id);

            // Check if filling submap should be finalized
            if filling_submap.overlap_count >= self.config.overlap_scans {
                filling_submap.finalize();
                self.filling_idx = None;
                result.submap_finalized = true;
            }
        }

        // Mark global grid as dirty
        self.global_dirty = true;

        result
    }

    /// Check if we should create a new submap.
    fn should_create_new_submap(&self, world_pose: Pose2D) -> bool {
        let Some(active_idx) = self.active_idx else {
            return true;
        };

        let active = &self.submaps[active_idx];

        // Check scan count threshold
        if active.scan_count >= self.config.scans_per_submap {
            return true;
        }

        // Check distance threshold
        if active.distance_traveled >= self.config.max_distance_per_submap {
            return true;
        }

        // Check if robot is near edge of local grid
        if !active.contains_world_point(world_pose.position()) {
            return true;
        }

        false
    }

    /// Start the transition from active to filling, create new active.
    fn start_submap_transition(&mut self, new_origin: Pose2D) {
        if let Some(active_idx) = self.active_idx {
            // Current active becomes filling
            self.submaps[active_idx].start_filling();
            self.filling_idx = Some(active_idx);
        }

        // Create new active submap
        self.create_new_submap(new_origin);
    }

    /// Create a new submap at the given origin.
    fn create_new_submap(&mut self, origin: Pose2D) {
        let id = SubmapId::new(self.next_id);
        self.next_id += 1;

        let submap = Submap::new(
            id,
            origin,
            self.config.local_grid_size,
            self.config.resolution,
        );

        self.submaps.push(submap);
        self.active_idx = Some(self.submaps.len() - 1);
    }

    /// Get or regenerate the global grid.
    ///
    /// Uses lazy regeneration - only regenerates when dirty.
    pub fn global_grid(&mut self) -> &GridStorage {
        if self.global_dirty || self.global_grid.is_none() {
            self.regenerate_global_grid();
        }
        &self.global_grid.as_ref().unwrap().grid
    }

    /// Force regeneration of the global grid.
    pub fn regenerate_global_grid(&mut self) {
        let bounds = self.compute_global_bounds();

        // Create new grid covering all submaps
        let width = ((bounds.1.x - bounds.0.x) / self.config.resolution).ceil() as usize + 1;
        let height = ((bounds.1.y - bounds.0.y) / self.config.resolution).ceil() as usize + 1;

        // Clamp to reasonable size
        let width = width.min(self.grid_config.max_width).max(100);
        let height = height.min(self.grid_config.max_height).max(100);

        let mut grid = GridStorage::new(width, height, self.config.resolution, bounds.0);

        // Insert all scans from all submaps
        for submap in &self.submaps {
            for stored in &submap.scans {
                let world_pose = submap.origin.compose(&stored.local_pose);
                crate::grid::lidar_update::update_from_lidar(
                    &mut grid,
                    &stored.scan,
                    world_pose,
                    &self.sensor_config,
                    &self.grid_config,
                );
            }
        }

        // Cache version info
        let versions: Vec<_> = self.submaps.iter().map(|s| (s.id, s.origin)).collect();

        self.global_grid = Some(CachedGlobalGrid {
            grid,
            _submap_versions: versions,
        });
        self.global_dirty = false;
    }

    /// Compute the bounding box of all submaps in world coordinates.
    fn compute_global_bounds(&self) -> (WorldPoint, WorldPoint) {
        if self.submaps.is_empty() {
            // Default bounds centered at origin
            let half = (self.config.local_grid_size as f32 * self.config.resolution) / 2.0;
            return (WorldPoint::new(-half, -half), WorldPoint::new(half, half));
        }

        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        for submap in &self.submaps {
            let (sub_min, sub_max) = submap.world_bounds();
            min_x = min_x.min(sub_min.x);
            min_y = min_y.min(sub_min.y);
            max_x = max_x.max(sub_max.x);
            max_y = max_y.max(sub_max.y);
        }

        // Add small margin
        let margin = self.config.resolution * 10.0;
        (
            WorldPoint::new(min_x - margin, min_y - margin),
            WorldPoint::new(max_x + margin, max_y + margin),
        )
    }

    /// Apply pose corrections from optimizer.
    ///
    /// After pose graph optimization, call this method to update submap origins.
    /// The global grid will be regenerated on next access.
    pub fn apply_corrections(&mut self, corrections: &[SubmapCorrection]) {
        for correction in corrections {
            if let Some(submap) = self.get_submap_mut(correction.submap_id) {
                submap.origin = correction.new_origin;
            }
        }

        // Mark global grid as needing regeneration
        self.global_dirty = true;
    }

    /// Apply corrections and immediately regenerate affected submap grids.
    ///
    /// Use this when you need immediate consistency (e.g., for scan matching).
    pub fn apply_corrections_and_regenerate(&mut self, corrections: &[SubmapCorrection]) {
        // First, apply all corrections
        for correction in corrections {
            if let Some(submap) = self
                .submaps
                .iter_mut()
                .find(|s| s.id == correction.submap_id)
            {
                submap.origin = correction.new_origin;
            }
        }

        // Then regenerate all affected submaps
        // Clone configs to avoid borrow conflict
        let sensor_config = self.sensor_config.clone();
        let grid_config = self.grid_config.clone();

        for correction in corrections {
            if let Some(submap) = self
                .submaps
                .iter_mut()
                .find(|s| s.id == correction.submap_id)
            {
                submap.regenerate_grid(&sensor_config, &grid_config);
            }
        }

        self.global_dirty = true;
    }

    /// Get submaps that overlap with a world point.
    ///
    /// Useful for scan matching against multiple submaps.
    pub fn submaps_containing(&self, point: WorldPoint) -> Vec<&Submap> {
        self.submaps
            .iter()
            .filter(|s| s.contains_world_point(point))
            .collect()
    }

    /// Get submaps within a distance of a world point.
    pub fn submaps_near(&self, point: WorldPoint, max_distance: f32) -> Vec<&Submap> {
        self.submaps
            .iter()
            .filter(|s| s.origin.position().distance(&point) <= max_distance)
            .collect()
    }

    /// Get finalized submaps suitable for loop closure with the active submap.
    ///
    /// Returns submaps that are:
    /// - Finalized
    /// - Have sufficient ID gap (not recent)
    /// - Within maximum distance
    pub fn loop_closure_candidates(&self, current_pose: Pose2D) -> Vec<&Submap> {
        let Some(active) = self.active_submap() else {
            return Vec::new();
        };

        let min_id = active
            .id
            .value()
            .saturating_sub(self.config.min_loop_closure_gap as u32);

        self.submaps
            .iter()
            .filter(|s| s.is_finalized())
            .filter(|s| s.id.value() < min_id)
            .filter(|s| {
                s.origin.position().distance(&current_pose.position())
                    <= self.config.max_loop_closure_distance
            })
            .collect()
    }

    /// Check if global grid is dirty (needs regeneration).
    pub fn is_global_dirty(&self) -> bool {
        self.global_dirty
    }

    /// Clear all submaps and reset state.
    pub fn clear(&mut self) {
        self.submaps.clear();
        self.next_id = 0;
        self.active_idx = None;
        self.filling_idx = None;
        self.global_grid = None;
        self.global_dirty = true;
    }
}

#[cfg(test)]
mod tests {
    use super::super::types::SubmapState;
    use super::*;

    fn make_test_scan() -> LidarScan {
        // Simple scan with a few points
        let angles: Vec<f32> = (0..36).map(|i| (i as f32) * 0.1745).collect(); // 0 to ~360°
        let ranges: Vec<f32> = angles.iter().map(|_| 2.0).collect(); // 2m range
        LidarScan::new(ranges, angles, 0.15, 8.0)
    }

    #[test]
    fn test_manager_creation() {
        let config = SubmapConfig::default();
        let map_config = MapConfig::default();
        let manager = SubmapManager::new(config, &map_config);

        assert_eq!(manager.submap_count(), 0);
        assert!(manager.active_submap().is_none());
    }

    #[test]
    fn test_first_scan_creates_submap() {
        let config = SubmapConfig::default();
        let map_config = MapConfig::default();
        let mut manager = SubmapManager::new(config, &map_config);

        let scan = make_test_scan();
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let result = manager.insert_scan(&scan, pose, 0);

        assert!(result.new_submap_created);
        assert_eq!(result.submaps.len(), 1);
        assert_eq!(manager.submap_count(), 1);
        assert!(manager.active_submap().is_some());
    }

    #[test]
    fn test_submap_transition() {
        let mut config = SubmapConfig::default();
        config.scans_per_submap = 3; // Small for testing
        config.overlap_scans = 2;

        let map_config = MapConfig::default();
        let mut manager = SubmapManager::new(config, &map_config);

        let scan = make_test_scan();

        // Insert scans until transition
        for i in 0..5 {
            let pose = Pose2D::new(i as f32 * 0.1, 0.0, 0.0);
            let result = manager.insert_scan(&scan, pose, i as u64);

            if i == 3 {
                // After threshold, new submap should be created
                assert!(result.new_submap_created);
            }
        }

        // Should have 2 submaps now
        assert_eq!(manager.submap_count(), 2);

        // First submap should be filling or finalized
        let first = manager.get_submap(SubmapId::new(0)).unwrap();
        assert!(first.state != SubmapState::Active);
    }

    #[test]
    fn test_submap_finalization() {
        let mut config = SubmapConfig::default();
        config.scans_per_submap = 2;
        config.overlap_scans = 2;

        let map_config = MapConfig::default();
        let mut manager = SubmapManager::new(config, &map_config);

        let scan = make_test_scan();

        // Insert enough scans to finalize first submap
        for i in 0..6 {
            let pose = Pose2D::new(i as f32 * 0.1, 0.0, 0.0);
            manager.insert_scan(&scan, pose, i as u64);
        }

        // First submap should be finalized
        let first = manager.get_submap(SubmapId::new(0)).unwrap();
        assert!(first.is_finalized());
    }

    #[test]
    fn test_global_grid_regeneration() {
        let config = SubmapConfig::default();
        let map_config = MapConfig::default();
        let mut manager = SubmapManager::new(config, &map_config);

        let scan = make_test_scan();
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        manager.insert_scan(&scan, pose, 0);

        // Should be dirty
        assert!(manager.is_global_dirty());

        // Access global grid (triggers regeneration)
        let _grid = manager.global_grid();

        // Should no longer be dirty
        assert!(!manager.is_global_dirty());
    }

    #[test]
    fn test_apply_corrections() {
        let config = SubmapConfig::default();
        let map_config = MapConfig::default();
        let mut manager = SubmapManager::new(config, &map_config);

        let scan = make_test_scan();
        manager.insert_scan(&scan, Pose2D::new(0.0, 0.0, 0.0), 0);

        // Get grid to clear dirty flag
        let _ = manager.global_grid();
        assert!(!manager.is_global_dirty());

        // Apply correction
        let correction = SubmapCorrection {
            submap_id: SubmapId::new(0),
            new_origin: Pose2D::new(1.0, 1.0, 0.5),
        };
        manager.apply_corrections(&[correction]);

        // Should be dirty again
        assert!(manager.is_global_dirty());

        // Origin should be updated
        let submap = manager.get_submap(SubmapId::new(0)).unwrap();
        assert!((submap.origin.x - 1.0).abs() < 1e-6);
        assert!((submap.origin.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_loop_closure_candidates() {
        let mut config = SubmapConfig::default();
        config.scans_per_submap = 2;
        config.overlap_scans = 1;
        config.min_loop_closure_gap = 2;
        config.max_loop_closure_distance = 10.0;

        let map_config = MapConfig::default();
        let mut manager = SubmapManager::new(config, &map_config);

        let scan = make_test_scan();

        // Create several submaps
        for i in 0..10 {
            let pose = Pose2D::new(i as f32 * 0.2, 0.0, 0.0);
            manager.insert_scan(&scan, pose, i as u64);
        }

        // Get candidates from current position
        let current = Pose2D::new(0.0, 0.0, 0.0); // Back at origin
        let candidates = manager.loop_closure_candidates(current);

        // Should find finalized submaps with sufficient gap
        assert!(!candidates.is_empty());
        for c in &candidates {
            assert!(c.is_finalized());
        }
    }
}
