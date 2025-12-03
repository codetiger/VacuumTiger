//! Submap management for local SLAM.
//!
//! Submaps are local occupancy grids that are built incrementally as the
//! robot moves through the environment. Once a submap is "finished" (has
//! enough scans), it becomes immutable and is handed off to the global
//! SLAM backend for loop closure and optimization.
//!
//! # Benefits of Submaps
//!
//! - Bounded computational cost per scan (only update local submap)
//! - Natural parallelization (global SLAM runs in background)
//! - Memory efficiency (can compress finished submaps)
//! - Better loop closure (match against complete submaps)

use crate::algorithms::mapping::{
    MapIntegrator, MapIntegratorConfig, OccupancyGrid, OccupancyGridConfig,
};
use crate::core::types::{PointCloud2D, Pose2D};

/// State of a submap in its lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubmapState {
    /// Actively being built with new scans.
    Active,
    /// Being finished (last scans being integrated).
    Finishing,
    /// Complete and immutable.
    Finished,
}

/// A local map built from a sequence of scans.
#[derive(Debug)]
pub struct Submap {
    /// Unique identifier.
    pub id: u64,

    /// Occupancy grid for this submap.
    grid: OccupancyGrid,

    /// Global pose of the submap origin.
    ///
    /// All scans in the submap are stored relative to this pose.
    pub origin: Pose2D,

    /// Number of scans integrated into this submap.
    pub num_scans: u32,

    /// IDs of keyframes in this submap.
    pub keyframe_ids: Vec<u64>,

    /// Current state.
    state: SubmapState,

    /// Timestamp of first scan.
    pub start_time_us: u64,

    /// Timestamp of last scan.
    pub end_time_us: u64,

    /// Map integrator for adding scans.
    integrator: MapIntegrator,
}

impl Submap {
    /// Create a new submap at the given origin.
    pub fn new(
        id: u64,
        origin: Pose2D,
        grid_config: OccupancyGridConfig,
        integrator_config: MapIntegratorConfig,
        timestamp_us: u64,
    ) -> Self {
        Self {
            id,
            grid: OccupancyGrid::new(grid_config),
            origin,
            num_scans: 0,
            keyframe_ids: Vec::new(),
            state: SubmapState::Active,
            start_time_us: timestamp_us,
            end_time_us: timestamp_us,
            integrator: MapIntegrator::new(integrator_config),
        }
    }

    /// Get the occupancy grid.
    pub fn grid(&self) -> &OccupancyGrid {
        &self.grid
    }

    /// Get mutable grid (only valid while active).
    pub fn grid_mut(&mut self) -> Option<&mut OccupancyGrid> {
        if self.state == SubmapState::Active {
            Some(&mut self.grid)
        } else {
            None
        }
    }

    /// Get the current state.
    pub fn state(&self) -> SubmapState {
        self.state
    }

    /// Check if submap is active.
    pub fn is_active(&self) -> bool {
        self.state == SubmapState::Active
    }

    /// Check if submap is finished.
    pub fn is_finished(&self) -> bool {
        self.state == SubmapState::Finished
    }

    /// Integrate a scan into this submap.
    ///
    /// # Arguments
    ///
    /// * `scan` - Point cloud in robot frame
    /// * `global_pose` - Robot pose in global frame
    /// * `timestamp_us` - Scan timestamp
    ///
    /// # Returns
    ///
    /// `true` if scan was integrated, `false` if submap is not active.
    pub fn integrate_scan(
        &mut self,
        scan: &PointCloud2D,
        global_pose: &Pose2D,
        timestamp_us: u64,
    ) -> bool {
        if self.state != SubmapState::Active {
            return false;
        }

        // Convert global pose to submap-local pose
        let local_pose = self.global_to_local(global_pose);

        // Transform scan to submap-local coordinates
        let global_cloud = scan.transform(&local_pose);

        // Integrate cloud (which expects global coordinates, but we provide submap-local)
        // The robot position is at local_pose, and points are already transformed
        self.integrator
            .integrate_cloud(&mut self.grid, &global_cloud, &local_pose);

        self.num_scans += 1;
        self.end_time_us = timestamp_us;

        true
    }

    /// Add a keyframe ID to this submap.
    pub fn add_keyframe(&mut self, keyframe_id: u64) {
        self.keyframe_ids.push(keyframe_id);
    }

    /// Convert a global pose to submap-local coordinates.
    pub fn global_to_local(&self, global: &Pose2D) -> Pose2D {
        // local = origin^-1 * global
        let inv_origin = self.origin.inverse();
        inv_origin.compose(global)
    }

    /// Convert a submap-local pose to global coordinates.
    pub fn local_to_global(&self, local: &Pose2D) -> Pose2D {
        // global = origin * local
        self.origin.compose(local)
    }

    /// Start finishing this submap.
    pub fn start_finishing(&mut self) {
        if self.state == SubmapState::Active {
            self.state = SubmapState::Finishing;
        }
    }

    /// Mark this submap as finished.
    pub fn finish(&mut self) {
        self.state = SubmapState::Finished;
    }

    /// Update the origin pose (e.g., after graph optimization).
    pub fn update_origin(&mut self, new_origin: Pose2D) {
        self.origin = new_origin;
    }

    /// Get memory usage in bytes.
    pub fn memory_usage(&self) -> usize {
        self.grid.memory_usage() + self.keyframe_ids.len() * std::mem::size_of::<u64>()
    }
}

/// Configuration for submap management.
#[derive(Debug, Clone)]
pub struct SubmapManagerConfig {
    /// Grid configuration for submaps.
    pub grid_config: OccupancyGridConfig,

    /// Integrator configuration.
    pub integrator_config: MapIntegratorConfig,

    /// Number of scans per submap before finishing.
    pub scans_per_submap: u32,

    /// Overlap: keep integrating into old submap while building new one.
    ///
    /// Number of scans to continue adding to the old submap after
    /// starting a new one.
    pub overlap_scans: u32,

    /// Maximum number of active submaps (for overlap).
    pub max_active_submaps: usize,
}

impl Default for SubmapManagerConfig {
    fn default() -> Self {
        Self {
            grid_config: OccupancyGridConfig {
                resolution: 0.05,    // 5cm cells
                initial_width: 10.0, // 10m (smaller than global map)
                initial_height: 10.0,
                ..OccupancyGridConfig::default()
            },
            integrator_config: MapIntegratorConfig::default(),
            scans_per_submap: 100,
            overlap_scans: 20,
            max_active_submaps: 2,
        }
    }
}

/// Manages the lifecycle of submaps.
pub struct SubmapManager {
    config: SubmapManagerConfig,

    /// All submaps (active and finished).
    submaps: Vec<Submap>,

    /// ID of the currently active submap.
    active_submap_id: Option<u64>,

    /// Next submap ID.
    next_id: u64,

    /// Scan counter for current submap.
    scans_in_current: u32,

    /// IDs of submaps currently in overlap mode.
    overlap_submaps: Vec<(u64, u32)>, // (id, remaining_scans)
}

impl SubmapManager {
    /// Create a new submap manager.
    pub fn new(config: SubmapManagerConfig) -> Self {
        Self {
            config,
            submaps: Vec::new(),
            active_submap_id: None,
            next_id: 0,
            scans_in_current: 0,
            overlap_submaps: Vec::new(),
        }
    }

    /// Get the active submap.
    pub fn active_submap(&self) -> Option<&Submap> {
        self.active_submap_id.and_then(|id| self.get(id))
    }

    /// Get mutable active submap.
    pub fn active_submap_mut(&mut self) -> Option<&mut Submap> {
        let id = self.active_submap_id?;
        self.submaps.iter_mut().find(|s| s.id == id)
    }

    /// Get a submap by ID.
    pub fn get(&self, id: u64) -> Option<&Submap> {
        self.submaps.iter().find(|s| s.id == id)
    }

    /// Get mutable submap by ID.
    pub fn get_mut(&mut self, id: u64) -> Option<&mut Submap> {
        self.submaps.iter_mut().find(|s| s.id == id)
    }

    /// Get all submaps.
    pub fn submaps(&self) -> &[Submap] {
        &self.submaps
    }

    /// Get all finished submaps.
    pub fn finished_submaps(&self) -> impl Iterator<Item = &Submap> {
        self.submaps.iter().filter(|s| s.is_finished())
    }

    /// Process a new scan.
    ///
    /// This handles submap lifecycle:
    /// 1. Creates a new submap if needed
    /// 2. Integrates scan into active submap(s)
    /// 3. Manages overlap period
    /// 4. Finishes old submaps when ready
    ///
    /// # Arguments
    ///
    /// * `scan` - Point cloud in robot frame
    /// * `global_pose` - Robot pose in global frame
    /// * `timestamp_us` - Scan timestamp
    ///
    /// # Returns
    ///
    /// ID of the primary active submap.
    pub fn process_scan(
        &mut self,
        scan: &PointCloud2D,
        global_pose: &Pose2D,
        timestamp_us: u64,
    ) -> u64 {
        // Create first submap if needed
        if self.active_submap_id.is_none() {
            let id = self.create_submap(*global_pose, timestamp_us);
            self.active_submap_id = Some(id);
        }

        let active_id = self.active_submap_id.unwrap();

        // Integrate into active submap
        if let Some(submap) = self.active_submap_mut() {
            submap.integrate_scan(scan, global_pose, timestamp_us);
        }
        self.scans_in_current += 1;

        // Integrate into overlap submaps
        let mut finished_overlaps = Vec::new();
        for (overlap_id, remaining) in &mut self.overlap_submaps {
            if let Some(submap) = self.submaps.iter_mut().find(|s| s.id == *overlap_id) {
                submap.integrate_scan(scan, global_pose, timestamp_us);
            }
            *remaining -= 1;
            if *remaining == 0 {
                finished_overlaps.push(*overlap_id);
            }
        }

        // Finish overlapping submaps
        for id in finished_overlaps {
            if let Some(submap) = self.submaps.iter_mut().find(|s| s.id == id) {
                submap.finish();
            }
            self.overlap_submaps.retain(|(i, _)| *i != id);
        }

        // Check if we need to create a new submap
        if self.scans_in_current >= self.config.scans_per_submap {
            self.transition_to_new_submap(*global_pose, timestamp_us);
        }

        active_id
    }

    /// Create a new submap.
    fn create_submap(&mut self, origin: Pose2D, timestamp_us: u64) -> u64 {
        let id = self.next_id;
        self.next_id += 1;

        let submap = Submap::new(
            id,
            origin,
            self.config.grid_config.clone(),
            self.config.integrator_config.clone(),
            timestamp_us,
        );

        self.submaps.push(submap);
        id
    }

    /// Transition from current submap to a new one.
    fn transition_to_new_submap(&mut self, origin: Pose2D, timestamp_us: u64) {
        // Move current submap to overlap mode
        if let Some(old_id) = self.active_submap_id {
            if let Some(submap) = self.get_mut(old_id) {
                submap.start_finishing();
            }

            if self.config.overlap_scans > 0 {
                self.overlap_submaps
                    .push((old_id, self.config.overlap_scans));
            } else {
                // No overlap, finish immediately
                if let Some(submap) = self.get_mut(old_id) {
                    submap.finish();
                }
            }
        }

        // Create new active submap
        let new_id = self.create_submap(origin, timestamp_us);
        self.active_submap_id = Some(new_id);
        self.scans_in_current = 0;

        log::debug!(
            "Transitioned to new submap {} (finished submaps: {})",
            new_id,
            self.submaps.iter().filter(|s| s.is_finished()).count()
        );
    }

    /// Add a keyframe to the active submap.
    pub fn add_keyframe_to_active(&mut self, keyframe_id: u64) {
        if let Some(submap) = self.active_submap_mut() {
            submap.add_keyframe(keyframe_id);
        }
    }

    /// Get the global map by combining all submaps.
    ///
    /// This creates a new occupancy grid that merges all submap data.
    pub fn global_map(&self, config: &OccupancyGridConfig) -> OccupancyGrid {
        let mut global = OccupancyGrid::new(config.clone());

        for submap in &self.submaps {
            self.merge_submap_into(&mut global, submap);
        }

        global
    }

    /// Merge a submap into a global map.
    fn merge_submap_into(&self, global: &mut OccupancyGrid, submap: &Submap) {
        let (width, height) = submap.grid.dimensions();
        let (sin_t, cos_t) = submap.origin.theta.sin_cos();

        for cy in 0..height {
            for cx in 0..width {
                let log_odds = submap.grid.get_log_odds(cx, cy);
                if log_odds.abs() < 0.1 {
                    continue; // Skip unknown cells
                }

                // Convert to world coordinates
                let (local_x, local_y) = submap.grid.cell_to_world(cx, cy);

                // Apply submap origin transform
                let world_x = submap.origin.x + local_x * cos_t - local_y * sin_t;
                let world_y = submap.origin.y + local_x * sin_t + local_y * cos_t;

                // Ensure global map can contain this point
                global.ensure_contains(world_x, world_y);

                // Update global map
                if let Some((gx, gy)) = global.world_to_cell(world_x, world_y) {
                    // Simple merge: use max absolute log-odds
                    let current = global.get_log_odds(gx, gy);
                    if log_odds.abs() > current.abs() {
                        // Direct cell manipulation (bypass normal update)
                        let occupied = log_odds > 0.0;
                        for _ in 0..((log_odds.abs() / 0.5) as i32).max(1) {
                            global.update_cell(gx, gy, occupied);
                        }
                    }
                }
            }
        }
    }

    /// Update submap origins after graph optimization.
    pub fn update_origins(&mut self, new_origins: &[(u64, Pose2D)]) {
        for (id, new_origin) in new_origins {
            if let Some(submap) = self.get_mut(*id) {
                submap.update_origin(*new_origin);
            }
        }
    }

    /// Get total memory usage.
    pub fn memory_usage(&self) -> usize {
        self.submaps.iter().map(|s| s.memory_usage()).sum()
    }

    /// Get number of submaps.
    pub fn len(&self) -> usize {
        self.submaps.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.submaps.is_empty()
    }

    /// Clear all submaps.
    pub fn clear(&mut self) {
        self.submaps.clear();
        self.active_submap_id = None;
        self.next_id = 0;
        self.scans_in_current = 0;
        self.overlap_submaps.clear();
    }

    /// Get configuration.
    pub fn config(&self) -> &SubmapManagerConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;

    fn create_test_scan() -> PointCloud2D {
        let mut scan = PointCloud2D::new();
        for i in 0..20 {
            let angle = i as f32 * 0.1;
            scan.push(Point2D::new(2.0 * angle.cos(), 2.0 * angle.sin()));
        }
        scan
    }

    #[test]
    fn test_submap_creation() {
        let grid_config = OccupancyGridConfig::default();
        let integrator_config = MapIntegratorConfig::default();
        let origin = Pose2D::new(1.0, 2.0, 0.5);

        let submap = Submap::new(0, origin, grid_config, integrator_config, 0);

        assert_eq!(submap.id, 0);
        assert_eq!(submap.origin.x, 1.0);
        assert_eq!(submap.num_scans, 0);
        assert!(submap.is_active());
    }

    #[test]
    fn test_submap_integration() {
        let grid_config = OccupancyGridConfig::default();
        let integrator_config = MapIntegratorConfig::default();

        let mut submap = Submap::new(0, Pose2D::identity(), grid_config, integrator_config, 0);

        let scan = create_test_scan();
        let pose = Pose2D::new(0.5, 0.0, 0.0);

        assert!(submap.integrate_scan(&scan, &pose, 1000));
        assert_eq!(submap.num_scans, 1);
    }

    #[test]
    fn test_submap_coordinate_conversion() {
        let grid_config = OccupancyGridConfig::default();
        let integrator_config = MapIntegratorConfig::default();
        let origin = Pose2D::new(1.0, 2.0, std::f32::consts::FRAC_PI_2);

        let submap = Submap::new(0, origin, grid_config, integrator_config, 0);

        // Global pose (1, 3) with origin at (1, 2) rotated 90°
        let global = Pose2D::new(1.0, 3.0, std::f32::consts::FRAC_PI_2);
        let local = submap.global_to_local(&global);

        // In local frame, this should be approximately (1, 0, 0)
        assert!((local.x - 1.0).abs() < 0.01, "local.x = {}", local.x);
        assert!(local.y.abs() < 0.01, "local.y = {}", local.y);
    }

    #[test]
    fn test_submap_lifecycle() {
        let grid_config = OccupancyGridConfig::default();
        let integrator_config = MapIntegratorConfig::default();

        let mut submap = Submap::new(0, Pose2D::identity(), grid_config, integrator_config, 0);

        assert_eq!(submap.state(), SubmapState::Active);

        submap.start_finishing();
        assert_eq!(submap.state(), SubmapState::Finishing);

        submap.finish();
        assert_eq!(submap.state(), SubmapState::Finished);
        assert!(submap.is_finished());
    }

    #[test]
    fn test_submap_manager_auto_creation() {
        let config = SubmapManagerConfig::default();
        let mut manager = SubmapManager::new(config);

        let scan = create_test_scan();
        let pose = Pose2D::new(0.5, 0.0, 0.0);

        // First scan should create a submap
        let id = manager.process_scan(&scan, &pose, 0);
        assert_eq!(id, 0);
        assert_eq!(manager.len(), 1);
        assert!(manager.active_submap().is_some());
    }

    #[test]
    fn test_submap_manager_transition() {
        let config = SubmapManagerConfig {
            scans_per_submap: 5,
            overlap_scans: 0, // No overlap for simpler testing
            ..Default::default()
        };
        let mut manager = SubmapManager::new(config);

        let scan = create_test_scan();

        // Add scans until transition
        for i in 0..6 {
            let pose = Pose2D::new(i as f32 * 0.1, 0.0, 0.0);
            manager.process_scan(&scan, &pose, i * 1000);
        }

        // Should have transitioned to new submap
        assert_eq!(manager.len(), 2);
        assert_eq!(manager.finished_submaps().count(), 1);
    }

    #[test]
    fn test_submap_manager_overlap() {
        let config = SubmapManagerConfig {
            scans_per_submap: 3,
            overlap_scans: 2,
            ..Default::default()
        };
        let mut manager = SubmapManager::new(config);

        let scan = create_test_scan();

        // Add scans: 3 to fill first submap, then it goes to overlap
        for i in 0..5 {
            let pose = Pose2D::new(i as f32 * 0.1, 0.0, 0.0);
            manager.process_scan(&scan, &pose, i * 1000);
        }

        // First submap should still be finishing (2 overlap scans remaining)
        assert_eq!(manager.finished_submaps().count(), 0);

        // Two more scans to finish overlap
        for i in 5..7 {
            let pose = Pose2D::new(i as f32 * 0.1, 0.0, 0.0);
            manager.process_scan(&scan, &pose, i * 1000);
        }

        // Now first submap should be finished
        assert_eq!(manager.finished_submaps().count(), 1);
    }
}
