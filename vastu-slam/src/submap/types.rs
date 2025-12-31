//! Core submap data types.

use crate::core::{LidarScan, Pose2D, WorldPoint};
use crate::grid::GridStorage;
use crate::matching::PrecomputedGrids;
use std::cell::RefCell;

/// Unique identifier for a submap.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SubmapId(pub u32);

impl SubmapId {
    /// Create a new submap ID.
    #[inline]
    pub fn new(id: u32) -> Self {
        Self(id)
    }

    /// Get the numeric value.
    #[inline]
    pub fn value(&self) -> u32 {
        self.0
    }
}

impl std::fmt::Display for SubmapId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Submap({})", self.0)
    }
}

/// State of a submap in its lifecycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubmapState {
    /// Actively receiving scans.
    /// Only one submap can be Active at a time.
    Active,

    /// Full but still receiving overlap scans.
    /// A new Active submap has been created in parallel.
    Filling,

    /// Finalized - no more scan updates allowed.
    /// Eligible for loop closure matching.
    /// Origin can be adjusted by pose graph optimizer.
    Finalized,
}

impl SubmapState {
    /// Can this submap receive new scans?
    #[inline]
    pub fn accepts_scans(&self) -> bool {
        matches!(self, SubmapState::Active | SubmapState::Filling)
    }

    /// Is this submap eligible for loop closure matching?
    #[inline]
    pub fn is_matchable(&self) -> bool {
        matches!(self, SubmapState::Finalized)
    }
}

/// A stored lidar scan with its local pose.
///
/// Scans are stored in polar format (original `LidarScan`) for memory efficiency.
/// The pose is relative to the submap origin, not world frame.
#[derive(Clone, Debug)]
pub struct StoredScan {
    /// Pose relative to submap origin (in submap frame).
    pub local_pose: Pose2D,

    /// Timestamp in microseconds since epoch.
    pub timestamp_us: u64,

    /// Original lidar scan data (polar coordinates).
    /// Stored as-is for perfect regeneration fidelity.
    pub scan: LidarScan,
}

impl StoredScan {
    /// Create a new stored scan.
    pub fn new(local_pose: Pose2D, timestamp_us: u64, scan: LidarScan) -> Self {
        Self {
            local_pose,
            timestamp_us,
            scan,
        }
    }

    /// Get the world pose given the submap origin.
    #[inline]
    pub fn world_pose(&self, submap_origin: Pose2D) -> Pose2D {
        submap_origin.compose(&self.local_pose)
    }

    /// Approximate memory usage in bytes.
    pub fn memory_bytes(&self) -> usize {
        // Pose2D (12 bytes) + u64 (8 bytes) + LidarScan overhead + data
        20 + std::mem::size_of::<LidarScan>()
            + self.scan.ranges.len() * 4
            + self.scan.angles.len() * 4
    }
}

/// A locally-consistent map fragment with adjustable origin.
///
/// Submaps store both a local occupancy grid (for scan matching) and
/// raw scans (for regeneration after pose graph optimization).
pub struct Submap {
    /// Unique identifier.
    pub id: SubmapId,

    /// Origin pose in world frame.
    /// This is what gets adjusted during pose graph optimization.
    pub origin: Pose2D,

    /// Local occupancy grid (origin at local 0,0).
    /// Uses the same resolution as global grid.
    pub grid: GridStorage,

    /// Raw scans with poses relative to submap origin.
    /// Used for regeneration after optimization.
    pub scans: Vec<StoredScan>,

    /// Current lifecycle state.
    pub state: SubmapState,

    /// Number of scans inserted (including overlap).
    pub scan_count: usize,

    /// Number of overlap scans (received during Filling state).
    pub overlap_count: usize,

    /// Distance traveled within this submap (for transition trigger).
    pub distance_traveled: f32,

    /// Last pose in local frame (for distance tracking).
    last_local_pose: Option<Pose2D>,

    /// Cached precomputed multi-resolution grids for branch-and-bound matching.
    /// Lazily initialized when first needed, invalidated on grid regeneration.
    precomputed_cache: RefCell<Option<PrecomputedGrids>>,
}

impl Submap {
    /// Create a new active submap at the given world origin.
    pub fn new(id: SubmapId, origin: Pose2D, grid_size: usize, resolution: f32) -> Self {
        // Local grid is centered at (0, 0) in local frame
        // Origin is at center of grid
        let half_size = (grid_size as f32 * resolution) / 2.0;
        let local_origin = WorldPoint::new(-half_size, -half_size);

        let grid = GridStorage::new(grid_size, grid_size, resolution, local_origin);

        Self {
            id,
            origin,
            grid,
            scans: Vec::with_capacity(60), // Typical 50 + 10 overlap
            state: SubmapState::Active,
            scan_count: 0,
            overlap_count: 0,
            distance_traveled: 0.0,
            last_local_pose: None,
            precomputed_cache: RefCell::new(None),
        }
    }

    /// Check if the submap can accept new scans.
    #[inline]
    pub fn accepts_scans(&self) -> bool {
        self.state.accepts_scans()
    }

    /// Check if the submap is finalized and matchable.
    #[inline]
    pub fn is_finalized(&self) -> bool {
        self.state == SubmapState::Finalized
    }

    /// Insert a scan into this submap.
    ///
    /// # Arguments
    /// * `scan` - The lidar scan to insert
    /// * `world_pose` - Robot pose in world frame when scan was taken
    /// * `timestamp_us` - Timestamp in microseconds
    /// * `sensor_config` - Sensor configuration for grid update
    /// * `grid_config` - Grid configuration
    ///
    /// # Returns
    /// `true` if scan was accepted, `false` if submap doesn't accept scans
    pub fn insert_scan(
        &mut self,
        scan: &LidarScan,
        world_pose: Pose2D,
        timestamp_us: u64,
        sensor_config: &crate::grid::SensorConfig,
        grid_config: &crate::grid::GridConfig,
    ) -> bool {
        if !self.accepts_scans() {
            return false;
        }

        // Convert world pose to local pose
        let local_pose = self.origin.inverse().compose(&world_pose);

        // Update distance traveled
        if let Some(last) = self.last_local_pose {
            self.distance_traveled += local_pose.distance(&last);
        }
        self.last_local_pose = Some(local_pose);

        // Store raw scan for regeneration
        self.scans
            .push(StoredScan::new(local_pose, timestamp_us, scan.clone()));

        // Update local grid
        crate::grid::lidar_update::update_from_lidar(
            &mut self.grid,
            scan,
            local_pose,
            sensor_config,
            grid_config,
        );

        self.scan_count += 1;
        if self.state == SubmapState::Filling {
            self.overlap_count += 1;
        }

        true
    }

    /// Transition to Filling state.
    /// Called when scan count threshold is reached.
    pub fn start_filling(&mut self) {
        if self.state == SubmapState::Active {
            self.state = SubmapState::Filling;
        }
    }

    /// Finalize the submap - no more updates allowed.
    /// Also builds the precomputed grids cache for efficient matching.
    pub fn finalize(&mut self) {
        self.state = SubmapState::Finalized;
        // Pre-build the cache since finalized submaps are used for matching
        self.build_precomputed_grids();
    }

    /// Get the precomputed multi-resolution grids for branch-and-bound matching.
    /// Lazily computes and caches the grids on first access.
    /// Only useful for finalized submaps.
    pub fn precomputed_grids(&self) -> std::cell::Ref<'_, PrecomputedGrids> {
        // Ensure cache is populated
        {
            let mut cache = self.precomputed_cache.borrow_mut();
            if cache.is_none() {
                *cache = Some(PrecomputedGrids::from_storage(&self.grid));
            }
        }
        std::cell::Ref::map(self.precomputed_cache.borrow(), |opt| {
            opt.as_ref().expect("Cache should be populated")
        })
    }

    /// Check if precomputed grids are already cached.
    #[inline]
    pub fn has_precomputed_grids(&self) -> bool {
        self.precomputed_cache.borrow().is_some()
    }

    /// Force rebuild of precomputed grids cache.
    /// Call this after finalization or after origin adjustment.
    pub fn build_precomputed_grids(&mut self) {
        *self.precomputed_cache.borrow_mut() = Some(PrecomputedGrids::from_storage(&self.grid));
    }

    /// Invalidate the precomputed grids cache.
    /// Called when the grid is regenerated.
    fn invalidate_precomputed_cache(&mut self) {
        *self.precomputed_cache.borrow_mut() = None;
    }

    /// Regenerate the local grid from stored scans.
    /// Called after the submap origin has been adjusted.
    /// Invalidates and rebuilds the precomputed grids cache if finalized.
    pub fn regenerate_grid(
        &mut self,
        sensor_config: &crate::grid::SensorConfig,
        grid_config: &crate::grid::GridConfig,
    ) {
        // Invalidate the cache since grid is changing
        self.invalidate_precomputed_cache();

        // Clear the grid
        self.grid.clear();

        // Re-insert all scans at their local poses
        for stored in &self.scans {
            crate::grid::lidar_update::update_from_lidar(
                &mut self.grid,
                &stored.scan,
                stored.local_pose,
                sensor_config,
                grid_config,
            );
        }

        // Rebuild cache if this is a finalized submap
        if self.is_finalized() {
            self.build_precomputed_grids();
        }
    }

    /// Get the bounds of this submap in world coordinates.
    pub fn world_bounds(&self) -> (WorldPoint, WorldPoint) {
        let (local_min, local_max) = self.grid.bounds();

        // Transform corners to world frame
        let corners = [
            self.origin.transform_point(local_min),
            self.origin
                .transform_point(WorldPoint::new(local_max.x, local_min.y)),
            self.origin.transform_point(local_max),
            self.origin
                .transform_point(WorldPoint::new(local_min.x, local_max.y)),
        ];

        let min_x = corners.iter().map(|p| p.x).fold(f32::INFINITY, f32::min);
        let min_y = corners.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
        let max_x = corners
            .iter()
            .map(|p| p.x)
            .fold(f32::NEG_INFINITY, f32::max);
        let max_y = corners
            .iter()
            .map(|p| p.y)
            .fold(f32::NEG_INFINITY, f32::max);

        (WorldPoint::new(min_x, min_y), WorldPoint::new(max_x, max_y))
    }

    /// Check if a world point is within this submap's local grid bounds.
    pub fn contains_world_point(&self, world_point: WorldPoint) -> bool {
        let local_point = self.origin.inverse_transform_point(world_point);
        let coord = self.grid.world_to_grid(local_point);
        self.grid.is_valid_coord(coord)
    }

    /// Approximate memory usage in bytes.
    pub fn memory_bytes(&self) -> usize {
        let grid_bytes = self.grid.width() * self.grid.height() * 6; // ~6 bytes per cell
        let scans_bytes: usize = self.scans.iter().map(|s| s.memory_bytes()).sum();
        grid_bytes + scans_bytes + std::mem::size_of::<Self>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_submap_id() {
        let id = SubmapId::new(42);
        assert_eq!(id.value(), 42);
        assert_eq!(format!("{}", id), "Submap(42)");
    }

    #[test]
    fn test_submap_state_transitions() {
        let state = SubmapState::Active;
        assert!(state.accepts_scans());
        assert!(!state.is_matchable());

        let state = SubmapState::Filling;
        assert!(state.accepts_scans());
        assert!(!state.is_matchable());

        let state = SubmapState::Finalized;
        assert!(!state.accepts_scans());
        assert!(state.is_matchable());
    }

    #[test]
    fn test_submap_creation() {
        let origin = Pose2D::new(1.0, 2.0, 0.5);
        let submap = Submap::new(SubmapId::new(0), origin, 200, 0.025);

        assert_eq!(submap.id, SubmapId::new(0));
        assert_eq!(submap.origin, origin);
        assert_eq!(submap.state, SubmapState::Active);
        assert_eq!(submap.scan_count, 0);
        assert!(submap.accepts_scans());
    }

    #[test]
    fn test_stored_scan_world_pose() {
        let submap_origin = Pose2D::new(1.0, 0.0, std::f32::consts::FRAC_PI_2);
        let local_pose = Pose2D::new(1.0, 0.0, 0.0);

        let scan = LidarScan::new(vec![1.0], vec![0.0], 0.15, 8.0);
        let stored = StoredScan::new(local_pose, 0, scan);

        let world_pose = stored.world_pose(submap_origin);

        // At origin (1, 0) facing +Y, moving 1m forward in local frame
        // should put us at (1, 1) in world frame
        assert!((world_pose.x - 1.0).abs() < 1e-5);
        assert!((world_pose.y - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_submap_lifecycle() {
        let origin = Pose2D::new(0.0, 0.0, 0.0);
        let mut submap = Submap::new(SubmapId::new(0), origin, 200, 0.025);

        assert_eq!(submap.state, SubmapState::Active);

        submap.start_filling();
        assert_eq!(submap.state, SubmapState::Filling);
        assert!(submap.accepts_scans());

        submap.finalize();
        assert_eq!(submap.state, SubmapState::Finalized);
        assert!(!submap.accepts_scans());
        assert!(submap.is_finalized());
    }
}
