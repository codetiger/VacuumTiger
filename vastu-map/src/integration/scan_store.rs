//! Scan storage for exploration mode.
//!
//! Retains raw scan data during exploration for visualization,
//! debugging, and future point-cloud-based line fitting.

use crate::core::{Point2D, PointCloud2D, Pose2D};

/// A stored scan with associated pose information.
#[derive(Clone, Debug)]
pub struct StoredScan {
    /// Unique scan identifier (sequential).
    pub scan_id: u32,

    /// Raw odometry pose (input to observe).
    pub odometry: Pose2D,

    /// Estimated pose after ICP matching.
    /// This is the corrected pose used for map integration.
    pub estimated_pose: Pose2D,

    /// Scan points in robot frame.
    /// Transform with `estimated_pose` to get world-frame points.
    points: Vec<Point2D>,

    /// Number of features extracted from this scan.
    pub features_extracted: usize,

    /// ICP matching confidence (0.0 - 1.0).
    pub match_confidence: f32,
}

impl StoredScan {
    /// Create a new stored scan.
    pub fn new(
        scan_id: u32,
        odometry: Pose2D,
        estimated_pose: Pose2D,
        points: Vec<Point2D>,
        features_extracted: usize,
        match_confidence: f32,
    ) -> Self {
        Self {
            scan_id,
            odometry,
            estimated_pose,
            points,
            features_extracted,
            match_confidence,
        }
    }

    /// Get robot-frame points.
    #[inline]
    pub fn points(&self) -> &[Point2D] {
        &self.points
    }

    /// Get the number of points in this scan.
    #[inline]
    pub fn point_count(&self) -> usize {
        self.points.len()
    }

    /// Get world-frame points (transformed by estimated pose).
    pub fn world_points(&self) -> Vec<Point2D> {
        self.points
            .iter()
            .map(|p| self.estimated_pose.transform_point(*p))
            .collect()
    }

    /// Fill a buffer with world-frame points (zero-allocation if buffer is large enough).
    pub fn world_points_into(&self, buffer: &mut Vec<Point2D>) {
        buffer.clear();
        buffer.reserve(self.points.len());
        for p in &self.points {
            buffer.push(self.estimated_pose.transform_point(*p));
        }
    }

    /// Get the pose correction applied by ICP.
    /// Returns the difference between estimated and odometry-predicted pose.
    pub fn pose_correction(&self) -> Pose2D {
        // correction = estimated * odometry.inverse()
        // This gives the transform that was applied to correct odometry
        self.estimated_pose.compose(self.odometry.inverse())
    }

    /// Memory usage in bytes (approximate).
    pub fn memory_bytes(&self) -> usize {
        std::mem::size_of::<Self>() + self.points.len() * std::mem::size_of::<Point2D>()
    }
}

/// Configuration for scan storage.
#[derive(Clone, Debug)]
pub struct ScanStoreConfig {
    /// Maximum number of scans to store.
    /// Oldest scans are removed when limit is reached.
    /// None = unlimited. Default: None
    pub max_scans: Option<usize>,

    /// Whether to store point data.
    /// If false, only poses are stored (much less memory).
    /// Default: true
    pub store_points: bool,

    /// Minimum distance traveled to store a new scan (meters).
    /// Helps reduce redundant scans when robot is stationary.
    /// Default: 0.0 (store all scans)
    pub min_distance: f32,

    /// Minimum rotation to store a new scan (radians).
    /// Default: 0.0 (store all scans)
    pub min_rotation: f32,
}

impl Default for ScanStoreConfig {
    fn default() -> Self {
        Self {
            max_scans: None,
            store_points: true,
            min_distance: 0.0,
            min_rotation: 0.0,
        }
    }
}

impl ScanStoreConfig {
    /// Create a new configuration with defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set maximum number of scans to store.
    pub fn with_max_scans(mut self, max: usize) -> Self {
        self.max_scans = Some(max);
        self
    }

    /// Set unlimited scan storage.
    pub fn with_unlimited_scans(mut self) -> Self {
        self.max_scans = None;
        self
    }

    /// Enable or disable point storage.
    pub fn with_store_points(mut self, store: bool) -> Self {
        self.store_points = store;
        self
    }

    /// Set minimum distance between stored scans.
    pub fn with_min_distance(mut self, meters: f32) -> Self {
        self.min_distance = meters;
        self
    }

    /// Set minimum rotation between stored scans.
    pub fn with_min_rotation(mut self, radians: f32) -> Self {
        self.min_rotation = radians;
        self
    }
}

/// Storage for scans during exploration.
///
/// Retains raw scan data for visualization, debugging, and
/// point-cloud-based algorithms.
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::integration::{ScanStore, ScanStoreConfig};
///
/// let config = ScanStoreConfig::default().with_max_scans(1000);
/// let mut store = ScanStore::new(config);
///
/// // After each observation...
/// store.add_scan(odometry, estimated_pose, &scan_points, features, confidence);
///
/// // For visualization
/// for scan in store.iter() {
///     let world_points = scan.world_points();
///     draw_points(&world_points);
/// }
/// ```
#[derive(Clone, Debug)]
pub struct ScanStore {
    /// Stored scans (oldest first).
    scans: Vec<StoredScan>,

    /// Next scan ID to assign.
    next_scan_id: u32,

    /// Configuration.
    config: ScanStoreConfig,

    /// Last stored pose (for distance/rotation filtering).
    last_stored_pose: Option<Pose2D>,
}

impl ScanStore {
    /// Create a new empty scan store.
    pub fn new(config: ScanStoreConfig) -> Self {
        Self {
            scans: Vec::new(),
            next_scan_id: 0,
            config,
            last_stored_pose: None,
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(ScanStoreConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &ScanStoreConfig {
        &self.config
    }

    /// Get the number of stored scans.
    #[inline]
    pub fn len(&self) -> usize {
        self.scans.len()
    }

    /// Check if the store is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.scans.is_empty()
    }

    /// Get the next scan ID that will be assigned.
    #[inline]
    pub fn next_scan_id(&self) -> u32 {
        self.next_scan_id
    }

    /// Add a scan to the store.
    ///
    /// Returns the assigned scan_id, or None if the scan was filtered
    /// (due to min_distance/min_rotation constraints).
    pub fn add_scan(
        &mut self,
        odometry: Pose2D,
        estimated_pose: Pose2D,
        points: &PointCloud2D,
        features_extracted: usize,
        match_confidence: f32,
    ) -> Option<u32> {
        // Check distance/rotation filter
        if let Some(last_pose) = self.last_stored_pose {
            let dx = estimated_pose.x - last_pose.x;
            let dy = estimated_pose.y - last_pose.y;
            let dist = (dx * dx + dy * dy).sqrt();
            let dtheta = crate::core::math::angle_diff(estimated_pose.theta, last_pose.theta).abs();

            // Determine which constraints are active (> 0 means enabled)
            let distance_check_enabled = self.config.min_distance > 0.0;
            let rotation_check_enabled = self.config.min_rotation > 0.0;

            let passes_distance = dist >= self.config.min_distance;
            let passes_rotation = dtheta >= self.config.min_rotation;

            // Filter logic:
            // - If only distance filter: require distance
            // - If only rotation filter: require rotation
            // - If both filters: require either (OR semantics)
            // - If no filters: store all
            let should_store = match (distance_check_enabled, rotation_check_enabled) {
                (false, false) => true,
                (true, false) => passes_distance,
                (false, true) => passes_rotation,
                (true, true) => passes_distance || passes_rotation,
            };

            if !should_store {
                return None;
            }
        }

        // Enforce max_scans limit
        if let Some(max) = self.config.max_scans {
            while self.scans.len() >= max {
                self.scans.remove(0); // Remove oldest
            }
        }

        let scan_id = self.next_scan_id;
        self.next_scan_id += 1;

        let stored_points = if self.config.store_points {
            points.iter().collect()
        } else {
            Vec::new()
        };

        let scan = StoredScan::new(
            scan_id,
            odometry,
            estimated_pose,
            stored_points,
            features_extracted,
            match_confidence,
        );

        self.last_stored_pose = Some(estimated_pose);
        self.scans.push(scan);

        Some(scan_id)
    }

    /// Add a scan from raw points (not PointCloud2D).
    pub fn add_scan_from_points(
        &mut self,
        odometry: Pose2D,
        estimated_pose: Pose2D,
        points: &[Point2D],
        features_extracted: usize,
        match_confidence: f32,
    ) -> Option<u32> {
        // Check distance/rotation filter
        if let Some(last_pose) = self.last_stored_pose {
            let dx = estimated_pose.x - last_pose.x;
            let dy = estimated_pose.y - last_pose.y;
            let dist = (dx * dx + dy * dy).sqrt();
            let dtheta = crate::core::math::angle_diff(estimated_pose.theta, last_pose.theta).abs();

            // Determine which constraints are active (> 0 means enabled)
            let distance_check_enabled = self.config.min_distance > 0.0;
            let rotation_check_enabled = self.config.min_rotation > 0.0;

            let passes_distance = dist >= self.config.min_distance;
            let passes_rotation = dtheta >= self.config.min_rotation;

            let should_store = match (distance_check_enabled, rotation_check_enabled) {
                (false, false) => true,
                (true, false) => passes_distance,
                (false, true) => passes_rotation,
                (true, true) => passes_distance || passes_rotation,
            };

            if !should_store {
                return None;
            }
        }

        // Enforce max_scans limit
        if let Some(max) = self.config.max_scans {
            while self.scans.len() >= max {
                self.scans.remove(0);
            }
        }

        let scan_id = self.next_scan_id;
        self.next_scan_id += 1;

        let stored_points = if self.config.store_points {
            points.to_vec()
        } else {
            Vec::new()
        };

        let scan = StoredScan::new(
            scan_id,
            odometry,
            estimated_pose,
            stored_points,
            features_extracted,
            match_confidence,
        );

        self.last_stored_pose = Some(estimated_pose);
        self.scans.push(scan);

        Some(scan_id)
    }

    /// Get a scan by its ID.
    pub fn get(&self, scan_id: u32) -> Option<&StoredScan> {
        self.scans.iter().find(|s| s.scan_id == scan_id)
    }

    /// Get a scan by index (0 = oldest).
    pub fn get_by_index(&self, index: usize) -> Option<&StoredScan> {
        self.scans.get(index)
    }

    /// Get the most recent scan.
    pub fn latest(&self) -> Option<&StoredScan> {
        self.scans.last()
    }

    /// Get the oldest scan.
    pub fn oldest(&self) -> Option<&StoredScan> {
        self.scans.first()
    }

    /// Iterate over all stored scans (oldest first).
    pub fn iter(&self) -> impl Iterator<Item = &StoredScan> {
        self.scans.iter()
    }

    /// Iterate over scans in reverse order (newest first).
    pub fn iter_rev(&self) -> impl Iterator<Item = &StoredScan> {
        self.scans.iter().rev()
    }

    /// Iterate over scans within a pose range.
    ///
    /// Returns scans whose estimated_pose is within `radius` of `center`.
    pub fn iter_near(&self, center: Point2D, radius: f32) -> impl Iterator<Item = &StoredScan> {
        let radius_sq = radius * radius;
        self.scans.iter().filter(move |s| {
            let dx = s.estimated_pose.x - center.x;
            let dy = s.estimated_pose.y - center.y;
            dx * dx + dy * dy <= radius_sq
        })
    }

    /// Get all scans as a slice.
    pub fn scans(&self) -> &[StoredScan] {
        &self.scans
    }

    /// Get all estimated poses.
    pub fn poses(&self) -> Vec<Pose2D> {
        self.scans.iter().map(|s| s.estimated_pose).collect()
    }

    /// Get the trajectory as a list of points.
    pub fn trajectory(&self) -> Vec<Point2D> {
        self.scans
            .iter()
            .map(|s| Point2D::new(s.estimated_pose.x, s.estimated_pose.y))
            .collect()
    }

    /// Get all world-frame points from all scans.
    ///
    /// Warning: This can be memory-intensive for large stores.
    pub fn all_world_points(&self) -> Vec<Point2D> {
        let total_points: usize = self.scans.iter().map(|s| s.point_count()).sum();
        let mut result = Vec::with_capacity(total_points);

        for scan in &self.scans {
            for p in scan.points() {
                result.push(scan.estimated_pose.transform_point(*p));
            }
        }

        result
    }

    /// Get world-frame points with scan IDs.
    ///
    /// Returns (point, scan_id) pairs for all stored points.
    pub fn all_world_points_with_ids(&self) -> Vec<(Point2D, u32)> {
        let total_points: usize = self.scans.iter().map(|s| s.point_count()).sum();
        let mut result = Vec::with_capacity(total_points);

        for scan in &self.scans {
            for p in scan.points() {
                result.push((scan.estimated_pose.transform_point(*p), scan.scan_id));
            }
        }

        result
    }

    /// Update the estimated pose for a scan (after loop closure correction).
    ///
    /// Returns true if the scan was found and updated.
    pub fn update_pose(&mut self, scan_id: u32, new_pose: Pose2D) -> bool {
        if let Some(scan) = self.scans.iter_mut().find(|s| s.scan_id == scan_id) {
            scan.estimated_pose = new_pose;
            true
        } else {
            false
        }
    }

    /// Apply pose corrections to multiple scans.
    ///
    /// Takes a map of scan_id -> corrected_pose.
    /// Returns the number of scans updated.
    pub fn apply_pose_corrections(
        &mut self,
        corrections: &std::collections::HashMap<u32, Pose2D>,
    ) -> usize {
        let mut updated = 0;
        for scan in &mut self.scans {
            if let Some(new_pose) = corrections.get(&scan.scan_id) {
                scan.estimated_pose = *new_pose;
                updated += 1;
            }
        }
        updated
    }

    /// Clear all stored scans.
    pub fn clear(&mut self) {
        self.scans.clear();
        self.last_stored_pose = None;
        // Note: next_scan_id is NOT reset to allow ID continuity
    }

    /// Reset the store completely (including scan ID counter).
    pub fn reset(&mut self) {
        self.scans.clear();
        self.next_scan_id = 0;
        self.last_stored_pose = None;
    }

    /// Total number of points stored across all scans.
    pub fn total_points(&self) -> usize {
        self.scans.iter().map(|s| s.point_count()).sum()
    }

    /// Memory usage statistics.
    pub fn memory_stats(&self) -> ScanStoreStats {
        let total_points = self.total_points();
        let point_bytes = total_points * std::mem::size_of::<Point2D>();
        let scan_overhead = self.scans.len() * std::mem::size_of::<StoredScan>();
        let vec_overhead = self.scans.capacity() * std::mem::size_of::<StoredScan>();

        ScanStoreStats {
            scan_count: self.scans.len(),
            total_points,
            point_bytes,
            total_bytes: point_bytes + scan_overhead + vec_overhead,
            avg_points_per_scan: if self.scans.is_empty() {
                0.0
            } else {
                total_points as f32 / self.scans.len() as f32
            },
        }
    }
}

impl Default for ScanStore {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Memory usage statistics for ScanStore.
#[derive(Clone, Debug)]
pub struct ScanStoreStats {
    /// Number of stored scans.
    pub scan_count: usize,
    /// Total number of points across all scans.
    pub total_points: usize,
    /// Memory used by points (bytes).
    pub point_bytes: usize,
    /// Total estimated memory usage (bytes).
    pub total_bytes: usize,
    /// Average points per scan.
    pub avg_points_per_scan: f32,
}

impl ScanStoreStats {
    /// Get memory usage in kilobytes.
    pub fn total_kb(&self) -> f32 {
        self.total_bytes as f32 / 1024.0
    }

    /// Get memory usage in megabytes.
    pub fn total_mb(&self) -> f32 {
        self.total_bytes as f32 / (1024.0 * 1024.0)
    }
}

impl std::fmt::Display for ScanStoreStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ScanStore: {} scans, {} points, {:.2} KB",
            self.scan_count,
            self.total_points,
            self.total_kb()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_points(n: usize) -> Vec<Point2D> {
        (0..n).map(|i| Point2D::new(i as f32 * 0.1, 0.0)).collect()
    }

    fn make_point_cloud(n: usize) -> PointCloud2D {
        let xs: Vec<f32> = (0..n).map(|i| i as f32 * 0.1).collect();
        let ys: Vec<f32> = vec![0.0; n];
        PointCloud2D { xs, ys }
    }

    #[test]
    fn test_stored_scan_creation() {
        let scan = StoredScan::new(
            0,
            Pose2D::identity(),
            Pose2D::new(1.0, 0.0, 0.0),
            make_test_points(10),
            5,
            0.95,
        );

        assert_eq!(scan.scan_id, 0);
        assert_eq!(scan.point_count(), 10);
        assert_eq!(scan.features_extracted, 5);
        assert!((scan.match_confidence - 0.95).abs() < 0.001);
    }

    #[test]
    fn test_stored_scan_world_points() {
        let points = vec![Point2D::new(1.0, 0.0), Point2D::new(0.0, 1.0)];
        let pose = Pose2D::new(10.0, 20.0, std::f32::consts::FRAC_PI_2); // 90° rotation

        let scan = StoredScan::new(0, Pose2D::identity(), pose, points, 0, 1.0);

        let world = scan.world_points();
        assert_eq!(world.len(), 2);

        // (1, 0) rotated 90° + translated = (10 + 0, 20 + 1) = (10, 21)
        assert!((world[0].x - 10.0).abs() < 0.01);
        assert!((world[0].y - 21.0).abs() < 0.01);

        // (0, 1) rotated 90° + translated = (10 - 1, 20 + 0) = (9, 20)
        assert!((world[1].x - 9.0).abs() < 0.01);
        assert!((world[1].y - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_scan_store_basic() {
        let mut store = ScanStore::with_defaults();
        assert!(store.is_empty());
        assert_eq!(store.len(), 0);

        let cloud = make_point_cloud(50);
        let id = store.add_scan(
            Pose2D::identity(),
            Pose2D::new(1.0, 0.0, 0.0),
            &cloud,
            3,
            0.9,
        );

        assert_eq!(id, Some(0));
        assert_eq!(store.len(), 1);
        assert!(!store.is_empty());

        let scan = store.get(0).unwrap();
        assert_eq!(scan.point_count(), 50);
    }

    #[test]
    fn test_scan_store_max_scans() {
        let config = ScanStoreConfig::default().with_max_scans(3);
        let mut store = ScanStore::new(config);

        for i in 0..5 {
            let cloud = make_point_cloud(10);
            store.add_scan(
                Pose2D::identity(),
                Pose2D::new(i as f32, 0.0, 0.0),
                &cloud,
                0,
                1.0,
            );
        }

        assert_eq!(store.len(), 3);
        // Oldest scans (0, 1) should be removed
        assert!(store.get(0).is_none());
        assert!(store.get(1).is_none());
        assert!(store.get(2).is_some());
        assert!(store.get(3).is_some());
        assert!(store.get(4).is_some());
    }

    #[test]
    fn test_scan_store_distance_filter() {
        let config = ScanStoreConfig::default().with_min_distance(0.5);
        let mut store = ScanStore::new(config);

        let cloud = make_point_cloud(10);

        // First scan always stored
        let id1 = store.add_scan(
            Pose2D::identity(),
            Pose2D::new(0.0, 0.0, 0.0),
            &cloud,
            0,
            1.0,
        );
        assert!(id1.is_some());

        // Second scan too close, filtered
        let id2 = store.add_scan(
            Pose2D::identity(),
            Pose2D::new(0.1, 0.0, 0.0),
            &cloud,
            0,
            1.0,
        );
        assert!(id2.is_none());

        // Third scan far enough
        let id3 = store.add_scan(
            Pose2D::identity(),
            Pose2D::new(1.0, 0.0, 0.0),
            &cloud,
            0,
            1.0,
        );
        assert!(id3.is_some());

        assert_eq!(store.len(), 2);
    }

    #[test]
    fn test_scan_store_rotation_filter() {
        let config = ScanStoreConfig::default().with_min_rotation(0.5);
        let mut store = ScanStore::new(config);

        let cloud = make_point_cloud(10);

        store.add_scan(
            Pose2D::identity(),
            Pose2D::new(0.0, 0.0, 0.0),
            &cloud,
            0,
            1.0,
        );

        // Small rotation, filtered
        let id = store.add_scan(
            Pose2D::identity(),
            Pose2D::new(0.0, 0.0, 0.1),
            &cloud,
            0,
            1.0,
        );
        assert!(id.is_none());

        // Large rotation, stored
        let id = store.add_scan(
            Pose2D::identity(),
            Pose2D::new(0.0, 0.0, 1.0),
            &cloud,
            0,
            1.0,
        );
        assert!(id.is_some());
    }

    #[test]
    fn test_scan_store_no_points() {
        let config = ScanStoreConfig::default().with_store_points(false);
        let mut store = ScanStore::new(config);

        let cloud = make_point_cloud(100);
        store.add_scan(Pose2D::identity(), Pose2D::identity(), &cloud, 0, 1.0);

        let scan = store.latest().unwrap();
        assert_eq!(scan.point_count(), 0); // Points not stored
    }

    #[test]
    fn test_scan_store_trajectory() {
        let mut store = ScanStore::with_defaults();

        for i in 0..5 {
            let cloud = make_point_cloud(10);
            store.add_scan(
                Pose2D::identity(),
                Pose2D::new(i as f32, i as f32 * 2.0, 0.0),
                &cloud,
                0,
                1.0,
            );
        }

        let traj = store.trajectory();
        assert_eq!(traj.len(), 5);
        assert!((traj[2].x - 2.0).abs() < 0.01);
        assert!((traj[2].y - 4.0).abs() < 0.01);
    }

    #[test]
    fn test_scan_store_iter_near() {
        let mut store = ScanStore::with_defaults();
        let cloud = make_point_cloud(10);

        // Add scans at various positions
        for i in 0..10 {
            store.add_scan(
                Pose2D::identity(),
                Pose2D::new(i as f32, 0.0, 0.0),
                &cloud,
                0,
                1.0,
            );
        }

        // Find scans near (5, 0) with radius 1.5
        let near: Vec<_> = store.iter_near(Point2D::new(5.0, 0.0), 1.5).collect();
        assert_eq!(near.len(), 3); // scans at x=4, 5, 6
    }

    #[test]
    fn test_scan_store_update_pose() {
        let mut store = ScanStore::with_defaults();
        let cloud = make_point_cloud(10);

        store.add_scan(
            Pose2D::identity(),
            Pose2D::new(1.0, 0.0, 0.0),
            &cloud,
            0,
            1.0,
        );

        let new_pose = Pose2D::new(1.5, 0.5, 0.1);
        assert!(store.update_pose(0, new_pose));

        let scan = store.get(0).unwrap();
        assert!((scan.estimated_pose.x - 1.5).abs() < 0.01);
        assert!((scan.estimated_pose.y - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_scan_store_memory_stats() {
        let mut store = ScanStore::with_defaults();

        for _ in 0..10 {
            let cloud = make_point_cloud(100);
            store.add_scan(Pose2D::identity(), Pose2D::identity(), &cloud, 0, 1.0);
        }

        let stats = store.memory_stats();
        assert_eq!(stats.scan_count, 10);
        assert_eq!(stats.total_points, 1000);
        assert!((stats.avg_points_per_scan - 100.0).abs() < 0.1);
        assert!(stats.total_kb() > 0.0);
    }

    #[test]
    fn test_scan_store_all_world_points() {
        let mut store = ScanStore::with_defaults();

        // Add two scans at different positions
        let cloud1 = make_point_cloud(5); // Points at (0,0), (0.1,0), ...
        store.add_scan(
            Pose2D::identity(),
            Pose2D::new(10.0, 0.0, 0.0), // Translate by (10, 0)
            &cloud1,
            0,
            1.0,
        );

        let cloud2 = make_point_cloud(5);
        store.add_scan(
            Pose2D::identity(),
            Pose2D::new(20.0, 0.0, 0.0), // Translate by (20, 0)
            &cloud2,
            0,
            1.0,
        );

        let all_points = store.all_world_points();
        assert_eq!(all_points.len(), 10);

        // First 5 points should be around x=10
        assert!((all_points[0].x - 10.0).abs() < 0.01);
        // Last 5 points should be around x=20
        assert!((all_points[5].x - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_scan_store_clear_and_reset() {
        let mut store = ScanStore::with_defaults();
        let cloud = make_point_cloud(10);

        store.add_scan(Pose2D::identity(), Pose2D::identity(), &cloud, 0, 1.0);
        store.add_scan(Pose2D::identity(), Pose2D::identity(), &cloud, 0, 1.0);

        assert_eq!(store.next_scan_id(), 2);

        store.clear();
        assert!(store.is_empty());
        assert_eq!(store.next_scan_id(), 2); // ID preserved

        store.add_scan(Pose2D::identity(), Pose2D::identity(), &cloud, 0, 1.0);
        assert_eq!(store.latest().unwrap().scan_id, 2);

        store.reset();
        assert!(store.is_empty());
        assert_eq!(store.next_scan_id(), 0); // ID reset
    }
}
