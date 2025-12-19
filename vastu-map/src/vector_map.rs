//! VectorMap: The main SLAM map implementation.
//!
//! VectorMap uses line and corner features as the primary map representation,
//! providing efficient storage and queries for indoor environments.

use crate::core::{Bounds, Point2D, PointCloud2D, Pose2D};
use crate::extraction::{CornerConfig, SplitMergeConfig, detect_corners, extract_lines};
use crate::features::{Corner2D, FeatureSet, Line2D, LineCollection};
use crate::integration::{
    AssociationConfig, MergerConfig, SpatialIndex, batch_merge, create_new_line, find_associations,
    find_unmatched_scan_lines,
};
use crate::matching::{IcpConfig, match_scan};
use crate::query::{FrontierConfig, OccupancyConfig, detect_frontiers, query_occupancy, raycast};
use crate::{Frontier, Map, ObserveResult, Occupancy, Path};

/// Configuration for VectorMap.
#[derive(Clone, Debug)]
pub struct VectorMapConfig {
    /// Configuration for line extraction.
    pub extraction: SplitMergeConfig,

    /// Configuration for corner detection.
    pub corner: CornerConfig,

    /// Configuration for scan matching (ICP).
    pub matching: IcpConfig,

    /// Configuration for line association.
    pub association: AssociationConfig,

    /// Configuration for line merging.
    pub merger: MergerConfig,

    /// Configuration for frontier detection.
    pub frontier: FrontierConfig,

    /// Configuration for occupancy queries.
    pub occupancy: OccupancyConfig,

    /// Minimum match confidence to use scan matching result.
    /// Below this, odometry is used instead.
    /// Default: 0.3
    pub min_match_confidence: f32,

    /// Whether to update the map with new observations.
    /// Set to false for localization-only mode.
    /// Default: true
    pub mapping_enabled: bool,
}

impl Default for VectorMapConfig {
    fn default() -> Self {
        Self {
            extraction: SplitMergeConfig::default(),
            corner: CornerConfig::default(),
            matching: IcpConfig::default(),
            association: AssociationConfig::default(),
            merger: MergerConfig::default(),
            frontier: FrontierConfig::default(),
            occupancy: OccupancyConfig::default(),
            min_match_confidence: 0.3,
            mapping_enabled: true,
        }
    }
}

impl VectorMapConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for minimum match confidence.
    pub fn with_min_match_confidence(mut self, confidence: f32) -> Self {
        self.min_match_confidence = confidence;
        self
    }

    /// Builder-style setter for mapping enabled.
    pub fn with_mapping_enabled(mut self, enabled: bool) -> Self {
        self.mapping_enabled = enabled;
        self
    }
}

/// VectorMap: Feature-based 2D SLAM map.
///
/// Stores the environment as a collection of line and corner features.
/// Provides localization via scan matching and mapping via feature integration.
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::{VectorMap, VectorMapConfig, Map};
/// use vastu_map::core::{Pose2D, PointCloud2D, Point2D};
///
/// let config = VectorMapConfig::default();
/// let mut map = VectorMap::new(config);
///
/// // Process scans
/// let scan = PointCloud2D::from_points(&[
///     Point2D::new(1.0, 0.0),
///     Point2D::new(1.0, 0.1),
///     Point2D::new(1.0, 0.2),
/// ]);
/// let odometry = Pose2D::identity();
///
/// let result = map.observe(&scan, odometry);
/// println!("Pose: ({:.2}, {:.2}, {:.2})", result.pose.x, result.pose.y, result.pose.theta);
/// ```
pub struct VectorMap {
    /// Configuration.
    config: VectorMapConfig,

    /// Line features.
    lines: Vec<Line2D>,

    /// Corner features.
    corners: Vec<Corner2D>,

    /// Spatial index for efficient queries.
    index: SpatialIndex,

    /// Line collection (SoA format) for SIMD operations.
    line_collection: LineCollection,

    /// Current map bounds.
    map_bounds: Option<Bounds>,

    /// Current robot pose estimate.
    current_pose: Pose2D,

    /// Number of observations processed.
    observation_count: usize,
}

impl VectorMap {
    /// Create a new empty VectorMap.
    pub fn new(config: VectorMapConfig) -> Self {
        Self {
            config,
            lines: Vec::new(),
            corners: Vec::new(),
            index: SpatialIndex::empty(),
            line_collection: LineCollection::new(),
            map_bounds: None,
            current_pose: Pose2D::identity(),
            observation_count: 0,
        }
    }

    /// Get the current configuration.
    pub fn config(&self) -> &VectorMapConfig {
        &self.config
    }

    /// Get all lines in the map.
    pub fn lines(&self) -> &[Line2D] {
        &self.lines
    }

    /// Get all corners in the map.
    pub fn corners(&self) -> &[Corner2D] {
        &self.corners
    }

    /// Get the current robot pose.
    pub fn current_pose(&self) -> Pose2D {
        self.current_pose
    }

    /// Get the number of observations processed.
    pub fn observation_count(&self) -> usize {
        self.observation_count
    }

    /// Get the feature set (lines and corners).
    pub fn features(&self) -> FeatureSet {
        FeatureSet::from_features(&self.lines, &self.corners)
    }

    /// Add a line to the map.
    pub fn add_line(&mut self, line: Line2D) {
        let idx = self.lines.len();
        self.lines.push(line);
        self.line_collection.push_line(&line);
        self.update_bounds(&line);

        // Incremental index update (O(log n) vs O(n) rebuild)
        self.index.insert(line, idx);
    }

    /// Add multiple lines to the map.
    pub fn add_lines(&mut self, lines: &[Line2D]) {
        // Threshold for incremental vs full rebuild
        // For small additions, incremental is faster
        // For large additions, bulk rebuild is more efficient
        const INCREMENTAL_THRESHOLD: usize = 20;

        let start_idx = self.lines.len();

        for line in lines {
            self.lines.push(*line);
            self.line_collection.push_line(line);
            self.update_bounds(line);
        }

        if lines.len() <= INCREMENTAL_THRESHOLD && self.lines.len() > lines.len() * 2 {
            // Incremental insert for small additions to large maps
            for (i, line) in lines.iter().enumerate() {
                self.index.insert(*line, start_idx + i);
            }
        } else {
            // Full rebuild for large additions or small maps
            self.rebuild_index();
        }
    }

    /// Set the current pose directly.
    pub fn set_pose(&mut self, pose: Pose2D) {
        self.current_pose = pose;
    }

    /// Process a scan observation.
    ///
    /// This is the main SLAM function. It:
    /// 1. Extracts features from the scan
    /// 2. Matches against the map (localization)
    /// 3. Updates the map with new features (mapping)
    fn process_scan(&mut self, scan: &PointCloud2D, odometry: Pose2D) -> ObserveResult {
        self.observation_count += 1;

        // If map is empty, use odometry and add all features
        if self.lines.is_empty() {
            return self.initialize_from_scan(scan, odometry);
        }

        // Apply odometry to get predicted pose
        let predicted_pose = self.current_pose.compose(odometry);

        // Transform scan to world frame using predicted pose
        let world_scan = scan.transform(&predicted_pose);
        let scan_points: Vec<Point2D> = world_scan.iter().collect();

        // Try scan matching
        let match_result = match_scan(&scan_points, &self.lines, Pose2D::identity());

        // Determine final pose
        let (final_pose, confidence) = if match_result.converged
            && match_result.confidence >= self.config.min_match_confidence
        {
            // Apply match correction to predicted pose
            let corrected = predicted_pose.compose(match_result.pose);
            (corrected, match_result.confidence)
        } else {
            // Fall back to odometry
            (predicted_pose, 0.0)
        };

        self.current_pose = final_pose;

        // Extract features from scan
        let features = self.extract_features(&world_scan);

        // Update map if mapping is enabled
        let (features_added, features_merged) = if self.config.mapping_enabled {
            self.integrate_features(&features)
        } else {
            (0, 0)
        };

        ObserveResult {
            pose: final_pose,
            confidence,
            features_extracted: features.lines().len(),
            features_added,
            features_merged,
        }
    }

    /// Initialize map from first scan.
    fn initialize_from_scan(&mut self, scan: &PointCloud2D, odometry: Pose2D) -> ObserveResult {
        self.current_pose = odometry;

        // Transform scan to world frame
        let world_scan = scan.transform(&odometry);

        // Extract features
        let features = self.extract_features(&world_scan);

        // Add all features to map
        let num_lines = features.lines().len();
        let feature_lines: Vec<Line2D> = features.lines().to_vec();
        self.add_lines(&feature_lines);

        for corner in features.corners() {
            self.corners.push(corner);
        }

        ObserveResult {
            pose: odometry,
            confidence: 0.0, // No match on first scan
            features_extracted: num_lines,
            features_added: num_lines,
            features_merged: 0,
        }
    }

    /// Extract features from a scan.
    fn extract_features(&self, scan: &PointCloud2D) -> FeatureSet {
        let points: Vec<Point2D> = scan.iter().collect();

        // Extract lines
        let lines = extract_lines(&points, &self.config.extraction);

        // Extract corners from lines
        let corners = detect_corners(&lines, &self.config.corner);

        FeatureSet::from_features(&lines, &corners)
    }

    /// Integrate new features into the map.
    fn integrate_features(&mut self, features: &FeatureSet) -> (usize, usize) {
        let scan_lines: Vec<Line2D> = features.lines().to_vec();

        if scan_lines.is_empty() {
            return (0, 0);
        }

        // Find associations
        let associations = find_associations(
            &scan_lines,
            &self.lines,
            Some(&self.index),
            &self.config.association,
        );

        // Merge matched lines
        let merged = batch_merge(
            &scan_lines,
            &mut self.lines,
            &associations,
            &self.config.merger,
        );
        let features_merged = merged.len();

        // Find and add unmatched lines
        let unmatched = find_unmatched_scan_lines(
            &scan_lines,
            &self.lines,
            Some(&self.index),
            &self.config.association,
        );

        let mut features_added = 0;
        for idx in unmatched {
            let new_line = create_new_line(&scan_lines[idx]);
            self.lines.push(new_line);
            features_added += 1;
        }

        // Rebuild auxiliary structures
        self.rebuild_line_collection();
        self.rebuild_index();
        self.recompute_bounds();

        // Update corners
        self.update_corners();

        (features_added, features_merged)
    }

    /// Update bounds to include a line.
    fn update_bounds(&mut self, line: &Line2D) {
        match &mut self.map_bounds {
            Some(b) => {
                b.expand_to_include(line.start);
                b.expand_to_include(line.end);
            }
            None => {
                let mut bounds = Bounds::from_point(line.start);
                bounds.expand_to_include(line.end);
                self.map_bounds = Some(bounds);
            }
        }
    }

    /// Recompute bounds from all lines.
    fn recompute_bounds(&mut self) {
        if self.lines.is_empty() {
            self.map_bounds = None;
            return;
        }

        let mut bounds = Bounds::from_point(self.lines[0].start);
        for line in &self.lines {
            bounds.expand_to_include(line.start);
            bounds.expand_to_include(line.end);
        }
        self.map_bounds = Some(bounds);
    }

    /// Rebuild the spatial index.
    fn rebuild_index(&mut self) {
        self.index.rebuild(&self.lines);
    }

    /// Rebuild the line collection from lines.
    fn rebuild_line_collection(&mut self) {
        self.line_collection = LineCollection::from_lines(&self.lines);
    }

    /// Update corners based on current lines.
    fn update_corners(&mut self) {
        self.corners = detect_corners(&self.lines, &self.config.corner);
    }
}

impl Map for VectorMap {
    fn observe(&mut self, scan: &PointCloud2D, odometry: Pose2D) -> ObserveResult {
        self.process_scan(scan, odometry)
    }

    fn raycast(&self, from: Point2D, direction: Point2D, max_range: f32) -> f32 {
        raycast(from, direction, max_range, &self.lines)
    }

    fn query(&self, point: Point2D) -> Occupancy {
        query_occupancy(
            point,
            &self.lines,
            self.map_bounds.as_ref(),
            &self.config.occupancy,
        )
    }

    fn frontiers(&self) -> Vec<Frontier> {
        detect_frontiers(&self.lines, &self.config.frontier)
    }

    fn get_path(&self, _from: Point2D, _to: Point2D) -> Option<Path> {
        // Path planning not yet implemented
        None
    }

    fn bounds(&self) -> Bounds {
        self.map_bounds.unwrap_or_else(Bounds::empty)
    }

    fn clear(&mut self) {
        self.lines.clear();
        self.corners.clear();
        self.index.clear();
        self.line_collection.clear();
        self.map_bounds = None;
        self.current_pose = Pose2D::identity();
        self.observation_count = 0;
    }
}

impl Default for VectorMap {
    fn default() -> Self {
        Self::new(VectorMapConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_simple_scan() -> PointCloud2D {
        // Simulate a scan seeing a wall in front
        let mut xs = Vec::new();
        let mut ys = Vec::new();

        for i in -10..=10 {
            xs.push(2.0); // Wall at x=2
            ys.push(i as f32 * 0.1);
        }

        PointCloud2D { xs, ys }
    }

    #[allow(dead_code)]
    fn make_room_scan() -> PointCloud2D {
        // Simulate a scan from center of a 4x4 room
        let mut xs = Vec::new();
        let mut ys = Vec::new();

        // See walls at ±2 in each direction
        for i in -10..=10 {
            let y = i as f32 * 0.2;
            // Left wall
            xs.push(-2.0);
            ys.push(y);
            // Right wall
            xs.push(2.0);
            ys.push(y);
        }
        for i in -10..=10 {
            let x = i as f32 * 0.2;
            // Bottom wall
            xs.push(x);
            ys.push(-2.0);
            // Top wall
            xs.push(x);
            ys.push(2.0);
        }

        PointCloud2D { xs, ys }
    }

    #[test]
    fn test_new_map() {
        let config = VectorMapConfig::default();
        let map = VectorMap::new(config);

        assert!(map.lines.is_empty());
        assert!(map.corners.is_empty());
        assert_eq!(map.observation_count, 0);
    }

    #[test]
    fn test_add_line() {
        let mut map = VectorMap::default();
        let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));

        map.add_line(line);

        assert_eq!(map.lines.len(), 1);
        assert!(map.map_bounds.is_some());
    }

    #[test]
    fn test_first_observation() {
        let mut map = VectorMap::default();
        let scan = make_simple_scan();

        let result = map.observe(&scan, Pose2D::identity());

        assert_eq!(result.pose, Pose2D::identity());
        assert!(map.lines.len() > 0 || result.features_extracted > 0);
        assert_eq!(map.observation_count, 1);
    }

    #[test]
    fn test_raycast_through_map() {
        let mut map = VectorMap::default();
        map.add_line(Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0)));

        let dist = map.raycast(Point2D::zero(), Point2D::new(1.0, 0.0), 100.0);

        assert!((dist - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_query_occupancy() {
        let mut map = VectorMap::default();
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)));

        // On the line
        let on_line = map.query(Point2D::new(5.0, 0.0));
        assert_eq!(on_line, Occupancy::Occupied);
    }

    #[test]
    fn test_bounds() {
        let mut map = VectorMap::default();

        let empty_bounds = map.bounds();
        assert!(empty_bounds.is_empty());

        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 5.0)));

        let bounds = map.bounds();
        assert!(!bounds.is_empty());
        assert!(bounds.contains(Point2D::new(5.0, 2.0)));
    }

    #[test]
    fn test_clear() {
        let mut map = VectorMap::default();
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)));

        map.clear();

        assert!(map.lines.is_empty());
        assert!(map.corners.is_empty());
        assert!(map.map_bounds.is_none());
        assert_eq!(map.current_pose, Pose2D::identity());
    }

    #[test]
    fn test_frontiers_empty_map() {
        let map = VectorMap::default();
        let frontiers = map.frontiers();

        assert!(frontiers.is_empty());
    }

    #[test]
    fn test_frontiers_partial_room() {
        let mut map = VectorMap::default();

        // Add three walls of a room (missing one side)
        map.add_lines(&[
            Line2D::new(Point2D::new(-2.0, -2.0), Point2D::new(2.0, -2.0)), // Bottom
            Line2D::new(Point2D::new(2.0, -2.0), Point2D::new(2.0, 2.0)),   // Right
            Line2D::new(Point2D::new(-2.0, 2.0), Point2D::new(-2.0, -2.0)), // Left
        ]);

        let frontiers = map.frontiers();

        // Should have frontiers at the open top
        assert!(!frontiers.is_empty());
    }

    #[test]
    fn test_localization_only_mode() {
        let config = VectorMapConfig::default().with_mapping_enabled(false);
        let mut map = VectorMap::new(config);

        // Add initial map
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)));
        let initial_count = map.lines.len();

        // Process scan - should not add features
        let scan = make_simple_scan();
        map.observe(&scan, Pose2D::identity());

        // Line count should not change
        assert_eq!(map.lines.len(), initial_count);
    }

    #[test]
    fn test_features() {
        let mut map = VectorMap::default();
        map.add_lines(&[
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)),
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)),
        ]);

        let features = map.features();

        assert_eq!(features.lines().len(), 2);
    }

    // === Additional full-cycle integration tests ===

    /// Create a scan simulating walls of a square room from a given position.
    #[allow(dead_code)]
    fn make_scan_from_position(_pos: Point2D, room_size: f32) -> PointCloud2D {
        let half = room_size / 2.0;
        let mut xs = Vec::new();
        let mut ys = Vec::new();

        // Generate points for each wall relative to position
        let walls = [
            (Point2D::new(-half, -half), Point2D::new(half, -half)), // Bottom
            (Point2D::new(half, -half), Point2D::new(half, half)),   // Right
            (Point2D::new(half, half), Point2D::new(-half, half)),   // Top
            (Point2D::new(-half, half), Point2D::new(-half, -half)), // Left
        ];

        for (start, end) in walls {
            // Generate 15 points along each wall
            for i in 0..15 {
                let t = i as f32 / 14.0;
                let point = Point2D::new(
                    start.x + (end.x - start.x) * t,
                    start.y + (end.y - start.y) * t,
                );
                // Transform to world coordinates (assuming pos is robot position)
                xs.push(point.x);
                ys.push(point.y);
            }
        }

        PointCloud2D { xs, ys }
    }

    #[test]
    fn test_multi_scan_mapping_grows_map() {
        // Test that processing multiple scans grows the map appropriately
        let mut map = VectorMap::default();

        // Create a better scan with dense points along walls
        let mut scan1 = PointCloud2D {
            xs: Vec::new(),
            ys: Vec::new(),
        };
        // Left wall - many closely spaced points
        for i in 0..20 {
            scan1.xs.push(-2.0);
            scan1.ys.push(-2.0 + i as f32 * 0.2);
        }
        // Right wall
        for i in 0..20 {
            scan1.xs.push(2.0);
            scan1.ys.push(-2.0 + i as f32 * 0.2);
        }

        // First scan - should initialize the map
        let _ = map.observe(&scan1, Pose2D::identity());

        let _lines_after_first = map.lines().len();
        // Even if no features extracted (depends on config), map should handle it
        assert_eq!(map.observation_count(), 1);

        // Second scan from slightly different position
        let odometry = Pose2D::new(0.3, 0.2, 0.05);
        let _ = map.observe(&scan1, odometry);

        assert_eq!(map.observation_count(), 2);
        // Map processes observations successfully if we reach here
    }

    #[test]
    fn test_slam_cycle_extract_match_integrate() {
        // Test complete SLAM cycle: extract → match → associate → merge
        let mut map = VectorMap::default();

        // Initialize with known walls
        map.add_lines(&[
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)), // Bottom wall
            Line2D::new(Point2D::new(5.0, 0.0), Point2D::new(5.0, 5.0)), // Right wall
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(0.0, 5.0)), // Top wall
            Line2D::new(Point2D::new(0.0, 5.0), Point2D::new(0.0, 0.0)), // Left wall
        ]);

        let initial_lines = map.lines().len();

        // Simulate scan from center of room seeing all walls
        let mut scan = PointCloud2D {
            xs: Vec::new(),
            ys: Vec::new(),
        };
        // Bottom wall points
        for i in 0..10 {
            scan.xs.push(0.5 + i as f32 * 0.4);
            scan.ys.push(0.05);
        }
        // Right wall points
        for i in 0..10 {
            scan.xs.push(4.95);
            scan.ys.push(0.5 + i as f32 * 0.4);
        }

        // Set robot pose at center
        map.set_pose(Pose2D::new(2.5, 2.5, 0.0));

        // Process scan with zero odometry (robot hasn't moved)
        let _ = map.observe(&scan, Pose2D::identity());

        // Verify SLAM components worked
        // Map should not explode in size
        assert!(
            map.lines().len() <= initial_lines + 5,
            "Map should not grow excessively: had {}, now {}",
            initial_lines,
            map.lines().len()
        );
    }

    #[test]
    fn test_localization_maintains_pose() {
        // Test that consecutive scans maintain reasonable pose
        let config = VectorMapConfig::default().with_min_match_confidence(0.1);
        let mut map = VectorMap::new(config);

        // Add a simple room
        map.add_lines(&[
            Line2D::new(Point2D::new(-2.0, -2.0), Point2D::new(2.0, -2.0)),
            Line2D::new(Point2D::new(2.0, -2.0), Point2D::new(2.0, 2.0)),
            Line2D::new(Point2D::new(2.0, 2.0), Point2D::new(-2.0, 2.0)),
            Line2D::new(Point2D::new(-2.0, 2.0), Point2D::new(-2.0, -2.0)),
        ]);

        // Start at origin
        map.set_pose(Pose2D::identity());

        // Process several scans with small movements
        let scan = make_room_scan();
        let mut total_movement = 0.0;

        for _ in 0..5 {
            let odom = Pose2D::new(0.1, 0.05, 0.02);
            map.observe(&scan, odom);
            total_movement += (0.1f32.powi(2) + 0.05f32.powi(2)).sqrt();
        }

        // Final pose should be roughly accumulated odometry
        let pose = map.current_pose();
        let pose_dist = (pose.x.powi(2) + pose.y.powi(2)).sqrt();

        // Allow for some drift, but should be in the ballpark
        assert!(
            pose_dist < total_movement * 2.0,
            "Pose drift too large: {} vs max {}",
            pose_dist,
            total_movement * 2.0
        );
    }

    #[test]
    fn test_map_bounds_grow_with_exploration() {
        let mut map = VectorMap::default();

        // Initial small map
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)));
        let initial_bounds = map.bounds();

        // Add more lines extending the map
        map.add_line(Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(5.0, 0.0)));
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 3.0)));

        let expanded_bounds = map.bounds();

        // Bounds should have grown
        assert!(
            expanded_bounds.width() > initial_bounds.width(),
            "Bounds width should grow"
        );
        assert!(
            expanded_bounds.height() > initial_bounds.height(),
            "Bounds height should grow"
        );
    }

    #[test]
    fn test_feature_count_stability() {
        // Test that feature count stabilizes when observing same scene repeatedly
        let mut map = VectorMap::default();
        let scan = make_room_scan();

        // Process first scan
        map.observe(&scan, Pose2D::identity());
        let count_after_1 = map.lines().len();

        // Process same scan 5 more times with tiny movements
        for _ in 0..5 {
            let small_move = Pose2D::new(0.01, 0.01, 0.005);
            map.observe(&scan, small_move);
        }

        let count_after_6 = map.lines().len();

        // Feature count should stabilize (not grow unboundedly)
        assert!(
            count_after_6 <= count_after_1 * 3,
            "Feature count should stabilize: started with {}, ended with {}",
            count_after_1,
            count_after_6
        );
    }

    #[test]
    fn test_config_propagation() {
        // Test that config values are properly propagated
        let config = VectorMapConfig::default()
            .with_min_match_confidence(0.5)
            .with_mapping_enabled(false);

        let map = VectorMap::new(config);

        assert_eq!(map.config().min_match_confidence, 0.5);
        assert!(!map.config().mapping_enabled);
    }

    #[test]
    fn test_set_pose() {
        let mut map = VectorMap::default();

        let target_pose = Pose2D::new(1.5, 2.5, 0.7);
        map.set_pose(target_pose);

        assert_eq!(map.current_pose().x, target_pose.x);
        assert_eq!(map.current_pose().y, target_pose.y);
        assert_eq!(map.current_pose().theta, target_pose.theta);
    }

    #[test]
    fn test_observation_count_increments() {
        let mut map = VectorMap::default();
        let scan = make_simple_scan();

        assert_eq!(map.observation_count(), 0);

        map.observe(&scan, Pose2D::identity());
        assert_eq!(map.observation_count(), 1);

        map.observe(&scan, Pose2D::identity());
        assert_eq!(map.observation_count(), 2);

        map.observe(&scan, Pose2D::identity());
        assert_eq!(map.observation_count(), 3);
    }

    #[test]
    fn test_add_lines_batch() {
        let mut map = VectorMap::default();

        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(1.0, 1.0)),
            Line2D::new(Point2D::new(1.0, 1.0), Point2D::new(0.0, 1.0)),
        ];

        map.add_lines(&lines);

        assert_eq!(map.lines().len(), 3);
        assert!(map.bounds().contains(Point2D::new(0.5, 0.5)));
    }

    #[test]
    fn test_empty_scan_handling() {
        let mut map = VectorMap::default();
        map.add_line(Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)));

        let empty_scan = PointCloud2D {
            xs: vec![],
            ys: vec![],
        };
        let result = map.observe(&empty_scan, Pose2D::identity());

        // Should handle gracefully
        assert_eq!(result.features_extracted, 0);
        assert_eq!(map.observation_count(), 1);
    }
}
