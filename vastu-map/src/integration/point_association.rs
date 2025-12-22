//! Point-to-line association for line re-fitting.
//!
//! Associates world-frame points from ScanStore with existing map lines,
//! enabling periodic re-fitting of lines from accumulated point clouds.

use crate::core::Point2D;
use crate::features::Line2D;
use crate::integration::{ScanStore, SpatialIndex};

/// Configuration for point-to-line association.
#[derive(Clone, Debug)]
pub struct PointAssociationConfig {
    /// Maximum perpendicular distance from line to associate a point (meters).
    /// Points further than this from all lines are unassociated.
    /// Default: 0.10m (10cm)
    pub max_distance: f32,

    /// Maximum projection extension beyond line endpoints.
    /// Points can project up to this fraction beyond line ends.
    /// 0.0 = strict segment bounds, 0.2 = 20% extension allowed.
    /// Default: 0.1 (10% extension)
    pub max_projection_extension: f32,
}

impl Default for PointAssociationConfig {
    fn default() -> Self {
        Self {
            max_distance: 0.10,
            max_projection_extension: 0.1,
        }
    }
}

impl PointAssociationConfig {
    /// Create a new configuration with defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set maximum association distance.
    pub fn with_max_distance(mut self, meters: f32) -> Self {
        self.max_distance = meters;
        self
    }

    /// Set maximum projection extension.
    pub fn with_max_projection_extension(mut self, fraction: f32) -> Self {
        self.max_projection_extension = fraction;
        self
    }
}

/// A point associated with a map line.
#[derive(Clone, Debug)]
pub struct AssociatedPoint {
    /// World-frame point.
    pub point: Point2D,
    /// Scan ID the point came from.
    pub scan_id: u32,
    /// Perpendicular distance to the line.
    pub distance: f32,
    /// Projection parameter on the line (0=start, 1=end).
    pub projection_t: f32,
}

/// Result of point-to-line association.
#[derive(Clone, Debug)]
pub struct PointAssociationResult {
    /// Points associated with each map line, indexed by line index.
    pub line_points: Vec<Vec<AssociatedPoint>>,
    /// Number of points associated.
    pub total_associated: usize,
    /// Number of points not associated (too far from all lines).
    pub total_unassociated: usize,
}

impl PointAssociationResult {
    /// Get the total number of points processed.
    pub fn total_points(&self) -> usize {
        self.total_associated + self.total_unassociated
    }

    /// Get association rate (0.0 - 1.0).
    pub fn association_rate(&self) -> f32 {
        let total = self.total_points();
        if total == 0 {
            0.0
        } else {
            self.total_associated as f32 / total as f32
        }
    }
}

/// Associate all points from ScanStore with map lines.
///
/// Each point is associated with its nearest line if within `max_distance`
/// and within projection bounds (considering extension).
///
/// # Arguments
/// * `scan_store` - Storage containing accumulated scans
/// * `lines` - Map lines to associate points with
/// * `index` - Spatial index for efficient line lookup
/// * `config` - Association configuration
///
/// # Returns
/// Association result with points grouped by line index.
pub fn associate_points_to_lines(
    scan_store: &ScanStore,
    lines: &[Line2D],
    index: &SpatialIndex,
    config: &PointAssociationConfig,
) -> PointAssociationResult {
    let num_lines = lines.len();
    let mut line_points: Vec<Vec<AssociatedPoint>> = vec![Vec::new(); num_lines];
    let mut total_associated = 0usize;
    let mut total_unassociated = 0usize;

    // Iterate through all scans and their world-frame points
    for scan in scan_store.iter() {
        let pose = &scan.estimated_pose;

        for robot_point in scan.points() {
            // Transform to world frame
            let world_point = pose.transform_point(*robot_point);

            // Find nearest line using spatial index
            if let Some((line_idx, distance)) = index.nearest_line(world_point) {
                // Check distance threshold
                if distance <= config.max_distance && line_idx < num_lines {
                    let line = &lines[line_idx];

                    // Check projection bounds
                    let t = line.project_point(world_point);
                    if line.contains_projection_extended(t, config.max_projection_extension) {
                        line_points[line_idx].push(AssociatedPoint {
                            point: world_point,
                            scan_id: scan.scan_id,
                            distance,
                            projection_t: t,
                        });
                        total_associated += 1;
                        continue;
                    }
                }
            }
            total_unassociated += 1;
        }
    }

    PointAssociationResult {
        line_points,
        total_associated,
        total_unassociated,
    }
}

/// Associate points from a single scan with map lines.
///
/// Similar to `associate_points_to_lines` but for a single scan.
/// Useful for incremental association during observations.
pub fn associate_scan_points(
    points: &[Point2D],
    scan_id: u32,
    lines: &[Line2D],
    index: &SpatialIndex,
    config: &PointAssociationConfig,
) -> PointAssociationResult {
    let num_lines = lines.len();
    let mut line_points: Vec<Vec<AssociatedPoint>> = vec![Vec::new(); num_lines];
    let mut total_associated = 0usize;
    let mut total_unassociated = 0usize;

    for &world_point in points {
        if let Some((line_idx, distance)) = index.nearest_line(world_point) {
            if distance <= config.max_distance && line_idx < num_lines {
                let line = &lines[line_idx];
                let t = line.project_point(world_point);
                if line.contains_projection_extended(t, config.max_projection_extension) {
                    line_points[line_idx].push(AssociatedPoint {
                        point: world_point,
                        scan_id,
                        distance,
                        projection_t: t,
                    });
                    total_associated += 1;
                    continue;
                }
            }
        }
        total_unassociated += 1;
    }

    PointAssociationResult {
        line_points,
        total_associated,
        total_unassociated,
    }
}

/// Get line indices that have enough points for re-fitting.
///
/// Returns indices of lines with at least `min_points` associated points.
pub fn lines_with_enough_points(result: &PointAssociationResult, min_points: usize) -> Vec<usize> {
    result
        .line_points
        .iter()
        .enumerate()
        .filter(|(_, points)| points.len() >= min_points)
        .map(|(idx, _)| idx)
        .collect()
}

/// Extract just the points for a specific line (without metadata).
///
/// Useful when only the point coordinates are needed for fitting.
pub fn extract_points_for_line(
    result: &PointAssociationResult,
    line_idx: usize,
) -> Vec<Point2D> {
    result
        .line_points
        .get(line_idx)
        .map(|pts| pts.iter().map(|ap| ap.point).collect())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{PointCloud2D, Pose2D};

    fn make_horizontal_line(y: f32, x_start: f32, x_end: f32) -> Line2D {
        Line2D::new(Point2D::new(x_start, y), Point2D::new(x_end, y))
    }

    fn make_vertical_line(x: f32, y_start: f32, y_end: f32) -> Line2D {
        Line2D::new(Point2D::new(x, y_start), Point2D::new(x, y_end))
    }

    fn make_scan_store_with_points(points: &[(f32, f32)]) -> ScanStore {
        use crate::integration::ScanStoreConfig;

        let mut store = ScanStore::new(ScanStoreConfig::default());
        let xs: Vec<f32> = points.iter().map(|(x, _)| *x).collect();
        let ys: Vec<f32> = points.iter().map(|(_, y)| *y).collect();
        let cloud = PointCloud2D { xs, ys };

        // Use identity pose so robot-frame = world-frame
        store.add_scan(Pose2D::identity(), Pose2D::identity(), &cloud, 0, 1.0);
        store
    }

    #[test]
    fn test_config_defaults() {
        let config = PointAssociationConfig::default();
        assert!((config.max_distance - 0.10).abs() < 0.001);
        assert!((config.max_projection_extension - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_config_builder() {
        let config = PointAssociationConfig::new()
            .with_max_distance(0.15)
            .with_max_projection_extension(0.2);
        assert!((config.max_distance - 0.15).abs() < 0.001);
        assert!((config.max_projection_extension - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_empty_inputs() {
        let store = ScanStore::default();
        let lines: Vec<Line2D> = vec![];
        let index = SpatialIndex::empty();
        let config = PointAssociationConfig::default();

        let result = associate_points_to_lines(&store, &lines, &index, &config);
        assert_eq!(result.total_associated, 0);
        assert_eq!(result.total_unassociated, 0);
        assert!(result.line_points.is_empty());
    }

    #[test]
    fn test_points_associate_to_nearest_line() {
        // Create two horizontal lines
        let lines = vec![
            make_horizontal_line(0.0, 0.0, 10.0), // y=0
            make_horizontal_line(5.0, 0.0, 10.0), // y=5
        ];
        let index = SpatialIndex::new(&lines);
        let config = PointAssociationConfig::default().with_max_distance(0.5);

        // Points close to line 0 (y=0)
        let store = make_scan_store_with_points(&[
            (2.0, 0.05),  // Very close to y=0
            (5.0, 0.02),  // Very close to y=0
            (8.0, 4.98),  // Very close to y=5
        ]);

        let result = associate_points_to_lines(&store, &lines, &index, &config);

        assert_eq!(result.total_associated, 3);
        assert_eq!(result.line_points[0].len(), 2); // Two points near y=0
        assert_eq!(result.line_points[1].len(), 1); // One point near y=5
    }

    #[test]
    fn test_points_outside_max_distance() {
        let lines = vec![make_horizontal_line(0.0, 0.0, 10.0)];
        let index = SpatialIndex::new(&lines);
        let config = PointAssociationConfig::default().with_max_distance(0.1);

        // Points too far from line
        let store = make_scan_store_with_points(&[
            (5.0, 0.5),  // 0.5m away, too far
            (5.0, 1.0),  // 1.0m away, too far
        ]);

        let result = associate_points_to_lines(&store, &lines, &index, &config);

        assert_eq!(result.total_associated, 0);
        assert_eq!(result.total_unassociated, 2);
    }

    #[test]
    fn test_points_outside_projection_bounds() {
        let lines = vec![make_horizontal_line(0.0, 0.0, 10.0)];
        let index = SpatialIndex::new(&lines);
        // Strict projection bounds (no extension)
        let config = PointAssociationConfig::default()
            .with_max_distance(0.5)
            .with_max_projection_extension(0.0);

        // Points beyond line endpoints
        let store = make_scan_store_with_points(&[
            (-2.0, 0.0),  // Before start
            (12.0, 0.0),  // After end
            (5.0, 0.0),   // On segment (should associate)
        ]);

        let result = associate_points_to_lines(&store, &lines, &index, &config);

        assert_eq!(result.total_associated, 1);
        assert_eq!(result.total_unassociated, 2);
    }

    #[test]
    fn test_projection_extension() {
        // Line from (0,0) to (10,0), length 10
        let lines = vec![make_horizontal_line(0.0, 0.0, 10.0)];
        let index = SpatialIndex::new(&lines);
        // Allow 10% extension = 1m beyond endpoints
        let config = PointAssociationConfig::default()
            .with_max_distance(0.5)
            .with_max_projection_extension(0.1);

        let store = make_scan_store_with_points(&[
            (-0.5, 0.0),  // 5% before start, within extension
            (10.5, 0.0),  // 5% after end, within extension
            (-2.0, 0.0),  // 20% before start, outside extension
        ]);

        let result = associate_points_to_lines(&store, &lines, &index, &config);

        assert_eq!(result.total_associated, 2);
        assert_eq!(result.total_unassociated, 1);
    }

    #[test]
    fn test_associated_point_metadata() {
        let lines = vec![make_horizontal_line(0.0, 0.0, 10.0)];
        let index = SpatialIndex::new(&lines);
        let config = PointAssociationConfig::default().with_max_distance(0.5);

        let store = make_scan_store_with_points(&[(5.0, 0.1)]);

        let result = associate_points_to_lines(&store, &lines, &index, &config);

        assert_eq!(result.line_points[0].len(), 1);
        let ap = &result.line_points[0][0];
        assert!((ap.point.x - 5.0).abs() < 0.01);
        assert!((ap.point.y - 0.1).abs() < 0.01);
        assert_eq!(ap.scan_id, 0);
        assert!((ap.distance - 0.1).abs() < 0.01); // Perpendicular distance
        assert!((ap.projection_t - 0.5).abs() < 0.01); // Midpoint of line
    }

    #[test]
    fn test_multiple_scans() {
        use crate::integration::ScanStoreConfig;

        let lines = vec![make_horizontal_line(0.0, 0.0, 10.0)];
        let index = SpatialIndex::new(&lines);
        let config = PointAssociationConfig::default().with_max_distance(0.5);

        let mut store = ScanStore::new(ScanStoreConfig::default());

        // Add first scan
        let cloud1 = PointCloud2D {
            xs: vec![2.0, 3.0],
            ys: vec![0.05, 0.05],
        };
        store.add_scan(Pose2D::identity(), Pose2D::identity(), &cloud1, 0, 1.0);

        // Add second scan
        let cloud2 = PointCloud2D {
            xs: vec![7.0, 8.0],
            ys: vec![0.05, 0.05],
        };
        store.add_scan(Pose2D::identity(), Pose2D::identity(), &cloud2, 0, 1.0);

        let result = associate_points_to_lines(&store, &lines, &index, &config);

        assert_eq!(result.total_associated, 4);
        assert_eq!(result.line_points[0].len(), 4);

        // Verify scan IDs are preserved
        let scan_ids: Vec<u32> = result.line_points[0].iter().map(|ap| ap.scan_id).collect();
        assert!(scan_ids.contains(&0));
        assert!(scan_ids.contains(&1));
    }

    #[test]
    fn test_transformed_poses() {
        use crate::integration::ScanStoreConfig;

        // Line at y=10 in world frame
        let lines = vec![make_horizontal_line(10.0, 0.0, 10.0)];
        let index = SpatialIndex::new(&lines);
        let config = PointAssociationConfig::default().with_max_distance(0.5);

        let mut store = ScanStore::new(ScanStoreConfig::default());

        // Robot-frame points at (5, 0) - directly in front of robot
        let cloud = PointCloud2D {
            xs: vec![5.0],
            ys: vec![0.0],
        };

        // Robot at (0, 10), facing +X -> world point at (5, 10)
        let pose = Pose2D::new(0.0, 10.0, 0.0);
        store.add_scan(Pose2D::identity(), pose, &cloud, 0, 1.0);

        let result = associate_points_to_lines(&store, &lines, &index, &config);

        // Point at (5, 10) should associate with line at y=10
        assert_eq!(result.total_associated, 1);
        let ap = &result.line_points[0][0];
        assert!((ap.point.x - 5.0).abs() < 0.01);
        assert!((ap.point.y - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_associate_scan_points() {
        let lines = vec![
            make_horizontal_line(0.0, 0.0, 10.0),
            make_vertical_line(10.0, 0.0, 10.0),
        ];
        let index = SpatialIndex::new(&lines);
        let config = PointAssociationConfig::default().with_max_distance(0.5);

        let points = vec![
            Point2D::new(5.0, 0.05),   // Near line 0
            Point2D::new(9.95, 5.0),   // Near line 1
            Point2D::new(5.0, 5.0),    // Far from both
        ];

        let result = associate_scan_points(&points, 42, &lines, &index, &config);

        assert_eq!(result.total_associated, 2);
        assert_eq!(result.total_unassociated, 1);
        assert_eq!(result.line_points[0].len(), 1);
        assert_eq!(result.line_points[1].len(), 1);
        assert_eq!(result.line_points[0][0].scan_id, 42);
    }

    #[test]
    fn test_lines_with_enough_points() {
        let lines = vec![
            make_horizontal_line(0.0, 0.0, 10.0),
            make_horizontal_line(5.0, 0.0, 10.0),
            make_horizontal_line(10.0, 0.0, 10.0),
        ];
        let index = SpatialIndex::new(&lines);
        let config = PointAssociationConfig::default().with_max_distance(0.5);

        // Create points: 5 near line 0, 3 near line 1, 10 near line 2
        let mut points = vec![];
        for i in 0..5 {
            points.push((i as f32, 0.05)); // Near line 0
        }
        for i in 0..3 {
            points.push((i as f32, 5.05)); // Near line 1
        }
        for i in 0..10 {
            points.push((i as f32, 10.05)); // Near line 2
        }

        let store = make_scan_store_with_points(&points);
        let result = associate_points_to_lines(&store, &lines, &index, &config);

        // Lines with at least 5 points
        let enough = lines_with_enough_points(&result, 5);
        assert_eq!(enough.len(), 2);
        assert!(enough.contains(&0));
        assert!(enough.contains(&2));
        assert!(!enough.contains(&1)); // Only 3 points
    }

    #[test]
    fn test_extract_points_for_line() {
        let lines = vec![make_horizontal_line(0.0, 0.0, 10.0)];
        let index = SpatialIndex::new(&lines);
        let config = PointAssociationConfig::default().with_max_distance(0.5);

        let store = make_scan_store_with_points(&[
            (2.0, 0.05),
            (5.0, 0.05),
            (8.0, 0.05),
        ]);

        let result = associate_points_to_lines(&store, &lines, &index, &config);
        let points = extract_points_for_line(&result, 0);

        assert_eq!(points.len(), 3);
        assert!((points[0].x - 2.0).abs() < 0.01);
        assert!((points[1].x - 5.0).abs() < 0.01);
        assert!((points[2].x - 8.0).abs() < 0.01);
    }

    #[test]
    fn test_association_rate() {
        let lines = vec![make_horizontal_line(0.0, 0.0, 10.0)];
        let index = SpatialIndex::new(&lines);
        let config = PointAssociationConfig::default().with_max_distance(0.1);

        let store = make_scan_store_with_points(&[
            (5.0, 0.05),  // Close enough
            (5.0, 0.5),   // Too far
            (5.0, 0.08),  // Close enough
            (5.0, 1.0),   // Too far
        ]);

        let result = associate_points_to_lines(&store, &lines, &index, &config);

        assert_eq!(result.total_associated, 2);
        assert_eq!(result.total_unassociated, 2);
        assert!((result.association_rate() - 0.5).abs() < 0.001);
        assert_eq!(result.total_points(), 4);
    }
}
