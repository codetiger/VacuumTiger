//! Point occupancy queries for VectorMap.
//!
//! Determines whether a point is in free space, occupied, or unknown
//! based on the map's line features.

use crate::Occupancy;
use crate::core::{Bounds, Point2D};
use crate::features::Line2D;
use crate::integration::SpatialIndex;

/// Configuration for occupancy queries.
#[derive(Clone, Debug)]
pub struct OccupancyConfig {
    /// Distance threshold for considering a point "on" a line (meters).
    /// Points within this distance of a line are considered occupied.
    /// Default: 0.1m
    pub obstacle_distance: f32,

    /// Distance threshold for "near" checks (meters).
    /// Points must be within this distance of some map feature to be known.
    /// Default: 2.0m
    pub known_distance: f32,

    /// Whether to use winding number for inside/outside determination.
    /// If true, uses contour-based occupancy (like font rendering).
    /// If false, uses simple distance-based occupancy.
    /// Default: false (distance-based is simpler and usually sufficient)
    pub use_winding_number: bool,
}

impl Default for OccupancyConfig {
    fn default() -> Self {
        Self {
            obstacle_distance: 0.1,
            known_distance: 2.0,
            use_winding_number: false,
        }
    }
}

impl OccupancyConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for obstacle distance.
    pub fn with_obstacle_distance(mut self, meters: f32) -> Self {
        self.obstacle_distance = meters;
        self
    }

    /// Builder-style setter for known distance.
    pub fn with_known_distance(mut self, meters: f32) -> Self {
        self.known_distance = meters;
        self
    }
}

/// Query point occupancy.
///
/// # Arguments
/// * `point` - Point to query
/// * `lines` - Map lines
/// * `bounds` - Map bounds (for unknown check)
/// * `config` - Occupancy configuration
///
/// # Returns
/// Occupancy state of the point.
pub fn query_occupancy(
    point: Point2D,
    lines: &[Line2D],
    bounds: Option<&Bounds>,
    config: &OccupancyConfig,
) -> Occupancy {
    // First check if point is outside map bounds
    if let Some(b) = bounds
        && !b.contains(point)
    {
        return Occupancy::Unknown;
    }

    if lines.is_empty() {
        return Occupancy::Unknown;
    }

    // Find minimum distance to any line
    let mut min_distance = f32::MAX;
    let mut has_nearby_line = false;

    for line in lines {
        let dist = line.distance_to_point(point);
        if dist < min_distance {
            min_distance = dist;
        }
        if dist < config.known_distance {
            has_nearby_line = true;
        }
    }

    // Point is very close to a line - occupied
    if min_distance < config.obstacle_distance {
        return Occupancy::Occupied;
    }

    // Point is within known region - free
    if has_nearby_line {
        return Occupancy::Free;
    }

    // Point is far from all lines - unknown
    Occupancy::Unknown
}

/// Query occupancy using spatial index for efficiency.
pub fn query_occupancy_indexed(
    point: Point2D,
    lines: &[Line2D],
    index: &SpatialIndex,
    bounds: Option<&Bounds>,
    config: &OccupancyConfig,
) -> Occupancy {
    // First check if point is outside map bounds
    if let Some(b) = bounds
        && !b.contains(point)
    {
        return Occupancy::Unknown;
    }

    if lines.is_empty() {
        return Occupancy::Unknown;
    }

    // Use spatial index to find nearest line
    if let Some((_idx, dist)) = index.nearest_line(point) {
        // Point is very close to a line - occupied
        if dist < config.obstacle_distance {
            return Occupancy::Occupied;
        }

        // Check if any lines are within known distance
        let nearby = index.lines_within_distance(point, config.known_distance);
        if !nearby.is_empty() {
            return Occupancy::Free;
        }
    }

    // No nearby lines - unknown
    Occupancy::Unknown
}

/// Query occupancy for multiple points.
///
/// Returns occupancy for each point.
pub fn query_occupancy_batch(
    points: &[Point2D],
    lines: &[Line2D],
    bounds: Option<&Bounds>,
    config: &OccupancyConfig,
) -> Vec<Occupancy> {
    points
        .iter()
        .map(|&p| query_occupancy(p, lines, bounds, config))
        .collect()
}

/// Check if a path is collision-free.
///
/// # Arguments
/// * `start` - Path start point
/// * `end` - Path end point
/// * `lines` - Map lines
/// * `config` - Occupancy configuration
///
/// # Returns
/// True if the path doesn't collide with any obstacles.
pub fn is_path_clear(
    start: Point2D,
    end: Point2D,
    lines: &[Line2D],
    config: &OccupancyConfig,
) -> bool {
    // Create a line segment representing the path
    let path = Line2D::new(start, end);
    let path_len = path.length();

    if path_len < f32::EPSILON {
        // Zero-length path - just check the point
        return query_occupancy(start, lines, None, config) != Occupancy::Occupied;
    }

    // Check for intersections with map lines
    for line in lines {
        if path.intersection(line).is_some() {
            return false;
        }

        // Also check if path passes too close to any line
        // Sample points along the path
        let num_samples = (path_len / config.obstacle_distance).ceil() as usize;
        let num_samples = num_samples.max(2);

        for i in 0..=num_samples {
            let t = i as f32 / num_samples as f32;
            let point = path.point_at(t);
            let dist = line.distance_to_point(point);

            if dist < config.obstacle_distance {
                return false;
            }
        }
    }

    true
}

/// Check if a circular region is collision-free.
///
/// # Arguments
/// * `center` - Circle center
/// * `radius` - Circle radius
/// * `lines` - Map lines
///
/// # Returns
/// True if no obstacles within the circle.
pub fn is_region_clear(center: Point2D, radius: f32, lines: &[Line2D]) -> bool {
    for line in lines {
        let dist = line.distance_to_point(center);
        if dist < radius {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_room() -> Vec<Line2D> {
        vec![
            Line2D::new(Point2D::new(-5.0, -5.0), Point2D::new(5.0, -5.0)), // Bottom
            Line2D::new(Point2D::new(5.0, -5.0), Point2D::new(5.0, 5.0)),   // Right
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(-5.0, 5.0)),   // Top
            Line2D::new(Point2D::new(-5.0, 5.0), Point2D::new(-5.0, -5.0)), // Left
        ]
    }

    #[test]
    fn test_query_center_is_free() {
        let lines = make_room();
        // Center is 5m from walls, so need larger known_distance
        let config = OccupancyConfig::default().with_known_distance(6.0);

        let result = query_occupancy(Point2D::zero(), &lines, None, &config);

        assert_eq!(result, Occupancy::Free);
    }

    #[test]
    fn test_query_on_wall_is_occupied() {
        let lines = make_room();
        let config = OccupancyConfig::default();

        // Point on the bottom wall
        let result = query_occupancy(Point2D::new(0.0, -5.0), &lines, None, &config);

        assert_eq!(result, Occupancy::Occupied);
    }

    #[test]
    fn test_query_near_wall_is_occupied() {
        let lines = make_room();
        let config = OccupancyConfig::default().with_obstacle_distance(0.2);

        // Point very close to wall
        let result = query_occupancy(Point2D::new(0.0, -4.9), &lines, None, &config);

        assert_eq!(result, Occupancy::Occupied);
    }

    #[test]
    fn test_query_outside_bounds_is_unknown() {
        let lines = make_room();
        let bounds = Bounds::new(Point2D::new(-6.0, -6.0), Point2D::new(6.0, 6.0));
        let config = OccupancyConfig::default();

        // Point outside bounds
        let result = query_occupancy(Point2D::new(10.0, 10.0), &lines, Some(&bounds), &config);

        assert_eq!(result, Occupancy::Unknown);
    }

    #[test]
    fn test_query_far_from_lines_is_unknown() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0))];
        let config = OccupancyConfig::default().with_known_distance(1.0);

        // Point far from the only line
        let result = query_occupancy(Point2D::new(10.0, 10.0), &lines, None, &config);

        assert_eq!(result, Occupancy::Unknown);
    }

    #[test]
    fn test_query_empty_map_is_unknown() {
        let lines: Vec<Line2D> = vec![];
        let config = OccupancyConfig::default();

        let result = query_occupancy(Point2D::zero(), &lines, None, &config);

        assert_eq!(result, Occupancy::Unknown);
    }

    #[test]
    fn test_query_batch() {
        let lines = make_room();
        // Center is 5m from walls, so need larger known_distance
        let config = OccupancyConfig::default().with_known_distance(6.0);

        let points = vec![
            Point2D::zero(),         // Free (center)
            Point2D::new(0.0, -5.0), // Occupied (on wall)
            Point2D::new(2.0, 0.0),  // Free (inside room, near walls)
        ];

        let results = query_occupancy_batch(&points, &lines, None, &config);

        assert_eq!(results[0], Occupancy::Free);
        assert_eq!(results[1], Occupancy::Occupied);
        assert_eq!(results[2], Occupancy::Free);
    }

    #[test]
    fn test_is_path_clear_straight() {
        let lines = make_room();
        let config = OccupancyConfig::default();

        // Path through center - should be clear
        let clear = is_path_clear(
            Point2D::new(-2.0, 0.0),
            Point2D::new(2.0, 0.0),
            &lines,
            &config,
        );

        assert!(clear);
    }

    #[test]
    fn test_is_path_blocked() {
        let lines = make_room();
        let config = OccupancyConfig::default();

        // Path through wall - should be blocked
        let clear = is_path_clear(
            Point2D::new(0.0, 0.0),
            Point2D::new(10.0, 0.0), // Goes through right wall
            &lines,
            &config,
        );

        assert!(!clear);
    }

    #[test]
    fn test_is_path_too_close() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))];
        let config = OccupancyConfig::default().with_obstacle_distance(0.2);

        // Path very close to line
        let clear = is_path_clear(
            Point2D::new(0.0, 0.1),
            Point2D::new(10.0, 0.1),
            &lines,
            &config,
        );

        assert!(!clear);
    }

    #[test]
    fn test_is_region_clear() {
        let lines = make_room();

        // Center region should be clear
        assert!(is_region_clear(Point2D::zero(), 2.0, &lines));

        // Region touching wall should not be clear
        assert!(!is_region_clear(Point2D::new(4.0, 0.0), 2.0, &lines));
    }
}
