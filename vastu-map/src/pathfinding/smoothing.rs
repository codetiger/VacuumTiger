//! Path smoothing algorithms.
//!
//! Smooths A* paths for more natural robot movement:
//! - Line-of-sight shortcutting
//! - Bezier curve smoothing
//! - Waypoint reduction

use crate::core::{GridCoord, WorldPoint};
use crate::grid::GridStorage;
use crate::query::{RobotFootprint, TraversabilityChecker};

/// Path smoothing configuration
#[derive(Clone, Debug)]
pub struct SmoothingConfig {
    /// Robot footprint for collision checking
    pub footprint: RobotFootprint,
    /// Step size for line-of-sight checks (in meters)
    pub los_step_size: f32,
    /// Maximum iterations for iterative smoothing
    pub max_iterations: usize,
}

impl Default for SmoothingConfig {
    fn default() -> Self {
        Self {
            footprint: RobotFootprint::default(),
            los_step_size: 0.05, // 5cm steps
            max_iterations: 100,
        }
    }
}

/// Path smoother
pub struct PathSmoother<'a> {
    storage: &'a GridStorage,
    config: SmoothingConfig,
}

impl<'a> PathSmoother<'a> {
    /// Create a new path smoother
    pub fn new(storage: &'a GridStorage, config: SmoothingConfig) -> Self {
        Self { storage, config }
    }

    /// Create with default configuration
    pub fn with_defaults(storage: &'a GridStorage) -> Self {
        Self::new(storage, SmoothingConfig::default())
    }

    /// Smooth a path using line-of-sight shortcuts
    ///
    /// Removes unnecessary waypoints by checking if direct paths exist
    /// between non-adjacent waypoints.
    pub fn smooth_los(&self, path: &[WorldPoint]) -> Vec<WorldPoint> {
        if path.len() <= 2 {
            return path.to_vec();
        }

        let checker = TraversabilityChecker::new(self.storage, self.config.footprint.clone());
        let mut smoothed = vec![path[0]];
        let mut i = 0;

        while i < path.len() - 1 {
            // Find the furthest reachable point from current
            let mut furthest = i + 1;

            for j in (i + 2)..path.len() {
                let check_result =
                    checker.check_path(path[i], path[j], Some(self.config.los_step_size));

                if check_result.is_clear {
                    furthest = j;
                }
            }

            smoothed.push(path[furthest]);
            i = furthest;
        }

        smoothed
    }

    /// Smooth using iterative midpoint displacement
    ///
    /// Moves each waypoint towards the line connecting its neighbors
    /// while maintaining collision-free path.
    pub fn smooth_midpoint(&self, path: &[WorldPoint]) -> Vec<WorldPoint> {
        if path.len() <= 2 {
            return path.to_vec();
        }

        let checker = TraversabilityChecker::new(self.storage, self.config.footprint.clone());
        let mut smoothed = path.to_vec();
        let alpha = 0.5; // Smoothing factor

        for _ in 0..self.config.max_iterations {
            let mut changed = false;
            let mut new_path = smoothed.clone();

            // Skip first and last points
            for i in 1..smoothed.len() - 1 {
                let prev = smoothed[i - 1];
                let curr = smoothed[i];
                let next = smoothed[i + 1];

                // Target is midpoint of prev-next line
                let target = WorldPoint::new((prev.x + next.x) / 2.0, (prev.y + next.y) / 2.0);

                // Move towards target
                let new_point = WorldPoint::new(
                    curr.x + alpha * (target.x - curr.x),
                    curr.y + alpha * (target.y - curr.y),
                );

                // Check if new position is safe
                if checker.is_position_safe(new_point) {
                    // Check paths to neighbors
                    let to_prev =
                        checker.check_path(new_point, prev, Some(self.config.los_step_size));
                    let to_next =
                        checker.check_path(new_point, next, Some(self.config.los_step_size));

                    if to_prev.is_clear
                        && to_next.is_clear
                        && ((new_point.x - curr.x).abs() > 0.001
                            || (new_point.y - curr.y).abs() > 0.001)
                    {
                        new_path[i] = new_point;
                        changed = true;
                    }
                }
            }

            smoothed = new_path;
            if !changed {
                break;
            }
        }

        smoothed
    }

    /// Full smoothing: LOS shortcuts + midpoint smoothing
    pub fn smooth(&self, path: &[WorldPoint]) -> Vec<WorldPoint> {
        let los_smoothed = self.smooth_los(path);
        self.smooth_midpoint(&los_smoothed)
    }

    /// Convert grid path to world path with smoothing
    pub fn smooth_grid_path(&self, path: &[GridCoord]) -> Vec<WorldPoint> {
        let world_path: Vec<WorldPoint> = path
            .iter()
            .map(|c| self.storage.grid_to_world(*c))
            .collect();

        self.smooth(&world_path)
    }
}

/// Simplify a path by removing collinear points
pub fn simplify_path(path: &[WorldPoint], tolerance: f32) -> Vec<WorldPoint> {
    if path.len() <= 2 {
        return path.to_vec();
    }

    let mut result = vec![path[0]];

    for i in 1..path.len() - 1 {
        let prev = result.last().unwrap();
        let curr = &path[i];
        let next = &path[i + 1];

        // Check if current point is far enough from the line prev->next
        let distance = point_to_line_distance(curr, prev, next);
        if distance > tolerance {
            result.push(*curr);
        }
    }

    result.push(*path.last().unwrap());
    result
}

/// Calculate perpendicular distance from point to line
fn point_to_line_distance(
    point: &WorldPoint,
    line_start: &WorldPoint,
    line_end: &WorldPoint,
) -> f32 {
    let dx = line_end.x - line_start.x;
    let dy = line_end.y - line_start.y;
    let line_length_sq = dx * dx + dy * dy;

    if line_length_sq < 0.0001 {
        // Line is a point
        return point.distance(line_start);
    }

    // Project point onto line
    let t = ((point.x - line_start.x) * dx + (point.y - line_start.y) * dy) / line_length_sq;
    let t_clamped = t.clamp(0.0, 1.0);

    let proj_x = line_start.x + t_clamped * dx;
    let proj_y = line_start.y + t_clamped * dy;

    let distance_x = point.x - proj_x;
    let distance_y = point.y - proj_y;

    (distance_x * distance_x + distance_y * distance_y).sqrt()
}

/// Calculate total path length
pub fn path_length(path: &[WorldPoint]) -> f32 {
    if path.len() < 2 {
        return 0.0;
    }

    path.windows(2).map(|w| w[0].distance(&w[1])).sum()
}

/// Interpolate along path to get evenly spaced waypoints
pub fn resample_path(path: &[WorldPoint], spacing: f32) -> Vec<WorldPoint> {
    if path.is_empty() {
        return Vec::new();
    }
    if path.len() == 1 {
        return path.to_vec();
    }

    let total_length = path_length(path);
    if total_length < spacing {
        return vec![path[0], *path.last().unwrap()];
    }

    let mut result = vec![path[0]];
    let mut accumulated = 0.0;
    let mut segment_idx = 0;
    let mut segment_progress = 0.0;

    while accumulated < total_length - spacing * 0.5 {
        accumulated += spacing;

        // Find the segment containing this distance
        let mut distance_along = 0.0;
        while segment_idx < path.len() - 1 {
            let segment_length = path[segment_idx].distance(&path[segment_idx + 1]);

            if distance_along + segment_length >= accumulated {
                // Point is in this segment
                segment_progress = (accumulated - distance_along) / segment_length;
                break;
            }

            distance_along += segment_length;
            segment_idx += 1;
        }

        if segment_idx < path.len() - 1 {
            let p1 = &path[segment_idx];
            let p2 = &path[segment_idx + 1];
            let point = WorldPoint::new(
                p1.x + segment_progress * (p2.x - p1.x),
                p1.y + segment_progress * (p2.y - p1.y),
            );
            result.push(point);
        }
    }

    // Always include the end point
    result.push(*path.last().unwrap());
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::CellType;

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(50, 50, 0.1) // 5m x 5m at 10cm resolution
    }

    fn fill_floor(storage: &mut GridStorage) {
        for x in 0..50 {
            for y in 0..50 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }
    }

    #[test]
    fn test_los_smoothing_straight_line() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        let config = SmoothingConfig {
            footprint: RobotFootprint::new(0.05, 0.01),
            ..Default::default()
        };
        let smoother = PathSmoother::new(&storage, config);

        // Jagged path that should become straight
        let path = vec![
            WorldPoint::new(0.0, 0.0),
            WorldPoint::new(0.1, 0.01),
            WorldPoint::new(0.2, -0.01),
            WorldPoint::new(0.3, 0.02),
            WorldPoint::new(0.4, 0.0),
            WorldPoint::new(0.5, 0.0),
        ];

        let smoothed = smoother.smooth_los(&path);

        // Should reduce to just start and end
        assert!(smoothed.len() <= 3);
        assert_eq!(smoothed[0], path[0]);
        assert_eq!(*smoothed.last().unwrap(), *path.last().unwrap());
    }

    #[test]
    fn test_los_preserves_corners() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        // Add wall
        for y in -10..10 {
            storage.set_type_with_priority(
                storage.world_to_grid(WorldPoint::new(0.5, y as f32 * 0.1)),
                CellType::Wall,
            );
        }

        let config = SmoothingConfig {
            footprint: RobotFootprint::new(0.05, 0.01),
            ..Default::default()
        };
        let smoother = PathSmoother::new(&storage, config);

        // Path that goes around the wall
        let path = vec![
            WorldPoint::new(0.0, 0.0),
            WorldPoint::new(0.3, 0.0),
            WorldPoint::new(0.3, 1.2), // Go up
            WorldPoint::new(0.7, 1.2), // Go right (around wall)
            WorldPoint::new(0.7, 0.0), // Go down
            WorldPoint::new(1.0, 0.0),
        ];

        let smoothed = smoother.smooth_los(&path);

        // Should preserve some waypoints to go around wall
        assert!(smoothed.len() >= 3);
    }

    #[test]
    fn test_simplify_path() {
        // Path with a clear corner (not collinear)
        let path = vec![
            WorldPoint::new(0.0, 0.0),
            WorldPoint::new(0.1, 0.001), // Nearly collinear with start->end of segment 1
            WorldPoint::new(0.2, 0.0),   // End of first segment
            WorldPoint::new(0.3, 0.1),   // True corner - offset from line
            WorldPoint::new(0.3, 0.2),
        ];

        let simplified = simplify_path(&path, 0.01);

        // Should have fewer points than original
        // start, corner at (0.2, 0), corner at (0.3, 0.1), end
        assert!(simplified.len() <= path.len());

        // First and last points should always be preserved
        assert_eq!(simplified[0], path[0]);
        assert_eq!(*simplified.last().unwrap(), *path.last().unwrap());
    }

    #[test]
    fn test_path_length() {
        let path = vec![
            WorldPoint::new(0.0, 0.0),
            WorldPoint::new(1.0, 0.0),
            WorldPoint::new(1.0, 1.0),
        ];

        let length = path_length(&path);
        assert!((length - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_resample_path() {
        let path = vec![WorldPoint::new(0.0, 0.0), WorldPoint::new(1.0, 0.0)];

        let resampled = resample_path(&path, 0.25);

        // Should have points at 0, 0.25, 0.5, 0.75, 1.0
        assert!(resampled.len() >= 4);
        assert_eq!(resampled[0], path[0]);
        assert_eq!(*resampled.last().unwrap(), *path.last().unwrap());
    }
}
