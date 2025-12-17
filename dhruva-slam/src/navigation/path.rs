//! Path types for navigation.
//!
//! A [`Path`] is the output of the A* planner - a sequence of waypoints
//! from the robot's current position to a target.
//!
//! Note: Some utility methods are defined for future use.

use serde::{Deserialize, Serialize};
use std::time::Instant;

/// A waypoint along a planned path.
///
/// Waypoints are in world coordinates (meters).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Waypoint {
    /// X coordinate in meters.
    pub x: f32,
    /// Y coordinate in meters.
    pub y: f32,
}

impl Waypoint {
    /// Create a new waypoint.
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Calculate distance to another waypoint.
    #[inline]
    pub fn distance_to(&self, other: &Waypoint) -> f32 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Calculate squared distance to another waypoint (avoids sqrt).
    #[inline]
    pub fn distance_squared_to(&self, other: &Waypoint) -> f32 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dx * dx + dy * dy
    }

    /// Calculate heading angle from this waypoint to another.
    ///
    /// Returns angle in radians in the range [-π, π].
    #[inline]
    pub fn heading_to(&self, other: &Waypoint) -> f32 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dy.atan2(dx)
    }
}

impl From<(f32, f32)> for Waypoint {
    fn from((x, y): (f32, f32)) -> Self {
        Self::new(x, y)
    }
}

impl From<crate::core::types::Point2D> for Waypoint {
    fn from(point: crate::core::types::Point2D) -> Self {
        Self::new(point.x, point.y)
    }
}

/// A planned path from robot to target.
///
/// The path consists of a sequence of waypoints that the robot should
/// follow. The first waypoint is typically near the robot's current
/// position, and the last waypoint is at or near the target.
#[derive(Debug, Clone)]
pub struct Path {
    /// Waypoints from start to goal (in world coordinates).
    ///
    /// - `waypoints[0]` is typically near the robot's current position
    /// - `waypoints[len-1]` is at or near the target
    pub waypoints: Vec<Waypoint>,

    /// Total path length in meters.
    pub total_length: f32,

    /// ID of the target this path leads to.
    pub target_id: u32,

    /// Timestamp when this path was planned.
    ///
    /// Note: This is not serialized since Instant is not Serialize.
    pub planned_at: Option<Instant>,
}

impl Path {
    /// Create a new path from waypoints.
    ///
    /// Automatically calculates total length.
    pub fn new(waypoints: Vec<Waypoint>, target_id: u32) -> Self {
        let total_length = Self::calculate_length(&waypoints);
        Self {
            waypoints,
            total_length,
            target_id,
            planned_at: Some(Instant::now()),
        }
    }

    /// Create an empty path (no waypoints).
    pub fn empty(target_id: u32) -> Self {
        Self {
            waypoints: Vec::new(),
            total_length: 0.0,
            target_id,
            planned_at: Some(Instant::now()),
        }
    }

    /// Check if the path is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    /// Get the number of waypoints.
    #[inline]
    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    /// Get a waypoint by index.
    #[inline]
    pub fn get(&self, index: usize) -> Option<&Waypoint> {
        self.waypoints.get(index)
    }

    /// Get the first waypoint.
    #[inline]
    pub fn first(&self) -> Option<&Waypoint> {
        self.waypoints.first()
    }

    /// Get the last waypoint (goal).
    #[inline]
    pub fn last(&self) -> Option<&Waypoint> {
        self.waypoints.last()
    }

    /// Calculate total path length from waypoints.
    fn calculate_length(waypoints: &[Waypoint]) -> f32 {
        if waypoints.len() < 2 {
            return 0.0;
        }

        waypoints.windows(2).map(|w| w[0].distance_to(&w[1])).sum()
    }

    /// Get remaining path length from a waypoint index.
    ///
    /// Returns the sum of distances from `from_index` to the end.
    pub fn remaining_length(&self, from_index: usize) -> f32 {
        if from_index >= self.waypoints.len() {
            return 0.0;
        }

        Self::calculate_length(&self.waypoints[from_index..])
    }

    /// Prune waypoints that have been passed.
    ///
    /// Removes waypoints from index 0 to `up_to_index - 1` (exclusive).
    /// Recalculates total length after pruning.
    pub fn prune_up_to(&mut self, up_to_index: usize) {
        if up_to_index > 0 && up_to_index < self.waypoints.len() {
            self.waypoints = self.waypoints.split_off(up_to_index);
            self.total_length = Self::calculate_length(&self.waypoints);
        } else if up_to_index >= self.waypoints.len() {
            self.waypoints.clear();
            self.total_length = 0.0;
        }
    }

    /// Find the closest waypoint to a given position.
    ///
    /// Returns the index of the closest waypoint and the distance to it.
    pub fn find_closest_waypoint(&self, x: f32, y: f32) -> Option<(usize, f32)> {
        if self.waypoints.is_empty() {
            return None;
        }

        let mut min_dist_sq = f32::MAX;
        let mut min_idx = 0;

        for (i, wp) in self.waypoints.iter().enumerate() {
            let dx = wp.x - x;
            let dy = wp.y - y;
            let dist_sq = dx * dx + dy * dy;

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                min_idx = i;
            }
        }

        Some((min_idx, min_dist_sq.sqrt()))
    }

    /// Simplify path using Douglas-Peucker algorithm.
    ///
    /// Removes waypoints that are within `tolerance` meters of the
    /// line between their neighbors.
    pub fn simplify(&mut self, tolerance: f32) {
        if self.waypoints.len() < 3 {
            return;
        }

        let simplified = douglas_peucker(&self.waypoints, tolerance);
        self.waypoints = simplified;
        self.total_length = Self::calculate_length(&self.waypoints);
    }
}

/// Douglas-Peucker path simplification algorithm.
///
/// Recursively simplifies a path by removing points that are within
/// `tolerance` of the line between the start and end points.
fn douglas_peucker(points: &[Waypoint], tolerance: f32) -> Vec<Waypoint> {
    if points.len() < 3 {
        return points.to_vec();
    }

    // Find the point with maximum distance from the line
    let start = &points[0];
    let end = &points[points.len() - 1];

    let mut max_dist = 0.0f32;
    let mut max_idx = 0;

    for (i, point) in points.iter().enumerate().skip(1).take(points.len() - 2) {
        let dist = perpendicular_distance(point, start, end);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }

    // If max distance is greater than tolerance, recursively simplify
    if max_dist > tolerance {
        let mut result1 = douglas_peucker(&points[..=max_idx], tolerance);
        let result2 = douglas_peucker(&points[max_idx..], tolerance);

        // Remove duplicate point at the join
        result1.pop();
        result1.extend(result2);
        result1
    } else {
        // All points are within tolerance, keep only start and end
        vec![*start, *end]
    }
}

/// Calculate perpendicular distance from a point to a line segment.
fn perpendicular_distance(point: &Waypoint, line_start: &Waypoint, line_end: &Waypoint) -> f32 {
    let dx = line_end.x - line_start.x;
    let dy = line_end.y - line_start.y;

    let line_len_sq = dx * dx + dy * dy;

    if line_len_sq < 1e-10 {
        // Line segment is essentially a point
        return point.distance_to(line_start);
    }

    // Calculate perpendicular distance using cross product
    let numerator = ((line_end.y - line_start.y) * point.x - (line_end.x - line_start.x) * point.y
        + line_end.x * line_start.y
        - line_end.y * line_start.x)
        .abs();

    numerator / line_len_sq.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_waypoint_distance() {
        let a = Waypoint::new(0.0, 0.0);
        let b = Waypoint::new(3.0, 4.0);

        assert!((a.distance_to(&b) - 5.0).abs() < 1e-6);
        assert!((a.distance_squared_to(&b) - 25.0).abs() < 1e-6);
    }

    #[test]
    fn test_waypoint_heading() {
        let origin = Waypoint::new(0.0, 0.0);

        // Heading to point on positive X axis
        let east = Waypoint::new(1.0, 0.0);
        assert!((origin.heading_to(&east) - 0.0).abs() < 1e-6);

        // Heading to point on positive Y axis
        let north = Waypoint::new(0.0, 1.0);
        assert!((origin.heading_to(&north) - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn test_path_creation() {
        let waypoints = vec![
            Waypoint::new(0.0, 0.0),
            Waypoint::new(1.0, 0.0),
            Waypoint::new(1.0, 1.0),
        ];

        let path = Path::new(waypoints, 42);

        assert_eq!(path.len(), 3);
        assert!((path.total_length - 2.0).abs() < 1e-6);
        assert_eq!(path.target_id, 42);
    }

    #[test]
    fn test_path_remaining_length() {
        let waypoints = vec![
            Waypoint::new(0.0, 0.0),
            Waypoint::new(1.0, 0.0),
            Waypoint::new(2.0, 0.0),
            Waypoint::new(3.0, 0.0),
        ];

        let path = Path::new(waypoints, 1);

        assert!((path.total_length - 3.0).abs() < 1e-6);
        assert!((path.remaining_length(0) - 3.0).abs() < 1e-6);
        assert!((path.remaining_length(1) - 2.0).abs() < 1e-6);
        assert!((path.remaining_length(2) - 1.0).abs() < 1e-6);
        assert!((path.remaining_length(3) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_path_prune() {
        let waypoints = vec![
            Waypoint::new(0.0, 0.0),
            Waypoint::new(1.0, 0.0),
            Waypoint::new(2.0, 0.0),
            Waypoint::new(3.0, 0.0),
        ];

        let mut path = Path::new(waypoints, 1);
        path.prune_up_to(2);

        assert_eq!(path.len(), 2);
        assert!((path.waypoints[0].x - 2.0).abs() < 1e-6);
        assert!((path.total_length - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_find_closest_waypoint() {
        let waypoints = vec![
            Waypoint::new(0.0, 0.0),
            Waypoint::new(5.0, 0.0),
            Waypoint::new(10.0, 0.0),
        ];

        let path = Path::new(waypoints, 1);

        let (idx, dist) = path.find_closest_waypoint(4.0, 0.0).unwrap();
        assert_eq!(idx, 1);
        assert!((dist - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_douglas_peucker_simple() {
        // Three collinear points - middle should be removed
        let points = vec![
            Waypoint::new(0.0, 0.0),
            Waypoint::new(1.0, 0.0),
            Waypoint::new(2.0, 0.0),
        ];

        let simplified = douglas_peucker(&points, 0.1);
        assert_eq!(simplified.len(), 2);
    }

    #[test]
    fn test_douglas_peucker_keep_point() {
        // Middle point is far from line, should be kept
        let points = vec![
            Waypoint::new(0.0, 0.0),
            Waypoint::new(1.0, 1.0),
            Waypoint::new(2.0, 0.0),
        ];

        let simplified = douglas_peucker(&points, 0.1);
        assert_eq!(simplified.len(), 3);
    }

    #[test]
    fn test_path_simplify() {
        // L-shaped path with many intermediate points
        let mut waypoints = Vec::new();
        for i in 0..10 {
            waypoints.push(Waypoint::new(i as f32 * 0.1, 0.0));
        }
        for i in 0..10 {
            waypoints.push(Waypoint::new(0.9, i as f32 * 0.1));
        }

        let mut path = Path::new(waypoints, 1);
        let original_len = path.len();

        path.simplify(0.05);

        // Should have significantly fewer points
        assert!(path.len() < original_len);
        // Should have at least 3 points (start, corner, end)
        assert!(path.len() >= 3);
    }

    #[test]
    fn test_empty_path() {
        let path = Path::empty(99);

        assert!(path.is_empty());
        assert_eq!(path.len(), 0);
        assert!((path.total_length - 0.0).abs() < 1e-6);
        assert!(path.first().is_none());
        assert!(path.last().is_none());
    }
}
