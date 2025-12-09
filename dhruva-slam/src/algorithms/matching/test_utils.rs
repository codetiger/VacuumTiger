//! Shared test utilities for scan matching algorithms.
//!
//! Contains helper functions for creating test point clouds used across
//! multiple matcher test suites.

use crate::core::types::{Point2D, PointCloud2D};

/// Create an L-shaped point cloud (two perpendicular walls).
///
/// Adds slight noise to avoid kiddo bucket size issues with collinear points.
///
/// # Arguments
///
/// * `n` - Number of points per arm (total points = 2n - 1)
/// * `length` - Length of each arm in meters
///
/// # Example
///
/// ```ignore
/// let cloud = create_l_shape(50, 2.0);
/// assert!(cloud.len() >= 99);
/// ```
pub fn create_l_shape(n: usize, length: f32) -> PointCloud2D {
    let mut cloud = PointCloud2D::with_capacity(2 * n);
    // Horizontal wall with tiny y variation
    for i in 0..n {
        let x = (i as f32 / (n - 1) as f32) * length;
        let y_noise = (i as f32) * 0.0001; // Tiny variation to avoid collinearity
        cloud.push(Point2D::new(x, y_noise));
    }
    // Vertical wall with tiny x variation
    for i in 1..n {
        let y = (i as f32 / (n - 1) as f32) * length;
        let x_noise = (i as f32) * 0.0001; // Tiny variation
        cloud.push(Point2D::new(x_noise, y));
    }
    cloud
}

/// Create a room-shaped point cloud (four walls).
///
/// Creates a rectangular room with walls on all sides.
///
/// # Arguments
///
/// * `n` - Total points distributed across all walls
/// * `width` - Room width in meters
/// * `height` - Room height in meters
///
/// # Example
///
/// ```ignore
/// let room = create_room(100, 4.0, 3.0);
/// assert_eq!(room.len(), 100);
/// ```
pub fn create_room(n: usize, width: f32, height: f32) -> PointCloud2D {
    let mut cloud = PointCloud2D::new();
    let points_per_wall = n / 4;

    // Bottom wall
    for i in 0..points_per_wall {
        let x = (i as f32 / points_per_wall as f32) * width;
        let noise = (i as f32) * 0.0001;
        cloud.push(Point2D::new(x, noise));
    }
    // Right wall
    for i in 0..points_per_wall {
        let y = (i as f32 / points_per_wall as f32) * height;
        let noise = (i as f32) * 0.0001;
        cloud.push(Point2D::new(width + noise, y));
    }
    // Top wall
    for i in 0..points_per_wall {
        let x = width - (i as f32 / points_per_wall as f32) * width;
        let noise = (i as f32) * 0.0001;
        cloud.push(Point2D::new(x, height + noise));
    }
    // Left wall
    for i in 0..points_per_wall {
        let y = height - (i as f32 / points_per_wall as f32) * height;
        let noise = (i as f32) * 0.0001;
        cloud.push(Point2D::new(noise, y));
    }
    cloud
}

/// Create a straight line point cloud.
///
/// # Arguments
///
/// * `n` - Number of points
/// * `length` - Length of line in meters
///
/// # Example
///
/// ```ignore
/// let line = create_line(10, 1.0);
/// assert_eq!(line.len(), 10);
/// ```
pub fn create_line(n: usize, length: f32) -> PointCloud2D {
    let mut cloud = PointCloud2D::with_capacity(n);
    for i in 0..n {
        let x = (i as f32 / (n - 1) as f32) * length;
        cloud.push(Point2D::new(x, 0.0));
    }
    cloud
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_l_shape() {
        let cloud = create_l_shape(50, 2.0);
        assert_eq!(cloud.len(), 99); // 50 + 49 (first point shared)
    }

    #[test]
    fn test_create_room() {
        let cloud = create_room(100, 4.0, 3.0);
        assert_eq!(cloud.len(), 100);
    }

    #[test]
    fn test_create_line() {
        let cloud = create_line(10, 1.0);
        assert_eq!(cloud.len(), 10);
    }
}
