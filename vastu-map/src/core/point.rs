//! Point and coordinate types for the occupancy grid.

use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Sub};

/// Grid coordinates (integer cell indices)
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub struct GridCoord {
    /// X coordinate (column index)
    pub x: i32,
    /// Y coordinate (row index)
    pub y: i32,
}

impl GridCoord {
    /// Create a new grid coordinate
    #[inline]
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// Manhattan distance to another coordinate
    #[inline]
    pub fn manhattan_distance(&self, other: &GridCoord) -> i32 {
        (self.x - other.x).abs() + (self.y - other.y).abs()
    }

    /// Chebyshev distance (max of x and y distance) - used for 8-connected grids
    #[inline]
    pub fn chebyshev_distance(&self, other: &GridCoord) -> i32 {
        (self.x - other.x).abs().max((self.y - other.y).abs())
    }

    /// Get the 4 cardinal neighbors (N, E, S, W)
    #[inline]
    pub fn neighbors_4(&self) -> [GridCoord; 4] {
        [
            GridCoord::new(self.x, self.y + 1), // North
            GridCoord::new(self.x + 1, self.y), // East
            GridCoord::new(self.x, self.y - 1), // South
            GridCoord::new(self.x - 1, self.y), // West
        ]
    }

    /// Get the 8 neighbors (including diagonals)
    #[inline]
    pub fn neighbors_8(&self) -> [GridCoord; 8] {
        [
            GridCoord::new(self.x, self.y + 1),     // N
            GridCoord::new(self.x + 1, self.y + 1), // NE
            GridCoord::new(self.x + 1, self.y),     // E
            GridCoord::new(self.x + 1, self.y - 1), // SE
            GridCoord::new(self.x, self.y - 1),     // S
            GridCoord::new(self.x - 1, self.y - 1), // SW
            GridCoord::new(self.x - 1, self.y),     // W
            GridCoord::new(self.x - 1, self.y + 1), // NW
        ]
    }
}

impl Add for GridCoord {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        GridCoord::new(self.x + other.x, self.y + other.y)
    }
}

impl Sub for GridCoord {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        GridCoord::new(self.x - other.x, self.y - other.y)
    }
}

/// World coordinates (meters, f32)
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize)]
pub struct WorldPoint {
    /// X coordinate in meters (forward in ROS convention)
    pub x: f32,
    /// Y coordinate in meters (left in ROS convention)
    pub y: f32,
}

impl WorldPoint {
    /// Create a new world point
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Zero point (origin)
    pub const ZERO: WorldPoint = WorldPoint { x: 0.0, y: 0.0 };

    /// Euclidean distance to another point
    #[inline]
    pub fn distance(&self, other: &WorldPoint) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Squared distance (faster, avoids sqrt)
    #[inline]
    pub fn distance_squared(&self, other: &WorldPoint) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    /// Angle from this point to another (radians, CCW from +X)
    #[inline]
    pub fn angle_to(&self, other: &WorldPoint) -> f32 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dy.atan2(dx)
    }

    /// Create a point at a given angle and distance from this point
    #[inline]
    pub fn point_at(&self, angle: f32, distance: f32) -> WorldPoint {
        WorldPoint::new(
            self.x + distance * angle.cos(),
            self.y + distance * angle.sin(),
        )
    }

    /// Rotate this point around the origin by angle (radians)
    #[inline]
    pub fn rotate(&self, angle: f32) -> WorldPoint {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        WorldPoint::new(
            self.x * cos_a - self.y * sin_a,
            self.x * sin_a + self.y * cos_a,
        )
    }

    /// Length (magnitude) of this point as a vector from origin
    #[inline]
    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Normalize to unit length
    #[inline]
    pub fn normalize(&self) -> WorldPoint {
        let len = self.length();
        if len > 0.0 {
            WorldPoint::new(self.x / len, self.y / len)
        } else {
            *self
        }
    }

    /// Dot product with another point (as vectors)
    #[inline]
    pub fn dot(&self, other: &WorldPoint) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// Cross product (z-component of 3D cross product)
    #[inline]
    pub fn cross(&self, other: &WorldPoint) -> f32 {
        self.x * other.y - self.y * other.x
    }
}

impl Add for WorldPoint {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        WorldPoint::new(self.x + other.x, self.y + other.y)
    }
}

impl Sub for WorldPoint {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        WorldPoint::new(self.x - other.x, self.y - other.y)
    }
}

impl Mul<f32> for WorldPoint {
    type Output = Self;

    #[inline]
    fn mul(self, scalar: f32) -> Self {
        WorldPoint::new(self.x * scalar, self.y * scalar)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_coord_neighbors() {
        let c = GridCoord::new(5, 5);
        let n4 = c.neighbors_4();
        assert_eq!(n4[0], GridCoord::new(5, 6)); // N
        assert_eq!(n4[1], GridCoord::new(6, 5)); // E
        assert_eq!(n4[2], GridCoord::new(5, 4)); // S
        assert_eq!(n4[3], GridCoord::new(4, 5)); // W
    }

    #[test]
    fn test_world_point_distance() {
        let a = WorldPoint::new(0.0, 0.0);
        let b = WorldPoint::new(3.0, 4.0);
        assert!((a.distance(&b) - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_world_point_angle() {
        let origin = WorldPoint::ZERO;
        let east = WorldPoint::new(1.0, 0.0);
        let north = WorldPoint::new(0.0, 1.0);

        assert!((origin.angle_to(&east) - 0.0).abs() < 1e-6);
        assert!((origin.angle_to(&north) - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn test_world_point_rotate() {
        let p = WorldPoint::new(1.0, 0.0);
        let rotated = p.rotate(std::f32::consts::FRAC_PI_2); // 90 degrees
        assert!((rotated.x - 0.0).abs() < 1e-6);
        assert!((rotated.y - 1.0).abs() < 1e-6);
    }
}
