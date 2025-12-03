//! Mapping module (Phase 5).
//!
//! Provides occupancy grid mapping from laser scans.
//!
//! # Components
//!
//! - [`OccupancyGrid`]: 2D grid map with log-odds probabilities
//! - [`RayTracer`]: Bresenham ray tracing for free space
//! - [`MapIntegrator`]: Integrates scans into map
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::mapping::{OccupancyGrid, OccupancyGridConfig, MapIntegrator};
//! use dhruva_slam::types::{PointCloud2D, Pose2D};
//!
//! let config = OccupancyGridConfig::default();
//! let mut map = OccupancyGrid::new(config);
//!
//! let integrator = MapIntegrator::default();
//! integrator.integrate(&mut map, &scan, &robot_pose);
//!
//! map.save("map.bin")?;
//! ```

mod occupancy_grid;
mod ray_tracer;
mod integrator;

pub use occupancy_grid::{OccupancyGrid, OccupancyGridConfig, CellState};
pub use ray_tracer::RayTracer;
pub use integrator::{MapIntegrator, MapIntegratorConfig};

use serde::{Deserialize, Serialize};

/// Map metadata for serialization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapMetadata {
    /// Map resolution in meters per cell.
    pub resolution: f32,
    /// Map width in cells.
    pub width: usize,
    /// Map height in cells.
    pub height: usize,
    /// World coordinates of the cell (0, 0).
    pub origin_x: f32,
    pub origin_y: f32,
}

/// Bounding box for map region.
#[derive(Debug, Clone, Copy)]
pub struct MapRegion {
    pub min_x: i32,
    pub min_y: i32,
    pub max_x: i32,
    pub max_y: i32,
}

impl MapRegion {
    /// Create a new region.
    pub fn new(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Self {
        Self { min_x, min_y, max_x, max_y }
    }

    /// Check if a cell is within this region.
    pub fn contains(&self, x: i32, y: i32) -> bool {
        x >= self.min_x && x <= self.max_x && y >= self.min_y && y <= self.max_y
    }

    /// Expand region to include a point.
    pub fn expand_to_include(&mut self, x: i32, y: i32) {
        self.min_x = self.min_x.min(x);
        self.min_y = self.min_y.min(y);
        self.max_x = self.max_x.max(x);
        self.max_y = self.max_y.max(y);
    }

    /// Merge with another region.
    pub fn merge(&self, other: &MapRegion) -> MapRegion {
        MapRegion {
            min_x: self.min_x.min(other.min_x),
            min_y: self.min_y.min(other.min_y),
            max_x: self.max_x.max(other.max_x),
            max_y: self.max_y.max(other.max_y),
        }
    }

    /// Width of the region in cells.
    pub fn width(&self) -> usize {
        (self.max_x - self.min_x + 1) as usize
    }

    /// Height of the region in cells.
    pub fn height(&self) -> usize {
        (self.max_y - self.min_y + 1) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_map_region_contains() {
        let region = MapRegion::new(0, 0, 10, 10);

        assert!(region.contains(0, 0));
        assert!(region.contains(5, 5));
        assert!(region.contains(10, 10));
        assert!(!region.contains(-1, 0));
        assert!(!region.contains(11, 5));
    }

    #[test]
    fn test_map_region_expand() {
        let mut region = MapRegion::new(0, 0, 10, 10);

        region.expand_to_include(-5, 15);

        assert_eq!(region.min_x, -5);
        assert_eq!(region.max_y, 15);
    }

    #[test]
    fn test_map_region_merge() {
        let r1 = MapRegion::new(0, 0, 10, 10);
        let r2 = MapRegion::new(-5, 5, 15, 20);

        let merged = r1.merge(&r2);

        assert_eq!(merged.min_x, -5);
        assert_eq!(merged.min_y, 0);
        assert_eq!(merged.max_x, 15);
        assert_eq!(merged.max_y, 20);
    }
}
