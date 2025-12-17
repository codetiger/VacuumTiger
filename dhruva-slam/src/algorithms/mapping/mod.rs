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
//!
//! Note: Some types are defined for planned features.

mod integrator;
mod occupancy_grid;
mod ray_tracer;

pub use integrator::{MapIntegrator, MapIntegratorConfig};
pub use occupancy_grid::{CellState, OccupancyGrid, OccupancyGridConfig};
pub use ray_tracer::RayTracer;

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
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    /// Expand region to include a point.
    pub fn expand_to_include(&mut self, x: i32, y: i32) {
        self.min_x = self.min_x.min(x);
        self.min_y = self.min_y.min(y);
        self.max_x = self.max_x.max(x);
        self.max_y = self.max_y.max(y);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_map_region_expand() {
        let mut region = MapRegion::new(0, 0, 10, 10);

        region.expand_to_include(-5, 15);

        assert_eq!(region.min_x, -5);
        assert_eq!(region.max_y, 15);
    }
}
