//! Scan integration into occupancy grid.
//!
//! Takes a laser scan and robot pose, then updates the occupancy grid:
//! 1. Ray trace from robot to each scan endpoint
//! 2. Mark cells along ray as free
//! 3. Mark endpoint as occupied (if valid range)
//!
//! Note: Some utility methods are defined for future use.

use super::{MapRegion, OccupancyGrid, RayTracer};
use crate::core::types::{PointCloud2D, Pose2D};

/// Configuration for map integrator.
#[derive(Debug, Clone)]
pub struct MapIntegratorConfig {
    /// Minimum valid range for marking occupied (meters).
    ///
    /// Points closer than this are ignored (may be sensor noise).
    pub min_range: f32,

    /// Maximum valid range for marking occupied (meters).
    ///
    /// Points farther than this are not marked as occupied,
    /// but the ray is still traced as free.
    pub max_range: f32,

    /// Whether to grow the map to accommodate new scan points.
    pub auto_resize: bool,

    /// Skip every N points for faster integration.
    ///
    /// 1 = use all points, 2 = use every other point, etc.
    pub point_skip: usize,
}

impl Default for MapIntegratorConfig {
    fn default() -> Self {
        Self {
            min_range: 0.15,
            max_range: 12.0,
            auto_resize: true,
            point_skip: 1,
        }
    }
}

/// Integrates laser scans into occupancy grid.
#[derive(Debug)]
pub struct MapIntegrator {
    ray_tracer: RayTracer,
    config: MapIntegratorConfig,
}

impl MapIntegrator {
    /// Create a new map integrator.
    pub fn new(config: MapIntegratorConfig) -> Self {
        Self {
            ray_tracer: RayTracer::default(),
            config,
        }
    }

    /// Integrate a point cloud into the occupancy grid.
    ///
    /// The point cloud should already be in global coordinates.
    ///
    /// # Arguments
    ///
    /// * `grid` - The occupancy grid to update
    /// * `cloud` - The point cloud (in global coordinates)
    /// * `robot_pose` - Robot pose when scan was taken (for ray tracing)
    pub fn integrate_cloud(
        &self,
        grid: &mut OccupancyGrid,
        cloud: &PointCloud2D,
        robot_pose: &Pose2D,
    ) -> Option<MapRegion> {
        if cloud.is_empty() {
            return None;
        }

        // Ensure grid can contain robot position
        if self.config.auto_resize {
            grid.ensure_contains(robot_pose.x, robot_pose.y);
        }

        // Pre-compute squared thresholds to avoid sqrt in hot loop
        let min_range_sq = self.config.min_range * self.config.min_range;
        let max_range_sq = self.config.max_range * self.config.max_range;
        let robot_x = robot_pose.x;
        let robot_y = robot_pose.y;

        for i in 0..cloud.len() {
            // Skip points if configured
            if self.config.point_skip > 1 && i % self.config.point_skip != 0 {
                continue;
            }

            let point_x = cloud.xs[i];
            let point_y = cloud.ys[i];

            // Compute squared range from robot to point (avoid sqrt)
            let dx = point_x - robot_x;
            let dy = point_y - robot_y;
            let range_sq = dx * dx + dy * dy;

            // Skip if too close (using squared comparison)
            if range_sq < min_range_sq {
                continue;
            }

            // Ensure grid contains point
            if self.config.auto_resize {
                grid.ensure_contains(point_x, point_y);
            }

            // Trace ray (using squared comparison for max_range)
            let mark_endpoint = range_sq <= max_range_sq;
            self.ray_tracer
                .trace_ray(grid, robot_x, robot_y, point_x, point_y, mark_endpoint);
        }

        grid.take_updated_region()
    }
}

impl Default for MapIntegrator {
    fn default() -> Self {
        Self::new(MapIntegratorConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::mapping::OccupancyGridConfig;
    use crate::core::types::Point2D;
    use std::f32::consts::TAU;

    fn create_test_grid() -> OccupancyGrid {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 10.0,
            initial_height: 10.0,
            ..Default::default()
        };
        OccupancyGrid::new(config)
    }

    #[test]
    fn test_integrate_point_cloud() {
        let mut grid = create_test_grid();
        let integrator = MapIntegrator::default();

        // Create a simple point cloud
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 0.0));
        cloud.push(Point2D::new(0.0, 1.0));
        cloud.push(Point2D::new(-1.0, 0.0));
        cloud.push(Point2D::new(0.0, -1.0));

        let robot_pose = Pose2D::identity();
        let region = integrator.integrate_cloud(&mut grid, &cloud, &robot_pose);

        assert!(region.is_some());

        // Check that points are marked
        let (free, _, occupied) = grid.count_cells();
        assert!(free > 0);
        assert_eq!(occupied, 4);
    }

    #[test]
    fn test_min_range_filter() {
        let mut grid = create_test_grid();
        let config = MapIntegratorConfig {
            min_range: 0.5,
            ..Default::default()
        };
        let integrator = MapIntegrator::new(config);

        // Create point cloud with some points too close
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(0.1, 0.0)); // Too close (range 0.1)
        cloud.push(Point2D::new(0.0, 0.3)); // Too close (range 0.3)
        cloud.push(Point2D::new(0.6, 0.0)); // Valid (range 0.6)
        cloud.push(Point2D::new(0.0, 1.0)); // Valid (range 1.0)

        integrator.integrate_cloud(&mut grid, &cloud, &Pose2D::identity());

        // Should have 2 occupied points (4 - 2 too close)
        let (_, _, occupied) = grid.count_cells();
        assert_eq!(occupied, 2);
    }

    #[test]
    fn test_max_range_filter() {
        let mut grid = create_test_grid();
        let config = MapIntegratorConfig {
            max_range: 2.0,
            ..Default::default()
        };
        let integrator = MapIntegrator::new(config);

        // Create point cloud with some points beyond max range
        // Use opposite directions to avoid ray overlap issues
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.5, 0.0)); // Valid (range 1.5, +X direction)
        cloud.push(Point2D::new(-1.5, 0.0)); // Valid (range 1.5, -X direction)
        cloud.push(Point2D::new(3.0, 3.0)); // Beyond max_range (range ~4.2)

        integrator.integrate_cloud(&mut grid, &cloud, &Pose2D::identity());

        // Should have 2 occupied points (3 - 1 beyond max_range)
        let (_, _, occupied) = grid.count_cells();
        assert_eq!(occupied, 2);
    }

    #[test]
    fn test_point_skip() {
        let mut grid1 = create_test_grid();
        let mut grid2 = create_test_grid();

        let config_no_skip = MapIntegratorConfig {
            point_skip: 1,
            ..Default::default()
        };
        let config_skip = MapIntegratorConfig {
            point_skip: 2,
            ..Default::default()
        };

        let integrator_no_skip = MapIntegrator::new(config_no_skip);
        let integrator_skip = MapIntegrator::new(config_skip);

        // Create point cloud with 100 points in a circle
        let mut cloud = PointCloud2D::new();
        for i in 0..100 {
            let angle = (i as f32) * TAU / 100.0;
            cloud.push(Point2D::new(2.0 * angle.cos(), 2.0 * angle.sin()));
        }

        integrator_no_skip.integrate_cloud(&mut grid1, &cloud, &Pose2D::identity());
        integrator_skip.integrate_cloud(&mut grid2, &cloud, &Pose2D::identity());

        let (_, _, occupied1) = grid1.count_cells();
        let (_, _, occupied2) = grid2.count_cells();

        // Skipping should result in roughly half the occupied cells
        assert!(occupied2 < occupied1);
        assert!(occupied2 >= occupied1 / 3); // Some tolerance
    }

    #[test]
    fn test_auto_resize() {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 2.0,
            initial_height: 2.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        let integrator_config = MapIntegratorConfig {
            auto_resize: true,
            ..Default::default()
        };
        let integrator = MapIntegrator::new(integrator_config);

        // Original grid is small
        let (w1, h1) = grid.dimensions();

        // Integrate a point cloud that extends beyond grid
        let mut cloud = PointCloud2D::new();
        for i in 0..36 {
            let angle = (i as f32) * TAU / 36.0;
            cloud.push(Point2D::new(5.0 * angle.cos(), 5.0 * angle.sin()));
        }
        integrator.integrate_cloud(&mut grid, &cloud, &Pose2D::identity());

        // Grid should have grown
        let (w2, h2) = grid.dimensions();
        assert!(w2 > w1 || h2 > h1);
    }

    #[test]
    fn test_empty_cloud() {
        let mut grid = create_test_grid();
        let integrator = MapIntegrator::default();

        let empty_cloud = PointCloud2D::new();
        let region = integrator.integrate_cloud(&mut grid, &empty_cloud, &Pose2D::identity());

        assert!(region.is_none());
    }
}
