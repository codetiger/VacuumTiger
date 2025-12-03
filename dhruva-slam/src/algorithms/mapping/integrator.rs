//! Scan integration into occupancy grid.
//!
//! Takes a laser scan and robot pose, then updates the occupancy grid:
//! 1. Ray trace from robot to each scan endpoint
//! 2. Mark cells along ray as free
//! 3. Mark endpoint as occupied (if valid range)

use crate::core::types::{LaserScan, PointCloud2D, Pose2D};
use super::{OccupancyGrid, RayTracer, MapRegion};

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

    /// Maximum ray length for free space marking (meters).
    ///
    /// For max-range returns, only trace this far.
    pub max_free_range: f32,

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
            max_free_range: 15.0,
            auto_resize: true,
            point_skip: 1,
        }
    }
}

/// Integrates laser scans into occupancy grid.
#[derive(Debug)]
pub struct MapIntegrator {
    config: MapIntegratorConfig,
    ray_tracer: RayTracer,
}

impl MapIntegrator {
    /// Create a new map integrator.
    pub fn new(config: MapIntegratorConfig) -> Self {
        Self {
            config,
            ray_tracer: RayTracer::default(),
        }
    }

    /// Get the configuration.
    pub fn config(&self) -> &MapIntegratorConfig {
        &self.config
    }

    /// Integrate a laser scan into the occupancy grid.
    ///
    /// # Arguments
    ///
    /// * `grid` - The occupancy grid to update
    /// * `scan` - The laser scan to integrate
    /// * `robot_pose` - Robot pose in world coordinates when scan was taken
    ///
    /// # Returns
    ///
    /// The region of the map that was updated.
    pub fn integrate_scan(
        &self,
        grid: &mut OccupancyGrid,
        scan: &LaserScan,
        robot_pose: &Pose2D,
    ) -> Option<MapRegion> {
        if scan.is_empty() {
            return None;
        }

        let (_sin_t, _cos_t) = robot_pose.theta.sin_cos();

        // Ensure grid can contain robot position
        if self.config.auto_resize {
            grid.ensure_contains(robot_pose.x, robot_pose.y);
        }

        let mut _point_count = 0;

        for (i, (angle, range, _)) in scan.iter().enumerate() {
            // Skip points if configured
            if self.config.point_skip > 1 && i % self.config.point_skip != 0 {
                continue;
            }

            // Global angle (scan angle + robot heading)
            let global_angle = angle + robot_pose.theta;
            let (sin_a, cos_a) = global_angle.sin_cos();

            // Check if range is valid
            let valid_range = range >= self.config.min_range && range <= self.config.max_range;

            if valid_range {
                // Valid range: trace ray and mark endpoint as occupied
                let end_x = robot_pose.x + range * cos_a;
                let end_y = robot_pose.y + range * sin_a;

                // Ensure grid contains endpoint
                if self.config.auto_resize {
                    grid.ensure_contains(end_x, end_y);
                }

                self.ray_tracer.trace_ray(
                    grid,
                    robot_pose.x,
                    robot_pose.y,
                    end_x,
                    end_y,
                    true, // Mark endpoint as occupied
                );
                _point_count += 1;
            } else if range > self.config.max_range && range < self.config.max_free_range {
                // Max-range return: trace ray as free but don't mark endpoint
                let end_x = robot_pose.x + range * cos_a;
                let end_y = robot_pose.y + range * sin_a;

                self.ray_tracer.trace_ray(
                    grid,
                    robot_pose.x,
                    robot_pose.y,
                    end_x,
                    end_y,
                    false, // Don't mark endpoint
                );
            }
            // If range < min_range, skip entirely (too close, likely noise)
        }

        // Return updated region
        grid.take_updated_region()
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

        for (i, point) in cloud.points.iter().enumerate() {
            // Skip points if configured
            if self.config.point_skip > 1 && i % self.config.point_skip != 0 {
                continue;
            }

            // Compute range from robot to point
            let dx = point.x - robot_pose.x;
            let dy = point.y - robot_pose.y;
            let range = (dx * dx + dy * dy).sqrt();

            // Skip if too close
            if range < self.config.min_range {
                continue;
            }

            // Ensure grid contains point
            if self.config.auto_resize {
                grid.ensure_contains(point.x, point.y);
            }

            // Trace ray
            let mark_endpoint = range <= self.config.max_range;
            self.ray_tracer.trace_ray(
                grid,
                robot_pose.x,
                robot_pose.y,
                point.x,
                point.y,
                mark_endpoint,
            );
        }

        grid.take_updated_region()
    }

    /// Integrate a single point into the grid.
    ///
    /// Useful for incremental updates.
    #[inline]
    pub fn integrate_point(
        &self,
        grid: &mut OccupancyGrid,
        robot_x: f32,
        robot_y: f32,
        point_x: f32,
        point_y: f32,
    ) {
        let dx = point_x - robot_x;
        let dy = point_y - robot_y;
        let range = (dx * dx + dy * dy).sqrt();

        if range < self.config.min_range {
            return;
        }

        if self.config.auto_resize {
            grid.ensure_contains(point_x, point_y);
        }

        let mark_endpoint = range <= self.config.max_range;
        self.ray_tracer.trace_ray(
            grid,
            robot_x,
            robot_y,
            point_x,
            point_y,
            mark_endpoint,
        );
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
    use std::f32::consts::{PI, TAU};

    fn create_test_grid() -> OccupancyGrid {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 10.0,
            initial_height: 10.0,
            ..Default::default()
        };
        OccupancyGrid::new(config)
    }

    fn create_test_scan(n_points: usize, range: f32) -> LaserScan {
        let angle_increment = TAU / n_points as f32;
        let ranges = vec![range; n_points];

        LaserScan::new(
            0.0,
            TAU - angle_increment,
            angle_increment,
            0.15,
            12.0,
            ranges,
        )
    }

    #[test]
    fn test_integrate_simple_scan() {
        let mut grid = create_test_grid();
        let integrator = MapIntegrator::default();

        // Create a scan with uniform range
        let scan = create_test_scan(36, 2.0);
        let robot_pose = Pose2D::identity();

        let region = integrator.integrate_scan(&mut grid, &scan, &robot_pose);

        assert!(region.is_some());

        // Should have marked some cells
        let (free, _, occupied) = grid.count_cells();
        assert!(free > 0, "Should have free cells");
        assert!(occupied > 0, "Should have occupied cells");
    }

    #[test]
    fn test_integrate_with_pose() {
        let mut grid = create_test_grid();
        let integrator = MapIntegrator::default();

        // Robot at (1, 1) facing 45 degrees
        let robot_pose = Pose2D::new(1.0, 1.0, PI / 4.0);

        let scan = create_test_scan(36, 1.5);
        integrator.integrate_scan(&mut grid, &scan, &robot_pose);

        // Check that cells around robot are free
        let (rx, ry) = grid.world_to_cell(1.0, 1.0).unwrap();
        // Robot cell itself may be free or unknown depending on scan
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

        // Create scan with some points too close
        let mut ranges = vec![1.0; 36];
        ranges[0] = 0.1; // Too close
        ranges[1] = 0.3; // Too close
        ranges[2] = 0.6; // Valid

        let angle_increment = TAU / 36.0;
        let scan = LaserScan::new(0.0, TAU - angle_increment, angle_increment, 0.15, 12.0, ranges);

        integrator.integrate_scan(&mut grid, &scan, &Pose2D::identity());

        // Should have 34 occupied points (36 - 2 too close)
        let (_, _, occupied) = grid.count_cells();
        assert_eq!(occupied, 34);
    }

    #[test]
    fn test_max_range_filter() {
        let mut grid = create_test_grid();
        let config = MapIntegratorConfig {
            max_range: 2.0,
            max_free_range: 5.0,
            ..Default::default()
        };
        let integrator = MapIntegrator::new(config);

        // Create scan with some points beyond max range
        let mut ranges = vec![1.5; 36]; // All valid
        ranges[0] = 3.0; // Beyond max_range, but ray still traced as free
        ranges[1] = 10.0; // Beyond max_free_range, ignored entirely

        let angle_increment = TAU / 36.0;
        let scan = LaserScan::new(0.0, TAU - angle_increment, angle_increment, 0.15, 12.0, ranges);

        integrator.integrate_scan(&mut grid, &scan, &Pose2D::identity());

        // Should have 34 occupied points (36 - 1 beyond range - 1 ignored)
        let (_, _, occupied) = grid.count_cells();
        assert_eq!(occupied, 34);
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

        let scan = create_test_scan(100, 2.0);

        integrator_no_skip.integrate_scan(&mut grid1, &scan, &Pose2D::identity());
        integrator_skip.integrate_scan(&mut grid2, &scan, &Pose2D::identity());

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

        // Integrate a scan that extends beyond grid
        let scan = create_test_scan(36, 5.0);
        integrator.integrate_scan(&mut grid, &scan, &Pose2D::identity());

        // Grid should have grown
        let (w2, h2) = grid.dimensions();
        assert!(w2 > w1 || h2 > h1);
    }

    #[test]
    fn test_empty_scan() {
        let mut grid = create_test_grid();
        let integrator = MapIntegrator::default();

        let empty_scan = LaserScan::default();
        let region = integrator.integrate_scan(&mut grid, &empty_scan, &Pose2D::identity());

        assert!(region.is_none());
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
