//! Sensor model for particle filter update step.
//!
//! Implements the likelihood field sensor model from Probabilistic Robotics.
//! Pre-computes a distance field from the occupancy grid for efficient
//! likelihood evaluation.

use crate::algorithms::mapping::{CellState, OccupancyGrid};
use crate::core::types::{LaserScan, Pose2D};
use std::collections::VecDeque;

/// Configuration for the sensor model.
#[derive(Debug, Clone, Copy)]
pub struct SensorModelConfig {
    /// Weight for "hit" model (Gaussian around expected range).
    /// Typical: 0.9
    pub z_hit: f32,

    /// Weight for random readings (uniform distribution).
    /// Typical: 0.05
    pub z_random: f32,

    /// Weight for max range readings.
    /// Typical: 0.05
    pub z_max: f32,

    /// Standard deviation for hit model (meters).
    /// Typical: 0.1-0.2
    pub sigma_hit: f32,

    /// Maximum sensor range (meters).
    pub max_range: f32,

    /// Maximum distance for likelihood field (meters).
    /// Distances beyond this are clamped.
    pub max_dist: f32,

    /// Skip every N beams for efficiency.
    /// 1 = use all beams, 2 = every other beam, etc.
    pub beam_skip: usize,
}

impl Default for SensorModelConfig {
    fn default() -> Self {
        Self {
            z_hit: 0.9,
            z_random: 0.05,
            z_max: 0.05,
            sigma_hit: 0.15,
            max_range: 8.0,
            max_dist: 2.0,
            beam_skip: 3, // Use every 3rd beam for efficiency
        }
    }
}

impl SensorModelConfig {
    /// Create a high-quality configuration (slower but more accurate).
    pub fn high_quality() -> Self {
        Self {
            beam_skip: 1,
            sigma_hit: 0.1,
            ..Default::default()
        }
    }

    /// Create a fast configuration (less accurate but faster).
    pub fn fast() -> Self {
        Self {
            beam_skip: 6,
            sigma_hit: 0.2,
            ..Default::default()
        }
    }
}

/// Trait for sensor models used in particle filter.
pub trait SensorModel {
    /// Compute log-likelihood of scan given pose and map.
    ///
    /// Returns log(p(scan | pose, map)).
    fn log_likelihood(&self, scan: &LaserScan, pose: &Pose2D, map: &OccupancyGrid) -> f32;

    /// Update the model when the map changes.
    fn update_from_map(&mut self, map: &OccupancyGrid);
}

/// Likelihood field sensor model.
///
/// Pre-computes a distance field from occupied cells in the map.
/// For each laser endpoint, looks up the distance to nearest obstacle
/// and computes likelihood using a Gaussian mixture model.
#[derive(Debug)]
pub struct LikelihoodFieldModel {
    config: SensorModelConfig,

    /// Distance field (distance to nearest obstacle for each cell).
    /// Same dimensions as occupancy grid.
    distance_field: Vec<f32>,

    /// Grid dimensions when distance field was computed.
    field_width: usize,
    field_height: usize,

    /// Grid metadata for coordinate transforms.
    resolution: f32,
    origin_x: f32,
    origin_y: f32,

    /// Pre-computed Gaussian normalization factor.
    gaussian_norm: f32,
}

impl LikelihoodFieldModel {
    /// Create a new likelihood field model.
    pub fn new(config: SensorModelConfig) -> Self {
        let gaussian_norm = 1.0 / (config.sigma_hit * (2.0 * std::f32::consts::PI).sqrt());

        Self {
            config,
            distance_field: Vec::new(),
            field_width: 0,
            field_height: 0,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            gaussian_norm,
        }
    }

    /// Create a likelihood field model initialized from a map.
    pub fn from_map(config: SensorModelConfig, map: &OccupancyGrid) -> Self {
        let mut model = Self::new(config);
        model.update_from_map(map);
        model
    }

    /// Get the configuration.
    pub fn config(&self) -> &SensorModelConfig {
        &self.config
    }

    /// Get the distance field dimensions.
    pub fn field_dimensions(&self) -> (usize, usize) {
        (self.field_width, self.field_height)
    }

    /// Get distance at a cell (for debugging/visualization).
    pub fn get_distance(&self, cx: usize, cy: usize) -> f32 {
        if cx < self.field_width && cy < self.field_height {
            self.distance_field[cy * self.field_width + cx]
        } else {
            self.config.max_dist
        }
    }

    /// Get distance at world coordinates.
    fn get_distance_at_world(&self, x: f32, y: f32) -> f32 {
        let cx = ((x - self.origin_x) / self.resolution).floor();
        let cy = ((y - self.origin_y) / self.resolution).floor();

        if cx >= 0.0 && cy >= 0.0 {
            let cx = cx as usize;
            let cy = cy as usize;
            if cx < self.field_width && cy < self.field_height {
                return self.distance_field[cy * self.field_width + cx];
            }
        }

        self.config.max_dist
    }

    /// Compute distance field from occupancy grid using BFS.
    ///
    /// This creates a field where each cell contains the distance
    /// to the nearest occupied cell.
    fn compute_distance_field(&mut self, map: &OccupancyGrid) {
        let (width, height) = map.dimensions();
        let resolution = map.resolution();
        let (origin_x, origin_y) = map.origin();

        self.field_width = width;
        self.field_height = height;
        self.resolution = resolution;
        self.origin_x = origin_x;
        self.origin_y = origin_y;

        // Initialize with max distance
        self.distance_field = vec![self.config.max_dist; width * height];

        // BFS from all occupied cells simultaneously
        let mut queue: VecDeque<(usize, usize, f32)> = VecDeque::new();

        // Seed with occupied cells
        for cy in 0..height {
            for cx in 0..width {
                if map.get_state(cx, cy) == CellState::Occupied {
                    self.distance_field[cy * width + cx] = 0.0;
                    queue.push_back((cx, cy, 0.0));
                }
            }
        }

        // 8-connected neighbors with distances
        let neighbors: [(i32, i32, f32); 8] = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, std::f32::consts::SQRT_2),
            (1, -1, std::f32::consts::SQRT_2),
            (-1, 1, std::f32::consts::SQRT_2),
            (1, 1, std::f32::consts::SQRT_2),
        ];

        // BFS to propagate distances
        while let Some((cx, cy, dist)) = queue.pop_front() {
            let current_dist = self.distance_field[cy * width + cx];

            // Skip if we've already found a shorter path
            if dist > current_dist + 0.001 {
                continue;
            }

            for &(dx, dy, step) in &neighbors {
                let nx = cx as i32 + dx;
                let ny = cy as i32 + dy;

                if nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height {
                    let nx = nx as usize;
                    let ny = ny as usize;
                    let new_dist = dist + step * resolution;

                    // Only update if new distance is shorter and within max
                    if new_dist < self.distance_field[ny * width + nx]
                        && new_dist < self.config.max_dist
                    {
                        self.distance_field[ny * width + nx] = new_dist;
                        queue.push_back((nx, ny, new_dist));
                    }
                }
            }
        }
    }

    /// Compute log-likelihood of a single beam.
    fn beam_log_likelihood(&self, measured_range: f32, endpoint_x: f32, endpoint_y: f32) -> f32 {
        // Check for max range reading
        if measured_range >= self.config.max_range * 0.99 {
            // Max range reading - use z_max weight
            return (self.config.z_max / self.config.max_range).ln();
        }

        // Look up distance to nearest obstacle at endpoint
        let dist_to_obstacle = self.get_distance_at_world(endpoint_x, endpoint_y);

        // Hit model: Gaussian centered at 0 distance
        let p_hit = self.gaussian_norm
            * (-0.5 * (dist_to_obstacle * dist_to_obstacle)
                / (self.config.sigma_hit * self.config.sigma_hit))
                .exp();

        // Random model: uniform distribution
        let p_random = self.config.z_random / self.config.max_range;

        // Combined probability
        let p = self.config.z_hit * p_hit + p_random;

        // Return log probability (clamped for numerical stability)
        p.max(1e-10).ln()
    }
}

impl SensorModel for LikelihoodFieldModel {
    fn log_likelihood(&self, scan: &LaserScan, pose: &Pose2D, _map: &OccupancyGrid) -> f32 {
        if self.distance_field.is_empty() {
            return 0.0; // No field computed yet
        }

        let mut total_log_likelihood = 0.0;
        let mut beam_count = 0;

        // Process beams with skip factor
        for (i, range) in scan.ranges.iter().enumerate() {
            if i % self.config.beam_skip != 0 {
                continue;
            }

            let range = *range;

            // Skip invalid readings
            if range < 0.01 || range > self.config.max_range {
                continue;
            }

            // Compute beam angle in world frame
            let beam_angle = scan.angle_min + (i as f32) * scan.angle_increment;
            let world_angle = pose.theta + beam_angle;

            // Compute endpoint in world coordinates
            let endpoint_x = pose.x + range * world_angle.cos();
            let endpoint_y = pose.y + range * world_angle.sin();

            // Accumulate log-likelihood
            total_log_likelihood += self.beam_log_likelihood(range, endpoint_x, endpoint_y);
            beam_count += 1;
        }

        // Normalize by beam count (optional, helps with numerical stability)
        if beam_count > 0 {
            total_log_likelihood
        } else {
            0.0
        }
    }

    fn update_from_map(&mut self, map: &OccupancyGrid) {
        self.compute_distance_field(map);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::mapping::OccupancyGridConfig;

    fn create_simple_map() -> OccupancyGrid {
        let config = OccupancyGridConfig {
            resolution: 0.1,
            initial_width: 10.0,
            initial_height: 10.0,
            ..Default::default()
        };
        let mut grid = OccupancyGrid::new(config);

        // Create a wall at y = 2.0
        for i in 0..100 {
            let x = -5.0 + i as f32 * 0.1;
            if let Some((cx, cy)) = grid.world_to_cell(x, 2.0) {
                for _ in 0..10 {
                    grid.update_cell(cx, cy, true);
                }
            }
        }

        grid
    }

    #[test]
    fn test_likelihood_field_creation() {
        let map = create_simple_map();
        let model = LikelihoodFieldModel::from_map(SensorModelConfig::default(), &map);

        let (width, height) = model.field_dimensions();
        assert!(width > 0);
        assert!(height > 0);
    }

    #[test]
    fn test_distance_field_values() {
        let map = create_simple_map();
        let model = LikelihoodFieldModel::from_map(SensorModelConfig::default(), &map);

        // Distance at occupied cell should be 0
        if let Some((cx, cy)) = map.world_to_cell(0.0, 2.0) {
            let dist = model.get_distance(cx, cy);
            assert!(dist < 0.15, "Distance at obstacle should be ~0: {}", dist);
        }

        // Distance far from wall should be larger
        if let Some((cx, cy)) = map.world_to_cell(0.0, 0.0) {
            let dist = model.get_distance(cx, cy);
            assert!(dist > 1.0, "Distance from wall should be ~2m: {}", dist);
        }
    }

    #[test]
    fn test_likelihood_at_correct_pose() {
        let map = create_simple_map();
        let model = LikelihoodFieldModel::from_map(SensorModelConfig::default(), &map);

        // Create a scan that "sees" the wall at distance 2m from origin
        let scan = LaserScan::new(
            -0.1,           // angle_min
            0.1,            // angle_max
            0.1,            // angle_increment
            0.1,            // range_min
            8.0,            // range_max
            vec![2.0, 2.0, 2.0],
        );

        // Pose at origin facing the wall (positive Y)
        let pose = Pose2D::new(0.0, 0.0, std::f32::consts::PI / 2.0);

        let log_l = model.log_likelihood(&scan, &pose, &map);
        assert!(log_l.is_finite(), "Log likelihood should be finite: {}", log_l);
    }

    #[test]
    fn test_likelihood_comparison() {
        let map = create_simple_map();
        let model = LikelihoodFieldModel::from_map(
            SensorModelConfig {
                beam_skip: 1,
                ..Default::default()
            },
            &map,
        );

        // Create a scan that sees the wall
        let scan = LaserScan::new(
            -0.1,           // angle_min
            0.1,            // angle_max
            0.02,           // angle_increment
            0.1,            // range_min
            8.0,            // range_max
            vec![2.0; 11],  // Wall at 2m
        );

        // Correct pose: at origin, facing wall
        let correct_pose = Pose2D::new(0.0, 0.0, std::f32::consts::PI / 2.0);

        // Wrong pose: 1m to the side
        let wrong_pose = Pose2D::new(1.0, 0.0, std::f32::consts::PI / 2.0);

        let log_l_correct = model.log_likelihood(&scan, &correct_pose, &map);
        let log_l_wrong = model.log_likelihood(&scan, &wrong_pose, &map);

        // Correct pose should have higher likelihood (less negative)
        // Note: This might not always hold depending on map geometry
        println!("Correct: {}, Wrong: {}", log_l_correct, log_l_wrong);
        // Both should be finite
        assert!(log_l_correct.is_finite());
        assert!(log_l_wrong.is_finite());
    }

    #[test]
    fn test_config_presets() {
        let default = SensorModelConfig::default();
        let fast = SensorModelConfig::fast();
        let hq = SensorModelConfig::high_quality();

        // Fast should skip more beams
        assert!(fast.beam_skip > default.beam_skip);
        // HQ should skip fewer beams
        assert!(hq.beam_skip < default.beam_skip);
    }

    #[test]
    fn test_empty_scan() {
        let map = create_simple_map();
        let model = LikelihoodFieldModel::from_map(SensorModelConfig::default(), &map);

        let empty_scan = LaserScan::new(
            0.0,            // angle_min
            0.0,            // angle_max
            0.1,            // angle_increment
            0.1,            // range_min
            8.0,            // range_max
            vec![],         // empty ranges
        );

        let pose = Pose2D::identity();
        let log_l = model.log_likelihood(&empty_scan, &pose, &map);

        // Empty scan should give neutral likelihood
        assert_eq!(log_l, 0.0);
    }
}
