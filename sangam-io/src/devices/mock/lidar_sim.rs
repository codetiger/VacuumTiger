//! Lidar simulator with ray-casting
//!
//! Simulates Delta-2D lidar sensor with configurable noise.

use super::config::LidarConfig;
use super::map_loader::SimulationMap;
use super::noise::NoiseGenerator;
use std::f32::consts::TAU;

/// Lidar simulator
pub struct LidarSimulator {
    config: LidarConfig,
    noise: NoiseGenerator,
}

impl LidarSimulator {
    /// Create new lidar simulator
    pub fn new(config: &LidarConfig, noise: NoiseGenerator) -> Self {
        Self {
            config: config.clone(),
            noise,
        }
    }

    /// Generate a complete 360° scan
    ///
    /// Returns vector of (angle_rad, distance_m, quality) tuples.
    pub fn generate_scan(
        &mut self,
        map: &SimulationMap,
        robot_x: f32,
        robot_y: f32,
        robot_theta: f32,
    ) -> Vec<(f32, f32, u8)> {
        let mut points = Vec::with_capacity(self.config.num_rays);
        let angle_step = TAU / self.config.num_rays as f32;

        // Compute lidar position in world frame
        let lidar_x = robot_x + self.config.mounting_x * robot_theta.cos()
            - self.config.mounting_y * robot_theta.sin();
        let lidar_y = robot_y
            + self.config.mounting_x * robot_theta.sin()
            + self.config.mounting_y * robot_theta.cos();

        for i in 0..self.config.num_rays {
            // Local angle (in lidar frame)
            let local_angle = i as f32 * angle_step;

            // World angle (including robot orientation and lidar offset)
            let world_angle = robot_theta + local_angle + self.config.angle_offset;

            // Adjust for optical offset (radial offset from lidar center)
            let ray_ox = lidar_x + self.config.optical_offset * world_angle.cos();
            let ray_oy = lidar_y + self.config.optical_offset * world_angle.sin();

            // Ray-cast against map
            let mut distance = map.ray_cast(ray_ox, ray_oy, world_angle, self.config.max_range);

            // Apply miss rate (random invalid readings)
            if self.noise.chance(self.config.noise.miss_rate) {
                continue; // Skip this point
            }

            // Apply range noise for valid readings
            if distance < self.config.max_range {
                distance += self
                    .noise
                    .biased_gaussian(self.config.noise.range_bias, self.config.noise.range_stddev);
                distance = distance.clamp(self.config.min_range, self.config.max_range);
            }

            // Apply angle noise
            let noisy_angle = local_angle + self.noise.gaussian(self.config.noise.angle_stddev);

            // Normalize angle to [0, 2π)
            let output_angle = noisy_angle.rem_euclid(TAU);

            // Compute quality based on distance
            let quality = if distance >= self.config.max_range {
                0 // Invalid reading
            } else {
                let base = self.config.noise.quality_base as f32;
                let decay = self.config.noise.quality_distance_decay as f32 * distance;
                (base - decay).clamp(1.0, 255.0) as u8
            };

            // Filter by range
            if distance >= self.config.min_range && distance < self.config.max_range {
                points.push((output_angle, distance, quality));
            }
        }

        points
    }
}
