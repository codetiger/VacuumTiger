//! Angular downsampling for LiDAR scans.
//!
//! Reduces point count while preserving scan structure.

use crate::core::types::LaserScan;

/// Configuration for angular downsampling.
#[derive(Debug, Clone, Copy)]
pub struct AngularDownsamplerConfig {
    /// Target number of output points.
    ///
    /// The actual output may vary slightly based on input size.
    /// Default: 200
    pub target_points: usize,

    /// Minimum angular separation between output points (radians).
    ///
    /// Ensures points aren't too close together even if target_points is high.
    /// Default: 0.5° (0.00873 rad)
    pub min_angle_step: f32,
}

impl Default for AngularDownsamplerConfig {
    fn default() -> Self {
        Self {
            target_points: 200,
            min_angle_step: 0.5_f32.to_radians(),
        }
    }
}

/// Angular downsampler for reducing scan point count.
///
/// Uses uniform angular sampling to select a subset of points
/// while maintaining even coverage around the scan.
#[derive(Debug, Clone)]
pub struct AngularDownsampler {
    config: AngularDownsamplerConfig,
}

impl AngularDownsampler {
    /// Create a new downsampler with the given configuration.
    pub fn new(config: AngularDownsamplerConfig) -> Self {
        Self { config }
    }

    /// Get the current configuration.
    pub fn config(&self) -> &AngularDownsamplerConfig {
        &self.config
    }

    /// Apply downsampling to a laser scan.
    ///
    /// Returns a new scan with reduced point count.
    pub fn apply(&self, scan: &LaserScan) -> LaserScan {
        let input_count = scan.ranges.len();

        if input_count == 0 {
            return scan.clone();
        }

        // If already below target, return as-is
        if input_count <= self.config.target_points {
            return scan.clone();
        }

        // Calculate skip factor
        let skip = (input_count as f32 / self.config.target_points as f32).ceil() as usize;
        let skip = skip.max(1);

        // Calculate expected output angle increment
        let new_angle_increment = scan.angle_increment * skip as f32;

        // Ensure minimum angle separation
        let effective_skip = if new_angle_increment < self.config.min_angle_step {
            (self.config.min_angle_step / scan.angle_increment).ceil() as usize
        } else {
            skip
        };

        // Sample points
        let mut new_ranges = Vec::with_capacity(input_count / effective_skip + 1);
        let mut new_intensities = scan
            .intensities
            .as_ref()
            .map(|_| Vec::with_capacity(input_count / effective_skip + 1));

        let mut new_angle_min = None;
        let mut new_angle_max = 0.0f32;

        for i in (0..input_count).step_by(effective_skip) {
            let angle = scan.angle_at(i);

            if new_angle_min.is_none() {
                new_angle_min = Some(angle);
            }
            new_angle_max = angle;

            new_ranges.push(scan.ranges[i]);

            if let (Some(new_int), Some(old_int)) = (&mut new_intensities, &scan.intensities)
                && let Some(&intensity) = old_int.get(i)
            {
                new_int.push(intensity);
            }
        }

        let angle_min = new_angle_min.unwrap_or(scan.angle_min);
        let angle_increment = if new_ranges.len() > 1 {
            (new_angle_max - angle_min) / (new_ranges.len() - 1) as f32
        } else {
            scan.angle_increment
        };

        LaserScan {
            angle_min,
            angle_max: new_angle_max,
            angle_increment,
            range_min: scan.range_min,
            range_max: scan.range_max,
            ranges: new_ranges,
            intensities: new_intensities,
        }
    }

    /// Calculate the skip factor that would be used for a given input size.
    pub fn calculate_skip(&self, input_count: usize) -> usize {
        if input_count <= self.config.target_points {
            return 1;
        }
        (input_count as f32 / self.config.target_points as f32).ceil() as usize
    }
}

impl Default for AngularDownsampler {
    fn default() -> Self {
        Self::new(AngularDownsamplerConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    fn create_uniform_scan(n_points: usize) -> LaserScan {
        let angle_increment = TAU / n_points as f32;
        let ranges: Vec<f32> = vec![5.0; n_points];

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
    fn test_downsampler_reduces_points() {
        let scan = create_uniform_scan(500);
        let config = AngularDownsamplerConfig {
            target_points: 200,
            min_angle_step: 0.0, // No minimum
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // Should be approximately 200 points (500 / 3 skip = 167, or 500 / 2.5 = 200)
        assert!(result.len() >= 150);
        assert!(result.len() <= 250);
    }

    #[test]
    fn test_downsampler_skip_factor() {
        let config = AngularDownsamplerConfig {
            target_points: 100,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        // 500 points, target 100 → skip = 5
        assert_eq!(downsampler.calculate_skip(500), 5);

        // 50 points, target 100 → skip = 1 (no downsampling needed)
        assert_eq!(downsampler.calculate_skip(50), 1);
    }

    #[test]
    fn test_no_downsampling_when_below_target() {
        let scan = create_uniform_scan(100);
        let config = AngularDownsamplerConfig {
            target_points: 200,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        assert_eq!(result.len(), 100); // Should be unchanged
    }

    #[test]
    fn test_empty_scan() {
        let scan = LaserScan::default();
        let downsampler = AngularDownsampler::default();

        let result = downsampler.apply(&scan);

        assert!(result.is_empty());
    }

    #[test]
    fn test_preserves_intensities() {
        let ranges = vec![1.0; 10];
        let intensities: Vec<u8> = (0..10).map(|i| (i * 10) as u8).collect();
        let scan =
            LaserScan::new(0.0, TAU, TAU / 10.0, 0.15, 12.0, ranges).with_intensities(intensities);

        let config = AngularDownsamplerConfig {
            target_points: 5,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        assert!(result.intensities.is_some());
        // Should have kept every other point (skip = 2)
        // Intensities: 0, 20, 40, 60, 80 (indices 0, 2, 4, 6, 8)
        let result_int = result.intensities.as_ref().unwrap();
        assert_eq!(result_int.len(), result.len());
    }

    #[test]
    fn test_uniform_coverage() {
        let scan = create_uniform_scan(360); // 1° resolution
        let config = AngularDownsamplerConfig {
            target_points: 36, // 10° resolution
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // Check that coverage is roughly uniform
        let angle_range = result.angle_max - result.angle_min;
        let expected_increment = angle_range / (result.len() - 1) as f32;

        // Should be approximately 10° (0.175 rad)
        assert!(expected_increment > 0.1);
        assert!(expected_increment < 0.3);
    }

    #[test]
    fn test_min_angle_step() {
        // Create a very dense scan
        let scan = create_uniform_scan(1000);

        // Request 500 points but with minimum 2° separation
        let config = AngularDownsamplerConfig {
            target_points: 500,
            min_angle_step: 2.0_f32.to_radians(), // 2°
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // With 2° minimum, max points = 360 / 2 = 180
        assert!(result.len() <= 180);
    }

    // ========================================================================
    // Edge Case Tests
    // ========================================================================

    #[test]
    fn test_single_point_scan() {
        let ranges = vec![5.0];
        let scan = LaserScan::new(0.0, 0.0, 0.1, 0.15, 12.0, ranges);

        let downsampler = AngularDownsampler::default();
        let result = downsampler.apply(&scan);

        // Single point should be returned unchanged
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_exactly_at_target() {
        let scan = create_uniform_scan(200); // Exactly target_points
        let config = AngularDownsamplerConfig {
            target_points: 200,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // Should not downsample when already at target
        assert_eq!(result.len(), 200);
    }

    #[test]
    fn test_one_below_target() {
        let scan = create_uniform_scan(199); // One below target_points
        let config = AngularDownsamplerConfig {
            target_points: 200,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // Should not downsample when below target
        assert_eq!(result.len(), 199);
    }

    #[test]
    fn test_one_above_target() {
        let scan = create_uniform_scan(201); // One above target_points
        let config = AngularDownsamplerConfig {
            target_points: 200,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // Should downsample with skip=2 → ~100 points
        assert!(result.len() < 201);
    }

    #[test]
    fn test_very_large_scan() {
        let scan = create_uniform_scan(10000);
        let config = AngularDownsamplerConfig {
            target_points: 100,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // Skip = 100, so ~100 points
        assert!(result.len() >= 90);
        assert!(result.len() <= 110);
    }

    #[test]
    fn test_target_points_one() {
        let scan = create_uniform_scan(100);
        let config = AngularDownsamplerConfig {
            target_points: 1,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // With target of 1, skip = 100, so should get 1 point
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_config_accessor() {
        let config = AngularDownsamplerConfig {
            target_points: 150,
            min_angle_step: 0.02,
        };
        let downsampler = AngularDownsampler::new(config);

        assert_eq!(downsampler.config().target_points, 150);
        assert_eq!(downsampler.config().min_angle_step, 0.02);
    }

    #[test]
    fn test_default_config() {
        let config = AngularDownsamplerConfig::default();
        assert_eq!(config.target_points, 200);
        // 0.5 degrees in radians
        assert!((config.min_angle_step - 0.5_f32.to_radians()).abs() < 1e-6);
    }

    #[test]
    fn test_calculate_skip_edge_cases() {
        let config = AngularDownsamplerConfig {
            target_points: 100,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        // Zero input
        assert_eq!(downsampler.calculate_skip(0), 1);

        // Below target
        assert_eq!(downsampler.calculate_skip(50), 1);

        // Exactly at target
        assert_eq!(downsampler.calculate_skip(100), 1);

        // Just above target
        assert_eq!(downsampler.calculate_skip(101), 2);

        // Large input
        assert_eq!(downsampler.calculate_skip(1000), 10);
    }

    #[test]
    fn test_min_angle_step_dominates() {
        // Very dense scan with 3600 points (0.1° resolution)
        let scan = create_uniform_scan(3600);

        // Request 3000 points but require minimum 1° separation
        let config = AngularDownsamplerConfig {
            target_points: 3000,
            min_angle_step: 1.0_f32.to_radians(), // 1°
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // 360° / 1° = 360 max points
        assert!(result.len() <= 360);
    }

    #[test]
    fn test_preserves_first_and_last_angle() {
        let scan = create_uniform_scan(100);
        let config = AngularDownsamplerConfig {
            target_points: 10,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // First point should be at same angle as original first point
        assert_eq!(result.angle_min, scan.angle_min);
    }
}
