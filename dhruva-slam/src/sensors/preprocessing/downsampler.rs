//! Angular downsampling for LiDAR scans.
//!
//! Reduces point count while preserving scan structure using angular bins
//! for deterministic sampling across consecutive scans.

use crate::core::types::LaserScan;
use std::f32::consts::TAU;

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
/// Uses **angular-binned sampling** to ensure deterministic point selection
/// across consecutive scans. This is critical for ICP accuracy because:
///
/// 1. Traditional index-based downsampling (`step_by(skip)`) causes aliasing
///    when consecutive scans have different point counts (e.g., after outlier
///    removal). The same wall might be sampled at different angles.
///
/// 2. Angular bins divide the 360° space into fixed regions. Each bin selects
///    the point closest to its center, ensuring consistent angular sampling
///    regardless of which points were removed by upstream filters.
///
/// This approach reduces ICP matching error significantly for static scans.
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

    /// Apply angular-binned downsampling to a laser scan.
    ///
    /// Divides the angular range into fixed bins and selects the point
    /// closest to each bin center. This ensures deterministic sampling
    /// across consecutive scans regardless of upstream filtering.
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

        // Calculate bin width based on target points
        // Use 360° (TAU) as the full angular range for consistent binning
        let bin_width = (TAU / self.config.target_points as f32).max(self.config.min_angle_step);
        let num_bins = (TAU / bin_width).ceil() as usize;

        // Pre-allocate bin storage: (best_index, best_distance_to_center)
        // Using Option to track empty bins
        let mut bins: Vec<Option<(usize, f32)>> = vec![None; num_bins];

        // Assign each point to its angular bin
        for i in 0..input_count {
            let angle = scan
                .angles
                .as_ref()
                .and_then(|a| a.get(i).copied())
                .unwrap_or_else(|| scan.angle_at(i));

            // Normalize angle to [0, TAU) for consistent binning
            let normalized_angle = angle.rem_euclid(TAU);
            let bin_idx = ((normalized_angle / bin_width) as usize).min(num_bins - 1);

            // Calculate bin center
            let bin_center = (bin_idx as f32 + 0.5) * bin_width;

            // Distance from point angle to bin center
            let dist = (normalized_angle - bin_center).abs();

            // Keep the point closest to bin center
            match &mut bins[bin_idx] {
                None => bins[bin_idx] = Some((i, dist)),
                Some((_, best_dist)) if dist < *best_dist => {
                    bins[bin_idx] = Some((i, dist));
                }
                _ => {}
            }
        }

        // Collect selected points in angular order
        let mut new_ranges = Vec::with_capacity(num_bins);
        let mut new_angles = Vec::with_capacity(num_bins);
        let mut new_intensities = scan
            .intensities
            .as_ref()
            .map(|_| Vec::with_capacity(num_bins));

        let mut new_angle_min = None;
        let mut new_angle_max = 0.0f32;

        for (idx, _) in bins.iter().flatten() {
            let angle = scan
                .angles
                .as_ref()
                .and_then(|a| a.get(*idx).copied())
                .unwrap_or_else(|| scan.angle_at(*idx));

            if new_angle_min.is_none() {
                new_angle_min = Some(angle);
            }
            new_angle_max = angle;

            new_ranges.push(scan.ranges[*idx]);
            new_angles.push(angle);

            if let (Some(new_int), Some(old_int)) = (&mut new_intensities, &scan.intensities)
                && let Some(&intensity) = old_int.get(*idx)
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
            // Preserve actual angles from the downsampled points
            angles: if scan.angles.is_some() {
                Some(new_angles)
            } else {
                None
            },
        }
    }

    /// Calculate the approximate number of output points for a given input.
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
    fn test_angular_bins_cover_full_range() {
        let scan = create_uniform_scan(100);
        let config = AngularDownsamplerConfig {
            target_points: 10,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result = downsampler.apply(&scan);

        // With angular binning, output spans most of the input range
        // First bin selects point closest to bin center (not necessarily first point)
        // Verify we get roughly uniform coverage
        let output_range = result.angle_max - result.angle_min;
        let input_range = scan.angle_max - scan.angle_min;

        // Output should cover significant portion of input range
        assert!(output_range > input_range * 0.5);
    }

    #[test]
    fn test_consistent_sampling_across_scans() {
        // This test verifies the key property: consecutive scans with
        // slightly different point counts produce consistent angular samples

        // Scan 1: 100 points
        let scan1 = create_uniform_scan(100);

        // Scan 2: 98 points (simulating some points removed by outlier filter)
        let ranges2: Vec<f32> = (0..98).map(|_| 5.0).collect();
        let scan2 = LaserScan::new(
            0.0,
            TAU * 98.0 / 100.0, // Slightly shorter range
            TAU / 100.0,        // Same increment
            0.15,
            12.0,
            ranges2,
        );

        let config = AngularDownsamplerConfig {
            target_points: 10,
            min_angle_step: 0.0,
        };
        let downsampler = AngularDownsampler::new(config);

        let result1 = downsampler.apply(&scan1);
        let result2 = downsampler.apply(&scan2);

        // Both should produce similar number of points
        assert!((result1.len() as i32 - result2.len() as i32).abs() <= 2);

        // First output angles should be very close (within half a bin width)
        let bin_width = TAU / 10.0;
        let angle_diff = (result1.angle_min - result2.angle_min).abs();
        assert!(
            angle_diff < bin_width,
            "Angle difference {} should be < bin_width {}",
            angle_diff,
            bin_width
        );
    }
}
