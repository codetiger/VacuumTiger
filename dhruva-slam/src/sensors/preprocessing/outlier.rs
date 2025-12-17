//! Outlier removal for LiDAR scans.
//!
//! Removes isolated noise points using statistical analysis of neighbors.

use crate::core::types::LaserScan;

/// Configuration for outlier filtering.
#[derive(Debug, Clone, Copy)]
pub struct OutlierFilterConfig {
    /// Number of neighboring points to consider on each side.
    ///
    /// Total neighbors checked = 2 * neighbor_count
    /// Default: 3
    pub neighbor_count: usize,

    /// Maximum allowed deviation from neighbor median (meters).
    ///
    /// Points deviating more than this are considered outliers.
    /// Default: 0.3m
    pub distance_threshold: f32,
}

impl Default for OutlierFilterConfig {
    fn default() -> Self {
        Self {
            neighbor_count: 3,
            distance_threshold: 0.3,
        }
    }
}

/// Outlier filter for removing noise spikes from LiDAR scans.
///
/// Uses a sliding window to compare each point's range with its neighbors.
/// Points that deviate significantly from the local median are removed.
///
/// This is effective for removing:
/// - Random noise spikes
/// - Reflections from dust particles
/// - Multi-path errors
#[derive(Debug, Clone)]
pub struct OutlierFilter {
    config: OutlierFilterConfig,
}

impl OutlierFilter {
    /// Create a new outlier filter with the given configuration.
    pub fn new(config: OutlierFilterConfig) -> Self {
        Self { config }
    }

    /// Check if a point is an outlier based on its neighbors.
    ///
    /// Returns true if the point should be removed.
    fn is_outlier(&self, ranges: &[f32], index: usize) -> bool {
        let n = ranges.len();
        if n < 3 {
            return false; // Not enough points to determine outliers
        }

        let range = ranges[index];
        if !range.is_finite() {
            return true; // Invalid range is always an outlier
        }

        // Collect valid neighbor ranges using stack array (no heap allocation)
        const MAX_NEIGHBORS: usize = 32;
        let mut neighbors: [f32; MAX_NEIGHBORS] = [0.0; MAX_NEIGHBORS];
        let mut neighbor_count = 0;

        // Look at neighbors on both sides
        let start = index.saturating_sub(self.config.neighbor_count);
        let end = (index + self.config.neighbor_count + 1).min(n);

        for (i, &r) in ranges.iter().enumerate().take(end).skip(start) {
            if i != index && r.is_finite() && neighbor_count < MAX_NEIGHBORS {
                neighbors[neighbor_count] = r;
                neighbor_count += 1;
            }
        }

        if neighbor_count == 0 {
            return false; // No valid neighbors to compare
        }

        // Calculate median of neighbors (sort in-place on stack array slice)
        let neighbor_slice = &mut neighbors[..neighbor_count];
        neighbor_slice.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = if neighbor_count % 2 == 0 {
            (neighbor_slice[neighbor_count / 2 - 1] + neighbor_slice[neighbor_count / 2]) / 2.0
        } else {
            neighbor_slice[neighbor_count / 2]
        };

        // Check if point deviates too much from median
        (range - median).abs() > self.config.distance_threshold
    }

    /// Apply outlier filtering to a laser scan.
    ///
    /// Returns a new scan with outlier points removed.
    pub fn apply(&self, scan: &LaserScan) -> LaserScan {
        if scan.ranges.len() < 3 {
            return scan.clone();
        }

        let mut new_ranges = Vec::with_capacity(scan.ranges.len());
        let mut new_angles = Vec::with_capacity(scan.ranges.len());
        let mut new_intensities = scan
            .intensities
            .as_ref()
            .map(|_| Vec::with_capacity(scan.ranges.len()));

        let mut new_angle_min = None;
        let mut new_angle_max = 0.0f32;

        for i in 0..scan.ranges.len() {
            if !self.is_outlier(&scan.ranges, i) {
                let angle = scan.get_angle(i);

                if new_angle_min.is_none() {
                    new_angle_min = Some(angle);
                }
                new_angle_max = angle;

                new_ranges.push(scan.ranges[i]);
                new_angles.push(angle);

                if let (Some(new_int), Some(old_int)) = (&mut new_intensities, &scan.intensities)
                    && let Some(&intensity) = old_int.get(i)
                {
                    new_int.push(intensity);
                }
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
            // Preserve actual angles from the filtered points
            angles: if scan.angles.is_some() {
                Some(new_angles)
            } else {
                None
            },
        }
    }
}

impl Default for OutlierFilter {
    fn default() -> Self {
        Self::new(OutlierFilterConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    fn create_smooth_scan(n_points: usize, base_range: f32) -> LaserScan {
        let angle_increment = TAU / n_points as f32;
        let ranges = vec![base_range; n_points];

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
    fn test_smooth_scan_unchanged() {
        let scan = create_smooth_scan(100, 5.0);
        let filter = OutlierFilter::default();

        let result = filter.apply(&scan);

        // Smooth scan should have no outliers
        assert_eq!(result.len(), scan.len());
    }

    #[test]
    fn test_removes_spike() {
        let mut scan = create_smooth_scan(100, 5.0);

        // Add a large spike in the middle
        scan.ranges[50] = 10.0; // 5m deviation from 5m base

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        // Should have removed the spike
        assert_eq!(result.len(), 99);
    }

    #[test]
    fn test_removes_multiple_spikes() {
        let mut scan = create_smooth_scan(100, 5.0);

        // Add several isolated spikes
        scan.ranges[20] = 0.5; // Much closer
        scan.ranges[50] = 10.0; // Much farther
        scan.ranges[80] = 8.0; // Farther

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        // Should have removed all spikes
        assert_eq!(result.len(), 97);
    }

    #[test]
    fn test_preserves_gradual_changes() {
        let angle_increment = TAU / 100.0;
        let ranges: Vec<f32> = (0..100)
            .map(|i| 5.0 + 0.05 * i as f32) // Gradual increase
            .collect();

        let scan = LaserScan::new(
            0.0,
            TAU - angle_increment,
            angle_increment,
            0.15,
            12.0,
            ranges,
        );

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        // Gradual changes should not be removed
        assert_eq!(result.len(), 100);
    }

    #[test]
    fn test_custom_threshold() {
        let mut scan = create_smooth_scan(100, 5.0);

        // Add a small deviation
        scan.ranges[50] = 5.2; // 0.2m deviation

        // Default threshold (0.3m) should keep it
        let filter_default = OutlierFilter::default();
        let result_default = filter_default.apply(&scan);
        assert_eq!(result_default.len(), 100);

        // Stricter threshold should remove it
        let config = OutlierFilterConfig {
            neighbor_count: 3,
            distance_threshold: 0.1,
        };
        let filter_strict = OutlierFilter::new(config);
        let result_strict = filter_strict.apply(&scan);
        assert_eq!(result_strict.len(), 99);
    }

    #[test]
    fn test_empty_scan() {
        let scan = LaserScan::default();
        let filter = OutlierFilter::default();

        let result = filter.apply(&scan);

        assert!(result.is_empty());
    }

    #[test]
    fn test_very_short_scan() {
        let ranges = vec![5.0, 10.0]; // Only 2 points
        let scan = LaserScan::new(0.0, 1.0, 0.5, 0.15, 12.0, ranges);

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        // Too few points to determine outliers
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_handles_nan() {
        let mut scan = create_smooth_scan(100, 5.0);
        scan.ranges[50] = f32::NAN;

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        // NaN should be removed
        assert_eq!(result.len(), 99);
    }

    #[test]
    fn test_preserves_intensities() {
        let mut scan = create_smooth_scan(10, 5.0);
        scan.ranges[5] = 10.0; // Add outlier
        scan.intensities = Some(vec![10, 20, 30, 40, 50, 60, 70, 80, 90, 100]);

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        assert_eq!(result.len(), 9);
        assert!(result.intensities.is_some());

        let intensities = result.intensities.as_ref().unwrap();
        assert_eq!(intensities.len(), 9);
        // Intensity 60 (at index 5) should be removed
        assert!(!intensities.contains(&60));
    }

    // ========================================================================
    // Edge Case Tests
    // ========================================================================

    #[test]
    fn test_all_points_are_outliers() {
        // Alternating pattern where every point differs greatly from neighbors
        let ranges = vec![1.0, 10.0, 1.0, 10.0, 1.0, 10.0, 1.0, 10.0, 1.0, 10.0];
        let scan = LaserScan::new(0.0, TAU, TAU / 10.0, 0.15, 12.0, ranges);

        let config = OutlierFilterConfig {
            neighbor_count: 1,       // Look at 1 neighbor each side
            distance_threshold: 0.5, // 0.5m threshold
        };
        let filter = OutlierFilter::new(config);
        let result = filter.apply(&scan);

        // In alternating pattern, every point is an outlier relative to neighbors
        // But edges might be preserved due to limited neighbors
        assert!(result.len() < 10);
    }

    #[test]
    fn test_single_point_scan() {
        let ranges = vec![5.0];
        let scan = LaserScan::new(0.0, 0.0, 0.1, 0.15, 12.0, ranges);

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        // Single point scan should be returned unchanged
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_three_points_minimum() {
        // With only 3 points, outlier detection is challenging because:
        // - Edge points only have 1 neighbor (the middle point)
        // - Middle point has 2 neighbors (the edge points)
        //
        // Let's verify behavior with a moderate spike:
        // ranges = [5.0, 5.5, 5.0]
        // - Index 0: neighbor=[5.5], median=5.5, |5-5.5|=0.5 > 0.3 → outlier
        // - Index 1: neighbors=[5.0, 5.0], median=5.0, |5.5-5|=0.5 > 0.3 → outlier
        // - Index 2: neighbor=[5.5], median=5.5, |5-5.5|=0.5 > 0.3 → outlier
        //
        // Even small deviations can make all 3 points outliers with small thresholds.
        // This test verifies the filter handles minimum length scans gracefully.
        let ranges = vec![5.0, 5.1, 5.0]; // Very small deviation
        let scan = LaserScan::new(0.0, 1.0, 0.5, 0.15, 12.0, ranges);

        let config = OutlierFilterConfig {
            neighbor_count: 1,
            distance_threshold: 0.2, // Threshold larger than deviation
        };
        let filter = OutlierFilter::new(config);
        let result = filter.apply(&scan);

        // With 0.2m threshold:
        // - Index 0: neighbor=[5.1], |5-5.1|=0.1 < 0.2 → NOT outlier
        // - Index 1: neighbors=[5.0, 5.0], median=5.0, |5.1-5|=0.1 < 0.2 → NOT outlier
        // - Index 2: neighbor=[5.1], |5-5.1|=0.1 < 0.2 → NOT outlier
        assert_eq!(
            result.len(),
            3,
            "Expected 3 points preserved, got {}",
            result.len()
        );
    }

    #[test]
    fn test_all_nan_neighbors() {
        // Point surrounded by NaN values
        let ranges = vec![f32::NAN, f32::NAN, 5.0, f32::NAN, f32::NAN];
        let scan = LaserScan::new(0.0, TAU, TAU / 5.0, 0.15, 12.0, ranges);

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        // Only the valid point at index 2 should remain
        // It can't be determined as outlier since no valid neighbors
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_outlier_at_start() {
        let mut scan = create_smooth_scan(20, 5.0);
        scan.ranges[0] = 15.0; // First point is outlier

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        assert_eq!(result.len(), 19);
    }

    #[test]
    fn test_outlier_at_end() {
        let mut scan = create_smooth_scan(20, 5.0);
        scan.ranges[19] = 15.0; // Last point is outlier

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        assert_eq!(result.len(), 19);
    }

    #[test]
    fn test_consecutive_outliers() {
        let mut scan = create_smooth_scan(20, 5.0);
        // Three consecutive outliers in the middle
        scan.ranges[9] = 15.0;
        scan.ranges[10] = 15.0;
        scan.ranges[11] = 15.0;

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        // Middle consecutive outlier might not be detected
        // because its neighbors are also outliers
        // Edge outliers should be detected
        assert!(result.len() < 20);
    }

    #[test]
    fn test_zero_neighbor_count() {
        let config = OutlierFilterConfig {
            neighbor_count: 0,
            distance_threshold: 0.3,
        };
        let filter = OutlierFilter::new(config);

        let mut scan = create_smooth_scan(10, 5.0);
        scan.ranges[5] = 15.0; // Outlier

        let result = filter.apply(&scan);

        // With no neighbors to compare, nothing should be removed
        // (except NaN which is always invalid)
        assert_eq!(result.len(), 10);
    }

    #[test]
    fn test_very_large_neighbor_count() {
        let config = OutlierFilterConfig {
            neighbor_count: 100, // More than scan length
            distance_threshold: 0.3,
        };
        let filter = OutlierFilter::new(config);

        let mut scan = create_smooth_scan(10, 5.0);
        scan.ranges[5] = 15.0; // Outlier

        let result = filter.apply(&scan);

        // Should still work - neighbor count is clamped to available neighbors
        assert_eq!(result.len(), 9);
    }

    #[test]
    fn test_default_config() {
        let config = OutlierFilterConfig::default();
        assert_eq!(config.neighbor_count, 3);
        assert_eq!(config.distance_threshold, 0.3);
    }

    #[test]
    fn test_infinity_is_outlier() {
        let mut scan = create_smooth_scan(10, 5.0);
        scan.ranges[5] = f32::INFINITY;

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        assert_eq!(result.len(), 9);
    }

    #[test]
    fn test_negative_infinity_is_outlier() {
        let mut scan = create_smooth_scan(10, 5.0);
        scan.ranges[5] = f32::NEG_INFINITY;

        let filter = OutlierFilter::default();
        let result = filter.apply(&scan);

        assert_eq!(result.len(), 9);
    }
}
