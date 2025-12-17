//! Range gating filter for LiDAR scans.
//!
//! Removes points with invalid or out-of-bounds range values.

use crate::core::types::LaserScan;

/// Configuration for range filtering.
#[derive(Debug, Clone, Copy)]
pub struct RangeFilterConfig {
    /// Minimum valid range in meters.
    ///
    /// Points closer than this are removed (may be self-reflection).
    /// Default: 0.1m
    pub min_range: f32,

    /// Maximum valid range in meters.
    ///
    /// Points farther than this are removed (unreliable).
    /// Default: 12.0m (typical for Delta-2D)
    pub max_range: f32,
}

impl Default for RangeFilterConfig {
    fn default() -> Self {
        Self {
            min_range: 0.1,
            max_range: 12.0,
        }
    }
}

/// Range filter for removing invalid LiDAR returns.
///
/// Filters out:
/// - Ranges less than `min_range`
/// - Ranges greater than `max_range`
/// - Zero or negative ranges
/// - NaN or infinite ranges
#[derive(Debug, Clone)]
pub struct RangeFilter {
    config: RangeFilterConfig,
}

impl RangeFilter {
    /// Create a new range filter with the given configuration.
    pub fn new(config: RangeFilterConfig) -> Self {
        Self { config }
    }

    /// Check if a range value is valid.
    #[inline]
    pub fn is_valid(&self, range: f32) -> bool {
        range.is_finite() && range >= self.config.min_range && range <= self.config.max_range
    }

    /// Apply range filtering to a laser scan.
    ///
    /// Returns a new scan with only valid points.
    pub fn apply(&self, scan: &LaserScan) -> LaserScan {
        let mut new_ranges = Vec::with_capacity(scan.ranges.len());
        let mut new_angles = Vec::with_capacity(scan.ranges.len());
        let mut new_intensities = scan
            .intensities
            .as_ref()
            .map(|_| Vec::with_capacity(scan.ranges.len()));
        let mut new_angle_min = None;
        let mut new_angle_max = 0.0f32;

        for (i, &range) in scan.ranges.iter().enumerate() {
            if self.is_valid(range) {
                let angle = scan.get_angle(i);

                if new_angle_min.is_none() {
                    new_angle_min = Some(angle);
                }
                new_angle_max = angle;

                new_ranges.push(range);
                new_angles.push(angle);

                if let (Some(new_int), Some(old_int)) = (&mut new_intensities, &scan.intensities)
                    && let Some(&intensity) = old_int.get(i)
                {
                    new_int.push(intensity);
                }
            }
        }

        let angle_min = new_angle_min.unwrap_or(0.0);
        let angle_increment = if new_ranges.len() > 1 {
            (new_angle_max - angle_min) / (new_ranges.len() - 1) as f32
        } else {
            scan.angle_increment
        };

        LaserScan {
            angle_min,
            angle_max: new_angle_max,
            angle_increment,
            range_min: self.config.min_range,
            range_max: self.config.max_range,
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

impl Default for RangeFilter {
    fn default() -> Self {
        Self::new(RangeFilterConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    fn create_test_scan() -> LaserScan {
        // Create scan with various range values including invalid ones
        let ranges = vec![
            0.05,          // Too close (< 0.1)
            0.15,          // Valid
            1.0,           // Valid
            5.0,           // Valid
            10.0,          // Valid
            15.0,          // Too far (> 12.0)
            0.0,           // Invalid (zero)
            -1.0,          // Invalid (negative)
            f32::NAN,      // Invalid (NaN)
            f32::INFINITY, // Invalid (infinity)
        ];

        LaserScan::new(0.0, TAU, TAU / 10.0, 0.1, 12.0, ranges)
    }

    #[test]
    fn test_range_filter_removes_invalid() {
        let scan = create_test_scan();
        let filter = RangeFilter::default();

        let filtered = filter.apply(&scan);

        // Only 4 valid points: 0.15, 1.0, 5.0, 10.0
        assert_eq!(filtered.len(), 4);

        for &range in &filtered.ranges {
            assert!(filter.is_valid(range));
        }
    }

    #[test]
    fn test_custom_range_config() {
        let config = RangeFilterConfig {
            min_range: 0.5,
            max_range: 8.0,
        };
        let filter = RangeFilter::new(config);

        let scan = create_test_scan();
        let filtered = filter.apply(&scan);

        // Only 2 valid: 1.0, 5.0 (0.15 now too close, 10.0 now too far)
        assert_eq!(filtered.len(), 2);
    }

    #[test]
    fn test_empty_scan() {
        let scan = LaserScan::default();
        let filter = RangeFilter::default();

        let filtered = filter.apply(&scan);

        assert!(filtered.is_empty());
    }

    #[test]
    fn test_all_valid() {
        let ranges = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let scan = LaserScan::new(0.0, TAU, TAU / 5.0, 0.1, 12.0, ranges);
        let filter = RangeFilter::default();

        let filtered = filter.apply(&scan);

        assert_eq!(filtered.len(), 5);
    }

    #[test]
    fn test_preserves_intensities() {
        let ranges = vec![1.0, 0.0, 2.0, 0.0, 3.0]; // 3 valid
        let intensities = vec![100, 200, 150, 250, 175];
        let scan =
            LaserScan::new(0.0, TAU, TAU / 5.0, 0.1, 12.0, ranges).with_intensities(intensities);

        let filter = RangeFilter::default();
        let filtered = filter.apply(&scan);

        assert_eq!(filtered.len(), 3);
        assert!(filtered.intensities.is_some());

        let filtered_int = filtered.intensities.as_ref().unwrap();
        assert_eq!(filtered_int.len(), 3);
        assert_eq!(filtered_int[0], 100);
        assert_eq!(filtered_int[1], 150);
        assert_eq!(filtered_int[2], 175);
    }

    // ========================================================================
    // Edge Case Tests
    // ========================================================================

    #[test]
    fn test_all_invalid_ranges() {
        // Scan where every point is invalid
        let ranges = vec![0.0, -1.0, f32::NAN, f32::INFINITY, 0.05, 15.0];
        let scan = LaserScan::new(0.0, TAU, TAU / 6.0, 0.1, 12.0, ranges);
        let filter = RangeFilter::default();

        let filtered = filter.apply(&scan);

        assert!(filtered.is_empty());
    }

    #[test]
    fn test_range_exactly_at_min_boundary() {
        let config = RangeFilterConfig {
            min_range: 0.5,
            max_range: 10.0,
        };
        let filter = RangeFilter::new(config);

        // Exactly at min_range should be valid
        assert!(filter.is_valid(0.5));
        // Just below should be invalid
        assert!(!filter.is_valid(0.4999));
    }

    #[test]
    fn test_range_exactly_at_max_boundary() {
        let config = RangeFilterConfig {
            min_range: 0.1,
            max_range: 10.0,
        };
        let filter = RangeFilter::new(config);

        // Exactly at max_range should be valid
        assert!(filter.is_valid(10.0));
        // Just above should be invalid
        assert!(!filter.is_valid(10.001));
    }

    #[test]
    fn test_single_valid_point() {
        let ranges = vec![0.0, 0.0, 5.0, 0.0, 0.0];
        let scan = LaserScan::new(0.0, TAU, TAU / 5.0, 0.1, 12.0, ranges);
        let filter = RangeFilter::default();

        let filtered = filter.apply(&scan);

        assert_eq!(filtered.len(), 1);
        assert_eq!(filtered.ranges[0], 5.0);
    }

    #[test]
    fn test_negative_infinity() {
        let filter = RangeFilter::default();
        assert!(!filter.is_valid(f32::NEG_INFINITY));
    }

    #[test]
    fn test_very_small_positive_range() {
        let config = RangeFilterConfig {
            min_range: 0.0, // Allow any positive
            max_range: 100.0,
        };
        let filter = RangeFilter::new(config);

        // f32::MIN_POSITIVE is the smallest positive number
        assert!(filter.is_valid(f32::MIN_POSITIVE));
    }

    #[test]
    fn test_default_config() {
        let config = RangeFilterConfig::default();
        assert_eq!(config.min_range, 0.1);
        assert_eq!(config.max_range, 12.0);
    }

    #[test]
    fn test_large_scan_all_invalid() {
        // 1000 invalid points
        let ranges = vec![0.0; 1000];
        let scan = LaserScan::new(0.0, TAU, TAU / 1000.0, 0.1, 12.0, ranges);
        let filter = RangeFilter::default();

        let filtered = filter.apply(&scan);

        assert!(filtered.is_empty());
    }
}
