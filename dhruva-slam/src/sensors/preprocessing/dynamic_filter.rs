//! Dynamic object filtering for LiDAR scans.
//!
//! This module provides filtering of dynamic (moving) objects from LiDAR scans
//! using temporal consistency and optional ray-tracing verification.
//!
//! # Algorithm
//!
//! The filter uses two methods to identify static vs dynamic points:
//!
//! 1. **Temporal Consistency**: Track point ranges over multiple frames
//!    using angular binning. Points that persist at similar ranges for
//!    ≥N frames are considered static.
//!
//! 2. **Ray-Tracing Verification** (optional): Check if a point is consistent
//!    with the existing occupancy grid map. Points that see occupied cells
//!    are more likely static.
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::preprocessing::{DynamicFilter, DynamicFilterConfig};
//! use dhruva_slam::types::{LaserScan, Pose2D};
//!
//! let config = DynamicFilterConfig::default();
//! let mut filter = DynamicFilter::new(config);
//!
//! // Process scans sequentially
//! let filtered_scan = filter.filter(&scan, &current_pose, None);
//! ```

use crate::algorithms::mapping::OccupancyGrid;
use crate::core::types::{LaserScan, Pose2D};

/// Number of angular bins (1 degree resolution for 360 degrees).
const NUM_BINS: usize = 360;

/// Number of frames to track for temporal consistency.
const NUM_FRAMES: usize = 5;

/// Configuration for the dynamic object filter.
#[derive(Debug, Clone)]
pub struct DynamicFilterConfig {
    /// Minimum number of frames a point must persist to be considered static.
    /// Default: 3
    pub persistence_threshold: u8,

    /// Range tolerance for considering two measurements as the same point (meters).
    /// Points within this tolerance are considered matching.
    /// Default: 0.15m
    pub range_tolerance: f32,

    /// Enable ray-tracing verification against occupancy grid.
    /// When enabled, points are also verified against the map.
    /// Default: true
    pub enable_ray_tracing: bool,

    /// Maximum range for ray-tracing checks (meters).
    /// Points beyond this range skip ray-tracing.
    /// Default: 6.0m
    pub max_ray_trace_range: f32,

    /// Occupancy threshold for ray-tracing (0.0-1.0).
    /// Cells with probability above this are considered occupied.
    /// Default: 0.6
    pub occupancy_threshold: f32,

    /// Require BOTH temporal and ray-tracing to pass.
    /// - false (default): OR logic - keep point if EITHER passes (conservative)
    /// - true: AND logic - keep point only if BOTH pass (aggressive filtering)
    pub require_both: bool,

    /// Enable the filter. When disabled, all points pass through.
    /// Default: true
    pub enabled: bool,
}

impl Default for DynamicFilterConfig {
    fn default() -> Self {
        Self {
            persistence_threshold: 3,
            range_tolerance: 0.15,
            enable_ray_tracing: true,
            max_ray_trace_range: 6.0,
            occupancy_threshold: 0.6,
            require_both: false,
            enabled: true,
        }
    }
}

/// Range history for a single angular bin.
#[derive(Debug, Clone)]
struct BinHistory {
    /// Ring buffer of range measurements (NaN = no measurement).
    ranges: [f32; NUM_FRAMES],
    /// Number of frames this bin has had consistent measurements.
    consistency_count: u8,
}

impl Default for BinHistory {
    fn default() -> Self {
        Self {
            ranges: [f32::NAN; NUM_FRAMES],
            consistency_count: 0,
        }
    }
}

impl BinHistory {
    /// Update the bin with a new range measurement.
    fn update(&mut self, range: f32, frame_idx: usize, tolerance: f32) {
        let prev_idx = if frame_idx == 0 {
            NUM_FRAMES - 1
        } else {
            frame_idx - 1
        };

        let prev_range = self.ranges[prev_idx];
        self.ranges[frame_idx] = range;

        // Check if consistent with previous
        if prev_range.is_nan() {
            // First measurement in this bin
            self.consistency_count = 1;
        } else if (range - prev_range).abs() <= tolerance {
            // Consistent with previous
            self.consistency_count = self.consistency_count.saturating_add(1);
        } else {
            // Inconsistent - reset
            self.consistency_count = 1;
        }
    }

    /// Mark this bin as having no measurement for this frame.
    fn mark_empty(&mut self, frame_idx: usize) {
        self.ranges[frame_idx] = f32::NAN;
        // Decay consistency when we lose sight of a point
        self.consistency_count = self.consistency_count.saturating_sub(1);
    }

    /// Check if the bin has sufficient temporal consistency.
    fn is_consistent(&self, threshold: u8) -> bool {
        self.consistency_count >= threshold
    }
}

/// Filter for removing dynamic objects from LiDAR scans.
///
/// Uses temporal consistency tracking and optional ray-tracing
/// to distinguish static structures from moving objects like
/// people and pets.
pub struct DynamicFilter {
    config: DynamicFilterConfig,
    /// Histogram of range history for each angular bin.
    bin_history: Vec<BinHistory>,
    /// Current frame index (cycles 0..NUM_FRAMES).
    current_frame: usize,
    /// Robot poses for each tracked frame (for coordinate transforms).
    frame_poses: [Pose2D; NUM_FRAMES],
}

impl DynamicFilter {
    /// Create a new dynamic filter with the given configuration.
    pub fn new(config: DynamicFilterConfig) -> Self {
        Self {
            config,
            bin_history: vec![BinHistory::default(); NUM_BINS],
            current_frame: 0,
            frame_poses: [Pose2D::identity(); NUM_FRAMES],
        }
    }

    /// Get the current configuration.
    pub fn config(&self) -> &DynamicFilterConfig {
        &self.config
    }

    /// Filter a laser scan, removing likely dynamic points.
    ///
    /// # Arguments
    /// * `scan` - The input laser scan to filter
    /// * `robot_pose` - Current robot pose in world frame
    /// * `map` - Optional occupancy grid for ray-tracing verification
    ///
    /// # Returns
    /// A new LaserScan with dynamic points removed (ranges set to 0.0)
    pub fn filter(
        &mut self,
        scan: &LaserScan,
        robot_pose: &Pose2D,
        map: Option<&OccupancyGrid>,
    ) -> LaserScan {
        if !self.config.enabled || scan.is_empty() {
            return scan.clone();
        }

        // Store current pose
        self.frame_poses[self.current_frame] = *robot_pose;

        // Mark all bins as empty initially for this frame
        let seen_bins = self.update_bins(scan);

        // Mark bins that didn't receive a measurement
        for (bin_idx, bin) in self.bin_history.iter_mut().enumerate() {
            if !seen_bins.contains(&bin_idx) {
                bin.mark_empty(self.current_frame);
            }
        }

        // Filter the scan
        let filtered_ranges = self.filter_ranges(scan, map, robot_pose);

        // Advance frame counter
        self.current_frame = (self.current_frame + 1) % NUM_FRAMES;

        // Create filtered scan
        LaserScan::new(
            scan.angle_min,
            scan.angle_max,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            filtered_ranges,
        )
    }

    /// Update bin history from scan, returning set of bins that received measurements.
    fn update_bins(&mut self, scan: &LaserScan) -> std::collections::HashSet<usize> {
        let mut seen_bins = std::collections::HashSet::new();

        for (i, &range) in scan.ranges.iter().enumerate() {
            if range < scan.range_min || range > scan.range_max {
                continue;
            }

            let angle = scan.angle_min + (i as f32) * scan.angle_increment;
            let bin_idx = self.angle_to_bin(angle);

            self.bin_history[bin_idx].update(
                range,
                self.current_frame,
                self.config.range_tolerance,
            );
            seen_bins.insert(bin_idx);
        }

        seen_bins
    }

    /// Filter ranges based on temporal and ray-tracing checks.
    fn filter_ranges(
        &self,
        scan: &LaserScan,
        map: Option<&OccupancyGrid>,
        robot_pose: &Pose2D,
    ) -> Vec<f32> {
        scan.ranges
            .iter()
            .enumerate()
            .map(|(i, &range)| {
                // Invalid ranges pass through unchanged
                if range < scan.range_min || range > scan.range_max {
                    return range;
                }

                let angle = scan.angle_min + (i as f32) * scan.angle_increment;
                let bin_idx = self.angle_to_bin(angle);

                let temporal_pass =
                    self.bin_history[bin_idx].is_consistent(self.config.persistence_threshold);

                // Determine if ray-tracing should be used and its result
                let ray_trace_result = if self.config.enable_ray_tracing {
                    Some(self.check_ray_tracing(map, robot_pose, angle, range))
                } else {
                    None // Ray-tracing disabled
                };

                let keep_point = match (ray_trace_result, self.config.require_both) {
                    // Ray-tracing disabled: only temporal matters
                    (None, _) => temporal_pass,
                    // Ray-tracing enabled with AND logic: both must pass
                    (Some(ray_pass), true) => temporal_pass && ray_pass,
                    // Ray-tracing enabled with OR logic: either passes
                    (Some(ray_pass), false) => temporal_pass || ray_pass,
                };

                if keep_point {
                    range
                } else {
                    0.0 // Mark as invalid
                }
            })
            .collect()
    }

    /// Check if a point is consistent with the map using ray-tracing.
    fn check_ray_tracing(
        &self,
        map: Option<&OccupancyGrid>,
        robot_pose: &Pose2D,
        angle: f32,
        range: f32,
    ) -> bool {
        // No map available - assume pass
        let Some(grid) = map else {
            return true;
        };

        // Beyond ray-trace range - assume pass
        if range > self.config.max_ray_trace_range {
            return true;
        }

        // Calculate endpoint in world frame
        let world_angle = robot_pose.theta + angle;
        let end_x = robot_pose.x + range * world_angle.cos();
        let end_y = robot_pose.y + range * world_angle.sin();

        // Convert to grid coordinates
        let Some((cell_x, cell_y)) = grid.world_to_cell(end_x, end_y) else {
            return true; // Outside grid bounds
        };

        // Check if cell is occupied (static structure)
        let occupancy = grid.get_probability(cell_x, cell_y);
        occupancy >= self.config.occupancy_threshold
    }

    /// Convert angle to bin index.
    #[inline]
    fn angle_to_bin(&self, angle: f32) -> usize {
        // Normalize angle to [0, 2π)
        let normalized = if angle < 0.0 {
            angle + std::f32::consts::TAU
        } else if angle >= std::f32::consts::TAU {
            angle - std::f32::consts::TAU
        } else {
            angle
        };

        // Convert to bin (0-359)
        let bin = (normalized * (NUM_BINS as f32) / std::f32::consts::TAU) as usize;
        bin.min(NUM_BINS - 1)
    }

    /// Reset the filter history.
    pub fn reset(&mut self) {
        for bin in &mut self.bin_history {
            *bin = BinHistory::default();
        }
        self.current_frame = 0;
        self.frame_poses = [Pose2D::identity(); NUM_FRAMES];
    }

    /// Get statistics about current filter state.
    pub fn stats(&self) -> DynamicFilterStats {
        let consistent_bins = self
            .bin_history
            .iter()
            .filter(|b| b.is_consistent(self.config.persistence_threshold))
            .count();

        // Check the most recently written frame (one before current_frame)
        let last_frame = if self.current_frame == 0 {
            NUM_FRAMES - 1
        } else {
            self.current_frame - 1
        };

        let active_bins = self
            .bin_history
            .iter()
            .filter(|b| !b.ranges[last_frame].is_nan())
            .count();

        DynamicFilterStats {
            total_bins: NUM_BINS,
            consistent_bins,
            active_bins,
            current_frame: self.current_frame,
        }
    }
}

impl Default for DynamicFilter {
    fn default() -> Self {
        Self::new(DynamicFilterConfig::default())
    }
}

/// Statistics about the dynamic filter state.
#[derive(Debug, Clone)]
pub struct DynamicFilterStats {
    /// Total number of angular bins.
    pub total_bins: usize,
    /// Number of bins with consistent history.
    pub consistent_bins: usize,
    /// Number of bins with active measurements.
    pub active_bins: usize,
    /// Current frame index.
    pub current_frame: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_scan(ranges: Vec<f32>) -> LaserScan {
        use std::f32::consts::TAU;
        let n = ranges.len();
        let angle_increment = TAU / n as f32;
        LaserScan::new(
            0.0,
            TAU - angle_increment,
            angle_increment,
            0.1,
            10.0,
            ranges,
        )
    }

    #[test]
    fn test_dynamic_filter_default() {
        let filter = DynamicFilter::default();
        assert!(filter.config.enabled);
        assert_eq!(filter.config.persistence_threshold, 3);
    }

    #[test]
    fn test_angle_to_bin() {
        let filter = DynamicFilter::default();

        assert_eq!(filter.angle_to_bin(0.0), 0);
        assert_eq!(filter.angle_to_bin(std::f32::consts::PI), 180);
        assert_eq!(filter.angle_to_bin(-std::f32::consts::PI), 180);
    }

    #[test]
    fn test_static_points_persist() {
        let config = DynamicFilterConfig {
            persistence_threshold: 3,
            enabled: true,
            enable_ray_tracing: false, // Test temporal only
            ..Default::default()
        };
        let mut filter = DynamicFilter::new(config);
        let pose = Pose2D::identity();

        // Create scan with consistent range at bin 0
        let scan = create_test_scan(vec![5.0; 360]);

        // First few frames - points may be filtered
        for _ in 0..3 {
            filter.filter(&scan, &pose, None);
        }

        // After persistence threshold, points should pass
        let result = filter.filter(&scan, &pose, None);

        // Check that points near bin 0 are kept
        assert!(result.ranges[0] > 0.0);
    }

    #[test]
    fn test_dynamic_points_filtered() {
        let config = DynamicFilterConfig {
            persistence_threshold: 3,
            enabled: true,
            enable_ray_tracing: false,
            ..Default::default()
        };
        let mut filter = DynamicFilter::new(config);
        let pose = Pose2D::identity();

        // First scan with one set of ranges
        let scan1 = create_test_scan(vec![5.0; 360]);
        filter.filter(&scan1, &pose, None);
        filter.filter(&scan1, &pose, None);

        // Sudden change in one area
        let mut ranges = vec![5.0; 360];
        ranges[0] = 2.0; // Big jump in range
        let scan2 = create_test_scan(ranges);

        let result = filter.filter(&scan2, &pose, None);

        // The changed point should be filtered (inconsistent)
        assert_eq!(result.ranges[0], 0.0);
    }

    #[test]
    fn test_disabled_filter_passes_all() {
        let config = DynamicFilterConfig {
            enabled: false,
            ..Default::default()
        };
        let mut filter = DynamicFilter::new(config);
        let pose = Pose2D::identity();

        let scan = create_test_scan(vec![5.0; 360]);
        let result = filter.filter(&scan, &pose, None);

        // All points should pass when disabled
        assert_eq!(result.ranges, scan.ranges);
    }

    #[test]
    fn test_empty_scan() {
        let mut filter = DynamicFilter::default();
        let pose = Pose2D::identity();

        let scan = LaserScan::default();
        let result = filter.filter(&scan, &pose, None);

        assert!(result.is_empty());
    }

    #[test]
    fn test_filter_stats() {
        let mut filter = DynamicFilter::default();
        let pose = Pose2D::identity();

        // Process a scan
        let scan = create_test_scan(vec![5.0; 360]);
        filter.filter(&scan, &pose, None);

        let stats = filter.stats();
        assert_eq!(stats.total_bins, NUM_BINS);
        assert!(stats.active_bins > 0);
    }

    #[test]
    fn test_reset() {
        let mut filter = DynamicFilter::default();
        let pose = Pose2D::identity();

        // Process some scans
        let scan = create_test_scan(vec![5.0; 360]);
        filter.filter(&scan, &pose, None);
        filter.filter(&scan, &pose, None);

        // Reset
        filter.reset();

        let stats = filter.stats();
        assert_eq!(stats.consistent_bins, 0);
        assert_eq!(stats.current_frame, 0);
    }

    #[test]
    fn test_and_vs_or_logic() {
        // Test with OR logic (default)
        let or_config = DynamicFilterConfig {
            require_both: false,
            enable_ray_tracing: false,
            persistence_threshold: 3,
            ..Default::default()
        };
        let mut or_filter = DynamicFilter::new(or_config);

        // Test with AND logic
        let and_config = DynamicFilterConfig {
            require_both: true,
            enable_ray_tracing: false,
            persistence_threshold: 3,
            ..Default::default()
        };
        let mut and_filter = DynamicFilter::new(and_config);

        let pose = Pose2D::identity();
        let scan = create_test_scan(vec![5.0; 360]);

        // Process same scans
        for _ in 0..5 {
            or_filter.filter(&scan, &pose, None);
            and_filter.filter(&scan, &pose, None);
        }

        let or_result = or_filter.filter(&scan, &pose, None);
        let and_result = and_filter.filter(&scan, &pose, None);

        // Without ray-tracing map, both should behave similarly
        // (temporal passes, ray-tracing defaults to pass)
        let or_valid = or_result.ranges.iter().filter(|&&r| r > 0.0).count();
        let and_valid = and_result.ranges.iter().filter(|&&r| r > 0.0).count();

        // Both should have similar results in this case
        assert!(or_valid > 0);
        assert!(and_valid > 0);
    }

    #[test]
    fn test_range_tolerance() {
        let config = DynamicFilterConfig {
            persistence_threshold: 2,
            range_tolerance: 0.2, // 20cm tolerance
            enable_ray_tracing: false,
            ..Default::default()
        };
        let mut filter = DynamicFilter::new(config);
        let pose = Pose2D::identity();

        // Scan with slight range variation (within tolerance)
        let scan1 = create_test_scan(vec![5.0; 360]);
        let scan2 = create_test_scan(vec![5.1; 360]); // +10cm

        filter.filter(&scan1, &pose, None);
        let result = filter.filter(&scan2, &pose, None);

        // Points should be kept despite small variation
        assert!(result.ranges[0] > 0.0);
    }
}
