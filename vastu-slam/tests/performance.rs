//! Performance integration tests for VastuSLAM.
//!
//! These tests verify that SLAM operations meet performance targets.

mod common;

use std::time::{Duration, Instant};
use vastu_slam::evaluation::{OperationTiming, PerformanceTracker};
use vastu_slam::{
    CorrelativeMatcher, CorrelativeMatcherConfig, MapConfig, OccupancyGridMap, Pose2D,
};

/// Performance thresholds (in milliseconds) for release builds.
/// Debug builds are expected to be ~5-10x slower.
const SCAN_MATCHING_THRESHOLD_MS: f32 = 15.0;
const MAP_UPDATE_THRESHOLD_MS: f32 = 2.0;

/// Multiplier for debug mode thresholds.
#[cfg(debug_assertions)]
const THRESHOLD_MULTIPLIER: f32 = 10.0;
#[cfg(not(debug_assertions))]
const THRESHOLD_MULTIPLIER: f32 = 1.0;

// ============================================================================
// Operation Timing Tests
// ============================================================================

#[test]
fn test_scan_matching_performance() {
    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    // Initialize map with some data
    for _ in 0..10 {
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let scan = common::room_scan(6.0, 6.0, 3.0, 3.0, 360);
        map.observe_lidar(&scan, pose);
    }

    let matcher_config = CorrelativeMatcherConfig::default();
    let matcher = CorrelativeMatcher::new(matcher_config);

    let scan = common::room_scan(6.0, 6.0, 3.1, 3.0, 360);
    let initial_pose = Pose2D::new(3.0, 3.0, 0.0);

    // Warm-up
    for _ in 0..3 {
        let _ = matcher.match_scan(&scan, initial_pose, map.storage());
    }

    // Measure
    let mut times = Vec::new();
    for _ in 0..20 {
        let start = Instant::now();
        let _ = matcher.match_scan(&scan, initial_pose, map.storage());
        times.push(start.elapsed());
    }

    let timing = OperationTiming::from_durations("scan_matching", &times);

    println!("Scan matching: {}", timing.summary());

    let threshold = SCAN_MATCHING_THRESHOLD_MS * THRESHOLD_MULTIPLIER;
    assert!(
        timing.mean_ms < threshold,
        "Scan matching mean {:.2}ms exceeds threshold {}ms",
        timing.mean_ms,
        threshold
    );
}

#[test]
fn test_map_update_performance() {
    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    let scan = common::room_scan(10.0, 10.0, 5.0, 5.0, 360);
    let pose = Pose2D::new(5.0, 5.0, 0.0);

    // Warm-up
    for _ in 0..3 {
        map.observe_lidar(&scan, pose);
    }
    map.clear();

    // Measure
    let mut times = Vec::new();
    for i in 0..50 {
        let p = Pose2D::new(5.0 + (i as f32 * 0.01), 5.0, 0.0);
        let start = Instant::now();
        map.observe_lidar(&scan, p);
        times.push(start.elapsed());
    }

    let timing = OperationTiming::from_durations("map_update", &times);

    println!("Map update: {}", timing.summary());

    let threshold = MAP_UPDATE_THRESHOLD_MS * THRESHOLD_MULTIPLIER;
    assert!(
        timing.mean_ms < threshold,
        "Map update mean {:.2}ms exceeds threshold {}ms",
        timing.mean_ms,
        threshold
    );
}

// ============================================================================
// Performance Tracker Tests
// ============================================================================

#[test]
fn test_performance_tracker_integration() {
    let mut tracker = PerformanceTracker::new();
    tracker.start();

    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);
    let matcher_config = CorrelativeMatcherConfig::default();
    let matcher = CorrelativeMatcher::new(matcher_config);

    // Simulate a SLAM session
    for i in 0..20 {
        let pose = Pose2D::new(i as f32 * 0.2, 0.0, 0.0);
        let scan = common::room_scan(8.0, 8.0, 4.0 + i as f32 * 0.2, 4.0, 360);

        // Time scan matching
        let match_start = Instant::now();
        let _ = matcher.match_scan(&scan, pose, map.storage());
        tracker.record("scan_matching", match_start.elapsed());

        // Time map update
        let update_start = Instant::now();
        map.observe_lidar(&scan, pose);
        tracker.record("map_update", update_start.elapsed());

        tracker.record_scan();
    }

    let metrics = tracker.metrics();

    println!("\n=== Performance Report ===");
    metrics.print();

    // Verify we have timing data
    assert!(
        metrics.operations.contains_key("scan_matching"),
        "Should have scan matching timing"
    );
    assert!(
        metrics.operations.contains_key("map_update"),
        "Should have map update timing"
    );
    assert_eq!(metrics.num_scans, 20);
    assert!(metrics.scans_per_second > 0.0);
}

#[test]
fn test_threshold_violations() {
    let mut tracker = PerformanceTracker::new();

    // Simulate some operations
    tracker.record("fast_op", Duration::from_millis(5));
    tracker.record("slow_op", Duration::from_millis(50));

    let metrics = tracker.metrics();

    let mut thresholds = std::collections::HashMap::new();
    thresholds.insert("fast_op".to_string(), 10.0);
    thresholds.insert("slow_op".to_string(), 20.0);

    let violations = metrics.check_thresholds(&thresholds);

    assert_eq!(violations.len(), 1, "Should have one violation");
    assert!(
        violations[0].contains("slow_op"),
        "Violation should be for slow_op"
    );
}

// ============================================================================
// Real-time Factor Tests
// ============================================================================

#[test]
fn test_realtime_factor() {
    let mut tracker = PerformanceTracker::with_scan_interval(Duration::from_millis(200)); // 5Hz
    tracker.start();

    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    // Simulate processing faster than real-time
    for i in 0..50 {
        let pose = Pose2D::new(i as f32 * 0.1, 0.0, 0.0);
        let scan = common::simple_scan(360, 8.0);

        let start = Instant::now();
        map.observe_lidar(&scan, pose);
        tracker.record("total", start.elapsed());
        tracker.record_scan();
    }

    let metrics = tracker.metrics();

    println!(
        "Real-time factor: {:.2}x (scans/sec: {:.1})",
        metrics.real_time_factor, metrics.scans_per_second
    );

    // Should be faster than real-time
    assert!(
        metrics.real_time_factor > 1.0,
        "Should process faster than real-time, got {:.2}x",
        metrics.real_time_factor
    );
}

// ============================================================================
// Stress Tests
// ============================================================================

#[test]
fn test_sustained_performance() {
    let mut tracker = PerformanceTracker::new();
    tracker.start();

    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    // Process many scans to check for performance degradation
    let num_scans = 100;
    let mut early_times = Vec::new();
    let mut late_times = Vec::new();

    for i in 0..num_scans {
        let angle = i as f32 * 0.1;
        let x = 5.0 + 3.0 * angle.cos();
        let y = 5.0 + 3.0 * angle.sin();
        let pose = Pose2D::new(x, y, angle);
        let scan = common::room_scan(10.0, 10.0, x, y, 360);

        let start = Instant::now();
        map.observe_lidar(&scan, pose);
        let elapsed = start.elapsed();

        tracker.record("map_update", elapsed);
        tracker.record_scan();

        if i < 10 {
            early_times.push(elapsed);
        } else if i >= num_scans - 10 {
            late_times.push(elapsed);
        }
    }

    let early_avg =
        early_times.iter().sum::<Duration>().as_secs_f32() / early_times.len() as f32 * 1000.0;
    let late_avg =
        late_times.iter().sum::<Duration>().as_secs_f32() / late_times.len() as f32 * 1000.0;

    println!(
        "Early avg: {:.3}ms, Late avg: {:.3}ms (ratio: {:.2}x)",
        early_avg,
        late_avg,
        late_avg / early_avg
    );

    // Performance shouldn't degrade by more than 2x
    assert!(
        late_avg < early_avg * 2.0,
        "Performance degradation too high: early={:.2}ms, late={:.2}ms",
        early_avg,
        late_avg
    );

    let metrics = tracker.metrics();
    println!("\nSustained performance report:");
    metrics.print();
}
