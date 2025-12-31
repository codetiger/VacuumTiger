//! Performance metrics for SLAM evaluation.
//!
//! This module provides timing and resource tracking for benchmarking SLAM operations.
//! Unlike accuracy metrics, performance metrics are not part of Cartographer but are
//! essential for maintaining real-time performance.
//!
//! ## Usage
//!
//! ```rust,ignore
//! use vastu_slam::evaluation::PerformanceTracker;
//!
//! let mut tracker = PerformanceTracker::new();
//!
//! // Time an operation
//! let start = std::time::Instant::now();
//! // ... do scan matching ...
//! tracker.record("scan_matching", start.elapsed());
//!
//! // Get metrics
//! let metrics = tracker.metrics();
//! metrics.print();
//! ```

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Timing statistics for a single operation.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct OperationTiming {
    /// Operation name.
    pub name: String,

    /// Mean duration (milliseconds).
    pub mean_ms: f32,

    /// Standard deviation (milliseconds).
    pub std_ms: f32,

    /// Minimum duration (milliseconds).
    pub min_ms: f32,

    /// Maximum duration (milliseconds).
    pub max_ms: f32,

    /// Median duration (milliseconds).
    pub median_ms: f32,

    /// Total number of samples.
    pub count: usize,

    /// Total time spent (milliseconds).
    pub total_ms: f32,
}

impl OperationTiming {
    /// Create from a list of durations.
    pub fn from_durations(name: &str, durations: &[Duration]) -> Self {
        if durations.is_empty() {
            return Self {
                name: name.to_string(),
                ..Default::default()
            };
        }

        let ms_values: Vec<f32> = durations.iter().map(|d| d.as_secs_f32() * 1000.0).collect();
        let n = ms_values.len() as f32;

        let total_ms = ms_values.iter().sum::<f32>();
        let mean_ms = total_ms / n;

        let variance = ms_values.iter().map(|v| (v - mean_ms).powi(2)).sum::<f32>() / n;
        let std_ms = variance.sqrt();

        let min_ms = ms_values.iter().cloned().fold(f32::INFINITY, f32::min);
        let max_ms = ms_values.iter().cloned().fold(f32::NEG_INFINITY, f32::max);

        let mut sorted = ms_values.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median_ms = if sorted.len().is_multiple_of(2) {
            (sorted[sorted.len() / 2 - 1] + sorted[sorted.len() / 2]) / 2.0
        } else {
            sorted[sorted.len() / 2]
        };

        Self {
            name: name.to_string(),
            mean_ms,
            std_ms,
            min_ms,
            max_ms,
            median_ms,
            count: durations.len(),
            total_ms,
        }
    }

    /// Check if the operation exceeds a threshold (for regression testing).
    pub fn exceeds_threshold(&self, threshold_ms: f32) -> bool {
        self.mean_ms > threshold_ms
    }

    /// Format as a single line.
    pub fn summary(&self) -> String {
        format!(
            "{}: {:.2} ± {:.2} ms (min: {:.2}, max: {:.2}, n={})",
            self.name, self.mean_ms, self.std_ms, self.min_ms, self.max_ms, self.count
        )
    }
}

/// Aggregate performance metrics.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    /// Per-operation timing.
    pub operations: HashMap<String, OperationTiming>,

    /// Scans processed per second.
    pub scans_per_second: f32,

    /// Real-time factor (1.0 = real-time, >1.0 = faster than real-time).
    pub real_time_factor: f32,

    /// Total processing time (milliseconds).
    pub total_time_ms: f32,

    /// Number of scans processed.
    pub num_scans: usize,
}

impl PerformanceMetrics {
    /// Print all metrics.
    pub fn print(&self) {
        println!("=== Performance Metrics ===");
        println!("Scans processed: {}", self.num_scans);
        println!("Scans per second: {:.2}", self.scans_per_second);
        println!("Real-time factor: {:.2}x", self.real_time_factor);
        println!("Total time: {:.2} ms", self.total_time_ms);
        println!();
        println!("Operation timings:");

        let mut ops: Vec<_> = self.operations.values().collect();
        ops.sort_by(|a, b| {
            b.total_ms
                .partial_cmp(&a.total_ms)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        for op in ops {
            println!("  {}", op.summary());
        }
    }

    /// Check if any operation exceeds its threshold.
    pub fn check_thresholds(&self, thresholds: &HashMap<String, f32>) -> Vec<String> {
        let mut violations = Vec::new();
        for (name, threshold) in thresholds {
            if let Some(op) = self.operations.get(name)
                && op.exceeds_threshold(*threshold)
            {
                violations.push(format!(
                    "{}: {:.2}ms exceeds threshold of {}ms",
                    name, op.mean_ms, threshold
                ));
            }
        }
        violations
    }
}

/// Performance tracker for recording operation timings.
#[derive(Clone, Debug, Default)]
pub struct PerformanceTracker {
    /// Recorded durations for each operation.
    durations: HashMap<String, Vec<Duration>>,

    /// Start time of tracking.
    start_time: Option<Instant>,

    /// Number of scans tracked.
    scan_count: usize,

    /// Expected scan interval (for real-time factor calculation).
    expected_scan_interval: Duration,
}

impl PerformanceTracker {
    /// Create a new performance tracker.
    pub fn new() -> Self {
        Self {
            durations: HashMap::new(),
            start_time: None,
            scan_count: 0,
            expected_scan_interval: Duration::from_millis(200), // 5Hz default
        }
    }

    /// Create with a custom expected scan interval.
    pub fn with_scan_interval(interval: Duration) -> Self {
        Self {
            expected_scan_interval: interval,
            ..Self::new()
        }
    }

    /// Start tracking (call at beginning of processing).
    pub fn start(&mut self) {
        self.start_time = Some(Instant::now());
    }

    /// Record a timed operation.
    pub fn record(&mut self, operation: &str, duration: Duration) {
        self.durations
            .entry(operation.to_string())
            .or_default()
            .push(duration);
    }

    /// Record the start of an operation and return a guard for automatic timing.
    pub fn time(&mut self, operation: &str) -> TimingGuard {
        TimingGuard {
            operation: operation.to_string(),
            start: Instant::now(),
            tracker: self as *mut Self,
        }
    }

    /// Increment scan count.
    pub fn record_scan(&mut self) {
        self.scan_count += 1;
    }

    /// Get the number of recorded scans.
    pub fn scan_count(&self) -> usize {
        self.scan_count
    }

    /// Compute aggregate metrics.
    pub fn metrics(&self) -> PerformanceMetrics {
        let operations: HashMap<String, OperationTiming> = self
            .durations
            .iter()
            .map(|(name, durs)| (name.clone(), OperationTiming::from_durations(name, durs)))
            .collect();

        let total_time_ms = self
            .start_time
            .map(|s| s.elapsed().as_secs_f32() * 1000.0)
            .unwrap_or(0.0);

        let scans_per_second = if total_time_ms > 0.0 {
            self.scan_count as f32 / (total_time_ms / 1000.0)
        } else {
            0.0
        };

        // Real-time factor: how fast are we compared to expected scan rate
        let expected_scans_per_second = 1.0 / self.expected_scan_interval.as_secs_f32();
        let real_time_factor = if expected_scans_per_second > 0.0 {
            scans_per_second / expected_scans_per_second
        } else {
            0.0
        };

        PerformanceMetrics {
            operations,
            scans_per_second,
            real_time_factor,
            total_time_ms,
            num_scans: self.scan_count,
        }
    }

    /// Reset all tracking data.
    pub fn reset(&mut self) {
        self.durations.clear();
        self.start_time = None;
        self.scan_count = 0;
    }
}

/// Guard for automatic timing of operations.
///
/// When dropped, records the elapsed time to the tracker.
pub struct TimingGuard {
    operation: String,
    start: Instant,
    tracker: *mut PerformanceTracker,
}

impl Drop for TimingGuard {
    fn drop(&mut self) {
        let duration = self.start.elapsed();
        // SAFETY: The guard is always created with a valid mutable reference
        // and should not outlive the tracker.
        unsafe {
            (*self.tracker).record(&self.operation, duration);
        }
    }
}

/// Convenience macro for timing a block.
#[macro_export]
macro_rules! time_operation {
    ($tracker:expr, $name:expr, $block:block) => {{
        let _guard = $tracker.time($name);
        $block
    }};
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_operation_timing() {
        let durations = vec![
            Duration::from_millis(10),
            Duration::from_millis(20),
            Duration::from_millis(30),
        ];

        let timing = OperationTiming::from_durations("test", &durations);

        assert_eq!(timing.count, 3);
        assert!((timing.mean_ms - 20.0).abs() < 0.1);
        assert!((timing.min_ms - 10.0).abs() < 0.1);
        assert!((timing.max_ms - 30.0).abs() < 0.1);
    }

    #[test]
    fn test_performance_tracker() {
        let mut tracker = PerformanceTracker::new();
        tracker.start();

        // Record some operations
        tracker.record("scan_matching", Duration::from_millis(15));
        tracker.record("scan_matching", Duration::from_millis(12));
        tracker.record("map_update", Duration::from_millis(5));

        tracker.record_scan();
        tracker.record_scan();

        let metrics = tracker.metrics();

        assert_eq!(metrics.num_scans, 2);
        assert!(metrics.operations.contains_key("scan_matching"));
        assert!(metrics.operations.contains_key("map_update"));

        let scan_matching = metrics.operations.get("scan_matching").unwrap();
        assert_eq!(scan_matching.count, 2);
        assert!((scan_matching.mean_ms - 13.5).abs() < 0.1);
    }

    #[test]
    fn test_timing_guard() {
        let mut tracker = PerformanceTracker::new();

        {
            let _guard = tracker.time("test_op");
            thread::sleep(Duration::from_millis(10));
        }

        let metrics = tracker.metrics();
        let test_op = metrics.operations.get("test_op").unwrap();

        assert_eq!(test_op.count, 1);
        assert!(test_op.mean_ms >= 10.0); // At least 10ms
    }

    #[test]
    fn test_threshold_checking() {
        let mut tracker = PerformanceTracker::new();
        tracker.record("fast_op", Duration::from_millis(5));
        tracker.record("slow_op", Duration::from_millis(50));

        let metrics = tracker.metrics();

        let mut thresholds = HashMap::new();
        thresholds.insert("fast_op".to_string(), 10.0);
        thresholds.insert("slow_op".to_string(), 20.0);

        let violations = metrics.check_thresholds(&thresholds);

        assert_eq!(violations.len(), 1);
        assert!(violations[0].contains("slow_op"));
    }
}
