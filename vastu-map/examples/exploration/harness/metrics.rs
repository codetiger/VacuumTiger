//! Quantitative validation metrics
//!
//! Computes error metrics comparing SLAM output against ground truth:
//! - Position and orientation error
//! - Wall accuracy (how close detected lines are to actual walls in map)
//! - Timing statistics for algorithm performance

use sangam_io::devices::mock::map_loader::SimulationMap;

use vastu_map::TimingBreakdown;
use vastu_map::VectorMap;
use vastu_map::core::Pose2D;
use vastu_map::features::Line2D;

/// Statistics collected during ICP matching over a test run.
#[derive(Debug, Clone, Default)]
pub struct ConvergenceStats {
    /// Total ICP iterations across all observations
    pub total_iterations: usize,
    /// Number of observations processed
    pub observation_count: usize,
    /// Maximum ICP iterations in any single observation
    pub max_iterations: usize,
    /// Number of observations where ICP failed to converge
    pub convergence_failures: usize,
}

impl ConvergenceStats {
    /// Create a new empty stats tracker.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record an observation result.
    pub fn record(&mut self, iterations: usize, converged: bool) {
        self.total_iterations += iterations;
        self.observation_count += 1;
        if iterations > self.max_iterations {
            self.max_iterations = iterations;
        }
        if !converged && iterations > 0 {
            self.convergence_failures += 1;
        }
    }

    /// Get average ICP iterations per observation.
    pub fn avg_iterations(&self) -> f32 {
        if self.observation_count == 0 {
            0.0
        } else {
            self.total_iterations as f32 / self.observation_count as f32
        }
    }
}

/// Single stage timing accumulator.
#[derive(Debug, Clone, Default)]
struct StageTiming {
    total_us: u64,
    min_us: u64,
    max_us: u64,
    count: usize,
}

impl StageTiming {
    fn new() -> Self {
        Self {
            total_us: 0,
            min_us: u64::MAX,
            max_us: 0,
            count: 0,
        }
    }

    fn record(&mut self, us: u64) {
        self.total_us += us;
        self.min_us = self.min_us.min(us);
        self.max_us = self.max_us.max(us);
        self.count += 1;
    }

    fn avg_us(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.total_us as f64 / self.count as f64
        }
    }

    fn min_display(&self) -> u64 {
        if self.min_us == u64::MAX {
            0
        } else {
            self.min_us
        }
    }
}

/// Accumulated timing statistics across all observations.
#[derive(Debug, Clone, Default)]
pub struct TimingStats {
    icp_matching: StageTiming,
    line_extraction: StageTiming,
    corner_detection: StageTiming,
    association: StageTiming,
    merging: StageTiming,
    new_features: StageTiming,
    loop_closure: StageTiming,
    total: StageTiming,
}

impl TimingStats {
    /// Create a new empty timing stats tracker.
    pub fn new() -> Self {
        Self {
            icp_matching: StageTiming::new(),
            line_extraction: StageTiming::new(),
            corner_detection: StageTiming::new(),
            association: StageTiming::new(),
            merging: StageTiming::new(),
            new_features: StageTiming::new(),
            loop_closure: StageTiming::new(),
            total: StageTiming::new(),
        }
    }

    /// Record timing from a single observation.
    pub fn record(&mut self, timing: &TimingBreakdown) {
        self.icp_matching.record(timing.icp_matching_us);
        self.line_extraction.record(timing.line_extraction_us);
        self.corner_detection.record(timing.corner_detection_us);
        self.association.record(timing.association_us);
        self.merging.record(timing.merging_us);
        self.new_features.record(timing.new_features_us);
        self.loop_closure.record(timing.loop_closure_us);
        self.total.record(timing.total_us);
    }

    /// Get number of observations recorded.
    fn observation_count(&self) -> usize {
        self.total.count
    }

    /// Print a formatted timing summary.
    pub fn print_summary(&self) {
        let count = self.observation_count();
        if count == 0 {
            println!("No timing data collected.");
            return;
        }

        println!("\n=== Timing Summary ({} observations) ===", count);
        println!(
            "{:<24} | {:>10} | {:>10} | {:>10} | {:>10}",
            "Stage", "Total", "Avg", "Min", "Max"
        );
        println!("{}", "-".repeat(73));

        self.print_stage("ICP Matching", &self.icp_matching);
        self.print_stage("Line Extraction", &self.line_extraction);
        self.print_stage("Corner Detection", &self.corner_detection);
        self.print_stage("Association", &self.association);
        self.print_stage("Merging", &self.merging);
        self.print_stage("New Features", &self.new_features);
        self.print_stage("Loop Closure", &self.loop_closure);

        println!("{}", "-".repeat(73));
        self.print_stage("Total per Observe", &self.total);
    }

    fn print_stage(&self, name: &str, stage: &StageTiming) {
        println!(
            "{:<24} | {:>10} | {:>10} | {:>10} | {:>10}",
            name,
            Self::format_us(stage.total_us),
            Self::format_us(stage.avg_us() as u64),
            Self::format_us(stage.min_display()),
            Self::format_us(stage.max_us),
        );
    }

    fn format_us(us: u64) -> String {
        if us >= 1_000_000 {
            format!("{:.2} s", us as f64 / 1_000_000.0)
        } else if us >= 1_000 {
            format!("{:.2} ms", us as f64 / 1_000.0)
        } else {
            format!("{} µs", us)
        }
    }
}

/// Test metrics results.
#[derive(Debug, Clone)]
pub struct TestMetrics {
    /// Position error in meters (Euclidean distance)
    pub position_error: f32,
    /// Orientation error in radians
    pub orientation_error: f32,
    /// Percentage of detected line points that are near actual walls (0.0-1.0)
    pub wall_accuracy: f32,
    /// Number of detected walls/features
    pub detected_walls: usize,
    /// ICP convergence statistics
    pub convergence: ConvergenceStats,
}

impl TestMetrics {
    /// Compute all metrics comparing SLAM output to ground truth map.
    pub fn compute(
        slam: &VectorMap,
        map: &SimulationMap,
        final_pose: Pose2D,
        truth_pose: Pose2D,
        convergence: ConvergenceStats,
    ) -> Self {
        // Position error
        let position_error =
            ((final_pose.x - truth_pose.x).powi(2) + (final_pose.y - truth_pose.y).powi(2)).sqrt();

        // Orientation error (normalized to [-PI, PI])
        let orientation_error = angle_diff(final_pose.theta, truth_pose.theta).abs();

        // Get detected lines
        let detected_lines = slam.lines();
        let detected_walls = detected_lines.len();

        // Compute wall accuracy using the simulation map
        let wall_accuracy = compute_wall_accuracy(detected_lines, map);

        Self {
            position_error,
            orientation_error,
            wall_accuracy,
            detected_walls,
            convergence,
        }
    }
}

/// Compute angle difference normalized to [-PI, PI].
fn angle_diff(a: f32, b: f32) -> f32 {
    let diff = a - b;
    if diff > std::f32::consts::PI {
        diff - 2.0 * std::f32::consts::PI
    } else if diff < -std::f32::consts::PI {
        diff + 2.0 * std::f32::consts::PI
    } else {
        diff
    }
}

/// Maximum distance (meters) from line point to wall to count as valid.
const WALL_DISTANCE_THRESHOLD: f32 = 0.15; // 15cm

/// Number of sample points along each line.
const SAMPLES_PER_LINE: usize = 20;

/// Compute wall accuracy by checking if detected lines lie on occupied pixels.
///
/// Returns accuracy: fraction of sample points that are within threshold of a wall.
fn compute_wall_accuracy(detected: &[Line2D], map: &SimulationMap) -> f32 {
    if detected.is_empty() {
        return 0.0;
    }

    let mut near_wall_count = 0;
    let mut total_samples = 0;

    let resolution = map.resolution();

    for line in detected {
        // Sample points along the line
        for i in 0..SAMPLES_PER_LINE {
            let t = i as f32 / (SAMPLES_PER_LINE - 1) as f32;
            let x = line.start.x + t * (line.end.x - line.start.x);
            let y = line.start.y + t * (line.end.y - line.start.y);

            // Find distance to nearest occupied cell
            let dist = distance_to_nearest_wall(x, y, map, resolution);
            total_samples += 1;

            if dist <= WALL_DISTANCE_THRESHOLD {
                near_wall_count += 1;
            }
        }
    }

    near_wall_count as f32 / total_samples as f32
}

/// Find distance from point to nearest occupied cell in map.
///
/// Searches in a spiral pattern outward from the point.
fn distance_to_nearest_wall(x: f32, y: f32, map: &SimulationMap, resolution: f32) -> f32 {
    // Check if point itself is on a wall
    if map.is_occupied(x, y) {
        return 0.0;
    }

    // Search outward in a grid pattern
    let max_search = (WALL_DISTANCE_THRESHOLD * 2.0 / resolution) as i32;

    for radius in 1..=max_search {
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                // Only check cells at this radius (perimeter)
                if dx.abs() != radius && dy.abs() != radius {
                    continue;
                }

                let check_x = x + dx as f32 * resolution;
                let check_y = y + dy as f32 * resolution;

                if map.is_occupied(check_x, check_y) {
                    let dist = ((check_x - x).powi(2) + (check_y - y).powi(2)).sqrt();
                    return dist;
                }
            }
        }
    }

    // No wall found within search radius
    WALL_DISTANCE_THRESHOLD * 2.0
}
