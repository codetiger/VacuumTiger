//! Core test harness for integration tests
//!
//! Provides synchronous simulation using sangam-io's mock device components
//! directly (without spawning daemon threads).

use sangam_io::devices::mock::config::{LidarConfig, RobotConfig};
use sangam_io::devices::mock::lidar_sim::LidarSimulator;
use sangam_io::devices::mock::map_loader::SimulationMap;
use sangam_io::devices::mock::noise::NoiseGenerator;
use sangam_io::devices::mock::physics::PhysicsState;

use vastu_map::core::Pose2D;
use vastu_map::{Map, VectorMap, VectorMapConfig};

use crate::adapters::lidar_to_point_cloud;
use crate::metrics::{ConvergenceStats, TestMetrics};
use crate::path_generator::PathSegment;
use crate::visualization::Visualizer;

/// Test harness configuration.
#[derive(Clone)]
pub struct HarnessConfig {
    /// Path to map YAML file
    pub map_file: String,
    /// Starting X position in meters
    pub start_x: f32,
    /// Starting Y position in meters
    pub start_y: f32,
    /// Starting orientation in radians
    pub start_theta: f32,
    /// Robot configuration
    pub robot: RobotConfig,
    /// Lidar configuration
    pub lidar: LidarConfig,
    /// SLAM configuration
    pub slam: VectorMapConfig,
    /// Random seed (0 = random)
    pub random_seed: u64,
    /// Minimum lidar quality to include
    pub min_quality: u8,
    /// Minimum lidar range in meters
    pub min_range: f32,
    /// Maximum lidar range in meters
    pub max_range: f32,
    /// Simulation timestep in seconds
    pub dt: f32,
    /// Lidar scan interval (in simulation steps)
    pub lidar_interval: usize,
    /// Fast mode: skip intermediate physics, run only at lidar rate (~22x speedup)
    pub fast_mode: bool,
}

impl Default for HarnessConfig {
    fn default() -> Self {
        // Use lidar config with zero offsets for test accuracy
        // The defaults match the physical robot but cause systematic errors:
        // - angle_offset=0.2182 rad (~12.5°) rotates the scan
        // - mounting_x=-0.110m shifts scan origin 11cm behind robot center
        // For testing we want the lidar at robot center with no angular offset.
        let mut lidar = LidarConfig::default();
        lidar.angle_offset = 0.0;
        lidar.mounting_x = 0.0;
        lidar.mounting_y = 0.0;
        lidar.optical_offset = 0.0;

        Self {
            map_file: String::new(),
            start_x: 2.5,
            start_y: 2.5,
            start_theta: 0.0,
            robot: RobotConfig::default(),
            lidar,
            slam: VectorMapConfig::default(),
            random_seed: 42, // Fixed seed for reproducibility
            min_quality: 50,
            min_range: 0.15,
            max_range: 8.0,
            dt: 1.0 / 110.0,    // 110Hz simulation
            lidar_interval: 22, // ~5Hz lidar (110/22 = 5)
            fast_mode: true,    // Skip physics substeps for faster tests
        }
    }
}

impl HarnessConfig {
    /// Create config for the simple_room map.
    pub fn simple_room() -> Self {
        // Use env!() to get project root at compile time
        let map_file = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../sangam-io/maps/simple_room.yaml"
        );
        Self {
            map_file: map_file.to_string(),
            start_x: 2.5,
            start_y: 2.5,
            ..Default::default()
        }
    }

    /// Create config for the large_room_obstacles map.
    pub fn large_room_obstacles() -> Self {
        let map_file = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../sangam-io/maps/large_room_obstacles.yaml"
        );
        Self {
            map_file: map_file.to_string(),
            start_x: 5.0,
            start_y: 5.0,
            ..Default::default()
        }
    }
}

/// Result of running a test scenario.
#[derive(Debug)]
pub struct TestResult {
    /// Computed metrics
    pub metrics: TestMetrics,
    /// Final SLAM pose estimate
    pub final_pose: Pose2D,
    /// Ground truth pose from physics
    pub ground_truth_pose: Pose2D,
    /// Number of observations processed
    pub observations: usize,
    /// Total simulation time in seconds
    pub sim_time: f32,
}

/// Interval for storing lidar scans in trajectory history.
const SCAN_STORAGE_INTERVAL: usize = 5;

/// Per-observation record for trajectory visualization.
#[derive(Clone, Debug)]
pub struct ObservationRecord {
    /// Ground truth pose from physics.
    pub truth_pose: Pose2D,
    /// Estimated pose from SLAM.
    pub slam_pose: Pose2D,
    /// Match confidence (0.0-1.0).
    pub confidence: f32,
    /// Drift magnitude (translation error in meters).
    pub drift_translation: f32,
    /// Lidar scan at this observation (robot frame) - stored at intervals for visualization.
    pub scan: Option<vastu_map::core::PointCloud2D>,
}

/// Trajectory history for visualization.
#[derive(Clone, Debug, Default)]
pub struct TrajectoryHistory {
    /// All observation records in chronological order.
    pub observations: Vec<ObservationRecord>,
}

impl TrajectoryHistory {
    /// Create a new empty trajectory history.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record an observation.
    pub fn record(
        &mut self,
        truth_pose: Pose2D,
        slam_pose: Pose2D,
        confidence: f32,
        scan: Option<vastu_map::core::PointCloud2D>,
    ) {
        let drift_translation =
            ((truth_pose.x - slam_pose.x).powi(2) + (truth_pose.y - slam_pose.y).powi(2)).sqrt();

        self.observations.push(ObservationRecord {
            truth_pose,
            slam_pose,
            confidence,
            drift_translation,
            scan,
        });
    }
}

/// Main test harness for running integration tests.
pub struct TestHarness {
    config: HarnessConfig,
    map: SimulationMap,
    physics: PhysicsState,
    lidar_sim: LidarSimulator,
    slam: VectorMap,
    visualizer: Option<Visualizer>,
    sim_time: f32,
    tick_count: usize,
    observations: usize,
    last_slam_pose: Pose2D,
    /// Ground truth pose from last scan (for computing odometry)
    last_physics_pose: Pose2D,
    /// Last lidar scan in robot frame (for visualization)
    last_scan_robot_frame: Option<vastu_map::core::PointCloud2D>,
    /// ICP convergence statistics across all observations
    convergence_stats: ConvergenceStats,
    /// Trajectory history for enhanced visualization
    trajectory: TrajectoryHistory,
}

impl TestHarness {
    /// Create a new test harness.
    pub fn new(config: HarnessConfig) -> Result<Self, Box<dyn std::error::Error>> {
        // Load map
        let map = SimulationMap::load(&config.map_file)?;

        // Initialize physics at start pose
        let physics = PhysicsState::new(
            config.start_x,
            config.start_y,
            config.start_theta,
            &config.robot,
        );

        // Create noise generator with fixed seed
        let noise = NoiseGenerator::new(config.random_seed);

        // Create lidar simulator
        let lidar_sim = LidarSimulator::new(&config.lidar, noise);

        // Create SLAM map
        let slam = VectorMap::new(config.slam.clone());

        let start_pose = Pose2D::new(config.start_x, config.start_y, config.start_theta);

        Ok(Self {
            config,
            map,
            physics,
            lidar_sim,
            slam,
            visualizer: None,
            sim_time: 0.0,
            tick_count: 0,
            observations: 0,
            last_slam_pose: start_pose,
            last_physics_pose: start_pose,
            last_scan_robot_frame: None,
            convergence_stats: ConvergenceStats::new(),
            trajectory: TrajectoryHistory::new(),
        })
    }

    /// Enable visualization output.
    pub fn with_visualization(mut self, output_path: &str) -> Self {
        self.visualizer = Some(Visualizer::new(output_path));
        self
    }

    /// Run simulation along a path.
    pub fn run_path(&mut self, segments: &[PathSegment]) -> TestResult {
        for segment in segments {
            self.run_segment(segment);
        }

        self.finalize()
    }

    /// Run a single path segment.
    fn run_segment(&mut self, segment: &PathSegment) {
        if self.config.fast_mode {
            // Fast mode: skip intermediate physics, run only at lidar rate (~22x speedup)
            // Each step covers the time of one lidar interval
            let lidar_dt = self.config.dt * self.config.lidar_interval as f32;
            let lidar_steps = (segment.duration / lidar_dt).max(1.0) as usize;

            for _ in 0..lidar_steps {
                // Single physics update covering the full lidar interval
                self.physics.update(
                    lidar_dt,
                    segment.linear_vel,
                    segment.angular_vel,
                    &self.map,
                    &self.config.robot,
                );

                self.sim_time += lidar_dt;
                self.process_lidar_scan();
            }
        } else {
            // Normal mode: full physics simulation at configured rate
            let steps = (segment.duration / self.config.dt) as usize;

            for _ in 0..steps {
                // Update physics
                self.physics.update(
                    self.config.dt,
                    segment.linear_vel,
                    segment.angular_vel,
                    &self.map,
                    &self.config.robot,
                );

                self.tick_count += 1;
                self.sim_time += self.config.dt;

                // Generate lidar scan at configured interval
                if self.tick_count >= self.config.lidar_interval {
                    self.tick_count = 0;
                    self.process_lidar_scan();
                }
            }
        }
    }

    /// Process a lidar scan and feed to SLAM.
    fn process_lidar_scan(&mut self) {
        // Generate lidar scan
        let scan_data = self.lidar_sim.generate_scan(
            &self.map,
            self.physics.x(),
            self.physics.y(),
            self.physics.theta(),
        );

        // Convert to point cloud
        let cloud = lidar_to_point_cloud(
            scan_data,
            self.config.min_quality,
            self.config.min_range,
            self.config.max_range,
        );

        // Skip if scan is too sparse
        if cloud.len() < 10 {
            return;
        }

        // Store the robot-frame scan for visualization
        self.last_scan_robot_frame = Some(cloud.clone());

        // Compute relative odometry from ground truth (simplified)
        // In real tests, this would come from encoder simulation
        let current_truth = Pose2D::new(self.physics.x(), self.physics.y(), self.physics.theta());

        let odometry = if self.observations == 0 {
            // First observation - use absolute pose as initial guess
            current_truth
        } else {
            // Subsequent observations - compute relative pose
            self.compute_relative_odometry(current_truth)
        };

        // Feed to SLAM
        let result = self.slam.observe(&cloud, odometry);
        self.last_slam_pose = result.pose;
        self.last_physics_pose = current_truth; // Update for next odometry computation
        self.observations += 1;

        // Record ICP convergence stats
        self.convergence_stats
            .record(result.icp_iterations, result.icp_converged);

        // Record to trajectory history for visualization
        // Store scan at intervals to avoid excessive memory usage
        let scan_to_store = if self.observations % SCAN_STORAGE_INTERVAL == 0 {
            Some(cloud.clone())
        } else {
            None
        };

        self.trajectory
            .record(current_truth, result.pose, result.confidence, scan_to_store);

        log::debug!(
            "Observation {}: truth=({:.2}, {:.2}, {:.1}°), slam=({:.2}, {:.2}, {:.1}°), conf={:.2}, icp_iters={}",
            self.observations,
            current_truth.x,
            current_truth.y,
            current_truth.theta.to_degrees(),
            result.pose.x,
            result.pose.y,
            result.pose.theta.to_degrees(),
            result.confidence,
            result.icp_iterations
        );
    }

    /// Compute relative odometry between consecutive ground truth poses.
    ///
    /// Uses physics poses (not SLAM poses) to mimic real-world encoders that
    /// measure actual wheel motion independent of SLAM corrections.
    fn compute_relative_odometry(&self, current_truth: Pose2D) -> Pose2D {
        // Compute delta from previous physics pose (like real encoders)
        let dx = current_truth.x - self.last_physics_pose.x;
        let dy = current_truth.y - self.last_physics_pose.y;
        let dtheta = normalize_angle(current_truth.theta - self.last_physics_pose.theta);

        // Transform to robot frame using previous physics pose orientation
        let cos_th = self.last_physics_pose.theta.cos();
        let sin_th = self.last_physics_pose.theta.sin();
        let dx_robot = dx * cos_th + dy * sin_th;
        let dy_robot = -dx * sin_th + dy * cos_th;

        Pose2D::new(dx_robot, dy_robot, dtheta)
    }

    /// Finalize the test and compute metrics.
    fn finalize(&mut self) -> TestResult {
        let final_pose = self.slam.current_pose();
        let ground_truth_pose =
            Pose2D::new(self.physics.x(), self.physics.y(), self.physics.theta());

        // Compute metrics using the simulation map, including convergence stats
        let metrics = TestMetrics::compute(
            &self.slam,
            &self.map,
            final_pose,
            ground_truth_pose,
            self.convergence_stats.clone(),
        );

        // Generate enhanced visualization if enabled
        if let Some(ref viz) = self.visualizer {
            viz.render_full(
                &self.slam,
                &self.map,
                final_pose,
                ground_truth_pose,
                self.last_scan_robot_frame.as_ref(),
                &self.trajectory,
                &metrics,
            );
        }

        TestResult {
            metrics,
            final_pose,
            ground_truth_pose,
            observations: self.observations,
            sim_time: self.sim_time,
        }
    }

    /// Get current SLAM map reference.
    pub fn slam(&self) -> &VectorMap {
        &self.slam
    }

    /// Get current physics state.
    pub fn physics(&self) -> &PhysicsState {
        &self.physics
    }
}

/// Normalize angle to [-PI, PI].
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle;
    while a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    }
    while a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}
