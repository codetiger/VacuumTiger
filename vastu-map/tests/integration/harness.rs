//! Core test harness for integration tests
//!
//! Provides synchronous simulation using sangam-io's mock device components
//! directly (without spawning daemon threads).

use sangam_io::devices::mock::config::{LidarConfig, RobotConfig};
use sangam_io::devices::mock::lidar_sim::LidarSimulator;
use sangam_io::devices::mock::map_loader::SimulationMap;
use sangam_io::devices::mock::noise::NoiseGenerator;
use sangam_io::devices::mock::physics::PhysicsState;

use vastu_map::config::ExplorationConfig;
use vastu_map::core::Pose2D;
use vastu_map::core::math::normalize_angle;
use vastu_map::integration::{ScanStore, ScanStoreConfig};
use vastu_map::odometry::WheelOdometry;
use vastu_map::query::PathPlanningConfig;
use vastu_map::{Map, VectorMap, VectorMapConfig};

use crate::adapters::lidar_to_point_cloud;
use crate::metrics::{ConvergenceStats, TestMetrics, TimingStats};
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

    /// Create config for the medium_room map (8x8m with obstacles).
    ///
    /// Uses non-conservative path planning to allow paths to frontiers
    /// (which are at map edges and may be in "unknown" space).
    pub fn medium_room() -> Self {
        let map_file = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/integration/scenarios/maps/medium_room.yaml"
        );
        // Use non-conservative path planning for exploration
        let slam = VectorMapConfig::default()
            .with_path_planning(PathPlanningConfig::default().with_conservative(false));
        Self {
            map_file: map_file.to_string(),
            start_x: 4.0,
            start_y: 4.0,
            slam,
            ..Default::default()
        }
    }

    /// Create config for the large_room map.
    pub fn large_room() -> Self {
        let map_file = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/integration/scenarios/maps/large_room.yaml"
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
    /// Algorithm timing statistics across all observations
    timing_stats: TimingStats,
    /// Wheel odometry tracker using vastu-map's odometry module
    wheel_odometry: WheelOdometry,
    /// Simulated left encoder ticks (accumulated)
    left_encoder_ticks: i32,
    /// Simulated right encoder ticks (accumulated)
    right_encoder_ticks: i32,
    /// Scan storage for visualization (stores ALL scans)
    scan_store: ScanStore,
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

        // Create wheel odometry tracker with robot configuration
        let wheel_odometry =
            WheelOdometry::new(config.robot.wheel_base, config.robot.ticks_per_meter);

        // Create scan store for visualization (stores all scans)
        let scan_store = ScanStore::new(ScanStoreConfig::default());

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
            timing_stats: TimingStats::new(),
            wheel_odometry,
            left_encoder_ticks: 0,
            right_encoder_ticks: 0,
            scan_store,
        })
    }

    /// Enable visualization output.
    pub fn with_visualization(mut self, output_path: &str) -> Self {
        self.visualizer = Some(Visualizer::new(output_path));
        self
    }

    /// Enable exploration mode on the SLAM map.
    ///
    /// In exploration mode, VectorMap stores raw scan data and periodically
    /// re-fits lines from accumulated point clouds, eliminating first-scan bias.
    ///
    /// # Arguments
    /// * `config` - Exploration configuration (uses default if None)
    ///
    /// # Example
    /// ```rust,ignore
    /// let harness = TestHarness::new(HarnessConfig::simple_room())?
    ///     .with_exploration_mode(None);  // Use default config
    /// ```
    pub fn with_exploration_mode(mut self, config: Option<ExplorationConfig>) -> Self {
        let exploration_config = config.unwrap_or_default();
        self.slam.enable_exploration_mode(exploration_config);
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
                // Simulate encoder ticks from wheel velocities
                self.simulate_encoder_ticks(lidar_dt, segment.linear_vel, segment.angular_vel);

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
                // Simulate encoder ticks from wheel velocities
                self.simulate_encoder_ticks(
                    self.config.dt,
                    segment.linear_vel,
                    segment.angular_vel,
                );

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

        // Get ground truth pose for metrics/visualization
        let current_truth = Pose2D::new(self.physics.x(), self.physics.y(), self.physics.theta());

        // Compute relative odometry from ground truth physics poses
        // This mimics perfect wheel encoders that measure actual motion.
        // Note: We also update WheelOdometry for cumulative tracking, but use
        // ground-truth-derived odometry for SLAM to avoid commanded vs actual mismatch.
        let odometry = if self.observations == 0 {
            // First observation - use absolute pose as initial guess
            let _ = self
                .wheel_odometry
                .update(self.left_encoder_ticks, self.right_encoder_ticks);
            current_truth
        } else {
            // Update wheel odometry for tracking (though we use ground truth for SLAM)
            let _ = self
                .wheel_odometry
                .update(self.left_encoder_ticks, self.right_encoder_ticks);
            // Compute relative pose from physics (like perfect encoders)
            self.compute_relative_odometry(current_truth)
        };

        // Feed to SLAM
        let result = self.slam.observe(&cloud, odometry);
        self.last_slam_pose = result.pose;
        self.last_physics_pose = current_truth; // Update for metrics computation
        self.observations += 1;

        // Store scan in scan store for visualization (stores ALL scans)
        self.scan_store.add_scan(
            odometry,
            result.pose,
            &cloud,
            result.features_extracted,
            result.confidence,
        );

        // Record ICP convergence stats
        self.convergence_stats
            .record(result.icp_iterations, result.icp_converged);

        // Record timing stats
        self.timing_stats.record(&result.timing);

        // Record to trajectory history for visualization
        // Note: We no longer store scans here since ScanStore handles it
        self.trajectory
            .record(current_truth, result.pose, result.confidence, None);

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

    /// Simulate encoder ticks from commanded wheel velocities.
    ///
    /// Computes individual wheel velocities from linear/angular velocity commands
    /// and integrates them over the timestep to produce encoder tick counts.
    /// This mimics how real wheel encoders would accumulate ticks.
    fn simulate_encoder_ticks(&mut self, dt: f32, linear_vel: f32, angular_vel: f32) {
        // Compute individual wheel velocities from (v, ω) commands
        // Using differential drive kinematics:
        //   v_left = v - ω * wheel_base / 2
        //   v_right = v + ω * wheel_base / 2
        let (left_vel, right_vel) =
            self.physics
                .wheel_velocities(linear_vel, angular_vel, self.config.robot.wheel_base);

        // Integrate wheel velocities to get distance traveled per wheel
        let left_distance = left_vel * dt;
        let right_distance = right_vel * dt;

        // Convert distance to encoder ticks
        let left_delta_ticks = (left_distance * self.config.robot.ticks_per_meter) as i32;
        let right_delta_ticks = (right_distance * self.config.robot.ticks_per_meter) as i32;

        // Accumulate encoder ticks
        self.left_encoder_ticks += left_delta_ticks;
        self.right_encoder_ticks += right_delta_ticks;
    }

    /// Finalize the test and compute metrics.
    ///
    /// This generates the visualization SVG if enabled.
    pub fn finalize(&mut self) -> TestResult {
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
                &self.scan_store,
                &self.trajectory,
                &metrics,
            );
        }

        // Print timing summary
        self.timing_stats.print_summary();

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

    /// Get mutable reference to SLAM map (for adding virtual walls).
    pub fn slam_mut(&mut self) -> &mut VectorMap {
        &mut self.slam
    }

    /// Get current physics state.
    pub fn physics(&self) -> &PhysicsState {
        &self.physics
    }

    /// Get current SLAM pose estimate.
    pub fn slam_pose(&self) -> Pose2D {
        self.last_slam_pose
    }

    /// Get current physics (ground truth) pose.
    pub fn physics_pose(&self) -> Pose2D {
        Pose2D::new(self.physics.x(), self.physics.y(), self.physics.theta())
    }

    /// Get configuration.
    pub fn config(&self) -> &HarnessConfig {
        &self.config
    }

    /// Get simulation map (for collision checking).
    pub fn simulation_map(&self) -> &SimulationMap {
        &self.map
    }

    /// Run a single simulation step and return collision status.
    ///
    /// This is used for interactive exploration tests where the controller
    /// needs to react to sensor feedback at each step.
    ///
    /// # Arguments
    /// * `linear_vel` - Linear velocity command (m/s)
    /// * `angular_vel` - Angular velocity command (rad/s)
    ///
    /// # Returns
    /// StepResult containing collision status and updated poses.
    pub fn step(&mut self, linear_vel: f32, angular_vel: f32) -> StepResult {
        let dt = if self.config.fast_mode {
            self.config.dt * self.config.lidar_interval as f32
        } else {
            self.config.dt
        };

        // Simulate encoder ticks
        self.simulate_encoder_ticks(dt, linear_vel, angular_vel);

        // Update physics and get collision status
        let collided =
            self.physics
                .update(dt, linear_vel, angular_vel, &self.map, &self.config.robot);

        self.sim_time += dt;
        self.tick_count += 1;

        // Check bumper sensors using the physics simulation
        let (left_bumper, right_bumper) = self.check_bumpers();

        StepResult {
            collided,
            left_bumper,
            right_bumper,
            physics_pose: Pose2D::new(self.physics.x(), self.physics.y(), self.physics.theta()),
            slam_pose: self.last_slam_pose,
            sim_time: self.sim_time,
        }
    }

    /// Check bumper sensors based on current position.
    ///
    /// Uses simple distance-based collision detection against the map.
    fn check_bumpers(&self) -> (bool, bool) {
        use std::f32::consts::FRAC_PI_4;

        let x = self.physics.x();
        let y = self.physics.y();
        let theta = self.physics.theta();
        let radius = self.config.robot.robot_radius;

        // Check points on left and right front bumper zones
        let check_distance = radius + 0.02; // Slightly past robot surface

        // Left bumper zone (front-left quadrant)
        let left_angle = theta + FRAC_PI_4;
        let left_x = x + check_distance * left_angle.cos();
        let left_y = y + check_distance * left_angle.sin();
        let left_triggered = self.map.is_occupied(left_x, left_y);

        // Right bumper zone (front-right quadrant)
        let right_angle = theta - FRAC_PI_4;
        let right_x = x + check_distance * right_angle.cos();
        let right_y = y + check_distance * right_angle.sin();
        let right_triggered = self.map.is_occupied(right_x, right_y);

        // Also check center front
        let center_x = x + check_distance * theta.cos();
        let center_y = y + check_distance * theta.sin();
        let center_triggered = self.map.is_occupied(center_x, center_y);

        (
            left_triggered || center_triggered,
            right_triggered || center_triggered,
        )
    }

    /// Process a lidar scan and feed to SLAM (public version).
    ///
    /// Call this at lidar intervals during exploration.
    pub fn process_lidar(&mut self) {
        self.process_lidar_scan();
    }

    /// Check if it's time for a lidar scan (based on tick count).
    pub fn should_scan(&self) -> bool {
        self.tick_count >= self.config.lidar_interval
    }

    /// Get the lidar scan interval.
    pub fn lidar_interval(&self) -> usize {
        self.config.lidar_interval
    }

    /// Get current simulation time.
    pub fn sim_time(&self) -> f32 {
        self.sim_time
    }

    /// Get number of observations processed.
    pub fn observation_count(&self) -> usize {
        self.observations
    }

    /// Get scan store for visualization.
    pub fn scan_store(&self) -> &ScanStore {
        &self.scan_store
    }
}

/// Result of a single simulation step.
#[derive(Clone, Debug)]
pub struct StepResult {
    /// Whether a collision occurred during this step.
    pub collided: bool,
    /// Whether the left bumper is triggered.
    pub left_bumper: bool,
    /// Whether the right bumper is triggered.
    pub right_bumper: bool,
    /// Current physics (ground truth) pose.
    pub physics_pose: Pose2D,
    /// Current SLAM pose estimate.
    pub slam_pose: Pose2D,
    /// Current simulation time.
    pub sim_time: f32,
}

impl StepResult {
    /// Check if any bumper is triggered.
    pub fn any_bumper(&self) -> bool {
        self.left_bumper || self.right_bumper
    }
}
