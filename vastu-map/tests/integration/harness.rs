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
use crate::metrics::TestMetrics;
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
}

impl Default for HarnessConfig {
    fn default() -> Self {
        Self {
            map_file: String::new(),
            start_x: 2.5,
            start_y: 2.5,
            start_theta: 0.0,
            robot: RobotConfig::default(),
            lidar: LidarConfig::default(),
            slam: VectorMapConfig::default(),
            random_seed: 42, // Fixed seed for reproducibility
            min_quality: 50,
            min_range: 0.15,
            max_range: 8.0,
            dt: 1.0 / 110.0,    // 110Hz simulation
            lidar_interval: 22, // ~5Hz lidar (110/22 = 5)
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
        self.observations += 1;

        log::debug!(
            "Observation {}: truth=({:.2}, {:.2}, {:.1}°), slam=({:.2}, {:.2}, {:.1}°), conf={:.2}",
            self.observations,
            current_truth.x,
            current_truth.y,
            current_truth.theta.to_degrees(),
            result.pose.x,
            result.pose.y,
            result.pose.theta.to_degrees(),
            result.confidence
        );
    }

    /// Compute relative odometry between last SLAM pose and current truth.
    fn compute_relative_odometry(&self, current_truth: Pose2D) -> Pose2D {
        // For simplicity, use a small delta pose based on expected motion
        // This simulates imperfect odometry
        let dx = current_truth.x - self.last_slam_pose.x;
        let dy = current_truth.y - self.last_slam_pose.y;
        let dtheta = normalize_angle(current_truth.theta - self.last_slam_pose.theta);

        // Transform to robot frame
        let cos_th = self.last_slam_pose.theta.cos();
        let sin_th = self.last_slam_pose.theta.sin();
        let dx_robot = dx * cos_th + dy * sin_th;
        let dy_robot = -dx * sin_th + dy * cos_th;

        Pose2D::new(dx_robot, dy_robot, dtheta)
    }

    /// Finalize the test and compute metrics.
    fn finalize(&mut self) -> TestResult {
        let final_pose = self.slam.current_pose();
        let ground_truth_pose =
            Pose2D::new(self.physics.x(), self.physics.y(), self.physics.theta());

        // Compute metrics using the simulation map
        let metrics = TestMetrics::compute(&self.slam, &self.map, final_pose, ground_truth_pose);

        // Generate visualization if enabled
        if let Some(ref viz) = self.visualizer {
            viz.render(&self.slam, &self.map, final_pose, ground_truth_pose);
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
