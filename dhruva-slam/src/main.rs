//! DhruvaSLAM - Modular SLAM implementation for robotic vacuum cleaners
//!
//! Production-optimized SLAM daemon using:
//! - **Matcher**: Configurable scan matching (MultiRes, ICP, Correlative, Hybrid)
//! - **Odometry**: Configurable fusion (Mahony, ESKF, Complementary, Wheel-only)
//! - **Maps**: Dual representation (Occupancy Grid + Feature-based)
//! - **Loop Closure**: LiDAR-IRIS descriptor + Sparse CG optimization
//!
//! # Architecture
//!
//! The crate is organized into 5 logical layers:
//!
//! ```text
//! ┌─────────────────────────────────────────────────────┐
//! │                      main                           │  ← Entry point
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                      io/                            │  ← Infrastructure
//! │         (sangam_client, bag, streaming)             │
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                    engine/                          │  ← Orchestration
//! │              (slam, graph optimization)             │
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                  algorithms/                        │  ← Core algorithms
//! │         (matching, mapping, localization)           │
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                   sensors/                          │  ← Sensor processing
//! │            (odometry, preprocessing)                │
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                     core/                           │  ← Foundation
//! │                (types, math)                        │
//! └─────────────────────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```bash
//! # With default config
//! cargo run --release
//!
//! # With custom config file
//! cargo run --release -- --config dhruva-slam.toml
//!
//! # With command line overrides
//! cargo run --release -- --sangam 192.168.68.101:5555 --port 5557
//! ```

// Layer 1: Core foundation (no internal deps)
mod core;

// Layer 2: Sensor processing (depends on core)
mod sensors;

// Layer 3: Algorithms (depends on core, sensors)
mod algorithms;

// Layer 4: SLAM engine (depends on core, sensors, algorithms)
mod engine;

// Layer 5: I/O infrastructure (depends on all layers)
mod io;

// Layer 6: Multi-threaded state management
mod state;

// Layer 7: Thread infrastructure
mod threads;

// Layer 8: Autonomous exploration
mod exploration;

// Layer 9: Navigation (path planning and execution)
mod navigation;

// ============================================================================
// Imports
// ============================================================================

use std::fs;
use std::io::Write;
use std::path::Path;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use serde::Deserialize;

use crate::algorithms::descriptors::LidarIrisConfig;
use crate::algorithms::mapping::{MapIntegratorConfig, OccupancyGridConfig};
use crate::algorithms::matching::{
    CorrelativeConfig, DynMatcherConfig, IcpConfig, MatcherType, MultiResolutionConfig,
    PointToLineIcpConfig, RobustKernel,
};
use crate::algorithms::planning::AStarConfig;
use crate::engine::graph::{GraphOptimizerConfig, LoopDetectorConfig};
use crate::engine::slam::{
    KeyframeManagerConfig, LoopClosureConfig as SlamLoopClosureConfig, OnlineSlamConfig,
    SubmapManagerConfig,
};
use crate::exploration::FrontierConfig;
use crate::io::map_manager::MapManager;
use crate::io::motion_controller::{MotionConfig, create_shared_motion_controller};
use crate::navigation::NavigatorConfig;
use crate::sensors::odometry::{
    DynOdometryConfig, MahonyConfig, OdometryType, WheelOdometryConfig,
};
use crate::sensors::preprocessing::{
    AngularDownsamplerConfig, LidarOffset, OutlierFilterConfig, PreprocessorConfig,
    RangeFilterConfig,
};
use crate::state::{create_command_channel, create_shared_state};
use crate::threads::{
    ExplorationConfig, ExplorationThread, NavigationThread, NavigationThreadConfig,
    PublisherThread, SlamThread, SlamThreadConfig, StreamConfig as ThreadStreamConfig,
    create_nav_channel,
};

// ============================================================================
// Configuration
// ============================================================================

#[derive(Debug, Deserialize, Default)]
struct Config {
    #[serde(default)]
    source: SourceConfig,
    #[serde(default)]
    output: OutputConfig,
    #[serde(default)]
    stream: StreamConfig,
    #[serde(default)]
    map_storage: MapStorageConfig,
    #[serde(default)]
    bag: BagConfig,
    #[serde(default)]
    preprocessing: PreprocessingConfig,
    #[serde(default)]
    slam: SlamConfig,
    #[serde(default)]
    lidar: LidarConfig,
    #[serde(default)]
    odometry: OdometryConfig,
    #[serde(default)]
    matcher: MatcherConfig,
    #[serde(default)]
    keyframe: KeyframeConfig,
    #[serde(default)]
    submap: SubmapConfig,
    #[serde(default)]
    map: MapConfig,
    #[serde(default)]
    loop_closure: LoopClosureConfig,
    #[serde(default)]
    optimization: OptimizationConfig,
    #[serde(default)]
    exploration: ExplorationCfg,
    #[serde(default)]
    navigation: NavigationConfig,
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct SourceConfig {
    /// SangamIO address for TCP commands and UDP sensor data
    /// Format: "host:port" (e.g., "192.168.68.101:5555")
    /// Both TCP and UDP use the same port for simpler configuration.
    sangam_address: String,
    /// UDP port override for sensor streaming (optional).
    /// If specified, UDP receiver will bind to this port instead of the TCP port.
    /// Useful for localhost testing where TCP and UDP need different ports.
    udp_port: Option<u16>,
}

impl Default for SourceConfig {
    fn default() -> Self {
        Self {
            sangam_address: "192.168.68.101:5555".to_string(),
            udp_port: None,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct OutputConfig {
    bind_port: u16,
    odometry_rate_hz: f32,
    map_rate_hz: f32,
    feature_rate_hz: f32,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            bind_port: 5557,
            odometry_rate_hz: 50.0,
            map_rate_hz: 1.0,
            feature_rate_hz: 0.2, // Every 5 seconds
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct StreamConfig {
    robot_status_hz: f32,
    sensor_status_hz: f32,
    map_hz: f32,
    navigation_hz: f32,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            robot_status_hz: 10.0,
            sensor_status_hz: 10.0,
            map_hz: 1.0,
            navigation_hz: 5.0,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct MapStorageConfig {
    path: String,
}

impl Default for MapStorageConfig {
    fn default() -> Self {
        Self {
            path: "/var/lib/dhruva/maps".to_string(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
#[derive(Default)]
struct BagConfig {
    /// Bag file path for offline playback (None for live mode)
    file: Option<String>,
    /// Loop bag file playback
    loop_playback: bool,
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct PreprocessingConfig {
    min_range: f32,
    max_range: f32,
    target_points: usize,
    min_angle_step: f32,
}

impl Default for PreprocessingConfig {
    fn default() -> Self {
        Self {
            min_range: 0.15,
            max_range: 8.0,
            target_points: 180,
            min_angle_step: 0.0175, // ~1 degree
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct SlamConfig {
    min_match_score: f32,
    lost_threshold: f32,
    min_scan_points: usize,
    encoder_weight: f32,
    use_submap_matching: bool,
    /// Maximum rotation deviation from odometry before rejecting scan match (radians)
    max_theta_deviation: f32,
    /// Maximum translation deviation from odometry before rejecting scan match (meters)
    max_translation_deviation: f32,
}

impl Default for SlamConfig {
    fn default() -> Self {
        Self {
            min_match_score: 0.3,
            lost_threshold: 0.1,
            min_scan_points: 50,
            encoder_weight: 0.8,
            use_submap_matching: false,
            max_theta_deviation: 0.35,      // ~20 degrees
            max_translation_deviation: 0.5, // 50cm
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct LidarConfig {
    mounting_x: f32,
    mounting_y: f32,
    optical_offset: f32,
    angle_offset: f32,
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            mounting_x: -0.0936,
            mounting_y: 0.0,
            optical_offset: 0.0258,
            angle_offset: 0.0,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct OdometryConfig {
    /// Odometry algorithm: "wheel", "complementary", "eskf", "mahony"
    algorithm: String,
    /// Encoder ticks per meter of wheel travel
    ticks_per_meter: f32,
    /// Distance between wheel centers in meters
    wheel_base: f32,
}

impl Default for OdometryConfig {
    fn default() -> Self {
        Self {
            algorithm: "mahony".to_string(),
            ticks_per_meter: 4464.0,
            wheel_base: 0.233,
        }
    }
}

impl OdometryConfig {
    fn algorithm_type(&self) -> OdometryType {
        match self.algorithm.to_lowercase().as_str() {
            "wheel" => OdometryType::Wheel,
            "complementary" => OdometryType::Complementary,
            "eskf" => OdometryType::Eskf,
            _ => OdometryType::Mahony,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct MatcherConfig {
    /// Matcher algorithm: "icp", "p2l", "correlative", "multi_res", "hybrid_icp", "hybrid_p2l"
    algorithm: String,
    /// Always run correlative first
    always_correlative: bool,
    /// Encoder weight for initial guess
    encoder_weight: f32,
    // Correlative search window
    search_window_x: f32,
    search_window_y: f32,
    search_window_theta: f32,
    linear_resolution: f32,
    angular_resolution: f32,
    grid_resolution: f32,
    correlative_min_score: f32,
    // ICP configuration
    icp_max_iterations: u32,
    icp_translation_epsilon: f32,
    icp_rotation_epsilon: f32,
    icp_max_correspondence_distance: f32,
    icp_min_correspondences: usize,
    icp_outlier_ratio: f32,
    icp_robust_kernel: String,
    icp_kernel_scale: f32,
}

impl Default for MatcherConfig {
    fn default() -> Self {
        Self {
            algorithm: "hybrid_p2l".to_string(),
            always_correlative: true,
            encoder_weight: 1.0,
            search_window_x: 0.3,
            search_window_y: 0.3,
            search_window_theta: 0.5,
            linear_resolution: 0.03,
            angular_resolution: 0.03,
            grid_resolution: 0.05,
            correlative_min_score: 0.4,
            icp_max_iterations: 50,
            icp_translation_epsilon: 0.001,
            icp_rotation_epsilon: 0.001,
            icp_max_correspondence_distance: 0.5,
            icp_min_correspondences: 10,
            icp_outlier_ratio: 0.1,
            icp_robust_kernel: "welsch".to_string(),
            icp_kernel_scale: 0.1,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct KeyframeConfig {
    min_translation: f32,
    min_rotation: f32,
    max_keyframes: usize,
    min_interval_ms: u64,
}

impl Default for KeyframeConfig {
    fn default() -> Self {
        Self {
            min_translation: 0.5,
            min_rotation: 0.5,
            max_keyframes: 1000,
            min_interval_ms: 500,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct SubmapConfig {
    scans_per_submap: u32,
    overlap_scans: u32,
    max_active_submaps: usize,
}

impl Default for SubmapConfig {
    fn default() -> Self {
        Self {
            scans_per_submap: 100,
            overlap_scans: 10,
            max_active_submaps: 2,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct MapConfig {
    resolution: f32,
    initial_width: f32,
    initial_height: f32,
    log_odds_occupied: f32,
    log_odds_free: f32,
    log_odds_max: f32,
    log_odds_min: f32,
    occupied_threshold: f32,
    free_threshold: f32,
}

impl Default for MapConfig {
    fn default() -> Self {
        Self {
            resolution: 0.02,
            initial_width: 20.0,
            initial_height: 20.0,
            log_odds_occupied: 0.9,
            log_odds_free: -0.7,
            log_odds_max: 50.0,
            log_odds_min: -50.0,
            occupied_threshold: 0.5,
            free_threshold: -0.5,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct LoopClosureConfig {
    enabled: bool,
    detection_interval: usize,
    min_keyframe_gap: usize,
    optimization_threshold: usize,
    max_candidates: usize,
    #[serde(default)]
    iris: IrisConfig,
    #[serde(default)]
    matching: LoopMatchingConfig,
}

impl Default for LoopClosureConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            detection_interval: 5,
            min_keyframe_gap: 20,
            optimization_threshold: 3,
            max_candidates: 5,
            iris: IrisConfig::default(),
            matching: LoopMatchingConfig::default(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct IrisConfig {
    num_rows: usize,
    num_cols: usize,
    max_range: f32,
    signature_bits: usize,
}

impl Default for IrisConfig {
    fn default() -> Self {
        Self {
            num_rows: 80,
            num_cols: 360,
            max_range: 8.0,
            signature_bits: 640,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct LoopMatchingConfig {
    max_hamming_distance: u32,
    min_icp_score: f32,
}

impl Default for LoopMatchingConfig {
    fn default() -> Self {
        Self {
            max_hamming_distance: 100,
            min_icp_score: 0.5,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct OptimizationConfig {
    solver: String,
    max_iterations: usize,
    tolerance: f64,
}

impl Default for OptimizationConfig {
    fn default() -> Self {
        Self {
            solver: "sparse_cg".to_string(),
            max_iterations: 50,
            tolerance: 1e-6,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct ExplorationCfg {
    /// Enable exploration thread.
    enabled: bool,
    /// Exploration strategy: "frontier".
    strategy: String,
    /// Loop rate for exploration thread (Hz).
    loop_rate_hz: f32,
    #[serde(default)]
    motion: ExplorationMotionConfig,
    #[serde(default)]
    frontier: ExplorationFrontierConfig,
}

impl Default for ExplorationCfg {
    fn default() -> Self {
        Self {
            enabled: true,
            strategy: "frontier".to_string(),
            loop_rate_hz: 10.0,
            motion: ExplorationMotionConfig::default(),
            frontier: ExplorationFrontierConfig::default(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct ExplorationMotionConfig {
    /// Maximum linear velocity (m/s).
    max_linear_vel: f32,
    /// Maximum angular velocity (rad/s).
    max_angular_vel: f32,
    /// Minimum distance from obstacles (m).
    obstacle_clearance: f32,
    /// Normal linear velocity (m/s).
    linear_vel: f32,
    /// Normal angular velocity for rotation (rad/s).
    angular_vel: f32,
}

impl Default for ExplorationMotionConfig {
    fn default() -> Self {
        Self {
            max_linear_vel: 0.3,
            max_angular_vel: 0.5,
            obstacle_clearance: 0.15,
            linear_vel: 0.2,
            angular_vel: 0.3,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct ExplorationFrontierConfig {
    /// Minimum frontier size to consider (cells).
    min_frontier_size: usize,
    /// Maximum detection range (m).
    detection_range: f32,
    /// Radius to block after bumper trigger (m).
    blocked_radius: f32,
    /// Clustering threshold for frontier grouping (m).
    clustering_threshold: f32,
}

impl Default for ExplorationFrontierConfig {
    fn default() -> Self {
        Self {
            min_frontier_size: 5,
            detection_range: 10.0,
            blocked_radius: 0.3,
            clustering_threshold: 0.5,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
struct NavigationConfig {
    /// Enable navigation thread.
    enabled: bool,
    /// Update rate for navigation thread (Hz).
    update_rate_hz: f32,
    /// Robot radius for A* inflation (m).
    robot_radius: f32,
    /// Maximum linear velocity (m/s).
    max_linear_vel: f32,
    /// Maximum angular velocity (rad/s).
    max_angular_vel: f32,
    /// Waypoint reached threshold (m).
    waypoint_threshold: f32,
    /// Goal position threshold (m).
    goal_threshold: f32,
    /// Heading threshold (rad).
    heading_threshold: f32,
    /// Treat unknown cells as obstacles for path planning.
    /// Set to false to allow navigation through unexplored areas.
    unknown_is_obstacle: bool,
}

impl Default for NavigationConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            update_rate_hz: 10.0,
            robot_radius: 0.18,
            max_linear_vel: 0.3,
            max_angular_vel: 0.5,
            waypoint_threshold: 0.15,
            goal_threshold: 0.08,
            heading_threshold: 0.15,
            unknown_is_obstacle: true,
        }
    }
}

// ============================================================================
// CLI Arguments
// ============================================================================

struct Args {
    config_path: Option<String>,
}

fn parse_args() -> Args {
    let args: Vec<String> = std::env::args().collect();
    let mut result = Args { config_path: None };

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--config" | "-c" => {
                if i + 1 < args.len() {
                    result.config_path = Some(args[i + 1].clone());
                    i += 1;
                }
            }
            "--help" | "-h" => {
                print_help();
                std::process::exit(0);
            }
            _ => {
                eprintln!("Unknown argument: {}", args[i]);
                print_help();
                std::process::exit(1);
            }
        }
        i += 1;
    }

    result
}

fn print_help() {
    println!("dhruva-slam - SLAM daemon for robotic vacuum cleaners");
    println!();
    println!("USAGE:");
    println!("    dhruva-slam [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    -c, --config <FILE>     Configuration file (default: dhruva-slam.toml)");
    println!("    -h, --help              Print help information");
    println!();
    println!("CONFIGURATION:");
    println!("    All settings are configured via the TOML config file:");
    println!("    - [source] sangam_address: SangamIO daemon address");
    println!("    - [output] bind_port: TCP stream port");
    println!("    - [bag] file, loop: Bag file playback settings");
    println!();
    println!("THREADS:");
    println!("    The daemon runs with 3 fixed threads:");
    println!("    - SLAM Thread: Sensor processing and scan matching");
    println!("    - Publisher Thread: Streams data to clients");
    println!("    - Command Thread: Handles control commands");
}

fn load_config(args: &Args) -> Config {
    match &args.config_path {
        Some(path) => match fs::read_to_string(path) {
            Ok(contents) => match basic_toml::from_str(&contents) {
                Ok(cfg) => {
                    log::info!("Loaded config from {}", path);
                    cfg
                }
                Err(e) => {
                    log::warn!("Failed to parse config {}: {}", path, e);
                    Config::default()
                }
            },
            Err(e) => {
                log::warn!("Failed to read config {}: {}", path, e);
                Config::default()
            }
        },
        None => {
            // Try default paths
            for path in &["dhruva-slam.toml", "/etc/dhruva-slam.toml"] {
                if let Ok(contents) = fs::read_to_string(path)
                    && let Ok(cfg) = basic_toml::from_str(&contents)
                {
                    log::info!("Loaded config from {}", path);
                    return cfg;
                }
            }
            Config::default()
        }
    }
}

/// Build OnlineSlamConfig from TOML configuration.
fn build_slam_config(config: &Config) -> OnlineSlamConfig {
    // Parse matcher type from config string
    let matcher_type = match config.matcher.algorithm.to_lowercase().as_str() {
        "icp" => MatcherType::Icp,
        "p2l" => MatcherType::P2l,
        "correlative" => MatcherType::Correlative,
        "multi_res" => MatcherType::MultiRes,
        "hybrid_icp" => MatcherType::HybridIcp,
        _ => MatcherType::HybridP2l, // default
    };

    // Build correlative config from matcher settings
    let correlative_config = CorrelativeConfig {
        search_window_x: config.matcher.search_window_x,
        search_window_y: config.matcher.search_window_y,
        search_window_theta: config.matcher.search_window_theta,
        linear_resolution: config.matcher.linear_resolution,
        angular_resolution: config.matcher.angular_resolution,
        grid_resolution: config.matcher.grid_resolution,
        min_score: config.matcher.correlative_min_score,
    };

    // Build P2P ICP config from matcher settings (with robust kernel)
    let robust_kernel = match config.matcher.icp_robust_kernel.to_lowercase().as_str() {
        "huber" => RobustKernel::Huber,
        "cauchy" => RobustKernel::Cauchy,
        "welsch" => RobustKernel::Welsch,
        _ => RobustKernel::None,
    };

    let icp_config = IcpConfig {
        max_iterations: config.matcher.icp_max_iterations,
        translation_epsilon: config.matcher.icp_translation_epsilon,
        rotation_epsilon: config.matcher.icp_rotation_epsilon,
        max_correspondence_distance: config.matcher.icp_max_correspondence_distance,
        min_correspondences: config.matcher.icp_min_correspondences,
        outlier_ratio: config.matcher.icp_outlier_ratio,
        robust_kernel,
        kernel_scale: config.matcher.icp_kernel_scale,
        ..Default::default()
    };

    // Build P2L ICP config from matcher settings
    // Note: P2L ICP doesn't support robust kernels - those are only in P2P ICP
    let p2l_config = PointToLineIcpConfig {
        max_iterations: config.matcher.icp_max_iterations,
        translation_epsilon: config.matcher.icp_translation_epsilon,
        rotation_epsilon: config.matcher.icp_rotation_epsilon,
        max_correspondence_distance: config.matcher.icp_max_correspondence_distance,
        min_correspondences: config.matcher.icp_min_correspondences,
        outlier_ratio: config.matcher.icp_outlier_ratio,
        ..Default::default()
    };

    // Build multi-resolution config (uses default values for most settings)
    let multi_res_config = MultiResolutionConfig {
        min_score: config.matcher.correlative_min_score,
        ..Default::default()
    };

    // Build dynamic matcher config
    let matcher_config = DynMatcherConfig {
        correlative: correlative_config,
        icp: icp_config,
        p2l: p2l_config,
        multi_res: multi_res_config,
        always_correlative: config.matcher.always_correlative,
        encoder_weight: config.matcher.encoder_weight,
    };

    // Build global map config
    let global_map_config = OccupancyGridConfig {
        resolution: config.map.resolution,
        initial_width: config.map.initial_width,
        initial_height: config.map.initial_height,
        log_odds_occupied: config.map.log_odds_occupied,
        log_odds_free: config.map.log_odds_free,
        log_odds_max: config.map.log_odds_max,
        log_odds_min: config.map.log_odds_min,
        occupied_threshold: config.map.occupied_threshold,
        free_threshold: config.map.free_threshold,
    };

    // Build keyframe manager config
    let keyframe_config = KeyframeManagerConfig {
        min_translation: config.keyframe.min_translation,
        min_rotation: config.keyframe.min_rotation,
        max_keyframes: config.keyframe.max_keyframes,
        min_interval_us: config.keyframe.min_interval_ms * 1000,
    };

    // Build submap manager config
    let submap_config = SubmapManagerConfig {
        grid_config: OccupancyGridConfig {
            resolution: config.map.resolution,
            initial_width: 10.0, // Smaller for submaps
            initial_height: 10.0,
            ..global_map_config.clone()
        },
        integrator_config: MapIntegratorConfig::default(),
        scans_per_submap: config.submap.scans_per_submap,
        overlap_scans: config.submap.overlap_scans,
    };

    OnlineSlamConfig {
        keyframe: keyframe_config,
        submap: submap_config,
        matcher_type,
        matcher: matcher_config,
        global_map: global_map_config,
        min_match_score: config.slam.min_match_score,
        lost_threshold: config.slam.lost_threshold,
        use_submap_matching: config.slam.use_submap_matching,
        min_scan_points: config.slam.min_scan_points,
        max_theta_deviation: config.slam.max_theta_deviation,
        max_translation_deviation: config.slam.max_translation_deviation,
        loop_closure: SlamLoopClosureConfig {
            enabled: config.loop_closure.enabled,
            detection_interval: config.loop_closure.detection_interval,
            optimization_threshold: config.loop_closure.optimization_threshold,
            detector: LoopDetectorConfig {
                min_node_distance: config.loop_closure.min_keyframe_gap,
                max_hamming_distance: config.loop_closure.matching.max_hamming_distance,
                min_match_score: config.loop_closure.matching.min_icp_score,
                max_candidates: config.loop_closure.max_candidates,
                iris_config: LidarIrisConfig {
                    num_rows: config.loop_closure.iris.num_rows,
                    num_cols: config.loop_closure.iris.num_cols,
                    max_range: config.loop_closure.iris.max_range,
                    signature_bits: config.loop_closure.iris.signature_bits,
                    ..Default::default()
                },
                ..Default::default()
            },
            optimizer: GraphOptimizerConfig {
                max_iterations: config.optimization.max_iterations as u32,
                convergence_threshold: config.optimization.tolerance,
                ..Default::default()
            },
        },
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

fn main() {
    // Initialize logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format(|buf, record| {
            writeln!(
                buf,
                "[{}] {} - {}",
                record.level(),
                record.target(),
                record.args()
            )
        })
        .init();

    let args = parse_args();
    let config = load_config(&args);

    log::info!("dhruva-slam starting");
    if let Some(ref bag) = config.bag.file {
        log::info!("  Input: Bag file {}", bag);
        if config.bag.loop_playback {
            log::info!("  Loop: enabled");
        }
    } else {
        log::info!(
            "  SangamIO: {} (TCP commands + UDP sensors)",
            config.source.sangam_address
        );
    }
    log::info!(
        "  Output port: {} (TCP commands+maps, UDP status)",
        config.output.bind_port
    );
    log::info!("  Odometry: {}", config.odometry.algorithm);
    log::info!("  Matcher: {}", config.matcher.algorithm);
    log::info!(
        "  Loop Closure: {} ({})",
        if config.loop_closure.enabled {
            "enabled"
        } else {
            "disabled"
        },
        config.optimization.solver
    );

    // Setup signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        log::info!("Received shutdown signal");
        r.store(false, Ordering::Relaxed);
    })
    .expect("Error setting Ctrl-C handler");

    // Run multi-threaded daemon (always)
    if let Err(e) = run_threaded_mode(&config, running.clone()) {
        log::error!("Daemon error: {}", e);
    }

    log::info!("dhruva-slam shutdown complete");
}

// ============================================================================
// Multi-Threaded Daemon
// ============================================================================

fn run_threaded_mode(
    config: &Config,
    running: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error>> {
    log::info!("Initializing multi-threaded SLAM daemon...");

    // 1. Create shared state
    let shared_state = create_shared_state();
    log::info!("  Shared state initialized");

    // 2. Create command channel for SLAM thread communication
    let (command_tx, command_rx) = create_command_channel();
    log::info!("  Command channel created");

    // 3. Create map manager
    let map_storage_path = Path::new(&config.map_storage.path);
    let map_manager = match MapManager::new(map_storage_path) {
        Ok(mgr) => {
            log::info!("  Map manager initialized ({})", config.map_storage.path);
            Arc::new(Mutex::new(mgr))
        }
        Err(e) => {
            log::warn!(
                "  Map manager init failed ({}): {}",
                config.map_storage.path,
                e
            );
            // Create with temp path instead
            let temp_path = std::env::temp_dir().join("dhruva_maps");
            let mgr = MapManager::new(&temp_path)?;
            log::info!("  Using temp map storage: {:?}", temp_path);
            Arc::new(Mutex::new(mgr))
        }
    };

    // 4. Load map list into shared state
    {
        if let Ok(mgr) = map_manager.lock()
            && let Ok(mut state) = shared_state.write()
        {
            state.map_list = mgr.as_map_list();
            log::info!("  Loaded {} saved maps", state.map_list.maps.len());
        }
    }

    // 5. Build SLAM thread config
    let slam_config = build_slam_config(config);
    let odometry_config = DynOdometryConfig {
        wheel: WheelOdometryConfig {
            ticks_per_meter: config.odometry.ticks_per_meter,
            wheel_base: config.odometry.wheel_base,
        },
        mahony: MahonyConfig::default(),
        ..Default::default()
    };
    let preprocessor_config = PreprocessorConfig {
        range_filter: RangeFilterConfig {
            min_range: config.preprocessing.min_range,
            max_range: config.preprocessing.max_range,
        },
        outlier_filter: OutlierFilterConfig::default(),
        downsampler: AngularDownsamplerConfig {
            target_points: config.preprocessing.target_points,
            min_angle_step: config.preprocessing.min_angle_step,
        },
        lidar_offset: LidarOffset::new(
            config.lidar.mounting_x,
            config.lidar.mounting_y,
            config.lidar.optical_offset,
            config.lidar.angle_offset,
        ),
    };

    let slam_thread_config = SlamThreadConfig {
        sangam_address: config.source.sangam_address.clone(),
        udp_port: config.source.udp_port,
        slam_config,
        odometry_type: config.odometry.algorithm_type(),
        odometry_config,
        preprocessor_config,
        bag_file: config.bag.file.clone(),
        loop_bag: config.bag.loop_playback,
    };

    // 6. Build stream config
    let stream_config = ThreadStreamConfig {
        robot_status_hz: config.stream.robot_status_hz,
        sensor_status_hz: config.stream.sensor_status_hz,
        map_hz: config.stream.map_hz,
        navigation_hz: config.stream.navigation_hz,
    };

    // 7. Create navigation channel
    let (nav_tx, nav_rx) = create_nav_channel();
    log::info!("  Navigation channel created");

    // 8. Create shared motion controller (shared between navigation and exploration)
    // This ensures a single TCP connection to SangamIO
    let motion_config = MotionConfig {
        max_linear_vel: config
            .navigation
            .max_linear_vel
            .max(config.exploration.motion.max_linear_vel),
        max_angular_vel: config
            .navigation
            .max_angular_vel
            .max(config.exploration.motion.max_angular_vel),
        obstacle_clearance: config.exploration.motion.obstacle_clearance,
        linear_vel: config.exploration.motion.linear_vel,
        angular_vel: config.exploration.motion.angular_vel,
    };
    let shared_motion = create_shared_motion_controller(motion_config.clone());

    // Connect motion controller to SangamIO (will auto-reconnect on operations)
    {
        let mut mc = shared_motion.lock().unwrap();
        match mc.connect(&config.source.sangam_address) {
            Ok(()) => log::info!("  Motion controller connected to SangamIO"),
            Err(e) => log::warn!(
                "  Motion controller initial connect failed (will retry): {}",
                e
            ),
        }
    }

    // 9. Spawn threads
    log::info!("Spawning threads...");

    // SLAM Thread - processes sensors, updates shared state
    let slam_thread = SlamThread::spawn(
        slam_thread_config,
        shared_state.clone(),
        command_rx,
        running.clone(),
    );
    log::info!("  SLAM thread started");

    // Navigation Thread - goal-based path following (optional)
    // Must be spawned before Publisher so we can pass nav_tx to Publisher
    let (navigation_thread, nav_command_tx) = if config.navigation.enabled {
        let nav_config = NavigationThreadConfig {
            navigator: NavigatorConfig {
                planner: AStarConfig {
                    robot_radius: config.navigation.robot_radius,
                    unknown_is_obstacle: config.navigation.unknown_is_obstacle,
                    ..Default::default()
                },
                waypoint_reached_threshold: config.navigation.waypoint_threshold,
                goal_position_threshold: config.navigation.goal_threshold,
                heading_threshold: config.navigation.heading_threshold,
                max_linear_vel: config.navigation.max_linear_vel,
                max_angular_vel: config.navigation.max_angular_vel,
                ..Default::default()
            },
            mapping: Default::default(),
            update_rate_hz: config.navigation.update_rate_hz,
        };

        let thread = NavigationThread::spawn(
            nav_config,
            shared_state.clone(),
            shared_motion.clone(),
            nav_rx,
            running.clone(),
        );
        log::info!(
            "  Navigation thread started ({}Hz)",
            config.navigation.update_rate_hz
        );
        (Some(thread), Some(nav_tx))
    } else {
        log::info!("  Navigation thread disabled");
        // Drop nav_rx and nav_tx if navigation disabled
        drop(nav_rx);
        drop(nav_tx);
        (None, None)
    };

    // Publisher Thread - unified client interface (streams + commands)
    // Single port for TCP (commands + maps) and UDP (status)
    let publisher_thread = PublisherThread::spawn(
        config.output.bind_port,
        shared_state.clone(),
        stream_config,
        command_tx,
        nav_command_tx,
        map_manager.clone(),
        running.clone(),
    );
    log::info!(
        "  Publisher thread started (port {} - TCP commands+maps, UDP status)",
        config.output.bind_port
    );

    // Exploration Thread - autonomous mapping (optional)
    let exploration_thread = if config.exploration.enabled {
        let exploration_config = ExplorationConfig {
            motion: MotionConfig {
                max_linear_vel: config.exploration.motion.max_linear_vel,
                max_angular_vel: config.exploration.motion.max_angular_vel,
                obstacle_clearance: config.exploration.motion.obstacle_clearance,
                linear_vel: config.exploration.motion.linear_vel,
                angular_vel: config.exploration.motion.angular_vel,
            },
            frontier: FrontierConfig {
                min_frontier_size: config.exploration.frontier.min_frontier_size,
                frontier_detection_range: config.exploration.frontier.detection_range,
                blocked_radius: config.exploration.frontier.blocked_radius,
                clustering_threshold: config.exploration.frontier.clustering_threshold,
                ..FrontierConfig::default()
            },
            loop_rate_hz: config.exploration.loop_rate_hz,
        };

        let thread = ExplorationThread::spawn(
            exploration_config,
            shared_state.clone(),
            shared_motion.clone(),
            running.clone(),
        );
        log::info!(
            "  Exploration thread started (strategy: {})",
            config.exploration.strategy
        );
        Some(thread)
    } else {
        log::info!("  Exploration thread disabled");
        None
    };

    log::info!("Multi-threaded daemon running");
    log::info!(
        "  Output port: {} (TCP commands+maps, UDP status)",
        config.output.bind_port
    );
    if config.navigation.enabled {
        log::info!(
            "  Navigation: enabled ({}Hz)",
            config.navigation.update_rate_hz
        );
    }
    if config.exploration.enabled {
        log::info!("  Exploration: enabled ({})", config.exploration.strategy);
    }

    // 9. Wait for shutdown signal (main thread just monitors)
    while running.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_millis(100));
    }

    log::info!("Shutdown signal received, waiting for threads...");

    // 10. Join threads
    if let Err(e) = slam_thread.join() {
        log::error!("SLAM thread panicked: {:?}", e);
    }
    if let Err(e) = publisher_thread.join() {
        log::error!("Publisher thread panicked: {:?}", e);
    }
    if let Some(thread) = navigation_thread
        && let Err(e) = thread.join()
    {
        log::error!("Navigation thread panicked: {:?}", e);
    }
    if let Some(thread) = exploration_thread {
        thread.join();
    }

    log::info!("All threads stopped");
    Ok(())
}
