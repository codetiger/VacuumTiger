//! DhruvaSLAM - Modular SLAM implementation for robotic vacuum cleaners
//!
//! # Architecture
//!
//! The crate is organized into 5 logical layers:
//!
//! ```text
//! ┌─────────────────────────────────────────────────────┐
//! │                      bin/                           │  ← Executables
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
//! # Phases
//!
//! ## Phase 1: Foundation
//! - Core types: Pose2D, Point2D, Timestamped<T>
//! - Math primitives: angle normalization, pose composition
//! - SangamIO client: TCP connection with Protobuf wire format
//!
//! ## Phase 2: Odometry
//! - New types: Twist2D, ImuReading, Covariance2D
//! - Wheel odometry from encoder ticks
//! - Complementary filter for encoder + gyro fusion
//! - Pose interpolation
//!
//! ## Phase 3: Scan Processing
//! - New types: LaserScan, PointCloud2D
//! - Range filtering (remove invalid returns)
//! - Outlier removal (statistical filtering)
//! - Angular downsampling (reduce point count)
//! - Polar to Cartesian conversion
//!
//! ## Phase 4: Scan Matching
//! - Point-to-Point ICP with k-d tree acceleration
//! - Correlative scan matcher for large initial errors
//! - Multi-resolution hierarchical matching
//!
//! ## Phase 5: Mapping
//! - Log-odds occupancy grid representation
//! - Bresenham ray tracing for free space
//! - Scan integration with range filtering
//!
//! ## Phase 6: Localization
//! - Monte Carlo Localization (particle filter)
//! - Odometry motion model with configurable noise
//! - Likelihood field sensor model
//! - Adaptive resampling
//!
//! ## Phase 7: Online SLAM
//! - SlamEngine trait for SLAM algorithm abstraction
//! - Keyframe selection based on motion thresholds
//! - Submap management (local map partitioning)
//! - Scan context descriptors for place recognition
//! - OnlineSlam combining scan matching + mapping
//!
//! ## Phase 8: Graph Optimization
//! - Pose graph data structure (nodes + edges)
//! - Loop closure detection using scan context
//! - Gauss-Newton/Levenberg-Marquardt optimizer
//! - Information matrix weighting

// ============================================================================
// Layer 1: Core foundation (no internal deps)
// ============================================================================
pub mod core;

// ============================================================================
// Layer 2: Sensor processing (depends on core)
// ============================================================================
pub mod sensors;

// ============================================================================
// Layer 3: Algorithms (depends on core, sensors)
// ============================================================================
pub mod algorithms;

// ============================================================================
// Layer 4: SLAM engine (depends on core, sensors, algorithms)
// ============================================================================
pub mod engine;

// ============================================================================
// Layer 5: I/O infrastructure (depends on all layers)
// ============================================================================
pub mod io;

// ============================================================================
// Convenience re-exports (flat namespace for common use)
// ============================================================================

// Core types
pub use core::math;
pub use core::types::{Covariance2D, ImuReading, Twist2D};
pub use core::types::{LaserScan, PointCloud2D};
pub use core::types::{Point2D, Pose2D, Timestamped};

// Sensors - Odometry
pub use sensors::odometry::{
    ComplementaryConfig, ComplementaryFilter, Eskf, EskfConfig, EvaluationResult, MeasurementNoise,
    OdometryEvaluator, ProcessNoise, ScenarioBounds, Stats, TestScenario, WheelOdometry,
    WheelOdometryConfig,
};

// Sensors - Preprocessing
pub use sensors::preprocessing::{
    AngularDownsampler, AngularDownsamplerConfig, DynamicFilter, DynamicFilterConfig,
    DynamicFilterStats, OutlierFilter, OutlierFilterConfig, PreprocessorConfig, RangeFilter,
    RangeFilterConfig, ScanConverter, ScanPreprocessor,
};

// Algorithms - Matching
pub use algorithms::matching::{
    CorrelativeConfig, CorrelativeMatcher, IcpConfig, MultiResolutionConfig,
    MultiResolutionMatcher, PointToLineIcp, PointToLineIcpConfig, PointToPointIcp, ScanMatchResult,
    ScanMatcher,
};

// Algorithms - Mapping
pub use algorithms::mapping::{
    CellState, MapIntegrator, MapIntegratorConfig, MapMetadata, MapRegion, OccupancyGrid,
    OccupancyGridConfig, RayTracer,
};

// Algorithms - Localization
pub use algorithms::localization::{
    LikelihoodFieldModel, MotionModel, MotionModelConfig, Particle, ParticleFilter,
    ParticleFilterConfig, ParticleFilterState, SensorModel, SensorModelConfig,
};

// Engine - SLAM
pub use engine::slam::{
    Keyframe, KeyframeConfig, KeyframeManager, KidnappedDetection, KidnappedDetector,
    KidnappedDetectorConfig, KidnappedReason, OnlineSlam, OnlineSlamConfig, RecoveryAction,
    RecoveryConfig, RecoveryState, RecoveryStateMachine, RecoveryStats, RecoveryStrategy,
    ScanContext, SlamEngine, SlamMode, SlamResult, SlamStatus, Submap, SubmapConfig, SubmapManager,
    SubmapState,
};

// Engine - Graph
pub use engine::graph::{
    EdgeType, GraphOptimizer, GraphOptimizerConfig, Information2D, LoopClosureCandidate,
    LoopDetector, LoopDetectorConfig, OptimizationResult, PoseEdge, PoseGraph, PoseNode,
    TerminationReason,
};

// I/O - SangamIO client
pub use io::sangam_client::{LidarPoint, LidarScan, SangamClient};

// I/O - Bag
pub use io::bag::{BagHeader, BagInfo, BagMessage, BagPlayer, BagRecorder, SimulatedClient};

// I/O - Streaming
pub use io::streaming::{
    OdometryDiagnostics, OdometryMessage, OdometryPipeline, OdometryPipelineConfig,
    OdometryPublisher,
};
