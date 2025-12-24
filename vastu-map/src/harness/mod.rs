//! Test harness for integration testing and examples
//!
//! This module provides a complete simulation environment for testing
//! vastu-map's SLAM algorithms using sangam-io's mock device.
//!
//! # Features
//!
//! - Synchronous simulation with physics, lidar, and SLAM
//! - Ground truth tracking for accuracy metrics
//! - SVG visualization output
//! - YAML scenario support
//!
//! # Usage
//!
//! ```rust,ignore
//! use vastu_map::harness::{HarnessConfig, TestHarness};
//!
//! let config = HarnessConfig::medium_room();
//! let mut harness = TestHarness::new(config)?
//!     .with_visualization("output.svg");
//!
//! // Run exploration loop
//! loop {
//!     let step = harness.step(0.2, 0.0);  // forward at 0.2 m/s
//!     harness.process_lidar();
//!
//!     if step.any_bumper() {
//!         break;
//!     }
//! }
//!
//! let result = harness.finalize();
//! println!("Position error: {:.3}m", result.metrics.position_error);
//! ```

pub mod adapters;
pub mod harness;
pub mod metrics;
pub mod path;
pub mod scenario;
pub mod visualization;

// Re-exports for convenience
pub use adapters::lidar_to_point_cloud;
pub use harness::{
    HarnessConfig, ObservationRecord, StepResult, TestHarness, TestResult, TrajectoryHistory,
};
pub use metrics::{AcceptanceCriteria, ConvergenceStats, TestMetrics, TimingStats};
pub use path::PathSegment;
pub use scenario::{
    PathCommand, PoseConfig, ScenarioConfig, WheelCommand, convert_path_commands, load_scenario,
    run_all_scenarios, run_scenario,
};
pub use visualization::Visualizer;
