//! Integration tests for vastu-map using sangam-io mock device
//!
//! These tests verify the SLAM pipeline using simulated sensor data from
//! sangam-io's mock device, which provides ray-cast lidar in simulated environments.
//!
//! # Running Tests
//!
//! ```bash
//! # Run all integration tests
//! cargo test --features integration-tests -- --nocapture
//!
//! # Run specific scenario
//! cargo test --features integration-tests square_room
//!
//! # View generated visualizations
//! open test_output/square_room.svg
//! ```

mod adapters;
mod harness;
mod metrics;
mod path_generator;
mod scenarios;
mod visualization;
mod yaml_config;
mod yaml_runner;

// Re-export for test convenience
pub use harness::{HarnessConfig, TestHarness, TestResult};
pub use metrics::{AcceptanceCriteria, TestMetrics, TimingStats};
pub use path_generator::PathSegment;
