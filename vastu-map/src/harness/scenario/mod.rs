//! YAML scenario configuration and runner
//!
//! Provides structures and functions for defining and running
//! test scenarios from YAML configuration files.

pub mod config;
pub mod runner;

pub use config::{PathCommand, PoseConfig, ScenarioConfig, WheelCommand};
pub use runner::{convert_path_commands, load_scenario, run_all_scenarios, run_scenario};
