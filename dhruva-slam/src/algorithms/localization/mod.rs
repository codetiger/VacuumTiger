//! Localization module (Phase 6).
//!
//! Provides motion and sensor models for robot localization.
//!
//! # Components
//!
//! - [`MotionModel`]: Odometry-based motion model with configurable noise
//! - [`SensorModel`]: Likelihood field sensor model for laser scans
//!
//! # Future Work
//!
//! Monte Carlo Localization (particle filter) was removed as it was not
//! integrated into the SLAM pipeline. It can be re-added when needed for
//! global localization or kidnapped robot recovery.

mod motion_model;
mod sensor_model;

pub use motion_model::{MotionModel, MotionModelConfig};
pub use sensor_model::{LikelihoodFieldModel, SensorModel, SensorModelConfig};
