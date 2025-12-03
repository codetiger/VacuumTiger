//! Localization module (Phase 6).
//!
//! Provides Monte Carlo Localization (MCL) for robot pose estimation
//! within a known map.
//!
//! # Components
//!
//! - [`MotionModel`]: Odometry-based motion model with configurable noise
//! - [`SensorModel`]: Likelihood field sensor model for laser scans
//! - [`ParticleFilter`]: Full MCL implementation with adaptive resampling
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::localization::{ParticleFilter, ParticleFilterConfig};
//! use dhruva_slam::mapping::OccupancyGrid;
//!
//! let config = ParticleFilterConfig::default();
//! let mut filter = ParticleFilter::new(config, initial_pose, &map);
//!
//! // Predict step with odometry
//! filter.predict(&odom_delta);
//!
//! // Update step with laser scan
//! filter.update(&scan, &map);
//!
//! // Get best estimate
//! let pose = filter.estimate();
//! ```

mod motion_model;
mod sensor_model;
mod particle_filter;

pub use motion_model::{MotionModel, MotionModelConfig};
pub use sensor_model::{SensorModel, LikelihoodFieldModel, SensorModelConfig};
pub use particle_filter::{
    Particle, ParticleFilter, ParticleFilterConfig, ParticleFilterState,
};
