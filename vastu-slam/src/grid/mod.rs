//! Occupancy grid implementation with multi-sensor fusion.
//!
//! This module provides the core occupancy grid storage and update mechanisms
//! for VastuSLAM. The grid uses a Structure-of-Arrays (SoA) layout for
//! SIMD-optimized operations and log-odds Bayesian updates for robust
//! occupancy estimation.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                      Sensor Inputs                          │
//! │   LidarScan (5Hz)  │  CliffSensors (110Hz)  │  BumperSensors│
//! └─────────┬──────────┴───────────┬────────────┴───────┬───────┘
//!           │                      │                    │
//!           ▼                      ▼                    ▼
//! ┌─────────────────┐    ┌─────────────────┐   ┌─────────────────┐
//! │  lidar_update   │    │  cliff_update   │   │  bumper_update  │
//! │  (log-odds)     │    │  (priority)     │   │  (priority)     │
//! └────────┬────────┘    └────────┬────────┘   └────────┬────────┘
//!          │                      │                     │
//!          └──────────────────────┼─────────────────────┘
//!                                 ▼
//!                     ┌───────────────────────┐
//!                     │     GridStorage       │
//!                     │  (SoA, SIMD-ready)    │
//!                     └───────────────────────┘
//! ```
//!
//! ## Key Components
//!
//! - [`GridStorage`]: Core storage using SoA layout with log-odds occupancy
//! - [`GridConfig`]: Grid dimensions, resolution, and expansion settings
//! - [`SensorConfig`]: Robot geometry and sensor parameters
//! - [`LogOddsConfig`]: Bayesian update parameters (hit/miss log-odds)
//! - [`MapConfig`]: Combined configuration for the full map
//!
//! ## Sensor Update Modules
//!
//! - [`lidar_update`]: Process 360° lidar scans using Bresenham ray casting
//! - [`cliff_update`]: Mark cliff sensor detections with priority
//! - [`bumper_update`]: Mark bumper collisions with highest priority
//! - [`raycaster`]: Bresenham line algorithm for efficient ray tracing
//!
//! ## Log-Odds Model
//!
//! The grid uses log-odds representation for probabilistic occupancy:
//!
//! ```text
//! L(x) = log(P(x) / (1 - P(x)))   # Log-odds conversion
//! L_new = L_old + L_observation    # Bayesian update
//!
//! Thresholds:
//!   L >  50 → Occupied (Wall)
//!   L < -50 → Free (Floor)
//!   else    → Unknown
//! ```
//!
//! ## Example
//!
//! ```rust,ignore
//! use vastu_slam::grid::{GridStorage, MapConfig, lidar_update};
//!
//! // Create grid from configuration
//! let config = MapConfig::default();
//! let mut storage = GridStorage::centered(800, 800, 0.025);
//!
//! // Process lidar scan
//! let result = lidar_update::update_from_lidar(
//!     &mut storage,
//!     &lidar_scan,
//!     robot_pose,
//!     &config.sensor,
//!     &config.grid,
//! );
//!
//! // Query occupancy
//! let prob = storage.get_probability(coord);  // 0.0 - 1.0
//! let is_wall = storage.is_occupied(coord);   // L > 50
//! ```

pub mod bumper_update;
pub mod cliff_update;
mod config;
pub mod lidar_update;
pub mod raycaster;
mod storage;

pub use config::{GridConfig, LogOddsConfig, MapConfig, SensorConfig};
pub use storage::{CellCounts, CellMut, GridStorage};
