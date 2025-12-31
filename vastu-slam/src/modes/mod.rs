//! SLAM operation modes for different use cases.
//!
//! This module provides different modes of operation depending on whether the
//! environment is known or being explored for the first time.
//!
//! # Available Modes
//!
//! | Mode | Description | Map Updates |
//! |------|-------------|-------------|
//! | [`Localizer`] | Localize on known map | No |
//! | Full SLAM | Mapping + localization | Yes |
//!
//! # Localization Mode
//!
//! When the environment has already been mapped, use localization mode for
//! efficient pose tracking without modifying the map:
//!
//! ```ignore
//! use vastu_slam::modes::{Localizer, LocalizerConfig};
//! use vastu_slam::io::load_vastu;
//! use vastu_slam::core::Pose2D;
//! use std::path::Path;
//!
//! // Load a known map
//! let map = load_vastu(Path::new("map.vastu"))?;
//!
//! // Create localizer with default config
//! let mut localizer = Localizer::new(map, LocalizerConfig::default());
//!
//! // Set initial pose (from dock position, QR code, or user input)
//! localizer.set_initial_pose(Pose2D::new(1.0, 2.0, 0.0));
//!
//! // Localize with each scan
//! let result = localizer.localize(&scan, Some(odom_delta));
//! if result.converged {
//!     println!("Robot at: ({:.2}, {:.2})", result.pose.x, result.pose.y);
//!     println!("Confidence: {:.1}%", result.score * 100.0);
//! } else {
//!     println!("Using odometry fallback");
//! }
//! ```
//!
//! # Choosing a Mode
//!
//! **Use Localization Mode when:**
//! - Robot is navigating a previously mapped area
//! - Map is known to be accurate and current
//! - Lower CPU/memory usage is needed
//!
//! **Use Full SLAM when:**
//! - First-time mapping of environment
//! - Environment may have changed
//! - Exploring new areas

mod localization;

pub use localization::{LocalizationResult, Localizer, LocalizerConfig};
