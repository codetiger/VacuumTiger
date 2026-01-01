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
//! # Localization with IMU Fusion
//!
//! For improved accuracy, enable Cartographer-style motion filtering with IMU:
//!
//! ```ignore
//! use vastu_slam::modes::{Localizer, LocalizerConfig};
//! use vastu_slam::core::ImuMeasurement;
//!
//! // Enable motion filtering (includes pose extrapolator + motion filter)
//! let config = LocalizerConfig::with_motion_filtering();
//! let mut localizer = Localizer::new(map, config);
//! localizer.set_initial_pose_with_timestamp(Pose2D::new(0.0, 0.0, 0.0), 0);
//!
//! // Feed IMU at high rate (~110 Hz) - call this frequently
//! localizer.add_imu(&ImuMeasurement::from_raw(timestamp_us, gyro, accel));
//!
//! // Localize with timestamp for motion filtering
//! let result = localizer.localize_with_timestamp(&scan, Some(odom_delta), timestamp_us);
//!
//! if result.skipped {
//!     // Motion filter rejected scan (not enough motion)
//!     println!("Scan skipped, pose: {:?}", result.pose);
//! } else if result.converged {
//!     println!("Localized at: {:?}", result.pose);
//! }
//! ```
//!
//! ## Motion Filtering Algorithm
//!
//! The motion filter uses Cartographer-style thresholds:
//! - **Distance**: Insert scan if moved > 0.2m (default)
//! - **Rotation**: Insert scan if rotated > 2° (default)
//! - **Time**: Insert scan if > 5 seconds since last (default)
//!
//! ## IMU Fusion Algorithm
//!
//! The pose extrapolator combines odometry and IMU:
//! - **Translation**: 100% from wheel odometry (most accurate for ground robots)
//! - **Rotation**: Weighted blend (default: 70% odometry + 30% IMU)
//!
//! This prioritizes wheel encoders while using IMU to smooth rotation estimates.
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
