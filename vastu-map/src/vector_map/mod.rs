//! VectorMap: The main SLAM map implementation.
//!
//! VectorMap uses line and corner features as the primary map representation,
//! providing efficient storage and queries for indoor environments.
//!
//! # Components
//!
//! - [`VectorMap`]: The main SLAM map struct implementing the [`Map`](crate::Map) trait
//! - [`VectorMapConfig`]: Configuration for all SLAM components
//! - [`LineStore`]: Internal line storage with synchronized auxiliary structures
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_map::{VectorMap, VectorMapConfig, Map};
//! use vastu_map::core::{Pose2D, PointCloud2D, Point2D};
//!
//! let config = VectorMapConfig::default();
//! let mut map = VectorMap::new(config);
//!
//! // Process scans
//! let scan = PointCloud2D::from_points(&[
//!     Point2D::new(1.0, 0.0),
//!     Point2D::new(1.0, 0.1),
//! ]);
//! let odometry = Pose2D::identity();
//! let result = map.observe(&scan, odometry);
//! ```

mod config;
mod line_store;
mod map;

// Re-export public API
pub use config::{ConfigError, VectorMapConfig};
pub use line_store::LineStore;
pub use map::VectorMap;
