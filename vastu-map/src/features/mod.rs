//! Geometric feature types for VectorMap SLAM.
//!
//! This module provides:
//! - [`Line2D`]: Line segment defined by endpoints
//! - [`LineCollection`]: SoA collection for SIMD operations
//! - [`Corner2D`]: Corner feature at line intersections
//! - [`CornerCollection`]: SoA collection for corners
//! - [`FeatureSet`]: Combined container for lines and corners
//! - [`CornerDescriptor`]: Shape context descriptor for corners
//! - [`ScanDescriptor`]: Aggregate descriptor for loop closure

pub mod corner;
pub mod descriptors;
pub mod feature_set;
pub mod line;
pub mod line_collection;

pub use corner::{Corner2D, CornerCollection};
pub use descriptors::{CornerDescriptor, ScanDescriptor};
pub use feature_set::FeatureSet;
pub use line::Line2D;
pub use line_collection::LineCollection;
