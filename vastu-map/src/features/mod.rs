//! Geometric feature types for VectorMap SLAM.
//!
//! This module provides the core feature representations used throughout vastu-map:
//!
//! - [`Line2D`]: Line segment defined by endpoints
//! - [`LineCollection`]: SoA collection for SIMD batch operations
//! - [`Corner2D`]: Corner feature at line intersections
//! - [`CornerCollection`]: SoA collection for corners
//! - [`FeatureSet`]: Combined container for lines and corners
//! - [`CornerDescriptor`]: Shape context descriptor for loop closure
//! - [`ScanDescriptor`]: Aggregate scan descriptor for place recognition
//!
//! # Design Rationale
//!
//! ## Why Lines Instead of Points?
//!
//! Indoor environments are dominated by straight edges (walls, furniture, doorframes).
//! Using line segments as the primary feature type provides:
//!
//! - **Compact representation**: One line replaces dozens of points
//! - **Robust matching**: Lines survive partial occlusion and noise
//! - **Meaningful geometry**: Lines encode structure, not just shape
//! - **Efficient ICP**: Point-to-line ICP converges faster than point-to-point
//!
//! ## Why Endpoints, Not Parametric Form?
//!
//! Lines use endpoint representation `(start, end)` instead of parametric `(rho, theta)`:
//!
//! - **Simpler transforms**: Just rotate+translate two points
//! - **SIMD-friendly**: 4 floats per line, no trig during transform
//! - **No angle wrapping**: No discontinuity at ±π
//! - **Built-in bounds**: Segment extent is implicit in endpoints
//!
//! ## Why Corners?
//!
//! Corners (line intersections) provide:
//!
//! - **Reliable landmarks**: Corners are stable across viewpoints
//! - **Loop closure signatures**: Corner patterns are distinctive
//! - **Pose constraints**: Two corners fully constrain 2D pose
//!
//! # Data Layout
//!
//! Feature collections use Struct-of-Arrays (SoA) for SIMD efficiency:
//!
//! ```text
//! AoS (Array of Structs):     SoA (Struct of Arrays):
//! [Line{x1,y1,x2,y2}]         start_xs: [x1, x1, x1, ...]
//! [Line{x1,y1,x2,y2}]    →    start_ys: [y1, y1, y1, ...]
//! [Line{x1,y1,x2,y2}]         end_xs:   [x2, x2, x2, ...]
//!                             end_ys:   [y2, y2, y2, ...]
//! ```
//!
//! SoA enables processing 4 lines per SIMD instruction.

pub mod corner;
pub mod descriptors;
pub mod feature_set;
pub mod line;
pub mod line_collection;
pub mod spatial_index;

pub use corner::{Corner2D, CornerCollection};
pub use descriptors::{CornerDescriptor, ScanDescriptor};
pub use feature_set::FeatureSet;
pub use line::Line2D;
pub use line_collection::LineCollection;
pub use spatial_index::{
    LineSpatialIndex, MIN_CELL_SIZE, MIN_LINES_FOR_INDEX, should_use_spatial_index,
};
