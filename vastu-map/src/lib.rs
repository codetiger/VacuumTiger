//! # Vastu-Map: Feature-Based 2D SLAM Library
//!
//! A high-performance SLAM (Simultaneous Localization and Mapping) library
//! with SIMD-friendly data layouts, designed for indoor robot navigation.
//!
//! ## Features
//!
//! - **VectorMap Representation**: Uses line segments and corners instead of
//!   occupancy grids for memory-efficient mapping
//! - **Full SLAM Pipeline**: Localization and mapping integrated into a clean API
//! - **SIMD Optimized**: Data layouts designed for LLVM auto-vectorization (works on ARM, x86, etc.)
//! - **SoA Data Layout**: Struct-of-Arrays for cache-friendly SIMD operations
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use vastu_map::{VectorMap, VectorMapConfig, Map};
//! use vastu_map::core::{Pose2D, PointCloud2D, Point2D};
//!
//! // Create map with default configuration
//! let config = VectorMapConfig::default();
//! let mut map = VectorMap::new(config);
//!
//! // Process a scan observation
//! let scan = PointCloud2D::from_points(&[
//!     Point2D::new(1.0, 0.0),
//!     Point2D::new(1.0, 0.1),
//!     Point2D::new(1.0, 0.2),
//! ]);
//! let odometry = Pose2D::identity();
//! let result = map.observe(&scan, odometry);
//! println!("Pose: ({:.2}, {:.2}), confidence: {:.2}",
//!     result.pose.x, result.pose.y, result.confidence);
//! ```
//!
//! ## Coordinate Frame
//!
//! All coordinates follow the ROS REP-103 convention:
//! - **X-forward**: Positive X is in front of the robot
//! - **Y-left**: Positive Y is to the left of the robot
//! - **Z-up**: Positive Z is upward (not used in 2D)
//! - **Rotation**: Counter-clockwise positive
//!
//! ## Architecture
//!
//! The library is organized into modules:
//!
//! - [`core`]: Fundamental types (Point2D, Pose2D, PointCloud2D, etc.)
//! - [`simd`]: SIMD primitives for auto-vectorization
//! - [`config`]: Configuration types
//! - [`features`]: Line and corner feature types
//! - [`extraction`]: Line extraction from point clouds
//! - [`matching`]: Scan-to-map matching algorithms (ICP, RANSAC)
//! - [`integration`]: Map integration and merging
//! - [`query`]: Map queries (raycast, occupancy, frontiers)
//! - [`loop_closure`]: Loop closure detection with keyframes
//! - [`vector_map`]: Main VectorMap implementation
//!
//! ## Data Flow
//!
//! ```text
//!                          ┌─────────────────┐
//!                          │   Lidar Scan    │
//!                          │  (PolarScan)    │
//!                          └────────┬────────┘
//!                                   │ to_cartesian()
//!                                   ▼
//!                          ┌─────────────────┐
//!                          │  PointCloud2D   │
//!                          │   (SoA layout)  │
//!                          └────────┬────────┘
//!                                   │
//!              ┌────────────────────┼────────────────────┐
//!              │                    │                    │
//!              ▼                    ▼                    ▼
//!     ┌────────────────┐   ┌────────────────┐   ┌────────────────┐
//!     │   Extraction   │   │    Matching    │   │  Loop Closure  │
//!     │  (Split-Merge) │   │  (Point-Line   │   │  (Keyframes +  │
//!     │                │   │      ICP)      │   │  Descriptors)  │
//!     └───────┬────────┘   └───────┬────────┘   └───────┬────────┘
//!             │                    │                    │
//!             ▼                    ▼                    │
//!     ┌────────────────┐   ┌────────────────┐           │
//!     │  Lines/Corners │   │  Matched Pose  │           │
//!     │  (FeatureSet)  │   │  + Confidence  │           │
//!     └───────┬────────┘   └───────┬────────┘           │
//!             │                    │                    │
//!             └──────────┬─────────┘                    │
//!                        │                              │
//!                        ▼                              │
//!               ┌────────────────┐                      │
//!               │  Integration   │◄─────────────────────┘
//!               │ (Merge/Add to  │   Loop constraints
//!               │  VectorMap)    │
//!               └───────┬────────┘
//!                       │
//!                       ▼
//!               ┌────────────────┐
//!               │   VectorMap    │──► Query (raycast, occupancy)
//!               │   (Lines +     │──► Frontiers (exploration)
//!               │   Corners)     │──► Path Planning
//!               └────────────────┘
//! ```
//!
//! ## SIMD and Portability
//!
//! The library uses LLVM auto-vectorization patterns that work across different
//! architectures. No platform-specific intrinsics are used—users can configure
//! compiler flags appropriate for their target platform.

pub mod config;
pub mod core;
pub mod extraction;
pub mod features;
pub mod integration;
pub mod loop_closure;
pub mod matching;
pub mod motion_model;
pub mod query;
pub mod simd;
pub mod vector_map;

// Re-export main types at crate root
pub use motion_model::MotionModel;
pub use vector_map::{VectorMap, VectorMapConfig};

// Re-export extensibility traits
pub use extraction::{LineExtractor, SplitMergeExtractor};
pub use loop_closure::LoopDetector;
pub use matching::ScanMatcher;

// ─────────────────────────────────────────────────────────────────────────────
// Map Trait and Supporting Types
// ─────────────────────────────────────────────────────────────────────────────

use core::{Bounds, Point2D, PointCloud2D, Pose2D};

/// Point occupancy state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Occupancy {
    /// Point is in navigable free space.
    Free,
    /// Point is on or inside an obstacle.
    Occupied,
    /// Point has not been observed yet.
    Unknown,
}

/// Exploration frontier (unconnected line endpoint).
#[derive(Debug, Clone)]
pub struct Frontier {
    /// Position of the frontier point.
    pub point: Point2D,
    /// Index of the line this endpoint belongs to.
    pub line_idx: usize,
    /// True if this is the start endpoint, false if end.
    pub is_start: bool,
}

/// A planned path through the map.
#[derive(Debug, Clone)]
pub struct Path {
    /// Waypoints along the path.
    pub points: Vec<Point2D>,
    /// Total path length in meters.
    pub length: f32,
}

impl Path {
    /// Create a new empty path.
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            length: 0.0,
        }
    }

    /// Check if the path is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

impl Default for Path {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of observing a scan.
#[derive(Debug, Clone)]
pub struct ObserveResult {
    /// Localized pose in world frame.
    pub pose: Pose2D,
    /// Match confidence (0.0 = used odometry, 1.0 = excellent match).
    pub confidence: f32,
    /// Number of features extracted from scan.
    pub features_extracted: usize,
    /// Number of new features added to map.
    pub features_added: usize,
    /// Number of features merged with existing map features.
    pub features_merged: usize,
    /// Number of ICP iterations performed (0 if no matching was done).
    pub icp_iterations: usize,
    /// Whether the ICP algorithm converged.
    pub icp_converged: bool,
    /// Whether a loop closure was detected.
    pub loop_closure_detected: bool,
}

impl ObserveResult {
    /// Create a result with just pose and confidence.
    pub fn from_pose(pose: Pose2D, confidence: f32) -> Self {
        Self {
            pose,
            confidence,
            features_extracted: 0,
            features_added: 0,
            features_merged: 0,
            icp_iterations: 0,
            icp_converged: false,
            loop_closure_detected: false,
        }
    }
}

/// High-level map interface.
///
/// Encapsulates localization, mapping, and query algorithms.
/// All coordinates in meters, all angles in radians.
///
/// # Usage
///
/// ```rust,ignore
/// use vastu_map::{Map, core::{PointCloud2D, Pose2D, Point2D}};
///
/// fn robot_loop<M: Map>(map: &mut M, get_scan: impl Fn() -> PointCloud2D) {
///     let goal = Point2D::new(5.0, 3.0);
///
///     loop {
///         let scan = get_scan();
///         let odom = Pose2D::identity(); // Get from odometry
///
///         // Observe and localize
///         let result = map.observe(&scan, odom);
///         println!("Position: ({:.2}, {:.2}), confidence: {:.2}",
///             result.pose.x, result.pose.y, result.confidence);
///
///         // Plan path to goal
///         if let Some(path) = map.get_path(result.pose.position(), goal) {
///             println!("Path found: {} waypoints, {:.2}m", path.points.len(), path.length);
///         }
///     }
/// }
/// ```
pub trait Map: Send + Sync {
    /// Feed sensor observation and get localized pose.
    ///
    /// This is the main SLAM function. It:
    /// 1. Extracts features from the scan
    /// 2. Matches features against the map (localization)
    /// 3. Integrates new/updated features (mapping)
    ///
    /// # Arguments
    /// * `scan` - Point cloud in robot frame
    /// * `odometry` - Odometry estimate (used as initial guess and fallback)
    ///
    /// # Returns
    /// Observation result including localized pose and confidence.
    /// On match failure, returns odometry pose with low confidence.
    fn observe(&mut self, scan: &PointCloud2D, odometry: Pose2D) -> ObserveResult;

    /// Cast a ray and return distance to first obstacle.
    ///
    /// # Arguments
    /// * `from` - Ray origin in world frame
    /// * `direction` - Ray direction (will be normalized internally)
    /// * `max_range` - Maximum ray distance
    ///
    /// # Returns
    /// Distance to first intersection, or `max_range` if no intersection.
    fn raycast(&self, from: Point2D, direction: Point2D, max_range: f32) -> f32;

    /// Query point occupancy.
    ///
    /// Uses contour-based occupancy (like font outlines):
    /// - Inside navigable contour → Free
    /// - On or outside boundary → Occupied
    /// - Not yet mapped → Unknown
    fn query(&self, point: Point2D) -> Occupancy;

    /// Get exploration frontiers (unconnected line endpoints).
    ///
    /// Frontiers are line endpoints that don't connect to other lines,
    /// indicating unexplored areas.
    fn frontiers(&self) -> Vec<Frontier>;

    /// Plan path between two points.
    ///
    /// # Returns
    /// Path if one exists, None otherwise.
    /// Note: Path planning is deferred in initial implementation.
    fn get_path(&self, from: Point2D, to: Point2D) -> Option<Path>;

    /// Get current map bounds.
    ///
    /// Returns the axis-aligned bounding box containing all map features.
    /// The bounds grow dynamically as the robot explores.
    fn bounds(&self) -> Bounds;

    /// Clear all map data.
    fn clear(&mut self);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_occupancy_debug() {
        assert_eq!(format!("{:?}", Occupancy::Free), "Free");
        assert_eq!(format!("{:?}", Occupancy::Occupied), "Occupied");
        assert_eq!(format!("{:?}", Occupancy::Unknown), "Unknown");
    }

    #[test]
    fn test_path_new() {
        let path = Path::new();
        assert!(path.is_empty());
        assert_eq!(path.length, 0.0);
    }

    #[test]
    fn test_observe_result() {
        let result = ObserveResult::from_pose(Pose2D::new(1.0, 2.0, 0.5), 0.8);
        assert_eq!(result.pose.x, 1.0);
        assert_eq!(result.pose.y, 2.0);
        assert_eq!(result.confidence, 0.8);
        assert_eq!(result.features_extracted, 0);
        assert_eq!(result.icp_iterations, 0);
        assert!(!result.icp_converged);
        assert!(!result.loop_closure_detected);
    }
}
