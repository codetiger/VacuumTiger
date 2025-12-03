//! Pose graph optimization module (Phase 8).
//!
//! Provides graph-based SLAM backend for loop closure and global optimization.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                      POSE GRAPH                              │
//! │                                                              │
//! │    Nodes: Robot poses at keyframe times                     │
//! │                                                              │
//! │    Edges: Relative constraints between poses                │
//! │           - Odometry edges (sequential)                      │
//! │           - Loop closure edges (non-sequential)              │
//! │                                                              │
//! │    [P0] ──odom──▶ [P1] ──odom──▶ [P2] ──odom──▶ [P3]        │
//! │     │                              ▲                         │
//! │     └────────── loop closure ──────┘                         │
//! └─────────────────────────────────────────────────────────────┘
//!                              │
//!                              ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │                     OPTIMIZATION                             │
//! │                                                              │
//! │    Minimize: Σ ||error(edge)||² weighted by information      │
//! │                                                              │
//! │    Method: Gauss-Newton / Levenberg-Marquardt                │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Components
//!
//! - [`PoseGraph`]: Graph data structure with nodes (poses) and edges (constraints)
//! - [`LoopDetector`]: Detects potential loop closures using scan context
//! - [`GraphOptimizer`]: Optimizes the pose graph to minimize constraint errors
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::graph::{PoseGraph, GraphOptimizer, LoopDetector};
//!
//! let mut graph = PoseGraph::new();
//!
//! // Add poses from keyframes
//! graph.add_node(pose0);
//! graph.add_node(pose1);
//!
//! // Add odometry constraint
//! graph.add_odometry_edge(0, 1, relative_pose, information);
//!
//! // Detect and add loop closures
//! let detector = LoopDetector::new(config);
//! if let Some(closure) = detector.detect(&current_keyframe, &graph) {
//!     graph.add_loop_closure_edge(closure);
//! }
//!
//! // Optimize
//! let optimizer = GraphOptimizer::new(config);
//! let result = optimizer.optimize(&mut graph);
//! ```

mod pose_graph;
mod loop_detector;
mod optimizer;

pub use pose_graph::{
    PoseGraph, PoseNode, PoseEdge, EdgeType, Information2D,
};
pub use loop_detector::{
    LoopDetector, LoopDetectorConfig, LoopClosureCandidate,
};
pub use optimizer::{
    GraphOptimizer, GraphOptimizerConfig, OptimizationResult, TerminationReason,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_module_compiles() {
        let _ = PoseGraph::new();
        let _ = LoopDetectorConfig::default();
        let _ = GraphOptimizerConfig::default();
    }
}
