//! Pose graph data structure for SLAM backend.
//!
//! A pose graph represents robot trajectory as a graph where:
//! - Nodes are robot poses at different times
//! - Edges are relative pose constraints between nodes

use crate::core::types::Pose2D;
use serde::{Deserialize, Serialize};

/// Information matrix (inverse covariance) for 2D pose.
///
/// Stored as the upper triangle of a 3x3 symmetric matrix:
/// ```text
/// | xx  xy  xt |
/// | xy  yy  yt |
/// | xt  yt  tt |
/// ```
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Information2D {
    /// Information for x-x
    pub xx: f32,
    /// Information for x-y
    pub xy: f32,
    /// Information for x-theta
    pub xt: f32,
    /// Information for y-y
    pub yy: f32,
    /// Information for y-theta
    pub yt: f32,
    /// Information for theta-theta
    pub tt: f32,
}

impl Information2D {
    /// Create a diagonal information matrix.
    pub fn diagonal(xx: f32, yy: f32, tt: f32) -> Self {
        Self {
            xx,
            xy: 0.0,
            xt: 0.0,
            yy,
            yt: 0.0,
            tt,
        }
    }

    /// Create from standard deviations.
    pub fn from_std_dev(sigma_x: f32, sigma_y: f32, sigma_t: f32) -> Self {
        Self::diagonal(
            1.0 / (sigma_x * sigma_x),
            1.0 / (sigma_y * sigma_y),
            1.0 / (sigma_t * sigma_t),
        )
    }

    /// Identity information matrix.
    pub fn identity() -> Self {
        Self::diagonal(1.0, 1.0, 1.0)
    }

    /// Scale all information values.
    pub fn scale(&self, factor: f32) -> Self {
        Self {
            xx: self.xx * factor,
            xy: self.xy * factor,
            xt: self.xt * factor,
            yy: self.yy * factor,
            yt: self.yt * factor,
            tt: self.tt * factor,
        }
    }

    /// Apply information weighting to an error vector.
    ///
    /// Returns Ω * e where Ω is the information matrix.
    pub fn weight_error(&self, error: [f32; 3]) -> [f32; 3] {
        [
            self.xx * error[0] + self.xy * error[1] + self.xt * error[2],
            self.xy * error[0] + self.yy * error[1] + self.yt * error[2],
            self.xt * error[0] + self.yt * error[1] + self.tt * error[2],
        ]
    }

    /// Compute weighted squared error: e^T * Ω * e
    pub fn weighted_squared_error(&self, error: [f32; 3]) -> f32 {
        let we = self.weight_error(error);
        error[0] * we[0] + error[1] * we[1] + error[2] * we[2]
    }
}

impl Default for Information2D {
    fn default() -> Self {
        // Default: 10cm position std dev, 5 degree angle std dev
        Self::from_std_dev(0.1, 0.1, 0.087) // ~5 degrees
    }
}

/// Type of edge in the pose graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EdgeType {
    /// Sequential odometry constraint.
    Odometry,
    /// Loop closure constraint.
    LoopClosure,
    /// Prior constraint (e.g., GPS, known position).
    Prior,
}

/// A node in the pose graph representing a robot pose.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseNode {
    /// Unique node identifier.
    pub id: u64,

    /// Current pose estimate.
    pub pose: Pose2D,

    /// Associated keyframe ID (if any).
    pub keyframe_id: Option<u64>,

    /// Associated submap ID (if any).
    pub submap_id: Option<u64>,

    /// Timestamp in microseconds.
    pub timestamp_us: u64,

    /// Whether this node's pose is fixed (not optimized).
    pub fixed: bool,
}

impl PoseNode {
    /// Create a new pose node.
    pub fn new(id: u64, pose: Pose2D, timestamp_us: u64) -> Self {
        Self {
            id,
            pose,
            keyframe_id: None,
            submap_id: None,
            timestamp_us,
            fixed: false,
        }
    }

    /// Set the keyframe ID.
    pub fn with_keyframe(mut self, keyframe_id: u64) -> Self {
        self.keyframe_id = Some(keyframe_id);
        self
    }

    /// Set the submap ID.
    pub fn with_submap(mut self, submap_id: u64) -> Self {
        self.submap_id = Some(submap_id);
        self
    }

    /// Mark as fixed (won't be optimized).
    pub fn fixed(mut self) -> Self {
        self.fixed = true;
        self
    }
}

/// An edge in the pose graph representing a constraint between poses.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseEdge {
    /// Source node ID.
    pub from: u64,

    /// Target node ID.
    pub to: u64,

    /// Relative pose measurement: T_from^{-1} * T_to
    pub measurement: Pose2D,

    /// Information matrix (inverse covariance).
    pub information: Information2D,

    /// Type of constraint.
    pub edge_type: EdgeType,

    /// Confidence score for this edge (0-1).
    pub confidence: f32,
}

impl PoseEdge {
    /// Create a new edge.
    pub fn new(
        from: u64,
        to: u64,
        measurement: Pose2D,
        information: Information2D,
        edge_type: EdgeType,
    ) -> Self {
        Self {
            from,
            to,
            measurement,
            information,
            edge_type,
            confidence: 1.0,
        }
    }

    /// Create an odometry edge.
    pub fn odometry(from: u64, to: u64, measurement: Pose2D, information: Information2D) -> Self {
        Self::new(from, to, measurement, information, EdgeType::Odometry)
    }

    /// Create a loop closure edge.
    pub fn loop_closure(
        from: u64,
        to: u64,
        measurement: Pose2D,
        information: Information2D,
        confidence: f32,
    ) -> Self {
        let mut edge = Self::new(from, to, measurement, information, EdgeType::LoopClosure);
        edge.confidence = confidence;
        edge
    }

    /// Compute the error for this edge given current node poses.
    ///
    /// Error = T_from^{-1} * T_to - measurement
    pub fn compute_error(&self, pose_from: &Pose2D, pose_to: &Pose2D) -> [f32; 3] {
        // Compute relative pose: T_from^{-1} * T_to
        let predicted = pose_from.inverse().compose(pose_to);

        // Error = predicted - measurement
        [
            predicted.x - self.measurement.x,
            predicted.y - self.measurement.y,
            crate::core::math::angle_diff(self.measurement.theta, predicted.theta),
        ]
    }

    /// Compute weighted squared error.
    pub fn weighted_error(&self, pose_from: &Pose2D, pose_to: &Pose2D) -> f32 {
        let error = self.compute_error(pose_from, pose_to);
        self.information.weighted_squared_error(error)
    }
}

/// Pose graph for SLAM optimization.
#[derive(Debug, Default)]
pub struct PoseGraph {
    /// All nodes in the graph.
    nodes: Vec<PoseNode>,

    /// All edges in the graph.
    edges: Vec<PoseEdge>,

    /// Next node ID.
    next_node_id: u64,

    /// Index mapping node ID to position in nodes vector.
    node_index: std::collections::HashMap<u64, usize>,
}

impl PoseGraph {
    /// Create a new empty pose graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a node to the graph.
    ///
    /// Returns the assigned node ID.
    pub fn add_node(&mut self, pose: Pose2D, timestamp_us: u64) -> u64 {
        let id = self.next_node_id;
        self.next_node_id += 1;

        let node = PoseNode::new(id, pose, timestamp_us);
        self.node_index.insert(id, self.nodes.len());
        self.nodes.push(node);

        id
    }

    /// Add a node with full configuration.
    pub fn add_node_full(&mut self, node: PoseNode) -> u64 {
        let id = node.id;
        if id >= self.next_node_id {
            self.next_node_id = id + 1;
        }
        self.node_index.insert(id, self.nodes.len());
        self.nodes.push(node);
        id
    }

    /// Add an odometry edge between consecutive nodes.
    pub fn add_odometry_edge(
        &mut self,
        from: u64,
        to: u64,
        measurement: Pose2D,
        information: Information2D,
    ) {
        let edge = PoseEdge::odometry(from, to, measurement, information);
        self.edges.push(edge);
    }

    /// Add a loop closure edge.
    pub fn add_loop_closure_edge(
        &mut self,
        from: u64,
        to: u64,
        measurement: Pose2D,
        information: Information2D,
        confidence: f32,
    ) {
        let edge = PoseEdge::loop_closure(from, to, measurement, information, confidence);
        self.edges.push(edge);
    }

    /// Add a generic edge.
    pub fn add_edge(&mut self, edge: PoseEdge) {
        self.edges.push(edge);
    }

    /// Get a node by ID.
    pub fn get_node(&self, id: u64) -> Option<&PoseNode> {
        self.node_index.get(&id).map(|&idx| &self.nodes[idx])
    }

    /// Get index of node by ID.
    pub fn get_node_index(&self, id: u64) -> Option<usize> {
        self.node_index.get(&id).copied()
    }

    /// Get a mutable node by ID.
    pub fn get_node_mut(&mut self, id: u64) -> Option<&mut PoseNode> {
        if let Some(&idx) = self.node_index.get(&id) {
            Some(&mut self.nodes[idx])
        } else {
            None
        }
    }

    /// Get all nodes.
    pub fn nodes(&self) -> &[PoseNode] {
        &self.nodes
    }

    /// Get mutable nodes.
    pub fn nodes_mut(&mut self) -> &mut [PoseNode] {
        &mut self.nodes
    }

    /// Get all edges.
    pub fn edges(&self) -> &[PoseEdge] {
        &self.edges
    }

    /// Get number of nodes.
    pub fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    /// Get number of edges.
    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }

    /// Get number of loop closure edges.
    pub fn num_loop_closures(&self) -> usize {
        self.edges
            .iter()
            .filter(|e| e.edge_type == EdgeType::LoopClosure)
            .count()
    }

    /// Check if graph is empty.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Compute total error across all edges.
    pub fn total_error(&self) -> f32 {
        let mut total = 0.0;

        for edge in &self.edges {
            if let (Some(from), Some(to)) = (self.get_node(edge.from), self.get_node(edge.to)) {
                total += edge.weighted_error(&from.pose, &to.pose);
            }
        }

        total
    }

    /// Get edges connected to a node.
    pub fn edges_for_node(&self, node_id: u64) -> impl Iterator<Item = &PoseEdge> {
        self.edges
            .iter()
            .filter(move |e| e.from == node_id || e.to == node_id)
    }

    /// Get the latest node (most recent by ID).
    pub fn latest_node(&self) -> Option<&PoseNode> {
        self.nodes.last()
    }

    /// Clear the graph.
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.edges.clear();
        self.node_index.clear();
        self.next_node_id = 0;
    }

    /// Update a node's pose.
    pub fn update_pose(&mut self, id: u64, new_pose: Pose2D) {
        if let Some(node) = self.get_node_mut(id) {
            node.pose = new_pose;
        }
    }

    /// Get all optimized poses as a map from node ID to pose.
    pub fn poses(&self) -> std::collections::HashMap<u64, Pose2D> {
        self.nodes
            .iter()
            .map(|n| (n.id, n.pose))
            .collect()
    }

    /// Get poses for nodes with submap IDs.
    pub fn submap_poses(&self) -> Vec<(u64, Pose2D)> {
        self.nodes
            .iter()
            .filter_map(|n| n.submap_id.map(|sid| (sid, n.pose)))
            .collect()
    }

    /// Fix the first node (anchor for optimization).
    pub fn fix_first_node(&mut self) {
        if let Some(node) = self.nodes.first_mut() {
            node.fixed = true;
        }
    }

    /// Count non-fixed nodes.
    pub fn num_free_nodes(&self) -> usize {
        self.nodes.iter().filter(|n| !n.fixed).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_information_diagonal() {
        let info = Information2D::diagonal(100.0, 100.0, 400.0);

        assert_eq!(info.xx, 100.0);
        assert_eq!(info.yy, 100.0);
        assert_eq!(info.tt, 400.0);
        assert_eq!(info.xy, 0.0);
    }

    #[test]
    fn test_information_from_std_dev() {
        let info = Information2D::from_std_dev(0.1, 0.1, 0.1);

        // 1 / (0.1^2) = 100
        assert_relative_eq!(info.xx, 100.0, epsilon = 0.1);
        assert_relative_eq!(info.yy, 100.0, epsilon = 0.1);
        assert_relative_eq!(info.tt, 100.0, epsilon = 0.1);
    }

    #[test]
    fn test_information_weighted_error() {
        let info = Information2D::diagonal(1.0, 1.0, 1.0);
        let error = [1.0, 2.0, 0.5];

        let chi2 = info.weighted_squared_error(error);

        // Should be 1^2 + 2^2 + 0.5^2 = 1 + 4 + 0.25 = 5.25
        assert_relative_eq!(chi2, 5.25, epsilon = 0.01);
    }

    #[test]
    fn test_pose_graph_add_nodes() {
        let mut graph = PoseGraph::new();

        let id0 = graph.add_node(Pose2D::identity(), 0);
        let id1 = graph.add_node(Pose2D::new(1.0, 0.0, 0.0), 1000);

        assert_eq!(id0, 0);
        assert_eq!(id1, 1);
        assert_eq!(graph.num_nodes(), 2);
    }

    #[test]
    fn test_pose_graph_add_edges() {
        let mut graph = PoseGraph::new();

        let id0 = graph.add_node(Pose2D::identity(), 0);
        let id1 = graph.add_node(Pose2D::new(1.0, 0.0, 0.0), 1000);

        let measurement = Pose2D::new(1.0, 0.0, 0.0);
        let info = Information2D::default();

        graph.add_odometry_edge(id0, id1, measurement, info);

        assert_eq!(graph.num_edges(), 1);
        assert_eq!(graph.edges()[0].edge_type, EdgeType::Odometry);
    }

    #[test]
    fn test_edge_error_computation() {
        let pose_from = Pose2D::identity();
        let pose_to = Pose2D::new(1.0, 0.0, 0.0);
        let measurement = Pose2D::new(1.0, 0.0, 0.0);

        let edge = PoseEdge::odometry(0, 1, measurement, Information2D::default());
        let error = edge.compute_error(&pose_from, &pose_to);

        // Perfect measurement should have zero error
        assert_relative_eq!(error[0], 0.0, epsilon = 1e-6);
        assert_relative_eq!(error[1], 0.0, epsilon = 1e-6);
        assert_relative_eq!(error[2], 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_edge_error_with_noise() {
        let pose_from = Pose2D::identity();
        let pose_to = Pose2D::new(1.1, 0.1, 0.05); // Actual pose with noise
        let measurement = Pose2D::new(1.0, 0.0, 0.0); // Expected measurement

        let edge = PoseEdge::odometry(0, 1, measurement, Information2D::default());
        let error = edge.compute_error(&pose_from, &pose_to);

        // Error should be (0.1, 0.1, 0.05)
        assert_relative_eq!(error[0], 0.1, epsilon = 0.01);
        assert_relative_eq!(error[1], 0.1, epsilon = 0.01);
        assert_relative_eq!(error[2], 0.05, epsilon = 0.01);
    }

    #[test]
    fn test_graph_total_error() {
        let mut graph = PoseGraph::new();

        // Perfect graph - zero error
        let id0 = graph.add_node(Pose2D::identity(), 0);
        let id1 = graph.add_node(Pose2D::new(1.0, 0.0, 0.0), 1000);

        let measurement = Pose2D::new(1.0, 0.0, 0.0);
        graph.add_odometry_edge(id0, id1, measurement, Information2D::diagonal(1.0, 1.0, 1.0));

        let error = graph.total_error();
        assert_relative_eq!(error, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_graph_with_error() {
        let mut graph = PoseGraph::new();

        // Graph with error - the poses don't quite satisfy the loop closure constraint
        let id0 = graph.add_node(Pose2D::identity(), 0);
        let id1 = graph.add_node(Pose2D::new(1.0, 0.0, 0.0), 1000);
        let id2 = graph.add_node(Pose2D::new(1.0, 1.0, 0.0), 2000);

        // Odometry edges
        graph.add_odometry_edge(id0, id1, Pose2D::new(1.0, 0.0, 0.0), Information2D::identity());
        graph.add_odometry_edge(id1, id2, Pose2D::new(0.0, 1.0, 0.0), Information2D::identity());

        // Loop closure with error - measurement says (-1.1, -0.9) but actual is (-1, -1)
        // This creates a residual error that optimization should minimize
        graph.add_loop_closure_edge(
            id2,
            id0,
            Pose2D::new(-1.1, -0.9, 0.0),  // Slightly different from actual
            Information2D::identity(),
            0.9,
        );

        let error = graph.total_error();
        // Should be non-zero because measurement doesn't match actual relative pose
        assert!(error > 0.0, "Expected non-zero error, got {}", error);
    }

    #[test]
    fn test_fix_first_node() {
        let mut graph = PoseGraph::new();

        graph.add_node(Pose2D::identity(), 0);
        graph.add_node(Pose2D::new(1.0, 0.0, 0.0), 1000);

        assert!(!graph.nodes()[0].fixed);

        graph.fix_first_node();

        assert!(graph.nodes()[0].fixed);
        assert_eq!(graph.num_free_nodes(), 1);
    }

    #[test]
    fn test_loop_closure_count() {
        let mut graph = PoseGraph::new();

        let id0 = graph.add_node(Pose2D::identity(), 0);
        let id1 = graph.add_node(Pose2D::new(1.0, 0.0, 0.0), 1000);
        let id2 = graph.add_node(Pose2D::new(2.0, 0.0, 0.0), 2000);

        graph.add_odometry_edge(id0, id1, Pose2D::new(1.0, 0.0, 0.0), Information2D::identity());
        graph.add_odometry_edge(id1, id2, Pose2D::new(1.0, 0.0, 0.0), Information2D::identity());
        graph.add_loop_closure_edge(
            id2,
            id0,
            Pose2D::new(-2.0, 0.0, 0.0),
            Information2D::identity(),
            0.9,
        );

        assert_eq!(graph.num_loop_closures(), 1);
        assert_eq!(graph.num_edges(), 3);
    }
}
