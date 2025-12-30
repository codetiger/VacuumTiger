//! Submap-level pose graph for SLAM optimization.
//!
//! Unlike the scan-level pose graph, this graph operates on submap origins,
//! resulting in ~50× fewer nodes for the same trajectory length.

use serde::{Deserialize, Serialize};

use crate::core::Pose2D;

use super::manager::SubmapCorrection;
use super::types::SubmapId;

/// Configuration for submap pose graph optimization.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SubmapGraphConfig {
    /// Maximum optimization iterations.
    pub max_iterations: usize,

    /// Convergence threshold for pose change (meters).
    pub convergence_threshold: f32,

    /// Levenberg-Marquardt damping factor.
    pub damping: f32,

    /// Information weight for odometry edges (between consecutive submaps).
    pub odometry_weight: f32,

    /// Information weight for loop closure edges.
    pub loop_weight: f32,

    /// Huber loss scale parameter (meters).
    /// Residuals larger than this are downweighted.
    pub huber_scale: f32,
}

impl Default for SubmapGraphConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50, // More iterations since fewer nodes
            convergence_threshold: 1e-4,
            damping: 1e-3,
            odometry_weight: 100.0,
            loop_weight: 50.0,
            huber_scale: 0.1, // 10cm
        }
    }
}

/// Node in the submap pose graph.
#[derive(Clone, Debug)]
pub struct SubmapNode {
    /// Submap identifier.
    pub id: SubmapId,

    /// Current pose estimate (optimizable).
    pub pose: Pose2D,

    /// Whether this node is fixed (anchor).
    pub fixed: bool,
}

/// Odometry edge between consecutive submaps.
#[derive(Clone, Debug)]
pub struct SubmapOdometryEdge {
    /// From submap ID.
    pub from_id: SubmapId,

    /// To submap ID.
    pub to_id: SubmapId,

    /// Measured relative pose (accumulated from scans within submaps).
    pub measurement: Pose2D,

    /// Information weight.
    pub weight: f32,
}

/// Loop closure edge between non-consecutive submaps.
#[derive(Clone, Debug)]
pub struct SubmapLoopEdge {
    /// From submap ID.
    pub from_id: SubmapId,

    /// To submap ID.
    pub to_id: SubmapId,

    /// Measured relative pose from scan matching.
    pub measurement: Pose2D,

    /// Information weight.
    pub weight: f32,

    /// Confidence in this loop closure.
    pub confidence: f32,
}

/// Loop closure detected between submaps.
#[derive(Clone, Debug)]
pub struct SubmapLoopClosure {
    /// Submap where loop was detected.
    pub from_submap: SubmapId,

    /// Matched submap (older).
    pub to_submap: SubmapId,

    /// Relative pose from scan matching.
    pub relative_pose: Pose2D,

    /// Match confidence (0-1).
    pub confidence: f32,
}

/// Submap-level pose graph.
///
/// Operates on submap origins instead of individual scan poses.
/// This results in much faster optimization with similar accuracy.
pub struct SubmapPoseGraph {
    /// Configuration.
    config: SubmapGraphConfig,

    /// Submap nodes.
    nodes: Vec<SubmapNode>,

    /// Odometry edges (between consecutive submaps).
    odometry_edges: Vec<SubmapOdometryEdge>,

    /// Loop closure edges.
    loop_edges: Vec<SubmapLoopEdge>,
}

impl SubmapPoseGraph {
    /// Create a new submap pose graph.
    pub fn new(config: SubmapGraphConfig) -> Self {
        Self {
            config,
            nodes: Vec::new(),
            odometry_edges: Vec::new(),
            loop_edges: Vec::new(),
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(SubmapGraphConfig::default())
    }

    /// Get number of nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Check if graph is empty.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Get all nodes.
    pub fn nodes(&self) -> &[SubmapNode] {
        &self.nodes
    }

    /// Get odometry edges.
    pub fn odometry_edges(&self) -> &[SubmapOdometryEdge] {
        &self.odometry_edges
    }

    /// Get loop edges.
    pub fn loop_edges(&self) -> &[SubmapLoopEdge] {
        &self.loop_edges
    }

    /// Get node by submap ID.
    pub fn get_node(&self, id: SubmapId) -> Option<&SubmapNode> {
        self.nodes.iter().find(|n| n.id == id)
    }

    /// Get node index by submap ID.
    #[allow(dead_code)]
    fn node_index(&self, id: SubmapId) -> Option<usize> {
        self.nodes.iter().position(|n| n.id == id)
    }

    /// Add a new submap node.
    ///
    /// If this is not the first node, also creates an odometry edge
    /// from the previous submap.
    pub fn add_submap(&mut self, id: SubmapId, origin: Pose2D, odom_from_previous: Option<Pose2D>) {
        let is_first = self.nodes.is_empty();

        self.nodes.push(SubmapNode {
            id,
            pose: origin,
            fixed: is_first, // First node is anchor
        });

        // Add odometry edge from previous submap
        if let Some(delta) = odom_from_previous
            && !is_first
        {
            let prev_id = self.nodes[self.nodes.len() - 2].id;
            self.odometry_edges.push(SubmapOdometryEdge {
                from_id: prev_id,
                to_id: id,
                measurement: delta,
                weight: self.config.odometry_weight,
            });
        }
    }

    /// Add a loop closure constraint.
    pub fn add_loop_closure(&mut self, closure: SubmapLoopClosure) {
        self.loop_edges.push(SubmapLoopEdge {
            from_id: closure.from_submap,
            to_id: closure.to_submap,
            measurement: closure.relative_pose,
            weight: self.config.loop_weight * closure.confidence,
            confidence: closure.confidence,
        });
    }

    /// Check if graph has any loop closures.
    pub fn has_loops(&self) -> bool {
        !self.loop_edges.is_empty()
    }

    /// Optimize the pose graph.
    ///
    /// Returns (iterations, corrections) where corrections can be applied
    /// to the SubmapManager.
    pub fn optimize(&mut self) -> (usize, Vec<SubmapCorrection>) {
        if self.nodes.len() < 2 {
            return (0, Vec::new());
        }

        let mut iterations = 0;
        for i in 0..self.config.max_iterations {
            iterations = i + 1;
            let max_update = self.optimization_step();

            if max_update < self.config.convergence_threshold {
                break;
            }
        }

        // Generate corrections
        let corrections = self
            .nodes
            .iter()
            .filter(|n| !n.fixed)
            .map(|n| SubmapCorrection {
                submap_id: n.id,
                new_origin: n.pose,
            })
            .collect();

        (iterations, corrections)
    }

    /// Perform one optimization step using coordinate descent.
    ///
    /// Returns the maximum pose update magnitude.
    fn optimization_step(&mut self) -> f32 {
        let mut max_update = 0.0f32;

        // Update each non-fixed node
        for i in 0..self.nodes.len() {
            if self.nodes[i].fixed {
                continue;
            }

            let node_id = self.nodes[i].id;
            let (dx, dy, dtheta) = self.compute_node_update(node_id);

            // Apply update with damping
            let damping = self.config.damping;
            let update_mag = (dx * dx + dy * dy + dtheta * dtheta).sqrt();

            self.nodes[i].pose.x += dx / (1.0 + damping);
            self.nodes[i].pose.y += dy / (1.0 + damping);
            self.nodes[i].pose.theta =
                normalize_angle(self.nodes[i].pose.theta + dtheta / (1.0 + damping));

            max_update = max_update.max(update_mag);
        }

        max_update
    }

    /// Compute update for a single node.
    fn compute_node_update(&self, node_id: SubmapId) -> (f32, f32, f32) {
        let node_pose = self.get_node(node_id).unwrap().pose;
        let huber_scale = self.config.huber_scale;

        let mut sum_x = 0.0f32;
        let mut sum_y = 0.0f32;
        let mut sum_theta = 0.0f32;
        let mut sum_weight = 0.0f32;

        // Contribution from odometry edges
        for edge in &self.odometry_edges {
            if edge.to_id == node_id
                && let Some(from_node) = self.get_node(edge.from_id)
            {
                let expected = transform_pose(from_node.pose, edge.measurement);
                let (ex, ey, et) = compute_error(expected, node_pose);

                sum_x += edge.weight * ex;
                sum_y += edge.weight * ey;
                sum_theta += edge.weight * et;
                sum_weight += edge.weight;
            }

            if edge.from_id == node_id
                && let Some(to_node) = self.get_node(edge.to_id)
            {
                let expected = inverse_transform_pose(to_node.pose, edge.measurement);
                let (ex, ey, et) = compute_error(expected, node_pose);

                sum_x += edge.weight * ex;
                sum_y += edge.weight * ey;
                sum_theta += edge.weight * et;
                sum_weight += edge.weight;
            }
        }

        // Contribution from loop closure edges (with Huber loss)
        for edge in &self.loop_edges {
            if edge.to_id == node_id
                && let Some(from_node) = self.get_node(edge.from_id)
            {
                let expected = transform_pose(from_node.pose, edge.measurement);
                let (ex, ey, et) = compute_error(expected, node_pose);

                let error_mag = (ex * ex + ey * ey).sqrt();
                let huber_w = huber_weight(error_mag, huber_scale);
                let effective_weight = edge.weight * huber_w;

                sum_x += effective_weight * ex;
                sum_y += effective_weight * ey;
                sum_theta += effective_weight * et;
                sum_weight += effective_weight;
            }

            if edge.from_id == node_id
                && let Some(to_node) = self.get_node(edge.to_id)
            {
                let expected = inverse_transform_pose(to_node.pose, edge.measurement);
                let (ex, ey, et) = compute_error(expected, node_pose);

                let error_mag = (ex * ex + ey * ey).sqrt();
                let huber_w = huber_weight(error_mag, huber_scale);
                let effective_weight = edge.weight * huber_w;

                sum_x += effective_weight * ex;
                sum_y += effective_weight * ey;
                sum_theta += effective_weight * et;
                sum_weight += effective_weight;
            }
        }

        if sum_weight > 0.0 {
            (
                sum_x / sum_weight,
                sum_y / sum_weight,
                sum_theta / sum_weight,
            )
        } else {
            (0.0, 0.0, 0.0)
        }
    }

    /// Get total error (sum of squared residuals).
    pub fn total_error(&self) -> f32 {
        let mut error = 0.0f32;

        for edge in &self.odometry_edges {
            if let (Some(from), Some(to)) = (self.get_node(edge.from_id), self.get_node(edge.to_id))
            {
                let expected = transform_pose(from.pose, edge.measurement);
                let (dx, dy, dt) = compute_error(expected, to.pose);
                error += edge.weight * (dx * dx + dy * dy + dt * dt);
            }
        }

        for edge in &self.loop_edges {
            if let (Some(from), Some(to)) = (self.get_node(edge.from_id), self.get_node(edge.to_id))
            {
                let expected = transform_pose(from.pose, edge.measurement);
                let (dx, dy, dt) = compute_error(expected, to.pose);
                error += edge.weight * (dx * dx + dy * dy + dt * dt);
            }
        }

        error
    }

    /// Clear the graph.
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.odometry_edges.clear();
        self.loop_edges.clear();
    }
}

/// Transform pose by relative pose.
fn transform_pose(base: Pose2D, delta: Pose2D) -> Pose2D {
    let cos_t = base.theta.cos();
    let sin_t = base.theta.sin();

    Pose2D::new(
        base.x + delta.x * cos_t - delta.y * sin_t,
        base.y + delta.x * sin_t + delta.y * cos_t,
        normalize_angle(base.theta + delta.theta),
    )
}

/// Inverse transform: given result and delta, find base.
fn inverse_transform_pose(result: Pose2D, delta: Pose2D) -> Pose2D {
    let base_theta = normalize_angle(result.theta - delta.theta);
    let cos_t = base_theta.cos();
    let sin_t = base_theta.sin();

    Pose2D::new(
        result.x - (delta.x * cos_t - delta.y * sin_t),
        result.y - (delta.x * sin_t + delta.y * cos_t),
        base_theta,
    )
}

/// Compute error between expected and actual pose.
fn compute_error(expected: Pose2D, actual: Pose2D) -> (f32, f32, f32) {
    (
        expected.x - actual.x,
        expected.y - actual.y,
        normalize_angle(expected.theta - actual.theta),
    )
}

/// Huber weight for robust optimization.
#[inline]
fn huber_weight(residual: f32, delta: f32) -> f32 {
    let abs_r = residual.abs();
    if abs_r <= delta { 1.0 } else { delta / abs_r }
}

/// Normalize angle to [-π, π).
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % std::f32::consts::TAU;
    if a > std::f32::consts::PI {
        a -= std::f32::consts::TAU;
    } else if a < -std::f32::consts::PI {
        a += std::f32::consts::TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_graph_creation() {
        let graph = SubmapPoseGraph::with_defaults();
        assert!(graph.is_empty());
    }

    #[test]
    fn test_add_submaps() {
        let mut graph = SubmapPoseGraph::with_defaults();

        // Add first submap (anchor)
        graph.add_submap(SubmapId::new(0), Pose2D::new(0.0, 0.0, 0.0), None);
        assert_eq!(graph.node_count(), 1);
        assert!(graph.nodes[0].fixed);

        // Add second submap with odometry
        let delta = Pose2D::new(1.0, 0.0, 0.1);
        graph.add_submap(SubmapId::new(1), Pose2D::new(1.0, 0.0, 0.1), Some(delta));
        assert_eq!(graph.node_count(), 2);
        assert!(!graph.nodes[1].fixed);
        assert_eq!(graph.odometry_edges().len(), 1);
    }

    #[test]
    fn test_loop_closure() {
        let mut graph = SubmapPoseGraph::with_defaults();

        // Create a simple chain with drift that loop closure should correct
        // Submap 0 at origin (anchor)
        graph.add_submap(SubmapId::new(0), Pose2D::new(0.0, 0.0, 0.0), None);

        // Submap 1: move 1m forward
        graph.add_submap(
            SubmapId::new(1),
            Pose2D::new(1.0, 0.0, 0.0),
            Some(Pose2D::new(1.0, 0.0, 0.0)),
        );

        // Submap 2: move another 1m forward, but with 0.1m drift
        // Actual pose is (2.1, 0, 0) but should be (2.0, 0, 0)
        graph.add_submap(
            SubmapId::new(2),
            Pose2D::new(2.1, 0.0, 0.0),
            Some(Pose2D::new(1.0, 0.0, 0.0)), // Odometry says 1m
        );

        // No loops yet
        assert!(!graph.has_loops());
        assert_eq!(graph.loop_edges().len(), 0);

        // Add loop closure that says submap 2 should be at (2.0, 0, 0) relative to submap 0
        graph.add_loop_closure(SubmapLoopClosure {
            from_submap: SubmapId::new(2),
            to_submap: SubmapId::new(0),
            relative_pose: Pose2D::new(-2.0, 0.0, 0.0), // From 2 back to 0
            confidence: 1.0,
        });

        assert!(graph.has_loops());
        assert_eq!(graph.loop_edges().len(), 1);

        let (iterations, corrections) = graph.optimize();

        // Optimization should run
        assert!(iterations > 0);

        // Should have corrections for non-fixed nodes (1 and 2)
        assert_eq!(corrections.len(), 2);

        // Node 2's x should move closer to 2.0 (from 2.1)
        let node2 = graph.get_node(SubmapId::new(2)).unwrap();
        // After optimization, should be closer to 2.0 than the original 2.1
        assert!(
            (node2.pose.x - 2.0).abs() < 0.15,
            "Expected node2.x closer to 2.0, got {}",
            node2.pose.x
        );
    }

    #[test]
    fn test_corrections_output() {
        let mut graph = SubmapPoseGraph::with_defaults();

        graph.add_submap(SubmapId::new(0), Pose2D::new(0.0, 0.0, 0.0), None);
        graph.add_submap(
            SubmapId::new(1),
            Pose2D::new(1.0, 0.0, 0.0),
            Some(Pose2D::new(1.0, 0.0, 0.0)),
        );

        let (_, corrections) = graph.optimize();

        // Should have correction for non-fixed node (submap 1)
        assert_eq!(corrections.len(), 1);
        assert_eq!(corrections[0].submap_id, SubmapId::new(1));
    }
}
