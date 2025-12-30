//! Pose graph for SLAM with Gauss-Newton optimization.

use serde::{Deserialize, Serialize};

use crate::core::Pose2D;

use super::detector::LoopClosure;

/// Configuration for pose graph optimization.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseGraphConfig {
    /// Maximum optimization iterations.
    #[serde(default = "default_max_iterations")]
    pub max_iterations: usize,

    /// Convergence threshold for pose change.
    #[serde(default = "default_convergence_threshold")]
    pub convergence_threshold: f32,

    /// Levenberg-Marquardt damping factor.
    #[serde(default = "default_damping")]
    pub damping: f32,

    /// Information matrix weight for odometry edges.
    #[serde(default = "default_odom_weight")]
    pub odometry_weight: f32,

    /// Information matrix weight for loop closure edges.
    #[serde(default = "default_loop_weight")]
    pub loop_weight: f32,

    /// Huber loss scale parameter (meters).
    /// Residuals larger than this are downweighted.
    /// Typical: 0.1 (10cm) - effectively ignores constraints with >10cm error.
    #[serde(default = "default_huber_scale")]
    pub huber_scale: f32,
}

fn default_max_iterations() -> usize {
    20
}

fn default_convergence_threshold() -> f32 {
    1e-4
}

fn default_damping() -> f32 {
    1e-3
}

fn default_odom_weight() -> f32 {
    100.0
}

fn default_loop_weight() -> f32 {
    50.0
}

fn default_huber_scale() -> f32 {
    0.1 // 10cm - downweights constraints with error > 10cm
}

impl Default for PoseGraphConfig {
    fn default() -> Self {
        Self {
            max_iterations: default_max_iterations(),
            convergence_threshold: default_convergence_threshold(),
            damping: default_damping(),
            odometry_weight: default_odom_weight(),
            loop_weight: default_loop_weight(),
            huber_scale: default_huber_scale(),
        }
    }
}

/// Odometry edge (constraint between consecutive poses).
#[derive(Clone, Debug)]
pub struct OdometryEdge {
    /// Index of the "from" pose.
    pub from_idx: usize,
    /// Index of the "to" pose.
    pub to_idx: usize,
    /// Measured relative pose (from → to).
    pub measurement: Pose2D,
    /// Information weight (inverse covariance).
    pub weight: f32,
}

/// Loop closure edge (constraint between non-consecutive poses).
#[derive(Clone, Debug)]
pub struct LoopEdge {
    /// Index of the "from" pose.
    pub from_idx: usize,
    /// Index of the "to" pose.
    pub to_idx: usize,
    /// Measured relative pose (from → to).
    pub measurement: Pose2D,
    /// Information weight (inverse covariance).
    pub weight: f32,
    /// Confidence in this loop closure.
    pub confidence: f32,
}

/// Pose graph for SLAM.
///
/// Stores robot poses and constraints (odometry + loop closures).
/// Supports optimization to correct accumulated drift.
pub struct PoseGraph {
    /// Configuration.
    config: PoseGraphConfig,
    /// Poses (nodes).
    poses: Vec<Pose2D>,
    /// Odometry edges (sequential constraints).
    odometry_edges: Vec<OdometryEdge>,
    /// Loop closure edges.
    loop_edges: Vec<LoopEdge>,
}

impl PoseGraph {
    /// Create a new pose graph.
    pub fn new(config: PoseGraphConfig) -> Self {
        Self {
            config,
            poses: Vec::new(),
            odometry_edges: Vec::new(),
            loop_edges: Vec::new(),
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(PoseGraphConfig::default())
    }

    /// Get number of poses in the graph.
    pub fn len(&self) -> usize {
        self.poses.len()
    }

    /// Check if graph is empty.
    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }

    /// Get pose at index.
    pub fn get_pose(&self, idx: usize) -> Option<Pose2D> {
        self.poses.get(idx).copied()
    }

    /// Get all poses.
    pub fn poses(&self) -> &[Pose2D] {
        &self.poses
    }

    /// Get odometry edges.
    pub fn odometry_edges(&self) -> &[OdometryEdge] {
        &self.odometry_edges
    }

    /// Get loop closure edges.
    pub fn loop_edges(&self) -> &[LoopEdge] {
        &self.loop_edges
    }

    /// Add a new pose with odometry constraint from previous pose.
    ///
    /// # Arguments
    /// * `pose` - The new pose in world frame
    /// * `odom_delta` - Relative pose from previous to this pose
    ///
    /// # Returns
    /// Index of the new pose.
    pub fn add_pose(&mut self, pose: Pose2D, odom_delta: Option<Pose2D>) -> usize {
        let new_idx = self.poses.len();
        self.poses.push(pose);

        // Add odometry edge if not first pose
        if let Some(delta) = odom_delta
            && new_idx > 0
        {
            self.odometry_edges.push(OdometryEdge {
                from_idx: new_idx - 1,
                to_idx: new_idx,
                measurement: delta,
                weight: self.config.odometry_weight,
            });
        }

        new_idx
    }

    /// Add a loop closure constraint.
    pub fn add_loop_closure(&mut self, closure: LoopClosure) {
        self.loop_edges.push(LoopEdge {
            from_idx: closure.from_idx,
            to_idx: closure.to_idx,
            measurement: closure.relative_pose,
            weight: self.config.loop_weight * closure.confidence,
            confidence: closure.confidence,
        });
    }

    /// Check if graph has any loop closures.
    pub fn has_loops(&self) -> bool {
        !self.loop_edges.is_empty()
    }

    /// Optimize the pose graph using Gauss-Newton.
    ///
    /// Returns the number of iterations performed.
    pub fn optimize(&mut self) -> usize {
        if self.poses.len() < 2 {
            return 0;
        }

        for iteration in 0..self.config.max_iterations {
            let max_update = self.optimization_step();

            if max_update < self.config.convergence_threshold {
                return iteration + 1;
            }
        }

        self.config.max_iterations
    }

    /// Perform one optimization step.
    ///
    /// Returns the maximum pose update magnitude.
    fn optimization_step(&mut self) -> f32 {
        let n = self.poses.len();
        if n < 2 {
            return 0.0;
        }

        // Build linear system Hx = b
        // For simplicity, we use a coordinate descent approach:
        // Update each pose based on its constraints

        let mut max_update = 0.0f32;

        // Skip pose 0 (anchor - fixed at origin)
        for i in 1..n {
            let (dx, dy, dtheta) = self.compute_pose_update(i);

            // Apply update with damping
            let damping = self.config.damping;
            let update_mag = (dx * dx + dy * dy + dtheta * dtheta).sqrt();

            self.poses[i].x += dx / (1.0 + damping);
            self.poses[i].y += dy / (1.0 + damping);
            self.poses[i].theta += dtheta / (1.0 + damping);

            // Normalize theta
            self.poses[i].theta = normalize_angle(self.poses[i].theta);

            max_update = max_update.max(update_mag);
        }

        max_update
    }

    /// Compute update for a single pose.
    ///
    /// Uses Huber loss weighting for loop closure edges to make
    /// optimization robust to outliers (false loop closures).
    fn compute_pose_update(&self, pose_idx: usize) -> (f32, f32, f32) {
        let mut sum_x = 0.0f32;
        let mut sum_y = 0.0f32;
        let mut sum_theta = 0.0f32;
        let mut sum_weight = 0.0f32;

        let huber_scale = self.config.huber_scale;

        // Contribution from odometry edges (no Huber - odometry is reliable)
        for edge in &self.odometry_edges {
            if edge.to_idx == pose_idx {
                // This pose is the "to" end
                let from_pose = self.poses[edge.from_idx];
                let expected = self.transform_pose(from_pose, edge.measurement);

                let error_x = expected.x - self.poses[pose_idx].x;
                let error_y = expected.y - self.poses[pose_idx].y;
                let error_theta = normalize_angle(expected.theta - self.poses[pose_idx].theta);

                sum_x += edge.weight * error_x;
                sum_y += edge.weight * error_y;
                sum_theta += edge.weight * error_theta;
                sum_weight += edge.weight;
            }

            if edge.from_idx == pose_idx {
                // This pose is the "from" end
                let to_pose = self.poses[edge.to_idx];
                let expected = self.inverse_transform_pose(to_pose, edge.measurement);

                let error_x = expected.x - self.poses[pose_idx].x;
                let error_y = expected.y - self.poses[pose_idx].y;
                let error_theta = normalize_angle(expected.theta - self.poses[pose_idx].theta);

                sum_x += edge.weight * error_x;
                sum_y += edge.weight * error_y;
                sum_theta += edge.weight * error_theta;
                sum_weight += edge.weight;
            }
        }

        // Contribution from loop closure edges (with Huber loss)
        for edge in &self.loop_edges {
            if edge.to_idx == pose_idx {
                let from_pose = self.poses[edge.from_idx];
                let expected = self.transform_pose(from_pose, edge.measurement);

                let error_x = expected.x - self.poses[pose_idx].x;
                let error_y = expected.y - self.poses[pose_idx].y;
                let error_theta = normalize_angle(expected.theta - self.poses[pose_idx].theta);

                // Compute Huber weight based on translation error magnitude
                let error_mag = (error_x * error_x + error_y * error_y).sqrt();
                let huber_w = huber_weight(error_mag, huber_scale);
                let effective_weight = edge.weight * huber_w;

                sum_x += effective_weight * error_x;
                sum_y += effective_weight * error_y;
                sum_theta += effective_weight * error_theta;
                sum_weight += effective_weight;
            }

            if edge.from_idx == pose_idx {
                let to_pose = self.poses[edge.to_idx];
                let expected = self.inverse_transform_pose(to_pose, edge.measurement);

                let error_x = expected.x - self.poses[pose_idx].x;
                let error_y = expected.y - self.poses[pose_idx].y;
                let error_theta = normalize_angle(expected.theta - self.poses[pose_idx].theta);

                // Compute Huber weight based on translation error magnitude
                let error_mag = (error_x * error_x + error_y * error_y).sqrt();
                let huber_w = huber_weight(error_mag, huber_scale);
                let effective_weight = edge.weight * huber_w;

                sum_x += effective_weight * error_x;
                sum_y += effective_weight * error_y;
                sum_theta += effective_weight * error_theta;
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

    /// Transform a pose by a relative pose.
    fn transform_pose(&self, base: Pose2D, delta: Pose2D) -> Pose2D {
        let cos_theta = base.theta.cos();
        let sin_theta = base.theta.sin();

        Pose2D::new(
            base.x + delta.x * cos_theta - delta.y * sin_theta,
            base.y + delta.x * sin_theta + delta.y * cos_theta,
            normalize_angle(base.theta + delta.theta),
        )
    }

    /// Inverse transform: given result and delta, find base.
    fn inverse_transform_pose(&self, result: Pose2D, delta: Pose2D) -> Pose2D {
        let base_theta = normalize_angle(result.theta - delta.theta);
        let cos_theta = base_theta.cos();
        let sin_theta = base_theta.sin();

        Pose2D::new(
            result.x - (delta.x * cos_theta - delta.y * sin_theta),
            result.y - (delta.x * sin_theta + delta.y * cos_theta),
            base_theta,
        )
    }

    /// Get the total error (sum of squared residuals).
    pub fn total_error(&self) -> f32 {
        let mut error = 0.0f32;

        for edge in &self.odometry_edges {
            let from_pose = self.poses[edge.from_idx];
            let to_pose = self.poses[edge.to_idx];
            let expected = self.transform_pose(from_pose, edge.measurement);

            let dx = expected.x - to_pose.x;
            let dy = expected.y - to_pose.y;
            let dtheta = normalize_angle(expected.theta - to_pose.theta);

            error += edge.weight * (dx * dx + dy * dy + dtheta * dtheta);
        }

        for edge in &self.loop_edges {
            let from_pose = self.poses[edge.from_idx];
            let to_pose = self.poses[edge.to_idx];
            let expected = self.transform_pose(from_pose, edge.measurement);

            let dx = expected.x - to_pose.x;
            let dy = expected.y - to_pose.y;
            let dtheta = normalize_angle(expected.theta - to_pose.theta);

            error += edge.weight * (dx * dx + dy * dy + dtheta * dtheta);
        }

        error
    }

    /// Clear the graph.
    pub fn reset(&mut self) {
        self.poses.clear();
        self.odometry_edges.clear();
        self.loop_edges.clear();
    }
}

/// Huber weight function for robust optimization.
///
/// Returns a weight that is:
/// - 1.0 when |residual| <= delta (quadratic region)
/// - delta / |residual| when |residual| > delta (linear region)
///
/// This downweights large residuals (outliers) to prevent them
/// from dominating the optimization.
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
        let graph = PoseGraph::with_defaults();
        assert!(graph.is_empty());
    }

    #[test]
    fn test_add_poses() {
        let mut graph = PoseGraph::with_defaults();

        // Add first pose (no odometry)
        let idx0 = graph.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        assert_eq!(idx0, 0);
        assert_eq!(graph.len(), 1);

        // Add second pose with odometry
        let delta = Pose2D::new(1.0, 0.0, 0.1);
        let idx1 = graph.add_pose(Pose2D::new(1.0, 0.0, 0.1), Some(delta));
        assert_eq!(idx1, 1);
        assert_eq!(graph.len(), 2);
        assert_eq!(graph.odometry_edges().len(), 1);
    }

    #[test]
    fn test_get_pose() {
        let mut graph = PoseGraph::with_defaults();
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        graph.add_pose(pose, None);

        let retrieved = graph.get_pose(0).unwrap();
        assert!((retrieved.x - 1.0).abs() < 0.001);
        assert!((retrieved.y - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(std::f32::consts::PI * 2.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(std::f32::consts::PI * 3.0) - std::f32::consts::PI).abs() < 0.001);
    }

    #[test]
    fn test_optimization_no_loops() {
        let mut graph = PoseGraph::with_defaults();

        // Add a chain of poses
        graph.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        graph.add_pose(Pose2D::new(1.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));
        graph.add_pose(Pose2D::new(2.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));

        let iterations = graph.optimize();

        // Should converge quickly with consistent odometry
        assert!(iterations < 5);
    }

    #[test]
    fn test_loop_closure() {
        let mut graph = PoseGraph::with_defaults();

        // Create a square path
        graph.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        graph.add_pose(
            Pose2D::new(1.0, 0.0, std::f32::consts::FRAC_PI_2),
            Some(Pose2D::new(1.0, 0.0, std::f32::consts::FRAC_PI_2)),
        );
        graph.add_pose(
            Pose2D::new(1.0, 1.0, std::f32::consts::PI),
            Some(Pose2D::new(1.0, 0.0, std::f32::consts::FRAC_PI_2)),
        );
        graph.add_pose(
            Pose2D::new(0.0, 1.0, -std::f32::consts::FRAC_PI_2),
            Some(Pose2D::new(1.0, 0.0, std::f32::consts::FRAC_PI_2)),
        );

        // Add drift (accumulated error)
        graph.poses[3].x = 0.1; // Should be 0.0

        // Add loop closure
        let closure = LoopClosure {
            from_idx: 3,
            to_idx: 0,
            relative_pose: Pose2D::new(0.0, -1.0, std::f32::consts::FRAC_PI_2),
            confidence: 1.0,
            descriptor_distance: 10,
            rotation_offset: 0,
        };
        graph.add_loop_closure(closure);

        assert!(graph.has_loops());

        let error_before = graph.total_error();
        graph.optimize();
        let error_after = graph.total_error();

        // Error should decrease after optimization
        assert!(error_after <= error_before);
    }

    #[test]
    fn test_reset() {
        let mut graph = PoseGraph::with_defaults();
        graph.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        graph.add_pose(Pose2D::new(1.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));

        graph.reset();
        assert!(graph.is_empty());
        assert!(graph.odometry_edges().is_empty());
    }
}
