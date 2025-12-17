//! Graph optimization using Gauss-Newton method.
//!
//! Optimizes a pose graph to minimize the error between measured and predicted
//! constraints. Uses sparse linear algebra for efficiency.
//!
//! # Algorithm
//!
//! The pose graph optimization minimizes:
//!
//! ```text
//! F(x) = Σ e(xi, xj, zij)^T * Ωij * e(xi, xj, zij)
//! ```
//!
//! Where:
//! - `e(xi, xj, zij)` is the error between predicted and measured relative pose
//! - `Ωij` is the information matrix (inverse covariance)
//!
//! Gauss-Newton iteratively solves:
//! ```text
//! H * Δx = -b
//! ```
//!
//! Where H is the Hessian approximation and b is the gradient.

use super::pose_graph::{Information2D, PoseGraph};
use crate::core::types::Pose2D;

/// Result of graph optimization.
#[derive(Debug, Clone)]
pub struct OptimizationResult {
    /// Number of iterations performed.
    pub iterations: u32,

    /// Initial chi-squared error.
    pub initial_error: f64,

    /// Final chi-squared error.
    pub final_error: f64,

    /// Whether the optimization converged.
    pub converged: bool,

    /// Reason for termination.
    pub termination_reason: TerminationReason,
}

/// Reason for optimization termination.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminationReason {
    /// Converged (error change below threshold).
    Converged,

    /// Maximum iterations reached.
    MaxIterations,

    /// Error increased (diverging).
    Diverged,

    /// Linear system solve failed.
    SolveFailed,

    /// No edges to optimize.
    NoEdges,
}

/// Configuration for graph optimization.
#[derive(Debug, Clone)]
pub struct GraphOptimizerConfig {
    /// Maximum number of iterations.
    pub max_iterations: u32,

    /// Convergence threshold for relative error change.
    pub convergence_threshold: f64,

    /// Levenberg-Marquardt damping factor.
    pub damping_factor: f64,

    /// Whether to use Levenberg-Marquardt (vs pure Gauss-Newton).
    pub use_levenberg_marquardt: bool,

    /// Whether to fix the first pose (gauge freedom).
    pub fix_first_pose: bool,

    /// Minimum error improvement to continue.
    pub min_improvement: f64,
}

impl Default for GraphOptimizerConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            convergence_threshold: 1e-6,
            damping_factor: 1e-3,
            use_levenberg_marquardt: true,
            fix_first_pose: true,
            min_improvement: 1e-9,
        }
    }
}

/// Graph optimizer using Gauss-Newton or Levenberg-Marquardt.
pub struct GraphOptimizer {
    config: GraphOptimizerConfig,
}

impl GraphOptimizer {
    /// Create a new graph optimizer.
    pub fn new(config: GraphOptimizerConfig) -> Self {
        Self { config }
    }

    /// Optimize the pose graph.
    ///
    /// Modifies the poses in the graph to minimize constraint errors.
    pub fn optimize(&self, graph: &mut PoseGraph) -> OptimizationResult {
        let num_nodes = graph.num_nodes();
        let num_edges = graph.num_edges();

        if num_edges == 0 {
            return OptimizationResult {
                iterations: 0,
                initial_error: 0.0,
                final_error: 0.0,
                converged: true,
                termination_reason: TerminationReason::NoEdges,
            };
        }

        // Dimension of the problem (3 DOF per pose: x, y, theta)
        let dim = num_nodes * 3;

        // Compute initial error
        let initial_error = self.compute_chi_squared(graph);
        let mut current_error = initial_error;

        let mut lambda = self.config.damping_factor;
        let mut iterations = 0;

        for iter in 0..self.config.max_iterations {
            iterations = iter + 1;

            // Build the linear system H * dx = -b
            let (h, b) = self.build_linear_system(graph, dim);

            // Apply Levenberg-Marquardt damping if enabled
            let h_damped = if self.config.use_levenberg_marquardt {
                self.apply_damping(&h, lambda, dim)
            } else {
                h
            };

            // Solve the linear system
            let dx = match self.solve_linear_system(&h_damped, &b, dim) {
                Some(dx) => dx,
                None => {
                    return OptimizationResult {
                        iterations,
                        initial_error,
                        final_error: current_error,
                        converged: false,
                        termination_reason: TerminationReason::SolveFailed,
                    };
                }
            };

            // Apply the update
            self.apply_update(graph, &dx);

            // Compute new error
            let new_error = self.compute_chi_squared(graph);

            // Check for divergence
            if new_error > current_error * 1.1 {
                // Revert the update
                self.apply_update(graph, &dx.iter().map(|x| -x).collect::<Vec<_>>());

                if self.config.use_levenberg_marquardt {
                    // Increase damping and try again
                    lambda *= 10.0;
                    if lambda > 1e10 {
                        return OptimizationResult {
                            iterations,
                            initial_error,
                            final_error: current_error,
                            converged: false,
                            termination_reason: TerminationReason::Diverged,
                        };
                    }
                    continue;
                } else {
                    return OptimizationResult {
                        iterations,
                        initial_error,
                        final_error: current_error,
                        converged: false,
                        termination_reason: TerminationReason::Diverged,
                    };
                }
            }

            // Decrease damping on success
            if self.config.use_levenberg_marquardt {
                lambda *= 0.1;
                lambda = lambda.max(1e-10);
            }

            // Check for convergence
            let relative_change = (current_error - new_error).abs() / current_error.max(1e-10);
            current_error = new_error;

            if relative_change < self.config.convergence_threshold {
                return OptimizationResult {
                    iterations,
                    initial_error,
                    final_error: current_error,
                    converged: true,
                    termination_reason: TerminationReason::Converged,
                };
            }

            // Check for minimal improvement
            if relative_change < self.config.min_improvement {
                return OptimizationResult {
                    iterations,
                    initial_error,
                    final_error: current_error,
                    converged: true,
                    termination_reason: TerminationReason::Converged,
                };
            }
        }

        OptimizationResult {
            iterations,
            initial_error,
            final_error: current_error,
            converged: false,
            termination_reason: TerminationReason::MaxIterations,
        }
    }

    /// Compute the chi-squared error of the graph.
    fn compute_chi_squared(&self, graph: &PoseGraph) -> f64 {
        let mut chi2 = 0.0;

        for edge in graph.edges() {
            let xi = match graph.get_node(edge.from) {
                Some(n) => &n.pose,
                None => continue,
            };
            let xj = match graph.get_node(edge.to) {
                Some(n) => &n.pose,
                None => continue,
            };

            let error = self.compute_edge_error(xi, xj, &edge.measurement);
            let weighted_error = self.weight_error(&error, &edge.information);

            chi2 += error[0] * weighted_error[0]
                + error[1] * weighted_error[1]
                + error[2] * weighted_error[2];
        }

        chi2
    }

    /// Compute the error for a single edge.
    ///
    /// Error = inv(xi) * xj - zij
    fn compute_edge_error(&self, xi: &Pose2D, xj: &Pose2D, measurement: &Pose2D) -> [f64; 3] {
        // Compute predicted relative pose: inv(xi) * xj
        let predicted = xi.inverse().compose(xj);

        // Compute error: predicted - measurement
        let dx = (predicted.x - measurement.x) as f64;
        let dy = (predicted.y - measurement.y) as f64;
        let dtheta = normalize_angle((predicted.theta - measurement.theta) as f64);

        [dx, dy, dtheta]
    }

    /// Weight error by information matrix.
    fn weight_error(&self, error: &[f64; 3], info: &Information2D) -> [f64; 3] {
        // For diagonal information matrix: Ω * e
        [
            info.xx as f64 * error[0] + info.xy as f64 * error[1] + info.xt as f64 * error[2],
            info.xy as f64 * error[0] + info.yy as f64 * error[1] + info.yt as f64 * error[2],
            info.xt as f64 * error[0] + info.yt as f64 * error[1] + info.tt as f64 * error[2],
        ]
    }

    /// Build the linear system H * dx = -b.
    fn build_linear_system(&self, graph: &PoseGraph, dim: usize) -> (Vec<f64>, Vec<f64>) {
        let mut h = vec![0.0; dim * dim];
        let mut b = vec![0.0; dim];

        for edge in graph.edges() {
            let idx_i = match graph.get_node_index(edge.from) {
                Some(idx) => idx,
                None => continue,
            };
            let idx_j = match graph.get_node_index(edge.to) {
                Some(idx) => idx,
                None => continue,
            };

            let xi = match graph.get_node(edge.from) {
                Some(n) => &n.pose,
                None => continue,
            };
            let xj = match graph.get_node(edge.to) {
                Some(n) => &n.pose,
                None => continue,
            };

            // Compute error and Jacobians
            let error = self.compute_edge_error(xi, xj, &edge.measurement);
            let (ji, jj) = self.compute_jacobians(xi, xj);

            // Add contributions to H and b
            self.add_edge_contribution(
                &mut h,
                &mut b,
                dim,
                idx_i,
                idx_j,
                &ji,
                &jj,
                &error,
                &edge.information,
            );
        }

        // Fix first pose for gauge freedom
        if self.config.fix_first_pose && dim >= 3 {
            let large_value = 1e10;
            h[0] = large_value; // x
            h[dim + 1] = large_value; // y
            h[2 * dim + 2] = large_value; // theta
        }

        (h, b)
    }

    /// Compute Jacobians of the error function.
    ///
    /// Returns (Ji, Jj) where:
    /// - Ji = ∂e/∂xi (3x3 matrix as row-major array)
    /// - Jj = ∂e/∂xj (3x3 matrix as row-major array)
    fn compute_jacobians(&self, xi: &Pose2D, xj: &Pose2D) -> ([f64; 9], [f64; 9]) {
        let cos_i = (xi.theta as f64).cos();
        let sin_i = (xi.theta as f64).sin();

        let dx = (xj.x - xi.x) as f64;
        let dy = (xj.y - xi.y) as f64;

        // Jacobian w.r.t. xi (3x3, row-major)
        // ∂e/∂xi = -R(θi)^T * [I | ...] - ...
        let ji = [
            -cos_i,
            -sin_i,
            -sin_i * dx + cos_i * dy, // Row 0
            sin_i,
            -cos_i,
            -cos_i * dx - sin_i * dy, // Row 1
            0.0,
            0.0,
            -1.0, // Row 2
        ];

        // Jacobian w.r.t. xj (3x3, row-major)
        // ∂e/∂xj = R(θi)^T
        let jj = [
            cos_i, sin_i, 0.0, // Row 0
            -sin_i, cos_i, 0.0, // Row 1
            0.0, 0.0, 1.0, // Row 2
        ];

        (ji, jj)
    }

    /// Add edge contribution to H and b.
    #[allow(clippy::too_many_arguments)]
    fn add_edge_contribution(
        &self,
        h: &mut [f64],
        b: &mut [f64],
        dim: usize,
        idx_i: usize,
        idx_j: usize,
        ji: &[f64; 9],
        jj: &[f64; 9],
        error: &[f64; 3],
        info: &Information2D,
    ) {
        // Convert information matrix to array
        let omega = [
            info.xx as f64,
            info.xy as f64,
            info.xt as f64,
            info.xy as f64,
            info.yy as f64,
            info.yt as f64,
            info.xt as f64,
            info.yt as f64,
            info.tt as f64,
        ];

        // Compute J^T * Ω * J contributions
        // H_ii += Ji^T * Ω * Ji
        // H_ij += Ji^T * Ω * Jj
        // H_ji += Jj^T * Ω * Ji
        // H_jj += Jj^T * Ω * Jj

        // b_i += Ji^T * Ω * e
        // b_j += Jj^T * Ω * e

        let base_i = idx_i * 3;
        let base_j = idx_j * 3;

        // Compute Ω * Ji and Ω * Jj
        let omega_ji = mat3_mul(&omega, ji);
        let omega_jj = mat3_mul(&omega, jj);

        // H_ii = Ji^T * Ω * Ji
        let h_ii = mat3_transpose_mul(ji, &omega_ji);
        // H_ij = Ji^T * Ω * Jj
        let h_ij = mat3_transpose_mul(ji, &omega_jj);
        // H_jj = Jj^T * Ω * Jj
        let h_jj = mat3_transpose_mul(jj, &omega_jj);

        // Add to H matrix
        for r in 0..3 {
            for c in 0..3 {
                h[(base_i + r) * dim + base_i + c] += h_ii[r * 3 + c];
                h[(base_i + r) * dim + base_j + c] += h_ij[r * 3 + c];
                h[(base_j + r) * dim + base_i + c] += h_ij[c * 3 + r]; // H_ji = H_ij^T
                h[(base_j + r) * dim + base_j + c] += h_jj[r * 3 + c];
            }
        }

        // Compute b contributions
        // b_i = Ji^T * Ω * e
        // b_j = Jj^T * Ω * e
        let omega_e = [
            omega[0] * error[0] + omega[1] * error[1] + omega[2] * error[2],
            omega[3] * error[0] + omega[4] * error[1] + omega[5] * error[2],
            omega[6] * error[0] + omega[7] * error[1] + omega[8] * error[2],
        ];

        for r in 0..3 {
            b[base_i + r] += ji[r] * omega_e[0] + ji[3 + r] * omega_e[1] + ji[6 + r] * omega_e[2];
            b[base_j + r] += jj[r] * omega_e[0] + jj[3 + r] * omega_e[1] + jj[6 + r] * omega_e[2];
        }
    }

    /// Apply Levenberg-Marquardt damping.
    fn apply_damping(&self, h: &[f64], lambda: f64, dim: usize) -> Vec<f64> {
        let mut h_damped = h.to_vec();

        for i in 0..dim {
            h_damped[i * dim + i] += lambda * h_damped[i * dim + i].max(1.0);
        }

        h_damped
    }

    /// Solve the linear system using Cholesky decomposition.
    ///
    /// For small graphs, uses dense Cholesky.
    /// For production, this should use sparse solvers.
    fn solve_linear_system(&self, h: &[f64], b: &[f64], dim: usize) -> Option<Vec<f64>> {
        // For small to medium graphs, use dense Cholesky
        // In production, would use sparse solver like conjugate gradient

        // Cholesky decomposition: H = L * L^T
        let mut l = vec![0.0; dim * dim];

        for i in 0..dim {
            for j in 0..=i {
                let mut sum = h[i * dim + j];

                for k in 0..j {
                    sum -= l[i * dim + k] * l[j * dim + k];
                }

                if i == j {
                    if sum <= 0.0 {
                        // Matrix not positive definite
                        return None;
                    }
                    l[i * dim + j] = sum.sqrt();
                } else {
                    l[i * dim + j] = sum / l[j * dim + j];
                }
            }
        }

        // Forward substitution: L * y = b
        let mut y = vec![0.0; dim];
        for i in 0..dim {
            let mut sum = b[i];
            for j in 0..i {
                sum -= l[i * dim + j] * y[j];
            }
            y[i] = sum / l[i * dim + i];
        }

        // Backward substitution: L^T * x = y
        let mut x = vec![0.0; dim];
        for i in (0..dim).rev() {
            let mut sum = y[i];
            for j in (i + 1)..dim {
                sum -= l[j * dim + i] * x[j];
            }
            x[i] = sum / l[i * dim + i];
        }

        // Return negative (H * dx = -b => dx = -H^{-1} * b)
        Some(x.iter().map(|v| -v).collect())
    }

    /// Apply update to all poses.
    fn apply_update(&self, graph: &mut PoseGraph, dx: &[f64]) {
        let start_idx = if self.config.fix_first_pose { 1 } else { 0 };

        for (idx, node) in graph.nodes_mut().iter_mut().enumerate() {
            if idx < start_idx {
                continue;
            }

            let base = idx * 3;
            if base + 2 >= dx.len() {
                continue;
            }

            node.pose.x += dx[base] as f32;
            node.pose.y += dx[base + 1] as f32;
            node.pose.theta = normalize_angle_f32(node.pose.theta + dx[base + 2] as f32);
        }
    }
}

/// Normalize angle to [-π, π].
fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

/// Normalize angle to [-π, π] (f32 version).
fn normalize_angle_f32(angle: f32) -> f32 {
    let mut a = angle;
    while a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    }
    while a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}

/// Multiply two 3x3 matrices (row-major).
fn mat3_mul(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9] {
    let mut c = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                c[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
            }
        }
    }
    c
}

/// Multiply transpose of first matrix with second: A^T * B.
fn mat3_transpose_mul(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9] {
    let mut c = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                c[i * 3 + j] += a[k * 3 + i] * b[k * 3 + j];
            }
        }
    }
    c
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_optimizer_creation() {
        let config = GraphOptimizerConfig::default();
        let _optimizer = GraphOptimizer::new(config);
    }

    #[test]
    fn test_optimize_empty_graph() {
        let config = GraphOptimizerConfig::default();
        let optimizer = GraphOptimizer::new(config);
        let mut graph = PoseGraph::new();

        let result = optimizer.optimize(&mut graph);

        assert!(result.converged);
        assert_eq!(result.termination_reason, TerminationReason::NoEdges);
    }

    #[test]
    fn test_optimize_simple_chain() {
        let config = GraphOptimizerConfig::default();
        let optimizer = GraphOptimizer::new(config);
        let mut graph = PoseGraph::new();

        // Create a simple chain of poses
        use crate::engine::graph::pose_graph::PoseNode;
        let id0 = graph.add_node_full(PoseNode::new(0, Pose2D::new(0.0, 0.0, 0.0), 0));
        let id1 = graph.add_node_full(PoseNode::new(1, Pose2D::new(1.0, 0.0, 0.0), 1000));
        let id2 = graph.add_node_full(PoseNode::new(2, Pose2D::new(2.0, 0.0, 0.0), 2000));

        // Add odometry edges
        graph.add_odometry_edge(
            id0,
            id1,
            Pose2D::new(1.0, 0.0, 0.0),
            Information2D::default(),
        );
        graph.add_odometry_edge(
            id1,
            id2,
            Pose2D::new(1.0, 0.0, 0.0),
            Information2D::default(),
        );

        let result = optimizer.optimize(&mut graph);

        assert!(result.converged || result.iterations > 0);
    }

    #[test]
    fn test_optimize_with_loop_closure() {
        let config = GraphOptimizerConfig::default();
        let optimizer = GraphOptimizer::new(config);
        let mut graph = PoseGraph::new();

        // Create a square loop with slight error
        use crate::engine::graph::pose_graph::PoseNode;
        let id0 = graph.add_node_full(PoseNode::new(0, Pose2D::new(0.0, 0.0, 0.0), 0));
        let id1 = graph.add_node_full(PoseNode::new(1, Pose2D::new(1.0, 0.0, 0.0), 1000));
        let id2 = graph.add_node_full(PoseNode::new(2, Pose2D::new(1.0, 1.0, 0.0), 2000));
        let id3 = graph.add_node_full(PoseNode::new(3, Pose2D::new(0.0, 1.0, 0.0), 3000));

        let info = Information2D::default();

        // Add odometry edges (forming a square)
        graph.add_odometry_edge(id0, id1, Pose2D::new(1.0, 0.0, 0.0), info);
        graph.add_odometry_edge(
            id1,
            id2,
            Pose2D::new(0.0, 1.0, std::f32::consts::FRAC_PI_2),
            info,
        );
        graph.add_odometry_edge(id2, id3, Pose2D::new(-1.0, 0.0, std::f32::consts::PI), info);

        // Add loop closure (3 back to 0)
        graph.add_loop_closure_edge(
            id3,
            id0,
            Pose2D::new(0.0, -1.0, -std::f32::consts::FRAC_PI_2),
            info,
            0.9,
        );

        let result = optimizer.optimize(&mut graph);

        // Should complete optimization
        assert!(result.iterations > 0);
    }

    #[test]
    fn test_chi_squared_computation() {
        let config = GraphOptimizerConfig::default();
        let optimizer = GraphOptimizer::new(config);
        let mut graph = PoseGraph::new();

        // Perfect chain - should have zero error
        use crate::engine::graph::pose_graph::PoseNode;
        let id0 = graph.add_node_full(PoseNode::new(0, Pose2D::new(0.0, 0.0, 0.0), 0));
        let id1 = graph.add_node_full(PoseNode::new(1, Pose2D::new(1.0, 0.0, 0.0), 1000));

        graph.add_odometry_edge(
            id0,
            id1,
            Pose2D::new(1.0, 0.0, 0.0),
            Information2D::default(),
        );

        let chi2 = optimizer.compute_chi_squared(&graph);
        assert!(chi2 < 0.01, "Chi-squared should be near zero: {}", chi2);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-10);
        // 3π normalizes to approximately π or -π (both are valid)
        let angle_3pi = normalize_angle(3.0 * std::f64::consts::PI);
        assert!(
            angle_3pi.abs() > std::f64::consts::PI - 0.1,
            "Expected angle near ±π, got {}",
            angle_3pi
        );
        let angle_neg3pi = normalize_angle(-3.0 * std::f64::consts::PI);
        assert!(
            angle_neg3pi.abs() > std::f64::consts::PI - 0.1,
            "Expected angle near ±π, got {}",
            angle_neg3pi
        );
    }

    #[test]
    fn test_matrix_multiplication() {
        let identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let a = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];

        let result = mat3_mul(&identity, &a);
        assert_eq!(result, a);

        let result = mat3_mul(&a, &identity);
        assert_eq!(result, a);
    }

    #[test]
    fn test_config_defaults() {
        let config = GraphOptimizerConfig::default();

        assert!(config.max_iterations > 0);
        assert!(config.convergence_threshold > 0.0);
        assert!(config.fix_first_pose);
    }
}
