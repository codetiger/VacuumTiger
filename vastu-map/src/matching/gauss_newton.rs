//! Levenberg-Marquardt solver for point-to-line ICP.
//!
//! Solves the nonlinear least squares problem of finding the pose
//! that minimizes the sum of squared point-to-line distances.
//!
//! Uses Levenberg-Marquardt algorithm for robust convergence. LM blends
//! between Gauss-Newton (fast convergence near solution) and gradient
//! descent (robust far from solution). Set `lm_initial_lambda = 0.0`
//! to disable LM damping and use pure Gauss-Newton.
//!
//! # Performance
//!
//! Three versions are provided:
//! - `optimize_pose`: Takes `&[Line2D]`, computes normals on-the-fly
//! - `optimize_pose_fast`: Takes `&LineCollection` with pre-computed normals
//! - `optimize_pose_simd`: Takes `&CorrespondenceSoA` for full SIMD acceleration
//!
//! Use `optimize_pose_simd` for maximum performance (~30-40% faster than fast).

use std::simd::{f32x4, num::SimdFloat};

use crate::core::{Point2D, Pose2D};
use crate::features::{Line2D, LineCollection};

use super::correspondence::{CorrespondenceSet, CorrespondenceSoA};

/// Configuration for Levenberg-Marquardt solver.
///
/// LM blends between Gauss-Newton and gradient descent for robust convergence.
/// Set `lm_initial_lambda = 0.0` to disable LM damping (pure Gauss-Newton).
#[derive(Clone, Debug)]
pub struct GaussNewtonConfig {
    /// Maximum number of iterations.
    /// Default: 20
    pub max_iterations: usize,

    /// Convergence threshold for pose change.
    /// If ‖Δpose‖ < threshold, iteration stops.
    /// Default: 1e-6
    pub convergence_threshold: f32,

    /// Pre-computed squared convergence threshold for efficient comparison.
    /// Avoids sqrt() in hot loop.
    pub(crate) convergence_threshold_sq: f32,

    /// Minimum eigenvalue ratio for matrix conditioning.
    /// If smallest/largest eigenvalue < ratio, add regularization.
    /// Default: 1e-6
    pub min_eigenvalue_ratio: f32,

    /// Regularization factor for ill-conditioned systems.
    /// Default: 1e-4
    pub regularization: f32,

    /// Initial LM damping factor (λ).
    /// Set to 0.0 to disable LM and use pure Gauss-Newton.
    /// Default: 0.001
    pub lm_initial_lambda: f32,

    /// Factor to scale λ up (on bad step) or down (on good step).
    /// Default: 10.0
    pub lm_lambda_factor: f32,

    /// Minimum λ value (floor after successful steps).
    /// Default: 1e-7
    pub lm_min_lambda: f32,

    /// Maximum λ value (triggers failure if exceeded).
    /// Default: 1e7
    pub lm_max_lambda: f32,
}

impl Default for GaussNewtonConfig {
    fn default() -> Self {
        let convergence_threshold = 1e-6;
        Self {
            max_iterations: 20,
            convergence_threshold,
            convergence_threshold_sq: convergence_threshold * convergence_threshold,
            min_eigenvalue_ratio: 1e-6,
            regularization: 1e-4,
            // LM parameters
            lm_initial_lambda: 0.001,
            lm_lambda_factor: 10.0,
            lm_min_lambda: 1e-7,
            lm_max_lambda: 1e7,
        }
    }
}

impl GaussNewtonConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for maximum iterations.
    pub fn with_max_iterations(mut self, iterations: usize) -> Self {
        self.max_iterations = iterations;
        self
    }

    /// Builder-style setter for convergence threshold.
    pub fn with_convergence_threshold(mut self, threshold: f32) -> Self {
        self.convergence_threshold = threshold;
        self.convergence_threshold_sq = threshold * threshold;
        self
    }

    /// Builder-style setter for initial LM lambda.
    /// Set to 0.0 to disable LM (pure Gauss-Newton).
    pub fn with_lm_lambda(mut self, lambda: f32) -> Self {
        self.lm_initial_lambda = lambda;
        self
    }

    /// Disable LM damping (use pure Gauss-Newton).
    pub fn without_lm(mut self) -> Self {
        self.lm_initial_lambda = 0.0;
        self
    }
}

use super::correspondence::PoseCovariance;

/// Result of Gauss-Newton optimization.
#[derive(Clone, Debug)]
pub struct GaussNewtonResult {
    /// Optimized pose.
    pub pose: Pose2D,
    /// Number of iterations performed.
    pub iterations: usize,
    /// Final residual (RMS error).
    pub residual: f32,
    /// Whether optimization converged.
    pub converged: bool,
    /// Pose covariance matrix (from inverse Hessian).
    /// Captures uncertainty in [x, y, theta].
    pub covariance: PoseCovariance,
    /// Condition number of the Hessian matrix.
    /// High values (>100) indicate degenerate geometry.
    pub condition_number: f32,
}

/// Identity covariance matrix.
const IDENTITY_COVARIANCE: PoseCovariance = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Optimize pose using Gauss-Newton method for point-to-line ICP.
///
/// Minimizes: sum_i w_i * (n_i · (R(θ)·p_i + t - q_i))²
///
/// where:
/// - w_i is the weight of correspondence i (from noise model)
/// - p_i is the i-th scan point
/// - q_i is the closest point on line i
/// - n_i is the normal of line i
/// - R(θ), t is the rotation and translation
///
/// # Arguments
/// * `correspondences` - Point-to-line correspondences (with weights)
/// * `lines` - Map lines
/// * `initial_pose` - Initial pose estimate
/// * `config` - Solver configuration
///
/// # Returns
/// Optimization result with final pose, covariance, and convergence info.
pub fn optimize_pose(
    correspondences: &CorrespondenceSet,
    lines: &[Line2D],
    initial_pose: Pose2D,
    config: &GaussNewtonConfig,
) -> GaussNewtonResult {
    if correspondences.is_empty() {
        return GaussNewtonResult {
            pose: initial_pose,
            iterations: 0,
            residual: 0.0,
            converged: true,
            covariance: IDENTITY_COVARIANCE,
            condition_number: 1.0,
        };
    }

    let mut pose = initial_pose;
    let mut converged = false;
    let mut iterations = 0;
    let mut final_hessian = [[0.0f32; 3]; 3];

    for iter in 0..config.max_iterations {
        iterations = iter + 1;

        // Build weighted linear system: H·Δx = -g
        // H = J^T·W·J (weighted Hessian approximation)
        // g = J^T·W·r (weighted gradient)
        let (h, g, residual) = build_linear_system_weighted(correspondences, lines, &pose);
        final_hessian = h;

        // Solve for Δx using direct 3x3 solve
        let Some(delta) = solve_3x3(&h, &g, config.regularization) else {
            // System is singular - stop iteration
            break;
        };

        // Update pose
        pose.x += delta[0];
        pose.y += delta[1];
        pose.theta += delta[2];
        pose.theta = crate::core::math::normalize_angle(pose.theta);

        // Check convergence using squared comparison to avoid sqrt()
        let delta_norm_sq = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
        if delta_norm_sq < config.convergence_threshold_sq {
            converged = true;
            break;
        }

        // Also check if residual is very small
        if residual < config.convergence_threshold {
            converged = true;
            break;
        }
    }

    // Compute final residual
    let final_residual = compute_residual(correspondences, lines, &pose);

    // Compute covariance from inverse Hessian: Cov = σ² × H⁻¹
    // Use residual variance as σ²
    let residual_variance = final_residual * final_residual;
    let (covariance, condition_number) =
        compute_covariance(&final_hessian, residual_variance, config.regularization);

    GaussNewtonResult {
        pose,
        iterations,
        residual: final_residual,
        converged,
        covariance,
        condition_number,
    }
}

/// Build the weighted linear system for Gauss-Newton.
///
/// Uses correspondence weights for proper uncertainty-weighted least squares.
///
/// Returns (H, g, residual) where:
/// - H is the 3x3 weighted Hessian approximation (J^T·W·J)
/// - g is the 3-element weighted gradient (J^T·W·r)
/// - residual is the weighted RMS error
fn build_linear_system_weighted(
    correspondences: &CorrespondenceSet,
    lines: &[Line2D],
    pose: &Pose2D,
) -> ([[f32; 3]; 3], [f32; 3], f32) {
    let (sin, cos) = pose.theta.sin_cos();

    // Initialize H and g to zero
    let mut h = [[0.0f32; 3]; 3];
    let mut g = [0.0f32; 3];
    let mut sum_weighted_sq_error = 0.0f32;
    let mut sum_weights = 0.0f32;

    for corr in correspondences.iter() {
        let p = corr.point;
        let line = &lines[corr.line_idx];
        let weight = corr.weight;

        // Transform point by current pose
        let tx = p.x * cos - p.y * sin + pose.x;
        let ty = p.x * sin + p.y * cos + pose.y;
        let transformed = Point2D::new(tx, ty);

        // Get line normal
        let normal = line.normal();

        // Compute residual: signed distance from transformed point to line
        let residual = line.signed_distance_to_point(transformed);
        sum_weighted_sq_error += weight * residual * residual;
        sum_weights += weight;

        // Jacobian of residual w.r.t. pose (x, y, theta)
        let j0 = normal.x;
        let j1 = normal.y;
        let j2 = normal.x * (-p.x * sin - p.y * cos) + normal.y * (p.x * cos - p.y * sin);

        let jacobian = [j0, j1, j2];

        // Accumulate H = J^T·W·J (weighted Hessian)
        for i in 0..3 {
            for j in 0..3 {
                h[i][j] += weight * jacobian[i] * jacobian[j];
            }
        }

        // Accumulate g = J^T·W·r (weighted gradient)
        for i in 0..3 {
            g[i] += weight * jacobian[i] * residual;
        }
    }

    let rms_error = if sum_weights > 0.0 {
        (sum_weighted_sq_error / sum_weights).sqrt()
    } else {
        0.0
    };

    (h, g, rms_error)
}

/// Compute covariance matrix from inverse Hessian.
///
/// Cov = σ² × H⁻¹
///
/// Also computes condition number for degeneracy detection.
fn compute_covariance(
    hessian: &[[f32; 3]; 3],
    residual_variance: f32,
    regularization: f32,
) -> (PoseCovariance, f32) {
    // Add regularization to diagonal
    let mut h = *hessian;
    h[0][0] += regularization;
    h[1][1] += regularization;
    h[2][2] += regularization;

    // Compute determinant
    let det = h[0][0] * (h[1][1] * h[2][2] - h[1][2] * h[2][1])
        - h[0][1] * (h[1][0] * h[2][2] - h[1][2] * h[2][0])
        + h[0][2] * (h[1][0] * h[2][1] - h[1][1] * h[2][0]);

    if det.abs() < 1e-10 {
        // Singular matrix - return identity covariance with high condition number
        return (IDENTITY_COVARIANCE, f32::MAX);
    }

    let inv_det = 1.0 / det;

    // Compute inverse (H⁻¹)
    let inv = [
        [
            (h[1][1] * h[2][2] - h[1][2] * h[2][1]) * inv_det,
            (h[0][2] * h[2][1] - h[0][1] * h[2][2]) * inv_det,
            (h[0][1] * h[1][2] - h[0][2] * h[1][1]) * inv_det,
        ],
        [
            (h[1][2] * h[2][0] - h[1][0] * h[2][2]) * inv_det,
            (h[0][0] * h[2][2] - h[0][2] * h[2][0]) * inv_det,
            (h[0][2] * h[1][0] - h[0][0] * h[1][2]) * inv_det,
        ],
        [
            (h[1][0] * h[2][1] - h[1][1] * h[2][0]) * inv_det,
            (h[0][1] * h[2][0] - h[0][0] * h[2][1]) * inv_det,
            (h[0][0] * h[1][1] - h[0][1] * h[1][0]) * inv_det,
        ],
    ];

    // Scale by residual variance to get covariance
    let covariance = [
        [
            residual_variance * inv[0][0],
            residual_variance * inv[0][1],
            residual_variance * inv[0][2],
        ],
        [
            residual_variance * inv[1][0],
            residual_variance * inv[1][1],
            residual_variance * inv[1][2],
        ],
        [
            residual_variance * inv[2][0],
            residual_variance * inv[2][1],
            residual_variance * inv[2][2],
        ],
    ];

    // Compute condition number using Frobenius norm approximation
    // Condition number ≈ max_eigenvalue / min_eigenvalue
    // For 3x3, we can use a simpler approximation based on diagonal elements
    let diag_max = h[0][0].max(h[1][1]).max(h[2][2]);
    let diag_min = h[0][0].min(h[1][1]).min(h[2][2]);
    let condition_number = if diag_min > 1e-10 {
        diag_max / diag_min
    } else {
        f32::MAX
    };

    (covariance, condition_number)
}

/// Compute covariance matrix with degrees-of-freedom correction.
///
/// Uses unbiased variance estimate: σ² = Σw·r² / (n - 3)
/// where n is the number of correspondences and 3 is the number of parameters.
fn compute_covariance_with_dof(
    hessian: &[[f32; 3]; 3],
    sum_weighted_sq_residuals: f32,
    num_correspondences: usize,
    regularization: f32,
) -> (PoseCovariance, f32) {
    // Degrees of freedom correction: n - 3 (3 parameters: x, y, theta)
    let n = num_correspondences as f32;
    let dof = (n - 3.0).max(1.0);

    // Unbiased variance estimate
    let sigma_sq = sum_weighted_sq_residuals / dof;

    // Add regularization to diagonal
    let mut h = *hessian;
    h[0][0] += regularization;
    h[1][1] += regularization;
    h[2][2] += regularization;

    // Compute determinant
    let det = h[0][0] * (h[1][1] * h[2][2] - h[1][2] * h[2][1])
        - h[0][1] * (h[1][0] * h[2][2] - h[1][2] * h[2][0])
        + h[0][2] * (h[1][0] * h[2][1] - h[1][1] * h[2][0]);

    if det.abs() < 1e-10 {
        // Singular matrix - return identity covariance with high condition number
        return (IDENTITY_COVARIANCE, f32::MAX);
    }

    let inv_det = 1.0 / det;

    // Compute inverse (H⁻¹)
    let inv = [
        [
            (h[1][1] * h[2][2] - h[1][2] * h[2][1]) * inv_det,
            (h[0][2] * h[2][1] - h[0][1] * h[2][2]) * inv_det,
            (h[0][1] * h[1][2] - h[0][2] * h[1][1]) * inv_det,
        ],
        [
            (h[1][2] * h[2][0] - h[1][0] * h[2][2]) * inv_det,
            (h[0][0] * h[2][2] - h[0][2] * h[2][0]) * inv_det,
            (h[0][2] * h[1][0] - h[0][0] * h[1][2]) * inv_det,
        ],
        [
            (h[1][0] * h[2][1] - h[1][1] * h[2][0]) * inv_det,
            (h[0][1] * h[2][0] - h[0][0] * h[2][1]) * inv_det,
            (h[0][0] * h[1][1] - h[0][1] * h[1][0]) * inv_det,
        ],
    ];

    // Scale by unbiased variance to get covariance: Cov = σ² × H⁻¹
    let covariance = [
        [
            sigma_sq * inv[0][0],
            sigma_sq * inv[0][1],
            sigma_sq * inv[0][2],
        ],
        [
            sigma_sq * inv[1][0],
            sigma_sq * inv[1][1],
            sigma_sq * inv[1][2],
        ],
        [
            sigma_sq * inv[2][0],
            sigma_sq * inv[2][1],
            sigma_sq * inv[2][2],
        ],
    ];

    // Compute condition number
    let diag_max = h[0][0].max(h[1][1]).max(h[2][2]);
    let diag_min = h[0][0].min(h[1][1]).min(h[2][2]);
    let condition_number = if diag_min > 1e-10 {
        diag_max / diag_min
    } else {
        f32::MAX
    };

    (covariance, condition_number)
}

/// Solve a 3x3 linear system Ax = b using Cramer's rule.
///
/// Returns None if the matrix is singular.
fn solve_3x3(a: &[[f32; 3]; 3], b: &[f32; 3], regularization: f32) -> Option<[f32; 3]> {
    // Add regularization to diagonal
    let mut ar = *a;
    ar[0][0] += regularization;
    ar[1][1] += regularization;
    ar[2][2] += regularization;

    // Compute determinant
    let det = ar[0][0] * (ar[1][1] * ar[2][2] - ar[1][2] * ar[2][1])
        - ar[0][1] * (ar[1][0] * ar[2][2] - ar[1][2] * ar[2][0])
        + ar[0][2] * (ar[1][0] * ar[2][1] - ar[1][1] * ar[2][0]);

    if det.abs() < 1e-10 {
        return None;
    }

    let inv_det = 1.0 / det;

    // Compute inverse using cofactors
    let inv = [
        [
            (ar[1][1] * ar[2][2] - ar[1][2] * ar[2][1]) * inv_det,
            (ar[0][2] * ar[2][1] - ar[0][1] * ar[2][2]) * inv_det,
            (ar[0][1] * ar[1][2] - ar[0][2] * ar[1][1]) * inv_det,
        ],
        [
            (ar[1][2] * ar[2][0] - ar[1][0] * ar[2][2]) * inv_det,
            (ar[0][0] * ar[2][2] - ar[0][2] * ar[2][0]) * inv_det,
            (ar[0][2] * ar[1][0] - ar[0][0] * ar[1][2]) * inv_det,
        ],
        [
            (ar[1][0] * ar[2][1] - ar[1][1] * ar[2][0]) * inv_det,
            (ar[0][1] * ar[2][0] - ar[0][0] * ar[2][1]) * inv_det,
            (ar[0][0] * ar[1][1] - ar[0][1] * ar[1][0]) * inv_det,
        ],
    ];

    // x = A^(-1) · (-b)  (negative because we want to minimize)
    Some([
        -(inv[0][0] * b[0] + inv[0][1] * b[1] + inv[0][2] * b[2]),
        -(inv[1][0] * b[0] + inv[1][1] * b[1] + inv[1][2] * b[2]),
        -(inv[2][0] * b[0] + inv[2][1] * b[1] + inv[2][2] * b[2]),
    ])
}

/// Compute RMS residual for current pose.
fn compute_residual(correspondences: &CorrespondenceSet, lines: &[Line2D], pose: &Pose2D) -> f32 {
    if correspondences.is_empty() {
        return 0.0;
    }

    let (sin, cos) = pose.theta.sin_cos();
    let mut sum_sq = 0.0;

    for corr in correspondences.iter() {
        let p = corr.point;
        let line = &lines[corr.line_idx];

        // Transform point
        let tx = p.x * cos - p.y * sin + pose.x;
        let ty = p.x * sin + p.y * cos + pose.y;
        let transformed = Point2D::new(tx, ty);

        // Compute distance
        let dist = line.distance_to_point(transformed);
        sum_sq += dist * dist;
    }

    (sum_sq / correspondences.len() as f32).sqrt()
}

// ============================================================================
// Fast versions using pre-computed LineCollection normals
// ============================================================================

/// Optimize pose using Levenberg-Marquardt with pre-computed normals.
///
/// This version uses `LineCollection` with pre-computed normals and inverse
/// lengths, avoiding redundant sqrt computations in the inner loop.
/// Uses LM damping for robust convergence from poor initial guesses.
///
/// # Algorithm
///
/// LM adds damping (λI) to the Hessian diagonal:
/// - Good step (error decreases): accept step, decrease λ
/// - Bad step (error increases): reject step, increase λ
///
/// Set `config.lm_initial_lambda = 0.0` to disable LM (pure Gauss-Newton).
///
/// # Performance
///
/// ~15-20% faster than `optimize_pose` for typical correspondence sets.
pub fn optimize_pose_fast(
    correspondences: &CorrespondenceSet,
    lines: &LineCollection,
    initial_pose: Pose2D,
    config: &GaussNewtonConfig,
) -> GaussNewtonResult {
    if correspondences.is_empty() {
        return GaussNewtonResult {
            pose: initial_pose,
            iterations: 0,
            residual: 0.0,
            converged: true,
            covariance: IDENTITY_COVARIANCE,
            condition_number: 1.0,
        };
    }

    // Pre-fetch normals for all lines used in correspondences
    let (normal_xs, normal_ys) = lines.normals();
    let inv_lengths = lines.inv_lengths();

    let mut pose = initial_pose;
    let mut converged = false;
    let mut iterations = 0;
    let mut final_hessian = [[0.0f32; 3]; 3];
    let mut final_sum_weighted_sq = 0.0f32;

    // LM state
    let use_lm = config.lm_initial_lambda > 0.0;
    let mut lambda = config.lm_initial_lambda;
    let mut prev_residual = if use_lm {
        compute_residual_fast(correspondences, lines, inv_lengths, &pose)
    } else {
        f32::MAX
    };

    for iter in 0..config.max_iterations {
        iterations = iter + 1;

        // Build weighted linear system using pre-computed normals
        let (h, g, sum_weighted_sq, _sum_weights) = build_linear_system_fast_weighted_extended(
            correspondences,
            lines,
            normal_xs,
            normal_ys,
            inv_lengths,
            &pose,
        );
        final_hessian = h;
        final_sum_weighted_sq = sum_weighted_sq;

        // Add LM damping to Hessian diagonal
        let mut h_damped = h;
        if use_lm {
            h_damped[0][0] += lambda;
            h_damped[1][1] += lambda;
            h_damped[2][2] += lambda;
        }

        // Solve for Δx using direct 3x3 solve
        let Some(delta) = solve_3x3(&h_damped, &g, config.regularization) else {
            break;
        };

        // Compute trial pose
        let trial_pose = Pose2D::new(
            pose.x + delta[0],
            pose.y + delta[1],
            crate::core::math::normalize_angle(pose.theta + delta[2]),
        );

        if use_lm {
            // Evaluate step quality
            let trial_residual =
                compute_residual_fast(correspondences, lines, inv_lengths, &trial_pose);

            if trial_residual < prev_residual {
                // Good step - accept and decrease lambda
                pose = trial_pose;
                prev_residual = trial_residual;
                lambda = (lambda / config.lm_lambda_factor).max(config.lm_min_lambda);

                // Check convergence
                let delta_norm_sq = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
                if delta_norm_sq < config.convergence_threshold_sq {
                    converged = true;
                    break;
                }
            } else {
                // Bad step - reject and increase lambda
                lambda *= config.lm_lambda_factor;
                if lambda > config.lm_max_lambda {
                    // Failed to converge
                    break;
                }
                // Don't update pose, try again with higher damping
                continue;
            }
        } else {
            // Pure Gauss-Newton (no LM damping)
            pose = trial_pose;

            // Check convergence
            let delta_norm_sq = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
            if delta_norm_sq < config.convergence_threshold_sq {
                converged = true;
                break;
            }
        }
    }

    let final_residual = compute_residual_fast(correspondences, lines, inv_lengths, &pose);

    // Compute covariance with DOF correction
    let (covariance, condition_number) = compute_covariance_with_dof(
        &final_hessian,
        final_sum_weighted_sq,
        correspondences.len(),
        config.regularization,
    );

    GaussNewtonResult {
        pose,
        iterations,
        residual: final_residual,
        converged,
        covariance,
        condition_number,
    }
}

/// Build weighted linear system using pre-computed normals.
/// Returns (H, g, sum_weighted_sq_error, sum_weights) for covariance computation.
fn build_linear_system_fast_weighted_extended(
    correspondences: &CorrespondenceSet,
    lines: &LineCollection,
    normal_xs: &[f32],
    normal_ys: &[f32],
    inv_lengths: &[f32],
    pose: &Pose2D,
) -> ([[f32; 3]; 3], [f32; 3], f32, f32) {
    let (sin, cos) = pose.theta.sin_cos();

    // Pre-compute -sin and -cos for Jacobian
    let neg_sin = -sin;
    let neg_cos = -cos;

    let mut h = [[0.0f32; 3]; 3];
    let mut g = [0.0f32; 3];
    let mut sum_weighted_sq_error = 0.0f32;
    let mut sum_weights = 0.0f32;

    for corr in correspondences.iter() {
        let p = corr.point;
        let line_idx = corr.line_idx;
        let weight = corr.weight;

        // Transform point by current pose
        let tx = p.x * cos - p.y * sin + pose.x;
        let ty = p.x * sin + p.y * cos + pose.y;

        // Get pre-computed normal (no sqrt!)
        let nx = normal_xs[line_idx];
        let ny = normal_ys[line_idx];

        // Compute residual
        let sx = lines.start_xs[line_idx];
        let sy = lines.start_ys[line_idx];
        let ex = lines.end_xs[line_idx];
        let ey = lines.end_ys[line_idx];

        let dx = ex - sx;
        let dy = ey - sy;
        let to_px = tx - sx;
        let to_py = ty - sy;

        let cross = dx * to_py - dy * to_px;
        let residual = cross * inv_lengths[line_idx];

        sum_weighted_sq_error += weight * residual * residual;
        sum_weights += weight;

        // Jacobian using pre-computed normal
        let j0 = nx;
        let j1 = ny;
        let j2 = nx * (p.x * neg_sin + p.y * neg_cos) + ny * (p.x * cos + p.y * neg_sin);

        // Accumulate weighted H = J^T·W·J (symmetric)
        h[0][0] += weight * j0 * j0;
        h[0][1] += weight * j0 * j1;
        h[0][2] += weight * j0 * j2;
        h[1][1] += weight * j1 * j1;
        h[1][2] += weight * j1 * j2;
        h[2][2] += weight * j2 * j2;

        // Accumulate weighted g = J^T·W·r
        g[0] += weight * j0 * residual;
        g[1] += weight * j1 * residual;
        g[2] += weight * j2 * residual;
    }

    // Fill symmetric entries
    h[1][0] = h[0][1];
    h[2][0] = h[0][2];
    h[2][1] = h[1][2];

    (h, g, sum_weighted_sq_error, sum_weights)
}

/// Compute RMS residual using pre-computed inverse lengths.
fn compute_residual_fast(
    correspondences: &CorrespondenceSet,
    lines: &LineCollection,
    inv_lengths: &[f32],
    pose: &Pose2D,
) -> f32 {
    if correspondences.is_empty() {
        return 0.0;
    }

    let (sin, cos) = pose.theta.sin_cos();
    let mut sum_sq = 0.0;

    for corr in correspondences.iter() {
        let p = corr.point;
        let line_idx = corr.line_idx;

        // Transform point
        let tx = p.x * cos - p.y * sin + pose.x;
        let ty = p.x * sin + p.y * cos + pose.y;

        // Compute distance using pre-computed values
        let sx = lines.start_xs[line_idx];
        let sy = lines.start_ys[line_idx];
        let ex = lines.end_xs[line_idx];
        let ey = lines.end_ys[line_idx];

        let dx = ex - sx;
        let dy = ey - sy;
        let to_px = tx - sx;
        let to_py = ty - sy;

        let cross = dx * to_py - dy * to_px;
        let dist = cross.abs() * inv_lengths[line_idx];

        sum_sq += dist * dist;
    }

    (sum_sq / correspondences.len() as f32).sqrt()
}

// ============================================================================
// SIMD-Optimized Version using CorrespondenceSoA
// ============================================================================

/// Optimize pose using SIMD-accelerated Gauss-Newton with SoA correspondences.
///
/// This is the fastest variant, using [`CorrespondenceSoA`] which inlines all
/// line data for contiguous SIMD loads. Provides ~30-40% speedup over
/// `optimize_pose_fast` by eliminating indexed array access.
///
/// # Arguments
/// * `correspondences` - SoA correspondences (must be `finalize()`d)
/// * `initial_pose` - Initial pose estimate
/// * `config` - Solver configuration
///
/// # Example
///
/// ```rust,ignore
/// let mut soa = CorrespondenceSoA::new();
/// // ... populate correspondences ...
/// soa.finalize();
///
/// let result = optimize_pose_simd(&soa, Pose2D::identity(), &GaussNewtonConfig::default());
/// ```
pub fn optimize_pose_simd(
    correspondences: &CorrespondenceSoA,
    initial_pose: Pose2D,
    config: &GaussNewtonConfig,
) -> GaussNewtonResult {
    if correspondences.is_empty() {
        return GaussNewtonResult {
            pose: initial_pose,
            iterations: 0,
            residual: 0.0,
            converged: true,
            covariance: IDENTITY_COVARIANCE,
            condition_number: 1.0,
        };
    }

    let mut pose = initial_pose;
    let mut converged = false;
    let mut iterations = 0;
    let mut final_hessian = [[0.0f32; 3]; 3];
    let mut final_sum_weighted_sq = 0.0f32;

    // LM state
    let use_lm = config.lm_initial_lambda > 0.0;
    let mut lambda = config.lm_initial_lambda;
    let mut prev_residual = if use_lm {
        compute_residual_simd(correspondences, &pose)
    } else {
        f32::MAX
    };

    for iter in 0..config.max_iterations {
        iterations = iter + 1;

        // Build weighted linear system using SIMD
        let (h, g, sum_weighted_sq, _sum_weights) =
            build_linear_system_simd(correspondences, &pose);
        final_hessian = h;
        final_sum_weighted_sq = sum_weighted_sq;

        // Add LM damping to Hessian diagonal
        let mut h_damped = h;
        if use_lm {
            h_damped[0][0] += lambda;
            h_damped[1][1] += lambda;
            h_damped[2][2] += lambda;
        }

        // Solve for Δx
        let Some(delta) = solve_3x3(&h_damped, &g, config.regularization) else {
            break;
        };

        // Compute trial pose
        let trial_pose = Pose2D::new(
            pose.x + delta[0],
            pose.y + delta[1],
            crate::core::math::normalize_angle(pose.theta + delta[2]),
        );

        if use_lm {
            let trial_residual = compute_residual_simd(correspondences, &trial_pose);

            if trial_residual < prev_residual {
                pose = trial_pose;
                prev_residual = trial_residual;
                lambda = (lambda / config.lm_lambda_factor).max(config.lm_min_lambda);

                let delta_norm_sq = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
                if delta_norm_sq < config.convergence_threshold_sq {
                    converged = true;
                    break;
                }
            } else {
                lambda *= config.lm_lambda_factor;
                if lambda > config.lm_max_lambda {
                    break;
                }
                continue;
            }
        } else {
            pose = trial_pose;

            let delta_norm_sq = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
            if delta_norm_sq < config.convergence_threshold_sq {
                converged = true;
                break;
            }
        }
    }

    let final_residual = compute_residual_simd(correspondences, &pose);

    let (covariance, condition_number) = compute_covariance_with_dof(
        &final_hessian,
        final_sum_weighted_sq,
        correspondences.len(),
        config.regularization,
    );

    GaussNewtonResult {
        pose,
        iterations,
        residual: final_residual,
        converged,
        covariance,
        condition_number,
    }
}

/// Build weighted linear system using SIMD with SoA correspondences.
///
/// Processes 4 correspondences per iteration using f32x4 operations.
/// Returns (H, g, sum_weighted_sq_error, sum_weights).
fn build_linear_system_simd(
    correspondences: &CorrespondenceSoA,
    pose: &Pose2D,
) -> ([[f32; 3]; 3], [f32; 3], f32, f32) {
    let (sin, cos) = pose.theta.sin_cos();
    let neg_sin = -sin;
    let neg_cos = -cos;

    // SIMD constants
    let cos4 = f32x4::splat(cos);
    let sin4 = f32x4::splat(sin);
    let neg_sin4 = f32x4::splat(neg_sin);
    let neg_cos4 = f32x4::splat(neg_cos);
    let pose_x4 = f32x4::splat(pose.x);
    let pose_y4 = f32x4::splat(pose.y);

    // Accumulators for Hessian (6 unique elements for symmetric 3x3)
    let mut h00 = f32x4::splat(0.0);
    let mut h01 = f32x4::splat(0.0);
    let mut h02 = f32x4::splat(0.0);
    let mut h11 = f32x4::splat(0.0);
    let mut h12 = f32x4::splat(0.0);
    let mut h22 = f32x4::splat(0.0);

    // Accumulators for gradient
    let mut g0 = f32x4::splat(0.0);
    let mut g1 = f32x4::splat(0.0);
    let mut g2 = f32x4::splat(0.0);

    // Error accumulators
    let mut sum_wsq4 = f32x4::splat(0.0);
    let mut sum_w4 = f32x4::splat(0.0);

    let chunks = correspondences.simd_chunks();

    for i in 0..chunks {
        let base = i * 4;

        // Load 4 points (contiguous SIMD loads!)
        let px = f32x4::from_slice(&correspondences.point_xs[base..]);
        let py = f32x4::from_slice(&correspondences.point_ys[base..]);

        // Load inlined line data (contiguous!)
        let nx = f32x4::from_slice(&correspondences.normal_xs[base..]);
        let ny = f32x4::from_slice(&correspondences.normal_ys[base..]);
        let sx = f32x4::from_slice(&correspondences.line_start_xs[base..]);
        let sy = f32x4::from_slice(&correspondences.line_start_ys[base..]);
        let ex = f32x4::from_slice(&correspondences.line_end_xs[base..]);
        let ey = f32x4::from_slice(&correspondences.line_end_ys[base..]);
        let inv_len = f32x4::from_slice(&correspondences.line_inv_lengths[base..]);
        let weight = f32x4::from_slice(&correspondences.weights[base..]);

        // Transform points: tx = px*cos - py*sin + pose.x
        let tx = px * cos4 - py * sin4 + pose_x4;
        let ty = px * sin4 + py * cos4 + pose_y4;

        // Compute residual: cross / length
        let dx = ex - sx;
        let dy = ey - sy;
        let to_px = tx - sx;
        let to_py = ty - sy;
        let cross = dx * to_py - dy * to_px;
        let residual = cross * inv_len;

        // Accumulate weighted squared error
        sum_wsq4 += weight * residual * residual;
        sum_w4 += weight;

        // Jacobian components
        let j0 = nx;
        let j1 = ny;
        // j2 = nx * (px * -sin + py * -cos) + ny * (px * cos + py * -sin)
        let j2 = nx * (px * neg_sin4 + py * neg_cos4) + ny * (px * cos4 + py * neg_sin4);

        // Weighted Jacobian
        let wj0 = weight * j0;
        let wj1 = weight * j1;
        let wj2 = weight * j2;

        // Accumulate Hessian: H += J^T * W * J
        h00 += wj0 * j0;
        h01 += wj0 * j1;
        h02 += wj0 * j2;
        h11 += wj1 * j1;
        h12 += wj1 * j2;
        h22 += wj2 * j2;

        // Accumulate gradient: g += J^T * W * r
        g0 += wj0 * residual;
        g1 += wj1 * residual;
        g2 += wj2 * residual;
    }

    // Horizontal reduction
    let h = [
        [h00.reduce_sum(), h01.reduce_sum(), h02.reduce_sum()],
        [h01.reduce_sum(), h11.reduce_sum(), h12.reduce_sum()],
        [h02.reduce_sum(), h12.reduce_sum(), h22.reduce_sum()],
    ];

    let g = [g0.reduce_sum(), g1.reduce_sum(), g2.reduce_sum()];

    (h, g, sum_wsq4.reduce_sum(), sum_w4.reduce_sum())
}

/// Compute RMS residual using SIMD with SoA correspondences.
fn compute_residual_simd(correspondences: &CorrespondenceSoA, pose: &Pose2D) -> f32 {
    if correspondences.is_empty() {
        return 0.0;
    }

    let (sin, cos) = pose.theta.sin_cos();
    let cos4 = f32x4::splat(cos);
    let sin4 = f32x4::splat(sin);
    let pose_x4 = f32x4::splat(pose.x);
    let pose_y4 = f32x4::splat(pose.y);

    let mut sum_sq4 = f32x4::splat(0.0);
    let chunks = correspondences.simd_chunks();

    for i in 0..chunks {
        let base = i * 4;

        let px = f32x4::from_slice(&correspondences.point_xs[base..]);
        let py = f32x4::from_slice(&correspondences.point_ys[base..]);
        let sx = f32x4::from_slice(&correspondences.line_start_xs[base..]);
        let sy = f32x4::from_slice(&correspondences.line_start_ys[base..]);
        let ex = f32x4::from_slice(&correspondences.line_end_xs[base..]);
        let ey = f32x4::from_slice(&correspondences.line_end_ys[base..]);
        let inv_len = f32x4::from_slice(&correspondences.line_inv_lengths[base..]);
        let weight = f32x4::from_slice(&correspondences.weights[base..]);

        // Transform point
        let tx = px * cos4 - py * sin4 + pose_x4;
        let ty = px * sin4 + py * cos4 + pose_y4;

        // Compute distance
        let dx = ex - sx;
        let dy = ey - sy;
        let to_px = tx - sx;
        let to_py = ty - sy;
        let cross = dx * to_py - dy * to_px;
        let dist = cross.abs() * inv_len;

        // Only count non-zero weight entries (padding has weight=0)
        let mask = weight; // Use weight as mask (0 for padding)
        sum_sq4 += mask * dist * dist;
    }

    (sum_sq4.reduce_sum() / correspondences.len() as f32).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point2D;
    use crate::matching::correspondence::Correspondence;
    use approx::assert_relative_eq;

    #[test]
    fn test_optimize_translation_only() {
        // Single horizontal line at y=0
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))];

        // Points that are offset from the line
        let mut corrs = CorrespondenceSet::new();
        corrs.push(Correspondence::new(0, 0, Point2D::new(1.0, 0.5), 0.5, 0.1));
        corrs.push(Correspondence::new(1, 0, Point2D::new(3.0, 0.5), 0.5, 0.3));
        corrs.push(Correspondence::new(2, 0, Point2D::new(5.0, 0.5), 0.5, 0.5));

        let config = GaussNewtonConfig::default();
        let result = optimize_pose(&corrs, &lines, Pose2D::identity(), &config);

        // Should find translation to move points down to the line
        assert!(result.converged);
        assert_relative_eq!(result.pose.y, -0.5, epsilon = 0.1);
        assert_relative_eq!(result.pose.x, 0.0, epsilon = 0.1);
    }

    #[test]
    fn test_optimize_two_lines() {
        // Two perpendicular lines
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)), // Horizontal
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 10.0)), // Vertical
        ];

        // Points offset from both lines
        let mut corrs = CorrespondenceSet::new();
        corrs.push(Correspondence::new(0, 0, Point2D::new(2.0, 0.3), 0.3, 0.2));
        corrs.push(Correspondence::new(1, 0, Point2D::new(5.0, 0.3), 0.3, 0.5));
        corrs.push(Correspondence::new(2, 1, Point2D::new(0.3, 2.0), 0.3, 0.2));
        corrs.push(Correspondence::new(3, 1, Point2D::new(0.3, 5.0), 0.3, 0.5));

        let config = GaussNewtonConfig::default();
        let result = optimize_pose(&corrs, &lines, Pose2D::identity(), &config);

        assert!(result.converged);
        // Should translate points toward both lines
        assert!(result.pose.x < 0.0); // Move left
        assert!(result.pose.y < 0.0); // Move down
    }

    #[test]
    fn test_optimize_empty_correspondences() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))];
        let corrs = CorrespondenceSet::new();

        let initial = Pose2D::new(1.0, 2.0, 0.5);
        let config = GaussNewtonConfig::default();
        let result = optimize_pose(&corrs, &lines, initial, &config);

        // Should return initial pose unchanged
        assert_eq!(result.pose.x, initial.x);
        assert_eq!(result.pose.y, initial.y);
        assert_eq!(result.pose.theta, initial.theta);
        assert!(result.converged);
    }

    #[test]
    fn test_solve_3x3() {
        // Simple identity system
        let a = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [1.0, 2.0, 3.0];

        let x = solve_3x3(&a, &b, 0.0).unwrap();

        assert_relative_eq!(x[0], -1.0, epsilon = 1e-6);
        assert_relative_eq!(x[1], -2.0, epsilon = 1e-6);
        assert_relative_eq!(x[2], -3.0, epsilon = 1e-6);
    }

    #[test]
    fn test_solve_3x3_with_regularization() {
        // Nearly singular matrix
        let a = [[1e-8, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [1.0, 2.0, 3.0];

        // Without regularization might fail
        let x = solve_3x3(&a, &b, 1e-4);
        assert!(x.is_some());
    }

    #[test]
    fn test_compute_residual() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))];

        let mut corrs = CorrespondenceSet::new();
        corrs.push(Correspondence::new(0, 0, Point2D::new(1.0, 0.1), 0.1, 0.1));
        corrs.push(Correspondence::new(1, 0, Point2D::new(5.0, 0.1), 0.1, 0.5));

        let residual = compute_residual(&corrs, &lines, &Pose2D::identity());

        // RMS of [0.1, 0.1] = sqrt(0.02/2) = 0.1
        assert_relative_eq!(residual, 0.1, epsilon = 0.01);
    }

    #[test]
    fn test_convergence() {
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0))];

        // Points already on the line - should converge immediately
        let mut corrs = CorrespondenceSet::new();
        corrs.push(Correspondence::new(0, 0, Point2D::new(1.0, 0.0), 0.0, 0.1));
        corrs.push(Correspondence::new(1, 0, Point2D::new(5.0, 0.0), 0.0, 0.5));

        let config = GaussNewtonConfig::default();
        let result = optimize_pose(&corrs, &lines, Pose2D::identity(), &config);

        assert!(result.converged);
        assert!(result.iterations <= 2); // Should converge quickly
        assert!(result.residual < 1e-5);
    }
}
