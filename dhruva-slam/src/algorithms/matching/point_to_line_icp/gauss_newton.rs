//! Gauss-Newton optimization for point-to-line ICP.

use super::correspondence::Correspondence;
use crate::core::types::{PointCloud2D, Pose2D};

/// Compute optimal transform using point-to-line minimization.
///
/// Minimizes sum of squared point-to-line distances using Gauss-Newton.
/// Uses original source coordinates for Jacobian (derivative of rotation).
///
/// # Arguments
/// * `source` - Original source point cloud (for Jacobian computation)
/// * `transformed_source` - Pre-transformed source cloud (for residuals)
/// * `correspondences` - Point-to-line correspondences
/// * `current_transform` - Current accumulated transform
///
/// # Returns
/// Incremental transform delta to apply
pub fn compute_transform(
    source: &PointCloud2D,
    transformed_source: &PointCloud2D,
    correspondences: &[Correspondence],
    current_transform: &Pose2D,
) -> Pose2D {
    if correspondences.len() < 3 {
        return Pose2D::identity();
    }

    let (sin_t, cos_t) = current_transform.theta.sin_cos();

    // Build linear system: A * [dx, dy, dtheta]^T = b
    // Using Gauss-Newton with point-to-line residuals
    //
    // Residual for correspondence i:
    // r_i = n_i · (R(theta) * p_i + t - q_i)
    // where n_i is line normal, p_i is source point, q_i is target point
    //
    // Linearized:
    // ∂r/∂x = n_x
    // ∂r/∂y = n_y
    // ∂r/∂theta = n_x * (-sin(theta) * p_x - cos(theta) * p_y)
    //           + n_y * (cos(theta) * p_x - sin(theta) * p_y)

    // Normal equations: H * delta = g
    // H = J^T * J (6x6 for 3D, 3x3 for 2D)
    // g = -J^T * r

    // Initialize 3x3 system
    let mut h = [[0.0f32; 3]; 3]; // Hessian approximation
    let mut g = [0.0f32; 3]; // Gradient

    for corr in correspondences {
        // Original source coordinates for Jacobian (derivative is w.r.t. original point)
        let px_orig = source.xs[corr.source_idx];
        let py_orig = source.ys[corr.source_idx];
        let n_x = corr.target_line.a;
        let n_y = corr.target_line.b;

        // Use pre-transformed source point for residual
        let px = transformed_source.xs[corr.source_idx];
        let py = transformed_source.ys[corr.source_idx];

        // Residual: r = n · p_transformed + c
        let r = n_x * px + n_y * py + corr.target_line.c;

        // Jacobian row
        // The derivative ∂r/∂θ uses the original point because:
        // p_transformed = R(θ) * p_original + t
        // So ∂p_transformed/∂θ = ∂R/∂θ * p_original
        let j0 = n_x;
        let j1 = n_y;
        let j2 =
            n_x * (-sin_t * px_orig - cos_t * py_orig) + n_y * (cos_t * px_orig - sin_t * py_orig);

        // Accumulate H = J^T * J
        h[0][0] += j0 * j0;
        h[0][1] += j0 * j1;
        h[0][2] += j0 * j2;
        h[1][0] += j1 * j0;
        h[1][1] += j1 * j1;
        h[1][2] += j1 * j2;
        h[2][0] += j2 * j0;
        h[2][1] += j2 * j1;
        h[2][2] += j2 * j2;

        // Accumulate g = -J^T * r
        g[0] -= j0 * r;
        g[1] -= j1 * r;
        g[2] -= j2 * r;
    }

    // Solve 3x3 system using Cramer's rule (small system, direct solve)
    solve_3x3(&h, &g)
}

/// Solve 3x3 linear system H * x = g using Cramer's rule.
///
/// Returns identity pose if system is singular.
#[inline]
fn solve_3x3(h: &[[f32; 3]; 3], g: &[f32; 3]) -> Pose2D {
    let det = h[0][0] * (h[1][1] * h[2][2] - h[1][2] * h[2][1])
        - h[0][1] * (h[1][0] * h[2][2] - h[1][2] * h[2][0])
        + h[0][2] * (h[1][0] * h[2][1] - h[1][1] * h[2][0]);

    if det.abs() < 1e-10 {
        return Pose2D::identity();
    }

    let inv_det = 1.0 / det;

    // Compute inverse of H
    let h_inv = [
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

    // delta = H^-1 * g
    let dx = h_inv[0][0] * g[0] + h_inv[0][1] * g[1] + h_inv[0][2] * g[2];
    let dy = h_inv[1][0] * g[0] + h_inv[1][1] * g[1] + h_inv[1][2] * g[2];
    let dtheta = h_inv[2][0] * g[0] + h_inv[2][1] * g[1] + h_inv[2][2] * g[2];

    Pose2D::new(dx, dy, dtheta)
}
