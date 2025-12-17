//! Odometry-related types.
//!
//! Note: Some utility methods are defined for future use.

use serde::{Deserialize, Serialize};

/// 3x3 covariance matrix for 2D pose uncertainty (x, y, theta).
///
/// Stored as row-major array: [xx, xy, xt, yx, yy, yt, tx, ty, tt]
/// where t = theta.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Covariance2D {
    /// Row-major 3x3 matrix data
    data: [f32; 9],
}

impl Covariance2D {
    /// Create a zero covariance matrix.
    #[inline]
    pub fn zero() -> Self {
        Self { data: [0.0; 9] }
    }

    /// Create a diagonal covariance matrix.
    ///
    /// Parameters are variances: xx = σ²_x, yy = σ²_y, tt = σ²_θ
    #[inline]
    pub fn diagonal(xx: f32, yy: f32, tt: f32) -> Self {
        Self {
            data: [xx, 0.0, 0.0, 0.0, yy, 0.0, 0.0, 0.0, tt],
        }
    }

    /// Create from row-major array.
    #[inline]
    pub fn from_array(data: [f32; 9]) -> Self {
        Self { data }
    }

    /// Get variance of theta (element [2,2]).
    #[inline]
    pub fn var_theta(&self) -> f32 {
        self.data[8]
    }

    /// Get raw data as slice.
    #[inline]
    pub fn as_slice(&self) -> &[f32; 9] {
        &self.data
    }
}

impl Default for Covariance2D {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_covariance2d() {
        let zero = Covariance2D::zero();
        assert_eq!(zero.var_theta(), 0.0);

        let diag = Covariance2D::diagonal(0.1, 0.2, 0.05);
        assert_eq!(diag.var_theta(), 0.05);
    }
}
