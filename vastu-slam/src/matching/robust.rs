//! Robust loss functions for outlier rejection in optimization.
//!
//! These kernels are used in pose graph optimization to reduce the
//! influence of outliers (e.g., false loop closures).
//!
//! # Available Kernels
//!
//! - **None**: Standard least squares (no robustness)
//! - **Huber**: Linear penalty beyond threshold, smooth transition
//! - **Cauchy**: Heavy-tailed, strong outlier rejection
//!
//! # Example
//!
//! ```
//! use vastu_slam::matching::RobustKernel;
//!
//! let kernel = RobustKernel::Huber(0.5);
//! let weight = kernel.weight(1.0); // Large residual gets down-weighted
//! assert!(weight < 1.0);
//! ```

use serde::{Deserialize, Serialize};

/// Robust kernel for outlier rejection.
///
/// Used in optimization to reduce the influence of outliers
/// on the solution.
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub enum RobustKernel {
    /// No robustness - standard least squares
    #[default]
    None,

    /// Huber kernel with given threshold.
    ///
    /// - Behaves like L2 for |r| < threshold
    /// - Behaves like L1 for |r| > threshold
    /// - Smooth transition at threshold
    Huber(f32),

    /// Cauchy kernel with given scale.
    ///
    /// - Heavy-tailed distribution
    /// - Strong outlier rejection
    /// - Commonly used in computer vision
    Cauchy(f32),
}

impl RobustKernel {
    /// Compute the weight for a given residual.
    ///
    /// In iteratively reweighted least squares (IRLS), this weight
    /// is applied to the residual to reduce outlier influence.
    ///
    /// # Arguments
    /// * `residual` - The residual value (error)
    ///
    /// # Returns
    /// Weight in range (0, 1] where smaller weights indicate outliers
    pub fn weight(&self, residual: f32) -> f32 {
        let abs_r = residual.abs();

        match self {
            RobustKernel::None => 1.0,

            RobustKernel::Huber(threshold) => {
                if abs_r <= *threshold {
                    1.0
                } else {
                    threshold / abs_r
                }
            }

            RobustKernel::Cauchy(scale) => {
                let c2 = scale * scale;
                c2 / (c2 + residual * residual)
            }
        }
    }

    /// Compute the robust cost for a given residual.
    ///
    /// # Arguments
    /// * `residual` - The residual value (error)
    ///
    /// # Returns
    /// The robust cost (less than r² for outliers)
    pub fn cost(&self, residual: f32) -> f32 {
        let r2 = residual * residual;
        let abs_r = residual.abs();

        match self {
            RobustKernel::None => r2,

            RobustKernel::Huber(threshold) => {
                let t = *threshold;
                if abs_r <= t {
                    r2
                } else {
                    2.0 * t * abs_r - t * t
                }
            }

            RobustKernel::Cauchy(scale) => {
                let c2 = scale * scale;
                c2 * (1.0 + r2 / c2).ln()
            }
        }
    }

    /// Check if this kernel provides robustness.
    pub fn is_robust(&self) -> bool {
        !matches!(self, RobustKernel::None)
    }

    /// Get the kernel name for logging
    pub fn name(&self) -> &str {
        match self {
            RobustKernel::None => "None",
            RobustKernel::Huber(_) => "Huber",
            RobustKernel::Cauchy(_) => "Cauchy",
        }
    }
}

/// Convenience function to create a Huber kernel with typical threshold.
pub fn huber() -> RobustKernel {
    RobustKernel::Huber(1.0)
}

/// Convenience function to create a Cauchy kernel with typical scale.
pub fn cauchy() -> RobustKernel {
    RobustKernel::Cauchy(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_none_kernel() {
        let kernel = RobustKernel::None;

        // Weight is always 1
        assert!((kernel.weight(0.0) - 1.0).abs() < 1e-6);
        assert!((kernel.weight(10.0) - 1.0).abs() < 1e-6);

        // Cost is r²
        assert!((kernel.cost(3.0) - 9.0).abs() < 1e-6);
    }

    #[test]
    fn test_huber_kernel() {
        let kernel = RobustKernel::Huber(1.0);

        // Within threshold: weight = 1
        assert!((kernel.weight(0.5) - 1.0).abs() < 1e-6);

        // Beyond threshold: weight < 1
        assert!(kernel.weight(2.0) < 1.0);
        assert!((kernel.weight(2.0) - 0.5).abs() < 1e-6);

        // Cost grows linearly beyond threshold (less than quadratic)
        let cost_at_2 = kernel.cost(2.0);
        let quadratic_at_2 = 4.0;
        assert!(cost_at_2 < quadratic_at_2);
    }

    #[test]
    fn test_cauchy_kernel() {
        let kernel = RobustKernel::Cauchy(1.0);

        // Weight decreases with residual magnitude
        let w0 = kernel.weight(0.0);
        let w1 = kernel.weight(1.0);
        let w5 = kernel.weight(5.0);

        assert!((w0 - 1.0).abs() < 1e-6);
        assert!(w1 < w0);
        assert!(w5 < w1);

        // Cost grows sub-quadratically
        let cost_at_5 = kernel.cost(5.0);
        let quadratic_at_5 = 25.0;
        assert!(cost_at_5 < quadratic_at_5);
    }

    #[test]
    fn test_is_robust() {
        assert!(!RobustKernel::None.is_robust());
        assert!(RobustKernel::Huber(1.0).is_robust());
        assert!(RobustKernel::Cauchy(1.0).is_robust());
    }
}
