//! Robust kernel functions for M-estimator weighting in ICP.
//!
//! Robust kernels down-weight outliers during scan matching to improve
//! convergence in the presence of noise, occlusions, and dynamic objects.
//!
//! # Available Kernels
//!
//! - [`RobustKernel::None`]: Standard least squares (no robustness)
//! - [`RobustKernel::Huber`]: Linear for large errors, quadratic for small
//! - [`RobustKernel::Cauchy`]: Heavy-tailed, very robust to outliers
//! - [`RobustKernel::Welsch`]: Smooth, strong outlier rejection (default)
//!
//! # Usage
//!
//! ```ignore
//! let kernel = RobustKernel::Welsch;
//! let scale = 0.10; // 10cm
//! let weight = kernel.weight(residual_sq, scale);
//! ```

/// Robust kernel type for M-estimator weighting.
///
/// These kernels are used to down-weight outlier correspondences during
/// ICP optimization, improving robustness to noise and occlusions.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum RobustKernel {
    /// No robust weighting (standard least squares).
    ///
    /// All correspondences are weighted equally. Fast but sensitive to outliers.
    None,

    /// Huber kernel: linear for large errors, quadratic for small.
    ///
    /// Good balance between robustness and efficiency. The influence function
    /// is bounded, making it robust to outliers while maintaining efficiency
    /// for inliers.
    ///
    /// Weight: `1` for `|r| < c`, `c/|r|` for `|r| >= c`
    Huber,

    /// Cauchy kernel: heavy-tailed, very robust to outliers.
    ///
    /// Also known as the Lorentzian kernel. Provides stronger outlier
    /// rejection than Huber but may be less efficient for inliers.
    ///
    /// Loss: `ρ(r) = (c²/2) * log(1 + (r/c)²)`
    /// Weight: `1 / (1 + (r/c)²)`
    Cauchy,

    /// Welsch kernel: smooth, strong outlier rejection.
    ///
    /// Provides the strongest outlier rejection with a smooth influence
    /// function that approaches zero for large residuals.
    ///
    /// Loss: `ρ(r) = (c²/2) * (1 - exp(-(r/c)²))`
    /// Weight: `exp(-(r/c)²)`
    #[default]
    Welsch,
}

impl RobustKernel {
    /// Compute the weight for a given squared residual.
    ///
    /// The weight is the derivative of the robust loss function, used in
    /// iteratively reweighted least squares (IRLS) optimization.
    ///
    /// # Arguments
    /// * `residual_sq` - Squared residual (distance²)
    /// * `scale` - Kernel scale parameter (in same units as residual)
    ///
    /// # Returns
    /// Weight in [0, 1] that down-weights outliers.
    ///
    /// # Example
    /// ```
    /// use dhruva_slam::algorithms::matching::RobustKernel;
    ///
    /// let kernel = RobustKernel::Welsch;
    /// let scale = 0.10; // 10cm
    ///
    /// // Small residual -> high weight
    /// let w1 = kernel.weight(0.001, scale); // 1cm² residual
    /// assert!(w1 > 0.9);
    ///
    /// // Large residual -> low weight
    /// let w2 = kernel.weight(0.25, scale); // 50cm² residual
    /// assert!(w2 < 0.1);
    /// ```
    #[inline(always)]
    pub fn weight(&self, residual_sq: f32, scale: f32) -> f32 {
        let c_sq = scale * scale;

        match self {
            RobustKernel::None => 1.0,
            RobustKernel::Huber => {
                // Huber weight: 1 for |r| < c, c/|r| for |r| >= c
                let r = residual_sq.sqrt();
                if r < scale { 1.0 } else { scale / r }
            }
            RobustKernel::Cauchy => {
                // Cauchy weight: 1 / (1 + (r/c)²)
                1.0 / (1.0 + residual_sq / c_sq)
            }
            RobustKernel::Welsch => {
                // Welsch weight: exp(-(r/c)²)
                (-residual_sq / c_sq).exp()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_none_kernel_always_one() {
        let kernel = RobustKernel::None;
        assert_eq!(kernel.weight(0.0, 0.1), 1.0);
        assert_eq!(kernel.weight(0.01, 0.1), 1.0);
        assert_eq!(kernel.weight(1.0, 0.1), 1.0);
        assert_eq!(kernel.weight(100.0, 0.1), 1.0);
    }

    #[test]
    fn test_huber_kernel() {
        let kernel = RobustKernel::Huber;
        let scale = 0.1;

        // Within threshold: weight = 1
        assert_eq!(kernel.weight(0.0, scale), 1.0);
        assert_eq!(kernel.weight(0.005 * 0.005, scale), 1.0); // 0.5cm < 10cm

        // Beyond threshold: weight = c/|r|
        let w = kernel.weight(0.04, scale); // 20cm residual
        assert!((w - 0.5).abs() < 0.01); // 0.1 / 0.2 = 0.5
    }

    #[test]
    fn test_cauchy_kernel() {
        let kernel = RobustKernel::Cauchy;
        let scale = 0.1;

        // Zero residual: weight = 1
        assert_eq!(kernel.weight(0.0, scale), 1.0);

        // Residual = scale: weight = 0.5
        let w = kernel.weight(0.01, scale); // r² = c²
        assert!((w - 0.5).abs() < 0.01);

        // Large residual: weight approaches 0
        let w_large = kernel.weight(1.0, scale);
        assert!(w_large < 0.01);
    }

    #[test]
    fn test_welsch_kernel() {
        let kernel = RobustKernel::Welsch;
        let scale = 0.1;

        // Zero residual: weight = 1
        assert!((kernel.weight(0.0, scale) - 1.0).abs() < 1e-6);

        // Residual = scale: weight = exp(-1) ≈ 0.368
        let w = kernel.weight(0.01, scale);
        assert!((w - (-1.0_f32).exp()).abs() < 0.01);

        // Large residual: weight approaches 0
        let w_large = kernel.weight(1.0, scale);
        assert!(w_large < 0.001);
    }

    #[test]
    fn test_default_is_welsch() {
        assert_eq!(RobustKernel::default(), RobustKernel::Welsch);
    }

    #[test]
    fn test_weights_decrease_with_residual() {
        for kernel in [
            RobustKernel::Huber,
            RobustKernel::Cauchy,
            RobustKernel::Welsch,
        ] {
            let scale = 0.1;
            let w1 = kernel.weight(0.001, scale);
            let w2 = kernel.weight(0.01, scale);
            let w3 = kernel.weight(0.1, scale);

            assert!(w1 >= w2, "{:?}: w1={} should be >= w2={}", kernel, w1, w2);
            assert!(w2 >= w3, "{:?}: w2={} should be >= w3={}", kernel, w2, w3);
        }
    }
}
