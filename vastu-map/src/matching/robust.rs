//! Robust cost functions for outlier rejection in ICP.
//!
//! These M-estimators compute weights that down-weight outliers based on
//! residual magnitude, enabling soft outlier rejection without hard thresholds.
//!
//! # Usage
//!
//! ```rust,ignore
//! use vastu_map::matching::RobustCostFunction;
//!
//! // Huber is recommended for most cases
//! let cost = RobustCostFunction::Huber { delta: 0.03 };
//!
//! // Compute weight for a residual
//! let weight = cost.weight(0.05);  // Returns ~0.6 (down-weighted)
//! ```
//!
//! # M-Estimator Comparison
//!
//! | Estimator | Behavior | Best For |
//! |-----------|----------|----------|
//! | Huber | Linear beyond delta | General use, balanced |
//! | Cauchy | Heavy-tailed | Cluttered environments |
//! | Tukey | Zero weight for outliers | Aggressive rejection |
//! | Geman-McClure | Redescending | Multimodal errors |

/// Robust cost function for weighted least squares (IRLS).
///
/// Computes weights that down-weight outliers based on residual magnitude.
/// Use with ICP to improve accuracy in presence of noise and clutter.
#[derive(Clone, Debug, PartialEq)]
pub enum RobustCostFunction {
    /// No robust weighting (standard least squares).
    /// Weight is always 1.0 regardless of residual.
    None,

    /// Huber loss: quadratic near zero, linear beyond delta.
    ///
    /// Good balance between robustness and efficiency.
    /// Recommended as default choice.
    ///
    /// Weight: 1.0 if |r| ≤ δ, else δ/|r|
    Huber {
        /// Threshold where loss transitions from quadratic to linear.
        /// Default: 0.03 (3cm)
        delta: f32,
    },

    /// Cauchy loss (Lorentzian): heavy-tailed distribution.
    ///
    /// More aggressive outlier rejection than Huber.
    /// Good for cluttered environments with many outliers.
    ///
    /// Weight: 1 / (1 + (r/scale)²)
    Cauchy {
        /// Scale parameter controlling tail heaviness.
        /// Default: 0.05
        scale: f32,
    },

    /// Tukey biweight: zero weight for large residuals.
    ///
    /// Most aggressive - completely ignores outliers beyond threshold.
    /// Use when outliers are clearly distinguishable.
    ///
    /// Weight: (1 - (r/c)²)² if |r| ≤ c, else 0
    Tukey {
        /// Cutoff threshold. Residuals beyond this get zero weight.
        /// Default: 0.1
        c: f32,
    },

    /// Geman-McClure: redescending M-estimator.
    ///
    /// Good for multimodal error distributions.
    /// Weights decrease rapidly for large residuals.
    ///
    /// Weight: 1 / (1 + (r/scale)²)²
    GemanMcClure {
        /// Scale parameter.
        /// Default: 0.05
        scale: f32,
    },
}

impl RobustCostFunction {
    /// Compute weight for IRLS (Iteratively Reweighted Least Squares).
    ///
    /// Returns a weight in [0, 1] that should multiply the correspondence weight.
    /// Small residuals get weight ≈ 1, large residuals get down-weighted.
    ///
    /// # Arguments
    /// * `residual` - The point-to-line distance (can be negative for signed distance)
    ///
    /// # Returns
    /// Weight in [0, 1] to apply to this correspondence
    #[inline]
    pub fn weight(&self, residual: f32) -> f32 {
        let r = residual.abs();
        match self {
            Self::None => 1.0,
            Self::Huber { delta } => {
                if r <= *delta {
                    1.0
                } else {
                    *delta / r
                }
            }
            Self::Cauchy { scale } => {
                let u = r / scale;
                1.0 / (1.0 + u * u)
            }
            Self::Tukey { c } => {
                if r <= *c {
                    let u = r / c;
                    let v = 1.0 - u * u;
                    v * v
                } else {
                    0.0
                }
            }
            Self::GemanMcClure { scale } => {
                let u = r / scale;
                let v = 1.0 + u * u;
                1.0 / (v * v)
            }
        }
    }

    /// Create Huber cost with default threshold (3cm).
    #[inline]
    pub fn default_huber() -> Self {
        Self::Huber { delta: 0.03 }
    }

    /// Create Cauchy cost with default scale (5cm).
    #[inline]
    pub fn default_cauchy() -> Self {
        Self::Cauchy { scale: 0.05 }
    }

    /// Create Tukey cost with default cutoff (10cm).
    #[inline]
    pub fn default_tukey() -> Self {
        Self::Tukey { c: 0.1 }
    }

    /// Create Geman-McClure cost with default scale (5cm).
    #[inline]
    pub fn default_geman_mcclure() -> Self {
        Self::GemanMcClure { scale: 0.05 }
    }

    /// Check if this is the None variant (no robust weighting).
    #[inline]
    pub fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }

    /// Check if this cost function is enabled (not None).
    #[inline]
    pub fn is_enabled(&self) -> bool {
        !self.is_none()
    }
}

impl Default for RobustCostFunction {
    /// Default to Huber with 3cm threshold.
    fn default() -> Self {
        Self::default_huber()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_none_always_returns_one() {
        let cost = RobustCostFunction::None;
        assert_eq!(cost.weight(0.0), 1.0);
        assert_eq!(cost.weight(0.1), 1.0);
        assert_eq!(cost.weight(1.0), 1.0);
        assert_eq!(cost.weight(-0.5), 1.0);
    }

    #[test]
    fn test_huber_within_delta() {
        let cost = RobustCostFunction::Huber { delta: 0.05 };
        assert_eq!(cost.weight(0.0), 1.0);
        assert_eq!(cost.weight(0.03), 1.0);
        assert_eq!(cost.weight(0.05), 1.0);
        assert_eq!(cost.weight(-0.03), 1.0);
    }

    #[test]
    fn test_huber_beyond_delta() {
        let cost = RobustCostFunction::Huber { delta: 0.05 };
        // At r = 0.1, weight = 0.05 / 0.1 = 0.5
        assert_relative_eq!(cost.weight(0.1), 0.5, epsilon = 1e-6);
        // At r = 0.2, weight = 0.05 / 0.2 = 0.25
        assert_relative_eq!(cost.weight(0.2), 0.25, epsilon = 1e-6);
    }

    #[test]
    fn test_cauchy() {
        let cost = RobustCostFunction::Cauchy { scale: 0.05 };
        // At r = 0, weight = 1
        assert_eq!(cost.weight(0.0), 1.0);
        // At r = scale, weight = 1 / (1 + 1) = 0.5
        assert_relative_eq!(cost.weight(0.05), 0.5, epsilon = 1e-6);
        // At r = 2*scale, weight = 1 / (1 + 4) = 0.2
        assert_relative_eq!(cost.weight(0.1), 0.2, epsilon = 1e-6);
    }

    #[test]
    fn test_tukey_within_cutoff() {
        let cost = RobustCostFunction::Tukey { c: 0.1 };
        // At r = 0, weight = 1
        assert_eq!(cost.weight(0.0), 1.0);
        // At r = c/2, weight = (1 - 0.25)^2 = 0.5625
        assert_relative_eq!(cost.weight(0.05), 0.5625, epsilon = 1e-6);
    }

    #[test]
    fn test_tukey_beyond_cutoff() {
        let cost = RobustCostFunction::Tukey { c: 0.1 };
        // Beyond cutoff, weight = 0
        assert_eq!(cost.weight(0.1), 0.0);
        assert_eq!(cost.weight(0.2), 0.0);
        assert_eq!(cost.weight(1.0), 0.0);
    }

    #[test]
    fn test_geman_mcclure() {
        let cost = RobustCostFunction::GemanMcClure { scale: 0.05 };
        // At r = 0, weight = 1
        assert_eq!(cost.weight(0.0), 1.0);
        // At r = scale, weight = 1 / (1 + 1)^2 = 0.25
        assert_relative_eq!(cost.weight(0.05), 0.25, epsilon = 1e-6);
    }

    #[test]
    fn test_default_is_huber() {
        let cost = RobustCostFunction::default();
        assert!(matches!(cost, RobustCostFunction::Huber { delta } if (delta - 0.03).abs() < 1e-6));
    }

    #[test]
    fn test_is_none() {
        assert!(RobustCostFunction::None.is_none());
        assert!(!RobustCostFunction::default_huber().is_none());
    }

    #[test]
    fn test_is_enabled() {
        assert!(!RobustCostFunction::None.is_enabled());
        assert!(RobustCostFunction::default_huber().is_enabled());
    }

    #[test]
    fn test_weights_are_symmetric() {
        let costs = [
            RobustCostFunction::None,
            RobustCostFunction::default_huber(),
            RobustCostFunction::default_cauchy(),
            RobustCostFunction::default_tukey(),
            RobustCostFunction::default_geman_mcclure(),
        ];

        for cost in &costs {
            assert_eq!(cost.weight(0.03), cost.weight(-0.03));
            assert_eq!(cost.weight(0.1), cost.weight(-0.1));
        }
    }

    #[test]
    fn test_weights_decrease_with_residual() {
        let costs = [
            RobustCostFunction::default_huber(),
            RobustCostFunction::default_cauchy(),
            RobustCostFunction::default_geman_mcclure(),
        ];

        for cost in &costs {
            let w0 = cost.weight(0.0);
            let w1 = cost.weight(0.05);
            let w2 = cost.weight(0.1);
            let w3 = cost.weight(0.2);

            assert!(w0 >= w1, "Weight should decrease: {} >= {}", w0, w1);
            assert!(w1 >= w2, "Weight should decrease: {} >= {}", w1, w2);
            assert!(w2 >= w3, "Weight should decrease: {} >= {}", w2, w3);
        }
    }
}
