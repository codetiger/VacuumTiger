//! Adaptive Levenberg-Marquardt optimizer.

/// Adaptive Levenberg-Marquardt optimizer for Gauss-Newton refinement.
///
/// Automatically adjusts the damping parameter (lambda) based on step quality:
/// - Good steps (actual reduction close to predicted): decrease lambda
/// - Bad steps (actual reduction much less than predicted): increase lambda
///
/// This provides faster convergence than fixed damping while maintaining stability.
pub(super) struct AdaptiveLM {
    /// Current damping parameter
    lambda: f32,
    /// Factor to increase/decrease lambda
    lambda_factor: f32,
    /// Minimum lambda value
    min_lambda: f32,
    /// Maximum lambda value
    max_lambda: f32,
}

impl AdaptiveLM {
    /// Create a new adaptive LM optimizer with default parameters.
    pub fn new() -> Self {
        Self {
            lambda: 1e-3,
            lambda_factor: 10.0,
            min_lambda: 1e-7,
            max_lambda: 1e7,
        }
    }

    /// Get the current damping value.
    pub fn damping(&self) -> f32 {
        self.lambda
    }

    /// Update lambda based on step quality.
    ///
    /// `rho` is the ratio of actual cost reduction to predicted cost reduction.
    /// - rho > 0.75: Very good step, decrease lambda aggressively
    /// - rho > 0.25: Good step, decrease lambda
    /// - rho > 0.0: Acceptable step, keep lambda
    /// - rho <= 0.0: Bad step (cost increased), increase lambda
    pub fn update(&mut self, rho: f32) {
        if rho > 0.75 {
            // Very good step - decrease lambda
            self.lambda = (self.lambda / self.lambda_factor).max(self.min_lambda);
        } else if rho > 0.25 {
            // Good step - slightly decrease lambda
            self.lambda = (self.lambda / (self.lambda_factor.sqrt())).max(self.min_lambda);
        } else if rho < 0.0 {
            // Bad step - increase lambda significantly
            self.lambda = (self.lambda * self.lambda_factor).min(self.max_lambda);
        }
        // 0.0 <= rho <= 0.25: marginal step, keep lambda unchanged
    }

    /// Reject a step (cost increased), increase lambda.
    pub fn reject_step(&mut self) {
        self.lambda = (self.lambda * self.lambda_factor).min(self.max_lambda);
    }

    /// Check if optimizer has reached maximum damping (likely stuck).
    pub fn is_stuck(&self) -> bool {
        self.lambda >= self.max_lambda * 0.99
    }
}
