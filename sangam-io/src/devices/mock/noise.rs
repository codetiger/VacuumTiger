//! Configurable noise generator for simulation
//!
//! Provides Gaussian noise generation with deterministic seeding support.

use rand::prelude::*;
use rand::rngs::SmallRng;
use rand_distr::{Distribution, StandardNormal, Uniform};

/// Noise generator with configurable seed for reproducibility
#[derive(Clone)]
pub struct NoiseGenerator {
    rng: SmallRng,
}

impl NoiseGenerator {
    /// Create a new noise generator
    ///
    /// If seed is 0, uses random entropy for non-deterministic behavior.
    /// Otherwise, uses the provided seed for reproducible results.
    pub fn new(seed: u64) -> Self {
        let rng = if seed == 0 {
            SmallRng::from_entropy()
        } else {
            SmallRng::seed_from_u64(seed)
        };
        Self { rng }
    }

    /// Generate Gaussian noise with given standard deviation
    #[inline]
    pub fn gaussian(&mut self, stddev: f32) -> f32 {
        if stddev == 0.0 {
            return 0.0;
        }
        let n: f32 = self.rng.sample(StandardNormal);
        n * stddev
    }

    /// Generate Gaussian noise with bias and standard deviation
    #[inline]
    pub fn biased_gaussian(&mut self, bias: f32, stddev: f32) -> f32 {
        bias + self.gaussian(stddev)
    }

    /// Generate uniform random in [0, 1)
    #[inline]
    pub fn uniform(&mut self) -> f32 {
        Uniform::new(0.0f32, 1.0).sample(&mut self.rng)
    }

    /// Returns true with given probability
    #[inline]
    pub fn chance(&mut self, probability: f32) -> bool {
        self.uniform() < probability
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deterministic_seed() {
        let mut noise1 = NoiseGenerator::new(42);
        let mut noise2 = NoiseGenerator::new(42);

        for _ in 0..100 {
            assert_eq!(noise1.gaussian(1.0), noise2.gaussian(1.0));
        }
    }

    #[test]
    fn test_zero_stddev() {
        let mut noise = NoiseGenerator::new(42);
        for _ in 0..10 {
            assert_eq!(noise.gaussian(0.0), 0.0);
        }
    }

    #[test]
    fn test_chance_probability() {
        let mut noise = NoiseGenerator::new(42);
        let mut count = 0;
        let trials = 10000;

        for _ in 0..trials {
            if noise.chance(0.3) {
                count += 1;
            }
        }

        let ratio = count as f32 / trials as f32;
        assert!((ratio - 0.3).abs() < 0.05); // Within 5% of expected
    }
}
