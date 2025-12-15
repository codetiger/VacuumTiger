//! Encoder simulator for mock device
//!
//! Simulates wheel encoder ticks with configurable slip and noise.

use super::config::EncoderConfig;
use super::noise::NoiseGenerator;

/// Encoder simulator for differential drive robot
pub struct EncoderSimulator {
    config: EncoderConfig,
    ticks_per_meter: f32,
    noise: NoiseGenerator,
    /// Accumulated fractional ticks (left wheel)
    left_accumulator: f32,
    /// Accumulated fractional ticks (right wheel)
    right_accumulator: f32,
    /// Current tick count (left, wrapping u16)
    left_ticks: u16,
    /// Current tick count (right, wrapping u16)
    right_ticks: u16,
}

impl EncoderSimulator {
    /// Create new encoder simulator
    pub fn new(config: &EncoderConfig, ticks_per_meter: f32, noise: NoiseGenerator) -> Self {
        Self {
            config: config.clone(),
            ticks_per_meter,
            noise,
            left_accumulator: 0.0,
            right_accumulator: 0.0,
            left_ticks: 0,
            right_ticks: 0,
        }
    }

    /// Update encoder state based on wheel velocities
    ///
    /// # Arguments
    /// * `left_vel` - Left wheel velocity (m/s)
    /// * `right_vel` - Right wheel velocity (m/s)
    /// * `dt` - Time step (seconds)
    ///
    /// # Returns
    /// (left_ticks, right_ticks) - Current encoder tick counts (wrapping u16)
    pub fn update(&mut self, left_vel: f32, right_vel: f32, dt: f32) -> (u16, u16) {
        // Calculate distance traveled by each wheel
        let left_distance = left_vel * dt;
        let right_distance = right_vel * dt;

        // Apply slip noise (multiplicative)
        let left_slip =
            1.0 + self.config.noise.slip_bias + self.noise.gaussian(self.config.noise.slip_stddev);
        let right_slip =
            1.0 + self.config.noise.slip_bias + self.noise.gaussian(self.config.noise.slip_stddev);

        // Convert distance to ticks with slip
        let left_ticks_float = left_distance * self.ticks_per_meter * left_slip;
        let right_ticks_float = right_distance * self.ticks_per_meter * right_slip;

        // Add quantization noise if enabled
        let left_noise = if self.config.noise.quantization_noise {
            self.noise.gaussian(0.5) // ±0.5 tick jitter
        } else {
            0.0
        };
        let right_noise = if self.config.noise.quantization_noise {
            self.noise.gaussian(0.5)
        } else {
            0.0
        };

        // Accumulate fractional ticks
        self.left_accumulator += left_ticks_float + left_noise;
        self.right_accumulator += right_ticks_float + right_noise;

        // Extract whole ticks
        let left_whole = self.left_accumulator.trunc() as i32;
        let right_whole = self.right_accumulator.trunc() as i32;

        // Keep fractional part for next iteration
        self.left_accumulator = self.left_accumulator.fract();
        self.right_accumulator = self.right_accumulator.fract();

        // Update tick counters (wrapping u16 arithmetic)
        self.left_ticks = self.left_ticks.wrapping_add(left_whole as u16);
        self.right_ticks = self.right_ticks.wrapping_add(right_whole as u16);

        (self.left_ticks, self.right_ticks)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encoder_forward_motion() {
        let config = EncoderConfig::default();
        let noise = NoiseGenerator::new(42);
        let mut encoder = EncoderSimulator::new(&config, 4464.0, noise);

        // Move forward at 0.3 m/s for 1 second
        let dt = 0.01; // 10ms steps
        let velocity = 0.3; // m/s

        for _ in 0..100 {
            encoder.update(velocity, velocity, dt);
        }

        // After 1 second at 0.3 m/s, expect ~1339 ticks (0.3 * 4464)
        let (left, right) = (encoder.left_ticks, encoder.right_ticks);

        // Allow for noise variation (±5%)
        assert!(left > 1200 && left < 1500, "left={}", left);
        assert!(right > 1200 && right < 1500, "right={}", right);
    }

    #[test]
    fn test_encoder_rotation() {
        let config = EncoderConfig::default();
        let noise = NoiseGenerator::new(42);
        let mut encoder = EncoderSimulator::new(&config, 4464.0, noise);

        // Pure rotation: left backward, right forward
        let dt = 0.01;
        let velocity = 0.1;

        for _ in 0..100 {
            encoder.update(-velocity, velocity, dt);
        }

        // Wheels should move in opposite directions
        // Left wheel going backward wraps around
        let (left, right) = (encoder.left_ticks, encoder.right_ticks);

        // Right should have positive ticks, left should wrap (be near max u16)
        assert!(right > 0);
        // Left wraps around to near u16::MAX
        assert!(left > 60000 || left < 1000);
    }

    #[test]
    fn test_encoder_wrapping() {
        let mut config = EncoderConfig::default();
        config.noise.slip_stddev = 0.0;
        config.noise.quantization_noise = false;
        let noise = NoiseGenerator::new(42);
        let mut encoder = EncoderSimulator::new(&config, 4464.0, noise);

        // Force encoder near max value
        encoder.left_ticks = u16::MAX - 100;

        // Move to cause overflow
        let (left, _) = encoder.update(0.1, 0.0, 1.0); // ~446 ticks

        // Should wrap correctly
        assert!(left < 500); // Wrapped around
    }
}
