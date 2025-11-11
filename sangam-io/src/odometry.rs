//! Odometry tracking for SLAM integration

use crate::config::SangamConfig;
use std::time::{Duration, Instant};

/// Tracks odometry deltas for SLAM algorithms
pub struct OdometryTracker {
    /// Last read encoder values
    last_left_encoder: Option<i32>,
    last_right_encoder: Option<i32>,

    /// Accumulated deltas since last reset
    accumulated_distance: f32, // meters
    accumulated_angle: f32, // radians
    accumulated_left: f32,  // left wheel distance
    accumulated_right: f32, // right wheel distance

    /// Last delta calculation time
    last_update: Instant,

    /// Configuration
    config: SangamConfig,

    /// Last encoder log time (for throttling)
    last_encoder_log: Option<Instant>,

    /// Last odometry log time (for throttling)
    last_odometry_log: Option<Instant>,
}

/// Odometry delta since last query
#[derive(Debug, Clone)]
pub struct OdometryDelta {
    /// Forward distance traveled (meters)
    pub distance: f32,

    /// Rotation angle (radians, positive = CCW)
    pub angle: f32,

    /// Left wheel distance (meters)
    pub left_wheel: f32,

    /// Right wheel distance (meters)
    pub right_wheel: f32,

    /// Time since last delta
    pub delta_time: f32,

    /// Timestamp of this reading
    pub timestamp: Instant,
}

impl OdometryTracker {
    /// Create new odometry tracker
    pub fn new(config: SangamConfig) -> Self {
        log::debug!(
            "OdometryTracker: Initialized with wheel_base={:.3}m, ticks_per_rev={}",
            config.wheel_base,
            config.ticks_per_revolution
        );

        Self {
            last_left_encoder: None,
            last_right_encoder: None,
            accumulated_distance: 0.0,
            accumulated_angle: 0.0,
            accumulated_left: 0.0,
            accumulated_right: 0.0,
            last_update: Instant::now(),
            config,
            last_encoder_log: None,
            last_odometry_log: None,
        }
    }

    /// Update with new encoder readings
    pub fn update_encoders(&mut self, left_encoder: i32, right_encoder: i32) {
        // Calculate delta ticks
        let (left_delta, right_delta) = match (self.last_left_encoder, self.last_right_encoder) {
            (Some(last_left), Some(last_right)) => {
                let left_d = left_encoder - last_left;
                let right_d = right_encoder - last_right;
                (left_d, right_d)
            }
            _ => {
                // First reading, no delta
                self.last_left_encoder = Some(left_encoder);
                self.last_right_encoder = Some(right_encoder);
                log::debug!(
                    "OdometryTracker: Initial encoder readings - L={}, R={}",
                    left_encoder,
                    right_encoder
                );
                return;
            }
        };

        // Check for large encoder jumps that might indicate an error
        const MAX_REASONABLE_DELTA: i32 = 10000; // Adjust based on your hardware
        if left_delta.abs() > MAX_REASONABLE_DELTA || right_delta.abs() > MAX_REASONABLE_DELTA {
            log::warn!(
                "OdometryTracker: Large encoder jump detected - ΔL={}, ΔR={} (possible overflow or error)",
                left_delta,
                right_delta
            );
        }

        // Update last encoder values
        self.last_left_encoder = Some(left_encoder);
        self.last_right_encoder = Some(right_encoder);

        // Convert ticks to distances
        let left_distance = self.config.ticks_to_meters(left_delta);
        let right_distance = self.config.ticks_to_meters(right_delta);

        // Calculate motion using differential drive model
        let distance = (left_distance + right_distance) / 2.0;
        let angle = (right_distance - left_distance) / self.config.wheel_base;

        // Accumulate deltas
        self.accumulated_distance += distance;
        self.accumulated_angle += angle;
        self.accumulated_left += left_distance;
        self.accumulated_right += right_distance;

        // Log encoder updates periodically (throttled to 1Hz)
        let should_log = if let Some(last_log) = self.last_encoder_log {
            last_log.elapsed() >= Duration::from_secs(1)
        } else {
            false
        };

        if should_log && (left_delta != 0 || right_delta != 0) {
            log::debug!(
                "OdometryTracker: Encoder update - L={}, R={}, ΔL={}, ΔR={}, dist={:.3}m, angle={:.3}rad",
                left_encoder,
                right_encoder,
                left_delta,
                right_delta,
                distance,
                angle
            );
            self.last_encoder_log = Some(Instant::now());
        }
    }

    /// Get accumulated odometry delta and reset
    pub fn get_delta_and_reset(&mut self) -> OdometryDelta {
        let now = Instant::now();
        let delta_time = now.duration_since(self.last_update).as_secs_f32();

        let delta = OdometryDelta {
            distance: self.accumulated_distance,
            angle: self.accumulated_angle,
            left_wheel: self.accumulated_left,
            right_wheel: self.accumulated_right,
            delta_time,
            timestamp: now,
        };

        // Log if there's significant motion
        if self.accumulated_distance.abs() > 0.001 || self.accumulated_angle.abs() > 0.001 {
            log::debug!(
                "OdometryTracker: Delta query - dist={:.3}m, angle={:.3}rad ({:.1}°) over {:.2}s",
                self.accumulated_distance,
                self.accumulated_angle,
                self.accumulated_angle.to_degrees(),
                delta_time
            );

            // Log detailed values periodically (throttled)
            let should_log_detailed = if let Some(last_log) = self.last_odometry_log {
                last_log.elapsed() >= Duration::from_secs(5)
            } else {
                true
            };

            if should_log_detailed {
                log::debug!(
                    "OdometryTracker: Wheel deltas - L={:.3}m, R={:.3}m",
                    self.accumulated_left,
                    self.accumulated_right
                );
                self.last_odometry_log = Some(now);
            }
        }

        // Reset accumulators
        self.accumulated_distance = 0.0;
        self.accumulated_angle = 0.0;
        self.accumulated_left = 0.0;
        self.accumulated_right = 0.0;
        self.last_update = now;

        delta
    }

    /// Reset odometry tracking
    pub fn reset(&mut self) {
        log::debug!("OdometryTracker: Reset - clearing encoder history and accumulators");

        self.last_left_encoder = None;
        self.last_right_encoder = None;
        self.accumulated_distance = 0.0;
        self.accumulated_angle = 0.0;
        self.accumulated_left = 0.0;
        self.accumulated_right = 0.0;
        self.last_update = Instant::now();
        self.last_encoder_log = None;
        self.last_odometry_log = None;
    }

    /// Get current accumulated values without reset
    pub fn peek(&self) -> OdometryDelta {
        let now = Instant::now();
        let delta_time = now.duration_since(self.last_update).as_secs_f32();

        OdometryDelta {
            distance: self.accumulated_distance,
            angle: self.accumulated_angle,
            left_wheel: self.accumulated_left,
            right_wheel: self.accumulated_right,
            delta_time,
            timestamp: now,
        }
    }
}

impl Default for OdometryDelta {
    fn default() -> Self {
        Self {
            distance: 0.0,
            angle: 0.0,
            left_wheel: 0.0,
            right_wheel: 0.0,
            delta_time: 0.0,
            timestamp: Instant::now(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_odometry_tracking() {
        let config = SangamConfig::default();
        let mut tracker = OdometryTracker::new(config);

        // First update establishes baseline
        tracker.update_encoders(1000, 1000);
        let delta = tracker.get_delta_and_reset();
        assert_eq!(delta.distance, 0.0);

        // Move forward (both wheels same)
        tracker.update_encoders(2000, 2000);
        let delta = tracker.get_delta_and_reset();
        assert!(delta.distance > 0.0);
        assert_eq!(delta.angle, 0.0);

        // Rotate in place (opposite directions)
        tracker.update_encoders(2500, 1500);
        let delta = tracker.get_delta_and_reset();
        assert_eq!(delta.distance, 0.0);
        assert!(delta.angle.abs() > 0.0);
    }

    #[test]
    fn test_reset() {
        let config = SangamConfig::default();
        let mut tracker = OdometryTracker::new(config);

        tracker.update_encoders(1000, 1000);
        tracker.update_encoders(2000, 2000);

        tracker.reset();
        let delta = tracker.peek();
        assert_eq!(delta.distance, 0.0);
        assert_eq!(delta.angle, 0.0);
    }
}
