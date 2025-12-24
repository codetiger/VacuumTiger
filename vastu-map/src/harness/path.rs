//! Robot path segment for simulation and testing
//!
//! Provides PathSegment for defining robot motion commands.

/// A path segment with velocity commands and duration.
#[derive(Clone, Debug)]
pub struct PathSegment {
    /// Linear velocity in m/s (positive = forward)
    pub linear_vel: f32,
    /// Angular velocity in rad/s (positive = CCW)
    pub angular_vel: f32,
    /// Duration in seconds
    pub duration: f32,
}

impl PathSegment {
    /// Create a forward motion segment.
    pub fn forward(speed: f32, duration: f32) -> Self {
        Self {
            linear_vel: speed,
            angular_vel: 0.0,
            duration,
        }
    }

    /// Create a left turn segment (CCW).
    pub fn turn_left(angular_speed: f32, angle: f32) -> Self {
        Self {
            linear_vel: 0.0,
            angular_vel: angular_speed,
            duration: angle / angular_speed,
        }
    }

    /// Create a right turn segment (CW).
    pub fn turn_right(angular_speed: f32, angle: f32) -> Self {
        Self {
            linear_vel: 0.0,
            angular_vel: -angular_speed,
            duration: angle / angular_speed,
        }
    }

    /// Create an arc motion segment.
    pub fn arc(linear: f32, angular: f32, duration: f32) -> Self {
        Self {
            linear_vel: linear,
            angular_vel: angular,
            duration,
        }
    }

    /// Create a stop segment (no motion).
    pub fn stop(duration: f32) -> Self {
        Self {
            linear_vel: 0.0,
            angular_vel: 0.0,
            duration,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_forward_segment() {
        let seg = PathSegment::forward(0.5, 2.0);
        assert_eq!(seg.linear_vel, 0.5);
        assert_eq!(seg.angular_vel, 0.0);
        assert_eq!(seg.duration, 2.0);
    }

    #[test]
    fn test_turn_segments() {
        let left = PathSegment::turn_left(1.0, PI / 2.0);
        assert!(left.angular_vel > 0.0);
        assert_eq!(left.linear_vel, 0.0);

        let right = PathSegment::turn_right(1.0, PI / 2.0);
        assert!(right.angular_vel < 0.0);
        assert_eq!(right.linear_vel, 0.0);
    }

    #[test]
    fn test_stop_segment() {
        let seg = PathSegment::stop(5.0);
        assert_eq!(seg.linear_vel, 0.0);
        assert_eq!(seg.angular_vel, 0.0);
        assert_eq!(seg.duration, 5.0);
    }
}
