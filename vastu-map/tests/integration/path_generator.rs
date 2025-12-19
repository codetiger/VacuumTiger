//! Robot path generation utilities
//!
//! Provides predefined path patterns for integration tests:
//! - Rectangle (wall following)
//! - Spiral outward (coverage)
//! - Random walk (exploration)
//! - Loop closure (return to start)

use rand::prelude::*;
use std::f32::consts::PI;

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

/// Path generator utilities.
pub struct PathGenerator;

impl PathGenerator {
    /// Generate a spiral outward coverage pattern.
    ///
    /// # Arguments
    /// * `initial_radius` - Starting radius in meters
    /// * `expansion_rate` - Radius increase per loop in meters
    /// * `num_loops` - Number of complete loops
    /// * `linear_speed` - Linear velocity in m/s
    pub fn spiral_outward(
        initial_radius: f32,
        expansion_rate: f32,
        num_loops: usize,
        linear_speed: f32,
    ) -> Vec<PathSegment> {
        let mut segments = Vec::new();
        let mut radius = initial_radius;

        for _ in 0..num_loops {
            // Arc for one loop
            let circumference = 2.0 * PI * radius;
            let duration = circumference / linear_speed;
            let angular_vel = linear_speed / radius;

            segments.push(PathSegment::arc(linear_speed, angular_vel, duration));

            // Expand radius
            radius += expansion_rate;
        }

        segments
    }

    /// Generate a rectangle pattern (wall following).
    ///
    /// # Arguments
    /// * `width` - Rectangle width in meters
    /// * `height` - Rectangle height in meters
    /// * `linear_speed` - Linear velocity in m/s
    /// * `turn_speed` - Angular velocity during turns in rad/s
    pub fn rectangle(
        width: f32,
        height: f32,
        linear_speed: f32,
        turn_speed: f32,
    ) -> Vec<PathSegment> {
        let mut segments = Vec::new();

        for i in 0..4 {
            let length = if i % 2 == 0 { width } else { height };

            // Forward segment
            segments.push(PathSegment::forward(linear_speed, length / linear_speed));

            // Turn 90 degrees left
            segments.push(PathSegment::turn_left(turn_speed, PI / 2.0));
        }

        segments
    }

    /// Generate a random exploration pattern.
    ///
    /// # Arguments
    /// * `num_segments` - Number of forward-turn pairs
    /// * `linear_speed` - Linear velocity in m/s
    /// * `segment_length` - Base forward distance in meters
    /// * `turn_speed` - Angular velocity during turns in rad/s
    /// * `seed` - Random seed for reproducibility
    pub fn random_walk(
        num_segments: usize,
        linear_speed: f32,
        segment_length: f32,
        turn_speed: f32,
        seed: u64,
    ) -> Vec<PathSegment> {
        let mut rng = StdRng::seed_from_u64(seed);
        let mut segments = Vec::new();

        for _ in 0..num_segments {
            // Random forward distance (0.5x to 1.5x base length)
            let distance = segment_length * (0.5 + rng.random::<f32>());
            segments.push(PathSegment::forward(linear_speed, distance / linear_speed));

            // Random turn (-90 to +90 degrees)
            let angle = (rng.random::<f32>() - 0.5) * PI;
            if angle > 0.0 {
                segments.push(PathSegment::turn_left(turn_speed, angle));
            } else {
                segments.push(PathSegment::turn_right(turn_speed, -angle));
            }
        }

        segments
    }

    /// Generate a loop closure path (exploration + return to start).
    ///
    /// Reverses the given exploration path to return to the starting position.
    pub fn loop_closure(exploration_path: Vec<PathSegment>) -> Vec<PathSegment> {
        let mut full_path = exploration_path.clone();

        // Reverse the path to return to start
        for segment in exploration_path.into_iter().rev() {
            full_path.push(PathSegment {
                linear_vel: -segment.linear_vel,
                angular_vel: -segment.angular_vel,
                duration: segment.duration,
            });
        }

        full_path
    }

    /// Generate a full rotation path for observing surrounding walls.
    ///
    /// # Arguments
    /// * `turn_speed` - Angular velocity in rad/s
    /// * `pause_duration` - Pause at start and end for observations
    pub fn full_rotation(turn_speed: f32, pause_duration: f32) -> Vec<PathSegment> {
        vec![
            PathSegment::stop(pause_duration),
            PathSegment::turn_left(turn_speed, 2.0 * PI),
            PathSegment::stop(pause_duration),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rectangle_path() {
        let path = PathGenerator::rectangle(2.0, 1.0, 0.5, 1.0);
        assert_eq!(path.len(), 8); // 4 forward + 4 turn segments
    }

    #[test]
    fn test_spiral_path() {
        let path = PathGenerator::spiral_outward(0.5, 0.2, 3, 0.2);
        assert_eq!(path.len(), 3); // 3 loops
    }

    #[test]
    fn test_random_walk_deterministic() {
        let path1 = PathGenerator::random_walk(5, 0.2, 0.5, 0.5, 42);
        let path2 = PathGenerator::random_walk(5, 0.2, 0.5, 0.5, 42);
        assert_eq!(path1.len(), path2.len());
        // Same seed should produce identical paths
        for (s1, s2) in path1.iter().zip(path2.iter()) {
            assert_eq!(s1.duration, s2.duration);
        }
    }

    #[test]
    fn test_loop_closure() {
        let exploration = vec![
            PathSegment::forward(0.2, 1.0),
            PathSegment::turn_left(0.5, PI / 2.0),
        ];
        let full = PathGenerator::loop_closure(exploration.clone());
        assert_eq!(full.len(), exploration.len() * 2);
    }
}
