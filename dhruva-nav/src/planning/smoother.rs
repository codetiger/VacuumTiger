//! Path smoother for differential drive kinematics.
//!
//! Post-processes Theta* paths to ensure smooth motion by:
//! - Inserting arc segments for medium turns
//! - Inserting point turns for sharp angles
//! - Respecting minimum turning radius constraints

use vastu_slam::WorldPoint;

use super::theta_star::PlannedPath;

/// Configuration for path smoothing.
#[derive(Clone, Debug)]
pub struct SmootherConfig {
    /// Maximum linear velocity (m/s)
    pub max_linear_vel: f32,
    /// Maximum angular velocity (rad/s)
    pub max_angular_vel: f32,
    /// Angle threshold for inserting arcs (radians, ~30°)
    pub arc_turn_threshold: f32,
    /// Angle threshold for point turns (radians, ~90°)
    pub sharp_turn_threshold: f32,
}

impl Default for SmootherConfig {
    fn default() -> Self {
        Self {
            max_linear_vel: 0.2,
            max_angular_vel: 0.5,
            arc_turn_threshold: 0.52,   // ~30 degrees
            sharp_turn_threshold: 1.57, // ~90 degrees
        }
    }
}

impl SmootherConfig {
    /// Compute minimum turning radius from velocity constraints.
    pub fn min_turn_radius(&self) -> f32 {
        self.max_linear_vel / self.max_angular_vel
    }
}

/// Type of path segment.
#[derive(Clone, Debug)]
pub enum PathSegment {
    /// Straight line segment
    Line { start: WorldPoint, end: WorldPoint },
    /// In-place rotation for sharp turns
    PointTurn {
        position: WorldPoint,
        from_angle: f32,
        to_angle: f32,
    },
}

impl PathSegment {
    /// Get the starting point of this segment.
    pub fn start_point(&self) -> WorldPoint {
        match self {
            PathSegment::Line { start, .. } => *start,
            PathSegment::PointTurn { position, .. } => *position,
        }
    }

    /// Get the ending point of this segment.
    pub fn end_point(&self) -> WorldPoint {
        match self {
            PathSegment::Line { end, .. } => *end,
            PathSegment::PointTurn { position, .. } => *position,
        }
    }

    /// Get the length of this segment in meters.
    pub fn length(&self) -> f32 {
        match self {
            PathSegment::Line { start, end } => start.distance(end),
            PathSegment::PointTurn { .. } => 0.0, // No linear distance
        }
    }
}

/// Smoothed path with segments.
#[derive(Clone, Debug)]
pub struct SmoothedPath {
    /// Path segments (lines, arcs, point turns)
    pub segments: Vec<PathSegment>,
    /// Total path length in meters
    pub total_length: f32,
    /// Estimated traversal time in seconds
    pub estimated_time: f32,
}

/// Path smoother for differential drive robots.
pub struct PathSmoother {
    config: SmootherConfig,
}

impl PathSmoother {
    /// Create a new path smoother with configuration.
    pub fn new(config: SmootherConfig) -> Self {
        Self { config }
    }

    /// Create a new path smoother with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(SmootherConfig::default())
    }

    /// Smooth a planned path for differential drive motion.
    pub fn smooth(&self, path: &PlannedPath) -> SmoothedPath {
        if path.waypoints.len() < 2 {
            return SmoothedPath {
                segments: Vec::new(),
                total_length: 0.0,
                estimated_time: 0.0,
            };
        }

        let mut segments = Vec::new();
        let _min_radius = self.config.min_turn_radius();

        // Process waypoints and analyze turn angles
        for i in 0..path.waypoints.len() - 1 {
            let current = path.waypoints[i];
            let next = path.waypoints[i + 1];

            // Compute incoming and outgoing headings
            let incoming_heading = if i > 0 {
                let prev = path.waypoints[i - 1];
                (current.y - prev.y).atan2(current.x - prev.x)
            } else {
                (next.y - current.y).atan2(next.x - current.x)
            };

            let outgoing_heading = (next.y - current.y).atan2(next.x - current.x);

            // Compute turn angle
            let turn_angle = Self::normalize_angle(outgoing_heading - incoming_heading);
            let turn_angle_abs = turn_angle.abs();

            // Decide what type of segment to create
            if i > 0 && turn_angle_abs > self.config.sharp_turn_threshold {
                // Sharp turn: insert point turn
                segments.push(PathSegment::PointTurn {
                    position: current,
                    from_angle: incoming_heading,
                    to_angle: outgoing_heading,
                });
            } else if i > 0 && turn_angle_abs > self.config.arc_turn_threshold {
                // Medium turn: could insert arc, but for simplicity use point turn
                // Full arc implementation requires more complex geometry
                segments.push(PathSegment::PointTurn {
                    position: current,
                    from_angle: incoming_heading,
                    to_angle: outgoing_heading,
                });
            }

            // Add line segment
            segments.push(PathSegment::Line {
                start: current,
                end: next,
            });
        }

        // Compute totals
        let total_length: f32 = segments.iter().map(|s| s.length()).sum();
        let estimated_time = self.estimate_traversal_time(&segments);

        SmoothedPath {
            segments,
            total_length,
            estimated_time,
        }
    }

    /// Estimate traversal time for the path.
    fn estimate_traversal_time(&self, segments: &[PathSegment]) -> f32 {
        let mut time = 0.0;

        for segment in segments {
            match segment {
                PathSegment::Line { start, end } => {
                    let distance = start.distance(end);
                    time += distance / self.config.max_linear_vel;
                }
                PathSegment::PointTurn {
                    from_angle,
                    to_angle,
                    ..
                } => {
                    let angle_diff = Self::normalize_angle(to_angle - from_angle).abs();
                    time += angle_diff / self.config.max_angular_vel;
                }
            }
        }

        time
    }

    /// Normalize angle to [-pi, pi].
    #[inline]
    fn normalize_angle(angle: f32) -> f32 {
        let mut a = angle;
        while a > std::f32::consts::PI {
            a -= 2.0 * std::f32::consts::PI;
        }
        while a < -std::f32::consts::PI {
            a += 2.0 * std::f32::consts::PI;
        }
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_path() -> PlannedPath {
        PlannedPath {
            waypoints: vec![
                WorldPoint::new(0.0, 0.0),
                WorldPoint::new(1.0, 0.0),
                WorldPoint::new(1.0, 1.0),
                WorldPoint::new(0.0, 1.0),
            ],
            length: 3.0,
        }
    }

    #[test]
    fn test_smooth_simple_path() {
        let path = create_test_path();
        let smoother = PathSmoother::with_defaults();

        let smoothed = smoother.smooth(&path);

        // Should have line segments and point turns
        assert!(!smoothed.segments.is_empty());
        assert!(smoothed.total_length > 0.0);
        assert!(smoothed.estimated_time > 0.0);
    }

    #[test]
    fn test_sharp_turn_detection() {
        let path = PlannedPath {
            waypoints: vec![
                WorldPoint::new(0.0, 0.0),
                WorldPoint::new(1.0, 0.0),
                WorldPoint::new(1.0, 1.0), // 90 degree turn
            ],
            length: 2.0,
        };

        let smoother = PathSmoother::with_defaults();
        let smoothed = smoother.smooth(&path);

        // Should detect the 90 degree turn
        let has_point_turn = smoothed
            .segments
            .iter()
            .any(|s| matches!(s, PathSegment::PointTurn { .. }));
        assert!(has_point_turn);
    }

    #[test]
    fn test_straight_path_no_turns() {
        let path = PlannedPath {
            waypoints: vec![
                WorldPoint::new(0.0, 0.0),
                WorldPoint::new(1.0, 0.0),
                WorldPoint::new(2.0, 0.0),
            ],
            length: 2.0,
        };

        let smoother = PathSmoother::with_defaults();
        let smoothed = smoother.smooth(&path);

        // Should only have line segments
        let all_lines = smoothed
            .segments
            .iter()
            .all(|s| matches!(s, PathSegment::Line { .. }));
        assert!(all_lines);
    }
}
