//! Collision event handling for exploration.
//!
//! Provides types for representing collision events from sensors
//! and computing virtual walls to add to the map.

use crate::core::Point2D;
use crate::features::Line2D;
use std::f32::consts::FRAC_PI_2;

/// Type of collision detected by sensors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionType {
    /// Left bumper triggered.
    BumperLeft,
    /// Right bumper triggered.
    BumperRight,
    /// Both bumpers triggered (head-on collision).
    BumperBoth,
    /// Cliff sensor triggered.
    Cliff,
}

impl CollisionType {
    /// Check if this is a bumper collision.
    pub fn is_bumper(&self) -> bool {
        matches!(
            self,
            CollisionType::BumperLeft | CollisionType::BumperRight | CollisionType::BumperBoth
        )
    }

    /// Check if this is a cliff detection.
    pub fn is_cliff(&self) -> bool {
        matches!(self, CollisionType::Cliff)
    }
}

/// Collision event from sensors.
///
/// This struct represents a collision detected by the robot's sensors.
/// It is provided by DhruvaSLAM (or the test harness) to the exploration
/// controller, which uses it to update the map and plan recovery.
#[derive(Clone, Debug)]
pub struct CollisionEvent {
    /// Type of collision detected.
    pub collision_type: CollisionType,
    /// Estimated collision point in world frame.
    pub point: Point2D,
    /// Robot heading at time of collision (radians).
    pub heading: f32,
}

impl CollisionEvent {
    /// Create a new collision event.
    pub fn new(collision_type: CollisionType, point: Point2D, heading: f32) -> Self {
        Self {
            collision_type,
            point,
            heading,
        }
    }

    /// Create a bumper collision event from robot pose.
    ///
    /// # Arguments
    /// * `left` - Left bumper triggered
    /// * `right` - Right bumper triggered
    /// * `robot_x` - Robot X position
    /// * `robot_y` - Robot Y position
    /// * `robot_theta` - Robot heading
    /// * `robot_radius` - Robot collision radius
    pub fn from_bumper(
        left: bool,
        right: bool,
        robot_x: f32,
        robot_y: f32,
        robot_theta: f32,
        robot_radius: f32,
    ) -> Option<Self> {
        let collision_type = match (left, right) {
            (true, true) => CollisionType::BumperBoth,
            (true, false) => CollisionType::BumperLeft,
            (false, true) => CollisionType::BumperRight,
            (false, false) => return None,
        };

        // Compute collision point slightly ahead of robot
        let offset = robot_radius + 0.02; // Small offset past radius
        let point = Point2D::new(
            robot_x + offset * robot_theta.cos(),
            robot_y + offset * robot_theta.sin(),
        );

        Some(Self {
            collision_type,
            point,
            heading: robot_theta,
        })
    }

    /// Compute virtual wall to add at collision point.
    ///
    /// Creates a wall perpendicular to the robot's heading at the
    /// collision point. This prevents future path planning through
    /// areas that are physically blocked (e.g., glass/mirrors).
    ///
    /// # Arguments
    /// * `wall_length` - Length of the virtual wall to create
    pub fn to_virtual_wall(&self, wall_length: f32) -> VirtualWall {
        // Wall is perpendicular to heading
        let perp_angle = self.heading + FRAC_PI_2;
        let half_len = wall_length / 2.0;

        let start = Point2D::new(
            self.point.x + half_len * perp_angle.cos(),
            self.point.y + half_len * perp_angle.sin(),
        );
        let end = Point2D::new(
            self.point.x - half_len * perp_angle.cos(),
            self.point.y - half_len * perp_angle.sin(),
        );

        VirtualWall {
            line: Line2D::new(start, end),
            source: self.collision_type,
        }
    }

    /// Get the opposite direction for backoff.
    ///
    /// Returns the heading angle pointing away from the collision.
    pub fn backoff_heading(&self) -> f32 {
        // Back off in the opposite direction
        self.heading + std::f32::consts::PI
    }
}

/// Virtual wall to add to map after collision.
///
/// When the robot collides with an obstacle that wasn't detected by lidar
/// (e.g., glass, mirrors), a virtual wall is added to the map to prevent
/// future path planning through that area.
#[derive(Clone, Debug)]
pub struct VirtualWall {
    /// The line segment representing the wall.
    pub line: Line2D,
    /// The collision type that caused this wall to be created.
    pub source: CollisionType,
}

impl VirtualWall {
    /// Create a new virtual wall.
    pub fn new(line: Line2D, source: CollisionType) -> Self {
        Self { line, source }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_collision_type_checks() {
        assert!(CollisionType::BumperLeft.is_bumper());
        assert!(CollisionType::BumperRight.is_bumper());
        assert!(CollisionType::BumperBoth.is_bumper());
        assert!(!CollisionType::Cliff.is_bumper());

        assert!(CollisionType::Cliff.is_cliff());
        assert!(!CollisionType::BumperLeft.is_cliff());
    }

    #[test]
    fn test_from_bumper() {
        // No collision
        assert!(CollisionEvent::from_bumper(false, false, 0.0, 0.0, 0.0, 0.15).is_none());

        // Left bumper
        let event = CollisionEvent::from_bumper(true, false, 1.0, 2.0, 0.0, 0.15).unwrap();
        assert_eq!(event.collision_type, CollisionType::BumperLeft);
        assert!(event.point.x > 1.0); // Point is ahead of robot

        // Both bumpers
        let event = CollisionEvent::from_bumper(true, true, 0.0, 0.0, PI / 2.0, 0.15).unwrap();
        assert_eq!(event.collision_type, CollisionType::BumperBoth);
        assert!(event.point.y > 0.0); // Point is ahead when facing +Y
    }

    #[test]
    fn test_to_virtual_wall() {
        let event = CollisionEvent::new(CollisionType::BumperBoth, Point2D::new(1.0, 0.0), 0.0);

        let wall = event.to_virtual_wall(0.3);

        // Wall should be perpendicular to heading (vertical when heading is 0)
        assert_eq!(wall.source, CollisionType::BumperBoth);
        assert!((wall.line.start.x - 1.0).abs() < 0.01);
        assert!((wall.line.end.x - 1.0).abs() < 0.01);
        assert!((wall.line.start.y - 0.15).abs() < 0.01);
        assert!((wall.line.end.y + 0.15).abs() < 0.01);
    }

    #[test]
    fn test_backoff_heading() {
        let event = CollisionEvent::new(CollisionType::BumperBoth, Point2D::new(0.0, 0.0), 0.0);
        let backoff = event.backoff_heading();
        assert!((backoff - PI).abs() < 0.01);

        let event2 =
            CollisionEvent::new(CollisionType::BumperBoth, Point2D::new(0.0, 0.0), PI / 2.0);
        let backoff2 = event2.backoff_heading();
        assert!((backoff2 - 3.0 * PI / 2.0).abs() < 0.01);
    }
}
