//! Navigation target types.
//!
//! A [`NavTarget`] is the universal unit of navigation. All features
//! (mapping, sweeping, docking, user go-to) create NavTargets and push
//! them onto the target stack.
//!
//! Note: Some utility methods are defined for future use.

use crate::core::types::Point2D;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU32, Ordering};

/// Global counter for generating unique target IDs.
static TARGET_ID_COUNTER: AtomicU32 = AtomicU32::new(1);

/// Generate a unique target ID.
pub fn generate_target_id() -> u32 {
    TARGET_ID_COUNTER.fetch_add(1, Ordering::Relaxed)
}

/// Movement direction for navigation.
///
/// Most navigation uses forward movement, but docking requires
/// the robot to reverse onto the charging contacts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum MovementDirection {
    /// Robot moves forward toward target (normal operation).
    #[default]
    Forward,

    /// Robot moves backward toward target (e.g., docking onto charger).
    Backward,
}

impl MovementDirection {
    /// Get the velocity sign for this direction.
    ///
    /// Returns 1.0 for forward, -1.0 for backward.
    #[inline]
    pub fn velocity_sign(&self) -> f32 {
        match self {
            MovementDirection::Forward => 1.0,
            MovementDirection::Backward => -1.0,
        }
    }
}

/// Type of navigation target.
///
/// Different target types have different default tolerances and behaviors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavTargetType {
    /// Navigate to exact position and heading (user click).
    /// Tight tolerances, explicit heading usually provided.
    Waypoint,

    /// Navigate to frontier for exploration (mapping).
    /// Looser tolerances, heading often auto-chosen.
    Frontier,

    /// Navigate for coverage cleaning (sweeping pattern).
    /// Moderate tolerances, heading specified for coverage direction.
    Coverage,

    /// Final docking approach (precise alignment required).
    /// Very tight tolerances, backward movement.
    DockApproach,

    /// Intermediate point (less strict tolerances).
    /// Used for approach waypoints before critical targets.
    Intermediate,
}

impl NavTargetType {
    /// Get default position tolerance for this target type (meters).
    pub fn default_position_tolerance(&self) -> f32 {
        match self {
            NavTargetType::Waypoint => 0.08,
            NavTargetType::Frontier => 0.30,
            NavTargetType::Coverage => 0.15,
            NavTargetType::DockApproach => 0.03,
            NavTargetType::Intermediate => 0.15,
        }
    }

    /// Get default heading tolerance for this target type (radians).
    pub fn default_heading_tolerance(&self) -> f32 {
        match self {
            NavTargetType::Waypoint => 0.15,     // ~8.6°
            NavTargetType::Frontier => 0.50,     // ~28.6°
            NavTargetType::Coverage => 0.20,     // ~11.5°
            NavTargetType::DockApproach => 0.08, // ~4.6°
            NavTargetType::Intermediate => 0.30, // ~17.2°
        }
    }
}

/// Source feature that created a navigation target.
///
/// Used to track which operation is generating targets and
/// for UI display purposes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavTargetSource {
    /// User clicked on map (SetGoalCommand).
    UserClick,

    /// Autonomous mapping exploration.
    Mapping,

    /// Sweeping/cleaning operation.
    Sweeping,

    /// Return to dock operation.
    Docking,

    /// Zone cleaning operation.
    ZoneCleaning,

    /// Room cleaning operation.
    RoomCleaning,
}

impl NavTargetSource {
    /// Get a human-readable name for this source.
    pub fn display_name(&self) -> &'static str {
        match self {
            NavTargetSource::UserClick => "Go To",
            NavTargetSource::Mapping => "Mapping",
            NavTargetSource::Sweeping => "Sweeping",
            NavTargetSource::Docking => "Docking",
            NavTargetSource::ZoneCleaning => "Zone Clean",
            NavTargetSource::RoomCleaning => "Room Clean",
        }
    }
}

/// A navigation target - the universal unit of navigation.
///
/// All features (mapping, sweeping, docking, user goto) create NavTargets
/// and push them onto the target stack. The navigation system then plans
/// paths to each target and executes them in sequence.
///
/// # Heading Behavior
///
/// - If `heading` is `Some(theta)`, the robot will rotate to face that
///   heading after reaching the position.
/// - If `heading` is `None`, the planner chooses the heading based on:
///   1. Direction to the next target in the stack (if any)
///   2. Direction of approach (if no next target)
///
/// # Movement Direction
///
/// Most targets use `Forward` movement. The `Backward` direction is used
/// for docking, where the robot must reverse onto the charging contacts.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavTarget {
    /// Unique identifier for this target.
    pub id: u32,

    /// Target position in world frame (meters).
    pub position: Point2D,

    /// Final heading at target (radians), None = planner chooses.
    ///
    /// If None, the planner picks heading based on:
    /// 1. Direction to next target (if any)
    /// 2. Direction of approach (if no next target)
    pub heading: Option<f32>,

    /// Movement direction: Forward (default) or Backward.
    ///
    /// Backward is used for docking where robot reverses onto charger.
    pub movement_direction: MovementDirection,

    /// Type of navigation target (affects default tolerances).
    pub target_type: NavTargetType,

    /// Human-readable description for UI.
    pub description: String,

    /// Position tolerance (meters).
    ///
    /// Target is considered reached when distance to position
    /// is less than this value.
    pub position_tolerance: f32,

    /// Heading tolerance (radians).
    ///
    /// Only used if `heading` is `Some`. Target heading is considered
    /// reached when angular error is less than this value.
    pub heading_tolerance: f32,

    /// Source feature that created this target.
    pub source: NavTargetSource,
}

impl NavTarget {
    /// Create a new navigation target with specified parameters.
    pub fn new(
        position: Point2D,
        heading: Option<f32>,
        target_type: NavTargetType,
        source: NavTargetSource,
        description: impl Into<String>,
    ) -> Self {
        Self {
            id: generate_target_id(),
            position,
            heading,
            movement_direction: MovementDirection::Forward,
            target_type,
            description: description.into(),
            position_tolerance: target_type.default_position_tolerance(),
            heading_tolerance: target_type.default_heading_tolerance(),
            source,
        }
    }

    /// Create a user waypoint target (from SetGoalCommand).
    pub fn user_waypoint(x: f32, y: f32, heading: Option<f32>, description: String) -> Self {
        Self::new(
            Point2D::new(x, y),
            heading,
            NavTargetType::Waypoint,
            NavTargetSource::UserClick,
            if description.is_empty() {
                format!("Go to ({:.1}, {:.1})", x, y)
            } else {
                description
            },
        )
    }

    /// Create a frontier target for mapping exploration.
    pub fn frontier(x: f32, y: f32) -> Self {
        Self::new(
            Point2D::new(x, y),
            None, // Planner chooses heading
            NavTargetType::Frontier,
            NavTargetSource::Mapping,
            format!("Frontier ({:.1}, {:.1})", x, y),
        )
    }

    /// Create a docking target with backward movement.
    pub fn dock_target(x: f32, y: f32, heading: f32) -> Self {
        let mut target = Self::new(
            Point2D::new(x, y),
            Some(heading),
            NavTargetType::DockApproach,
            NavTargetSource::Docking,
            "Dock (reversing)",
        );
        target.movement_direction = MovementDirection::Backward;
        target
    }

    /// Set custom position tolerance.
    pub fn with_position_tolerance(mut self, tolerance: f32) -> Self {
        self.position_tolerance = tolerance;
        self
    }

    /// Set custom heading tolerance.
    pub fn with_heading_tolerance(mut self, tolerance: f32) -> Self {
        self.heading_tolerance = tolerance;
        self
    }

    /// Set movement direction.
    pub fn with_movement_direction(mut self, direction: MovementDirection) -> Self {
        self.movement_direction = direction;
        self
    }

    /// Check if position is reached given current robot position.
    pub fn is_position_reached(&self, robot_x: f32, robot_y: f32) -> bool {
        let dx = self.position.x - robot_x;
        let dy = self.position.y - robot_y;
        let distance = (dx * dx + dy * dy).sqrt();
        distance < self.position_tolerance
    }

    /// Check if heading is reached given current robot heading.
    ///
    /// Returns `true` if:
    /// - No heading is specified (heading is None), OR
    /// - The angular error is within tolerance
    pub fn is_heading_reached(&self, robot_theta: f32) -> bool {
        match self.heading {
            None => true, // No heading requirement
            Some(target_theta) => {
                let error = crate::core::math::normalize_angle(target_theta - robot_theta);
                error.abs() < self.heading_tolerance
            }
        }
    }

    /// Check if target is fully reached (position and heading).
    pub fn is_reached(&self, robot_x: f32, robot_y: f32, robot_theta: f32) -> bool {
        self.is_position_reached(robot_x, robot_y) && self.is_heading_reached(robot_theta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_generate_unique_ids() {
        let id1 = generate_target_id();
        let id2 = generate_target_id();
        let id3 = generate_target_id();

        assert_ne!(id1, id2);
        assert_ne!(id2, id3);
        assert_ne!(id1, id3);
    }

    #[test]
    fn test_movement_direction_velocity_sign() {
        assert_eq!(MovementDirection::Forward.velocity_sign(), 1.0);
        assert_eq!(MovementDirection::Backward.velocity_sign(), -1.0);
    }

    #[test]
    fn test_user_waypoint_creation() {
        let target = NavTarget::user_waypoint(1.0, 2.0, Some(0.5), "Test".to_string());

        assert_eq!(target.position.x, 1.0);
        assert_eq!(target.position.y, 2.0);
        assert_eq!(target.heading, Some(0.5));
        assert_eq!(target.target_type, NavTargetType::Waypoint);
        assert_eq!(target.source, NavTargetSource::UserClick);
        assert_eq!(target.movement_direction, MovementDirection::Forward);
    }

    #[test]
    fn test_frontier_creation() {
        let target = NavTarget::frontier(3.0, 4.0);

        assert_eq!(target.position.x, 3.0);
        assert_eq!(target.position.y, 4.0);
        assert_eq!(target.heading, None);
        assert_eq!(target.target_type, NavTargetType::Frontier);
        assert_eq!(target.source, NavTargetSource::Mapping);
    }

    #[test]
    fn test_dock_target_creation() {
        let target = NavTarget::dock_target(0.0, 0.0, PI);

        assert_eq!(target.target_type, NavTargetType::DockApproach);
        assert_eq!(target.movement_direction, MovementDirection::Backward);
        assert_eq!(target.heading, Some(PI));
    }

    #[test]
    fn test_position_reached() {
        let target =
            NavTarget::user_waypoint(1.0, 1.0, None, String::new()).with_position_tolerance(0.1);

        // Within tolerance
        assert!(target.is_position_reached(1.05, 1.05));

        // Outside tolerance
        assert!(!target.is_position_reached(1.2, 1.2));
    }

    #[test]
    fn test_heading_reached_no_heading() {
        let target = NavTarget::frontier(0.0, 0.0);

        // No heading specified, should always return true
        assert!(target.is_heading_reached(0.0));
        assert!(target.is_heading_reached(PI));
        assert!(target.is_heading_reached(-PI));
    }

    #[test]
    fn test_heading_reached_with_heading() {
        let target = NavTarget::user_waypoint(0.0, 0.0, Some(0.0), String::new())
            .with_heading_tolerance(0.2);

        // Within tolerance
        assert!(target.is_heading_reached(0.1));
        assert!(target.is_heading_reached(-0.1));

        // Outside tolerance
        assert!(!target.is_heading_reached(0.5));
    }

    #[test]
    fn test_is_reached_full() {
        let target = NavTarget::user_waypoint(1.0, 1.0, Some(0.0), String::new())
            .with_position_tolerance(0.1)
            .with_heading_tolerance(0.2);

        // Both position and heading within tolerance
        assert!(target.is_reached(1.05, 1.05, 0.1));

        // Position OK, heading not OK
        assert!(!target.is_reached(1.05, 1.05, 0.5));

        // Heading OK, position not OK
        assert!(!target.is_reached(2.0, 2.0, 0.1));
    }

    #[test]
    fn test_default_tolerances() {
        assert!(NavTargetType::Waypoint.default_position_tolerance() < 0.1);
        assert!(NavTargetType::DockApproach.default_position_tolerance() < 0.05);
        assert!(NavTargetType::Frontier.default_position_tolerance() > 0.2);
    }
}
