//! Exploration strategy trait for extensible exploration algorithms.
//!
//! This module defines the `ExplorationStrategy` trait that all exploration
//! algorithms must implement. This allows easy swapping between different
//! exploration strategies.
//!
//! Note: Some types are defined for planned exploration features.

use crate::core::types::Pose2D;
use crate::state::CurrentMapData;

/// Type of hazard detected by contact sensors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HazardType {
    /// Bumper collision - solid obstacle in front.
    BumperLeft,
    BumperRight,
    BumperBoth,
    /// Cliff sensor - drop-off hazard.
    CliffLeftSide,
    CliffLeftFront,
    CliffRightFront,
    CliffRightSide,
}

impl HazardType {
    /// Get the relative angle offset from robot heading where the hazard was detected.
    /// Returns (angle_offset_rad, distance_from_center_m).
    /// Positive angle = left of center, negative = right of center.
    pub fn sensor_offset(&self) -> (f32, f32) {
        // Robot dimensions (approximate for CRL-200S):
        // - Robot radius: ~0.17m
        // - Bumper arc: ~120° front
        // - Cliff sensors at corners
        match self {
            HazardType::BumperLeft => (0.3, 0.17), // ~17° left, at front edge
            HazardType::BumperRight => (-0.3, 0.17), // ~17° right, at front edge
            HazardType::BumperBoth => (0.0, 0.17), // Dead center front
            HazardType::CliffLeftSide => (1.57, 0.15), // 90° left (side)
            HazardType::CliffLeftFront => (0.52, 0.16), // ~30° left front
            HazardType::CliffRightFront => (-0.52, 0.16), // ~30° right front
            HazardType::CliffRightSide => (-1.57, 0.15), // 90° right (side)
        }
    }

    /// Whether this is a cliff hazard (vs bumper).
    pub fn is_cliff(&self) -> bool {
        matches!(
            self,
            HazardType::CliffLeftSide
                | HazardType::CliffLeftFront
                | HazardType::CliffRightFront
                | HazardType::CliffRightSide
        )
    }
}

/// A hazard event detected by contact sensors.
#[derive(Debug, Clone)]
pub struct HazardEvent {
    /// Type of hazard detected.
    pub hazard_type: HazardType,
    /// World position where hazard was detected.
    pub world_position: Pose2D,
}

impl HazardEvent {
    /// Create a new hazard event, computing world position from robot pose and sensor offset.
    pub fn new(hazard_type: HazardType, robot_pose: &Pose2D) -> Self {
        let (angle_offset, distance) = hazard_type.sensor_offset();
        let hazard_angle = robot_pose.theta + angle_offset;
        let world_position = Pose2D::new(
            robot_pose.x + distance * hazard_angle.cos(),
            robot_pose.y + distance * hazard_angle.sin(),
            hazard_angle,
        );
        Self {
            hazard_type,
            world_position,
        }
    }
}

/// Result of exploration planning.
#[derive(Debug, Clone)]
pub enum ExplorationAction {
    /// Move toward target pose.
    MoveTo {
        /// Target pose to navigate to.
        target: Pose2D,
        /// Human-readable reason for this target.
        reason: &'static str,
    },
    /// Rotate in place to scan surroundings.
    RotateInPlace {
        /// Angle to rotate (radians, positive = CCW).
        angle_rad: f32,
    },
    /// Exploration complete - all reachable areas mapped.
    Complete,
    /// Cannot proceed - stuck or unreachable.
    Stuck {
        /// Reason for being stuck.
        reason: String,
    },
}

/// Trait for exploration strategies.
///
/// Implement this trait to create custom exploration algorithms.
/// The exploration thread will call `next_action` periodically to
/// determine what the robot should do next.
pub trait ExplorationStrategy: Send + Sync {
    /// Name of this strategy for logging.
    fn name(&self) -> &'static str;

    /// Compute next action given current state.
    ///
    /// # Arguments
    /// * `map` - Current occupancy grid map data
    /// * `current_pose` - Current robot pose in map frame
    /// * `hazard` - Optional hazard event if bumper/cliff triggered since last call
    ///
    /// # Returns
    /// The next action the robot should take.
    fn next_action(
        &mut self,
        map: &CurrentMapData,
        current_pose: &Pose2D,
        hazard: Option<&HazardEvent>,
    ) -> ExplorationAction;

    /// Reset strategy state (e.g., when starting new map).
    fn reset(&mut self);

    /// Get the number of frontiers currently being tracked.
    fn frontier_count(&self) -> usize {
        0
    }

    /// Get estimated completion percentage (0-100).
    fn completion_percent(&self) -> f32 {
        0.0
    }
}
