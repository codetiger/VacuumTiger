//! Motion and odometry types

/// Robot velocity (linear and angular)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Velocity {
    /// Linear velocity in m/s
    pub linear: f32,
    /// Angular velocity in rad/s
    pub angular: f32,
}

impl Velocity {
    /// Create new velocity
    pub fn new(linear: f32, angular: f32) -> Self {
        Self { linear, angular }
    }

    /// Zero velocity
    pub fn zero() -> Self {
        Self {
            linear: 0.0,
            angular: 0.0,
        }
    }
}

/// Odometry data (position and velocity)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Odometry {
    /// X position in meters
    pub x: f32,
    /// Y position in meters
    pub y: f32,
    /// Heading angle in radians
    pub theta: f32,
    /// Current velocity
    pub velocity: Velocity,
    /// Left wheel encoder count
    pub encoder_left: i32,
    /// Right wheel encoder count
    pub encoder_right: i32,
}

impl Odometry {
    /// Create zero odometry
    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            velocity: Velocity::zero(),
            encoder_left: 0,
            encoder_right: 0,
        }
    }
}

/// 2D pose (position and orientation)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2D {
    /// X position in meters
    pub x: f32,
    /// Y position in meters
    pub y: f32,
    /// Heading angle in radians
    pub theta: f32,
}
