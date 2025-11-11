//! Motion controller for executing movement commands

use super::commands::{MotionCommand, MotionCommandStatus};
use super::constraints::MotionConstraints;
use crate::config::SangamConfig;
use crate::odometry::OdometryDelta;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};

/// Maximum motor speed value for GD32 controller (range: -2000 to +2000)
const MOTOR_SPEED_MAX: f32 = 2000.0;

/// 2D velocity representation
#[derive(Debug, Clone, Copy, Default)]
pub struct Velocity2D {
    /// Linear velocity (m/s)
    pub linear: f32,

    /// Angular velocity (rad/s)
    pub angular: f32,
}

/// Motion controller state
pub struct MotionController {
    /// Current actual velocity
    pub current_velocity: Velocity2D,

    /// Target velocity
    pub target_velocity: Velocity2D,

    /// Motion constraints
    pub constraints: MotionConstraints,

    /// Active command
    pub active_command: Option<MotionCommand>,

    /// Command start time
    pub command_start: Option<Instant>,

    /// Command progress tracking
    pub command_progress: CommandProgress,

    /// Configuration
    config: SangamConfig,

    /// Last progress log time (for throttling)
    last_progress_log: Option<Instant>,

    /// Last velocity log time (for throttling)
    last_velocity_log: Option<Instant>,
}

/// Tracks progress for position-based commands
#[derive(Debug, Clone, Default)]
pub struct CommandProgress {
    /// Initial distance/angle when command started (reserved for future use)
    #[allow(dead_code)]
    pub initial_value: f32,

    /// Target distance/angle to achieve
    pub target_value: f32,

    /// Current accumulated progress
    pub current_value: f32,

    /// Is command complete
    pub is_complete: bool,
}

/// Motion status information
#[derive(Debug, Clone)]
pub struct MotionStatus {
    /// Currently in motion
    pub is_moving: bool,

    /// Current velocity
    pub current_velocity: Velocity2D,

    /// Target velocity
    pub target_velocity: Velocity2D,

    /// Active command status
    pub active_command: Option<MotionCommandStatus>,

    /// Last odometry reading
    pub odometry_delta: Option<OdometryDelta>,
}

impl MotionController {
    /// Create new motion controller
    pub fn new(config: SangamConfig) -> Self {
        let constraints = MotionConstraints::from_config(&config);

        log::debug!(
            "MotionController: Initialized with constraints: max_linear={:.2}m/s, max_angular={:.2}rad/s",
            config.max_linear_velocity,
            config.max_angular_velocity
        );

        Self {
            current_velocity: Velocity2D::default(),
            target_velocity: Velocity2D::default(),
            constraints,
            active_command: None,
            command_start: None,
            command_progress: CommandProgress::default(),
            config,
            last_progress_log: None,
            last_velocity_log: None,
        }
    }

    /// Execute a motion command
    pub fn execute_command(&mut self, command: MotionCommand) {
        self.active_command = Some(command.clone());
        self.command_start = Some(Instant::now());
        self.command_progress = CommandProgress::default();
        self.last_progress_log = None;

        // Set target velocity based on command
        match command {
            MotionCommand::Velocity { linear, angular } => {
                let (linear, angular) = self.constraints.constrain_velocity(linear, angular);
                self.target_velocity = Velocity2D { linear, angular };

                log::info!(
                    "MotionController: Velocity command - linear={:.3}m/s, angular={:.3}rad/s",
                    linear,
                    angular
                );
                log::debug!(
                    "MotionController: Target velocity set to ({:.3}, {:.3})",
                    self.target_velocity.linear,
                    self.target_velocity.angular
                );
            }
            MotionCommand::MoveDistance {
                distance,
                max_velocity,
            } => {
                self.command_progress.target_value = distance.abs();
                let velocity = max_velocity.min(self.constraints.max_linear_velocity);
                let direction = distance.signum();
                self.target_velocity = Velocity2D {
                    linear: direction * velocity,
                    angular: 0.0,
                };

                let direction_str = if distance >= 0.0 {
                    "forward"
                } else {
                    "backward"
                };
                log::info!(
                    "MotionController: Move {} {:.3}m at max {:.3}m/s",
                    direction_str,
                    distance.abs(),
                    velocity
                );
                log::debug!(
                    "MotionController: Target velocity={:.3}m/s",
                    self.target_velocity.linear
                );
            }
            MotionCommand::Rotate {
                angle,
                max_velocity,
            } => {
                self.command_progress.target_value = angle.abs();
                let velocity = max_velocity.min(self.constraints.max_angular_velocity);
                let direction = angle.signum();
                self.target_velocity = Velocity2D {
                    linear: 0.0,
                    angular: direction * velocity,
                };

                let direction_str = if angle >= 0.0 { "CCW" } else { "CW" };
                log::info!(
                    "MotionController: Rotate {:.3}rad ({:.1}°) {} at max {:.3}rad/s",
                    angle.abs(),
                    angle.abs().to_degrees(),
                    direction_str,
                    velocity
                );
                log::debug!(
                    "MotionController: Target angular velocity={:.3}rad/s",
                    self.target_velocity.angular
                );
            }
            MotionCommand::Stop | MotionCommand::EmergencyStop => {
                self.target_velocity = Velocity2D::default();

                let emergency = matches!(command, MotionCommand::EmergencyStop);
                log::info!(
                    "MotionController: {} command executed",
                    if emergency { "Emergency stop" } else { "Stop" }
                );
            }
        }
    }

    /// Update motion controller (called at control frequency)
    /// Returns (left_wheel_speed, right_wheel_speed) in raw motor units
    pub fn update(&mut self, dt: f32, odometry: Option<&OdometryDelta>) -> (i32, i32) {
        // Update progress tracking for position-based commands
        if let Some(ref command) = self.active_command
            && command.is_position_based()
            && let Some(odom) = odometry
        {
            match command {
                MotionCommand::MoveDistance { .. } => {
                    self.command_progress.current_value += odom.distance.abs();
                }
                MotionCommand::Rotate { .. } => {
                    self.command_progress.current_value += odom.angle.abs();
                }
                MotionCommand::Velocity { .. }
                | MotionCommand::Stop
                | MotionCommand::EmergencyStop => {}
            }

            // Log progress periodically (every second or 10%)
            let progress_percent = if self.command_progress.target_value > 0.0 {
                (self.command_progress.current_value / self.command_progress.target_value * 100.0)
                    .min(100.0)
            } else {
                0.0
            };

            let should_log = if let Some(last_log) = self.last_progress_log {
                last_log.elapsed() >= Duration::from_secs(1)
            } else {
                true
            };

            if should_log && !self.command_progress.is_complete {
                log::debug!(
                    "MotionController: Command progress: {:.1}% ({:.3}/{:.3})",
                    progress_percent,
                    self.command_progress.current_value,
                    self.command_progress.target_value
                );
                self.last_progress_log = Some(Instant::now());
            }

            // Check if command is complete
            if self.command_progress.current_value >= self.command_progress.target_value {
                self.command_progress.is_complete = true;
                self.target_velocity = Velocity2D::default();

                // Log completion
                if let Some(start_time) = self.command_start {
                    let elapsed = start_time.elapsed();
                    match command {
                        MotionCommand::MoveDistance { distance, .. } => {
                            log::info!(
                                "MotionController: Move complete - traveled {:.3}m in {:.2}s",
                                distance.abs(),
                                elapsed.as_secs_f32()
                            );
                        }
                        MotionCommand::Rotate { angle, .. } => {
                            log::info!(
                                "MotionController: Rotation complete - rotated {:.3}rad ({:.1}°) in {:.2}s",
                                angle.abs(),
                                angle.abs().to_degrees(),
                                elapsed.as_secs_f32()
                            );
                        }
                        _ => {}
                    }
                }

                self.active_command = None;
            }
        }

        // Apply acceleration constraints
        let emergency = matches!(self.active_command, Some(MotionCommand::EmergencyStop));

        let (new_linear, new_angular) = self.constraints.apply_acceleration(
            self.current_velocity.linear,
            self.current_velocity.angular,
            self.target_velocity.linear,
            self.target_velocity.angular,
            dt,
            emergency,
        );

        self.current_velocity = Velocity2D {
            linear: new_linear,
            angular: new_angular,
        };

        // Log velocity changes periodically (throttled to 1Hz)
        let should_log_velocity = if let Some(last_log) = self.last_velocity_log {
            last_log.elapsed() >= Duration::from_secs(1)
        } else {
            false
        };

        if should_log_velocity
            && (self.current_velocity.linear.abs() > 0.01
                || self.current_velocity.angular.abs() > 0.01)
        {
            log::debug!(
                "MotionController: Velocity update - current=({:.3}, {:.3}), target=({:.3}, {:.3})",
                self.current_velocity.linear,
                self.current_velocity.angular,
                self.target_velocity.linear,
                self.target_velocity.angular
            );
            self.last_velocity_log = Some(Instant::now());
        }

        // Convert to differential drive
        let (left_speed, right_speed) = self
            .config
            .differential_drive_kinematics(new_linear, new_angular);

        // Scale to motor units (-2000 to 2000 range for GD32)
        // This assumes max_linear_velocity corresponds to motor value MOTOR_SPEED_MAX
        let scale = MOTOR_SPEED_MAX / self.config.max_linear_velocity;
        let left_motor = (left_speed * scale) as i32;
        let right_motor = (right_speed * scale) as i32;

        // Debug: Log motor calculations (first 20 iterations only to avoid spam)
        static DEBUG_COUNT: AtomicU32 = AtomicU32::new(0);
        let count = DEBUG_COUNT.fetch_add(1, Ordering::Relaxed);
        if count < 20 {
            log::debug!(
                "MotionController: Motor calc - new_vel=({:.3}, {:.3}), wheel_speeds=({:.3}, {:.3}), scale={:.1}, motors=({}, {})",
                new_linear, new_angular, left_speed, right_speed, scale, left_motor, right_motor
            );
        }

        (left_motor, right_motor)
    }

    /// Stop all motion
    pub fn stop(&mut self, emergency: bool) {
        let command = if emergency {
            MotionCommand::EmergencyStop
        } else {
            MotionCommand::Stop
        };
        self.execute_command(command);
    }

    /// Check if currently moving
    pub fn is_moving(&self) -> bool {
        self.current_velocity.linear.abs() > 0.01 || self.current_velocity.angular.abs() > 0.01
    }

    /// Get motion status
    pub fn status(&self) -> MotionStatus {
        let active_command = self.active_command.as_ref().and_then(|cmd| {
            self.command_start.map(|start| {
                let elapsed = start.elapsed();
                let progress = if self.command_progress.target_value > 0.0 {
                    self.command_progress.current_value / self.command_progress.target_value
                } else {
                    0.0
                };

                MotionCommandStatus {
                    command_type: cmd.command_type(),
                    progress: progress.min(1.0),
                    elapsed_time: elapsed,
                    start_time: start,
                    time_remaining: None, // TODO: Estimate based on velocity
                }
            })
        });

        MotionStatus {
            is_moving: self.is_moving(),
            current_velocity: self.current_velocity,
            target_velocity: self.target_velocity,
            active_command,
            odometry_delta: None,
        }
    }

    /// Reset controller state (reserved for future use)
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        log::debug!("MotionController: Reset controller state");

        self.current_velocity = Velocity2D::default();
        self.target_velocity = Velocity2D::default();
        self.active_command = None;
        self.command_start = None;
        self.command_progress = CommandProgress::default();
        self.last_progress_log = None;
        self.last_velocity_log = None;
    }
}

impl Velocity2D {
    /// Create new velocity
    pub fn new(linear: f32, angular: f32) -> Self {
        Self { linear, angular }
    }

    /// Check if velocity is near zero
    pub fn is_zero(&self) -> bool {
        self.linear.abs() < 0.001 && self.angular.abs() < 0.001
    }
}
