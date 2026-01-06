//! Shared state for multi-threaded architecture.
//!
//! Provides thread-safe shared state between:
//! - Sensor thread (UDP reading, odometry, safety, commands)
//! - Mapping thread (scan matching, map updates)
//! - Exploration thread (frontier detection, path planning)

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, RwLock};

use vastu_slam::{GridStorage, Pose2D};

use crate::utils::normalize_angle;

/// Atomic wrapper for f32 values.
/// Uses AtomicU32 with bit reinterpretation.
#[derive(Debug)]
pub struct AtomicF32(AtomicU32);

impl AtomicF32 {
    pub fn new(val: f32) -> Self {
        Self(AtomicU32::new(val.to_bits()))
    }

    pub fn load(&self, order: Ordering) -> f32 {
        f32::from_bits(self.0.load(order))
    }

    pub fn store(&self, val: f32, order: Ordering) {
        self.0.store(val.to_bits(), order);
    }
}

/// Atomic wrapper for Pose2D.
/// Packs x, y, theta into atomic u64 values for lock-free access.
/// Uses fixed-point representation: 16-bit integer + 16-bit fraction per component.
#[derive(Debug)]
pub struct AtomicPose {
    // Pack x and y into one u64 (each as i32 in mm for precision)
    xy: AtomicU64,
    // Theta as fixed-point (multiplied by 10000)
    theta: AtomicU32,
}

impl AtomicPose {
    pub fn new(pose: Pose2D) -> Self {
        let x_mm = (pose.x * 1000.0) as i32;
        let y_mm = (pose.y * 1000.0) as i32;
        let xy = ((x_mm as u64) << 32) | (y_mm as u32 as u64);
        let theta = (pose.theta * 10000.0) as i32 as u32;

        Self {
            xy: AtomicU64::new(xy),
            theta: AtomicU32::new(theta),
        }
    }

    pub fn load(&self, order: Ordering) -> Pose2D {
        let xy = self.xy.load(order);
        let x_mm = (xy >> 32) as i32;
        let y_mm = xy as i32;
        let theta_fp = self.theta.load(order) as i32;

        Pose2D::new(
            x_mm as f32 / 1000.0,
            y_mm as f32 / 1000.0,
            theta_fp as f32 / 10000.0,
        )
    }

    pub fn store(&self, pose: Pose2D, order: Ordering) {
        let x_mm = (pose.x * 1000.0) as i32;
        let y_mm = (pose.y * 1000.0) as i32;
        let xy = ((x_mm as u64) << 32) | (y_mm as u32 as u64);
        let theta = (pose.theta * 10000.0) as i32 as u32;

        self.xy.store(xy, order);
        self.theta.store(theta, order);
    }
}

/// Shared state between all threads.
#[derive(Debug)]
pub struct SharedState {
    /// Last corrected pose from scan matching (updated by mapping thread)
    corrected_pose: AtomicPose,

    /// Raw odometry pose at the time of last correction
    odometry_at_correction: AtomicPose,

    /// Current raw odometry pose (updated by sensor thread)
    current_odometry: AtomicPose,

    /// Velocity command: linear (m/s)
    pub linear_vel: AtomicF32,

    /// Velocity command: angular (rad/s)
    pub angular_vel: AtomicF32,

    /// Safety stop flag
    pub safety_stop: AtomicBool,

    /// Safety stop reason (if any)
    pub safety_reason: RwLock<Option<String>>,

    /// Exploration complete flag
    pub exploration_complete: AtomicBool,

    /// Shutdown signal for graceful termination
    pub shutdown: AtomicBool,

    /// Number of scans processed (for status reporting)
    pub scan_count: AtomicU32,

    /// Number of frontiers remaining
    pub frontiers_remaining: AtomicU32,
}

impl SharedState {
    /// Create new shared state with initial pose.
    pub fn new(initial_pose: Pose2D) -> Self {
        Self {
            corrected_pose: AtomicPose::new(initial_pose),
            odometry_at_correction: AtomicPose::new(initial_pose),
            current_odometry: AtomicPose::new(initial_pose),
            linear_vel: AtomicF32::new(0.0),
            angular_vel: AtomicF32::new(0.0),
            safety_stop: AtomicBool::new(false),
            safety_reason: RwLock::new(None),
            exploration_complete: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
            scan_count: AtomicU32::new(0),
            frontiers_remaining: AtomicU32::new(0),
        }
    }

    /// Get current pose (corrected pose + odometry delta since last correction).
    ///
    /// This provides a smooth, frequently-updated pose that incorporates:
    /// - Scan matching corrections from the mapping thread (5Hz)
    /// - Odometry updates from the sensor thread (110Hz)
    pub fn pose(&self) -> Pose2D {
        let corrected = self.corrected_pose.load(Ordering::Acquire);
        let odom_at_correction = self.odometry_at_correction.load(Ordering::Acquire);
        let current_odom = self.current_odometry.load(Ordering::Acquire);

        // Compute odometry delta since last correction
        let dx = current_odom.x - odom_at_correction.x;
        let dy = current_odom.y - odom_at_correction.y;
        // Normalize dtheta to handle wraparound (e.g., -179° to +179° = 2°, not 358°)
        let dtheta = normalize_angle(current_odom.theta - odom_at_correction.theta);

        // Apply delta to corrected pose (normalize final theta to [-π, π])
        Pose2D::new(
            corrected.x + dx,
            corrected.y + dy,
            normalize_angle(corrected.theta + dtheta),
        )
    }

    /// Update raw odometry pose (called by sensor thread at 110Hz).
    pub fn set_odometry(&self, pose: Pose2D) {
        self.current_odometry.store(pose, Ordering::Release);
    }

    /// Apply scan matching correction (called by mapping thread at 5Hz).
    ///
    /// This updates the corrected pose and records the odometry at correction time,
    /// so future pose() calls can apply the correct delta.
    pub fn apply_correction(&self, corrected: Pose2D, odometry_at_scan: Pose2D) {
        self.corrected_pose.store(corrected, Ordering::Release);
        self.odometry_at_correction
            .store(odometry_at_scan, Ordering::Release);
    }

    /// Get velocity command (linear, angular).
    pub fn velocity(&self) -> (f32, f32) {
        (
            self.linear_vel.load(Ordering::Acquire),
            self.angular_vel.load(Ordering::Acquire),
        )
    }

    /// Set velocity command.
    pub fn set_velocity(&self, linear: f32, angular: f32) {
        self.linear_vel.store(linear, Ordering::Release);
        self.angular_vel.store(angular, Ordering::Release);
    }

    /// Trigger safety stop with reason.
    pub fn trigger_safety_stop(&self, reason: String) {
        if let Ok(mut guard) = self.safety_reason.write() {
            *guard = Some(reason);
        }
        self.safety_stop.store(true, Ordering::Release);
    }

    /// Check if safety stop is triggered.
    pub fn is_safety_stop(&self) -> bool {
        self.safety_stop.load(Ordering::Acquire)
    }

    /// Get safety stop reason.
    pub fn safety_reason(&self) -> Option<String> {
        self.safety_reason.read().ok().and_then(|g| g.clone())
    }

    /// Signal shutdown.
    pub fn signal_shutdown(&self) {
        self.shutdown.store(true, Ordering::Release);
    }

    /// Check if shutdown is signaled.
    pub fn should_shutdown(&self) -> bool {
        self.shutdown.load(Ordering::Acquire)
    }

    /// Mark exploration as complete.
    pub fn set_exploration_complete(&self) {
        self.exploration_complete.store(true, Ordering::Release);
    }

    /// Check if exploration is complete.
    pub fn is_exploration_complete(&self) -> bool {
        self.exploration_complete.load(Ordering::Acquire)
    }

    /// Increment scan count.
    pub fn increment_scan_count(&self) {
        self.scan_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Get scan count.
    pub fn scan_count(&self) -> u32 {
        self.scan_count.load(Ordering::Relaxed)
    }

    /// Set frontiers remaining.
    pub fn set_frontiers_remaining(&self, count: usize) {
        self.frontiers_remaining
            .store(count as u32, Ordering::Relaxed);
    }
}

/// Thread-safe map storage wrapper.
pub type SharedGrid = Arc<RwLock<GridStorage>>;

/// Thread-safe trajectory storage for SVG visualization.
pub type SharedTrajectory = Arc<RwLock<Vec<Pose2D>>>;

/// Debug visualization data for SVG output.
#[derive(Clone, Debug, Default)]
pub struct DebugVisualization {
    /// Frontier centroids that were visited
    pub visited_frontiers: Vec<vastu_slam::WorldPoint>,
    /// Waypoints from planned paths
    pub waypoints: Vec<vastu_slam::WorldPoint>,
}

/// Thread-safe debug visualization data.
pub type SharedDebugViz = Arc<RwLock<DebugVisualization>>;

/// Message types for inter-thread communication.
pub mod messages {
    use super::*;

    /// Lidar scan data sent from sensor thread to mapping thread.
    #[derive(Clone)]
    pub struct LidarScanMsg {
        /// Lidar points: (angle_rad, distance_m, quality)
        pub points: Vec<(f32, f32, u8)>,
        /// Encoder-based pose at scan time
        pub encoder_pose: Pose2D,
    }
}
