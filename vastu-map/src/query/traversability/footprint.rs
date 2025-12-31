//! Robot footprint definition for collision checking.

use serde::{Deserialize, Serialize};

/// Robot footprint for collision checking.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotFootprint {
    /// Robot radius in meters.
    pub radius: f32,
    /// Safety margin in meters.
    pub safety_margin: f32,
}

impl Default for RobotFootprint {
    fn default() -> Self {
        Self {
            radius: 0.17,        // CRL-200S robot radius
            safety_margin: 0.05, // 5cm safety margin
        }
    }
}

impl RobotFootprint {
    /// Create a new robot footprint.
    pub fn new(radius: f32, safety_margin: f32) -> Self {
        Self {
            radius,
            safety_margin,
        }
    }

    /// Total radius including safety margin.
    pub fn total_radius(&self) -> f32 {
        self.radius + self.safety_margin
    }
}
