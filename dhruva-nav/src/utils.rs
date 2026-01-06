//! Shared utility functions

use std::f32::consts::PI;

/// Normalize angle to [-π, π]
#[inline]
pub fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}
