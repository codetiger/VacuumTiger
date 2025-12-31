//! Default value functions for serde deserialization.

pub fn resolution() -> f32 {
    0.025
}

pub fn grid_size() -> usize {
    800
}

pub fn origin_mode() -> String {
    "center".to_string()
}

pub fn enabled() -> bool {
    true
}

pub fn lidar_offset_x() -> f32 {
    -0.110
}

pub fn min_range() -> f32 {
    0.15
}

pub fn max_range() -> f32 {
    8.0
}

pub fn robot_radius() -> f32 {
    0.17
}

pub fn left_bumper_angle() -> f32 {
    0.5
}

pub fn right_bumper_angle() -> f32 {
    -0.5
}

pub fn min_frontier_size() -> usize {
    5
}

pub fn safety_margin() -> f32 {
    0.05
}

pub fn diagonal_cost() -> f32 {
    std::f32::consts::SQRT_2
}

pub fn max_iterations() -> usize {
    100_000
}

pub fn los_step() -> f32 {
    0.05
}

pub fn smooth_iterations() -> usize {
    100
}

pub fn frontier_threshold() -> f32 {
    0.3
}

pub fn waypoint_threshold() -> f32 {
    0.1
}

pub fn max_plan_distance() -> f32 {
    5.0
}

pub fn replan_interval() -> f32 {
    1.0
}

pub fn max_failures() -> usize {
    5
}

pub fn turn_angle() -> f32 {
    std::f32::consts::FRAC_PI_4
}

pub fn backup_distance() -> f32 {
    0.1
}

pub fn output_format() -> String {
    "both".to_string()
}

pub fn output_dir() -> String {
    "./output".to_string()
}

pub fn motion_max_time() -> f32 {
    5.0
}

pub fn motion_max_distance() -> f32 {
    0.2
}

pub fn motion_max_angle() -> f32 {
    0.17
}
