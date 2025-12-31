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

pub fn output_format() -> String {
    "both".to_string()
}

pub fn output_dir() -> String {
    "./output".to_string()
}
