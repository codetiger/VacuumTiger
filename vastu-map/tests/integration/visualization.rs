//! SVG visualization for manual inspection
//!
//! Generates SVG files showing:
//! - Map walls from PGM (gray background)
//! - Detected walls (blue solid lines)
//! - Ground truth pose (green marker)
//! - Estimated pose (red marker)

use sangam_io::devices::mock::map_loader::SimulationMap;
use svg::Document;
use svg::node::element::{Circle, Group, Line, Rectangle, Text};

use vastu_map::VectorMap;
use vastu_map::core::{Point2D, Pose2D};

/// Visualization generator.
pub struct Visualizer {
    output_path: String,
    scale: f32,
    margin: f32,
}

impl Visualizer {
    /// Create a new visualizer.
    ///
    /// # Arguments
    /// * `output_path` - Path for the output SVG file
    pub fn new(output_path: &str) -> Self {
        Self {
            output_path: output_path.to_string(),
            scale: 100.0, // 100 pixels per meter
            margin: 50.0,
        }
    }

    /// Render the SLAM result to an SVG file.
    pub fn render(
        &self,
        slam: &VectorMap,
        map: &SimulationMap,
        final_pose: Pose2D,
        truth_pose: Pose2D,
    ) {
        // Compute bounds from map
        let map_width_m = map.width() as f32 * map.resolution();
        let map_height_m = map.height() as f32 * map.resolution();

        let width = (map_width_m * self.scale + 2.0 * self.margin) as i32;
        let height = (map_height_m * self.scale + 2.0 * self.margin) as i32;

        // Create document
        let mut doc = Document::new()
            .set("width", width)
            .set("height", height)
            .set("viewBox", (0, 0, width, height));

        // Background
        doc = doc.add(
            Rectangle::new()
                .set("x", 0)
                .set("y", 0)
                .set("width", width)
                .set("height", height)
                .set("fill", "white"),
        );

        // Map walls (gray)
        let map_group = self.render_map_walls(map, height as f32);
        doc = doc.add(map_group);

        // Detected walls (blue solid)
        let detected_group = self.render_detected(slam, height as f32);
        doc = doc.add(detected_group);

        // Poses
        let pose_group = self.render_poses(final_pose, truth_pose, height as f32);
        doc = doc.add(pose_group);

        // Legend
        let legend = self.render_legend();
        doc = doc.add(legend);

        // Ensure output directory exists
        if let Some(parent) = std::path::Path::new(&self.output_path).parent() {
            std::fs::create_dir_all(parent).ok();
        }

        // Save
        svg::save(&self.output_path, &doc).expect("Failed to save SVG");
    }

    /// Render map walls by sampling the occupancy grid.
    fn render_map_walls(&self, map: &SimulationMap, svg_height: f32) -> Group {
        let mut group = Group::new().set("id", "map_walls");

        let resolution = map.resolution();
        let sample_step = resolution * 2.0; // Sample every 2 pixels for efficiency

        let map_width_m = map.width() as f32 * resolution;
        let map_height_m = map.height() as f32 * resolution;

        // Sample the map and draw small rectangles for occupied cells
        let mut x = 0.0;
        while x < map_width_m {
            let mut y = 0.0;
            while y < map_height_m {
                if map.is_occupied(x, y) {
                    let (sx, sy) = self.transform_point(Point2D::new(x, y), 0.0, 0.0, svg_height);
                    let rect_size = sample_step * self.scale;

                    let rect = Rectangle::new()
                        .set("x", sx - rect_size / 2.0)
                        .set("y", sy - rect_size / 2.0)
                        .set("width", rect_size)
                        .set("height", rect_size)
                        .set("fill", "#CCCCCC")
                        .set("stroke", "none");

                    group = group.add(rect);
                }
                y += sample_step;
            }
            x += sample_step;
        }

        group
    }

    fn render_detected(&self, slam: &VectorMap, svg_height: f32) -> Group {
        let mut group = Group::new().set("id", "detected");

        for line in slam.lines() {
            let svg_line = self.create_line(
                line.start, line.end, 0.0, 0.0, svg_height, "blue", 2.0, None,
            );
            group = group.add(svg_line);
        }

        group
    }

    fn render_poses(&self, estimated: Pose2D, truth: Pose2D, svg_height: f32) -> Group {
        let mut group = Group::new().set("id", "poses");

        // Ground truth pose (green)
        group = group.add(self.create_robot_marker(truth, 0.0, 0.0, svg_height, "green"));

        // Estimated pose (red)
        group = group.add(self.create_robot_marker(estimated, 0.0, 0.0, svg_height, "red"));

        group
    }

    fn render_legend(&self) -> Group {
        Group::new().set("id", "legend").add(
            Text::new("Blue=Detected, Gray=MapWalls, Green=TruePose, Red=EstimatedPose")
                .set("x", 10)
                .set("y", 20)
                .set("font-size", 12)
                .set("font-family", "monospace")
                .set("fill", "black"),
        )
    }

    fn transform_point(
        &self,
        p: Point2D,
        origin_x: f32,
        origin_y: f32,
        svg_height: f32,
    ) -> (f32, f32) {
        // Transform world coordinates to SVG coordinates
        // SVG has Y increasing downward, world has Y increasing upward
        let x = (p.x - origin_x) * self.scale + self.margin;
        let y = svg_height - ((p.y - origin_y) * self.scale + self.margin);
        (x, y)
    }

    fn create_line(
        &self,
        start: Point2D,
        end: Point2D,
        origin_x: f32,
        origin_y: f32,
        svg_height: f32,
        color: &str,
        width: f32,
        dash: Option<&str>,
    ) -> Line {
        let (x1, y1) = self.transform_point(start, origin_x, origin_y, svg_height);
        let (x2, y2) = self.transform_point(end, origin_x, origin_y, svg_height);

        let mut line = Line::new()
            .set("x1", x1)
            .set("y1", y1)
            .set("x2", x2)
            .set("y2", y2)
            .set("stroke", color)
            .set("stroke-width", width);

        if let Some(pattern) = dash {
            line = line.set("stroke-dasharray", pattern);
        }

        line
    }

    fn create_robot_marker(
        &self,
        pose: Pose2D,
        origin_x: f32,
        origin_y: f32,
        svg_height: f32,
        color: &str,
    ) -> Group {
        let (cx, cy) = self.transform_point(pose.position(), origin_x, origin_y, svg_height);

        // Robot body (circle)
        let radius = 10.0;
        let body = Circle::new()
            .set("cx", cx)
            .set("cy", cy)
            .set("r", radius)
            .set("fill", color)
            .set("opacity", 0.7);

        // Direction indicator (line)
        let dir_length = 15.0;
        // Note: In SVG, Y increases downward, so we negate the sin component
        let dir_x = cx + dir_length * pose.theta.cos();
        let dir_y = cy - dir_length * pose.theta.sin();

        let direction = Line::new()
            .set("x1", cx)
            .set("y1", cy)
            .set("x2", dir_x)
            .set("y2", dir_y)
            .set("stroke", color)
            .set("stroke-width", 3);

        Group::new().add(body).add(direction)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transform_point() {
        let viz = Visualizer::new("test.svg");
        let (x, y) = viz.transform_point(Point2D::new(0.0, 0.0), 0.0, 0.0, 600.0);

        // At origin (0,0) with svg_height=600, margin=50, scale=100
        // x = 0 * 100 + 50 = 50
        // y = 600 - (0 * 100 + 50) = 550
        assert!((x - 50.0).abs() < 0.001);
        assert!((y - 550.0).abs() < 0.001);
    }
}
