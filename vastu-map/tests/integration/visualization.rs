//! SVG visualization for manual inspection
//!
//! Generates SVG files showing:
//! - Map walls from PGM (gray background)
//! - Detected walls (teal solid lines)
//! - Robot trajectories (ground truth blue, estimated with confidence heatmap)
//! - Drift vectors at observation points
//! - Scan history at key points
//! - Ground truth pose (blue marker)
//! - Estimated pose (orange marker)
//! - Metrics overlay

use sangam_io::devices::mock::map_loader::SimulationMap;
use svg::Document;
use svg::node::element::{
    Circle, Definitions, Group, Line, Marker, Path, Polyline, Rectangle, Text,
};

use vastu_map::VectorMap;
use vastu_map::core::{Point2D, PointCloud2D, Pose2D};

use crate::harness::TrajectoryHistory;
use crate::metrics::TestMetrics;

/// Colorblind-friendly color palette (Okabe-Ito).
mod colors {
    /// Ground truth trajectory - blue
    pub const TRUTH_PATH: &str = "#0072B2";
    /// Estimated trajectory default - orange (used when not showing confidence)
    pub const ESTIMATED_PATH: &str = "#E69F00";
    /// Drift vectors - vermillion
    pub const DRIFT_VECTOR: &str = "#D55E00";
    /// Detected walls - teal
    pub const DETECTED_WALLS: &str = "#009E73";
    /// Map walls - gray
    pub const MAP_WALLS: &str = "#BBBBBB";
    /// Lidar scan - sky blue
    pub const LIDAR_SCAN: &str = "#56B4E9";
    /// Lidar scan history - lighter, semi-transparent
    pub const LIDAR_SCAN_HISTORY: &str = "#56B4E9";
    /// High confidence - green
    pub const CONFIDENCE_HIGH: &str = "#00FF00";
    /// Low confidence - red
    pub const CONFIDENCE_LOW: &str = "#FF0000";
}

/// Interval for displaying drift vectors (every Nth observation).
const DRIFT_VECTOR_INTERVAL: usize = 5;

/// Opacity for historical scans.
const SCAN_HISTORY_OPACITY: f32 = 0.3;

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

    /// Convert confidence value (0.0-1.0) to a color.
    /// 0.0 = red, 0.5 = yellow, 1.0 = green
    fn confidence_to_color(confidence: f32) -> String {
        let conf = confidence.clamp(0.0, 1.0);
        let r = if conf < 0.5 {
            255
        } else {
            ((1.0 - conf) * 2.0 * 255.0) as u8
        };
        let g = if conf > 0.5 {
            255
        } else {
            (conf * 2.0 * 255.0) as u8
        };
        format!("#{:02X}{:02X}00", r, g)
    }

    /// Render the enhanced SLAM visualization with trajectory, drift, and metrics.
    #[allow(clippy::too_many_arguments)]
    pub fn render_full(
        &self,
        slam: &VectorMap,
        map: &SimulationMap,
        final_pose: Pose2D,
        truth_pose: Pose2D,
        last_scan: Option<&PointCloud2D>,
        trajectory: &TrajectoryHistory,
        metrics: &TestMetrics,
    ) {
        // Compute bounds from map
        let map_width_m = map.width() as f32 * map.resolution();
        let map_height_m = map.height() as f32 * map.resolution();

        let width = (map_width_m * self.scale + 2.0 * self.margin) as i32;
        let height = (map_height_m * self.scale + 2.0 * self.margin) as i32;

        // Create document with arrow marker definition
        let mut doc = Document::new()
            .set("width", width)
            .set("height", height)
            .set("viewBox", (0, 0, width, height));

        // Add marker definitions for arrows
        let defs = self.create_marker_definitions();
        doc = doc.add(defs);

        // Layer 0: Background
        doc = doc.add(
            Rectangle::new()
                .set("x", 0)
                .set("y", 0)
                .set("width", width)
                .set("height", height)
                .set("fill", "white"),
        );

        // Layer 1: Map walls (gray)
        doc = doc.add(self.render_map_walls(map, height as f32));

        // Layer 2: Scan history (semi-transparent)
        doc = doc.add(self.render_scan_history(trajectory, height as f32));

        // Layer 3: Final lidar scan (full opacity)
        if let Some(scan) = last_scan {
            doc = doc.add(self.render_lidar_scan(scan, final_pose, height as f32));
        }

        // Layer 4: Detected walls (teal)
        doc = doc.add(self.render_detected(slam, height as f32));

        // Layer 5: Trajectories with confidence heatmap
        doc = doc.add(self.render_trajectories(trajectory, height as f32));

        // Layer 6: Drift vectors
        doc = doc.add(self.render_drift_vectors(trajectory, height as f32));

        // Layer 7: Final poses
        doc = doc.add(self.render_poses(final_pose, truth_pose, height as f32));

        // Layer 8: Enhanced legend
        doc = doc.add(self.render_enhanced_legend(height as f32));

        // Layer 9: Metrics overlay
        doc = doc.add(self.render_metrics_overlay(metrics, width));

        // Ensure output directory exists
        if let Some(parent) = std::path::Path::new(&self.output_path).parent() {
            std::fs::create_dir_all(parent).ok();
        }

        // Save
        svg::save(&self.output_path, &doc).expect("Failed to save SVG");
    }

    /// Create SVG marker definitions for arrows.
    fn create_marker_definitions(&self) -> Definitions {
        let arrow_marker = Marker::new()
            .set("id", "drift-arrow")
            .set("markerWidth", 6)
            .set("markerHeight", 6)
            .set("refX", 5)
            .set("refY", 3)
            .set("orient", "auto")
            .add(
                Path::new()
                    .set("d", "M0,0 L6,3 L0,6 Z")
                    .set("fill", colors::DRIFT_VECTOR),
            );

        Definitions::new().add(arrow_marker)
    }

    /// Render ground truth path (solid blue) and estimated path with confidence heatmap.
    fn render_trajectories(&self, trajectory: &TrajectoryHistory, svg_height: f32) -> Group {
        let mut group = Group::new().set("id", "trajectories");

        if trajectory.observations.len() < 2 {
            return group;
        }

        // Ground truth path - solid blue polyline
        let truth_points: String = trajectory
            .observations
            .iter()
            .map(|o| {
                let (x, y) = self.transform_point(o.truth_pose.position(), 0.0, 0.0, svg_height);
                format!("{},{}", x, y)
            })
            .collect::<Vec<_>>()
            .join(" ");

        let truth_path = Polyline::new()
            .set("points", truth_points)
            .set("fill", "none")
            .set("stroke", colors::TRUTH_PATH)
            .set("stroke-width", 2.5)
            .set("stroke-linecap", "round")
            .set("stroke-linejoin", "round");

        group = group.add(truth_path);

        // Estimated path - individual line segments colored by confidence
        for i in 1..trajectory.observations.len() {
            let prev = &trajectory.observations[i - 1];
            let curr = &trajectory.observations[i];

            // Use the current observation's confidence for the segment color
            let color = Self::confidence_to_color(curr.confidence);

            let (x1, y1) = self.transform_point(prev.slam_pose.position(), 0.0, 0.0, svg_height);
            let (x2, y2) = self.transform_point(curr.slam_pose.position(), 0.0, 0.0, svg_height);

            let segment = Line::new()
                .set("x1", x1)
                .set("y1", y1)
                .set("x2", x2)
                .set("y2", y2)
                .set("stroke", color)
                .set("stroke-width", 2.5)
                .set("stroke-linecap", "round");

            group = group.add(segment);
        }

        group
    }

    /// Render drift vectors at every N observations.
    fn render_drift_vectors(&self, trajectory: &TrajectoryHistory, svg_height: f32) -> Group {
        let mut group = Group::new().set("id", "drift_vectors");

        for (i, obs) in trajectory.observations.iter().enumerate() {
            // Show drift at fixed intervals and at the last observation
            if i % DRIFT_VECTOR_INTERVAL != 0 && i != trajectory.observations.len() - 1 {
                continue;
            }

            // Skip if drift is negligible
            if obs.drift_translation < 0.02 {
                continue;
            }

            let (x1, y1) = self.transform_point(obs.truth_pose.position(), 0.0, 0.0, svg_height);
            let (x2, y2) = self.transform_point(obs.slam_pose.position(), 0.0, 0.0, svg_height);

            // Drift vector line with arrowhead
            let line = Line::new()
                .set("x1", x1)
                .set("y1", y1)
                .set("x2", x2)
                .set("y2", y2)
                .set("stroke", colors::DRIFT_VECTOR)
                .set("stroke-width", 1.5)
                .set("marker-end", "url(#drift-arrow)");

            group = group.add(line);

            // Small circle at ground truth position
            let circle = Circle::new()
                .set("cx", x1)
                .set("cy", y1)
                .set("r", 3.0)
                .set("fill", colors::DRIFT_VECTOR)
                .set("opacity", 0.6);

            group = group.add(circle);
        }

        group
    }

    /// Render lidar scans at key points along the trajectory.
    fn render_scan_history(&self, trajectory: &TrajectoryHistory, svg_height: f32) -> Group {
        let mut group = Group::new().set("id", "scan_history");

        for obs in &trajectory.observations {
            if let Some(ref scan) = obs.scan {
                // Transform scan from robot frame to world frame
                let world_scan = scan.transform(&obs.slam_pose);

                for i in 0..world_scan.len() {
                    let point = Point2D::new(world_scan.xs[i], world_scan.ys[i]);
                    let (sx, sy) = self.transform_point(point, 0.0, 0.0, svg_height);

                    let circle = Circle::new()
                        .set("cx", sx)
                        .set("cy", sy)
                        .set("r", 2.0)
                        .set("fill", colors::LIDAR_SCAN_HISTORY)
                        .set("opacity", SCAN_HISTORY_OPACITY);

                    group = group.add(circle);
                }
            }
        }

        group
    }

    /// Render enhanced legend with all visual elements.
    fn render_enhanced_legend(&self, _svg_height: f32) -> Group {
        let mut group = Group::new().set("id", "enhanced_legend");

        let legend_x = 10.0;
        let mut legend_y = 20.0;
        let item_height = 16.0;
        let icon_width = 20.0;

        let items = vec![
            (colors::MAP_WALLS, "Map Walls", "rect"),
            (colors::DETECTED_WALLS, "Detected Walls", "line"),
            (colors::LIDAR_SCAN, "Lidar Scan", "circle"),
            (colors::TRUTH_PATH, "Ground Truth Path", "line"),
            ("gradient", "Estimated Path (Confidence)", "gradient"),
            (colors::DRIFT_VECTOR, "Drift Vector", "arrow"),
        ];

        for (color, label, icon_type) in items {
            // Draw icon based on type
            match icon_type {
                "rect" => {
                    let rect = Rectangle::new()
                        .set("x", legend_x)
                        .set("y", legend_y - 8.0)
                        .set("width", icon_width)
                        .set("height", 10.0)
                        .set("fill", color);
                    group = group.add(rect);
                }
                "line" => {
                    let line = Line::new()
                        .set("x1", legend_x)
                        .set("y1", legend_y - 3.0)
                        .set("x2", legend_x + icon_width)
                        .set("y2", legend_y - 3.0)
                        .set("stroke", color)
                        .set("stroke-width", 2.5);
                    group = group.add(line);
                }
                "circle" => {
                    let circle = Circle::new()
                        .set("cx", legend_x + icon_width / 2.0)
                        .set("cy", legend_y - 3.0)
                        .set("r", 4.0)
                        .set("fill", color);
                    group = group.add(circle);
                }
                "gradient" => {
                    // Show green-yellow-red gradient
                    let gradient_steps = 5;
                    let step_width = icon_width / gradient_steps as f32;
                    for i in 0..gradient_steps {
                        let conf = i as f32 / (gradient_steps - 1) as f32;
                        let grad_color = Self::confidence_to_color(conf);
                        let rect = Rectangle::new()
                            .set("x", legend_x + i as f32 * step_width)
                            .set("y", legend_y - 8.0)
                            .set("width", step_width + 0.5)
                            .set("height", 10.0)
                            .set("fill", grad_color);
                        group = group.add(rect);
                    }
                }
                "arrow" => {
                    let line = Line::new()
                        .set("x1", legend_x)
                        .set("y1", legend_y - 3.0)
                        .set("x2", legend_x + icon_width)
                        .set("y2", legend_y - 3.0)
                        .set("stroke", color)
                        .set("stroke-width", 1.5)
                        .set("marker-end", "url(#drift-arrow)");
                    group = group.add(line);
                }
                _ => {}
            }

            // Draw label
            let text = Text::new(label)
                .set("x", legend_x + icon_width + 8.0)
                .set("y", legend_y)
                .set("font-size", 11)
                .set("font-family", "sans-serif")
                .set("fill", "black");
            group = group.add(text);

            legend_y += item_height;
        }

        group
    }

    /// Render metrics overlay in top-right corner.
    fn render_metrics_overlay(&self, metrics: &TestMetrics, width: i32) -> Group {
        let mut group = Group::new().set("id", "metrics_overlay");

        let box_width = 210.0;
        let box_height = 110.0;
        let box_x = width as f32 - box_width - 10.0;
        let box_y = 10.0;

        // Semi-transparent background
        let bg = Rectangle::new()
            .set("x", box_x)
            .set("y", box_y)
            .set("width", box_width)
            .set("height", box_height)
            .set("fill", "white")
            .set("fill-opacity", 0.9)
            .set("stroke", "#ccc")
            .set("stroke-width", 1)
            .set("rx", 5);
        group = group.add(bg);

        // Title
        let title = Text::new("Test Metrics")
            .set("x", box_x + 10.0)
            .set("y", box_y + 18.0)
            .set("font-size", 12)
            .set("font-family", "sans-serif")
            .set("font-weight", "bold")
            .set("fill", "black");
        group = group.add(title);

        let mut text_y = box_y + 35.0;
        let line_height = 15.0;

        let lines = vec![
            format!("Position Error: {:.3} m", metrics.position_error),
            format!(
                "Orientation Error: {:.1}°",
                metrics.orientation_error.to_degrees()
            ),
            format!("Wall Accuracy: {:.0}%", metrics.wall_accuracy * 100.0),
            format!("Lines Detected: {}", metrics.detected_walls),
            format!("Avg ICP Iters: {:.1}", metrics.convergence.avg_iterations()),
        ];

        for line in lines {
            let text = Text::new(line)
                .set("x", box_x + 10.0)
                .set("y", text_y)
                .set("font-size", 11)
                .set("font-family", "monospace")
                .set("fill", "black");
            group = group.add(text);
            text_y += line_height;
        }

        group
    }

    /// Render the SLAM result to an SVG file (legacy method for backward compatibility).
    pub fn render(
        &self,
        slam: &VectorMap,
        map: &SimulationMap,
        final_pose: Pose2D,
        truth_pose: Pose2D,
        last_scan: Option<&PointCloud2D>,
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

        // Lidar scan points (cyan) - transformed to world frame using final pose
        if let Some(scan) = last_scan {
            let scan_group = self.render_lidar_scan(scan, final_pose, height as f32);
            doc = doc.add(scan_group);
        }

        // Detected walls (blue solid)
        let detected_group = self.render_detected(slam, height as f32);
        doc = doc.add(detected_group);

        // Poses
        let pose_group = self.render_poses(final_pose, truth_pose, height as f32);
        doc = doc.add(pose_group);

        // Legend
        let legend = self.render_legend(last_scan.is_some());
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
                        .set("fill", colors::MAP_WALLS)
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
                line.start,
                line.end,
                0.0,
                0.0,
                svg_height,
                colors::DETECTED_WALLS,
                2.0,
                None,
            );
            group = group.add(svg_line);
        }

        group
    }

    /// Render lidar scan points transformed to world frame.
    fn render_lidar_scan(&self, scan: &PointCloud2D, pose: Pose2D, svg_height: f32) -> Group {
        let mut group = Group::new().set("id", "lidar_scan");

        // Transform scan from robot frame to world frame
        let world_scan = scan.transform(&pose);

        for i in 0..world_scan.len() {
            let point = Point2D::new(world_scan.xs[i], world_scan.ys[i]);
            let (sx, sy) = self.transform_point(point, 0.0, 0.0, svg_height);

            let circle = Circle::new()
                .set("cx", sx)
                .set("cy", sy)
                .set("r", 3.0)
                .set("fill", colors::LIDAR_SCAN)
                .set("stroke", colors::TRUTH_PATH)
                .set("stroke-width", 0.5)
                .set("opacity", 0.8);

            group = group.add(circle);
        }

        group
    }

    fn render_poses(&self, estimated: Pose2D, truth: Pose2D, svg_height: f32) -> Group {
        let mut group = Group::new().set("id", "poses");

        // Ground truth pose (blue)
        group =
            group.add(self.create_robot_marker(truth, 0.0, 0.0, svg_height, colors::TRUTH_PATH));

        // Estimated pose (orange)
        group = group.add(self.create_robot_marker(
            estimated,
            0.0,
            0.0,
            svg_height,
            colors::ESTIMATED_PATH,
        ));

        group
    }

    fn render_legend(&self, has_scan: bool) -> Group {
        let legend_text = if has_scan {
            "Blue=Detected, Cyan=LidarScan, Gray=MapWalls, Green=TruePose, Red=EstimatedPose"
        } else {
            "Blue=Detected, Gray=MapWalls, Green=TruePose, Red=EstimatedPose"
        };

        Group::new().set("id", "legend").add(
            Text::new(legend_text)
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
