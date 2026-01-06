//! SVG visualization for scan matching audit.
//!
//! Renders occupancy grid maps and robot trajectories to SVG format.
//! The SVG serves as an audit file showing:
//! - The built occupancy grid map
//! - Ground truth trajectory
//! - Estimated (scan-matched) trajectory
//! - Odometry trajectory (optional)

use crate::core::{CellType, Pose2D, WorldPoint};
use crate::grid::GridStorage;
use std::fmt::Write;
use std::path::Path;

/// SVG color scheme for visualization
#[derive(Clone, Debug)]
pub struct SvgColorScheme {
    /// Wall/obstacle color
    pub wall: &'static str,
    /// Floor color
    pub floor: &'static str,
    /// Unknown cell color
    pub unknown: &'static str,
    /// Ground truth trajectory color
    pub gt_trajectory: &'static str,
    /// Estimated trajectory color
    pub estimated_trajectory: &'static str,
    /// Odometry trajectory color
    pub odom_trajectory: &'static str,
}

impl Default for SvgColorScheme {
    fn default() -> Self {
        Self {
            wall: "#333333",
            floor: "#FFFFFF",
            unknown: "#CCCCCC",
            gt_trajectory: "#22AA22",
            estimated_trajectory: "#2222AA",
            odom_trajectory: "#AA2222",
        }
    }
}

/// Configuration for SVG rendering
#[derive(Clone, Debug)]
pub struct SvgConfig {
    /// Pixels per meter
    pub scale: f32,
    /// Trajectory line width
    pub trajectory_width: f32,
    /// Pose marker radius
    pub marker_radius: f32,
    /// Color scheme
    pub colors: SvgColorScheme,
    /// Padding around the map in pixels
    pub padding: f32,
}

impl Default for SvgConfig {
    fn default() -> Self {
        Self {
            scale: 50.0,
            trajectory_width: 2.0,
            marker_radius: 4.0,
            colors: SvgColorScheme::default(),
            padding: 20.0,
        }
    }
}

/// A trajectory (sequence of poses) for visualization
#[derive(Clone, Debug)]
pub struct Trajectory {
    /// Name for legend
    pub name: String,
    /// Sequence of poses
    pub poses: Vec<Pose2D>,
    /// Line color
    pub color: String,
    /// Marker positions (indices into poses)
    pub marker_indices: Vec<usize>,
}

/// Debug marker for visualization
#[derive(Clone, Debug)]
pub struct DebugMarker {
    /// Position in world coordinates
    pub position: WorldPoint,
    /// Radius in pixels
    pub radius: f32,
    /// Fill color
    pub color: String,
    /// Optional label
    pub label: Option<String>,
}

/// SVG visualization builder
pub struct SvgVisualizer {
    config: SvgConfig,
    /// Grid storage for map
    storage: GridStorage,
    /// Ground truth trajectory
    gt_trajectory: Option<Trajectory>,
    /// Estimated/corrected trajectory
    est_trajectory: Option<Trajectory>,
    /// Odometry trajectory
    odom_trajectory: Option<Trajectory>,
    /// Title to display
    title: Option<String>,
    /// Debug markers (waypoints, frontiers, etc.)
    debug_markers: Vec<DebugMarker>,
    /// Whether frontiers were added (for legend)
    has_frontiers: bool,
    /// Whether waypoints were added (for legend)
    has_waypoints: bool,
}

impl SvgVisualizer {
    /// Create a new SVG visualizer
    pub fn new(storage: GridStorage, config: SvgConfig) -> Self {
        Self {
            config,
            storage,
            gt_trajectory: None,
            est_trajectory: None,
            odom_trajectory: None,
            title: None,
            debug_markers: Vec::new(),
            has_frontiers: false,
            has_waypoints: false,
        }
    }

    /// Set a title to display
    pub fn with_title(mut self, title: impl Into<String>) -> Self {
        self.title = Some(title.into());
        self
    }

    /// Add ground truth trajectory
    pub fn with_ground_truth(mut self, poses: Vec<Pose2D>, marker_indices: Vec<usize>) -> Self {
        self.gt_trajectory = Some(Trajectory {
            name: "Ground Truth".to_string(),
            poses,
            color: self.config.colors.gt_trajectory.to_string(),
            marker_indices,
        });
        self
    }

    /// Add estimated trajectory
    pub fn with_estimated(mut self, poses: Vec<Pose2D>, marker_indices: Vec<usize>) -> Self {
        self.est_trajectory = Some(Trajectory {
            name: "Estimated".to_string(),
            poses,
            color: self.config.colors.estimated_trajectory.to_string(),
            marker_indices,
        });
        self
    }

    /// Add odometry trajectory
    pub fn with_odometry(mut self, poses: Vec<Pose2D>, marker_indices: Vec<usize>) -> Self {
        self.odom_trajectory = Some(Trajectory {
            name: "Odometry".to_string(),
            poses,
            color: self.config.colors.odom_trajectory.to_string(),
            marker_indices,
        });
        self
    }

    /// Add waypoints as small markers
    pub fn with_waypoints(mut self, waypoints: Vec<WorldPoint>) -> Self {
        if !waypoints.is_empty() {
            self.has_waypoints = true;
            for wp in waypoints {
                self.debug_markers.push(DebugMarker {
                    position: wp,
                    radius: 3.0,
                    color: "#FF8800".to_string(), // Orange for waypoints
                    label: None,
                });
            }
        }
        self
    }

    /// Add frontier targets as larger markers
    pub fn with_frontiers(mut self, frontiers: Vec<WorldPoint>) -> Self {
        if !frontiers.is_empty() {
            self.has_frontiers = true;
            for (i, frontier) in frontiers.into_iter().enumerate() {
                self.debug_markers.push(DebugMarker {
                    position: frontier,
                    radius: 6.0,
                    color: "#00AA00".to_string(), // Green for frontiers
                    label: Some(format!("F{}", i + 1)),
                });
            }
        }
        self
    }

    /// Add custom debug markers
    pub fn with_markers(mut self, markers: Vec<DebugMarker>) -> Self {
        self.debug_markers.extend(markers);
        self
    }

    /// Render to SVG string
    pub fn render(&self) -> String {
        let mut svg = String::new();

        // Calculate bounds
        let (min_world, max_world) = self.storage.bounds();
        let map_width_px = (max_world.x - min_world.x) * self.config.scale;
        let map_height_px = (max_world.y - min_world.y) * self.config.scale;

        let padding = self.config.padding;
        let title_height = if self.title.is_some() { 30.0 } else { 0.0 };

        // Calculate dynamic legend height based on entries
        let mut left_entries = 0;
        if self.gt_trajectory.is_some() {
            left_entries += 1;
        }
        if self.est_trajectory.is_some() {
            left_entries += 1;
        }
        if self.odom_trajectory.is_some() {
            left_entries += 1;
        }
        if self.has_frontiers {
            left_entries += 1;
        }
        if self.has_waypoints {
            left_entries += 1;
        }
        let legend_height = (left_entries.max(3) * 20 + 25) as f32 + 10.0; // +10 for margin

        let width = map_width_px + 2.0 * padding;
        let height = map_height_px + 2.0 * padding + title_height + legend_height;

        // SVG header
        writeln!(&mut svg, r#"<?xml version="1.0" encoding="UTF-8"?>"#).unwrap();
        writeln!(
            &mut svg,
            r#"<svg xmlns="http://www.w3.org/2000/svg" width="{:.0}" height="{:.0}" viewBox="0 0 {:.0} {:.0}">"#,
            width, height, width, height
        ).unwrap();

        // Background
        writeln!(
            &mut svg,
            r##"  <rect width="100%" height="100%" fill="#F8F8F8"/>"##
        )
        .unwrap();

        // Title
        if let Some(ref title) = self.title {
            writeln!(
                &mut svg,
                r##"  <text x="{:.0}" y="22" font-family="sans-serif" font-size="16" font-weight="bold" text-anchor="middle" fill="#333">{}</text>"##,
                width / 2.0, title
            ).unwrap();
        }

        // Map group with translation
        let map_offset_x = padding;
        let map_offset_y = padding + title_height;
        writeln!(
            &mut svg,
            r#"  <g transform="translate({:.0}, {:.0})">"#,
            map_offset_x, map_offset_y
        )
        .unwrap();

        // Render grid cells
        self.render_grid(&mut svg, min_world, map_height_px);

        // Render trajectories (order: odom, estimated, ground truth - so GT is on top)
        if let Some(ref traj) = self.odom_trajectory {
            self.render_trajectory(&mut svg, traj, min_world, map_height_px);
        }
        if let Some(ref traj) = self.est_trajectory {
            self.render_trajectory(&mut svg, traj, min_world, map_height_px);
        }
        if let Some(ref traj) = self.gt_trajectory {
            self.render_trajectory(&mut svg, traj, min_world, map_height_px);
        }

        // Render debug markers (waypoints, frontiers)
        self.render_debug_markers(&mut svg, min_world, map_height_px);

        writeln!(&mut svg, "  </g>").unwrap();

        // Render legend
        let legend_y = map_offset_y + map_height_px + 10.0;
        self.render_legend(&mut svg, width, legend_y);

        // SVG footer
        writeln!(&mut svg, "</svg>").unwrap();

        svg
    }

    /// Render grid cells
    fn render_grid(&self, svg: &mut String, _min_world: WorldPoint, height_px: f32) {
        let resolution = self.storage.resolution();
        let cell_size = resolution * self.config.scale;

        writeln!(svg, r#"    <g id="grid">"#).unwrap();

        // Draw background for unknown area
        writeln!(
            svg,
            r#"      <rect width="{:.0}" height="{:.0}" fill="{}"/>"#,
            self.storage.width() as f32 * cell_size,
            self.storage.height() as f32 * cell_size,
            self.config.colors.unknown
        )
        .unwrap();

        for (coord, cell) in self.storage.iter() {
            let x = coord.x as usize;
            let y = coord.y as usize;

            let color = match cell.cell_type {
                CellType::Unknown => continue, // Skip - already background
                CellType::Floor => self.config.colors.floor,
                CellType::Wall | CellType::Cliff | CellType::Bump => self.config.colors.wall,
            };

            // SVG Y-axis is flipped (0 at top)
            let px_x = x as f32 * cell_size;
            let px_y = height_px - ((y + 1) as f32 * cell_size);

            writeln!(
                svg,
                r#"      <rect x="{:.1}" y="{:.1}" width="{:.1}" height="{:.1}" fill="{}"/>"#,
                px_x, px_y, cell_size, cell_size, color
            )
            .unwrap();
        }

        writeln!(svg, "    </g>").unwrap();
    }

    /// Render a trajectory
    fn render_trajectory(
        &self,
        svg: &mut String,
        traj: &Trajectory,
        min_world: WorldPoint,
        height_px: f32,
    ) {
        if traj.poses.is_empty() {
            return;
        }

        let id = traj.name.to_lowercase().replace(' ', "-");
        writeln!(svg, r#"    <g id="trajectory-{}">"#, id).unwrap();

        // Draw path line
        let mut path_d = String::new();
        for (i, pose) in traj.poses.iter().enumerate() {
            let px = (pose.x - min_world.x) * self.config.scale;
            let py = height_px - (pose.y - min_world.y) * self.config.scale;

            if i == 0 {
                write!(&mut path_d, "M {:.1} {:.1}", px, py).unwrap();
            } else {
                write!(&mut path_d, " L {:.1} {:.1}", px, py).unwrap();
            }
        }

        writeln!(
            svg,
            r#"      <path d="{}" fill="none" stroke="{}" stroke-width="{}" stroke-linecap="round" stroke-linejoin="round" opacity="0.8"/>"#,
            path_d, traj.color, self.config.trajectory_width
        ).unwrap();

        // Draw markers
        for &idx in &traj.marker_indices {
            if idx < traj.poses.len() {
                let pose = &traj.poses[idx];
                let px = (pose.x - min_world.x) * self.config.scale;
                let py = height_px - (pose.y - min_world.y) * self.config.scale;

                // Draw direction indicator
                let arrow_len = self.config.marker_radius * 2.0;
                let dx = arrow_len * pose.theta.cos();
                let dy = -arrow_len * pose.theta.sin(); // Flip Y for SVG

                // Circle marker
                writeln!(
                    svg,
                    r#"      <circle cx="{:.1}" cy="{:.1}" r="{:.1}" fill="{}" stroke="white" stroke-width="1"/>"#,
                    px, py, self.config.marker_radius, traj.color
                ).unwrap();

                // Direction line
                writeln!(
                    svg,
                    r#"      <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="white" stroke-width="2"/>"#,
                    px, py, px + dx, py + dy
                ).unwrap();
                writeln!(
                    svg,
                    r#"      <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="{}" stroke-width="1.5"/>"#,
                    px, py, px + dx, py + dy, traj.color
                ).unwrap();
            }
        }

        writeln!(svg, "    </g>").unwrap();
    }

    /// Render debug markers (waypoints, frontiers, etc.)
    fn render_debug_markers(&self, svg: &mut String, min_world: WorldPoint, height_px: f32) {
        if self.debug_markers.is_empty() {
            return;
        }

        writeln!(svg, r#"    <g id="debug-markers">"#).unwrap();

        for marker in &self.debug_markers {
            let px = (marker.position.x - min_world.x) * self.config.scale;
            let py = height_px - (marker.position.y - min_world.y) * self.config.scale;

            // Draw circle marker
            writeln!(
                svg,
                r#"      <circle cx="{:.1}" cy="{:.1}" r="{:.1}" fill="{}" stroke="white" stroke-width="1" opacity="0.8"/>"#,
                px, py, marker.radius, marker.color
            ).unwrap();

            // Draw label if present
            if let Some(ref label) = marker.label {
                writeln!(
                    svg,
                    r##"      <text x="{:.1}" y="{:.1}" font-family="sans-serif" font-size="10" fill="{}" text-anchor="middle" dy="-8">{}</text>"##,
                    px, py, marker.color, label
                ).unwrap();
            }
        }

        writeln!(svg, "    </g>").unwrap();
    }

    /// Render legend
    fn render_legend(&self, svg: &mut String, svg_width: f32, y_offset: f32) {
        writeln!(
            svg,
            r#"  <g id="legend" font-family="sans-serif" font-size="12" transform="translate(0, {:.0})">"#,
            y_offset
        ).unwrap();

        // Count left-side entries to calculate legend height
        let mut left_entries = 0;
        if self.gt_trajectory.is_some() {
            left_entries += 1;
        }
        if self.est_trajectory.is_some() {
            left_entries += 1;
        }
        if self.odom_trajectory.is_some() {
            left_entries += 1;
        }
        if self.has_frontiers {
            left_entries += 1;
        }
        if self.has_waypoints {
            left_entries += 1;
        }

        // Legend height: 20px per entry + 20px padding
        let legend_height = (left_entries.max(3) * 20 + 25) as f32;

        // Legend background
        writeln!(
            svg,
            r##"    <rect x="10" y="0" width="{:.0}" height="{:.0}" fill="white" stroke="#CCC" stroke-width="1" rx="4"/>"##,
            svg_width - 20.0,
            legend_height
        ).unwrap();

        let mut entry_y = 20.0;

        // Trajectory legend entries (lines)
        let traj_entries = [
            (
                self.gt_trajectory.as_ref(),
                "Ground Truth",
                self.config.colors.gt_trajectory,
            ),
            (
                self.est_trajectory.as_ref(),
                "Estimated",
                self.config.colors.estimated_trajectory,
            ),
            (
                self.odom_trajectory.as_ref(),
                "Odometry",
                self.config.colors.odom_trajectory,
            ),
        ];

        for (traj_opt, label, color) in traj_entries {
            if traj_opt.is_some() {
                // Color line
                writeln!(
                    svg,
                    r#"    <line x1="20" y1="{:.0}" x2="50" y2="{:.0}" stroke="{}" stroke-width="3"/>"#,
                    entry_y, entry_y, color
                ).unwrap();

                // Label
                writeln!(
                    svg,
                    r##"    <text x="60" y="{:.0}" fill="#333">{}</text>"##,
                    entry_y + 4.0,
                    label
                )
                .unwrap();

                entry_y += 20.0;
            }
        }

        // Debug marker legend entries (circles)
        if self.has_frontiers {
            // Green circle for frontiers
            writeln!(
                svg,
                r##"    <circle cx="35" cy="{:.0}" r="6" fill="#00AA00" stroke="white" stroke-width="1"/>"##,
                entry_y
            ).unwrap();
            writeln!(
                svg,
                r##"    <text x="60" y="{:.0}" fill="#333">Frontier Targets</text>"##,
                entry_y + 4.0
            )
            .unwrap();
            entry_y += 20.0;
        }

        if self.has_waypoints {
            // Orange circle for waypoints
            writeln!(
                svg,
                r##"    <circle cx="35" cy="{:.0}" r="3" fill="#FF8800" stroke="white" stroke-width="1"/>"##,
                entry_y
            ).unwrap();
            writeln!(
                svg,
                r##"    <text x="60" y="{:.0}" fill="#333">Path Waypoints</text>"##,
                entry_y + 4.0
            )
            .unwrap();
        }

        // Map legend (right side)
        let map_legend_x = svg_width - 150.0;
        writeln!(
            svg,
            r#"    <rect x="{:.0}" y="10" width="15" height="15" fill="{}"/>"#,
            map_legend_x, self.config.colors.wall
        )
        .unwrap();
        writeln!(
            svg,
            r##"    <text x="{:.0}" y="22" fill="#333">Walls/Obstacles</text>"##,
            map_legend_x + 20.0
        )
        .unwrap();

        writeln!(
            svg,
            r##"    <rect x="{:.0}" y="30" width="15" height="15" fill="{}" stroke="#CCC"/>"##,
            map_legend_x, self.config.colors.floor
        )
        .unwrap();
        writeln!(
            svg,
            r##"    <text x="{:.0}" y="42" fill="#333">Floor</text>"##,
            map_legend_x + 20.0
        )
        .unwrap();

        writeln!(
            svg,
            r#"    <rect x="{:.0}" y="50" width="15" height="15" fill="{}"/>"#,
            map_legend_x, self.config.colors.unknown
        )
        .unwrap();
        writeln!(
            svg,
            r##"    <text x="{:.0}" y="62" fill="#333">Unknown</text>"##,
            map_legend_x + 20.0
        )
        .unwrap();

        writeln!(svg, "  </g>").unwrap();
    }

    /// Save to file
    pub fn save(&self, path: &Path) -> Result<(), std::io::Error> {
        let svg_content = self.render();
        std::fs::write(path, svg_content)
    }
}

/// Calculate marker indices based on distance interval
pub fn markers_by_distance(poses: &[Pose2D], interval_m: f32) -> Vec<usize> {
    if poses.is_empty() {
        return vec![];
    }

    let mut markers = vec![0]; // Always include start
    let mut accumulated_dist = 0.0f32;

    for i in 1..poses.len() {
        let dist = poses[i].distance(&poses[i - 1]);
        accumulated_dist += dist;

        if accumulated_dist >= interval_m {
            markers.push(i);
            accumulated_dist = 0.0;
        }
    }

    // Always include end if different from last marker
    if poses.len() > 1 && markers.last() != Some(&(poses.len() - 1)) {
        markers.push(poses.len() - 1);
    }

    markers
}

/// Calculate marker indices based on time interval
pub fn markers_by_time(num_poses: usize, total_time_s: f32, interval_s: f32) -> Vec<usize> {
    if num_poses == 0 {
        return vec![];
    }

    let time_per_pose = total_time_s / num_poses as f32;
    let poses_per_marker = (interval_s / time_per_pose).max(1.0) as usize;

    let mut markers: Vec<usize> = (0..num_poses).step_by(poses_per_marker).collect();

    // Always include end
    if num_poses > 1 && markers.last() != Some(&(num_poses - 1)) {
        markers.push(num_poses - 1);
    }

    markers
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::GridCoord;

    #[test]
    fn test_markers_by_distance() {
        let poses = vec![
            Pose2D::new(0.0, 0.0, 0.0),
            Pose2D::new(0.3, 0.0, 0.0),
            Pose2D::new(0.6, 0.0, 0.0),
            Pose2D::new(0.9, 0.0, 0.0),
            Pose2D::new(1.2, 0.0, 0.0),
        ];

        let markers = markers_by_distance(&poses, 0.5);
        // Should have markers at: 0 (start), ~1 (0.6m), ~3 (1.2m), 4 (end)
        assert!(markers.contains(&0));
        assert!(markers.contains(&(poses.len() - 1)));
    }

    #[test]
    fn test_markers_by_time() {
        let markers = markers_by_time(100, 10.0, 1.0);
        // 100 poses over 10 seconds, marker every 1 second = every 10 poses
        assert!(markers.contains(&0));
        assert!(markers.contains(&99));
        assert!(markers.len() >= 10);
    }

    #[test]
    fn test_svg_render_basic() {
        let storage = GridStorage::new(10, 10, 0.1, WorldPoint::new(0.0, 0.0));
        let config = SvgConfig::default();
        let visualizer = SvgVisualizer::new(storage, config);

        let svg = visualizer.render();
        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));
        assert!(svg.contains("grid"));
    }

    #[test]
    fn test_svg_with_trajectory() {
        let mut storage = GridStorage::new(10, 10, 0.1, WorldPoint::new(0.0, 0.0));
        storage.set_type(GridCoord::new(5, 5), CellType::Wall);

        let poses = vec![Pose2D::new(0.1, 0.1, 0.0), Pose2D::new(0.5, 0.5, 0.785)];

        let config = SvgConfig::default();
        let visualizer = SvgVisualizer::new(storage, config)
            .with_title("Test")
            .with_ground_truth(poses.clone(), vec![0, 1]);

        let svg = visualizer.render();
        assert!(svg.contains("ground-truth"));
        assert!(svg.contains("Test"));
    }
}
