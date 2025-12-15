"""
SLAM visualization widget.

Map-only display with:
- Occupancy grid map background
- Trajectory trail
- Lidar scan overlay
- Robot marker with heading
- Auto-follow mode
- Navigation path and goal visualization
- Ctrl+Click to set navigation goal
"""

import math
import logging
import numpy as np
from collections import deque
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QPushButton
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont
import pyqtgraph as pg

logger = logging.getLogger(__name__)

# Color palette
COLORS = {
    'trajectory': (0, 200, 200),         # Cyan
    'robot': (255, 200, 0),              # Yellow/gold
    'heading_arrow': (255, 100, 0),      # Orange
    'lidar_points': (0, 255, 100),       # Green
    'grid': (60, 60, 60),                # Dark gray
    'origin': (100, 100, 100),           # Gray
    'background': (30, 30, 30),          # Dark background
    'nav_path': (0, 200, 255),           # Cyan/blue for nav path
    'nav_goal': (255, 0, 128),           # Magenta for goal
    'nav_waypoint': (100, 150, 255),     # Light blue for waypoints
}

# Pre-computed occupancy grid color lookup table for fast rendering
# Dhruva encoding: 0 = free, 100 = occupied, 255 = unknown
OCCUPANCY_COLOR_MAP = np.zeros((256, 3), dtype=np.uint8)
OCCUPANCY_COLOR_MAP[0] = [200, 200, 200]      # Free - light gray
OCCUPANCY_COLOR_MAP[100] = [255, 255, 255]    # Occupied - white
OCCUPANCY_COLOR_MAP[255] = [40, 40, 40]       # Unknown - dark gray
# Fill intermediate values (shouldn't occur, but handle gracefully)
for i in range(1, 100):
    OCCUPANCY_COLOR_MAP[i] = [200, 200, 200]  # Treat as free
for i in range(101, 255):
    OCCUPANCY_COLOR_MAP[i] = [255, 255, 255]  # Treat as occupied


class SlamView(QWidget):
    """SLAM map visualization with occupancy grid, trajectory, and lidar overlay."""

    # Signal emitted when user Ctrl+Clicks to set a goal
    # Parameters: x (float), y (float), theta (float)
    goal_selected = pyqtSignal(float, float, float)

    # Maximum trajectory points
    MAX_TRAJECTORY_POINTS = 2000

    def __init__(self, parent=None):
        super().__init__(parent)

        # Trajectory data
        self._trajectory_x = deque(maxlen=self.MAX_TRAJECTORY_POINTS)
        self._trajectory_y = deque(maxlen=self.MAX_TRAJECTORY_POINTS)

        # Current pose
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_theta = 0.0

        # Map data
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_resolution = 0.05
        self._map_width = 0
        self._map_height = 0

        # Auto-follow mode
        self._auto_follow = True

        # Navigation data
        self._nav_path = []
        self._nav_goal = None
        self._current_waypoint_idx = 0

        # Setup UI
        self._setup_ui()

    def _setup_ui(self):
        """Setup the UI components."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Create plot widget
        self._plot_widget = pg.PlotWidget()
        self._plot_widget.setBackground(COLORS['background'])
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self._plot_widget.setAspectLocked(True)
        self._plot_widget.setLabel('bottom', 'X (meters)')
        self._plot_widget.setLabel('left', 'Y (meters)')

        # Setup plot items
        self._setup_plot_items()

        layout.addWidget(self._plot_widget, 1)

        # Bottom info bar
        info_bar = self._create_info_bar()
        layout.addWidget(info_bar)

    def _setup_plot_items(self):
        """Setup plot items (map, trajectory, robot marker, etc.)."""
        # Map image item (background layer)
        self._map_item = pg.ImageItem()
        self._plot_widget.addItem(self._map_item)

        # Origin marker (crosshair at 0,0)
        origin_pen = pg.mkPen(color=COLORS['origin'], width=1)
        self._origin_h = pg.PlotDataItem([-0.5, 0.5], [0, 0], pen=origin_pen)
        self._origin_v = pg.PlotDataItem([0, 0], [-0.5, 0.5], pen=origin_pen)
        self._plot_widget.addItem(self._origin_h)
        self._plot_widget.addItem(self._origin_v)

        # Trajectory line
        trajectory_pen = pg.mkPen(color=COLORS['trajectory'], width=2)
        self._trajectory_line = pg.PlotDataItem([], [], pen=trajectory_pen)
        self._plot_widget.addItem(self._trajectory_line)

        # Lidar scan points
        self._lidar_scatter = pg.ScatterPlotItem(
            [], [],
            size=3,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(color=COLORS['lidar_points'])
        )
        self._plot_widget.addItem(self._lidar_scatter)

        # Robot marker (circle representing robot body)
        robot_brush = pg.mkBrush(color=COLORS['robot'])
        robot_pen = pg.mkPen(color=COLORS['robot'], width=2)
        self._robot_marker = pg.ScatterPlotItem(
            [0], [0],
            symbol='o',  # Circle for robot body
            size=18,
            brush=robot_brush,
            pen=robot_pen
        )
        self._plot_widget.addItem(self._robot_marker)

        # Heading line (shows direction robot is facing)
        # ROS REP-103: X=forward, Y=left, theta=CCW from X axis
        heading_pen = pg.mkPen(color=COLORS['heading_arrow'], width=4)
        self._heading_arrow = pg.PlotDataItem([0, 0.3], [0, 0], pen=heading_pen)
        self._plot_widget.addItem(self._heading_arrow)

        # Navigation path line (dashed)
        nav_path_pen = pg.mkPen(color=COLORS['nav_path'], width=3, style=Qt.DashLine)
        self._nav_path_line = pg.PlotDataItem([], [], pen=nav_path_pen)
        self._plot_widget.addItem(self._nav_path_line)

        # Navigation waypoints
        self._nav_waypoint_scatter = pg.ScatterPlotItem(
            [], [],
            size=8,
            pen=pg.mkPen(color=COLORS['nav_waypoint'], width=1),
            brush=pg.mkBrush(color=COLORS['nav_waypoint'])
        )
        self._plot_widget.addItem(self._nav_waypoint_scatter)

        # Navigation goal marker (larger cross/plus)
        self._nav_goal_marker = pg.ScatterPlotItem(
            [], [],
            symbol='+',
            size=24,
            pen=pg.mkPen(color=COLORS['nav_goal'], width=3),
            brush=pg.mkBrush(None)
        )
        self._plot_widget.addItem(self._nav_goal_marker)

        # Set initial view
        self._plot_widget.setXRange(-5, 5, padding=0.1)
        self._plot_widget.setYRange(-5, 5, padding=0.1)

        # Connect mouse click handler
        self._plot_widget.scene().sigMouseClicked.connect(self._on_mouse_clicked)

    def _create_info_bar(self) -> QFrame:
        """Create the bottom info bar with pose display and follow toggle."""
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background-color: #252525;
                border-top: 1px solid #444;
            }
            QLabel {
                color: #ccc;
                font-family: monospace;
                font-size: 12px;
            }
        """)

        layout = QHBoxLayout(frame)
        layout.setContentsMargins(10, 5, 10, 5)

        # Pose display
        self._pose_label = QLabel("X: 0.000  Y: 0.000  θ: 0.0°")
        self._pose_label.setFont(QFont("Courier", 11))
        layout.addWidget(self._pose_label)

        layout.addStretch()

        # Map info
        self._map_info_label = QLabel("No map")
        self._map_info_label.setStyleSheet("color: #777;")
        layout.addWidget(self._map_info_label)

        layout.addStretch()

        # Follow toggle button
        self._follow_btn = QPushButton("Follow: ON")
        self._follow_btn.setCheckable(True)
        self._follow_btn.setChecked(True)
        self._follow_btn.setFixedWidth(90)
        self._follow_btn.setStyleSheet("""
            QPushButton {
                background-color: #2a5a2a;
                color: white;
                border: 1px solid #444;
                border-radius: 3px;
                padding: 3px;
            }
            QPushButton:checked { background-color: #2a5a2a; }
            QPushButton:!checked { background-color: #444; }
        """)
        self._follow_btn.clicked.connect(self._toggle_follow)
        layout.addWidget(self._follow_btn)

        # Clear trajectory button
        self._clear_btn = QPushButton("Clear Trail")
        self._clear_btn.setFixedWidth(80)
        self._clear_btn.setStyleSheet("""
            QPushButton {
                background-color: #444;
                color: white;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 3px;
            }
            QPushButton:hover { background-color: #555; }
        """)
        self._clear_btn.clicked.connect(self.clear_trajectory)
        layout.addWidget(self._clear_btn)

        return frame

    def _toggle_follow(self):
        """Toggle auto-follow mode."""
        self._auto_follow = self._follow_btn.isChecked()
        self._follow_btn.setText("Follow: ON" if self._auto_follow else "Follow: OFF")

    def _on_mouse_clicked(self, event):
        """Handle mouse click events. Ctrl+Click sets navigation goal."""
        # Check if Ctrl key is held
        modifiers = event.modifiers()
        if not (modifiers & Qt.ControlModifier):
            return

        # Get click position in scene coordinates
        pos = event.scenePos()

        # Convert to view coordinates (map coordinates)
        view_pos = self._plot_widget.plotItem.vb.mapSceneToView(pos)
        x = view_pos.x()
        y = view_pos.y()

        # Compute theta pointing from robot to click point
        dx = x - self._pose_x
        dy = y - self._pose_y
        theta = math.atan2(dy, dx)

        logger.info(f"Goal selected: ({x:.2f}, {y:.2f}), theta={math.degrees(theta):.1f}°")

        # Emit signal for navigation
        self.goal_selected.emit(x, y, theta)

    # ==================== Data Update Methods ====================

    def update_pose(self, x: float, y: float, theta: float):
        """Update the robot pose and trajectory."""
        self._pose_x = x
        self._pose_y = y
        self._pose_theta = theta

        # Add to trajectory
        self._trajectory_x.append(x)
        self._trajectory_y.append(y)

        # Update trajectory line
        if len(self._trajectory_x) > 1:
            self._trajectory_line.setData(
                list(self._trajectory_x),
                list(self._trajectory_y)
            )

        # Update robot marker position
        self._robot_marker.setData([x], [y])

        # Update heading arrow
        arrow_length = 0.3
        arrow_end_x = x + arrow_length * math.cos(theta)
        arrow_end_y = y + arrow_length * math.sin(theta)
        self._heading_arrow.setData([x, arrow_end_x], [y, arrow_end_y])

        # Update pose label
        theta_deg = math.degrees(theta)
        self._pose_label.setText(f"X: {x:7.3f}  Y: {y:7.3f}  θ: {theta_deg:6.1f}°")

        # Auto-follow robot
        if self._auto_follow:
            self._center_on_robot()

    def update_map(self, map_data: dict):
        """Update occupancy grid map display.

        Args:
            map_data: Dict with resolution, width, height, origin_x, origin_y, cells
        """
        resolution = map_data.get('resolution', 0.05)
        width = map_data.get('width', 0)
        height = map_data.get('height', 0)
        origin_x = map_data.get('origin_x', 0.0)
        origin_y = map_data.get('origin_y', 0.0)
        cells = map_data.get('cells', b'')

        logger.info(f"SlamView.update_map(): {width}x{height}, resolution={resolution}m, cells={len(cells) if cells else 0} bytes")

        if not cells or width == 0 or height == 0:
            logger.debug(f"Skipping map update: empty data")
            return

        # Decode cells (raw bytes from protobuf)
        try:
            if isinstance(cells, bytes):
                cells_array = np.frombuffer(cells, dtype=np.uint8).reshape((height, width))
            else:
                logger.warning(f"Unexpected cells type: {type(cells)}")
                return
        except Exception as e:
            logger.error(f"Failed to decode map cells: {e}")
            return

        # Convert to RGB image using pre-computed lookup table (fast)
        # Dhruva encoding: 0 = free, 100 = occupied, 255 = unknown
        rgb_image = OCCUPANCY_COLOR_MAP[cells_array]  # Shape: (height, width, 3)

        # Store map metadata
        self._map_resolution = resolution
        self._map_origin_x = origin_x
        self._map_origin_y = origin_y
        self._map_width = width
        self._map_height = height

        # PyQtGraph expects image in (width, height, channels) format, so transpose
        rgb_image_transposed = np.transpose(rgb_image, (1, 0, 2))  # (h,w,c) -> (w,h,c)

        # Update map image
        self._map_item.setImage(rgb_image_transposed)
        self._map_item.setRect(
            origin_x,
            origin_y,
            width * resolution,
            height * resolution
        )

        # Update map info label (use efficient count methods)
        occupied_count = np.count_nonzero(cells_array == 100)
        free_count = np.count_nonzero(cells_array == 0)
        area_m2 = (width * height * resolution * resolution)
        self._map_info_label.setText(f"Map: {width}x{height} ({area_m2:.1f}m²)")

        logger.debug(
            f"Map updated: {width}x{height} @ {resolution}m, "
            f"occupied={occupied_count}, free={free_count}"
        )

    def update_scan(self, scan_data: dict):
        """Update lidar scan overlay.

        Args:
            scan_data: Dict with 'points' as list of [x, y] in robot frame
        """
        points = scan_data.get('points', [])

        if not points:
            self._lidar_scatter.setData([], [])
            return

        # Transform points from robot frame to global frame
        cos_theta = math.cos(self._pose_theta)
        sin_theta = math.sin(self._pose_theta)

        global_x = []
        global_y = []
        for p in points:
            # Rotate and translate
            rx = p[0] * cos_theta - p[1] * sin_theta + self._pose_x
            ry = p[0] * sin_theta + p[1] * cos_theta + self._pose_y
            global_x.append(rx)
            global_y.append(ry)

        self._lidar_scatter.setData(global_x, global_y)

    # ==================== Utility Methods ====================

    def _center_on_robot(self):
        """Center the view on the robot."""
        view_range = 5.0
        self._plot_widget.setXRange(
            self._pose_x - view_range,
            self._pose_x + view_range,
            padding=0
        )
        self._plot_widget.setYRange(
            self._pose_y - view_range,
            self._pose_y + view_range,
            padding=0
        )

    def clear_trajectory(self):
        """Clear trajectory history."""
        self._trajectory_x.clear()
        self._trajectory_y.clear()
        self._trajectory_line.setData([], [])

    def clear_map(self):
        """Clear the map display."""
        self._map_item.clear()
        self._map_info_label.setText("No map")

    def update_navigation(self, nav_data: dict):
        """Update navigation visualization (path, goal, waypoints).

        Args:
            nav_data: Dict with path, goal, current_waypoint_index, state
        """
        path = nav_data.get('path', [])
        goal = nav_data.get('goal')
        current_idx = nav_data.get('current_waypoint_index', 0)
        state = nav_data.get('state', 'Idle')

        # Store data
        self._nav_path = path
        self._nav_goal = goal
        self._current_waypoint_idx = current_idx

        # Update path line
        if path:
            path_x = [p['x'] for p in path]
            path_y = [p['y'] for p in path]
            self._nav_path_line.setData(path_x, path_y)

            # Update waypoint markers (show remaining waypoints)
            remaining_x = [p['x'] for p in path[current_idx:]]
            remaining_y = [p['y'] for p in path[current_idx:]]
            self._nav_waypoint_scatter.setData(remaining_x, remaining_y)
        else:
            self._nav_path_line.setData([], [])
            self._nav_waypoint_scatter.setData([], [])

        # Update goal marker
        if goal and state not in ('Idle', 'Reached', 'Cancelled'):
            self._nav_goal_marker.setData([goal['x']], [goal['y']])
        else:
            self._nav_goal_marker.setData([], [])

    def clear_navigation(self):
        """Clear navigation visualization."""
        self._nav_path = []
        self._nav_goal = None
        self._nav_path_line.setData([], [])
        self._nav_waypoint_scatter.setData([], [])
        self._nav_goal_marker.setData([], [])
