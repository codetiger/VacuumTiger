"""
SLAM visualization widget.

Full occupancy grid map display with:
- Occupancy grid map background
- Trajectory trail
- Lidar scan overlay
- Robot marker with heading
- Click-to-move control
- SLAM status panel
"""

import math
import base64
import numpy as np
from enum import Enum
from collections import deque
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QPushButton, QComboBox, QProgressBar, QFileDialog, QMessageBox,
    QScrollArea, QSplitter
)
from .diagnostics_panel import DiagnosticsPanel
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QImage
import pyqtgraph as pg


# Color palette
COLORS = {
    'trajectory': (0, 200, 200),         # Cyan
    'robot': (255, 200, 0),              # Yellow/gold
    'heading_arrow': (255, 100, 0),      # Orange
    'lidar_points': (0, 255, 100),       # Green
    'target_marker': (255, 50, 50),      # Red
    'grid': (60, 60, 60),                # Dark gray
    'origin': (100, 100, 100),           # Gray
    'background': (30, 30, 30),          # Dark background
    'text': (200, 200, 200),             # Light gray text
    'map_free': (255, 255, 255),         # White = free
    'map_occupied': (0, 0, 0),           # Black = occupied
    'map_unknown': (128, 128, 128),      # Gray = unknown
}


class GoToState(Enum):
    """Go-to-point state machine states."""
    IDLE = 0
    ROTATING = 1
    DRIVING = 2
    ARRIVED = 3


class SlamView(QWidget):
    """SLAM visualization with occupancy grid, lidar overlay, and click-to-move."""

    # Signal emitted when velocity command is requested
    # Payload: {'linear': float, 'angular': float}
    velocity_command = pyqtSignal(dict)

    # Signal emitted when SLAM command is requested
    # Payload: {'command': str, ...}
    slam_command = pyqtSignal(dict)

    # Maximum trajectory points
    MAX_TRAJECTORY_POINTS = 1000

    def __init__(self, parent=None):
        super().__init__(parent)

        # Trajectory data
        self._trajectory_x = deque(maxlen=self.MAX_TRAJECTORY_POINTS)
        self._trajectory_y = deque(maxlen=self.MAX_TRAJECTORY_POINTS)

        # Current pose
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_theta = 0.0

        # Current lidar scan (global frame points)
        self._scan_points = []

        # Map data
        self._map_image = None
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_resolution = 0.05
        self._map_width = 0
        self._map_height = 0

        # SLAM status
        self._slam_mode = "Idle"
        self._match_score = 0.0
        self._num_keyframes = 0
        self._num_submaps = 0
        self._is_lost = False

        # Go-to-point state
        self._goto_state = GoToState.IDLE
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_angle = 0.0

        # Auto-follow mode
        self._auto_follow = True

        # Setup UI
        self._setup_ui()

        # Go-to-point control timer
        self._goto_timer = QTimer()
        self._goto_timer.setInterval(50)  # 20Hz control loop
        self._goto_timer.timeout.connect(self._goto_tick)

    def _setup_ui(self):
        """Setup the UI components."""
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Left: Map visualization
        map_widget = self._create_map_widget()
        main_layout.addWidget(map_widget, 1)

        # Right: Side panel with status and diagnostics
        right_panel = self._create_right_panel()
        main_layout.addWidget(right_panel)

    def _create_map_widget(self) -> QWidget:
        """Create the map visualization widget."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Create plot widget
        self._plot_widget = pg.PlotWidget()
        self._plot_widget.setBackground(COLORS['background'])
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self._plot_widget.setAspectLocked(True)
        self._plot_widget.setLabel('bottom', 'X (meters)')
        self._plot_widget.setLabel('left', 'Y (meters)')

        # Enable mouse click for go-to-point
        self._plot_widget.scene().sigMouseClicked.connect(self._on_map_clicked)

        # Setup plot items
        self._setup_plot_items()

        layout.addWidget(self._plot_widget, 1)

        # Bottom info bar
        info_bar = self._create_info_bar()
        layout.addWidget(info_bar)

        return widget

    def _setup_plot_items(self):
        """Setup plot items (map, trajectory, robot marker, etc.)."""
        # Map image item (background layer)
        self._map_item = pg.ImageItem()
        self._plot_widget.addItem(self._map_item)

        # Origin marker
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
            size=4,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(color=COLORS['lidar_points'])
        )
        self._plot_widget.addItem(self._lidar_scatter)

        # Target marker (X shape)
        target_pen = pg.mkPen(color=COLORS['target_marker'], width=3)
        self._target_marker = pg.ScatterPlotItem(
            [], [],
            symbol='x',
            size=20,
            pen=target_pen
        )
        self._plot_widget.addItem(self._target_marker)

        # Robot marker (triangle)
        robot_brush = pg.mkBrush(color=COLORS['robot'])
        robot_pen = pg.mkPen(color=COLORS['robot'], width=2)
        self._robot_marker = pg.ScatterPlotItem(
            [0], [0],
            symbol='t',
            size=20,
            brush=robot_brush,
            pen=robot_pen
        )
        self._plot_widget.addItem(self._robot_marker)

        # Heading arrow
        heading_pen = pg.mkPen(color=COLORS['heading_arrow'], width=3)
        self._heading_arrow = pg.PlotDataItem([0, 0.3], [0, 0], pen=heading_pen)
        self._plot_widget.addItem(self._heading_arrow)

        # Set initial view
        self._plot_widget.setXRange(-5, 5, padding=0.1)
        self._plot_widget.setYRange(-5, 5, padding=0.1)

    def _create_info_bar(self) -> QFrame:
        """Create the bottom info bar."""
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
        self._pose_label = QLabel("X: 0.000  Y: 0.000  \u03b8: 0.0\u00b0")
        self._pose_label.setFont(QFont("Courier", 11))
        layout.addWidget(self._pose_label)

        layout.addStretch()

        # Go-to status
        self._goto_label = QLabel("")
        self._goto_label.setFont(QFont("Courier", 10))
        layout.addWidget(self._goto_label)

        layout.addStretch()

        # Control buttons
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

        return frame

    def _create_status_panel(self) -> QFrame:
        """Create the right-side status panel."""
        frame = QFrame()
        frame.setFixedWidth(200)
        frame.setStyleSheet("""
            QFrame {
                background-color: #202020;
                border-left: 1px solid #444;
            }
            QLabel {
                color: #ccc;
                font-size: 11px;
            }
            QComboBox, QPushButton {
                background-color: #333;
                color: white;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #444;
            }
            QProgressBar {
                background-color: #333;
                border: 1px solid #555;
                border-radius: 3px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
            }
        """)

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Title
        title = QLabel("SLAM Status")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        # Mode selector
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["Mapping", "Localization", "Idle"])
        self._mode_combo.currentTextChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self._mode_combo)
        layout.addLayout(mode_layout)

        # Match quality bar
        layout.addWidget(QLabel("Match Quality:"))
        self._match_bar = QProgressBar()
        self._match_bar.setRange(0, 100)
        self._match_bar.setValue(0)
        layout.addWidget(self._match_bar)

        # Stats
        self._keyframes_label = QLabel("Keyframes: 0")
        layout.addWidget(self._keyframes_label)

        self._submaps_label = QLabel("Submaps: 0")
        layout.addWidget(self._submaps_label)

        # Lost warning
        self._lost_label = QLabel("\u26a0 LOST")
        self._lost_label.setStyleSheet("color: #ff4444; font-weight: bold;")
        self._lost_label.hide()
        layout.addWidget(self._lost_label)

        layout.addSpacing(10)

        # Position display
        pos_title = QLabel("Position")
        pos_title.setFont(QFont("Arial", 11, QFont.Bold))
        layout.addWidget(pos_title)

        self._pos_x_label = QLabel("X: 0.000 m")
        layout.addWidget(self._pos_x_label)

        self._pos_y_label = QLabel("Y: 0.000 m")
        layout.addWidget(self._pos_y_label)

        self._pos_theta_label = QLabel("\u03b8: 0.0\u00b0")
        layout.addWidget(self._pos_theta_label)

        layout.addSpacing(10)

        # Control buttons
        btn_layout1 = QHBoxLayout()
        self._clear_btn = QPushButton("Clear Map")
        self._clear_btn.clicked.connect(self._on_clear_map)
        btn_layout1.addWidget(self._clear_btn)

        self._save_btn = QPushButton("Save Map")
        self._save_btn.clicked.connect(self._on_save_map)
        btn_layout1.addWidget(self._save_btn)
        layout.addLayout(btn_layout1)

        btn_layout2 = QHBoxLayout()
        self._reset_btn = QPushButton("Reset Pose")
        self._reset_btn.clicked.connect(self._on_reset_pose)
        btn_layout2.addWidget(self._reset_btn)

        self._stop_btn = QPushButton("STOP")
        self._stop_btn.setStyleSheet("QPushButton { background-color: #a33; }")
        self._stop_btn.clicked.connect(self._on_stop)
        btn_layout2.addWidget(self._stop_btn)
        layout.addLayout(btn_layout2)

        layout.addStretch()

        return frame

    def _create_right_panel(self) -> QWidget:
        """Create the right panel containing status and diagnostics."""
        # Container widget
        container = QWidget()
        container.setFixedWidth(300)
        container.setStyleSheet("background-color: #1a1a1a;")

        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Scroll area for both panels
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: #1a1a1a;
            }
            QScrollBar:vertical {
                background-color: #2a2a2a;
                width: 8px;
            }
            QScrollBar::handle:vertical {
                background-color: #555;
                border-radius: 4px;
                min-height: 20px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)

        # Inner widget for scroll content
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(0)

        # Status panel (top)
        status_panel = self._create_status_panel()
        scroll_layout.addWidget(status_panel)

        # Diagnostics panel (below status)
        self._diagnostics_panel = DiagnosticsPanel()
        scroll_layout.addWidget(self._diagnostics_panel)

        scroll_layout.addStretch()

        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)

        return container

    def _toggle_follow(self):
        """Toggle auto-follow mode."""
        self._auto_follow = self._follow_btn.isChecked()
        self._follow_btn.setText("Follow: ON" if self._auto_follow else "Follow: OFF")

    # ==================== Data Update Slots ====================

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

        # Update robot marker
        self._robot_marker.setData([x], [y])

        # Update heading arrow
        arrow_length = 0.3
        arrow_end_x = x + arrow_length * math.cos(theta)
        arrow_end_y = y + arrow_length * math.sin(theta)
        self._heading_arrow.setData([x, arrow_end_x], [y, arrow_end_y])

        # Update labels
        theta_deg = math.degrees(theta)
        self._pose_label.setText(f"X: {x:7.3f}  Y: {y:7.3f}  \u03b8: {theta_deg:6.1f}\u00b0")
        self._pos_x_label.setText(f"X: {x:.3f} m")
        self._pos_y_label.setText(f"Y: {y:.3f} m")
        self._pos_theta_label.setText(f"\u03b8: {theta_deg:.1f}\u00b0")

        # Auto-follow
        if self._auto_follow:
            self._center_on_robot()

    def update_status(self, status: dict):
        """Update SLAM status display."""
        self._slam_mode = status.get('mode', 'Idle')
        self._match_score = status.get('match_score', 0.0)
        self._num_keyframes = status.get('num_keyframes', 0)
        self._num_submaps = status.get('num_submaps', 0)
        self._is_lost = status.get('is_lost', False)

        # Update UI
        self._match_bar.setValue(int(self._match_score * 100))

        # Color code match bar
        if self._match_score > 0.7:
            self._match_bar.setStyleSheet("QProgressBar::chunk { background-color: #4CAF50; }")
        elif self._match_score > 0.3:
            self._match_bar.setStyleSheet("QProgressBar::chunk { background-color: #FFC107; }")
        else:
            self._match_bar.setStyleSheet("QProgressBar::chunk { background-color: #f44336; }")

        self._keyframes_label.setText(f"Keyframes: {self._num_keyframes}")
        self._submaps_label.setText(f"Submaps: {self._num_submaps}")

        if self._is_lost:
            self._lost_label.show()
        else:
            self._lost_label.hide()

        # Update mode combo (without triggering signal)
        self._mode_combo.blockSignals(True)
        idx = self._mode_combo.findText(self._slam_mode)
        if idx >= 0:
            self._mode_combo.setCurrentIndex(idx)
        self._mode_combo.blockSignals(False)

    def update_map(self, map_data: dict):
        """Update occupancy grid map display."""
        resolution = map_data.get('resolution', 0.05)
        width = map_data.get('width', 0)
        height = map_data.get('height', 0)
        origin_x = map_data.get('origin_x', 0.0)
        origin_y = map_data.get('origin_y', 0.0)
        cells = map_data.get('cells', b'')

        if not cells or width == 0 or height == 0:
            import logging
            logging.getLogger(__name__).warning(
                f"Skipping map update: cells_len={len(cells) if cells else 0}, w={width}, h={height}"
            )
            return

        # Decode cells - handle both raw bytes (from protobuf) and base64 (legacy)
        import logging
        logger = logging.getLogger(__name__)
        try:
            if isinstance(cells, bytes):
                # Raw bytes from protobuf
                cells_array = np.frombuffer(cells, dtype=np.uint8).reshape((height, width))
            else:
                # Base64 string (legacy JSON format)
                cells_bytes = base64.b64decode(cells)
                cells_array = np.frombuffer(cells_bytes, dtype=np.uint8).reshape((height, width))
            logger.debug(f"Decoded map cells: {len(cells)} bytes -> {height}x{width} array")
        except Exception as e:
            logger.error(f"Failed to decode map cells: {e} (cells type={type(cells)}, len={len(cells) if cells else 0}, w={width}, h={height})")
            return

        # Convert to RGB image
        # 0 = unknown (gray), 1-127 = free (white), 128-255 = occupied (black)
        rgb_image = np.zeros((height, width, 3), dtype=np.uint8)

        # Unknown cells (value == 0)
        unknown_mask = cells_array == 0
        rgb_image[unknown_mask] = [128, 128, 128]

        # Free cells (1-127) -> white
        free_mask = (cells_array > 0) & (cells_array < 128)
        rgb_image[free_mask] = [255, 255, 255]

        # Occupied cells (128-255) -> black
        occupied_mask = cells_array >= 128
        rgb_image[occupied_mask] = [0, 0, 0]

        # Store map metadata
        self._map_resolution = resolution
        self._map_origin_x = origin_x
        self._map_origin_y = origin_y
        self._map_width = width
        self._map_height = height

        # pyqtgraph ImageItem expects image data where:
        # - image[row, col] corresponds to (x=col, y=row) in image coordinates
        # - By default, row 0 is at the BOTTOM of the display (y increases upward)
        # - Our cells_array[y, x] has row 0 at top (y=0 is origin_y in world coords)
        #
        # To correctly map world coordinates:
        # - World (origin_x, origin_y) should map to image row=0, col=0
        # - World Y increases upward, image row increases upward in pyqtgraph
        # - So no flip needed if we set the rect correctly
        #
        # The cells are in row-major order: cells[y][x] where y=0 is at origin_y
        # pyqtgraph will display image[0,:] at y=origin_y (bottom of rect)
        self._map_item.setImage(rgb_image)
        self._map_item.setRect(
            origin_x,
            origin_y,
            width * resolution,
            height * resolution
        )

        # Debug: log map stats
        occupied_count = np.sum(occupied_mask)
        free_count = np.sum(free_mask)
        logger.debug(
            f"Map updated: {width}x{height} @ {resolution}m, "
            f"origin=({origin_x:.1f}, {origin_y:.1f}), "
            f"occupied={occupied_count}, free={free_count}"
        )

    def update_scan(self, scan_data: dict):
        """Update lidar scan overlay."""
        points = scan_data.get('points', [])

        if not points:
            self._lidar_scatter.setData([], [])
            return

        # Points are already in global frame
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]

        self._lidar_scatter.setData(x_coords, y_coords)

    def update_diagnostics(self, data: dict):
        """Update SLAM diagnostics panel.

        Args:
            data: Dictionary with timing, scan_match, mapping, loop_closure keys
        """
        self._diagnostics_panel.update_diagnostics(data)

    # ==================== Click-to-Move ====================

    def _on_map_clicked(self, event):
        """Handle map click for go-to-point."""
        if event.button() != Qt.LeftButton:
            return

        # Get click position in plot coordinates
        pos = self._plot_widget.plotItem.vb.mapSceneToView(event.scenePos())
        target_x = pos.x()
        target_y = pos.y()

        # Start go-to-point
        self._start_goto(target_x, target_y)

    def _start_goto(self, target_x: float, target_y: float):
        """Start go-to-point navigation."""
        self._target_x = target_x
        self._target_y = target_y

        # Compute target angle
        dx = target_x - self._pose_x
        dy = target_y - self._pose_y
        self._target_angle = math.atan2(dy, dx)

        # Show target marker
        self._target_marker.setData([target_x], [target_y])

        # Start rotating
        self._goto_state = GoToState.ROTATING
        self._goto_label.setText(f"Going to ({target_x:.1f}, {target_y:.1f})")
        self._goto_timer.start()

    def _goto_tick(self):
        """Go-to-point control loop tick."""
        if self._goto_state == GoToState.IDLE or self._goto_state == GoToState.ARRIVED:
            self._goto_timer.stop()
            return

        # Angular error (normalize to [-pi, pi])
        angle_to_target = math.atan2(
            self._target_y - self._pose_y,
            self._target_x - self._pose_x
        )
        angle_error = angle_to_target - self._pose_theta
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Distance to target
        distance = math.hypot(
            self._target_x - self._pose_x,
            self._target_y - self._pose_y
        )

        if self._goto_state == GoToState.ROTATING:
            if abs(angle_error) < 0.1:  # ~5 degrees
                self._goto_state = GoToState.DRIVING
                self._goto_label.setText(f"Driving to ({self._target_x:.1f}, {self._target_y:.1f})")
            else:
                # Rotate toward target
                angular_vel = 0.3 * (1 if angle_error > 0 else -1)
                self.velocity_command.emit({'linear': 0.0, 'angular': angular_vel})

        elif self._goto_state == GoToState.DRIVING:
            if distance < 0.05:  # 5cm tolerance
                self._goto_state = GoToState.ARRIVED
                self.velocity_command.emit({'linear': 0.0, 'angular': 0.0})
                self._goto_label.setText("Arrived")
                self._target_marker.setData([], [])
                self._goto_timer.stop()
            else:
                # Drive toward target, with heading correction
                linear_vel = min(0.2, distance)  # Max 20cm/s
                angular_vel = 0.5 * angle_error  # Proportional steering
                self.velocity_command.emit({'linear': linear_vel, 'angular': angular_vel})

    def _cancel_goto(self):
        """Cancel go-to-point navigation."""
        self._goto_state = GoToState.IDLE
        self._goto_timer.stop()
        self._target_marker.setData([], [])
        self._goto_label.setText("")
        self.velocity_command.emit({'linear': 0.0, 'angular': 0.0})

    # ==================== Control Buttons ====================

    def _on_mode_changed(self, mode: str):
        """Handle mode selector change."""
        self.slam_command.emit({'command': 'set_mode', 'mode': mode})

    def _on_clear_map(self):
        """Handle Clear Map button."""
        self.slam_command.emit({'command': 'reset'})
        self._trajectory_x.clear()
        self._trajectory_y.clear()
        self._trajectory_line.setData([], [])
        self._map_item.clear()

    def _on_save_map(self):
        """Handle Save Map button."""
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Map", "", "Map files (*.bin);;All files (*)"
        )
        if path:
            self.slam_command.emit({'command': 'save_map', 'path': path})
            QMessageBox.information(self, "Save Map", f"Map save requested: {path}")

    def _on_reset_pose(self):
        """Handle Reset Pose button."""
        self.slam_command.emit({'command': 'reset_pose', 'x': 0.0, 'y': 0.0, 'theta': 0.0})

    def _on_stop(self):
        """Handle STOP button."""
        self._cancel_goto()
        self.velocity_command.emit({'linear': 0.0, 'angular': 0.0})

    # ==================== Utility ====================

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
