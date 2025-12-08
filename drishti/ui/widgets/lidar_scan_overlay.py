"""
Lidar scan overlay widget for real-time 360-degree point cloud visualization.

Displays lidar points in a polar/radial view centered on the robot,
showing distance and angle of detected obstacles.

Coordinate Convention (ROS REP-103):
  SangamIO transforms raw lidar data to standard robot frame:
  - angle = 0 rad → forward (+X direction)
  - angles increase counter-clockwise (CCW) when viewed from above
  - angle = π/2 rad → left (+Y direction)

  Robot frame:
  - X = forward (direction robot drives)
  - Y = left (port side)
  - Z = up

Note: The angle transform from hardware to ROS convention is done in SangamIO
via the [device.hardware.frame_transforms.lidar] config.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QRadialGradient
import math


class LidarScanWidget(QWidget):
    """Custom widget that renders lidar points in polar coordinates."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self._points = []  # List of (angle_rad, distance_m, quality)
        self._max_range = 3.2  # Max display range in meters (2.5x zoom vs 8m)
        self._robot_radius = 0.175  # Robot radius in meters (350mm diameter)

    def set_points(self, points: list):
        """Set lidar points to display.

        SangamIO sends lidar data as a separate sensor group at ~5Hz,
        so this is only called when a new scan arrives.

        Args:
            points: List of (angle_rad, distance_m, quality) tuples
        """
        if not points:
            return
        self._points = points
        self.update()

    def set_max_range(self, range_m: float):
        """Set maximum display range in meters."""
        self._max_range = max(1.0, range_m)
        self.update()

    def paintEvent(self, event):
        """Paint the lidar visualization."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Get widget dimensions
        w = self.width()
        h = self.height()
        size = min(w, h) - 10
        center_x = w // 2
        center_y = h // 2
        scale = size / (2 * self._max_range)  # pixels per meter

        # Draw background with gradient
        gradient = QRadialGradient(center_x, center_y, size // 2)
        gradient.setColorAt(0, QColor(30, 30, 35))
        gradient.setColorAt(1, QColor(20, 20, 25))
        painter.setBrush(QBrush(gradient))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(center_x - size // 2, center_y - size // 2, size, size)

        # Draw range rings
        ring_pen = QPen(QColor(60, 60, 70))
        ring_pen.setWidth(1)
        painter.setPen(ring_pen)
        painter.setBrush(Qt.NoBrush)

        # Draw rings at 0.5m, 1m, 1.5m, 2m, 2.5m, 3m intervals
        ring_distances = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
        for dist in ring_distances:
            if dist <= self._max_range:
                radius = int(dist * scale)
                painter.drawEllipse(center_x - radius, center_y - radius,
                                    radius * 2, radius * 2)

        # Draw cardinal direction lines
        line_pen = QPen(QColor(50, 50, 60))
        line_pen.setWidth(1)
        painter.setPen(line_pen)
        max_r = int(self._max_range * scale)

        # Forward (up), Back, Left, Right
        painter.drawLine(center_x, center_y - max_r, center_x, center_y + max_r)
        painter.drawLine(center_x - max_r, center_y, center_x + max_r, center_y)

        # Diagonal lines
        diag = int(max_r * 0.707)
        painter.drawLine(center_x - diag, center_y - diag,
                         center_x + diag, center_y + diag)
        painter.drawLine(center_x + diag, center_y - diag,
                         center_x - diag, center_y + diag)

        # Draw robot body (small circle at center)
        robot_r = int(self._robot_radius * scale)
        robot_pen = QPen(QColor(100, 100, 120))
        robot_pen.setWidth(2)
        painter.setPen(robot_pen)
        painter.setBrush(QColor(50, 50, 60))
        painter.drawEllipse(center_x - robot_r, center_y - robot_r,
                            robot_r * 2, robot_r * 2)

        # Draw forward direction indicator on robot
        painter.setPen(QPen(QColor(80, 180, 80), 2))
        painter.drawLine(center_x, center_y, center_x, center_y - robot_r - 5)

        # Draw lidar points
        if not self._points:
            # Draw "No Data" text
            painter.setPen(QColor(100, 100, 100))
            painter.drawText(self.rect(), Qt.AlignCenter, "No Lidar Data")
            painter.end()
            return

        # Point colors based on quality
        for angle_rad, distance_m, quality in self._points:
            if distance_m <= 0 or distance_m > self._max_range:
                continue

            # Convert polar to cartesian for screen display
            # ROS REP-103: angle=0 is forward (robot +X), CCW positive
            # Standard polar: robot_x = r*cos(θ), robot_y = r*sin(θ)
            # Screen mapping: forward=up, left=left (not mirrored)
            #   screen_x = -robot_y = -r*sin(θ)  (negate to keep left on left side)
            #   screen_y = -robot_x = -r*cos(θ)  (negative because screen Y points down)
            x = -distance_m * math.sin(angle_rad)
            y = -distance_m * math.cos(angle_rad)

            # Scale to screen coordinates
            px = center_x + int(x * scale)
            py = center_y + int(y * scale)

            # Color based on quality (0-255)
            if quality > 180:
                color = QColor(50, 255, 100)  # High quality - green
            elif quality > 100:
                color = QColor(255, 220, 50)  # Medium quality - yellow
            else:
                color = QColor(255, 80, 80)   # Low quality - red

            # Point size based on quality
            point_size = 2 + (quality // 100)

            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(px - point_size // 2, py - point_size // 2,
                                point_size, point_size)

        painter.end()


class LidarScanOverlay(QWidget):
    """Floating overlay widget for lidar scan visualization."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            background-color: rgba(45, 45, 45, 200);
            border-radius: 8px;
        """)
        self.setFixedSize(250, 290)

        self._setup_ui()
        self._point_count = 0

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        # Header with title and stats
        header_layout = QHBoxLayout()
        header_layout.setSpacing(8)

        title = QLabel("Lidar Scan")
        title.setStyleSheet(
            "color: #ddd; font-weight: bold; font-size: 11px; background: transparent;"
        )
        header_layout.addWidget(title)

        header_layout.addStretch()

        self.points_label = QLabel("0 pts")
        self.points_label.setStyleSheet(
            "color: #888; font-size: 10px; background: transparent;"
        )
        header_layout.addWidget(self.points_label)

        layout.addLayout(header_layout)

        # Lidar visualization widget
        self.scan_widget = LidarScanWidget()
        self.scan_widget.setStyleSheet("background: transparent;")
        layout.addWidget(self.scan_widget, 1)

        # Range indicator
        range_layout = QHBoxLayout()
        range_layout.setSpacing(4)

        range_label = QLabel("Range:")
        range_label.setStyleSheet(
            "color: #888; font-size: 9px; background: transparent;"
        )
        range_layout.addWidget(range_label)

        self.range_value = QLabel("8.0m")
        self.range_value.setStyleSheet(
            "color: #aaa; font-size: 9px; background: transparent;"
        )
        range_layout.addWidget(self.range_value)

        range_layout.addStretch()

        # Quality legend
        legend = QLabel("H/M/L quality")
        legend.setStyleSheet(
            "color: #666; font-size: 9px; background: transparent;"
        )
        range_layout.addWidget(legend)

        layout.addLayout(range_layout)

    def update_scan(self, points: list):
        """Update the lidar scan display.

        Args:
            points: List of (angle_rad, distance_m, quality) tuples
        """
        if not points:
            return
        self._point_count = len(points)
        self.points_label.setText(f"{self._point_count} pts")
        self.scan_widget.set_points(points)

    def set_max_range(self, range_m: float):
        """Set maximum display range."""
        self.scan_widget.set_max_range(range_m)
        self.range_value.setText(f"{range_m:.1f}m")
