"""
Lidar point cloud visualization widget.

Displays 360° polar plot of lidar scan data updating at ~5Hz.
"""

import pyqtgraph as pg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
import numpy as np
import math


class LidarWidget(QWidget):
    """2D polar plot for lidar point cloud visualization"""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Layout
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Title label
        self.title_label = QLabel("Lidar Point Cloud (360° View)")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("color: white; font-size: 12pt; font-weight: bold;")
        layout.addWidget(self.title_label)

        # Info label
        self.info_label = QLabel("Waiting for lidar data...")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setStyleSheet("color: #888; font-size: 9pt;")
        layout.addWidget(self.info_label)

        # Create pyqtgraph plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#2b2b2b')
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', 'Y (meters)')
        self.plot_widget.setLabel('bottom', 'X (meters)')
        self.plot_widget.setXRange(-4, 4)
        self.plot_widget.setYRange(-4, 4)

        layout.addWidget(self.plot_widget)

        # Scatter plot item for points
        self.scatter = pg.ScatterPlotItem(size=5, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 255, 200))
        self.plot_widget.addItem(self.scatter)

        # Robot position indicator (center)
        self.robot_marker = pg.ScatterPlotItem(
            pos=[(0, 0)],
            size=20,
            pen=pg.mkPen('r', width=2),
            brush=pg.mkBrush(255, 0, 0, 100),
            symbol='o'
        )
        self.plot_widget.addItem(self.robot_marker)

        # Robot orientation indicator (forward direction)
        self.orientation_line = pg.PlotDataItem(
            [0, 0], [0, 0.3],
            pen=pg.mkPen('r', width=3)
        )
        self.plot_widget.addItem(self.orientation_line)

        # Statistics
        self.scan_number = 0
        self.point_count = 0

    def update_lidar_scan(self, scan_data):
        """
        Update lidar visualization with new scan data

        Args:
            scan_data: Dict with 'timestamp', 'scan_number', 'points' (list of dicts)
        """
        if not scan_data or 'points' not in scan_data:
            return

        self.scan_number = scan_data.get('scan_number', 0)
        points = scan_data['points']
        self.point_count = len(points)

        if self.point_count == 0:
            return

        # Convert polar coordinates (angle, distance) to Cartesian (x, y)
        x_coords = []
        y_coords = []
        colors = []

        for point in points:
            angle = point['angle']  # Radians
            distance = point['distance']  # Meters
            quality = point.get('quality', 128)  # 0-255

            # Convert to Cartesian (robot frame: forward is +Y, right is +X)
            # Lidar angle 0 is forward, increases counter-clockwise
            x = distance * math.sin(angle)
            y = distance * math.cos(angle)

            x_coords.append(x)
            y_coords.append(y)

            # Color based on quality (green=high quality, red=low quality)
            quality_norm = quality / 255.0
            r = int((1 - quality_norm) * 255)
            g = int(quality_norm * 255)
            b = 0
            colors.append((r, g, b, 200))

        # Update scatter plot
        pos = np.column_stack((x_coords, y_coords))
        brushes = [pg.mkBrush(*color) for color in colors]
        self.scatter.setData(pos=pos, brush=brushes)

        # Update info label
        self.info_label.setText(f"Scan #{self.scan_number} | Points: {self.point_count} | Rate: ~5 Hz")

    def clear(self):
        """Clear the plot"""
        self.scatter.setData(pos=[])
        self.info_label.setText("Waiting for lidar data...")
