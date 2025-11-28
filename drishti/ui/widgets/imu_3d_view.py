"""
IMU 3D Visualization Widget

Displays a 3D rotating robot model showing real-time orientation
based on IMU sensor data (gyro + accelerometer + tilt).
"""

import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

import pyqtgraph.opengl as gl

from ..processors.imu_processor import IMUProcessor


class IMU3DView(QWidget):
    """
    3D robot orientation visualization using OpenGL.

    Shows a cylinder representing the robot body that rotates
    according to IMU-derived Euler angles.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.imu_processor = IMUProcessor()
        self._setup_ui()
        self._create_scene()

    def _setup_ui(self):
        """Set up the widget layout."""
        self.setStyleSheet("background-color: #2d2d2d; border-radius: 5px;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(2)

        # Title
        title = QLabel("IMU Orientation")
        title.setFont(QFont("Arial", 9, QFont.Bold))
        title.setStyleSheet("color: #e0e0e0; background: transparent;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # OpenGL widget
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setCameraPosition(distance=150, elevation=30, azimuth=45)
        self.gl_widget.setBackgroundColor('#2d2d2d')
        layout.addWidget(self.gl_widget, 1)

        # Euler angle display
        angle_layout = QHBoxLayout()
        angle_layout.setSpacing(5)

        label_style = "color: #aaa; background: transparent; font-size: 9px;"

        self.roll_label = QLabel("R: 0.0")
        self.roll_label.setStyleSheet(label_style)
        angle_layout.addWidget(self.roll_label)

        self.pitch_label = QLabel("P: 0.0")
        self.pitch_label.setStyleSheet(label_style)
        angle_layout.addWidget(self.pitch_label)

        self.yaw_label = QLabel("Y: 0.0")
        self.yaw_label.setStyleSheet(label_style)
        angle_layout.addWidget(self.yaw_label)

        layout.addLayout(angle_layout)

    def _create_scene(self):
        """Create the 3D scene with robot mesh and reference elements."""
        # Floor grid
        grid = gl.GLGridItem()
        grid.setSize(100, 100)
        grid.setSpacing(10, 10)
        grid.translate(0, 0, -20)
        self.gl_widget.addItem(grid)

        # XYZ axes for reference
        axis = gl.GLAxisItem()
        axis.setSize(40, 40, 40)
        self.gl_widget.addItem(axis)

        # Robot mesh (flat cylinder/disc)
        self.robot_mesh = self._create_robot_mesh()
        self.gl_widget.addItem(self.robot_mesh)

        # Forward direction indicator (red arrow)
        self.forward_arrow = self._create_forward_arrow()
        self.gl_widget.addItem(self.forward_arrow)

    def _create_robot_mesh(self):
        """Create a flat cylinder mesh representing the robot."""
        segments = 32
        radius = 35.0
        height = 8.0

        # Generate vertices for top and bottom circles
        angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)

        # Top circle vertices
        top_x = radius * np.cos(angles)
        top_y = radius * np.sin(angles)
        top_z = np.full(segments, height / 2)
        top_verts = np.column_stack([top_x, top_y, top_z])

        # Bottom circle vertices
        bottom_x = radius * np.cos(angles)
        bottom_y = radius * np.sin(angles)
        bottom_z = np.full(segments, -height / 2)
        bottom_verts = np.column_stack([bottom_x, bottom_y, bottom_z])

        # Center vertices for caps
        top_center = np.array([[0, 0, height / 2]])
        bottom_center = np.array([[0, 0, -height / 2]])

        # Combine all vertices
        # Layout: top circle (0..31), bottom circle (32..63), top center (64), bottom center (65)
        verts = np.vstack([top_verts, bottom_verts, top_center, bottom_center])

        # Generate faces
        faces = []

        # Top cap (triangles from center to edge)
        top_center_idx = segments * 2
        for i in range(segments):
            next_i = (i + 1) % segments
            faces.append([top_center_idx, i, next_i])

        # Bottom cap (triangles from center to edge, reversed winding)
        bottom_center_idx = segments * 2 + 1
        for i in range(segments):
            next_i = (i + 1) % segments
            faces.append([bottom_center_idx, segments + next_i, segments + i])

        # Side faces (quads as two triangles)
        for i in range(segments):
            next_i = (i + 1) % segments
            # Top-left triangle
            faces.append([i, segments + i, next_i])
            # Bottom-right triangle
            faces.append([next_i, segments + i, segments + next_i])

        faces = np.array(faces)

        # Create mesh item
        mesh = gl.GLMeshItem(
            vertexes=verts,
            faces=faces,
            color=(0.9, 0.9, 0.9, 1.0),
            shader='shaded',
            smooth=True
        )

        return mesh

    def _create_forward_arrow(self):
        """Create a red triangle indicating forward direction."""
        # Triangle pointing in +Y direction (forward)
        verts = np.array([
            [0, 30, 5],      # Tip (forward)
            [-8, 15, 5],     # Left base
            [8, 15, 5],      # Right base
        ])

        faces = np.array([[0, 1, 2]])

        arrow = gl.GLMeshItem(
            vertexes=verts,
            faces=faces,
            color=(0.9, 0.2, 0.2, 1.0),
            shader='shaded'
        )

        return arrow

    def update_orientation(self, gyro_x: int, gyro_y: int, gyro_z: int,
                           tilt_x: int, tilt_y: int, tilt_z: int):
        """
        Update the 3D visualization with new IMU data.

        Called at 500Hz from the telemetry thread. The IMU processor
        handles decimation to reduce UI update rate to ~50Hz.

        Args:
            gyro_x, gyro_y, gyro_z: Raw gyroscope values (i16)
            tilt_x, tilt_y, tilt_z: Low-pass filtered gravity vector (i16)
        """
        # Process IMU data through complementary filter
        roll, pitch, yaw = self.imu_processor.process(
            gyro_x, gyro_y, gyro_z,
            tilt_x, tilt_y, tilt_z
        )

        # Check if UI should be updated (decimation)
        if not self.imu_processor.should_update_ui():
            return

        # Update angle labels
        self.roll_label.setText(f"R:{roll:+5.1f}")
        self.pitch_label.setText(f"P:{pitch:+5.1f}")
        self.yaw_label.setText(f"Y:{yaw:+5.1f}")

        # Apply rotation to robot mesh
        # Reset transform and apply new rotation
        self.robot_mesh.resetTransform()
        self.robot_mesh.rotate(yaw, 0, 0, 1)    # Yaw around Z
        self.robot_mesh.rotate(pitch, 1, 0, 0)  # Pitch around X
        self.robot_mesh.rotate(roll, 0, 1, 0)   # Roll around Y

        # Apply same rotation to forward arrow
        self.forward_arrow.resetTransform()
        self.forward_arrow.rotate(yaw, 0, 0, 1)
        self.forward_arrow.rotate(pitch, 1, 0, 0)
        self.forward_arrow.rotate(roll, 0, 1, 0)

    def reset_orientation(self):
        """Reset orientation to zero."""
        self.imu_processor.reset()
        self.robot_mesh.resetTransform()
        self.forward_arrow.resetTransform()
        self.roll_label.setText("R: 0.0")
        self.pitch_label.setText("P: 0.0")
        self.yaw_label.setText("Y: 0.0")
