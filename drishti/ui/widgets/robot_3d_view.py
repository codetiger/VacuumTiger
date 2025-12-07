"""
Full-screen 3D wireframe robot visualization widget.

Replaces the 2D top-view with a see-through 3D wireframe showing
animated internal components with IMU orientation overlay.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QProgressBar
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QColor, QLinearGradient

import pyqtgraph.opengl as gl

from .lidar_scan_overlay import LidarScanOverlay
from .robot_3d_geometry import (
    create_cylinder_wireframe,
    create_arc_wireframe,
    create_circle_wireframe,
    create_3arm_brush_wireframe,
    create_roller_brush_wireframe,
    create_fan_wireframe,
    create_wheel_wireframe,
    create_lidar_turret_wireframe,
    create_lidar_spinner_wireframe,
    translate_geometry,
    rotate_geometry_z,
    rotate_geometry_x,
    rotate_geometry_y,
    rotate_wheel_x,
)
from ..processors.imu_processor import IMUProcessor


# Color palette (RGBA, 0.0-1.0)
COLORS = {
    'body': (0.5, 0.5, 0.5, 0.6),
    'body_edge': (0.4, 0.4, 0.4, 1.0),
    'bumper_ok': (0.5, 0.5, 0.5, 1.0),
    'bumper_triggered': (1.0, 0.15, 0.15, 1.0),
    'lidar': (0.3, 0.3, 0.3, 1.0),
    'lidar_spinner': (0.5, 0.7, 0.9, 1.0),
    'lidar_spinner_active': (0.3, 0.9, 1.0, 1.0),
    'wheel': (0.25, 0.25, 0.25, 1.0),
    'wheel_moving': (0.3, 0.75, 0.3, 1.0),
    'brush_off': (0.6, 0.4, 0.2, 0.8),
    'brush_spinning': (1.0, 0.5, 0.2, 1.0),
    'side_brush': (0.7, 0.7, 0.7, 1.0),
    'cliff_ok': (0.2, 0.85, 0.2, 1.0),
    'cliff_triggered': (1.0, 0.15, 0.15, 1.0),
    'button_off': (0.4, 0.4, 0.4, 1.0),
    'button_on': (0.2, 0.85, 0.2, 1.0),
    'fan': (0.5, 0.5, 0.6, 0.8),
    'fan_spinning': (0.4, 0.6, 0.9, 1.0),
    'grid': (0.25, 0.25, 0.25, 0.5),
    'axis_x': (1.0, 0.2, 0.2, 1.0),
    'axis_y': (0.2, 1.0, 0.2, 1.0),
    'axis_z': (0.2, 0.2, 1.0, 1.0),
}

# Camera presets
CAMERA_PRESETS = {
    'iso': {'distance': 450, 'elevation': 30, 'azimuth': 45},
    'top': {'distance': 500, 'elevation': 89, 'azimuth': 0},
    'front': {'distance': 450, 'elevation': 15, 'azimuth': 90},
    'side': {'distance': 450, 'elevation': 15, 'azimuth': 0},
}


class BatteryBar(QWidget):
    """Custom battery bar widget with animated charging effect."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(10)
        self._level = 0  # 0-100
        self._charging = False
        self._animation_phase = 0

    def set_level(self, level: int):
        """Set battery level (0-100)."""
        self._level = max(0, min(100, level))
        self.update()

    def set_charging(self, charging: bool):
        """Set charging state."""
        self._charging = charging
        self.update()

    def advance_animation(self):
        """Advance charging animation phase."""
        self._animation_phase = (self._animation_phase + 1) % 4
        if self._charging:
            self.update()

    def paintEvent(self, event):
        """Paint the battery bar."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Background (empty battery)
        painter.setBrush(QColor(60, 60, 60))
        painter.setPen(QColor(80, 80, 80))
        painter.drawRoundedRect(0, 0, w, h, 3, 3)

        # Calculate fill width
        fill_width = int((w - 4) * self._level / 100)

        if fill_width > 0:
            # Determine color based on level
            if self._level > 50:
                color = QColor(76, 175, 80)  # Green
            elif self._level > 20:
                color = QColor(255, 193, 7)  # Yellow/Orange
            else:
                color = QColor(244, 67, 54)  # Red

            # If charging, create animated gradient
            if self._charging:
                gradient = QLinearGradient(2, 0, 2 + fill_width, 0)
                # Animate by shifting gradient stops
                offset = self._animation_phase * 0.25
                gradient.setColorAt(0, color.darker(120))
                gradient.setColorAt((0.5 + offset) % 1.0, color.lighter(130))
                gradient.setColorAt(1, color.darker(120))
                painter.setBrush(gradient)
            else:
                painter.setBrush(color)

            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(2, 2, fill_width, h - 4, 2, 2)

        painter.end()


class Robot3DView(QWidget):
    """
    Full-screen 3D wireframe robot visualization.

    Display coordinate system (for 3D rendering):
    - Origin at robot center
    - +X points right (robot's starboard side)
    - +Y points forward (front of robot)
    - +Z points up
    - All dimensions in mm

    Note: This display convention differs from ROS REP-103 sensor data convention
    (where +X is forward). Sensor data from SangamIO uses ROS REP-103 frame and
    is transformed for display as needed.
    """

    # Robot physical dimensions (mm) - CRL-200S
    ROBOT_DIAMETER = 350.0
    ROBOT_RADIUS = ROBOT_DIAMETER / 2
    ROBOT_HEIGHT = 50.0

    # Bumper arc
    BUMPER_ARC_ANGLE = 220  # degrees, centered on front
    BUMPER_THICKNESS = 6.0

    # Lidar turret
    LIDAR_RADIUS = 35.0
    LIDAR_HEIGHT = 20.0
    LIDAR_Y_OFFSET = -100.0  # rear
    LIDAR_SPINNER_RADIUS = 25.0  # Inner spinning element

    # Drive wheels (cylinder: radius = height/2, width = thickness)
    WHEEL_WIDTH = 18.0        # Thickness of wheel (was 25, now 75% = ~18)
    WHEEL_RADIUS = 40.0       # Wheel radius (was height/2 = 20, now 2x = 40)
    WHEEL_X_OFFSET = 130.0
    WHEEL_Z_OFFSET = -10.0    # Adjusted for larger wheel

    # Main brush roller
    MAIN_BRUSH_WIDTH = 140.0
    MAIN_BRUSH_RADIUS = 15.0
    MAIN_BRUSH_Y_OFFSET = -30.0
    MAIN_BRUSH_Z_OFFSET = -20.0

    # Side brush
    SIDE_BRUSH_RADIUS = 35.0
    SIDE_BRUSH_X_OFFSET = -100.0
    SIDE_BRUSH_Y_OFFSET = 80.0
    SIDE_BRUSH_Z_OFFSET = -20.0

    # Cliff sensors
    CLIFF_SENSOR_RADIUS = 8.0
    CLIFF_POSITIONS = {
        'left_front': (-70.0, 140.0),
        'right_front': (70.0, 140.0),
        'left_side': (-130.0, 60.0),
        'right_side': (130.0, 60.0),
    }
    CLIFF_Z_OFFSET = -25.0

    # Buttons
    BUTTON_RADIUS = 10.0
    BUTTON_Y_OFFSET = 100.0
    BUTTON_SPACING = 25.0
    BUTTON_Z_OFFSET = 25.0

    # Vacuum fan
    FAN_RADIUS = 30.0
    FAN_Z_OFFSET = 0.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.imu_processor = IMUProcessor()

        # Sensor state
        self.sensor_data = {
            'is_charging': False,
            'battery_level': 0,
            'wheel_left': 0,
            'wheel_right': 0,
            'bumper_left': False,
            'bumper_right': False,
            'cliff_left_front': False,
            'cliff_right_front': False,
            'cliff_left_side': False,
            'cliff_right_side': False,
            'start_button': 0,
            'dock_button': 0,
        }

        # Actuator speeds (0-100%)
        self.actuator_speeds = {
            'vacuum': 0.0,
            'main_brush': 0.0,
            'side_brush': 0.0,
        }

        # Animation state
        self.wheel_left_angle = 0.0
        self.wheel_right_angle = 0.0
        self.side_brush_angle = 0.0
        self.main_brush_angle = 0.0
        self.fan_angle = 0.0
        self.lidar_spinner_angle = 0.0
        self.last_wheel_left = 0
        self.last_wheel_right = 0
        self.lidar_enabled = False

        # Battery state
        self.battery_level = 0
        self.battery_voltage = 0.0
        self.is_charging = False
        self._charging_animation_phase = 0

        # Sensor state cache for change detection
        self._cache = {}

        # GL items storage
        self.gl_items = {}

        # Base geometry templates (for rotation)
        self._wheel_left_base = None
        self._wheel_right_base = None
        self._side_brush_base = None
        self._main_brush_base = None
        self._fan_base = None
        self._lidar_spinner_base = None

        self._setup_ui()
        self._create_scene()
        self._start_animation()

        # Cache items list for fast IMU transform iteration
        self._gl_items_list = list(self.gl_items.values())

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # OpenGL widget (full size)
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setCameraPosition(**CAMERA_PRESETS['iso'])
        self.gl_widget.setBackgroundColor('#1a1a1a')
        layout.addWidget(self.gl_widget, 1)

        # View controls overlay (top-left)
        self._create_view_overlay()

        # IMU display overlay (bottom-left)
        self._create_imu_overlay()

        # Battery status overlay (bottom-right)
        self._create_battery_overlay()

        # Button counter overlay (top-right, hidden by default)
        self._create_button_overlay()

        # Lidar scan overlay (top-right, below button overlay)
        self._create_lidar_overlay()

    def _create_view_overlay(self):
        """Create floating view control buttons."""
        self.view_overlay = QWidget(self.gl_widget)
        self.view_overlay.setStyleSheet("""
            background-color: rgba(45, 45, 45, 180);
            border-radius: 8px;
        """)
        self.view_overlay.setFixedSize(80, 160)
        self.view_overlay.move(10, 10)

        layout = QVBoxLayout(self.view_overlay)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(5)

        title = QLabel("View")
        title.setStyleSheet("color: #ddd; font-weight: bold; background: transparent;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        btn_style = """
            QPushButton {
                background: #555;
                color: white;
                border: none;
                padding: 5px;
                border-radius: 3px;
            }
            QPushButton:hover { background: #666; }
            QPushButton:pressed { background: #444; }
        """

        for name in ['Iso', 'Top', 'Front', 'Side']:
            btn = QPushButton(name)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(lambda _, n=name.lower(): self._set_camera_preset(n))
            layout.addWidget(btn)

        layout.addStretch()

    def _create_imu_overlay(self):
        """Create IMU angle display overlay."""
        self.imu_overlay = QWidget(self.gl_widget)
        self.imu_overlay.setStyleSheet("""
            background-color: rgba(45, 45, 45, 180);
            border-radius: 8px;
        """)
        self.imu_overlay.setFixedSize(150, 70)

        layout = QVBoxLayout(self.imu_overlay)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(2)

        title = QLabel("IMU Orientation")
        title.setStyleSheet("color: #ddd; font-weight: bold; font-size: 10px; background: transparent;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        angle_layout = QHBoxLayout()
        angle_layout.setSpacing(5)

        label_style = "color: #aaa; background: transparent; font-size: 10px;"

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

    def _create_battery_overlay(self):
        """Create battery status display overlay with charging animation."""
        self.battery_overlay = QWidget(self.gl_widget)
        self.battery_overlay.setStyleSheet("""
            background-color: rgba(45, 45, 45, 180);
            border-radius: 8px;
        """)
        self.battery_overlay.setFixedSize(130, 85)

        layout = QVBoxLayout(self.battery_overlay)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(3)

        # Title with charging indicator
        title_layout = QHBoxLayout()
        title_layout.setSpacing(4)

        title = QLabel("Battery")
        title.setStyleSheet("color: #ddd; font-weight: bold; font-size: 10px; background: transparent;")
        title_layout.addWidget(title)

        self.charging_indicator = QLabel("")
        self.charging_indicator.setStyleSheet("color: #4CAF50; font-size: 14px; background: transparent;")
        title_layout.addWidget(self.charging_indicator)
        title_layout.addStretch()

        layout.addLayout(title_layout)

        # Battery percentage and voltage in same row
        info_layout = QHBoxLayout()
        info_layout.setSpacing(8)

        self.battery_percent_label = QLabel("0%")
        self.battery_percent_label.setStyleSheet("color: #aaa; font-size: 18px; font-weight: bold; background: transparent;")
        info_layout.addWidget(self.battery_percent_label)

        self.battery_voltage_label = QLabel("0.0V")
        self.battery_voltage_label.setStyleSheet("color: #888; font-size: 11px; background: transparent;")
        self.battery_voltage_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        info_layout.addWidget(self.battery_voltage_label)

        layout.addLayout(info_layout)

        # Battery bar container
        bar_container = QWidget()
        bar_container.setFixedHeight(14)
        bar_container.setStyleSheet("background: transparent;")
        bar_layout = QHBoxLayout(bar_container)
        bar_layout.setContentsMargins(0, 0, 0, 0)
        bar_layout.setSpacing(0)

        # Battery bar (custom painted)
        self.battery_bar = BatteryBar()
        bar_layout.addWidget(self.battery_bar)

        layout.addWidget(bar_container)

    def _create_button_overlay(self):
        """Create button counter overlay (shown when buttons are pressed)."""
        self.button_overlay = QWidget(self.gl_widget)
        self.button_overlay.setStyleSheet("""
            background-color: rgba(45, 45, 45, 200);
            border-radius: 8px;
        """)
        self.button_overlay.setFixedSize(120, 70)
        self.button_overlay.hide()  # Hidden by default

        layout = QVBoxLayout(self.button_overlay)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(4)

        title = QLabel("Button Press")
        title.setStyleSheet("color: #ddd; font-weight: bold; font-size: 11px; background: transparent;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Start button counter
        start_layout = QHBoxLayout()
        start_layout.setSpacing(8)
        start_label = QLabel("Start:")
        start_label.setStyleSheet("color: #aaa; font-size: 11px; background: transparent;")
        start_layout.addWidget(start_label)
        self.start_counter_label = QLabel("0")
        self.start_counter_label.setStyleSheet("color: #4CAF50; font-size: 14px; font-weight: bold; background: transparent;")
        self.start_counter_label.setAlignment(Qt.AlignRight)
        start_layout.addWidget(self.start_counter_label)
        layout.addLayout(start_layout)

        # Home/Dock button counter
        home_layout = QHBoxLayout()
        home_layout.setSpacing(8)
        home_label = QLabel("Home:")
        home_label.setStyleSheet("color: #aaa; font-size: 11px; background: transparent;")
        home_layout.addWidget(home_label)
        self.home_counter_label = QLabel("0")
        self.home_counter_label.setStyleSheet("color: #4CAF50; font-size: 14px; font-weight: bold; background: transparent;")
        self.home_counter_label.setAlignment(Qt.AlignRight)
        home_layout.addWidget(self.home_counter_label)
        layout.addLayout(home_layout)

    def _create_lidar_overlay(self):
        """Create lidar scan visualization overlay."""
        self.lidar_overlay = LidarScanOverlay(self.gl_widget)
        self.lidar_overlay.hide()  # Hidden until lidar is enabled
        # Position will be set in resizeEvent

    def resizeEvent(self, event):
        """Handle resize to position overlays."""
        super().resizeEvent(event)
        # Position IMU overlay at bottom-left
        if hasattr(self, 'imu_overlay'):
            self.imu_overlay.move(10, self.height() - self.imu_overlay.height() - 10)
        # Position battery overlay at bottom-right
        if hasattr(self, 'battery_overlay'):
            self.battery_overlay.move(
                self.width() - self.battery_overlay.width() - 10,
                self.height() - self.battery_overlay.height() - 10
            )
        # Position button overlay at top-right
        if hasattr(self, 'button_overlay'):
            self.button_overlay.move(
                self.width() - self.button_overlay.width() - 10,
                10
            )
        # Position lidar overlay at top-right
        if hasattr(self, 'lidar_overlay'):
            self.lidar_overlay.move(
                self.width() - self.lidar_overlay.width() - 10,
                10
            )

    def _set_camera_preset(self, preset: str):
        """Apply a camera preset."""
        if preset in CAMERA_PRESETS:
            self.gl_widget.setCameraPosition(**CAMERA_PRESETS[preset])

    def _create_scene(self):
        """Create the 3D scene with all robot components."""
        # Floor grid
        grid = gl.GLGridItem()
        grid.setSize(400, 400)
        grid.setSpacing(50, 50)
        grid.translate(0, 0, -30)
        self.gl_widget.addItem(grid)

        # XYZ axes
        axis = gl.GLAxisItem()
        axis.setSize(80, 80, 80)
        self.gl_widget.addItem(axis)

        # Create all robot components
        self._create_body()
        self._create_bumper()
        self._create_lidar()
        self._create_wheels()
        self._create_main_brush()
        self._create_side_brush()
        self._create_cliff_sensors()
        self._create_buttons()
        self._create_fan()

    def _create_body(self):
        """Create robot body wireframe with denser geometry."""
        body_geom = create_cylinder_wireframe(
            self.ROBOT_RADIUS, self.ROBOT_HEIGHT,
            segments=64, vertical_lines=16, horizontal_rings=2
        )
        self.gl_items['body'] = gl.GLLinePlotItem(
            pos=body_geom,
            color=COLORS['body_edge'],
            width=1.5,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['body'])

    def _create_bumper(self):
        """Create front bumper arc (left and right halves) with 3D extrusion."""
        # Bumper spans from (90 - half_angle) to (90 + half_angle) degrees
        # where 90 degrees is +Y direction (front)
        half_angle = self.BUMPER_ARC_ANGLE / 2
        bumper_radius = self.ROBOT_RADIUS - self.BUMPER_THICKNESS / 2
        bumper_height = 15.0  # 3D extrusion height

        # Right half: 90 - half_angle to 90
        right_geom = create_arc_wireframe(
            bumper_radius, 90 - half_angle, 90,
            thickness=self.BUMPER_THICKNESS, segments=24,
            height=bumper_height, height_segments=2
        )
        right_geom = translate_geometry(right_geom, 0, 0, self.ROBOT_HEIGHT / 2 + 2)
        self.gl_items['bumper_right'] = gl.GLLinePlotItem(
            pos=right_geom,
            color=COLORS['bumper_ok'],
            width=2.0,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['bumper_right'])

        # Left half: 90 to 90 + half_angle
        left_geom = create_arc_wireframe(
            bumper_radius, 90, 90 + half_angle,
            thickness=self.BUMPER_THICKNESS, segments=24,
            height=bumper_height, height_segments=2
        )
        left_geom = translate_geometry(left_geom, 0, 0, self.ROBOT_HEIGHT / 2 + 2)
        self.gl_items['bumper_left'] = gl.GLLinePlotItem(
            pos=left_geom,
            color=COLORS['bumper_ok'],
            width=2.0,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['bumper_left'])

    def _create_lidar(self):
        """Create lidar turret with outer housing and inner spinning element."""
        lidar_z = self.ROBOT_HEIGHT / 2 + self.LIDAR_HEIGHT / 2

        # Outer housing (static)
        lidar_geom = create_lidar_turret_wireframe(
            self.LIDAR_RADIUS, self.LIDAR_HEIGHT,
            segments=32, vertical_lines=8
        )
        lidar_geom = translate_geometry(
            lidar_geom, 0, self.LIDAR_Y_OFFSET, lidar_z
        )
        self.gl_items['lidar'] = gl.GLLinePlotItem(
            pos=lidar_geom,
            color=COLORS['lidar'],
            width=1.5,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['lidar'])

        # Inner spinner (rotates when lidar is active)
        spinner_geom = create_lidar_spinner_wireframe(
            self.LIDAR_SPINNER_RADIUS, self.LIDAR_HEIGHT * 0.8,
            segments=16, blades=4
        )
        self._lidar_spinner_base = spinner_geom.copy()
        spinner_geom = translate_geometry(
            spinner_geom, 0, self.LIDAR_Y_OFFSET, lidar_z
        )
        self.gl_items['lidar_spinner'] = gl.GLLinePlotItem(
            pos=spinner_geom,
            color=COLORS['lidar_spinner'],
            width=1.5,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['lidar_spinner'])

    def _create_wheels(self):
        """Create drive wheels as cylinders."""
        # Left wheel - cylinder with axis along X
        left_geom = create_wheel_wireframe(self.WHEEL_WIDTH, self.WHEEL_RADIUS)
        self._wheel_left_base = left_geom.copy()
        left_geom = translate_geometry(
            left_geom, -self.WHEEL_X_OFFSET, 0, self.WHEEL_Z_OFFSET
        )
        self.gl_items['wheel_left'] = gl.GLLinePlotItem(
            pos=left_geom,
            color=COLORS['wheel'],
            width=1.5,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['wheel_left'])

        # Right wheel
        right_geom = create_wheel_wireframe(self.WHEEL_WIDTH, self.WHEEL_RADIUS)
        self._wheel_right_base = right_geom.copy()
        right_geom = translate_geometry(
            right_geom, self.WHEEL_X_OFFSET, 0, self.WHEEL_Z_OFFSET
        )
        self.gl_items['wheel_right'] = gl.GLLinePlotItem(
            pos=right_geom,
            color=COLORS['wheel'],
            width=1.5,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['wheel_right'])

    def _create_main_brush(self):
        """Create main brush roller."""
        brush_geom = create_roller_brush_wireframe(
            self.MAIN_BRUSH_WIDTH, self.MAIN_BRUSH_RADIUS
        )
        self._main_brush_base = brush_geom.copy()
        brush_geom = translate_geometry(
            brush_geom, 0, self.MAIN_BRUSH_Y_OFFSET, self.MAIN_BRUSH_Z_OFFSET
        )
        self.gl_items['main_brush'] = gl.GLLinePlotItem(
            pos=brush_geom,
            color=COLORS['brush_off'],
            width=1.5,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['main_brush'])

    def _create_side_brush(self):
        """Create 3-arm side brush."""
        brush_geom = create_3arm_brush_wireframe(self.SIDE_BRUSH_RADIUS)
        self._side_brush_base = brush_geom.copy()
        brush_geom = translate_geometry(
            brush_geom, self.SIDE_BRUSH_X_OFFSET,
            self.SIDE_BRUSH_Y_OFFSET, self.SIDE_BRUSH_Z_OFFSET
        )
        self.gl_items['side_brush'] = gl.GLLinePlotItem(
            pos=brush_geom,
            color=COLORS['side_brush'],
            width=2.0,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['side_brush'])

    def _create_cliff_sensors(self):
        """Create cliff sensor indicators."""
        for name, (x, y) in self.CLIFF_POSITIONS.items():
            sensor_geom = create_circle_wireframe(self.CLIFF_SENSOR_RADIUS, segments=16)
            sensor_geom = translate_geometry(sensor_geom, x, y, self.CLIFF_Z_OFFSET)
            self.gl_items[f'cliff_{name}'] = gl.GLLinePlotItem(
                pos=sensor_geom,
                color=COLORS['cliff_ok'],
                width=2.0,
                mode='lines',
                antialias=True
            )
            self.gl_widget.addItem(self.gl_items[f'cliff_{name}'])

    def _create_buttons(self):
        """Create start and dock button indicators."""
        # Dock button (top)
        dock_geom = create_circle_wireframe(self.BUTTON_RADIUS, segments=16)
        dock_geom = translate_geometry(
            dock_geom, 0, self.BUTTON_Y_OFFSET - self.BUTTON_SPACING / 2,
            self.BUTTON_Z_OFFSET
        )
        self.gl_items['button_dock'] = gl.GLLinePlotItem(
            pos=dock_geom,
            color=COLORS['button_off'],
            width=2.0,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['button_dock'])

        # Start button (bottom)
        start_geom = create_circle_wireframe(self.BUTTON_RADIUS, segments=16)
        start_geom = translate_geometry(
            start_geom, 0, self.BUTTON_Y_OFFSET + self.BUTTON_SPACING / 2,
            self.BUTTON_Z_OFFSET
        )
        self.gl_items['button_start'] = gl.GLLinePlotItem(
            pos=start_geom,
            color=COLORS['button_off'],
            width=2.0,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['button_start'])

    def _create_fan(self):
        """Create vacuum fan/impeller."""
        fan_geom = create_fan_wireframe(self.FAN_RADIUS, blades=6)
        self._fan_base = fan_geom.copy()
        fan_geom = translate_geometry(fan_geom, 0, 0, self.FAN_Z_OFFSET)
        self.gl_items['fan'] = gl.GLLinePlotItem(
            pos=fan_geom,
            color=COLORS['fan'],
            width=1.5,
            mode='lines',
            antialias=True
        )
        self.gl_widget.addItem(self.gl_items['fan'])

    def _start_animation(self):
        """Start animation timer."""
        self.animation_timer = QTimer(self)
        self.animation_timer.timeout.connect(self._update_animation)
        self.animation_timer.start(16)  # ~60 FPS

    def _update_animation(self):
        """Update brush/wheel/lidar animations. IMU is updated separately."""
        # Update wheel rotation from encoder deltas
        wheel_left = self.sensor_data.get('wheel_left', 0)
        wheel_right = self.sensor_data.get('wheel_right', 0)

        wheel_moving = False
        if wheel_left != self.last_wheel_left:
            delta = wheel_left - self.last_wheel_left
            # Handle wrap-around
            if abs(delta) < 30000:
                self.wheel_left_angle += delta * 0.5  # Degrees per tick
                wheel_moving = True
            self.last_wheel_left = wheel_left

        if wheel_right != self.last_wheel_right:
            delta = wheel_right - self.last_wheel_right
            if abs(delta) < 30000:
                self.wheel_right_angle += delta * 0.5
                wheel_moving = True
            self.last_wheel_right = wheel_right

        # Update wheel colors based on movement
        if wheel_moving:
            if 'wheel_left' in self.gl_items:
                self.gl_items['wheel_left'].setData(color=COLORS['wheel_moving'])
            if 'wheel_right' in self.gl_items:
                self.gl_items['wheel_right'].setData(color=COLORS['wheel_moving'])
        else:
            if 'wheel_left' in self.gl_items:
                self.gl_items['wheel_left'].setData(color=COLORS['wheel'])
            if 'wheel_right' in self.gl_items:
                self.gl_items['wheel_right'].setData(color=COLORS['wheel'])

        # Update brush/fan rotation from actuator speeds
        side_speed = self.actuator_speeds.get('side_brush', 0)
        main_speed = self.actuator_speeds.get('main_brush', 0)
        vacuum_speed = self.actuator_speeds.get('vacuum', 0)

        if side_speed > 0:
            self.side_brush_angle += 7.5 * (side_speed / 100.0)
        if main_speed > 0:
            self.main_brush_angle += 4.5 * (main_speed / 100.0)
        if vacuum_speed > 0:
            self.fan_angle += 12.0 * (vacuum_speed / 100.0)

        # Update lidar spinner rotation when enabled
        if self.lidar_enabled:
            self.lidar_spinner_angle += 8.0  # Constant spin rate

        # Update battery charging animation
        if self.is_charging:
            self._charging_animation_phase += 1
            if self._charging_animation_phase % 10 == 0:  # Every ~160ms
                if hasattr(self, 'battery_bar'):
                    self.battery_bar.advance_animation()
                # Update charging indicator color animation
                self.update_battery(self.battery_level, self.battery_voltage, self.is_charging)

        # Update animated geometry (brushes/fan/wheels/lidar)
        self._update_animated_geometry()

    def _update_animated_geometry(self):
        """Update animated geometry (brushes/fan/wheels/lidar)."""
        # Side brush rotation (Z axis)
        if self._side_brush_base is not None:
            rotated = rotate_geometry_z(self._side_brush_base, self.side_brush_angle)
            translated = translate_geometry(
                rotated, self.SIDE_BRUSH_X_OFFSET,
                self.SIDE_BRUSH_Y_OFFSET, self.SIDE_BRUSH_Z_OFFSET
            )
            self.gl_items['side_brush'].setData(pos=translated)

        # Main brush rotation (X axis - roller)
        if self._main_brush_base is not None:
            rotated = rotate_geometry_x(self._main_brush_base, self.main_brush_angle)
            translated = translate_geometry(
                rotated, 0, self.MAIN_BRUSH_Y_OFFSET, self.MAIN_BRUSH_Z_OFFSET
            )
            self.gl_items['main_brush'].setData(pos=translated)

        # Fan rotation (Z axis)
        if self._fan_base is not None:
            rotated = rotate_geometry_z(self._fan_base, self.fan_angle)
            translated = translate_geometry(rotated, 0, 0, self.FAN_Z_OFFSET)
            self.gl_items['fan'].setData(pos=translated)

        # Left wheel rotation (around X axis - rolling motion)
        if self._wheel_left_base is not None:
            rotated = rotate_wheel_x(self._wheel_left_base, self.wheel_left_angle)
            translated = translate_geometry(
                rotated, -self.WHEEL_X_OFFSET, 0, self.WHEEL_Z_OFFSET
            )
            self.gl_items['wheel_left'].setData(pos=translated)

        # Right wheel rotation (around X axis - rolling motion)
        if self._wheel_right_base is not None:
            rotated = rotate_wheel_x(self._wheel_right_base, self.wheel_right_angle)
            translated = translate_geometry(
                rotated, self.WHEEL_X_OFFSET, 0, self.WHEEL_Z_OFFSET
            )
            self.gl_items['wheel_right'].setData(pos=translated)

        # Lidar spinner rotation (Z axis)
        if self._lidar_spinner_base is not None:
            lidar_z = self.ROBOT_HEIGHT / 2 + self.LIDAR_HEIGHT / 2
            rotated = rotate_geometry_z(self._lidar_spinner_base, self.lidar_spinner_angle)
            translated = translate_geometry(
                rotated, 0, self.LIDAR_Y_OFFSET, lidar_z
            )
            self.gl_items['lidar_spinner'].setData(pos=translated)

    def _apply_imu_transform(self, roll: float, pitch: float, yaw: float):
        """Apply IMU orientation to all robot components."""
        for item in self._gl_items_list:
            item.resetTransform()
            item.rotate(yaw, 0, 0, 1)    # Yaw around Z
            item.rotate(pitch, 1, 0, 0)  # Pitch around X
            item.rotate(roll, 0, 1, 0)   # Roll around Y

    def update_sensors(self, data: dict):
        """Update sensor data from telemetry."""
        self.sensor_data.update(data)

        # Update bumper colors
        bumper_left = data.get('bumper_left', False)
        if bumper_left != self._cache.get('bumper_left'):
            color = COLORS['bumper_triggered'] if bumper_left else COLORS['bumper_ok']
            self.gl_items['bumper_left'].setData(color=color)
            self._cache['bumper_left'] = bumper_left

        bumper_right = data.get('bumper_right', False)
        if bumper_right != self._cache.get('bumper_right'):
            color = COLORS['bumper_triggered'] if bumper_right else COLORS['bumper_ok']
            self.gl_items['bumper_right'].setData(color=color)
            self._cache['bumper_right'] = bumper_right

        # Update cliff sensor colors
        cliff_map = {
            'cliff_left_front': 'cliff_left_front',
            'cliff_right_front': 'cliff_right_front',
            'cliff_left_side': 'cliff_left_side',
            'cliff_right_side': 'cliff_right_side',
        }
        for sensor_key, item_key in cliff_map.items():
            triggered = data.get(sensor_key, False)
            if triggered != self._cache.get(sensor_key):
                color = COLORS['cliff_triggered'] if triggered else COLORS['cliff_ok']
                self.gl_items[item_key].setData(color=color)
                self._cache[sensor_key] = triggered

        # Update button colors and counter overlay
        # Button counters increment from 0 while pressed, reset to 0 when released
        start_counter = data.get('start_button', 0)
        dock_counter = data.get('dock_button', 0)
        start_pressed = start_counter > 0
        dock_pressed = dock_counter > 0

        if start_pressed != self._cache.get('start_pressed'):
            color = COLORS['button_on'] if start_pressed else COLORS['button_off']
            self.gl_items['button_start'].setData(color=color)
            self._cache['start_pressed'] = start_pressed

        if dock_pressed != self._cache.get('dock_pressed'):
            color = COLORS['button_on'] if dock_pressed else COLORS['button_off']
            self.gl_items['button_dock'].setData(color=color)
            self._cache['dock_pressed'] = dock_pressed

        # Update button counter overlay
        if start_pressed or dock_pressed:
            if hasattr(self, 'button_overlay'):
                self.start_counter_label.setText(str(start_counter) if start_pressed else "—")
                self.home_counter_label.setText(str(dock_counter) if dock_pressed else "—")
                if not self.button_overlay.isVisible():
                    self.button_overlay.show()
        else:
            if hasattr(self, 'button_overlay') and self.button_overlay.isVisible():
                self.button_overlay.hide()

        # Update battery status
        battery_level = data.get('battery_level', None)
        battery_voltage = data.get('battery_voltage', None)
        is_charging = data.get('is_charging', None)
        if battery_level is not None or is_charging is not None or battery_voltage is not None:
            self.update_battery(
                battery_level if battery_level is not None else self.battery_level,
                battery_voltage if battery_voltage is not None else self.battery_voltage,
                is_charging if is_charging is not None else self.is_charging
            )

    def update_battery(self, level: int, voltage: float, charging: bool):
        """Update battery display.

        Args:
            level: Battery level 0-100
            voltage: Battery voltage in volts
            charging: True if charging
        """
        self.battery_level = level
        self.battery_voltage = voltage
        self.is_charging = charging

        # Update battery bar
        if hasattr(self, 'battery_bar'):
            self.battery_bar.set_level(level)
            self.battery_bar.set_charging(charging)

        # Update percentage label
        if hasattr(self, 'battery_percent_label'):
            self.battery_percent_label.setText(f"{level}%")
            # Color based on level
            if level > 50:
                color = "#4CAF50"  # Green
            elif level > 20:
                color = "#FFC107"  # Yellow/Orange
            else:
                color = "#F44336"  # Red
            self.battery_percent_label.setStyleSheet(
                f"color: {color}; font-size: 18px; font-weight: bold; background: transparent;"
            )

        # Update voltage label
        if hasattr(self, 'battery_voltage_label'):
            self.battery_voltage_label.setText(f"{voltage:.1f}V")

        # Update charging indicator with animation
        if hasattr(self, 'charging_indicator'):
            if charging:
                # Animate the charging indicator
                phase = self._charging_animation_phase % 4
                if phase == 0:
                    self.charging_indicator.setText("⚡")
                    self.charging_indicator.setStyleSheet("color: #4CAF50; font-size: 14px; background: transparent;")
                elif phase == 1:
                    self.charging_indicator.setText("⚡")
                    self.charging_indicator.setStyleSheet("color: #8BC34A; font-size: 14px; background: transparent;")
                elif phase == 2:
                    self.charging_indicator.setText("⚡")
                    self.charging_indicator.setStyleSheet("color: #CDDC39; font-size: 14px; background: transparent;")
                else:
                    self.charging_indicator.setText("⚡")
                    self.charging_indicator.setStyleSheet("color: #FFEB3B; font-size: 14px; background: transparent;")
            else:
                self.charging_indicator.setText("")

    def update_actuator(self, actuator_id: str, speed: float):
        """Update actuator speed for animation.

        Args:
            actuator_id: 'vacuum', 'brush', or 'side_brush'
            speed: Speed in percent (0-100)
        """
        if actuator_id == 'brush':
            self.actuator_speeds['main_brush'] = speed
            color = COLORS['brush_spinning'] if speed > 0 else COLORS['brush_off']
            self.gl_items['main_brush'].setData(color=color)
        elif actuator_id == 'side_brush':
            self.actuator_speeds['side_brush'] = speed
            color = COLORS['brush_spinning'] if speed > 0 else COLORS['side_brush']
            self.gl_items['side_brush'].setData(color=color)
        elif actuator_id == 'vacuum':
            self.actuator_speeds['vacuum'] = speed
            color = COLORS['fan_spinning'] if speed > 0 else COLORS['fan']
            self.gl_items['fan'].setData(color=color)

    def set_lidar_enabled(self, enabled: bool):
        """Set lidar enabled state for spinner animation and overlay visibility.

        Args:
            enabled: True to spin the lidar, False to stop
        """
        self.lidar_enabled = enabled
        if 'lidar_spinner' in self.gl_items:
            color = COLORS['lidar_spinner_active'] if enabled else COLORS['lidar_spinner']
            self.gl_items['lidar_spinner'].setData(color=color)
        # Show/hide lidar scan overlay based on enabled state
        if hasattr(self, 'lidar_overlay'):
            if enabled:
                self.lidar_overlay.show()
            else:
                self.lidar_overlay.hide()

    def update_lidar_scan(self, points: list):
        """Update lidar scan overlay with new point data.

        Args:
            points: List of (angle_rad, distance_m, quality) tuples
                    Angles are in ROS REP-103 convention:
                    - 0 rad = forward (+X)
                    - CCW positive when viewed from above

        Note: SangamIO transforms raw lidar angles to ROS REP-103 convention
        before streaming.
        """
        if hasattr(self, 'lidar_overlay'):
            self.lidar_overlay.update_scan(points)

    def update_orientation(self, gyro_x: int, gyro_y: int, gyro_z: int,
                           tilt_x: int, tilt_y: int, tilt_z: int):
        """
        Update orientation from IMU data.

        Called at 500Hz from telemetry thread. Updates every call for
        smooth real-time response (labels decimated separately).

        Args:
            gyro_x: Roll rate (X axis rotation) - ROS REP-103 frame
            gyro_y: Pitch rate (Y axis rotation) - ROS REP-103 frame
            gyro_z: Yaw rate (Z axis rotation, CCW positive) - ROS REP-103 frame
            tilt_x, tilt_y, tilt_z: LP-filtered gravity vector

        Note: SangamIO transforms raw IMU axes to ROS REP-103 convention before
        streaming, so the input data is already in standard robot frame.
        """
        roll, pitch, yaw = self.imu_processor.process(
            gyro_x, gyro_y, gyro_z,
            tilt_x, tilt_y, tilt_z
        )

        # Always apply IMU transform for smooth updates
        self._apply_imu_transform(roll, pitch, yaw)

        # Update labels at decimated rate to reduce string operations
        if self.imu_processor.should_update_ui():
            self.roll_label.setText(f"R:{roll:+5.1f}")
            self.pitch_label.setText(f"P:{pitch:+5.1f}")
            self.yaw_label.setText(f"Y:{yaw:+5.1f}")

    def reset_orientation(self):
        """Reset IMU orientation to zero."""
        self.imu_processor.reset()
        self.roll_label.setText("R: 0.0")
        self.pitch_label.setText("P: 0.0")
        self.yaw_label.setText("Y: 0.0")
        # Reset all transforms
        self._apply_imu_transform(0.0, 0.0, 0.0)

    def keyPressEvent(self, event):
        """Handle keyboard shortcuts for view control."""
        key = event.key()

        if key == Qt.Key_1:
            self._set_camera_preset('iso')
        elif key == Qt.Key_2:
            self._set_camera_preset('top')
        elif key == Qt.Key_3:
            self._set_camera_preset('front')
        elif key == Qt.Key_4:
            self._set_camera_preset('side')
        elif key == Qt.Key_R:
            self.reset_orientation()
        else:
            super().keyPressEvent(event)
