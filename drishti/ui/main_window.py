"""
Main window for Drishti robot visualization application.

Full-screen 3D wireframe robot visualization with real-time sensor overlays,
IMU-based orientation display, and SLAM map/trajectory visualization.
"""

from PyQt5.QtCore import Qt, pyqtSlot, QEvent
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QStatusBar, QTabWidget
)
from PyQt5.QtGui import QFont, QKeyEvent
import logging

from ui.threads.telemetry_thread import TelemetryThread
from ui.threads.odometry_thread import OdometryThread
from ui.widgets.robot_3d_view import Robot3DView
from ui.widgets.slam_view import SlamView
from ui.widgets.control_panel import ControlPanel

logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    """Main application window with full-screen robot diagram."""

    def __init__(self, robot_ip="192.168.68.101", port=5555, log_raw_packets=False,
                 slam_host=None, slam_port=5557, no_slam=False):
        super().__init__()

        self.robot_ip = robot_ip
        self.port = port
        self.log_raw_packets = log_raw_packets
        self.slam_host = slam_host or "localhost"
        self.slam_port = slam_port
        self.slam_enabled = not no_slam
        self.telemetry_thread = None
        self.odometry_thread = None
        self.lidar_enabled = False  # Lidar starts disabled
        self.side_brush_enabled = False
        self.main_brush_enabled = False
        self.vacuum_enabled = False

        # Drive control state
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.motor_enabled = False
        self.pressed_keys = set()  # Track currently pressed arrow keys for combinations

        # Initialize UI
        self.setWindowTitle(f"Drishti - CRL-200S Monitor ({robot_ip})")
        self.setGeometry(100, 100, 800, 800)
        self.setStyleSheet("background-color: #f5f5f5;")

        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)

        # Title bar
        title_bar = self._create_title_bar()
        main_layout.addWidget(title_bar)

        # Content area: tab widget (left) + control panel (right)
        content_layout = QHBoxLayout()
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)

        # Tab widget for different views
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: none;
                background-color: #1a1a1a;
            }
            QTabBar::tab {
                background-color: #333;
                color: #aaa;
                padding: 8px 20px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #1a1a1a;
                color: #fff;
            }
            QTabBar::tab:hover:!selected {
                background-color: #444;
            }
        """)

        # 3D robot view (Tab 1)
        self.robot_3d_view = Robot3DView()
        self.tab_widget.addTab(self.robot_3d_view, "Robot View")

        # SLAM view (Tab 2)
        self.slam_view = SlamView()
        self.slam_view.velocity_command.connect(self._on_slam_velocity)
        self.slam_view.slam_command.connect(self._on_slam_command)
        self.tab_widget.addTab(self.slam_view, "SLAM")

        # Handle tab changes to show/hide control panel
        self.tab_widget.currentChanged.connect(self._on_tab_changed)

        content_layout.addWidget(self.tab_widget, 1)  # Stretch factor 1 (expands)

        # Control panel (right side, fixed width) - dark theme
        self.control_panel = ControlPanel()
        self.control_panel.setStyleSheet("background-color: #2d2d2d;")
        self.control_panel.command_requested.connect(self._on_panel_command)
        content_layout.addWidget(self.control_panel, 0)  # Stretch factor 0 (fixed)

        main_layout.addLayout(content_layout, 1)

        # Status bar
        self.status_bar = QStatusBar()
        self.status_bar.setStyleSheet("background-color: #e0e0e0;")
        self.setStatusBar(self.status_bar)
        self.connection_status_label = QLabel("Connecting...")
        self.connection_status_label.setStyleSheet("color: #666; font-weight: bold;")
        self.status_bar.addPermanentWidget(self.connection_status_label)

        # Ensure arrow keys go to main window for robot control
        self.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)

        # Connect to robot
        self._start_telemetry()

        # Connect to SLAM daemon (optional)
        if self.slam_enabled:
            self._start_odometry()

    def _create_title_bar(self):
        """Create application title bar."""
        title_widget = QWidget()
        title_widget.setStyleSheet("background-color: #2d2d2d; padding: 8px;")
        title_widget.setFixedHeight(50)
        layout = QVBoxLayout(title_widget)
        layout.setContentsMargins(15, 5, 15, 5)

        title_label = QLabel(f"Drishti - CRL-200S Monitor")
        title_label.setFont(QFont("Arial", 14, QFont.Bold))
        title_label.setStyleSheet("color: white;")
        layout.addWidget(title_label)

        return title_widget

    def _start_telemetry(self):
        """Start telemetry thread to receive sensor data."""
        try:
            self.status_bar.showMessage("Connecting to robot...")

            # Create and start telemetry thread
            self.telemetry_thread = TelemetryThread(
                self.robot_ip, self.port, log_raw_packets=self.log_raw_packets
            )
            self.telemetry_thread.sensor_data_received.connect(self._on_sensor_data)
            self.telemetry_thread.connection_status.connect(self._on_connection_status)
            self.telemetry_thread.start()

        except Exception as e:
            logger.error(f"Failed to start telemetry: {e}")
            self.connection_status_label.setText("Connection Failed")
            self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
            self.status_bar.showMessage(f"Error: {e}")

    def _start_odometry(self):
        """Start odometry thread to receive pose data from dhruva-slam-node."""
        try:
            logger.info(f"Connecting to SLAM at {self.slam_host}:{self.slam_port}")

            # Create and start odometry thread
            self.odometry_thread = OdometryThread(
                self.slam_host, self.slam_port
            )
            self.odometry_thread.pose_received.connect(self._on_pose_data)
            self.odometry_thread.diagnostics_received.connect(self._on_diagnostics_data)
            self.odometry_thread.connection_status.connect(self._on_slam_connection_status)

            # Connect SLAM signals to SlamView
            self.odometry_thread.slam_status_received.connect(self.slam_view.update_status)
            self.odometry_thread.slam_map_received.connect(self.slam_view.update_map)
            self.odometry_thread.slam_scan_received.connect(self.slam_view.update_scan)
            self.odometry_thread.slam_diagnostics_received.connect(self.slam_view.update_diagnostics)

            self.odometry_thread.start()

        except Exception as e:
            logger.error(f"Failed to start odometry: {e}")
            self.status_bar.showMessage(f"SLAM Error: {e}", 3000)

    @pyqtSlot(dict)
    def _on_pose_data(self, pose: dict):
        """Handle pose data from odometry thread."""
        x = pose.get('x', 0.0)
        y = pose.get('y', 0.0)
        theta = pose.get('theta', 0.0)
        # Update SLAM view with pose for trajectory
        self.slam_view.update_pose(x, y, theta)

    @pyqtSlot(dict)
    def _on_diagnostics_data(self, diagnostics: dict):
        """Handle diagnostics data from odometry thread."""
        # Diagnostics are no longer shown in SLAM view
        pass

    @pyqtSlot(dict)
    def _on_slam_velocity(self, velocity: dict):
        """Handle velocity command from SLAM view (go-to-point controller)."""
        linear = velocity.get('linear', 0.0)
        angular = velocity.get('angular', 0.0)
        command = {
            "type": "ComponentControl",
            "id": "drive",
            "action": {
                "type": "Configure",
                "config": {
                    "linear": {"F32": linear},
                    "angular": {"F32": angular}
                }
            }
        }
        self._send_command(command)
        self.control_panel.set_velocity(linear, angular)
        logger.debug(f"SLAM velocity: linear={linear:.2f}, angular={angular:.2f}")

    @pyqtSlot(dict)
    def _on_slam_command(self, command: dict):
        """Handle SLAM command from SlamView (mode change, reset, etc.)."""
        # SLAM commands go to dhruva-slam-node, not SangamIO
        # TODO: Implement sending commands to dhruva-slam-node when supported
        cmd_type = command.get('command', '')
        if cmd_type == 'set_mode':
            mode = command.get('mode', 'Idle')
            logger.info(f"SLAM mode change requested: {mode}")
            self.status_bar.showMessage(f"SLAM mode: {mode}", 2000)
        elif cmd_type == 'reset':
            logger.info("SLAM reset requested")
            self.status_bar.showMessage("SLAM reset requested", 2000)
        elif cmd_type == 'reset_pose':
            x = command.get('x', 0)
            y = command.get('y', 0)
            theta = command.get('theta', 0)
            logger.info(f"SLAM pose reset requested: ({x:.2f}, {y:.2f}, {theta:.2f})")
            self.status_bar.showMessage("SLAM pose reset requested", 2000)
        elif cmd_type == 'save_map':
            path = command.get('path', '')
            logger.info(f"SLAM save map requested: {path}")
            self.status_bar.showMessage(f"SLAM save map: {path}", 2000)

    @pyqtSlot(bool, str)
    def _on_slam_connection_status(self, connected: bool, message: str):
        """Handle SLAM connection status changes."""
        if connected:
            logger.info(f"SLAM connected: {message}")
        else:
            logger.warning(f"SLAM disconnected: {message}")

    @pyqtSlot(int)
    def _on_tab_changed(self, index: int):
        """Handle tab change to show/hide control panel."""
        # Tab 0 = Robot View (show control panel)
        # Tab 1 = SLAM (hide control panel - it has its own status panel)
        if index == 0:
            self.control_panel.show()
        else:
            self.control_panel.hide()

    @pyqtSlot(dict)
    def _on_sensor_data(self, data: dict):
        """Handle sensor data from telemetry thread."""
        # Update 3D robot view with new sensor values
        self.robot_3d_view.update_sensors(data)

        # Update control panel with sensor data
        self.control_panel.update_sensors(data)

        # Forward lidar scan data to 3D view overlay
        scan_data = data.get('scan')
        if scan_data:
            self.robot_3d_view.update_lidar_scan(scan_data)

        # Forward IMU data to 3D view for orientation
        imu_keys = ['gyro_x', 'gyro_y', 'gyro_z', 'tilt_x', 'tilt_y', 'tilt_z']
        if all(k in data for k in imu_keys):
            self.robot_3d_view.update_orientation(
                data['gyro_x'], data['gyro_y'], data['gyro_z'],
                data['tilt_x'], data['tilt_y'], data['tilt_z']
            )
        else:
            # Debug: log which keys are missing (once)
            if not hasattr(self, '_imu_missing_logged'):
                self._imu_missing_logged = True
                missing = [k for k in imu_keys if k not in data]
                logger.warning(f"IMU keys missing from data: {missing}. Available keys: {list(data.keys())}")

        # Update status bar with timestamp
        timestamp = data.get('_timestamp_us', 0)
        group = data.get('_group_id', 'unknown')
        self.status_bar.showMessage(f"{group} @ {timestamp}", 500)

    @pyqtSlot(dict)
    def _on_panel_command(self, command: dict):
        """Handle command from control panel."""
        self._send_command(command)

        cmd_type = command.get('type', '')
        component_id = command.get('id', '')
        action = command.get('action', {})
        action_type = action.get('type', '') if isinstance(action, dict) else ''

        # Handle ComponentControl commands
        if cmd_type == 'ComponentControl':
            # Handle lidar enable/disable
            if component_id == 'lidar':
                if action_type == 'Enable':
                    self.lidar_enabled = True
                    self.robot_3d_view.set_lidar_enabled(True)
                    self.status_bar.showMessage("Lidar ENABLED", 2000)
                elif action_type == 'Disable':
                    self.lidar_enabled = False
                    self.robot_3d_view.set_lidar_enabled(False)
                    self.status_bar.showMessage("Lidar DISABLED", 2000)
                return

            # Handle drive enable/disable
            if component_id == 'drive':
                if action_type == 'Enable':
                    self.motor_enabled = True
                    self.status_bar.showMessage("Drive ENABLED", 2000)
                elif action_type == 'Disable':
                    self.motor_enabled = False
                    self.status_bar.showMessage("Drive DISABLED", 2000)
                return

            # Handle actuator commands (vacuum, main_brush, side_brush)
            if component_id in ('vacuum', 'main_brush', 'side_brush'):
                config = action.get('config', {})
                speed = 0
                if 'speed' in config:
                    speed_val = config['speed']
                    speed = speed_val.get('U8', 0) if isinstance(speed_val, dict) else speed_val

                # Sync main window state with panel
                if component_id == 'vacuum':
                    self.vacuum_enabled = speed > 0
                elif component_id == 'side_brush':
                    self.side_brush_enabled = speed > 0
                elif component_id == 'main_brush':
                    self.main_brush_enabled = speed > 0

                # Map component_id back to actuator for 3D view
                actuator = component_id if component_id != 'main_brush' else 'brush'
                self.robot_3d_view.update_actuator(actuator, speed)
                self.status_bar.showMessage(f"{component_id}: {speed}%", 2000)
                return

    @pyqtSlot(bool, str)
    def _on_connection_status(self, connected: bool, message: str):
        """Handle connection status changes."""
        if connected:
            self.connection_status_label.setText(f"Connected to {self.robot_ip}")
            self.connection_status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.connection_status_label.setText("Disconnected")
            self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")

        self.status_bar.showMessage(message, 3000)
        logger.info(message)

    def eventFilter(self, obj, event):
        """Intercept arrow keys to ensure they control robot movement."""
        if event.type() == QEvent.KeyPress:
            key = event.key()
            if key in (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right):
                self.keyPressEvent(event)
                return True  # Event handled, don't propagate to widgets
        elif event.type() == QEvent.KeyRelease:
            key = event.key()
            if key in (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right):
                self.keyReleaseEvent(event)
                return True
        return super().eventFilter(obj, event)

    def keyPressEvent(self, event: QKeyEvent):
        """Handle key press events."""
        if event.key() == Qt.Key_L:
            # Toggle lidar
            self.lidar_enabled = not self.lidar_enabled
            self.control_panel.set_lidar_enabled(self.lidar_enabled)
            self.robot_3d_view.set_lidar_enabled(self.lidar_enabled)
            if self.lidar_enabled:
                self._send_command({
                    "type": "ComponentControl",
                    "id": "lidar",
                    "action": {"type": "Enable"}
                })
                self.status_bar.showMessage("Lidar ENABLED", 2000)
                logger.info("Lidar enabled")
            else:
                self._send_command({
                    "type": "ComponentControl",
                    "id": "lidar",
                    "action": {"type": "Disable"}
                })
                self.status_bar.showMessage("Lidar DISABLED", 2000)
                logger.info("Lidar disabled")
        elif event.key() == Qt.Key_S:
            # Toggle side brush
            self.side_brush_enabled = not self.side_brush_enabled
            speed = 100 if self.side_brush_enabled else 0
            if speed > 0:
                self._send_command({
                    "type": "ComponentControl",
                    "id": "side_brush",
                    "action": {"type": "Configure", "config": {"speed": {"U8": speed}}}
                })
            else:
                self._send_command({
                    "type": "ComponentControl",
                    "id": "side_brush",
                    "action": {"type": "Disable"}
                })
            self.control_panel.set_actuator_state("side_brush", self.side_brush_enabled)
            self.robot_3d_view.update_actuator("side_brush", speed)
            state = "ON" if self.side_brush_enabled else "OFF"
            self.status_bar.showMessage(f"Side Brush {state}", 2000)
            logger.info(f"Side brush {state}")
        elif event.key() == Qt.Key_B:
            # Toggle main brush (rolling brush)
            self.main_brush_enabled = not self.main_brush_enabled
            speed = 100 if self.main_brush_enabled else 0
            if speed > 0:
                self._send_command({
                    "type": "ComponentControl",
                    "id": "main_brush",
                    "action": {"type": "Configure", "config": {"speed": {"U8": speed}}}
                })
            else:
                self._send_command({
                    "type": "ComponentControl",
                    "id": "main_brush",
                    "action": {"type": "Disable"}
                })
            self.control_panel.set_actuator_state("brush", self.main_brush_enabled)
            self.robot_3d_view.update_actuator("brush", speed)
            state = "ON" if self.main_brush_enabled else "OFF"
            self.status_bar.showMessage(f"Main Brush {state}", 2000)
            logger.info(f"Main brush {state}")
        elif event.key() == Qt.Key_V:
            # Toggle vacuum pump
            self.vacuum_enabled = not self.vacuum_enabled
            speed = 100 if self.vacuum_enabled else 0
            if speed > 0:
                self._send_command({
                    "type": "ComponentControl",
                    "id": "vacuum",
                    "action": {"type": "Configure", "config": {"speed": {"U8": speed}}}
                })
            else:
                self._send_command({
                    "type": "ComponentControl",
                    "id": "vacuum",
                    "action": {"type": "Disable"}
                })
            self.control_panel.set_actuator_state("vacuum", self.vacuum_enabled)
            self.robot_3d_view.update_actuator("vacuum", speed)
            state = "ON" if self.vacuum_enabled else "OFF"
            self.status_bar.showMessage(f"Vacuum {state}", 2000)
            logger.info(f"Vacuum {state}")
        elif event.key() == Qt.Key_M:
            # Toggle wheel motor (drive)
            self.motor_enabled = not self.motor_enabled
            self.control_panel.set_motor_enabled(self.motor_enabled)
            if self.motor_enabled:
                self._send_command({
                    "type": "ComponentControl",
                    "id": "drive",
                    "action": {"type": "Enable"}
                })
                self.status_bar.showMessage("Drive ENABLED", 2000)
                logger.info("Drive enabled")
            else:
                self.pressed_keys.clear()
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0
                self._send_command({
                    "type": "ComponentControl",
                    "id": "drive",
                    "action": {"type": "Disable"}
                })
                self.control_panel.set_velocity(0.0, 0.0)
                self.status_bar.showMessage("Drive DISABLED", 2000)
                logger.info("Drive disabled")
        elif event.key() == Qt.Key_Escape:
            self.close()
        # Arrow keys for drive control (only when motor is enabled)
        # Supports key combinations: Up+Left, Up+Right, Down+Left, Down+Right
        elif event.key() in (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right):
            if not event.isAutoRepeat() and self.motor_enabled:
                self.pressed_keys.add(event.key())
                self._update_velocity_from_keys()
        elif event.key() == Qt.Key_Space:
            # Emergency stop - send Reset command to immediately halt
            self.pressed_keys.clear()
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self._send_command({
                "type": "ComponentControl",
                "id": "drive",
                "action": {"type": "Reset"}
            })
            self.control_panel.set_velocity(0.0, 0.0)
            self.status_bar.showMessage("E-STOP", 2000)
            logger.info("Emergency stop triggered")
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent):
        """Handle key release events for drive control."""
        if event.key() in (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right):
            if not event.isAutoRepeat():
                self.pressed_keys.discard(event.key())
                self._update_velocity_from_keys()
        else:
            super().keyReleaseEvent(event)

    def _update_velocity_from_keys(self):
        """Update velocity based on currently pressed arrow keys.

        Supports key combinations for diagonal movement:
        - Up/Down controls linear velocity (forward/backward)
        - Left/Right controls angular velocity (turn left/right)
        - Combinations like Up+Left move forward while turning left

        Speed is controlled by the DriveControl widget's speed slider.
        """
        # Get speed from DriveControl's slider setting
        linear_speed = self.control_panel.drive_control._get_linear_speed()
        angular_speed = self.control_panel.drive_control._get_angular_speed()

        # Calculate linear velocity from Up/Down keys
        linear = 0.0
        if Qt.Key_Up in self.pressed_keys:
            linear += linear_speed
        if Qt.Key_Down in self.pressed_keys:
            linear -= linear_speed

        # Calculate angular velocity from Left/Right keys
        angular = 0.0
        if Qt.Key_Left in self.pressed_keys:
            angular += angular_speed
        if Qt.Key_Right in self.pressed_keys:
            angular -= angular_speed

        self.linear_velocity = linear
        self.angular_velocity = angular
        self._send_velocity()

    def _send_velocity(self):
        """Send current velocity to robot using ComponentControl protocol."""
        command = {
            "type": "ComponentControl",
            "id": "drive",
            "action": {
                "type": "Configure",
                "config": {
                    "linear": {"F32": self.linear_velocity},
                    "angular": {"F32": self.angular_velocity}
                }
            }
        }
        self._send_command(command)
        self.control_panel.set_velocity(self.linear_velocity, self.angular_velocity)
        logger.debug(f"Velocity: linear={self.linear_velocity:.2f}, angular={self.angular_velocity:.2f}")

    def _send_command(self, command: dict):
        """Send a command to the robot via existing telemetry connection."""
        if self.telemetry_thread:
            if self.telemetry_thread.send_command(command):
                logger.debug(f"Command sent: {command}")
            else:
                self.status_bar.showMessage("Command failed: not connected", 3000)
        else:
            self.status_bar.showMessage("Command failed: no connection", 3000)

    def closeEvent(self, event):
        """Handle window close event."""
        if self.telemetry_thread:
            self.telemetry_thread.stop()
        if self.odometry_thread:
            self.odometry_thread.stop()
        event.accept()
