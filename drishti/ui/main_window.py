"""
Main window for Drishti robot visualization application.

Single full-screen widget showing robot diagram with sensor overlays.
"""

from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QStatusBar)
from PyQt5.QtGui import QFont, QKeyEvent
import logging

from ui.threads.telemetry_thread import TelemetryThread
from ui.widgets.robot_diagram import RobotDiagram
from ui.widgets.control_panel import ControlPanel

logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    """Main application window with full-screen robot diagram."""

    def __init__(self, robot_ip="192.168.68.101", port=5555, log_raw_packets=False):
        super().__init__()

        self.robot_ip = robot_ip
        self.port = port
        self.log_raw_packets = log_raw_packets
        self.telemetry_thread = None
        self.lidar_enabled = False  # Lidar starts disabled
        self.side_brush_enabled = False
        self.main_brush_enabled = False
        self.vacuum_enabled = False

        # Drive control state
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_step = 0.5  # m/s
        self.angular_step = 0.5  # rad/s
        self.motor_enabled = False

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

        # Content area: robot diagram (80%) + control panel (20%)
        content_layout = QHBoxLayout()
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)

        # Robot diagram (left side, 80%)
        self.robot_diagram = RobotDiagram()
        content_layout.addWidget(self.robot_diagram, 4)  # Stretch factor 4

        # Control panel (right side, 20%)
        self.control_panel = ControlPanel()
        self.control_panel.setStyleSheet("background-color: #e8e8e8;")
        self.control_panel.command_requested.connect(self._on_panel_command)
        content_layout.addWidget(self.control_panel, 1)  # Stretch factor 1

        main_layout.addLayout(content_layout, 1)

        # Status bar
        self.status_bar = QStatusBar()
        self.status_bar.setStyleSheet("background-color: #e0e0e0;")
        self.setStatusBar(self.status_bar)
        self.connection_status_label = QLabel("Connecting...")
        self.connection_status_label.setStyleSheet("color: #666; font-weight: bold;")
        self.status_bar.addPermanentWidget(self.connection_status_label)

        # Connect to robot
        self._start_telemetry()

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

    @pyqtSlot(dict)
    def _on_sensor_data(self, data: dict):
        """Handle sensor data from telemetry thread."""
        # Update robot diagram with new sensor values
        self.robot_diagram.update_sensors(data)

        # Update control panel with sensor data
        self.control_panel.update_sensors(data)

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
                    self.status_bar.showMessage("Lidar ENABLED", 2000)
                elif action_type == 'Disable':
                    self.lidar_enabled = False
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

                # Map component_id back to actuator for robot diagram
                actuator = component_id if component_id != 'main_brush' else 'brush'
                self.robot_diagram.update_actuator(actuator, speed)
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

    def keyPressEvent(self, event: QKeyEvent):
        """Handle key press events."""
        if event.key() == Qt.Key_L:
            # Toggle lidar
            self.lidar_enabled = not self.lidar_enabled
            self.control_panel.set_lidar_enabled(self.lidar_enabled)
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
                self._send_command({
                    "type": "ComponentControl",
                    "id": "drive",
                    "action": {"type": "Disable"}
                })
                self.status_bar.showMessage("Drive DISABLED", 2000)
                logger.info("Drive disabled")
        elif event.key() == Qt.Key_Escape:
            self.close()
        # Arrow keys for drive control (only when motor is enabled)
        elif event.key() == Qt.Key_Up:
            if not event.isAutoRepeat() and self.motor_enabled:
                self.linear_velocity = self.linear_step
                self.angular_velocity = 0.0
                self._send_velocity()
        elif event.key() == Qt.Key_Down:
            if not event.isAutoRepeat() and self.motor_enabled:
                self.linear_velocity = -self.linear_step
                self.angular_velocity = 0.0
                self._send_velocity()
        elif event.key() == Qt.Key_Left:
            if not event.isAutoRepeat() and self.motor_enabled:
                self.linear_velocity = 0.0
                self.angular_velocity = self.angular_step
                self._send_velocity()
        elif event.key() == Qt.Key_Right:
            if not event.isAutoRepeat() and self.motor_enabled:
                self.linear_velocity = 0.0
                self.angular_velocity = -self.angular_step
                self._send_velocity()
        elif event.key() == Qt.Key_Space:
            # Emergency stop
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self._send_velocity()
            self.status_bar.showMessage("STOP", 2000)
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent):
        """Handle key release events for drive control."""
        if event.key() in (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right):
            if not event.isAutoRepeat():
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0
                self._send_velocity()
        else:
            super().keyReleaseEvent(event)

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
        event.accept()
