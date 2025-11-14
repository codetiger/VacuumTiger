"""
Main window for Drishti robot visualization application.

Integrates all widgets into a 4-panel layout with telemetry thread.
"""

from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                              QGridLayout, QLabel, QStatusBar, QMessageBox)
from PyQt5.QtGui import QFont
import sys
import os

# Add parent directory to path to import drishti
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drishti import RobotClient
from ui.threads.telemetry_thread import TelemetryThread
from ui.widgets.robot_outline_widget import RobotOutlineWidget
from ui.widgets.lidar_widget import LidarWidget
from ui.widgets.control_panel import ControlPanel
from ui.widgets.telemetry_graphs import TelemetryGraphs


class MainWindow(QMainWindow):
    """Main application window"""

    def __init__(self, robot_ip="192.168.68.101", pub_port=5555, cmd_port=5556):
        super().__init__()

        self.robot_ip = robot_ip
        self.robot_client = None
        self.telemetry_thread = None

        # Initialize UI
        self.setWindowTitle(f"Drishti - Robot Monitor ({robot_ip})")
        self.setGeometry(100, 100, 1400, 900)
        self.setStyleSheet("background-color: #1e1e1e;")

        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Title bar
        title_bar = self._create_title_bar()
        main_layout.addWidget(title_bar)

        # Create 4-panel layout
        content_layout = QGridLayout()

        # Top-left: Robot outline
        self.robot_widget = RobotOutlineWidget()
        content_layout.addWidget(self.robot_widget, 0, 0)

        # Top-right: Control panel
        self.control_panel = ControlPanel()
        content_layout.addWidget(self.control_panel, 0, 1)

        # Middle: Lidar visualization (spans 2 columns)
        self.lidar_widget = LidarWidget()
        content_layout.addWidget(self.lidar_widget, 1, 0, 1, 2)

        # Bottom: Telemetry graphs (spans 2 columns)
        self.telemetry_graphs = TelemetryGraphs()
        content_layout.addWidget(self.telemetry_graphs, 2, 0, 1, 2)

        # Set row/column stretch
        content_layout.setRowStretch(0, 2)  # Top row
        content_layout.setRowStretch(1, 3)  # Middle row (lidar)
        content_layout.setRowStretch(2, 2)  # Bottom row (graphs)
        content_layout.setColumnStretch(0, 2)  # Left column
        content_layout.setColumnStretch(1, 1)  # Right column

        main_layout.addLayout(content_layout)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.connection_status_label = QLabel("Not Connected")
        self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
        self.status_bar.addPermanentWidget(self.connection_status_label)

        # Connect control panel signals
        self._connect_control_signals()

        # Connect to robot
        self._connect_to_robot(robot_ip, pub_port, cmd_port)

    def _create_title_bar(self):
        """Create application title bar"""
        title_widget = QWidget()
        title_widget.setStyleSheet("background-color: #2d2d2d; padding: 10px;")
        layout = QHBoxLayout(title_widget)

        title_label = QLabel("🤖 Drishti - Robot Vacuum Visualization")
        title_label.setFont(QFont("Arial", 18, QFont.Bold))
        title_label.setStyleSheet("color: white;")
        layout.addWidget(title_label)

        layout.addStretch()

        subtitle = QLabel("Real-time telemetry • Lidar visualization • Manual control")
        subtitle.setFont(QFont("Arial", 10))
        subtitle.setStyleSheet("color: #aaa;")
        layout.addWidget(subtitle)

        return title_widget

    def _connect_to_robot(self, robot_ip, pub_port, cmd_port):
        """Connect to robot and start telemetry thread"""
        try:
            self.status_bar.showMessage("Connecting to robot...")

            # Create robot client
            self.robot_client = RobotClient(robot_ip, pub_port, cmd_port)

            # Create and start telemetry thread
            self.telemetry_thread = TelemetryThread(self.robot_client)
            self.telemetry_thread.sensor_update.connect(self._on_sensor_update)
            self.telemetry_thread.connection_quality.connect(self._on_connection_quality)
            self.telemetry_thread.lidar_scan.connect(self._on_lidar_scan)
            self.telemetry_thread.connection_error.connect(self._on_connection_error)
            self.telemetry_thread.start()

            self.connection_status_label.setText(f"Connected to {robot_ip}")
            self.connection_status_label.setStyleSheet("color: green; font-weight: bold;")
            self.status_bar.showMessage(f"Connected to robot at {robot_ip}", 3000)

        except Exception as e:
            self.connection_status_label.setText("Connection Failed")
            self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
            self.status_bar.showMessage(f"Connection error: {e}")
            QMessageBox.critical(self, "Connection Error",
                                 f"Failed to connect to robot at {robot_ip}:\n{e}")

    def _connect_control_signals(self):
        """Connect control panel signals to command methods"""
        self.control_panel.velocity_command.connect(self._send_velocity_command)
        self.control_panel.side_brush_command.connect(self._send_side_brush_command)
        self.control_panel.main_brush_command.connect(self._send_main_brush_command)
        self.control_panel.air_pump_command.connect(self._send_air_pump_command)
        self.control_panel.lidar_enable_command.connect(self._send_lidar_enable_command)
        self.control_panel.lidar_disable_command.connect(self._send_lidar_disable_command)
        self.control_panel.emergency_stop_command.connect(self._send_emergency_stop)

    @pyqtSlot(object)
    def _on_sensor_update(self, sensor_data):
        """Handle SensorUpdate from telemetry thread (list from MessagePack)"""
        # Update robot outline widget
        self.robot_widget.update_sensor_data(sensor_data)

        # Update telemetry graphs
        self.telemetry_graphs.update_sensor_data(sensor_data)

        # Update brush speeds in robot widget
        side_speed = self.control_panel.side_brush_slider.value()
        main_speed = self.control_panel.main_brush_slider.value()
        self.robot_widget.set_brush_speeds(side_speed, main_speed)

    @pyqtSlot(object)
    def _on_connection_quality(self, quality_data):
        """Handle ConnectionQuality from telemetry thread (list from MessagePack)"""
        # Update telemetry graphs
        self.telemetry_graphs.update_connection_quality(quality_data)

        # Update status bar
        if len(quality_data) >= 4:
            success_rate = quality_data[3] * 100
            self.status_bar.showMessage(f"Connection: {success_rate:.1f}% | "
                                        f"RX: {quality_data[1]} | TX: {quality_data[2]}", 2000)

    @pyqtSlot(object)
    def _on_lidar_scan(self, scan_data):
        """Handle LidarScan from telemetry thread (dict for lidar)"""
        # Update lidar widget
        self.lidar_widget.update_lidar_scan(scan_data)

        # Update lidar angle in robot widget (if scan has points)
        if scan_data and 'points' in scan_data and len(scan_data['points']) > 0:
            # Use angle of last point as indicator
            last_point = scan_data['points'][-1]
            self.robot_widget.update_lidar_angle(last_point['angle'])

    @pyqtSlot(str)
    def _on_connection_error(self, error_msg):
        """Handle connection errors"""
        self.status_bar.showMessage(f"Error: {error_msg}", 5000)

    def _send_velocity_command(self, left_ticks, right_ticks):
        """Send wheel velocity command"""
        if self.robot_client:
            self.robot_client.set_wheel_velocity(left_ticks, right_ticks)
            self.status_bar.showMessage(f"Velocity: L={left_ticks:.0f}, R={right_ticks:.0f}", 1000)

    def _send_side_brush_command(self, speed):
        """Send side brush command"""
        if self.robot_client:
            self.robot_client.set_side_brush_speed(speed)
            self.status_bar.showMessage(f"Side brush: {speed}%", 1000)

    def _send_main_brush_command(self, speed):
        """Send main brush command"""
        if self.robot_client:
            self.robot_client.set_main_brush_speed(speed)
            self.status_bar.showMessage(f"Main brush: {speed}%", 1000)

    def _send_air_pump_command(self, speed):
        """Send air pump command"""
        if self.robot_client:
            self.robot_client.set_air_pump_speed(speed)
            self.status_bar.showMessage(f"Air pump: {speed}%", 1000)

    def _send_lidar_enable_command(self, pwm):
        """Send lidar enable command"""
        if self.robot_client:
            self.robot_client.enable_lidar(pwm)
            self.status_bar.showMessage(f"Lidar enabled at {pwm}% PWM", 2000)

    def _send_lidar_disable_command(self):
        """Send lidar disable command"""
        if self.robot_client:
            self.robot_client.disable_lidar()
            self.status_bar.showMessage("Lidar disabled", 2000)

    def _send_emergency_stop(self):
        """Send emergency stop command"""
        if self.robot_client:
            self.robot_client.emergency_stop()
            self.status_bar.showMessage("⚠ EMERGENCY STOP ACTIVATED ⚠", 5000)

    def closeEvent(self, event):
        """Handle window close event"""
        if self.telemetry_thread:
            self.telemetry_thread.stop()

        if self.robot_client:
            self.robot_client.close()

        event.accept()
