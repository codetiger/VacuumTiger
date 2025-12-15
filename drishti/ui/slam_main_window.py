"""
SLAM main window for Drishti visualization application.

Connects to DhruvaSLAM daemon for:
- Real-time map visualization
- Robot pose and trajectory
- Lidar scan overlay
- Mapping control (start/stop/save)
- Map management (list/enable/rename/delete)
"""

import math
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QStatusBar
)
from PyQt5.QtGui import QFont
import logging

from ui.threads.slam_thread import SlamThread
from ui.widgets.slam_view import SlamView
from ui.widgets.slam_control_panel import SlamControlPanel
from ui.widgets.navigation_panel import NavigationPanel

logger = logging.getLogger(__name__)


class SlamMainWindow(QMainWindow):
    """Main application window for SLAM mode."""

    def __init__(self, slam_host="localhost", slam_port=5557, command_port=5558):
        super().__init__()

        self.slam_host = slam_host
        self.slam_port = slam_port
        self.command_port = command_port
        self.slam_thread = None

        # Initialize UI
        self.setWindowTitle(f"Drishti SLAM - {slam_host}:{slam_port}")
        self.setGeometry(100, 100, 1200, 800)
        self.setStyleSheet("background-color: #1a1a1a;")

        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)

        # Title bar
        title_bar = self._create_title_bar()
        main_layout.addWidget(title_bar)

        # Content area: SLAM view (left) + control panel (right)
        content_layout = QHBoxLayout()
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)

        # SLAM view
        self.slam_view = SlamView()
        content_layout.addWidget(self.slam_view, 1)  # Stretch factor 1 (expands)

        # Right side panel with control panel and navigation panel
        right_panel = QWidget()
        right_panel.setStyleSheet("background-color: #2d2d2d;")
        right_panel.setFixedWidth(280)
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)

        # Control panel (top of right side)
        self.control_panel = SlamControlPanel()
        self.control_panel.setStyleSheet("background-color: #2d2d2d;")
        self._connect_control_panel_signals()
        right_layout.addWidget(self.control_panel, 1)

        # Navigation panel (bottom of right side)
        self.navigation_panel = NavigationPanel()
        self.navigation_panel.setStyleSheet("background-color: #2d2d2d;")
        self._connect_navigation_signals()
        right_layout.addWidget(self.navigation_panel, 0)

        content_layout.addWidget(right_panel, 0)  # Stretch factor 0 (fixed)

        main_layout.addLayout(content_layout, 1)

        # Status bar
        self.status_bar = QStatusBar()
        self.status_bar.setStyleSheet("background-color: #252525; color: #aaa;")
        self.setStatusBar(self.status_bar)
        self.connection_status_label = QLabel("Connecting...")
        self.connection_status_label.setStyleSheet("color: #666; font-weight: bold;")
        self.status_bar.addPermanentWidget(self.connection_status_label)

        # Connect to SLAM daemon
        self._start_slam_connection()

    def _create_title_bar(self):
        """Create application title bar."""
        title_widget = QWidget()
        title_widget.setStyleSheet("background-color: #2d2d2d; padding: 8px;")
        title_widget.setFixedHeight(50)
        layout = QVBoxLayout(title_widget)
        layout.setContentsMargins(15, 5, 15, 5)

        title_label = QLabel("Drishti SLAM Visualization")
        title_label.setFont(QFont("Arial", 14, QFont.Bold))
        title_label.setStyleSheet("color: white;")
        layout.addWidget(title_label)

        return title_widget

    def _connect_control_panel_signals(self):
        """Connect control panel signals to SLAM commands."""
        self.control_panel.start_mapping_requested.connect(self._on_start_mapping)
        self.control_panel.stop_mapping_requested.connect(self._on_stop_mapping)
        self.control_panel.clear_map_requested.connect(self._on_clear_map)
        self.control_panel.emergency_stop_requested.connect(self._on_emergency_stop)
        self.control_panel.enable_map_requested.connect(self._on_enable_map)
        self.control_panel.rename_map_requested.connect(self._on_rename_map)
        self.control_panel.delete_map_requested.connect(self._on_delete_map)

    def _connect_navigation_signals(self):
        """Connect navigation-related signals."""
        # SlamView Ctrl+Click -> set navigation goal
        self.slam_view.goal_selected.connect(self._on_goal_selected)

        # NavigationPanel cancel button -> cancel navigation
        self.navigation_panel.cancel_navigation.connect(self._on_cancel_navigation)

    def _start_slam_connection(self):
        """Start SLAM thread to receive data."""
        try:
            self.status_bar.showMessage("Connecting to DhruvaSLAM...")

            # Create and start SLAM thread
            self.slam_thread = SlamThread(
                self.slam_host, self.slam_port, self.command_port
            )

            # Connect data signals
            self.slam_thread.robot_status_received.connect(self._on_robot_status)
            self.slam_thread.sensor_status_received.connect(self._on_sensor_status)
            self.slam_thread.current_map_received.connect(self._on_current_map)
            self.slam_thread.map_list_received.connect(self._on_map_list)
            self.slam_thread.navigation_status_received.connect(self._on_navigation_status)

            # Connect status signals
            self.slam_thread.connection_status.connect(self._on_connection_status)
            self.slam_thread.command_response_received.connect(self._on_command_response)

            self.slam_thread.start()

        except Exception as e:
            logger.error(f"Failed to start SLAM connection: {e}")
            self.connection_status_label.setText("Connection Failed")
            self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
            self.status_bar.showMessage(f"Error: {e}")

    # ==================== Data Handlers ====================

    @pyqtSlot(dict)
    def _on_robot_status(self, data: dict):
        """Handle robot status data."""
        # Update control panel
        self.control_panel.update_robot_status(data)

        # Update SLAM view with pose
        pose = data.get('pose', {})
        x = pose.get('x', 0.0)
        y = pose.get('y', 0.0)
        theta = pose.get('theta', 0.0)
        self.slam_view.update_pose(x, y, theta)

        # Update status bar
        state = data.get('state', 'Unknown')
        battery = data.get('battery_percent', 0.0)
        self.status_bar.showMessage(f"State: {state} | Battery: {battery:.0f}%")

    @pyqtSlot(dict)
    def _on_sensor_status(self, data: dict):
        """Handle sensor status data."""
        # Update SLAM view with lidar scan
        lidar = data.get('lidar')
        if lidar and lidar.get('ranges'):
            # Convert polar to cartesian points in robot frame
            ranges = lidar['ranges']
            angle_min = lidar.get('angle_min', -math.pi)
            angle_increment = lidar.get('angle_increment', 2 * math.pi / len(ranges))

            # Convert polar to cartesian in robot frame
            # The angles from DhruvaSLAM have lidar offset applied but may have
            # different zero-reference than expected. Apply 90° rotation to align
            # with how the map was built.
            # Rotate by -π/2: x' = r*cos(θ - π/2) = r*sin(θ), y' = r*sin(θ - π/2) = -r*cos(θ)
            points = []
            for i, r in enumerate(ranges):
                if r > 0.01 and r < 30.0:  # Valid range
                    angle = angle_min + i * angle_increment
                    x = r * math.sin(angle)   # Rotated by -90°
                    y = -r * math.cos(angle)  # Rotated by -90°
                    points.append([x, y])

            self.slam_view.update_scan({'points': points})

    @pyqtSlot(dict)
    def _on_current_map(self, data: dict):
        """Handle current map data."""
        # Convert to format expected by SlamView.update_map()
        map_data = {
            'resolution': data.get('resolution', 0.05),
            'width': data.get('width', 0),
            'height': data.get('height', 0),
            'origin_x': data.get('origin_x', 0.0),
            'origin_y': data.get('origin_y', 0.0),
            'cells': data.get('cells', b''),
        }
        self.slam_view.update_map(map_data)

    @pyqtSlot(dict)
    def _on_map_list(self, data: dict):
        """Handle map list data."""
        self.control_panel.update_map_list(data)

    @pyqtSlot(dict)
    def _on_navigation_status(self, data: dict):
        """Handle navigation status data."""
        # Update SLAM view with path and goal visualization
        self.slam_view.update_navigation(data)

        # Update navigation panel with status
        self.navigation_panel.update_navigation_status(data)

    @pyqtSlot(bool, str)
    def _on_connection_status(self, connected: bool, message: str):
        """Handle connection status changes."""
        self.control_panel.set_connection_status(connected, message)

        if connected:
            self.connection_status_label.setText(f"Connected to {self.slam_host}")
            self.connection_status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.connection_status_label.setText("Disconnected")
            self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")

        self.status_bar.showMessage(message, 3000)
        logger.info(message)

    @pyqtSlot(dict)
    def _on_command_response(self, response: dict):
        """Handle command response."""
        success = response.get('success', False)
        error = response.get('error_message', '')

        if success:
            data = response.get('data', {})
            if 'map_id' in data:
                self.status_bar.showMessage(f"Mapping started: {data['map_id']}", 3000)
            elif 'saved' in data:
                if data['saved']:
                    self.status_bar.showMessage(f"Map saved: {data.get('area_m2', 0):.1f} m²", 3000)
                else:
                    self.status_bar.showMessage("Map discarded", 3000)
            else:
                self.status_bar.showMessage("Command successful", 2000)
        else:
            self.status_bar.showMessage(f"Command failed: {error}", 5000)
            logger.error(f"Command failed: {error}")

    # ==================== Command Handlers ====================

    @pyqtSlot(str)
    def _on_start_mapping(self, map_name: str):
        """Handle start mapping request."""
        if self.slam_thread:
            request_id = self.slam_thread.start_mapping(map_name)
            if request_id:
                self.status_bar.showMessage("Starting mapping...", 2000)
            else:
                self.status_bar.showMessage("Failed to send command", 3000)

    @pyqtSlot(bool)
    def _on_stop_mapping(self, save: bool):
        """Handle stop mapping request."""
        if self.slam_thread:
            request_id = self.slam_thread.stop_mapping(save)
            if request_id:
                action = "Saving" if save else "Discarding"
                self.status_bar.showMessage(f"{action} map...", 2000)
            else:
                self.status_bar.showMessage("Failed to send command", 3000)

    @pyqtSlot()
    def _on_clear_map(self):
        """Handle clear map request."""
        if self.slam_thread:
            request_id = self.slam_thread.clear_map()
            if request_id:
                self.status_bar.showMessage("Clearing map...", 2000)
                self.slam_view.clear_trajectory()
            else:
                self.status_bar.showMessage("Failed to send command", 3000)

    @pyqtSlot()
    def _on_emergency_stop(self):
        """Handle emergency stop request - immediate stop of all robot motion."""
        if self.slam_thread:
            request_id = self.slam_thread.emergency_stop()
            if request_id:
                self.status_bar.showMessage("EMERGENCY STOP activated!", 5000)
                logger.warning("Emergency stop activated by user")
            else:
                self.status_bar.showMessage("Failed to send emergency stop!", 3000)
                logger.error("Failed to send emergency stop command")

    @pyqtSlot(str)
    def _on_enable_map(self, map_id: str):
        """Handle enable map request."""
        if self.slam_thread:
            request_id = self.slam_thread.enable_map(map_id)
            if request_id:
                self.status_bar.showMessage(f"Enabling map: {map_id}", 2000)
            else:
                self.status_bar.showMessage("Failed to send command", 3000)

    @pyqtSlot(str, str)
    def _on_rename_map(self, map_id: str, new_name: str):
        """Handle rename map request."""
        if self.slam_thread:
            request_id = self.slam_thread.rename_map(map_id, new_name)
            if request_id:
                self.status_bar.showMessage(f"Renaming map to: {new_name}", 2000)
            else:
                self.status_bar.showMessage("Failed to send command", 3000)

    @pyqtSlot(str)
    def _on_delete_map(self, map_id: str):
        """Handle delete map request."""
        if self.slam_thread:
            request_id = self.slam_thread.delete_map(map_id)
            if request_id:
                self.status_bar.showMessage(f"Deleting map: {map_id}", 2000)
            else:
                self.status_bar.showMessage("Failed to send command", 3000)

    # ==================== Navigation Handlers ====================

    @pyqtSlot(float, float, float)
    def _on_goal_selected(self, x: float, y: float, theta: float):
        """Handle goal selection from SLAM view (Ctrl+Click)."""
        if self.slam_thread:
            description = f"({x:.2f}, {y:.2f})"
            request_id = self.slam_thread.set_goal(x, y, theta, description)
            if request_id:
                self.status_bar.showMessage(f"Setting goal: {description}", 2000)
                logger.info(f"Goal set: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
            else:
                self.status_bar.showMessage("Failed to send goal command", 3000)
        else:
            self.status_bar.showMessage("Not connected to SLAM daemon", 3000)

    @pyqtSlot()
    def _on_cancel_navigation(self):
        """Handle cancel navigation request."""
        if self.slam_thread:
            request_id = self.slam_thread.cancel_goal()
            if request_id:
                self.status_bar.showMessage("Cancelling navigation...", 2000)
                logger.info("Navigation cancelled by user")
                # Clear navigation visualization
                self.slam_view.clear_navigation()
            else:
                self.status_bar.showMessage("Failed to send cancel command", 3000)
        else:
            self.status_bar.showMessage("Not connected to SLAM daemon", 3000)

    def closeEvent(self, event):
        """Handle window close event."""
        if self.slam_thread:
            self.slam_thread.stop()
        event.accept()
