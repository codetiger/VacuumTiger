"""
Main window for Drishti robot visualization application.

Single full-screen widget showing robot diagram with sensor overlays.
"""

from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QLabel, QStatusBar)
from PyQt5.QtGui import QFont
import logging

from ui.threads.telemetry_thread import TelemetryThread
from ui.widgets.robot_diagram import RobotDiagram

logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    """Main application window with full-screen robot diagram."""

    def __init__(self, robot_ip="192.168.68.101", port=5555):
        super().__init__()

        self.robot_ip = robot_ip
        self.port = port
        self.telemetry_thread = None

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

        # Robot diagram (full screen)
        self.robot_diagram = RobotDiagram()
        main_layout.addWidget(self.robot_diagram, 1)  # Stretch factor 1

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
            self.telemetry_thread = TelemetryThread(self.robot_ip, self.port)
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

        # Update status bar with timestamp
        timestamp = data.get('_timestamp_us', 0)
        group = data.get('_group_id', 'unknown')
        self.status_bar.showMessage(f"{group} @ {timestamp}", 500)

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

    def closeEvent(self, event):
        """Handle window close event."""
        if self.telemetry_thread:
            self.telemetry_thread.stop()
        event.accept()
