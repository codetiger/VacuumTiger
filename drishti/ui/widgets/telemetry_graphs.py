"""
Telemetry graphs widget for real-time data visualization.

Displays battery level, encoder ticks, and connection quality over time.
"""

import pyqtgraph as pg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from collections import deque
import time


class TelemetryGraphs(QWidget):
    """Real-time telemetry graphs (30-second window)"""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Data buffers (30 seconds at 30Hz = 900 samples)
        self.max_samples = 900
        self.time_data = deque(maxlen=self.max_samples)
        self.battery_data = deque(maxlen=self.max_samples)
        self.encoder_left_data = deque(maxlen=self.max_samples)
        self.encoder_right_data = deque(maxlen=self.max_samples)
        self.quality_data = deque(maxlen=self.max_samples)

        self.start_time = time.time()

        # Create layout
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Title
        title = QLabel("Real-time Telemetry (30s window)")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: white; font-size: 11pt; font-weight: bold;")
        layout.addWidget(title)

        # Create horizontal layout for 3 graphs
        graphs_layout = QHBoxLayout()

        # Battery graph
        self.battery_plot = pg.PlotWidget(title="Battery Level (%)")
        self.battery_plot.setBackground('#2b2b2b')
        self.battery_plot.setYRange(0, 100)
        self.battery_plot.showGrid(x=True, y=True, alpha=0.3)
        self.battery_curve = self.battery_plot.plot(pen=pg.mkPen('g', width=2))
        graphs_layout.addWidget(self.battery_plot)

        # Encoder graph
        self.encoder_plot = pg.PlotWidget(title="Encoder Ticks")
        self.encoder_plot.setBackground('#2b2b2b')
        self.encoder_plot.showGrid(x=True, y=True, alpha=0.3)
        self.encoder_left_curve = self.encoder_plot.plot(pen=pg.mkPen('c', width=2), name='Left')
        self.encoder_right_curve = self.encoder_plot.plot(pen=pg.mkPen('m', width=2), name='Right')
        self.encoder_plot.addLegend()
        graphs_layout.addWidget(self.encoder_plot)

        # Connection quality graph
        self.quality_plot = pg.PlotWidget(title="Connection Quality (%)")
        self.quality_plot.setBackground('#2b2b2b')
        self.quality_plot.setYRange(0, 100)
        self.quality_plot.showGrid(x=True, y=True, alpha=0.3)
        self.quality_curve = self.quality_plot.plot(pen=pg.mkPen('y', width=2))
        graphs_layout.addWidget(self.quality_plot)

        layout.addLayout(graphs_layout)

        # Downsample counter (update UI at 30Hz max, even if data comes at 500Hz)
        self.update_counter = 0
        self.last_battery = None
        self.last_encoders = (None, None)
        self.last_quality = None

    def update_sensor_data(self, sensor_data):
        """
        Update from SensorUpdate (downsampled to 30Hz)

        Args:
            sensor_data: List [timestamp, battery_level, is_charging, ...]
        """
        if not sensor_data or len(sensor_data) < 11:
            return

        # Downsample: Only update every 16th sample (500Hz / 30Hz ≈ 16)
        self.update_counter += 1
        if self.update_counter < 16:
            return
        self.update_counter = 0

        # Extract data
        battery_level = sensor_data[1] if sensor_data[1] is not None else 0
        encoder_left = sensor_data[9] if sensor_data[9] is not None else 0
        encoder_right = sensor_data[10] if sensor_data[10] is not None else 0

        # Time since start
        current_time = time.time() - self.start_time

        # Only add if values changed (reduce noise)
        if battery_level != self.last_battery:
            self.time_data.append(current_time)
            self.battery_data.append(battery_level)
            self.last_battery = battery_level
            self._update_battery_plot()

        if (encoder_left, encoder_right) != self.last_encoders:
            self.encoder_left_data.append(encoder_left)
            self.encoder_right_data.append(encoder_right)
            self.last_encoders = (encoder_left, encoder_right)
            self._update_encoder_plot()

    def update_connection_quality(self, quality_data):
        """
        Update from ConnectionQuality message

        Args:
            quality_data: List [timestamp, rx_packets, tx_packets, success_rate, telemetry_fresh]
        """
        if not quality_data or len(quality_data) < 5:
            return

        success_rate = quality_data[3]  # 0.0 - 1.0
        success_percent = success_rate * 100

        current_time = time.time() - self.start_time

        if success_percent != self.last_quality:
            self.time_data.append(current_time)
            self.quality_data.append(success_percent)
            self.last_quality = success_percent
            self._update_quality_plot()

    def _update_battery_plot(self):
        """Update battery graph"""
        if len(self.time_data) > 0 and len(self.battery_data) > 0:
            # Take last N samples that match battery data length
            time_subset = list(self.time_data)[-len(self.battery_data):]
            self.battery_curve.setData(time_subset, list(self.battery_data))

    def _update_encoder_plot(self):
        """Update encoder graph"""
        if len(self.encoder_left_data) > 0:
            # Use range for X axis (sample count)
            x_data = list(range(len(self.encoder_left_data)))
            self.encoder_left_curve.setData(x_data, list(self.encoder_left_data))
            self.encoder_right_curve.setData(x_data, list(self.encoder_right_data))

    def _update_quality_plot(self):
        """Update connection quality graph"""
        if len(self.time_data) > 0 and len(self.quality_data) > 0:
            time_subset = list(self.time_data)[-len(self.quality_data):]
            self.quality_curve.setData(time_subset, list(self.quality_data))
