"""
Telemetry reception thread for Drishti UI.

Runs RobotClient.receive_telemetry() in a background thread and emits
Qt signals when data arrives for thread-safe UI updates.
"""

from PyQt5.QtCore import QThread, pyqtSignal
import logging

logger = logging.getLogger(__name__)


class TelemetryThread(QThread):
    """Background thread for receiving robot telemetry data"""

    # Signals emitted when data arrives
    # NOTE: MessagePack serializes Rust structs as lists (positional arrays), not dicts
    sensor_update = pyqtSignal(object)  # SensorUpdate data (list from MessagePack)
    connection_quality = pyqtSignal(object)  # ConnectionQuality data (list from MessagePack)
    lidar_scan = pyqtSignal(object)  # LidarScan data (dict for lidar)
    connection_error = pyqtSignal(str)  # Connection error message

    def __init__(self, robot_client):
        """
        Initialize telemetry thread

        Args:
            robot_client: RobotClient instance (from drishti.py)
        """
        super().__init__()
        self.robot_client = robot_client
        self._running = True

    def run(self):
        """Main thread loop - receives telemetry and emits signals"""
        logger.info("Telemetry thread started")

        while self._running:
            try:
                # Receive telemetry with 100ms timeout
                result = self.robot_client.receive_telemetry(timeout_ms=100)

                if result:
                    topic, message = result

                    if topic == "telemetry":
                        # Parse telemetry message type
                        if "SensorUpdate" in message:
                            self.sensor_update.emit(message["SensorUpdate"])
                        elif "ConnectionQuality" in message:
                            self.connection_quality.emit(message["ConnectionQuality"])

                    elif topic == "lidar":
                        self.lidar_scan.emit(message)

            except Exception as e:
                logger.error(f"Telemetry reception error: {e}")
                self.connection_error.emit(str(e))
                # Continue running even on errors (might be temporary)

        logger.info("Telemetry thread stopped")

    def stop(self):
        """Stop the telemetry thread gracefully"""
        logger.info("Stopping telemetry thread...")
        self._running = False
        self.wait()  # Wait for thread to finish
