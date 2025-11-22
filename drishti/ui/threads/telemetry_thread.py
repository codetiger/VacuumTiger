"""
Telemetry thread for receiving sensor data from SangamIO daemon.
Supports JSON wire format with Message/Payload structure.
"""

import socket
import struct
import json
import logging
from PyQt5.QtCore import QThread, pyqtSignal

logger = logging.getLogger(__name__)


class TelemetryThread(QThread):
    """Background thread for receiving telemetry from SangamIO."""

    # Signal emitted when new sensor data is received
    sensor_data_received = pyqtSignal(dict)

    # Signal for connection status
    connection_status = pyqtSignal(bool, str)

    def __init__(self, host: str, port: int = 5555, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self._running = True
        self.socket = None

    def run(self):
        """Main thread loop - connect and receive data."""
        logger.info(f"Telemetry thread starting, connecting to {self.host}:{self.port}")

        while self._running:
            try:
                # Connect to SangamIO
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5.0)
                self.socket.connect((self.host, self.port))
                self.connection_status.emit(True, f"Connected to {self.host}:{self.port}")
                logger.info(f"Connected to {self.host}:{self.port}")

                # Receive loop
                while self._running:
                    try:
                        message = self._read_message()
                        if message:
                            self._process_message(message)
                    except socket.timeout:
                        continue
                    except Exception as e:
                        if self._running:
                            logger.error(f"Read error: {e}")
                            self.connection_status.emit(False, f"Read error: {e}")
                        break

            except Exception as e:
                if self._running:
                    logger.error(f"Connection failed: {e}")
                    self.connection_status.emit(False, f"Connection failed: {e}")
                    # Wait before retry
                    self.msleep(2000)

            finally:
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None

        logger.info("Telemetry thread stopped")

    def stop(self):
        """Stop the thread."""
        logger.info("Stopping telemetry thread...")
        self._running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.wait()

    def _read_message(self) -> dict:
        """Read a length-prefixed JSON message."""
        # Read 4-byte length prefix
        length_data = self._recv_exact(4)
        if not length_data:
            return None

        length = struct.unpack('>I', length_data)[0]

        # Sanity check
        if length > 1024 * 1024:
            raise ValueError(f"Message too large: {length} bytes")

        # Read message body
        data = self._recv_exact(length)
        if not data:
            return None

        # Parse JSON
        return json.loads(data.decode('utf-8'))

    def _recv_exact(self, n: int) -> bytes:
        """Receive exactly n bytes."""
        data = b''
        while len(data) < n:
            chunk = self.socket.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _process_message(self, message: dict):
        """Process a received message and emit signal."""
        topic = message.get('topic', '')
        payload = message.get('payload', {})

        # Handle sensor group messages
        if topic.startswith('sensors/'):
            group_id = payload.get('group_id', '')
            values = payload.get('values', {})

            # Convert sensor values from tagged format to simple values
            sensor_data = {}
            for key, value in values.items():
                sensor_data[key] = self._extract_value(value)

            # Add metadata
            sensor_data['_group_id'] = group_id
            sensor_data['_timestamp_us'] = payload.get('timestamp_us', 0)

            self.sensor_data_received.emit(sensor_data)

    def _extract_value(self, tagged_value):
        """Extract value from tagged SensorValue format.

        Format: {"Bool": true} or {"U16": 1234} etc.
        """
        if isinstance(tagged_value, dict):
            for type_tag, value in tagged_value.items():
                return value
        return tagged_value
