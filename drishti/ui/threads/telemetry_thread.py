"""
Telemetry thread for receiving sensor data from SangamIO daemon.
Supports JSON wire format with Message/Payload structure.
"""

import socket
import struct
import json
import logging
import os
from datetime import datetime
from PyQt5.QtCore import QThread, pyqtSignal

logger = logging.getLogger(__name__)


class TelemetryThread(QThread):
    """Background thread for receiving telemetry from SangamIO."""

    # Signal emitted when new sensor data is received
    sensor_data_received = pyqtSignal(dict)

    # Signal for connection status
    connection_status = pyqtSignal(bool, str)

    def __init__(self, host: str, port: int = 5555, log_raw_packets: bool = False, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self._running = True
        self.socket = None
        self.log_raw_packets = log_raw_packets
        self.raw_packet_file = None
        self.sensor_data_file = None
        self._packet_count = 0
        self._sensor_log_count = 0

    def run(self):
        """Main thread loop - connect and receive data."""
        logger.info(f"Telemetry thread starting, connecting to {self.host}:{self.port}")

        # Open log files if enabled
        if self.log_raw_packets:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            raw_log_filename = f"raw_packets_{timestamp}.log"
            sensor_log_filename = f"sensor_data_{timestamp}.log"
            self.raw_packet_file = open(raw_log_filename, 'w')
            self.sensor_data_file = open(sensor_log_filename, 'w')
            logger.info(f"Raw packet logging enabled: {raw_log_filename}")
            logger.info(f"Sensor data logging enabled: {sensor_log_filename}")

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
        if self.raw_packet_file:
            try:
                self.raw_packet_file.close()
                logger.info(f"Raw packet log closed ({self._packet_count} packets logged)")
            except:
                pass
        if self.sensor_data_file:
            try:
                self.sensor_data_file.close()
                logger.info(f"Sensor data log closed ({self._sensor_log_count} entries logged)")
            except:
                pass
        self.wait()

    def send_command(self, command: dict) -> bool:
        """Send a command through the existing connection.

        Commands use the ComponentControl protocol:
        {
            "type": "ComponentControl",
            "id": "drive" | "vacuum" | "lidar" | etc.,
            "action": {
                "type": "Enable" | "Disable" | "Reset" | "Configure",
                "config": { ... }  # optional
            }
        }
        """
        if not self.socket:
            logger.error("Cannot send command: not connected")
            return False

        try:
            # Create message with topic and payload
            message = {
                "topic": "command",
                "payload": {
                    "type": "Command",
                    "command": command
                }
            }

            # Encode as JSON
            data = json.dumps(message).encode('utf-8')

            # Create length-prefixed message
            length = struct.pack('>I', len(data))

            # Send through existing socket
            self.socket.sendall(length + data)
            logger.debug(f"Command sent: {command}")
            return True

        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return False

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

            # Log raw_packet if enabled and present
            if self.raw_packet_file and 'raw_packet' in values:
                raw_packet = self._extract_value(values['raw_packet'])
                if raw_packet:
                    self._log_raw_packet(raw_packet, payload.get('timestamp_us', 0))

            # Convert sensor values from tagged format to simple values
            sensor_data = {}
            for key, value in values.items():
                sensor_data[key] = self._extract_value(value)

            # Add metadata
            sensor_data['_group_id'] = group_id
            sensor_data['_timestamp_us'] = payload.get('timestamp_us', 0)

            # Log parsed sensor data if enabled
            if self.sensor_data_file:
                self._log_sensor_data(sensor_data)

            self.sensor_data_received.emit(sensor_data)

    def _log_raw_packet(self, raw_packet: list, timestamp_us: int):
        """Log raw packet bytes to file.

        Format: timestamp_us,hex_bytes
        Example: 1234567890,FA FB 03 15 00 01 02 ...
        """
        try:
            hex_str = ' '.join(f'{b:02X}' for b in raw_packet)
            self.raw_packet_file.write(f"{timestamp_us},{hex_str}\n")
            self._packet_count += 1

            # Flush every 100 packets to ensure data is written
            if self._packet_count % 100 == 0:
                self.raw_packet_file.flush()
        except Exception as e:
            logger.error(f"Failed to log raw packet: {e}")

    def _log_sensor_data(self, sensor_data: dict):
        """Log parsed sensor data as JSON to file."""
        try:
            # Write as JSON line (exclude raw_packet to keep it readable)
            data_to_log = {k: v for k, v in sensor_data.items() if k != 'raw_packet'}
            self.sensor_data_file.write(json.dumps(data_to_log) + '\n')
            self._sensor_log_count += 1

            # Flush every 100 entries
            if self._sensor_log_count % 100 == 0:
                self.sensor_data_file.flush()
        except Exception as e:
            logger.error(f"Failed to log sensor data: {e}")

    def _extract_value(self, tagged_value):
        """Extract value from tagged SensorValue format.

        Format: {"Bool": true} or {"U16": 1234} etc.
        """
        if isinstance(tagged_value, dict):
            for type_tag, value in tagged_value.items():
                return value
        return tagged_value
