"""
Telemetry thread for receiving sensor data from SangamIO daemon.

Protocol Architecture:
- **TCP (port 5555)**: Commands + registration for UDP streaming
- **UDP (port 5556)**: Receives sensor data via unicast (fire-and-forget)

Connection Flow:
1. Connect via TCP to SangamIO (registers client IP for UDP streaming)
2. SangamIO starts sending UDP unicast packets to this client
3. TCP connection kept alive for sending commands
4. On TCP disconnect, UDP streaming stops

This design eliminates network flooding - only registered clients receive data.
"""

import socket
import struct
import json
import logging
import os
from datetime import datetime
from PyQt5.QtCore import QThread, pyqtSignal

# Import protobuf definitions
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from proto import sangamio_pb2

logger = logging.getLogger(__name__)

# Default ports
UDP_SENSOR_PORT = 5556  # UDP unicast for sensor data
TCP_COMMAND_PORT = 5555  # TCP for commands + UDP registration


class TelemetryThread(QThread):
    """Background thread for receiving telemetry from SangamIO.

    Connects via TCP first (to register for UDP streaming), then
    receives sensor data via UDP unicast. TCP is kept alive for commands.
    """

    # Signal emitted when new sensor data is received
    sensor_data_received = pyqtSignal(dict)

    # Signal for connection status
    connection_status = pyqtSignal(bool, str)

    def __init__(self, host: str, port: int = TCP_COMMAND_PORT,
                 udp_port: int = UDP_SENSOR_PORT,
                 log_raw_packets: bool = False, parent=None):
        super().__init__(parent)
        self.host = host
        self.tcp_port = port  # TCP port for commands
        self.udp_port = udp_port  # UDP port for sensor broadcast
        self._running = True
        self.udp_socket = None  # UDP socket for receiving sensors
        self.tcp_socket = None  # TCP socket for sending commands
        self.log_raw_packets = log_raw_packets
        self.raw_packet_file = None
        self.sensor_data_file = None
        self._packet_count = 0
        self._sensor_log_count = 0

    def run(self):
        """Main thread loop - connect TCP first, then receive UDP sensor data."""
        logger.info(f"Telemetry thread starting")
        logger.info(f"  TCP registration: {self.host}:{self.tcp_port}")
        logger.info(f"  UDP sensor data: port {self.udp_port}")

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
                # Step 1: Connect TCP first (registers us for UDP streaming)
                self.connection_status.emit(False, f"Connecting to {self.host}:{self.tcp_port}...")
                if not self._connect_tcp():
                    if self._running:
                        self.connection_status.emit(False, f"TCP connection failed, retrying...")
                        self.msleep(2000)
                    continue

                # Step 2: Create UDP socket for receiving sensor data
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                # Bind to UDP port to receive unicast packets
                self.udp_socket.bind(('0.0.0.0', self.udp_port))
                self.udp_socket.settimeout(2.0)  # Timeout for checking _running flag

                self.connection_status.emit(True, f"Connected to {self.host} (UDP streaming active)")
                logger.info(f"UDP socket bound to port {self.udp_port} (receiving unicast from {self.host})")

                # Step 3: Receive loop - UDP datagrams
                while self._running:
                    try:
                        # UDP receives complete datagrams
                        data, addr = self.udp_socket.recvfrom(65536)  # Max UDP packet size
                        if data:
                            message = self._parse_udp_message(data)
                            if message:
                                self._process_message(message)
                    except socket.timeout:
                        # Check if TCP connection is still alive
                        if not self._is_tcp_connected():
                            logger.warning("TCP connection lost, reconnecting...")
                            self.connection_status.emit(False, "TCP disconnected, reconnecting...")
                            break
                        continue
                    except Exception as e:
                        if self._running:
                            logger.error(f"UDP receive error: {e}")
                            self.connection_status.emit(False, f"UDP error: {e}")
                        break

            except Exception as e:
                if self._running:
                    logger.error(f"Connection setup failed: {e}")
                    self.connection_status.emit(False, f"Setup failed: {e}")
                    # Wait before retry
                    self.msleep(2000)

            finally:
                # Clean up sockets
                if self.udp_socket:
                    try:
                        self.udp_socket.close()
                    except:
                        pass
                    self.udp_socket = None
                if self.tcp_socket:
                    try:
                        self.tcp_socket.close()
                    except:
                        pass
                    self.tcp_socket = None

        logger.info("Telemetry thread stopped")

    def _connect_tcp(self) -> bool:
        """Connect TCP socket to register for UDP streaming."""
        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.settimeout(5.0)
            self.tcp_socket.connect((self.host, self.tcp_port))
            logger.info(f"TCP connected to {self.host}:{self.tcp_port} (registered for UDP streaming)")
            return True
        except Exception as e:
            logger.error(f"TCP connection failed: {e}")
            if self.tcp_socket:
                try:
                    self.tcp_socket.close()
                except:
                    pass
            self.tcp_socket = None
            return False

    def _is_tcp_connected(self) -> bool:
        """Check if TCP socket is still connected."""
        if not self.tcp_socket:
            return False
        try:
            # Use non-blocking check - recv with MSG_PEEK
            self.tcp_socket.setblocking(False)
            try:
                data = self.tcp_socket.recv(1, socket.MSG_PEEK)
                # If recv returns empty, connection was closed
                if data == b'':
                    return False
            except BlockingIOError:
                # No data available but connection is alive
                pass
            except ConnectionResetError:
                return False
            finally:
                self.tcp_socket.setblocking(True)
            return True
        except Exception:
            return False

    def stop(self):
        """Stop the thread."""
        logger.info("Stopping telemetry thread...")
        self._running = False
        if self.udp_socket:
            try:
                self.udp_socket.close()
            except:
                pass
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
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

    def _ensure_tcp_connected(self) -> bool:
        """Check if TCP socket is connected (established in run())."""
        return self.tcp_socket is not None and self._is_tcp_connected()

    def send_command(self, command: dict) -> bool:
        """Send a command via TCP connection.

        Commands use the ComponentControl protocol:
        {
            "type": "ComponentControl",
            "id": "drive" | "vacuum" | "lidar" | etc.,
            "action": {"type": "Enable" | "Disable" | "Reset" | "Configure", "config": {...}}
        }
        """
        if not self._ensure_tcp_connected():
            logger.error("Cannot send command: TCP not connected")
            return False

        try:
            # Create protobuf message
            msg = sangamio_pb2.Message()
            msg.topic = "command"

            cmd = msg.command
            ctrl = cmd.component_control
            ctrl.id = command.get('id', '')

            # Parse action - can be a string or a dict with "type" key
            action_data = command.get('action', {})
            if isinstance(action_data, str):
                action_type = action_data.lower()
                config_data = {}
            else:
                action_type = action_data.get('type', 'enable').lower()
                config_data = action_data.get('config', {})

            action = ctrl.action
            if action_type == 'enable':
                action.type = sangamio_pb2.ComponentAction.ENABLE
            elif action_type == 'disable':
                action.type = sangamio_pb2.ComponentAction.DISABLE
            elif action_type == 'reset':
                action.type = sangamio_pb2.ComponentAction.RESET
            elif action_type == 'configure':
                action.type = sangamio_pb2.ComponentAction.CONFIGURE

            # Add config values if present
            for key, value in config_data.items():
                sensor_val = sangamio_pb2.SensorValue()
                self._set_sensor_value(sensor_val, value)
                action.config[key].CopyFrom(sensor_val)

            # Serialize and send via TCP
            data = msg.SerializeToString()
            length = struct.pack('>I', len(data))
            self.tcp_socket.sendall(length + data)
            logger.debug(f"Command sent via TCP: {command}")
            return True

        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            # Connection error will be detected by run() loop on next timeout
            return False

    def _set_sensor_value(self, sensor_val: sangamio_pb2.SensorValue, value):
        """Set the appropriate field in a SensorValue based on the value type."""
        if isinstance(value, dict):
            # Handle typed values like {"U8": 100} or {"F32": 0.5}
            if 'Bool' in value:
                sensor_val.bool_val = value['Bool']
            elif 'U8' in value:
                sensor_val.u32_val = value['U8']
            elif 'U16' in value:
                sensor_val.u32_val = value['U16']
            elif 'U32' in value:
                sensor_val.u32_val = value['U32']
            elif 'U64' in value:
                sensor_val.u64_val = value['U64']
            elif 'I8' in value:
                sensor_val.i32_val = value['I8']
            elif 'I16' in value:
                sensor_val.i32_val = value['I16']
            elif 'I32' in value:
                sensor_val.i32_val = value['I32']
            elif 'I64' in value:
                sensor_val.i64_val = value['I64']
            elif 'F32' in value:
                sensor_val.f32_val = value['F32']
            elif 'F64' in value:
                sensor_val.f64_val = value['F64']
            elif 'String' in value:
                sensor_val.string_val = value['String']
        elif isinstance(value, bool):
            sensor_val.bool_val = value
        elif isinstance(value, int):
            sensor_val.i32_val = value
        elif isinstance(value, float):
            sensor_val.f32_val = value
        elif isinstance(value, str):
            sensor_val.string_val = value

    def _parse_udp_message(self, data: bytes) -> sangamio_pb2.Message:
        """Parse a UDP datagram containing a length-prefixed Protobuf message.

        UDP datagrams are received atomically, so we get the complete message
        in one receive. The format is: [4-byte length][protobuf payload]
        """
        if len(data) < 4:
            logger.warning(f"UDP packet too short: {len(data)} bytes")
            return None

        # Read 4-byte length prefix
        length = struct.unpack('>I', data[:4])[0]

        # Validate length matches packet size
        expected_size = 4 + length
        if len(data) != expected_size:
            logger.warning(f"UDP packet size mismatch: expected {expected_size}, got {len(data)}")
            return None

        # Sanity check
        if length > 1024 * 1024:
            logger.warning(f"UDP message too large: {length} bytes")
            return None

        # Parse Protobuf payload (after 4-byte length prefix)
        msg = sangamio_pb2.Message()
        msg.ParseFromString(data[4:])
        return msg

    def _process_message(self, message: sangamio_pb2.Message):
        """Process a received message and emit signal."""
        topic = message.topic

        # Handle sensor group messages
        if topic.startswith('sensors/') and message.HasField('sensor_group'):
            sg = message.sensor_group
            group_id = sg.group_id
            timestamp_us = sg.timestamp_us

            # Convert sensor values from protobuf to dict
            sensor_data = {}
            for key, value in sg.values.items():
                sensor_data[key] = self._extract_value(value)

            # Log raw_packet if enabled and present
            if self.raw_packet_file and 'raw_packet' in sensor_data:
                raw_packet = sensor_data['raw_packet']
                if raw_packet:
                    self._log_raw_packet(raw_packet, timestamp_us)

            # Add metadata
            sensor_data['_group_id'] = group_id
            sensor_data['_timestamp_us'] = timestamp_us

            # Log parsed sensor data if enabled
            if self.sensor_data_file:
                self._log_sensor_data(sensor_data)

            self.sensor_data_received.emit(sensor_data)

    def _log_raw_packet(self, raw_packet: bytes, timestamp_us: int):
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
            data_to_log = {k: v for k, v in sensor_data.items()
                          if k != 'raw_packet' and not isinstance(v, bytes)}
            self.sensor_data_file.write(json.dumps(data_to_log) + '\n')
            self._sensor_log_count += 1

            # Flush every 100 entries
            if self._sensor_log_count % 100 == 0:
                self.sensor_data_file.flush()
        except Exception as e:
            logger.error(f"Failed to log sensor data: {e}")

    def _extract_value(self, sensor_value: sangamio_pb2.SensorValue):
        """Extract value from protobuf SensorValue.

        Returns the appropriate Python type based on which field is set.
        """
        which = sensor_value.WhichOneof('value')
        if which == 'bool_val':
            return sensor_value.bool_val
        elif which == 'u32_val':
            return sensor_value.u32_val
        elif which == 'u64_val':
            return sensor_value.u64_val
        elif which == 'i32_val':
            return sensor_value.i32_val
        elif which == 'i64_val':
            return sensor_value.i64_val
        elif which == 'f32_val':
            return sensor_value.f32_val
        elif which == 'f64_val':
            return sensor_value.f64_val
        elif which == 'string_val':
            return sensor_value.string_val
        elif which == 'bytes_val':
            return sensor_value.bytes_val
        elif which == 'vector3_val':
            v = sensor_value.vector3_val
            return [v.x, v.y, v.z]
        elif which == 'pointcloud_val':
            pc = sensor_value.pointcloud_val
            # Return as list of tuples: (angle_rad, distance_m, quality)
            return [(p.angle_rad, p.distance_m, p.quality) for p in pc.points]
        else:
            return None
