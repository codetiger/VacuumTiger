"""
Odometry thread for receiving pose data from dhruva-slam-node daemon.
Supports JSON wire format with odometry/pose and odometry/diagnostics topics.
"""

import socket
import struct
import json
import logging
from PyQt5.QtCore import QThread, pyqtSignal

logger = logging.getLogger(__name__)


class OdometryThread(QThread):
    """Background thread for receiving odometry data from dhruva-slam-node."""

    # Signal emitted when new pose is received
    # Payload: {x: float, y: float, theta: float, timestamp_us: int}
    pose_received = pyqtSignal(dict)

    # Signal emitted when diagnostics are received
    # Payload: {drift_rate, tick_rate_left, tick_rate_right, distance_traveled, gyro_bias}
    diagnostics_received = pyqtSignal(dict)

    # Signal for connection status
    connection_status = pyqtSignal(bool, str)

    # SLAM signals
    # slam/status payload: {mode, num_scans, num_keyframes, num_submaps,
    #                       num_finished_submaps, match_score, is_lost, memory_usage_bytes}
    slam_status_received = pyqtSignal(dict)

    # slam/map payload: {resolution, width, height, origin_x, origin_y, cells (base64), timestamp_us}
    slam_map_received = pyqtSignal(dict)

    # slam/scan payload: {points: [[x,y],...], pose: [x,y,theta], timestamp_us}
    slam_scan_received = pyqtSignal(dict)

    # slam/diagnostics payload: {timestamp_us, timing: {...}, scan_match: {...},
    #                            particle_filter: {...}, mapping: {...}, loop_closure: {...}}
    slam_diagnostics_received = pyqtSignal(dict)

    def __init__(self, host: str, port: int = 5557, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self._running = True
        self.socket = None

    def run(self):
        """Main thread loop - connect and receive data."""
        logger.info(f"Odometry thread starting, connecting to {self.host}:{self.port}")

        while self._running:
            try:
                # Connect to dhruva-slam-node
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5.0)
                self.socket.connect((self.host, self.port))
                self.connection_status.emit(True, f"Connected to SLAM at {self.host}:{self.port}")
                logger.info(f"Connected to dhruva-slam-node at {self.host}:{self.port}")

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
                            self.connection_status.emit(False, f"SLAM read error: {e}")
                        break

            except Exception as e:
                if self._running:
                    logger.warning(f"SLAM connection failed: {e}")
                    self.connection_status.emit(False, f"SLAM connection failed: {e}")
                    # Wait before retry
                    self.msleep(3000)

            finally:
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None

        logger.info("Odometry thread stopped")

    def stop(self):
        """Stop the thread."""
        logger.info("Stopping odometry thread...")
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
        """Process a received message and emit appropriate signal."""
        topic = message.get('topic', '')
        payload = message.get('payload', {})

        if topic == 'odometry/pose':
            # Emit pose signal
            self.pose_received.emit(payload)
            logger.debug(f"Pose: x={payload.get('x', 0):.3f} y={payload.get('y', 0):.3f} θ={payload.get('theta', 0):.3f}")

        elif topic == 'odometry/diagnostics':
            # Emit diagnostics signal
            self.diagnostics_received.emit(payload)
            logger.debug(f"Diagnostics: distance={payload.get('distance_traveled', 0):.2f}m")

        elif topic == 'slam/status':
            # Emit SLAM status signal
            self.slam_status_received.emit(payload)
            logger.debug(f"SLAM status: mode={payload.get('mode')} scans={payload.get('num_scans')}")

        elif topic == 'slam/map':
            # Emit SLAM map signal
            self.slam_map_received.emit(payload)
            logger.debug(f"SLAM map: {payload.get('width')}x{payload.get('height')} @ {payload.get('resolution')}m")

        elif topic == 'slam/scan':
            # Emit SLAM scan signal
            self.slam_scan_received.emit(payload)
            logger.debug(f"SLAM scan: {len(payload.get('points', []))} points")

        elif topic == 'slam/diagnostics':
            # Emit SLAM diagnostics signal
            self.slam_diagnostics_received.emit(payload)
            timing = payload.get('timing', {})
            scan_match = payload.get('scan_match', {})
            logger.debug(
                f"SLAM diagnostics: total={timing.get('total_us', 0)}us "
                f"match={timing.get('scan_matching_us', 0)}us "
                f"score={scan_match.get('score', 0):.2f}"
            )

    def send_reset_command(self) -> bool:
        """Send reset odometry command to dhruva-slam-node.

        Note: This requires the daemon to support command messages.
        Currently a placeholder for future implementation.
        """
        # TODO: Implement when dhruva-slam-node supports commands
        logger.warning("Reset command not yet implemented")
        return False
