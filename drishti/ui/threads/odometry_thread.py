"""
Odometry thread for receiving pose data from dhruva-slam-node daemon.
Supports Protobuf wire format with DhruvaMessage wrapper.
"""

import socket
import struct
import logging
import os
from PyQt5.QtCore import QThread, pyqtSignal

# Import protobuf definitions
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from proto import dhruva_pb2

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

    # slam/map payload: {resolution, width, height, origin_x, origin_y, cells (bytes), timestamp_us}
    slam_map_received = pyqtSignal(dict)

    # slam/scan payload: {points: [[x,y],...], pose: [x,y,theta], timestamp_us}
    slam_scan_received = pyqtSignal(dict)

    # slam/diagnostics payload: {timestamp_us, timing: {...}, scan_match: {...},
    #                            mapping: {...}, loop_closure: {...}}
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
                logger.info(f"Connected to dhruva-slam-node at {self.host}:{self.port} (wire format: Protobuf)")

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

    def _read_message(self) -> dhruva_pb2.DhruvaMessage:
        """Read a length-prefixed Protobuf message."""
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

        # Parse Protobuf
        msg = dhruva_pb2.DhruvaMessage()
        msg.ParseFromString(data)
        return msg

    def _recv_exact(self, n: int) -> bytes:
        """Receive exactly n bytes."""
        data = b''
        while len(data) < n:
            chunk = self.socket.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _process_message(self, message: dhruva_pb2.DhruvaMessage):
        """Process a received message and emit appropriate signal."""
        topic = message.topic
        which = message.WhichOneof('payload')
        logger.info(f"Received message: topic={topic}, payload_type={which}")

        if topic == 'odometry/pose' and which == 'odometry_pose':
            pose = message.odometry_pose
            payload = {
                'x': pose.x,
                'y': pose.y,
                'theta': pose.theta,
                'timestamp_us': pose.timestamp_us
            }
            self.pose_received.emit(payload)
            logger.debug(f"Pose: x={pose.x:.3f} y={pose.y:.3f} θ={pose.theta:.3f}")

        elif topic == 'odometry/diagnostics' and which == 'odometry_diagnostics':
            diag = message.odometry_diagnostics
            payload = {
                'drift_rate': diag.drift_rate,
                'tick_rate_left': diag.tick_rate_left,
                'tick_rate_right': diag.tick_rate_right,
                'distance_traveled': diag.distance_traveled,
                'gyro_bias': diag.gyro_bias
            }
            self.diagnostics_received.emit(payload)
            logger.debug(f"Diagnostics: distance={diag.distance_traveled:.2f}m")

        elif topic == 'slam/status' and which == 'slam_status':
            status = message.slam_status
            payload = {
                'mode': status.mode,
                'num_scans': status.num_scans,
                'num_keyframes': status.num_keyframes,
                'num_submaps': status.num_submaps,
                'num_finished_submaps': status.num_finished_submaps,
                'match_score': status.match_score,
                'is_lost': status.is_lost,
                'memory_usage_bytes': status.memory_usage_bytes
            }
            self.slam_status_received.emit(payload)
            logger.debug(f"SLAM status: mode={status.mode} scans={status.num_scans}")

        elif topic == 'slam/map' and which == 'slam_map':
            slam_map = message.slam_map
            cells_bytes = slam_map.cells
            # Count non-zero (non-unknown) cells for debugging
            non_zero = sum(1 for b in cells_bytes if b != 0) if cells_bytes else 0
            payload = {
                'resolution': slam_map.resolution,
                'width': slam_map.width,
                'height': slam_map.height,
                'origin_x': slam_map.origin_x,
                'origin_y': slam_map.origin_y,
                'cells': cells_bytes,  # Raw bytes (no base64)
                'timestamp_us': slam_map.timestamp_us
            }
            self.slam_map_received.emit(payload)
            logger.info(f"SLAM map: {slam_map.width}x{slam_map.height} @ {slam_map.resolution}m, {non_zero} non-unknown cells")

        elif topic == 'slam/scan' and which == 'slam_scan':
            scan = message.slam_scan
            # Convert points to list of [x, y]
            points = [[p.x, p.y] for p in scan.points]
            pose = scan.pose
            payload = {
                'points': points,
                'pose': [pose.x, pose.y, pose.theta] if pose else [0, 0, 0],
                'timestamp_us': scan.timestamp_us
            }
            self.slam_scan_received.emit(payload)
            logger.debug(f"SLAM scan: {len(points)} points")

        elif topic == 'slam/diagnostics' and which == 'slam_diagnostics':
            diag = message.slam_diagnostics
            timing = diag.timing
            scan_match = diag.scan_match
            mapping = diag.mapping
            loop_closure = diag.loop_closure

            payload = {
                'timestamp_us': diag.timestamp_us,
                'timing': {
                    'total_us': timing.total_us if timing else 0,
                    'preprocessing_us': timing.preprocessing_us if timing else 0,
                    'scan_matching_us': timing.scan_matching_us if timing else 0,
                    'map_update_us': timing.map_update_us if timing else 0,
                    'particle_filter_us': timing.particle_filter_us if timing else 0,
                    'keyframe_check_us': timing.keyframe_check_us if timing else 0,
                    'avg_total_us': timing.avg_total_us if timing else 0,
                },
                'scan_match': {
                    'method': scan_match.method if scan_match else '',
                    'iterations': scan_match.iterations if scan_match else 0,
                    'score': scan_match.score if scan_match else 0,
                    'mse': scan_match.mse if scan_match else 0,
                    'converged': scan_match.converged if scan_match else False,
                    'correspondences': scan_match.correspondences if scan_match else 0,
                    'inlier_ratio': scan_match.inlier_ratio if scan_match else 0,
                },
                'mapping': {
                    'cells_updated': mapping.cells_updated if mapping else 0,
                    'map_size_bytes': mapping.map_size_bytes if mapping else 0,
                    'occupied_cells': mapping.occupied_cells if mapping else 0,
                    'free_cells': mapping.free_cells if mapping else 0,
                    'active_submap_id': mapping.active_submap_id if mapping and mapping.HasField('active_submap_id') else None,
                    'num_submaps': mapping.num_submaps if mapping else 0,
                },
                'loop_closure': {
                    'candidates_evaluated': loop_closure.candidates_evaluated if loop_closure else 0,
                    'closures_accepted': loop_closure.closures_accepted if loop_closure else 0,
                    'last_closure_score': loop_closure.last_closure_score if loop_closure else 0,
                    'pose_graph_nodes': loop_closure.pose_graph_nodes if loop_closure else 0,
                    'pose_graph_edges': loop_closure.pose_graph_edges if loop_closure else 0,
                },
            }
            self.slam_diagnostics_received.emit(payload)
            logger.debug(
                f"SLAM diagnostics: total={timing.total_us if timing else 0}us "
                f"match={timing.scan_matching_us if timing else 0}us "
                f"score={scan_match.score if scan_match else 0:.2f}"
            )

    def send_reset_command(self) -> bool:
        """Send reset odometry command to dhruva-slam-node.

        Note: This requires the daemon to support command messages.
        Currently a placeholder for future implementation.
        """
        # TODO: Implement when dhruva-slam-node supports commands
        logger.warning("Reset command not yet implemented")
        return False
