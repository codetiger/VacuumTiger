"""
SLAM thread for receiving data from DhruvaSLAM daemon.

Uses DhruvaStream protocol for continuous data (via TCP):
- RobotStatus: pose, state, battery, mapping progress
- SensorStatus: lidar, encoders, IMU
- CurrentMap: occupancy grid
- MapList: saved maps
- NavigationStatus: path planning

And DhruvaCommand/Response protocol for commands (via same TCP port):
- StartMapping, StopMapping, ClearMap
- RenameMap, DeleteMap, EnableMap, SetGoal, CancelGoal

Note: Both streams and commands use the same port (5557) following the
unified DhruvaSLAM architecture. Two separate TCP connections are used:
one for streaming data, one for command/response.
"""

import socket
import struct
import logging
import uuid
import os
from PyQt5.QtCore import QThread, pyqtSignal

# Import protobuf definitions
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from proto import dhruva_pb2

logger = logging.getLogger(__name__)


class SlamThread(QThread):
    """Background thread for receiving SLAM data from DhruvaSLAM daemon."""

    # Robot status signal (10Hz)
    # Payload: {pose: {x, y, theta}, state: str, active_map_id: str,
    #           battery_percent, is_charging, is_docked,
    #           distance_traveled_m, keyframe_count, loop_closures, map_area_m2,
    #           localization_confidence}
    robot_status_received = pyqtSignal(dict)

    # Sensor status signal (10Hz)
    # Payload: {lidar: {ranges, angle_min, angle_max, angle_increment},
    #           left_encoder_ticks, right_encoder_ticks,
    #           left_velocity_mps, right_velocity_mps,
    #           gyro_z_radps, accel_x_mps2, accel_y_mps2,
    #           raw_odometry: {x, y, theta}}
    sensor_status_received = pyqtSignal(dict)

    # Current map signal (1Hz)
    # Payload: {map_id, name, resolution, width, height, origin_x, origin_y,
    #           cells: bytes, explored_area_m2, obstacle_area_m2,
    #           min_x, max_x, min_y, max_y}
    current_map_received = pyqtSignal(dict)

    # Map list signal (on change)
    # Payload: {maps: [{map_id, name, created_at_us, updated_at_us, area_m2,
    #                   room_count, is_complete}, ...], active_map_id}
    map_list_received = pyqtSignal(dict)

    # Navigation status signal (5Hz)
    # Payload: {state: str, path: [{x, y, theta}, ...], current_waypoint_index,
    #           path_length_m, distance_remaining_m, estimated_time_remaining_s,
    #           goal: {x, y, theta}, goal_description}
    navigation_status_received = pyqtSignal(dict)

    # Connection status signal
    connection_status = pyqtSignal(bool, str)

    # Command response signal
    # Payload: {request_id, success, error_message, data: {...}}
    command_response_received = pyqtSignal(dict)

    def __init__(self, host: str, port: int = 5557, command_port: int = None, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        # Command port defaults to same as stream port (unified architecture)
        self.command_port = command_port if command_port is not None else port
        self._running = True
        self.stream_socket = None
        self.command_socket = None

    def run(self):
        """Main thread loop - connect and receive data."""
        logger.info(f"SLAM thread starting, connecting to {self.host}:{self.port}")

        while self._running:
            try:
                # Connect to stream port
                self.stream_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.stream_socket.settimeout(5.0)
                self.stream_socket.connect((self.host, self.port))
                self.connection_status.emit(True, f"Connected to DhruvaSLAM at {self.host}:{self.port}")
                logger.info(f"Connected to DhruvaSLAM stream at {self.host}:{self.port}")

                # Receive loop
                while self._running:
                    try:
                        message = self._read_stream_message()
                        if message:
                            self._process_stream_message(message)
                    except socket.timeout:
                        continue
                    except Exception as e:
                        if self._running:
                            logger.error(f"Stream read error: {e}")
                            self.connection_status.emit(False, f"SLAM read error: {e}")
                        break

            except Exception as e:
                if self._running:
                    logger.warning(f"SLAM connection failed: {e}")
                    self.connection_status.emit(False, f"SLAM connection failed: {e}")
                    # Wait before retry
                    self.msleep(3000)

            finally:
                if self.stream_socket:
                    try:
                        self.stream_socket.close()
                    except:
                        pass
                    self.stream_socket = None

        logger.info("SLAM thread stopped")

    def stop(self):
        """Stop the thread."""
        logger.info("Stopping SLAM thread...")
        self._running = False
        if self.stream_socket:
            try:
                self.stream_socket.close()
            except:
                pass
        if self.command_socket:
            try:
                self.command_socket.close()
            except:
                pass
        self.wait()

    def _read_stream_message(self) -> dhruva_pb2.DhruvaStream:
        """Read a length-prefixed DhruvaStream message."""
        # Read 4-byte length prefix
        length_data = self._recv_exact(self.stream_socket, 4)
        if not length_data:
            return None

        length = struct.unpack('>I', length_data)[0]

        # Sanity check
        if length > 10 * 1024 * 1024:  # 10MB max
            raise ValueError(f"Message too large: {length} bytes")

        # Read message body
        data = self._recv_exact(self.stream_socket, length)
        if not data:
            return None

        # Parse Protobuf
        try:
            msg = dhruva_pb2.DhruvaStream()
            msg.ParseFromString(data)
            return msg
        except Exception as e:
            # Save problematic data for debugging
            with open('/tmp/failed_proto.bin', 'wb') as f:
                f.write(data)
            logger.error(f"Parse error: {e}, data len={len(data)}, first 50 bytes: {data[:50].hex()}")
            logger.error(f"Saved failed message to /tmp/failed_proto.bin")
            raise

    def _recv_exact(self, sock: socket.socket, n: int) -> bytes:
        """Receive exactly n bytes."""
        data = b''
        while len(data) < n:
            chunk = sock.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _process_stream_message(self, message: dhruva_pb2.DhruvaStream):
        """Process a received DhruvaStream message and emit appropriate signal."""
        which = message.WhichOneof('data')

        if which == 'robot_status':
            status = message.robot_status
            state_names = {
                dhruva_pb2.ROBOT_STATE_IDLE: 'Idle',
                dhruva_pb2.ROBOT_STATE_MAPPING: 'Mapping',
                dhruva_pb2.ROBOT_STATE_LOCALIZING: 'Localizing',
                dhruva_pb2.ROBOT_STATE_LOST: 'Lost',
            }
            payload = {
                'pose': {
                    'x': status.pose.x if status.pose else 0.0,
                    'y': status.pose.y if status.pose else 0.0,
                    'theta': status.pose.theta if status.pose else 0.0,
                },
                'state': state_names.get(status.state, 'Unknown'),
                'active_map_id': status.active_map_id,
                'battery_percent': status.battery_percent,
                'is_charging': status.is_charging,
                'is_docked': status.is_docked,
                'distance_traveled_m': status.distance_traveled_m,
                'keyframe_count': status.keyframe_count,
                'loop_closures': status.loop_closures,
                'map_area_m2': status.map_area_m2,
                'localization_confidence': status.localization_confidence,
                'timestamp_us': message.timestamp_us,
            }
            self.robot_status_received.emit(payload)
            logger.debug(f"Robot status: state={payload['state']} pose=({status.pose.x:.2f}, {status.pose.y:.2f})")

        elif which == 'sensor_status':
            sensor = message.sensor_status
            lidar_data = None
            if sensor.HasField('lidar'):
                lidar = sensor.lidar
                lidar_data = {
                    'ranges': list(lidar.ranges),
                    'intensities': list(lidar.intensities),
                    'angle_min': lidar.angle_min,
                    'angle_max': lidar.angle_max,
                    'angle_increment': lidar.angle_increment,
                }

            raw_odom = None
            if sensor.HasField('raw_odometry'):
                raw_odom = {
                    'x': sensor.raw_odometry.x,
                    'y': sensor.raw_odometry.y,
                    'theta': sensor.raw_odometry.theta,
                }

            payload = {
                'lidar': lidar_data,
                'left_encoder_ticks': sensor.left_encoder_ticks,
                'right_encoder_ticks': sensor.right_encoder_ticks,
                'left_velocity_mps': sensor.left_velocity_mps,
                'right_velocity_mps': sensor.right_velocity_mps,
                'gyro_z_radps': sensor.gyro_z_radps,
                'accel_x_mps2': sensor.accel_x_mps2,
                'accel_y_mps2': sensor.accel_y_mps2,
                'raw_odometry': raw_odom,
                'timestamp_us': message.timestamp_us,
            }
            self.sensor_status_received.emit(payload)
            if lidar_data:
                logger.debug(f"Sensor status: lidar={len(lidar_data['ranges'])} rays")

        elif which == 'current_map':
            map_data = message.current_map
            payload = {
                'map_id': map_data.map_id,
                'name': map_data.name,
                'resolution': map_data.resolution,
                'width': map_data.width,
                'height': map_data.height,
                'origin_x': map_data.origin_x,
                'origin_y': map_data.origin_y,
                'cells': map_data.cells,
                'explored_area_m2': map_data.explored_area_m2,
                'obstacle_area_m2': map_data.obstacle_area_m2,
                'min_x': map_data.min_x,
                'max_x': map_data.max_x,
                'min_y': map_data.min_y,
                'max_y': map_data.max_y,
                'timestamp_us': message.timestamp_us,
            }
            self.current_map_received.emit(payload)
            logger.info(f"Current map received: {map_data.width}x{map_data.height} @ {map_data.resolution}m, cells={len(map_data.cells)} bytes")

        elif which == 'map_list':
            map_list = message.map_list
            maps = []
            for m in map_list.maps:
                maps.append({
                    'map_id': m.map_id,
                    'name': m.name,
                    'created_at_us': m.created_at_us,
                    'updated_at_us': m.updated_at_us,
                    'area_m2': m.area_m2,
                    'room_count': m.room_count,
                    'is_complete': m.is_complete,
                })
            payload = {
                'maps': maps,
                'active_map_id': map_list.active_map_id,
                'timestamp_us': message.timestamp_us,
            }
            self.map_list_received.emit(payload)
            logger.debug(f"Map list: {len(maps)} maps, active={map_list.active_map_id}")

        elif which == 'navigation_status':
            nav = message.navigation_status
            state_names = {
                dhruva_pb2.NAV_STATE_IDLE: 'Idle',
                dhruva_pb2.NAV_STATE_PLANNING: 'Planning',
                dhruva_pb2.NAV_STATE_NAVIGATING: 'Navigating',
                dhruva_pb2.NAV_STATE_ROTATING_TO_HEADING: 'Rotating',
                dhruva_pb2.NAV_STATE_REACHED: 'Reached',
                dhruva_pb2.NAV_STATE_FAILED: 'Failed',
                dhruva_pb2.NAV_STATE_CANCELLED: 'Cancelled',
            }
            path = [{'x': p.x, 'y': p.y, 'theta': p.theta} for p in nav.path]
            goal = None
            if nav.HasField('goal'):
                goal = {'x': nav.goal.x, 'y': nav.goal.y, 'theta': nav.goal.theta}

            payload = {
                'state': state_names.get(nav.state, 'Unknown'),
                'state_enum': nav.state,
                'path': path,
                'current_waypoint_index': nav.current_waypoint_index,
                'path_length_m': nav.path_length_m,
                'distance_remaining_m': nav.distance_remaining_m,
                'estimated_time_remaining_s': nav.estimated_time_remaining_s,
                'goal': goal,
                'goal_description': nav.goal_description,
                'target_id': nav.target_id,
                'targets_pending': nav.targets_pending,
                'targets_completed': nav.targets_completed,
                'status_message': nav.status_message,
                'failure_reason': nav.failure_reason,
                'timestamp_us': message.timestamp_us,
            }
            self.navigation_status_received.emit(payload)
            logger.debug(f"Nav status: state={payload['state']} waypoints={len(path)}")

    # ==================== Command Methods ====================

    def send_command(self, command: dhruva_pb2.DhruvaCommand) -> bool:
        """Send a command to DhruvaSLAM and wait for response.

        Returns True if command was sent successfully.
        Response will be emitted via command_response_received signal.
        """
        try:
            # Connect to command port if needed
            if not self.command_socket:
                self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.command_socket.settimeout(5.0)
                self.command_socket.connect((self.host, self.command_port))
                logger.info(f"Connected to DhruvaSLAM command port at {self.host}:{self.command_port}")

            # Serialize and send
            payload = command.SerializeToString()
            length = struct.pack('>I', len(payload))
            self.command_socket.sendall(length + payload)

            # Read response
            len_bytes = self._recv_exact(self.command_socket, 4)
            if not len_bytes:
                logger.error("No response from command port")
                return False

            resp_len = struct.unpack('>I', len_bytes)[0]
            resp_data = self._recv_exact(self.command_socket, resp_len)
            if not resp_data:
                logger.error("Incomplete response from command port")
                return False

            # Parse response
            response = dhruva_pb2.DhruvaResponse()
            response.ParseFromString(resp_data)

            # Emit response
            resp_payload = {
                'request_id': response.request_id,
                'success': response.success,
                'error_message': response.error_message,
            }

            which = response.WhichOneof('data')
            if which == 'mapping_started':
                resp_payload['data'] = {'map_id': response.mapping_started.map_id}
            elif which == 'mapping_stopped':
                resp_payload['data'] = {
                    'saved': response.mapping_stopped.saved,
                    'area_m2': response.mapping_stopped.area_m2,
                }
            elif which == 'goal_accepted':
                resp_payload['data'] = {'target_id': response.goal_accepted.target_id}
            elif which == 'goal_cancelled':
                resp_payload['data'] = {}

            self.command_response_received.emit(resp_payload)
            return True

        except Exception as e:
            logger.error(f"Command send failed: {e}")
            # Reset command socket
            if self.command_socket:
                try:
                    self.command_socket.close()
                except:
                    pass
                self.command_socket = None
            return False

    def start_mapping(self, map_name: str = "") -> str:
        """Start mapping with optional map name. Returns request_id."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())
        cmd.start_mapping.map_name = map_name
        if self.send_command(cmd):
            return cmd.request_id
        return None

    def stop_mapping(self, save: bool = True) -> str:
        """Stop mapping. Returns request_id."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())
        cmd.stop_mapping.save = save
        if self.send_command(cmd):
            return cmd.request_id
        return None

    def clear_map(self) -> str:
        """Clear current map. Returns request_id."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())
        cmd.clear_map.CopyFrom(dhruva_pb2.ClearMapCommand())
        if self.send_command(cmd):
            return cmd.request_id
        return None

    def rename_map(self, map_id: str, new_name: str) -> str:
        """Rename a saved map. Returns request_id."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())
        cmd.rename_map.map_id = map_id
        cmd.rename_map.new_name = new_name
        if self.send_command(cmd):
            return cmd.request_id
        return None

    def delete_map(self, map_id: str) -> str:
        """Delete a saved map. Returns request_id."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())
        cmd.delete_map.map_id = map_id
        if self.send_command(cmd):
            return cmd.request_id
        return None

    def enable_map(self, map_id: str) -> str:
        """Enable a map for localization. Returns request_id."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())
        cmd.enable_map.map_id = map_id
        if self.send_command(cmd):
            return cmd.request_id
        return None

    def emergency_stop(self) -> str:
        """Emergency stop - disable lidar, motors, and all motion. Returns request_id."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())
        cmd.emergency_stop.CopyFrom(dhruva_pb2.EmergencyStopCommand())
        if self.send_command(cmd):
            return cmd.request_id
        return None

    def set_goal(self, x: float, y: float, theta: float = None, description: str = "") -> str:
        """Set navigation goal. Returns request_id.

        Args:
            x: Target X position in meters
            y: Target Y position in meters
            theta: Optional target heading in radians (None = planner chooses)
            description: Optional description for UI
        """
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())
        cmd.set_goal.x = x
        cmd.set_goal.y = y
        if theta is not None:
            cmd.set_goal.theta = theta
        cmd.set_goal.description = description if description else f"({x:.1f}, {y:.1f})"
        logger.info(f"Sending SetGoal: ({x:.2f}, {y:.2f}), theta={theta}, desc={cmd.set_goal.description}")
        if self.send_command(cmd):
            return cmd.request_id
        return None

    def cancel_goal(self) -> str:
        """Cancel active navigation. Returns request_id."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())
        cmd.cancel_goal.CopyFrom(dhruva_pb2.CancelGoalCommand())
        logger.info("Sending CancelGoal")
        if self.send_command(cmd):
            return cmd.request_id
        return None
