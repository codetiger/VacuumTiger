#!/usr/bin/env python3
"""
Drishti - Real-time Robot Status Visualization System

Connects to SangamIO robot via TCP to:
- Subscribe to hardware telemetry and lidar data
- Send motion and actuator commands

This demo script:
1. Subscribes to telemetry updates
2. Initializes the side brush at 50% speed
3. Runs for 5 seconds
4. Stops the side brush
"""

import argparse
import logging
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional

import msgpack

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger(__name__)


@dataclass
class SensorUpdate:
    """Raw sensor data from GD32 motor controller"""
    timestamp: int
    battery_level: Optional[int]
    is_charging: Optional[bool]
    ir_sensor_1: Optional[int]
    start_button_ir: Optional[int]
    dock_button_ir: Optional[int]
    bumper_pressed: Optional[bool]
    cliff_sensors: Optional[list]
    error_code: Optional[int]
    encoder_left: Optional[int]
    encoder_right: Optional[int]


@dataclass
class ConnectionQuality:
    """Communication statistics for GD32 connection health"""
    timestamp: int
    rx_packets: int
    tx_packets: int
    success_rate: float
    telemetry_fresh: bool


class RobotClient:
    """Client for communicating with SangamIO robot via TCP"""

    def __init__(self, robot_ip: str = "vacuum", pub_port: int = 5555, cmd_port: int = 5556):
        """
        Initialize robot client

        Args:
            robot_ip: Robot hostname or IP address
            pub_port: Robot's TCP publisher port for telemetry/lidar (default 5555)
            cmd_port: Robot's TCP command port (default 5556)
        """
        self.robot_ip = robot_ip
        self.pub_port = pub_port
        self.cmd_port = cmd_port

        # Telemetry socket (receives telemetry and lidar from robot)
        self.telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.telemetry_sock.settimeout(5.0)  # 5 second connection timeout
        telemetry_addr = (robot_ip, pub_port)
        logger.info(f"Connecting to robot telemetry: {robot_ip}:{pub_port}")
        try:
            self.telemetry_sock.connect(telemetry_addr)
            self.telemetry_sock.settimeout(1.0)  # 1 second receive timeout
            logger.info("✓ Connected to telemetry stream")
        except Exception as e:
            logger.error(f"Failed to connect to telemetry: {e}")
            raise

        # Command socket (sends commands to robot)
        self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.command_sock.settimeout(5.0)
        command_addr = (robot_ip, cmd_port)
        logger.info(f"Connecting to robot command channel: {robot_ip}:{cmd_port}")
        try:
            self.command_sock.connect(command_addr)
            logger.info("✓ Connected to command channel")
        except Exception as e:
            logger.error(f"Failed to connect to command channel: {e}")
            self.telemetry_sock.close()
            raise

        logger.info("✓ Connected to robot")

    def receive_telemetry(self, timeout_ms: int = 1000) -> Optional[tuple]:
        """
        Receive one telemetry message

        Args:
            timeout_ms: Receive timeout in milliseconds

        Returns:
            Tuple of (topic, message_dict) or None if timeout
        """
        try:
            # Set timeout
            self.telemetry_sock.settimeout(timeout_ms / 1000.0)

            # Read 4-byte length prefix (big-endian)
            length_bytes = self._recv_exact(self.telemetry_sock, 4)
            if not length_bytes:
                return None

            length = struct.unpack('>I', length_bytes)[0]

            # Sanity check
            if length > 10_000_000:  # 10MB limit
                logger.error(f"Invalid message length: {length}")
                return None

            # Read frame: [topic\0][MessagePack payload]
            frame = self._recv_exact(self.telemetry_sock, length)
            if not frame:
                return None

            # Parse topic (null-terminated string)
            null_pos = frame.find(b'\x00')
            if null_pos == -1:
                logger.error("Invalid frame: no null terminator")
                return None

            topic = frame[:null_pos].decode('utf-8')
            payload = frame[null_pos + 1:]

            # Deserialize MessagePack payload
            message = msgpack.unpackb(payload, raw=False)

            return (topic, message)

        except socket.timeout:
            # Timeout
            return None
        except Exception as e:
            logger.error(f"Error receiving telemetry: {e}")
            return None

    def _recv_exact(self, sock: socket.socket, n: int) -> Optional[bytes]:
        """
        Receive exactly n bytes from socket

        Args:
            sock: Socket to receive from
            n: Number of bytes to receive

        Returns:
            Bytes received or None on error
        """
        data = b''
        while len(data) < n:
            try:
                chunk = sock.recv(n - len(data))
                if not chunk:
                    return None
                data += chunk
            except socket.timeout:
                # Timeout is expected when no data is available
                return None
            except Exception as e:
                logger.error(f"Socket receive error: {e}")
                return None
        return data

    def _reconnect_command_socket(self):
        """Reconnect the command socket if connection is lost"""
        try:
            logger.info("Attempting to reconnect command socket...")
            self.command_sock.close()
        except:
            pass

        try:
            self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.command_sock.settimeout(5.0)
            command_addr = (self.robot_ip, self.cmd_port)
            self.command_sock.connect(command_addr)
            logger.info("✓ Command socket reconnected")
            return True
        except Exception as e:
            logger.error(f"Failed to reconnect command socket: {e}")
            return False

    def send_command(self, command: dict):
        """
        Send a robot command with automatic reconnection

        Args:
            command: Command dictionary (will be serialized to MessagePack)
        """
        max_retries = 2
        for attempt in range(max_retries):
            try:
                # Serialize command to MessagePack
                payload = msgpack.packb(command, use_bin_type=True)

                # Create frame: [topic\0][payload]
                frame = b"command\x00" + payload

                # Create length-prefixed message: [4-byte length][frame]
                length = len(frame)
                message = struct.pack('>I', length) + frame

                # Send message
                self.command_sock.sendall(message)

                logger.debug(f"Sent command: {command}")
                return  # Success!

            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                logger.warning(f"Connection error on attempt {attempt + 1}/{max_retries}: {e}")
                if attempt < max_retries - 1:
                    # Try to reconnect
                    if self._reconnect_command_socket():
                        continue  # Retry with new connection
                logger.error(f"Failed to send command after {max_retries} attempts")
            except Exception as e:
                logger.error(f"Error sending command: {e}")
                break

    def set_side_brush_speed(self, speed: int):
        """
        Set side brush speed

        Args:
            speed: Speed percentage (0-100)
        """
        command = {
            "SetSideBrushSpeed": {
                "speed": max(0, min(100, speed))
            }
        }
        self.send_command(command)
        logger.info(f"Side brush speed set to {speed}%")

    def set_wheel_velocity(self, left_ticks: float, right_ticks: float):
        """
        Set wheel velocities

        Args:
            left_ticks: Left wheel velocity (ticks/sec)
            right_ticks: Right wheel velocity (ticks/sec)
        """
        command = {
            "SetWheelVelocity": {
                "left": left_ticks,
                "right": right_ticks
            }
        }
        self.send_command(command)
        logger.info(f"Wheel velocities set: left={left_ticks:.1f}, right={right_ticks:.1f} ticks/sec")

    def set_air_pump_speed(self, speed: int):
        """Set air pump speed (0-100%)"""
        command = {"SetAirPumpSpeed": {"speed": max(0, min(100, speed))}}
        self.send_command(command)
        logger.info(f"Air pump speed set to {speed}%")

    def set_main_brush_speed(self, speed: int):
        """Set main roller brush speed (0-100%)"""
        command = {"SetMainBrushSpeed": {"speed": max(0, min(100, speed))}}
        self.send_command(command)
        logger.info(f"Main brush speed set to {speed}%")

    def enable_lidar(self, pwm: int):
        """
        Enable lidar with specified PWM speed

        Args:
            pwm: PWM duty cycle 0-100% (lidar motor speed)
        """
        command = {"EnableLidar": {"pwm": max(0, min(100, pwm))}}
        self.send_command(command)
        logger.info(f"Lidar enabled at {pwm}% PWM")

    def disable_lidar(self):
        """Disable lidar motor"""
        command = "DisableLidar"
        self.send_command(command)
        logger.info("Lidar disabled")

    def set_lidar_pwm(self, pwm: int):
        """
        Set lidar PWM speed (without power cycling)

        Args:
            pwm: PWM duty cycle 0-100%
        """
        command = {"SetLidarPWM": {"pwm": max(0, min(100, pwm))}}
        self.send_command(command)
        logger.info(f"Lidar PWM set to {pwm}%")

    def emergency_stop(self):
        """Emergency stop all motors and actuators"""
        command = "EmergencyStopAll"
        self.send_command(command)
        logger.warning("EMERGENCY STOP sent")

    def close(self):
        """Close TCP sockets"""
        try:
            self.telemetry_sock.close()
        except:
            pass
        try:
            self.command_sock.close()
        except:
            pass
        logger.info("Disconnected from robot")


def parse_sensor_update(data) -> SensorUpdate:
    """Parse SensorUpdate from deserialized MessagePack

    MessagePack serializes Rust structs as positional arrays by default.
    TelemetryMessage::SensorUpdate(SensorUpdate{...}) -> {"SensorUpdate": [[field1, field2, ...]]}

    The data passed here is already the enum variant content (a list of fields).
    """
    logger.debug(f"parse_sensor_update received: type={type(data)}, len={len(data) if isinstance(data, list) else 'N/A'}")

    # Data should be a list of field values in struct order
    if not isinstance(data, list):
        logger.error(f"Expected list of fields, got {type(data)}: {data}")
        raise TypeError(f"Expected list for SensorUpdate fields, got {type(data)}")

    # Parse positional fields based on struct definition order
    # Ref: sangam-io/src/streaming/messages.rs SensorUpdate struct
    return SensorUpdate(
        timestamp=data[0] if len(data) > 0 else 0,
        battery_level=data[1] if len(data) > 1 else None,
        is_charging=data[2] if len(data) > 2 else None,
        ir_sensor_1=data[3] if len(data) > 3 else None,
        start_button_ir=data[4] if len(data) > 4 else None,
        dock_button_ir=data[5] if len(data) > 5 else None,
        bumper_pressed=data[6] if len(data) > 6 else None,
        cliff_sensors=data[7] if len(data) > 7 else None,
        error_code=data[8] if len(data) > 8 else None,
        encoder_left=data[9] if len(data) > 9 else None,
        encoder_right=data[10] if len(data) > 10 else None,
    )


def parse_connection_quality(data) -> ConnectionQuality:
    """Parse ConnectionQuality from deserialized MessagePack

    MessagePack serializes Rust structs as positional arrays.
    The data is a list of field values in struct definition order.
    """
    if not isinstance(data, list):
        raise TypeError(f"Expected list for ConnectionQuality fields, got {type(data)}")

    # Parse positional fields based on struct definition order
    # Ref: sangam-io/src/streaming/messages.rs ConnectionQuality struct
    return ConnectionQuality(
        timestamp=data[0] if len(data) > 0 else 0,
        rx_packets=data[1] if len(data) > 1 else 0,
        tx_packets=data[2] if len(data) > 2 else 0,
        success_rate=data[3] if len(data) > 3 else 0.0,
        telemetry_fresh=data[4] if len(data) > 4 else False,
    )


def demo_side_brush(robot_ip: str = "vacuum"):
    """
    Demo: Start side brush for 5 seconds, then stop

    Args:
        robot_ip: Robot hostname or IP address
    """
    client = RobotClient(robot_ip=robot_ip)

    try:
        logger.info("=" * 60)
        logger.info("DEMO: Side Brush Test (5 seconds)")
        logger.info("=" * 60)

        # Start side brush at 50% speed
        logger.info("Starting side brush at 50% speed...")
        client.set_side_brush_speed(50)

        # Monitor telemetry for 5 seconds
        start_time = time.time()
        telemetry_count = 0
        last_battery = None
        last_encoders = None

        logger.info("Monitoring telemetry (press Ctrl+C to stop early)...")

        while time.time() - start_time < 5.0:
            result = client.receive_telemetry(timeout_ms=100)

            if result:
                topic, message = result
                telemetry_count += 1

                if topic == "telemetry":
                    # Check if it's SensorUpdate or ConnectionQuality
                    if "SensorUpdate" in message:
                        sensor_data = parse_sensor_update(message["SensorUpdate"])

                        # Log battery level changes
                        if sensor_data.battery_level is not None and sensor_data.battery_level != last_battery:
                            last_battery = sensor_data.battery_level
                            charging = "⚡ CHARGING" if sensor_data.is_charging else ""
                            logger.info(f"  Battery: {sensor_data.battery_level}% {charging}")

                        # Log encoder changes
                        if sensor_data.encoder_left is not None and sensor_data.encoder_right is not None:
                            if (sensor_data.encoder_left, sensor_data.encoder_right) != last_encoders:
                                last_encoders = (sensor_data.encoder_left, sensor_data.encoder_right)
                                logger.debug(f"  Encoders: L={sensor_data.encoder_left}, R={sensor_data.encoder_right}")

                        # Log bumper/error
                        if sensor_data.bumper_pressed:
                            logger.warning("  ⚠️  BUMPER PRESSED")
                        if sensor_data.error_code and sensor_data.error_code > 0:
                            logger.warning(f"  ⚠️  ERROR CODE: {sensor_data.error_code}")

                    elif "ConnectionQuality" in message:
                        quality = parse_connection_quality(message["ConnectionQuality"])
                        logger.info(
                            f"  Connection: {quality.success_rate*100:.1f}% success "
                            f"(RX={quality.rx_packets}, TX={quality.tx_packets})"
                        )

                elif topic == "lidar":
                    scan = message
                    point_count = len(scan.get("points", []))
                    logger.debug(f"  Lidar scan #{scan.get('scan_number', 0)}: {point_count} points")

        logger.info(f"✓ Received {telemetry_count} telemetry messages in 5 seconds")

        # Stop side brush
        logger.info("Stopping side brush...")
        client.set_side_brush_speed(0)

        # Wait a moment for stop command to be sent
        time.sleep(0.2)

        logger.info("=" * 60)
        logger.info("✓ Demo complete")
        logger.info("=" * 60)

    except KeyboardInterrupt:
        logger.info("\nDemo interrupted by user")
        logger.info("Stopping side brush...")
        client.set_side_brush_speed(0)
        time.sleep(0.2)

    finally:
        client.close()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Drishti - Real-time Robot Status Visualization System"
    )
    parser.add_argument(
        "--robot",
        default="vacuum",
        help="Robot hostname or IP address (default: vacuum)"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose logging"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    try:
        demo_side_brush(robot_ip=args.robot)
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
