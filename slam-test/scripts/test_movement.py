#!/usr/bin/env python3
"""
Movement and Scan Matching Test Script

Tests robot movement accuracy independently from exploration/mapping logic.
Uses SetGoal commands to move robot to specific locations and monitors
pose estimates to check for drift and rotation errors.

Test modes:
1. rotation_only  - Pure rotation tests (no translation)
2. linear_only    - Straight line movement tests
3. square         - Square path to test cumulative error
4. waypoints      - Custom waypoint sequence

Usage:
    python test_movement.py --mode rotation_only --port 5557
    python test_movement.py --mode linear_only --distance 1.0
    python test_movement.py --mode square --size 1.0
    python test_movement.py --mode waypoints --waypoints "1,1,0;2,1,0;2,2,0"
"""

import argparse
import json
import math
import socket
import struct
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

# Add proto directory to path
SCRIPT_DIR = Path(__file__).parent
PROTO_DIR = SCRIPT_DIR / "proto"
sys.path.insert(0, str(PROTO_DIR))

try:
    import dhruva_pb2
except ImportError:
    print("ERROR: Proto files not compiled. Run 'make proto' first.")
    sys.exit(1)


@dataclass
class Pose:
    """2D pose with timestamp."""
    x: float
    y: float
    theta: float
    timestamp: float = 0.0

    def distance_to(self, other: 'Pose') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def angle_diff(self, other: 'Pose') -> float:
        """Compute smallest angle difference."""
        diff = self.theta - other.theta
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def __str__(self):
        return f"({self.x:.3f}, {self.y:.3f}, {math.degrees(self.theta):.1f}°)"


@dataclass
class GoalResult:
    """Result of a single goal navigation."""
    goal: Pose
    start_pose: Pose
    end_pose: Pose
    expected_pose: Pose
    duration_s: float
    position_error: float
    rotation_error: float
    success: bool
    status_message: str = ""


@dataclass
class TestResult:
    """Complete test results."""
    test_name: str
    test_mode: str
    timestamp: str
    goals: List[GoalResult] = field(default_factory=list)
    total_duration_s: float = 0.0

    def summary(self) -> dict:
        if not self.goals:
            return {"error": "No goals executed"}

        pos_errors = [g.position_error for g in self.goals]
        rot_errors = [abs(g.rotation_error) for g in self.goals]

        return {
            "test_name": self.test_name,
            "test_mode": self.test_mode,
            "timestamp": self.timestamp,
            "num_goals": len(self.goals),
            "success_rate": sum(1 for g in self.goals if g.success) / len(self.goals),
            "position_error": {
                "mean_m": sum(pos_errors) / len(pos_errors),
                "max_m": max(pos_errors),
                "min_m": min(pos_errors),
            },
            "rotation_error": {
                "mean_deg": math.degrees(sum(rot_errors) / len(rot_errors)),
                "max_deg": math.degrees(max(rot_errors)),
                "min_deg": math.degrees(min(rot_errors)),
            },
            "total_duration_s": self.total_duration_s,
        }


class MovementTester:
    """Test robot movement using SetGoal commands."""

    def __init__(self, host: str = "localhost", port: int = 5557):
        self.host = host
        self.port = port
        self.current_pose: Optional[Pose] = None
        self.nav_state: int = 0  # NAV_STATE_IDLE
        self.pose_history: List[Pose] = []
        self._lock = threading.Lock()
        self._running = False
        self._udp_thread: Optional[threading.Thread] = None

    def _send_command(self, cmd: dhruva_pb2.DhruvaCommand,
                      timeout: float = 5.0) -> Optional[dhruva_pb2.DhruvaResponse]:
        """Send command and receive response."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            sock.connect((self.host, self.port))

            # Send length-prefixed message
            data = cmd.SerializeToString()
            sock.sendall(struct.pack(">I", len(data)) + data)

            # Receive response
            try:
                length_data = b""
                while len(length_data) < 4:
                    chunk = sock.recv(4 - len(length_data))
                    if not chunk:
                        return None
                    length_data += chunk

                length = struct.unpack(">I", length_data)[0]

                resp_data = b""
                while len(resp_data) < length:
                    chunk = sock.recv(length - len(resp_data))
                    if not chunk:
                        return None
                    resp_data += chunk

                response = dhruva_pb2.DhruvaResponse()
                response.ParseFromString(resp_data)
                return response
            except socket.timeout:
                return None

    def _start_udp_listener(self):
        """Start UDP listener for pose updates.

        Uses SO_REUSEADDR and SO_REUSEPORT to share port with TCP.
        The TCP connection registers us for UDP streaming.
        """
        self._running = True

        # Use a dedicated port for UDP reception
        # We'll bind both UDP and TCP to this port
        self._udp_port = 5560  # Different from server port

        # Create UDP socket first
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass  # SO_REUSEPORT not available on all platforms
        self._udp_sock.bind(("0.0.0.0", self._udp_port))
        self._udp_sock.settimeout(0.5)

        # Create TCP socket bound to same port for registration
        self._tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        self._tcp_sock.bind(("0.0.0.0", self._udp_port))
        self._tcp_sock.connect((self.host, self.port))

        # Start listener thread
        self._udp_thread = threading.Thread(target=self._udp_listener_loop, daemon=True)
        self._udp_thread.start()

    def _stop_udp_listener(self):
        """Stop UDP listener."""
        self._running = False
        if self._udp_thread:
            self._udp_thread.join(timeout=2.0)
        if hasattr(self, '_tcp_sock') and self._tcp_sock:
            try:
                self._tcp_sock.close()
            except:
                pass
        if hasattr(self, '_udp_sock') and self._udp_sock:
            try:
                self._udp_sock.close()
            except:
                pass

    def _udp_listener_loop(self):
        """Listen for UDP status updates."""
        while self._running:
            try:
                data, addr = self._udp_sock.recvfrom(65535)
                if len(data) < 4:
                    continue

                length = struct.unpack(">I", data[:4])[0]
                if len(data) < 4 + length:
                    continue

                msg = dhruva_pb2.DhruvaStream()
                msg.ParseFromString(data[4:4+length])

                timestamp = msg.timestamp_us / 1e6

                if msg.HasField("robot_status"):
                    rs = msg.robot_status
                    with self._lock:
                        self.current_pose = Pose(
                            x=rs.pose.x,
                            y=rs.pose.y,
                            theta=rs.pose.theta,
                            timestamp=timestamp
                        )
                        self.pose_history.append(self.current_pose)

                if msg.HasField("navigation_status"):
                    with self._lock:
                        self.nav_state = msg.navigation_status.state

            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"UDP error: {e}")

    def get_current_pose(self) -> Optional[Pose]:
        """Get current pose (thread-safe)."""
        with self._lock:
            return self.current_pose

    def get_nav_state(self) -> int:
        """Get current navigation state (thread-safe)."""
        with self._lock:
            return self.nav_state

    def start_mapping(self, name: str = "movement_test") -> bool:
        """Start a mapping session."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())[:8]
        cmd.start_mapping.map_name = name

        response = self._send_command(cmd)
        if response and response.success:
            print(f"  Mapping started (map_id={response.mapping_started.map_id})")
            return True
        else:
            print(f"  Failed to start mapping: {response.error_message if response else 'No response'}")
            return False

    def stop_mapping(self, save: bool = False) -> bool:
        """Stop mapping session."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())[:8]
        cmd.stop_mapping.save = save

        response = self._send_command(cmd)
        return response is not None and response.success

    def set_goal(self, x: float, y: float, theta: Optional[float] = None) -> bool:
        """Send SetGoal command."""
        cmd = dhruva_pb2.DhruvaCommand()
        cmd.request_id = str(uuid.uuid4())[:8]
        cmd.set_goal.x = x
        cmd.set_goal.y = y
        if theta is not None:
            cmd.set_goal.theta = theta
        cmd.set_goal.description = f"Test goal ({x:.2f}, {y:.2f})"

        response = self._send_command(cmd)
        if response and response.success:
            return True
        else:
            print(f"  Failed to set goal: {response.error_message if response else 'No response'}")
            return False

    def wait_for_navigation_complete(self, timeout: float = 30.0) -> Tuple[bool, str]:
        """Wait for navigation to complete or fail."""
        start = time.time()

        # Wait for navigation to start
        while time.time() - start < 5.0:
            state = self.get_nav_state()
            if state in [2, 3]:  # NAVIGATING or ROTATING_TO_HEADING
                break
            time.sleep(0.1)

        # Wait for completion
        while time.time() - start < timeout:
            state = self.get_nav_state()

            if state == 4:  # NAV_STATE_REACHED
                return True, "Goal reached"
            elif state == 5:  # NAV_STATE_FAILED
                return False, "Navigation failed"
            elif state == 6:  # NAV_STATE_CANCELLED
                return False, "Navigation cancelled"
            elif state == 0:  # NAV_STATE_IDLE
                # Might have completed very quickly
                return True, "Navigation idle (possibly completed)"

            time.sleep(0.1)

        return False, "Navigation timeout"

    def execute_goal(self, goal: Pose, timeout: float = 30.0) -> GoalResult:
        """Execute a single goal and measure result."""
        start_pose = self.get_current_pose()
        if start_pose is None:
            start_pose = Pose(0, 0, 0)

        start_time = time.time()

        # Send goal
        success = self.set_goal(goal.x, goal.y, goal.theta if goal.theta != 0 else None)
        if not success:
            return GoalResult(
                goal=goal,
                start_pose=start_pose,
                end_pose=start_pose,
                expected_pose=goal,
                duration_s=0,
                position_error=goal.distance_to(start_pose),
                rotation_error=goal.angle_diff(start_pose),
                success=False,
                status_message="Failed to send goal"
            )

        # Wait for completion
        nav_success, status_msg = self.wait_for_navigation_complete(timeout)

        duration = time.time() - start_time
        end_pose = self.get_current_pose()
        if end_pose is None:
            end_pose = start_pose

        # Compute errors
        # Expected pose is the goal
        position_error = end_pose.distance_to(goal)
        rotation_error = end_pose.angle_diff(goal)

        return GoalResult(
            goal=goal,
            start_pose=start_pose,
            end_pose=end_pose,
            expected_pose=goal,
            duration_s=duration,
            position_error=position_error,
            rotation_error=rotation_error,
            success=nav_success and position_error < 0.5,  # 50cm tolerance
            status_message=status_msg
        )

    def test_rotation_only(self, angles_deg: List[float] = None) -> TestResult:
        """Test rotation accuracy using small movements that end with target heading.

        Note: Pure rotation (same position, different heading) is not supported
        by the current path planner. We use small movements instead.
        """
        if angles_deg is None:
            angles_deg = [90, 180, -90, 45, -45]

        result = TestResult(
            test_name="rotation_test",
            test_mode="rotation_only",
            timestamp=datetime.now().isoformat()
        )

        current_pose = self.get_current_pose()
        if current_pose is None:
            print("ERROR: Cannot get current pose")
            return result

        print(f"\n  Starting rotation test from {current_pose}")
        print("  Note: Using small movements (0.3m) to test rotation accuracy")
        start_time = time.time()

        # Use small movements with target heading
        move_distance = 0.3  # Small movement to trigger path planning

        for angle_deg in angles_deg:
            target_theta = math.radians(angle_deg)

            # Move slightly in the direction of target heading
            goal = Pose(
                x=current_pose.x + move_distance * math.cos(target_theta),
                y=current_pose.y + move_distance * math.sin(target_theta),
                theta=target_theta
            )

            print(f"  Moving 0.3m toward {angle_deg}° to {goal}...")
            goal_result = self.execute_goal(goal, timeout=20.0)
            result.goals.append(goal_result)

            status = "OK" if goal_result.success else "FAIL"
            print(f"    {status}: Position error = {goal_result.position_error*100:.1f}cm, "
                  f"Rotation error = {math.degrees(goal_result.rotation_error):.1f}°")

            # Update current pose for next goal
            new_pose = self.get_current_pose()
            if new_pose:
                current_pose = new_pose

            time.sleep(0.5)  # Pause between tests

        result.total_duration_s = time.time() - start_time
        return result

    def test_linear_only(self, distance: float = 1.0, directions: List[float] = None) -> TestResult:
        """Test straight line movement accuracy."""
        if directions is None:
            directions = [0, 90, 180, 270]  # Forward, left, back, right

        result = TestResult(
            test_name="linear_test",
            test_mode="linear_only",
            timestamp=datetime.now().isoformat()
        )

        start_pose = self.get_current_pose()
        if start_pose is None:
            print("ERROR: Cannot get current pose")
            return result

        print(f"\n  Starting linear test from {start_pose}")
        start_time = time.time()

        for direction_deg in directions:
            direction_rad = math.radians(direction_deg)
            goal = Pose(
                x=start_pose.x + distance * math.cos(direction_rad),
                y=start_pose.y + distance * math.sin(direction_rad),
                theta=direction_rad
            )

            print(f"  Moving {distance}m in direction {direction_deg}° to {goal}...")
            goal_result = self.execute_goal(goal)
            result.goals.append(goal_result)

            status = "OK" if goal_result.success else "FAIL"
            print(f"    {status}: Position error = {goal_result.position_error*100:.1f}cm, "
                  f"Rotation error = {math.degrees(goal_result.rotation_error):.1f}°")

            # Update start pose for next movement
            current = self.get_current_pose()
            if current:
                start_pose = current

            time.sleep(1.0)

        result.total_duration_s = time.time() - start_time
        return result

    def test_square(self, size: float = 1.0) -> TestResult:
        """Test square path to measure cumulative error."""
        result = TestResult(
            test_name="square_test",
            test_mode="square",
            timestamp=datetime.now().isoformat()
        )

        start_pose = self.get_current_pose()
        if start_pose is None:
            print("ERROR: Cannot get current pose")
            return result

        print(f"\n  Starting square test ({size}m x {size}m) from {start_pose}")
        start_time = time.time()

        # Define square corners relative to start
        corners = [
            (start_pose.x + size, start_pose.y, 0),          # Right
            (start_pose.x + size, start_pose.y + size, math.pi/2),  # Up
            (start_pose.x, start_pose.y + size, math.pi),    # Left
            (start_pose.x, start_pose.y, -math.pi/2),        # Back to start
        ]

        for i, (x, y, theta) in enumerate(corners):
            goal = Pose(x=x, y=y, theta=theta)
            print(f"  Corner {i+1}/4: Moving to {goal}...")

            goal_result = self.execute_goal(goal)
            result.goals.append(goal_result)

            status = "OK" if goal_result.success else "FAIL"
            print(f"    {status}: Position error = {goal_result.position_error*100:.1f}cm, "
                  f"Rotation error = {math.degrees(goal_result.rotation_error):.1f}°")

            time.sleep(0.5)

        # Measure return-to-start error
        final_pose = self.get_current_pose()
        if final_pose:
            return_error = final_pose.distance_to(start_pose)
            print(f"\n  Return-to-start error: {return_error*100:.1f}cm")

        result.total_duration_s = time.time() - start_time
        return result

    def test_waypoints(self, waypoints: List[Tuple[float, float, float]]) -> TestResult:
        """Test custom waypoint sequence."""
        result = TestResult(
            test_name="waypoints_test",
            test_mode="waypoints",
            timestamp=datetime.now().isoformat()
        )

        print(f"\n  Starting waypoint test with {len(waypoints)} waypoints")
        start_time = time.time()

        for i, (x, y, theta) in enumerate(waypoints):
            goal = Pose(x=x, y=y, theta=theta)
            print(f"  Waypoint {i+1}/{len(waypoints)}: Moving to {goal}...")

            goal_result = self.execute_goal(goal)
            result.goals.append(goal_result)

            status = "OK" if goal_result.success else "FAIL"
            print(f"    {status}: Position error = {goal_result.position_error*100:.1f}cm, "
                  f"Rotation error = {math.degrees(goal_result.rotation_error):.1f}°")

            time.sleep(0.5)

        result.total_duration_s = time.time() - start_time
        return result


def parse_waypoints(waypoints_str: str) -> List[Tuple[float, float, float]]:
    """Parse waypoints from string format 'x,y,theta;x,y,theta;...'"""
    waypoints = []
    for wp in waypoints_str.split(";"):
        parts = [float(p.strip()) for p in wp.split(",")]
        if len(parts) >= 2:
            x, y = parts[0], parts[1]
            theta = math.radians(parts[2]) if len(parts) > 2 else 0.0
            waypoints.append((x, y, theta))
    return waypoints


def main():
    parser = argparse.ArgumentParser(
        description="Test robot movement and scan matching accuracy",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test rotation accuracy
  python test_movement.py --mode rotation_only

  # Test linear movement (1m in each cardinal direction)
  python test_movement.py --mode linear_only --distance 1.0

  # Test square path (0.5m x 0.5m square)
  python test_movement.py --mode square --size 0.5

  # Test custom waypoints (x,y,theta_deg separated by semicolons)
  python test_movement.py --mode waypoints --waypoints "0.5,0,0;0.5,0.5,90;0,0.5,180;0,0,-90"
        """
    )

    parser.add_argument("--host", default="localhost", help="DhruvaSLAM host")
    parser.add_argument("--port", type=int, default=5557, help="DhruvaSLAM port")
    parser.add_argument("--mode", required=True,
                        choices=["rotation_only", "linear_only", "square", "waypoints"],
                        help="Test mode")
    parser.add_argument("--distance", type=float, default=1.0,
                        help="Distance for linear tests (meters)")
    parser.add_argument("--size", type=float, default=1.0,
                        help="Size for square test (meters)")
    parser.add_argument("--waypoints", type=str,
                        help="Waypoints for custom test: 'x,y,theta_deg;...'")
    parser.add_argument("--output", type=str,
                        help="Output JSON file for results")
    parser.add_argument("--skip-mapping", action="store_true",
                        help="Skip starting mapping (assume already started)")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="Goal timeout in seconds")

    args = parser.parse_args()

    print("=" * 60)
    print("  Movement & Scan Matching Test")
    print("=" * 60)
    print(f"  Host: {args.host}:{args.port}")
    print(f"  Mode: {args.mode}")

    tester = MovementTester(args.host, args.port)

    try:
        # Start UDP listener for pose updates
        print("\n[1] Starting pose monitor...")
        tester._start_udp_listener()
        time.sleep(1.0)  # Let UDP start

        # Start mapping if needed
        if not args.skip_mapping:
            print("\n[2] Starting mapping session...")
            if not tester.start_mapping("movement_test"):
                print("ERROR: Failed to start mapping")
                sys.exit(1)

            # Wait longer for mapping to build initial map data
            # The path planner needs map data to plan paths
            print("  Waiting for initial scan and map data (15s)...")
            time.sleep(15.0)

        # Wait for first pose
        print("\n[3] Waiting for initial pose...")
        for _ in range(50):  # 5 seconds
            if tester.get_current_pose() is not None:
                break
            time.sleep(0.1)

        initial_pose = tester.get_current_pose()
        if initial_pose is None:
            print("ERROR: Could not get initial pose")
            sys.exit(1)
        print(f"  Initial pose: {initial_pose}")

        # Run test
        print(f"\n[4] Running {args.mode} test...")

        if args.mode == "rotation_only":
            result = tester.test_rotation_only()
        elif args.mode == "linear_only":
            result = tester.test_linear_only(args.distance)
        elif args.mode == "square":
            result = tester.test_square(args.size)
        elif args.mode == "waypoints":
            if not args.waypoints:
                print("ERROR: --waypoints required for waypoints mode")
                sys.exit(1)
            waypoints = parse_waypoints(args.waypoints)
            result = tester.test_waypoints(waypoints)

        # Print summary
        print("\n" + "=" * 60)
        print("  Test Summary")
        print("=" * 60)
        summary = result.summary()
        print(f"  Mode: {summary['test_mode']}")
        print(f"  Goals executed: {summary['num_goals']}")
        print(f"  Success rate: {summary['success_rate']*100:.1f}%")
        print(f"  Position error (mean): {summary['position_error']['mean_m']*100:.1f}cm")
        print(f"  Position error (max):  {summary['position_error']['max_m']*100:.1f}cm")
        print(f"  Rotation error (mean): {summary['rotation_error']['mean_deg']:.1f}°")
        print(f"  Rotation error (max):  {summary['rotation_error']['max_deg']:.1f}°")
        print(f"  Total duration: {summary['total_duration_s']:.1f}s")

        # Save results if requested
        if args.output:
            output_path = Path(args.output)
            output_path.parent.mkdir(parents=True, exist_ok=True)

            # Convert to JSON-serializable format
            output_data = {
                "summary": summary,
                "goals": [
                    {
                        "goal": {"x": g.goal.x, "y": g.goal.y, "theta": g.goal.theta},
                        "start": {"x": g.start_pose.x, "y": g.start_pose.y, "theta": g.start_pose.theta},
                        "end": {"x": g.end_pose.x, "y": g.end_pose.y, "theta": g.end_pose.theta},
                        "position_error_m": g.position_error,
                        "rotation_error_rad": g.rotation_error,
                        "duration_s": g.duration_s,
                        "success": g.success,
                        "status": g.status_message
                    }
                    for g in result.goals
                ],
                "pose_history": [
                    {"x": p.x, "y": p.y, "theta": p.theta, "t": p.timestamp}
                    for p in tester.pose_history[-1000:]  # Last 1000 poses
                ]
            }

            with open(output_path, 'w') as f:
                json.dump(output_data, f, indent=2)
            print(f"\n  Results saved to: {output_path}")

        # Stop mapping
        if not args.skip_mapping:
            print("\n[5] Stopping mapping...")
            tester.stop_mapping(save=False)

        print("\nTest complete!")

        # Exit with error if tests failed
        if summary['success_rate'] < 0.5:
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        tester.stop_mapping(save=False)
        sys.exit(1)
    except ConnectionRefusedError:
        print(f"\nERROR: Could not connect to {args.host}:{args.port}")
        print("Is DhruvaSLAM running?")
        sys.exit(1)
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        tester._stop_udp_listener()


if __name__ == "__main__":
    main()
