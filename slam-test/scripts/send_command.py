#!/usr/bin/env python3
"""
Send commands to DhruvaSLAM via TCP.

Usage:
    python send_command.py start_mapping [--name NAME] [--port PORT]
    python send_command.py stop_mapping [--save] [--port PORT]
    python send_command.py emergency_stop [--port PORT]
"""

import argparse
import socket
import struct
import sys
import uuid
from pathlib import Path

# Add proto directory to path
SCRIPT_DIR = Path(__file__).parent
PROTO_DIR = SCRIPT_DIR / "proto"
sys.path.insert(0, str(PROTO_DIR))

try:
    import dhruva_pb2
except ImportError:
    print("ERROR: Proto files not compiled. Run:")
    print(f"  protoc --python_out={PROTO_DIR} -I ../dhruva-slam/proto dhruva.proto")
    print("")
    print("Or install protobuf: pip install protobuf")
    sys.exit(1)


def send_message(sock: socket.socket, msg: bytes) -> None:
    """Send length-prefixed message."""
    length = struct.pack(">I", len(msg))
    sock.sendall(length + msg)


def recv_message(sock: socket.socket, timeout: float = 5.0) -> bytes:
    """Receive length-prefixed message."""
    sock.settimeout(timeout)

    # Read length (4 bytes)
    length_data = b""
    while len(length_data) < 4:
        chunk = sock.recv(4 - len(length_data))
        if not chunk:
            raise ConnectionError("Connection closed")
        length_data += chunk

    length = struct.unpack(">I", length_data)[0]

    # Read message
    data = b""
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            raise ConnectionError("Connection closed")
        data += chunk

    return data


def start_mapping(host: str, port: int, name: str) -> bool:
    """Send StartMapping command."""
    request_id = str(uuid.uuid4())[:8]

    cmd = dhruva_pb2.DhruvaCommand()
    cmd.request_id = request_id
    cmd.start_mapping.map_name = name

    print(f"Connecting to {host}:{port}...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        print(f"Sending StartMapping (name={name})...")
        send_message(sock, cmd.SerializeToString())

        # Wait for response
        try:
            response_data = recv_message(sock)
            response = dhruva_pb2.DhruvaResponse()
            response.ParseFromString(response_data)

            if response.success:
                print(f"SUCCESS: Mapping started (map_id={response.mapping_started.map_id})")
                return True
            else:
                print(f"FAILED: {response.error_message}")
                return False
        except Exception as e:
            print(f"No response received (may still have started): {e}")
            return True  # Assume success if no error


def stop_mapping(host: str, port: int, save: bool) -> bool:
    """Send StopMapping command."""
    request_id = str(uuid.uuid4())[:8]

    cmd = dhruva_pb2.DhruvaCommand()
    cmd.request_id = request_id
    cmd.stop_mapping.save = save

    print(f"Connecting to {host}:{port}...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        print(f"Sending StopMapping (save={save})...")
        send_message(sock, cmd.SerializeToString())

        try:
            response_data = recv_message(sock)
            response = dhruva_pb2.DhruvaResponse()
            response.ParseFromString(response_data)

            if response.success:
                if response.HasField("mapping_stopped"):
                    ms = response.mapping_stopped
                    print(f"SUCCESS: Mapping stopped (saved={ms.saved}, area={ms.area_m2:.2f}m²)")
                else:
                    print("SUCCESS: Mapping stopped")
                return True
            else:
                print(f"FAILED: {response.error_message}")
                return False
        except Exception as e:
            print(f"No response received: {e}")
            return True


def emergency_stop(host: str, port: int) -> bool:
    """Send EmergencyStop command."""
    request_id = str(uuid.uuid4())[:8]

    cmd = dhruva_pb2.DhruvaCommand()
    cmd.request_id = request_id
    cmd.emergency_stop.CopyFrom(dhruva_pb2.EmergencyStopCommand())

    print(f"Connecting to {host}:{port}...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        print("Sending EmergencyStop...")
        send_message(sock, cmd.SerializeToString())

        try:
            response_data = recv_message(sock)
            response = dhruva_pb2.DhruvaResponse()
            response.ParseFromString(response_data)

            if response.success:
                print("SUCCESS: Emergency stop executed")
                return True
            else:
                print(f"FAILED: {response.error_message}")
                return False
        except Exception as e:
            print(f"No response received: {e}")
            return True


def main():
    parser = argparse.ArgumentParser(description="Send commands to DhruvaSLAM")
    parser.add_argument("--host", default="localhost", help="DhruvaSLAM host")
    parser.add_argument("--port", type=int, default=5557, help="DhruvaSLAM port")

    subparsers = parser.add_subparsers(dest="command", required=True)

    # start_mapping
    start_parser = subparsers.add_parser("start_mapping", help="Start mapping")
    start_parser.add_argument("--name", default="test_map", help="Map name")

    # stop_mapping
    stop_parser = subparsers.add_parser("stop_mapping", help="Stop mapping")
    stop_parser.add_argument("--save", action="store_true", help="Save map before stopping")

    # emergency_stop
    subparsers.add_parser("emergency_stop", help="Emergency stop")

    args = parser.parse_args()

    try:
        if args.command == "start_mapping":
            success = start_mapping(args.host, args.port, args.name)
        elif args.command == "stop_mapping":
            success = stop_mapping(args.host, args.port, args.save)
        elif args.command == "emergency_stop":
            success = emergency_stop(args.host, args.port)
        else:
            print(f"Unknown command: {args.command}")
            sys.exit(1)

        sys.exit(0 if success else 1)

    except ConnectionRefusedError:
        print(f"ERROR: Could not connect to {args.host}:{args.port}")
        print("Is DhruvaSLAM running?")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
