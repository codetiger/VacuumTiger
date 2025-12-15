#!/usr/bin/env python3
"""Debug script to capture and analyze DhruvaSLAM proto messages."""

import socket
import struct
import sys
sys.path.insert(0, 'proto')
from proto import dhruva_pb2

HOST = 'localhost'
PORT = 5557

def main():
    print(f"Connecting to {HOST}:{PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(30.0)
    sock.connect((HOST, PORT))
    print("Connected!")

    # Send StartMapping command to trigger data flow
    print("Sending StartMapping command...")
    cmd = dhruva_pb2.DhruvaCommand()
    cmd.request_id = "debug-001"
    cmd.start_mapping.map_name = "DebugMap"
    payload = cmd.SerializeToString()
    length = struct.pack('>I', len(payload))
    sock.sendall(length + payload)
    print(f"Sent {len(payload)} byte command")

    # Read response first
    print("Waiting for response...")
    try:
        resp_len_data = sock.recv(4)
        if resp_len_data:
            resp_len = struct.unpack('>I', resp_len_data)[0]
            resp_data = b''
            while len(resp_data) < resp_len:
                chunk = sock.recv(resp_len - len(resp_data))
                if not chunk:
                    break
                resp_data += chunk
            resp = dhruva_pb2.DhruvaResponse()
            resp.ParseFromString(resp_data)
            print(f"Response: success={resp.success}, error={resp.error_message}")
    except Exception as e:
        print(f"Response read error: {e}")

    print("Waiting for stream data...")

    try:
        while True:
            # Read length prefix
            length_data = b''
            while len(length_data) < 4:
                chunk = sock.recv(4 - len(length_data))
                if not chunk:
                    print("Connection closed")
                    return
                length_data += chunk

            length = struct.unpack('>I', length_data)[0]
            print(f"\nMessage length: {length} bytes")

            if length > 10 * 1024 * 1024:
                print(f"ERROR: Message too large!")
                return

            # Read message body
            data = b''
            while len(data) < length:
                chunk = sock.recv(min(65536, length - len(data)))
                if not chunk:
                    print("Connection closed mid-message")
                    return
                data += chunk

            print(f"Received {len(data)} bytes")
            print(f"First 100 bytes: {data[:100].hex()}")
            print(f"Last 100 bytes: {data[-100:].hex()}")

            # Save to file
            with open('/tmp/proto_message.bin', 'wb') as f:
                f.write(data)
            print("Saved to /tmp/proto_message.bin")

            # Try to parse
            try:
                msg = dhruva_pb2.DhruvaStream()
                msg.ParseFromString(data)
                which = msg.WhichOneof('data')
                print(f"SUCCESS! Message type: {which}")
                if which == 'current_map':
                    cm = msg.current_map
                    print(f"  map_id: {cm.map_id}")
                    print(f"  size: {cm.width}x{cm.height}")
                    print(f"  cells len: {len(cm.cells)}")
            except Exception as e:
                print(f"PARSE ERROR: {e}")

                # Try to find where the error is
                print("\nAnalyzing message structure...")
                analyze_proto_bytes(data)

            # Only process first message
            break

    finally:
        sock.close()

def decode_varint(data, start):
    """Decode a varint from data starting at start."""
    result = 0
    shift = 0
    pos = start
    while pos < len(data):
        byte = data[pos]
        result |= (byte & 0x7f) << shift
        pos += 1
        if not (byte & 0x80):
            return result, pos
        shift += 7
    return None, pos

def analyze_proto_bytes(data):
    """Analyze proto wire format."""
    pos = 0

    # Parse outer DhruvaStream
    print(f"[{pos}] First byte: {data[pos]:02x}")

    # Field 1 = timestamp_us (varint)
    if data[pos] == 0x08:
        pos += 1
        ts, pos = decode_varint(data, pos)
        print(f"  timestamp_us = {ts}")

    # Field 12 = current_map (length-delimited)
    if pos < len(data) and data[pos] == 0x62:
        pos += 1
        cm_len, pos = decode_varint(data, pos)
        print(f"  current_map length = {cm_len}")
        cm_start = pos
        cm_end = pos + cm_len

        # Parse CurrentMap fields
        print(f"\n  Parsing CurrentMap ({cm_len} bytes):")
        while pos < cm_end and pos < len(data):
            if pos >= len(data):
                break
            tag = data[pos]
            field = tag >> 3
            wire = tag & 7
            pos += 1

            if wire == 0:  # varint
                val, pos = decode_varint(data, pos)
                print(f"    field {field}: varint = {val}")
            elif wire == 2:  # length-delimited
                length, pos = decode_varint(data, pos)
                if pos + length <= len(data):
                    val = data[pos:pos+length]
                    if length < 50:
                        try:
                            print(f"    field {field}: len={length}, str='{val.decode()}'")
                        except:
                            print(f"    field {field}: len={length}, bytes={val[:20].hex()}...")
                    else:
                        print(f"    field {field}: len={length}, bytes (large)")
                    pos += length
                else:
                    print(f"    field {field}: len={length} TRUNCATED!")
                    break
            elif wire == 5:  # 32-bit
                if pos + 4 <= len(data):
                    val = struct.unpack('<f', data[pos:pos+4])[0]
                    print(f"    field {field}: float = {val}")
                    pos += 4
                else:
                    print(f"    field {field}: 32-bit TRUNCATED!")
                    break
            else:
                print(f"    field {field}: UNKNOWN wire type {wire}!")
                break

        print(f"\n  Parsed up to byte {pos} of {len(data)}")

if __name__ == '__main__':
    main()
