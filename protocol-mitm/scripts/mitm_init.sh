#!/bin/sh /etc/rc.common
# MITM Logger Init Script
# Starts serial MITM proxy BEFORE Monitor/AuxCtrl

START=89
STOP=10

ENABLED_FLAG="/mnt/UDISK/mitm_enabled"
MITM_BINARY="/mnt/UDISK/binary/serial_mitm"
RUN_COUNTER_FILE="/mnt/UDISK/mitm_run_counter"
REAL_PORT="/dev/ttyS3"
HARDWARE_PORT="/dev/ttyS3_hardware"
VIRTUAL_PORT="/tmp/ttyS3_tap"
LOG_DIR="/mnt/UDISK/log"

start() {
	echo "[MITM Init] Checking if MITM mode is enabled..."

	# Only run if flag file exists
	if [ ! -f "$ENABLED_FLAG" ]; then
		echo "[MITM Init] MITM mode disabled. Skipping."
		return 0
	fi

	echo "[MITM Init] MITM mode ENABLED. Starting setup..."

	# Check if binary exists
	if [ ! -f "$MITM_BINARY" ]; then
		echo "[MITM Init] ERROR: MITM binary not found at $MITM_BINARY"
		return 1
	fi

	# Create log directory
	mkdir -p "$LOG_DIR"

	# Read run counter
	if [ -f "$RUN_COUNTER_FILE" ]; then
		RUN_NUM=$(cat "$RUN_COUNTER_FILE")
	else
		RUN_NUM=1
		echo 1 > "$RUN_COUNTER_FILE"
	fi

	echo "[MITM Init] Current run: $RUN_NUM"

	# Check if real serial port exists
	if [ ! -c "$REAL_PORT" ]; then
		echo "[MITM Init] ERROR: Serial port $REAL_PORT not found!"
		return 1
	fi

	# Check if already set up
	if [ -c "$HARDWARE_PORT" ]; then
		echo "[MITM Init] MITM already set up (found $HARDWARE_PORT)"
	else
		# Move real port to backup location
		echo "[MITM Init] Moving $REAL_PORT -> $HARDWARE_PORT"
		mv "$REAL_PORT" "$HARDWARE_PORT"

		if [ $? -ne 0 ]; then
			echo "[MITM Init] ERROR: Failed to move serial port"
			return 1
		fi
	fi

	# Start MITM proxy in background
	echo "[MITM Init] Starting MITM proxy (run $RUN_NUM)..."
	$MITM_BINARY $RUN_NUM >/tmp/serial_mitm.log 2>&1 &
	MITM_PID=$!

	# Save PID
	echo $MITM_PID > /var/run/mitm.pid

	# Wait for PTY to be created
	sleep 2

	# Check if virtual port was created
	if [ ! -L "$VIRTUAL_PORT" ]; then
		echo "[MITM Init] ERROR: Virtual port $VIRTUAL_PORT not created!"
		kill $MITM_PID 2>/dev/null
		mv "$HARDWARE_PORT" "$REAL_PORT" 2>/dev/null
		return 1
	fi

	# Create symlink /dev/ttyS3 -> /tmp/ttyS3_tap
	echo "[MITM Init] Creating symlink $REAL_PORT -> $VIRTUAL_PORT"
	ln -sf "$VIRTUAL_PORT" "$REAL_PORT"

	if [ $? -ne 0 ]; then
		echo "[MITM Init] ERROR: Failed to create symlink"
		kill $MITM_PID 2>/dev/null
		mv "$HARDWARE_PORT" "$REAL_PORT" 2>/dev/null
		return 1
	fi

	echo "[MITM Init] ✓ MITM setup complete!"
	echo "[MITM Init]   - MITM proxy running (PID: $MITM_PID)"
	echo "[MITM Init]   - Real port: $HARDWARE_PORT"
	echo "[MITM Init]   - Virtual port: $VIRTUAL_PORT (via PTY)"
	echo "[MITM Init]   - Symlink: $REAL_PORT -> $VIRTUAL_PORT"
	echo "[MITM Init]   - Run number: $RUN_NUM"
	echo ""

	return 0
}

stop() {
	echo "[MITM Init] Stopping MITM proxy..."

	# Kill MITM process
	if [ -f /var/run/mitm.pid ]; then
		PID=$(cat /var/run/mitm.pid)
		kill -TERM $PID 2>/dev/null
		sleep 1
		kill -9 $PID 2>/dev/null
		rm -f /var/run/mitm.pid
	else
		killall -9 serial_mitm 2>/dev/null
	fi

	# Remove symlink
	if [ -L "$REAL_PORT" ]; then
		rm -f "$REAL_PORT"
	fi

	# Restore original port
	if [ -c "$HARDWARE_PORT" ]; then
		mv "$HARDWARE_PORT" "$REAL_PORT" 2>/dev/null
	fi

	# Remove virtual port
	rm -f "$VIRTUAL_PORT"

	echo "[MITM Init] ✓ MITM stopped and cleaned up"

	return 0
}
