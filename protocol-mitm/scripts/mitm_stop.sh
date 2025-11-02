#!/bin/sh
#
# MITM Stop Script
# Run this when vacuum cleaning is complete and robot has returned to dock
#
# Usage: /mnt/UDISK/mitm_stop.sh
#
# This script:
#   1. Sends SIGTERM to serial_mitm for graceful shutdown
#   2. Increments run counter
#   3. Shows statistics about the capture
#   4. Prepares for next iteration

PID_FILE="/tmp/mitm.pid"
RUN_COUNTER_FILE="/mnt/UDISK/mitm_run_counter"
LOG_DIR="/mnt/UDISK/log"
TARGET_RUNS=5

echo "========================================="
echo "  MITM Capture - Stop Logging"
echo "========================================="
echo ""

# Check if MITM is running
if [ ! -f "$PID_FILE" ]; then
    echo "ERROR: MITM proxy PID file not found."
    echo "Is MITM running? Check with: ps | grep serial_mitm"
    exit 1
fi

MITM_PID=$(cat "$PID_FILE")

# Check if process is actually running
if ! kill -0 $MITM_PID 2>/dev/null; then
    echo "WARNING: MITM proxy (PID $MITM_PID) is not running!"
    echo "It may have crashed. Check logs."
    rm "$PID_FILE"
    exit 1
fi

# Get current run number
if [ -f "$RUN_COUNTER_FILE" ]; then
    CURRENT_RUN=$(cat "$RUN_COUNTER_FILE")
else
    CURRENT_RUN=1
fi

echo "Stopping capture for Run $CURRENT_RUN..."
echo ""

# Send SIGTERM for graceful shutdown
echo "Sending SIGTERM to MITM proxy (PID $MITM_PID)..."
kill -TERM $MITM_PID

# Wait for process to exit (with timeout)
WAIT_COUNT=0
while kill -0 $MITM_PID 2>/dev/null && [ $WAIT_COUNT -lt 10 ]; do
    sleep 1
    WAIT_COUNT=$((WAIT_COUNT + 1))
done

if kill -0 $MITM_PID 2>/dev/null; then
    echo "WARNING: Process did not exit gracefully. Force killing..."
    kill -9 $MITM_PID
    sleep 1
fi

echo "✓ MITM proxy stopped"
rm "$PID_FILE"

# Find the most recent log file for this run
LATEST_LOG=$(ls -t "$LOG_DIR"/mitm_capture_run${CURRENT_RUN}_*.log 2>/dev/null | head -1)

if [ -n "$LATEST_LOG" ]; then
    LOG_SIZE=$(du -h "$LATEST_LOG" | awk '{print $1}')
    TX_PACKETS=$(grep -c "^\\[.*\\] TX " "$LATEST_LOG" 2>/dev/null || echo "?")
    RX_PACKETS=$(grep -c "^\\[.*\\] RX " "$LATEST_LOG" 2>/dev/null || echo "?")

    echo ""
    echo "Capture Statistics:"
    echo "  Run: $CURRENT_RUN"
    echo "  Log file: $(basename "$LATEST_LOG")"
    echo "  Size: $LOG_SIZE"
    echo "  TX packets: $TX_PACKETS"
    echo "  RX packets: $RX_PACKETS"
else
    echo ""
    echo "WARNING: Could not find log file for run $CURRENT_RUN"
fi

# Increment run counter
NEXT_RUN=$((CURRENT_RUN + 1))
echo $NEXT_RUN > "$RUN_COUNTER_FILE"

echo ""
echo "========================================="
echo "  Run $CURRENT_RUN Complete!"
echo "========================================="
echo ""

if [ $CURRENT_RUN -ge $TARGET_RUNS ]; then
    echo "✅ TARGET ACHIEVED: $TARGET_RUNS runs completed!"
    echo ""
    echo "Next steps:"
    echo "  1. Retrieve logs: (from dev machine)"
    echo "     ./tools/mitm_retrieve_logs.sh"
    echo "  2. Disable MITM mode:"
    echo "     ./tools/mitm_disable.sh"
    echo "  3. Reboot robot to restore normal operation"
else
    echo "Progress: $CURRENT_RUN/$TARGET_RUNS runs completed"
    echo ""
    echo "Next steps for Run $NEXT_RUN:"
    echo "  1. Reboot robot:"
    echo "     reboot"
    echo "  2. Wait for boot (MITM will auto-start)"
    echo "  3. Press robot button to start cleaning"
    echo "  4. Run this script again when done:"
    echo "     /mnt/UDISK/mitm_stop.sh"
fi

echo ""
