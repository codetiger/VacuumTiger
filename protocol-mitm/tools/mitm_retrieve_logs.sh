#!/bin/bash
#
# MITM Retrieve Logs Script
# Downloads all MITM capture logs from robot to local machine
#
# Usage: ./tools/mitm_retrieve_logs.sh [output_dir]

set -e

ROBOT_HOST="vacuum"
ROBOT_PASS="${ROBOT_PASSWORD:-}"
REMOTE_LOG_DIR="/mnt/UDISK/log"
DEFAULT_OUTPUT_DIR="./logs/mitm_captures_$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${1:-$DEFAULT_OUTPUT_DIR}"

echo "========================================="
echo "  MITM Retrieve Logs"
echo "========================================="
echo ""

# Test robot connectivity
echo "Connecting to robot..."
if ! sshpass -p "$ROBOT_PASS" ssh -o ConnectTimeout=5 root@$ROBOT_HOST "echo OK" &>/dev/null; then
    echo "ERROR: Cannot connect to robot at $ROBOT_HOST"
    exit 1
fi
echo "✓ Connected"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Get list of log files
echo "Scanning for log files..."
LOG_FILES=$(sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST \
  "ls -1 $REMOTE_LOG_DIR/mitm_capture_*.log 2>/dev/null" || echo "")

if [ -z "$LOG_FILES" ]; then
    echo "No MITM capture logs found on robot."
    echo ""
    echo "Checked: $REMOTE_LOG_DIR/mitm_capture_*.log"
    exit 0
fi

LOG_COUNT=$(echo "$LOG_FILES" | wc -l | tr -d ' ')
echo "Found $LOG_COUNT log file(s)"
echo ""

# Download each log file
echo "Downloading logs to: $OUTPUT_DIR"
echo ""

for REMOTE_FILE in $LOG_FILES; do
    FILENAME=$(basename "$REMOTE_FILE")
    LOCAL_FILE="$OUTPUT_DIR/$FILENAME"

    echo -n "  $FILENAME ... "

    sshpass -p "$ROBOT_PASS" scp -q root@$ROBOT_HOST:$REMOTE_FILE "$LOCAL_FILE"

    FILE_SIZE=$(du -h "$LOCAL_FILE" | awk '{print $1}')
    echo "✓ ($FILE_SIZE)"
done

echo ""

# Get summary
TOTAL_SIZE=$(du -sh "$OUTPUT_DIR" | awk '{print $1}')

echo "========================================="
echo "  Download Complete"
echo "========================================="
echo ""
echo "Summary:"
echo "  Files downloaded: $LOG_COUNT"
echo "  Total size: $TOTAL_SIZE"
echo "  Location: $OUTPUT_DIR"
echo ""
echo "To analyze logs:"
echo "  ls -lh $OUTPUT_DIR/"
echo "  cat $OUTPUT_DIR/mitm_capture_run1_*.log | grep 'CMD='"
echo ""
