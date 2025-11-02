#!/bin/sh
#
# MITM Disable Script
# Disables MITM logging mode and archives logs
#
# Usage: /mnt/UDISK/mitm_disable.sh
#
# This removes the flag file and runs cleanup

ENABLED_FLAG="/mnt/UDISK/mitm_enabled"
RUN_COUNTER_FILE="/mnt/UDISK/mitm_run_counter"
LOG_DIR="/mnt/UDISK/log"
ARCHIVE_DIR="/mnt/UDISK/log/archive"

echo "========================================="
echo "  MITM Logging - Disable"
echo "========================================="
echo ""

# Run cleanup first
if [ -f "/mnt/UDISK/mitm_cleanup.sh" ]; then
    echo "Running cleanup..."
    /mnt/UDISK/mitm_cleanup.sh
    echo ""
fi

# Remove enabled flag
if [ -f "$ENABLED_FLAG" ]; then
    echo "Disabling MITM mode..."
    rm "$ENABLED_FLAG"
    echo "✓ MITM mode disabled"
else
    echo "MITM mode was already disabled"
fi

# Archive logs with timestamp
if [ -d "$LOG_DIR" ] && [ "$(ls -A $LOG_DIR/mitm_capture_*.log 2>/dev/null)" ]; then
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    ARCHIVE_SUBDIR="$ARCHIVE_DIR/capture_$TIMESTAMP"

    echo ""
    echo "Archiving logs..."
    mkdir -p "$ARCHIVE_SUBDIR"

    # Move all MITM capture logs to archive
    mv "$LOG_DIR"/mitm_capture_*.log "$ARCHIVE_SUBDIR/" 2>/dev/null

    # Copy run counter for reference
    if [ -f "$RUN_COUNTER_FILE" ]; then
        RUNS_COMPLETED=$(($(cat "$RUN_COUNTER_FILE") - 1))
        echo "$RUNS_COMPLETED" > "$ARCHIVE_SUBDIR/runs_completed.txt"
    fi

    echo "✓ Logs archived to: $ARCHIVE_SUBDIR"

    # Show summary
    LOG_COUNT=$(ls -1 "$ARCHIVE_SUBDIR"/mitm_capture_*.log 2>/dev/null | wc -l)
    TOTAL_SIZE=$(du -sh "$ARCHIVE_SUBDIR" | awk '{print $1}')

    echo ""
    echo "Archive Summary:"
    echo "  Location: $ARCHIVE_SUBDIR"
    echo "  Files: $LOG_COUNT log files"
    echo "  Total size: $TOTAL_SIZE"
    echo "  Runs completed: $RUNS_COMPLETED"
fi

# Clean up run counter
if [ -f "$RUN_COUNTER_FILE" ]; then
    rm "$RUN_COUNTER_FILE"
fi

echo ""
echo "========================================="
echo "  MITM Mode Disabled"
echo "========================================="
echo ""
echo "IMPORTANT: Reboot required to restore normal operation"
echo ""
echo "To reboot:"
echo "  reboot"
echo ""
echo "To retrieve archived logs to dev machine:"
echo "  ./tools/mitm_retrieve_logs.sh"
echo ""
