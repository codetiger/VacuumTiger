#!/bin/sh
#
# MITM Enable Script
# Enables MITM logging mode (takes effect on next boot)
#
# Usage: /mnt/UDISK/mitm_enable.sh [target_runs]
#
# This creates a flag file that tells mitm_boot_setup.sh to activate on next boot

ENABLED_FLAG="/mnt/UDISK/mitm_enabled"
RUN_COUNTER_FILE="/mnt/UDISK/mitm_run_counter"
LOG_DIR="/mnt/UDISK/log"

# Get target runs from argument or use default
TARGET_RUNS=${1:-5}

echo "========================================="
echo "  MITM Logging - Enable"
echo "========================================="
echo ""

# Create log directory
mkdir -p "$LOG_DIR"

# Create enabled flag
echo "Enabling MITM logging mode..."
touch "$ENABLED_FLAG"
echo "✓ MITM mode enabled"

# Reset run counter
echo "1" > "$RUN_COUNTER_FILE"
echo "✓ Run counter reset to 1"

echo ""
echo "Configuration:"
echo "  Target runs: $TARGET_RUNS"
echo "  Log directory: $LOG_DIR"
echo ""
echo "========================================="
echo "  MITM Mode Activated"
echo "========================================="
echo ""
echo "Next steps:"
echo "  1. Reboot the robot:"
echo "     reboot"
echo ""
echo "  2. After boot, MITM will auto-start"
echo ""
echo "  3. Press physical button to start cleaning Run 1"
echo ""
echo "  4. When robot returns to dock, stop logging:"
echo "     /mnt/UDISK/mitm_stop.sh"
echo ""
echo "  5. Repeat steps 1-4 for $TARGET_RUNS runs total"
echo ""
echo "  6. When complete, disable MITM:"
echo "     /mnt/UDISK/mitm_disable.sh"
echo ""
