#!/bin/bash
#
# SLAM Round-Trip Test Script
# Tests DhruvaSLAM mapping against mock simulation ground truth
#
# Usage:
#   ./test_slam.sh [OPTIONS]
#
# Options:
#   --output DIR     Output directory (default: results/TIMESTAMP)
#   --duration SEC   Test duration in seconds (default: 120)
#   --config FILE    DhruvaSLAM config file (default: dhruva-test.toml)
#   --help           Show this help message

set -e

# Default values
DURATION=120
CONFIG="dhruva-test.toml"
OUTPUT_DIR=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --output|-o)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --duration|-d)
            DURATION="$2"
            shift 2
            ;;
        --config|-c)
            CONFIG="$2"
            shift 2
            ;;
        --help|-h)
            head -20 "$0" | tail -15
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Set output directory with timestamp if not specified (use absolute path)
if [[ -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="$SCRIPT_DIR/results/$(date +%Y%m%d_%H%M%S)"
else
    # Convert relative path to absolute
    OUTPUT_DIR="$(cd "$(dirname "$OUTPUT_DIR")" 2>/dev/null && pwd)/$(basename "$OUTPUT_DIR")" || OUTPUT_DIR="$SCRIPT_DIR/$OUTPUT_DIR"
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Use drishti venv Python (has protobuf installed)
PYTHON="$PROJECT_ROOT/drishti/venv/bin/python3"
if [[ ! -x "$PYTHON" ]]; then
    echo "ERROR: Python venv not found at $PYTHON"
    echo "Create it with: cd ../drishti && python3 -m venv venv && source venv/bin/activate && pip install protobuf numpy pillow matplotlib"
    exit 1
fi

echo "=============================================="
echo "       SLAM Round-Trip Test"
echo "=============================================="
echo "Output:   $OUTPUT_DIR"
echo "Duration: ${DURATION}s"
echo "Config:   $CONFIG"
echo "=============================================="

# Cleanup function
cleanup() {
    echo ""
    echo "Cleaning up..."
    if [[ -n "$SANGAM_PID" ]]; then
        kill $SANGAM_PID 2>/dev/null || true
    fi
    if [[ -n "$DHRUVA_PID" ]]; then
        kill $DHRUVA_PID 2>/dev/null || true
    fi
    wait 2>/dev/null || true
}
trap cleanup EXIT

# 1. Start SangamIO (mock mode)
echo ""
echo "[1/7] Starting SangamIO (mock mode)..."
(cd "$PROJECT_ROOT/sangam-io" && cargo run --release --features mock -- --config mock.toml) \
    > "$OUTPUT_DIR/sangamio.log" 2>&1 &
SANGAM_PID=$!
echo "  PID: $SANGAM_PID"
sleep 2

# Check if SangamIO started
if ! kill -0 $SANGAM_PID 2>/dev/null; then
    echo "ERROR: SangamIO failed to start. Check $OUTPUT_DIR/sangamio.log"
    exit 1
fi

# 2. Start DhruvaSLAM
echo ""
echo "[2/7] Starting DhruvaSLAM..."
(cd "$PROJECT_ROOT/dhruva-slam" && RUST_LOG=info cargo run --release -- --config "../slam-test/$CONFIG") \
    > "$OUTPUT_DIR/dhruva.log" 2>&1 &
DHRUVA_PID=$!
echo "  PID: $DHRUVA_PID"
sleep 3

# Check if DhruvaSLAM started
if ! kill -0 $DHRUVA_PID 2>/dev/null; then
    echo "ERROR: DhruvaSLAM failed to start. Check $OUTPUT_DIR/dhruva.log"
    exit 1
fi

# 3. Send start mapping command
echo ""
echo "[3/7] Sending StartMapping command..."
$PYTHON "$SCRIPT_DIR/scripts/send_command.py" --port 5557 start_mapping \
    --name "test_$(date +%H%M%S)"

if [[ $? -ne 0 ]]; then
    echo "ERROR: Failed to send StartMapping command"
    exit 1
fi

# 4. Wait for duration with progress
echo ""
echo "[4/7] Mapping in progress (${DURATION}s)..."
ELAPSED=0
while [[ $ELAPSED -lt $DURATION ]]; do
    # Check processes are still running
    if ! kill -0 $SANGAM_PID 2>/dev/null; then
        echo "ERROR: SangamIO died unexpectedly"
        exit 1
    fi
    if ! kill -0 $DHRUVA_PID 2>/dev/null; then
        echo "ERROR: DhruvaSLAM died unexpectedly"
        exit 1
    fi

    # Progress indicator
    REMAINING=$((DURATION - ELAPSED))
    printf "\r  Time remaining: %3ds " $REMAINING
    sleep 5
    ELAPSED=$((ELAPSED + 5))
done
echo ""

# 5. Stop mapping
echo ""
echo "[5/7] Stopping mapping..."
$PYTHON "$SCRIPT_DIR/scripts/send_command.py" --port 5557 stop_mapping --save
sleep 2

# 6. Collect outputs
echo ""
echo "[6/7] Collecting outputs..."

# Copy ground truth (use the map from mock.toml - currently large_room)
cp "$PROJECT_ROOT/sangam-io/maps/large_room.pgm" "$OUTPUT_DIR/ground_truth.pgm"
cp "$PROJECT_ROOT/sangam-io/maps/large_room.yaml" "$OUTPUT_DIR/ground_truth.yaml"
echo "  Copied ground truth map"

# Find and copy SLAM output
SLAM_MAP_DIR="/tmp/dhruva_test_maps"
if [[ -d "$SLAM_MAP_DIR" ]]; then
    # Find most recent map file (macOS compatible)
    LATEST_MAP=$(find "$SLAM_MAP_DIR" -name "*.pgm" -type f -exec stat -f '%m %N' {} \; 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)
    if [[ -n "$LATEST_MAP" ]]; then
        cp "$LATEST_MAP" "$OUTPUT_DIR/slam_output.pgm"
        # Copy associated yaml if exists
        YAML_FILE="${LATEST_MAP%.pgm}.yaml"
        if [[ -f "$YAML_FILE" ]]; then
            cp "$YAML_FILE" "$OUTPUT_DIR/slam_output.yaml"
        fi
        echo "  Copied SLAM output: $(basename "$LATEST_MAP")"
    else
        echo "  WARNING: No SLAM map file found in $SLAM_MAP_DIR"
    fi
else
    echo "  WARNING: SLAM map directory not found: $SLAM_MAP_DIR"
fi

# Kill processes before analysis
echo "  Stopping processes..."
kill $SANGAM_PID $DHRUVA_PID 2>/dev/null || true
wait 2>/dev/null || true
unset SANGAM_PID DHRUVA_PID

# 7. Run analysis
echo ""
echo "[7/7] Analyzing results..."
if [[ -f "$OUTPUT_DIR/slam_output.pgm" ]]; then
    $PYTHON "$SCRIPT_DIR/scripts/analyze_map.py" \
        --ground-truth "$OUTPUT_DIR/ground_truth.pgm" \
        --slam-output "$OUTPUT_DIR/slam_output.pgm" \
        --start-pose 1.0,1.0,0 \
        --resolution 0.05 \
        --output-dir "$OUTPUT_DIR"
else
    echo "  SKIPPED: No SLAM output to analyze"
    echo "  Check $OUTPUT_DIR/dhruva.log for errors"
fi

# Print summary
echo ""
echo "=============================================="
echo "       Test Complete"
echo "=============================================="
echo "Results: $OUTPUT_DIR"
echo ""
if [[ -f "$OUTPUT_DIR/summary.txt" ]]; then
    cat "$OUTPUT_DIR/summary.txt"
else
    echo "Logs available:"
    echo "  - $OUTPUT_DIR/sangamio.log"
    echo "  - $OUTPUT_DIR/dhruva.log"
fi
