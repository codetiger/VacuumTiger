#!/bin/bash
#
# Movement & Scan Matching Test Script
#
# Tests robot movement accuracy independently from exploration/mapping logic.
# Uses SetGoal commands to move robot to specific locations.
#
# Usage:
#   ./test_movement.sh [OPTIONS]
#
# Options:
#   --mode MODE      Test mode: rotation_only, linear_only, square, waypoints
#   --output DIR     Output directory (default: results/movement_TIMESTAMP)
#   --distance D     Distance for linear tests in meters (default: 0.5)
#   --size S         Size for square test in meters (default: 0.5)
#   --waypoints WP   Custom waypoints: "x,y,theta;..."
#   --help           Show this help message

set -e

# Default values
MODE="rotation_only"
DISTANCE=0.5
SIZE=0.5
WAYPOINTS=""
OUTPUT_DIR=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --mode|-m)
            MODE="$2"
            shift 2
            ;;
        --output|-o)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --distance|-d)
            DISTANCE="$2"
            shift 2
            ;;
        --size|-s)
            SIZE="$2"
            shift 2
            ;;
        --waypoints|-w)
            WAYPOINTS="$2"
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

# Set output directory with timestamp if not specified
if [[ -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="$SCRIPT_DIR/results/movement_$(date +%Y%m%d_%H%M%S)"
else
    OUTPUT_DIR="$(cd "$(dirname "$OUTPUT_DIR")" 2>/dev/null && pwd)/$(basename "$OUTPUT_DIR")" || OUTPUT_DIR="$SCRIPT_DIR/$OUTPUT_DIR"
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Use drishti venv Python
PYTHON="$PROJECT_ROOT/drishti/venv/bin/python3"
if [[ ! -x "$PYTHON" ]]; then
    echo "ERROR: Python venv not found at $PYTHON"
    echo "Create it with: cd ../drishti && python3 -m venv venv && source venv/bin/activate && pip install protobuf"
    exit 1
fi

echo "=============================================="
echo "   Movement & Scan Matching Test"
echo "=============================================="
echo "Mode:     $MODE"
echo "Output:   $OUTPUT_DIR"
case $MODE in
    linear_only)
        echo "Distance: ${DISTANCE}m"
        ;;
    square)
        echo "Size:     ${SIZE}m"
        ;;
    waypoints)
        echo "Waypoints: $WAYPOINTS"
        ;;
esac
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
echo "[1/5] Starting SangamIO (mock mode)..."
(cd "$PROJECT_ROOT/sangam-io" && cargo run --release --features mock -- --config mock.toml) \
    > "$OUTPUT_DIR/sangamio.log" 2>&1 &
SANGAM_PID=$!
echo "  PID: $SANGAM_PID"
sleep 2

# Check if SangamIO started
if ! kill -0 $SANGAM_PID 2>/dev/null; then
    echo "ERROR: SangamIO failed to start. Check $OUTPUT_DIR/sangamio.log"
    cat "$OUTPUT_DIR/sangamio.log" | tail -20
    exit 1
fi
echo "  SangamIO started successfully"

# 2. Start DhruvaSLAM with exploration disabled for manual control
echo ""
echo "[2/5] Starting DhruvaSLAM..."

# Create a modified config for manual movement tests (disable exploration)
DHRUVA_CONFIG="$OUTPUT_DIR/dhruva-movement.toml"
cat > "$DHRUVA_CONFIG" << 'EOF'
# DhruvaSLAM config for movement testing
# Exploration DISABLED - we control movement via SetGoal

[source]
sangam_address = "localhost:5555"
# Use separate UDP port for localhost testing (SangamIO sends on 5556)
udp_port = 5556

[output]
bind_port = 5557
odometry_rate_hz = 50
map_rate_hz = 5.0
feature_rate_hz = 1.0

[stream]
robot_status_hz = 10.0
sensor_status_hz = 10.0
map_hz = 2.0
navigation_hz = 5.0

[map_storage]
path = "/tmp/dhruva_movement_test"

[odometry]
algorithm = "wheel"
ticks_per_meter = 4464.0
wheel_base = 0.233

[lidar]
mounting_x = -0.110
mounting_y = 0.0
optical_offset = 0.025
angle_offset = 0.2182

[preprocessing]
min_range = 0.15
max_range = 5.0
target_points = 180
min_angle_step = 0.0175

[slam]
min_match_score = 0.3
lost_threshold = 0.1
min_scan_points = 50
encoder_weight = 0.8
use_submap_matching = false

[matcher]
algorithm = "hybrid_p2l"
always_correlative = true
encoder_weight = 1.0
search_window_x = 0.3
search_window_y = 0.3
search_window_theta = 0.5
linear_resolution = 0.02
angular_resolution = 0.01
grid_resolution = 0.05
correlative_min_score = 0.4
icp_max_iterations = 50
icp_translation_epsilon = 0.001
icp_rotation_epsilon = 0.001
icp_max_correspondence_distance = 0.5
icp_min_correspondences = 10
icp_outlier_ratio = 0.1
icp_robust_kernel = "welsch"
icp_kernel_scale = 0.1

[keyframe]
min_translation = 0.5
min_rotation = 0.5
max_keyframes = 1000
min_interval_ms = 500

[submap]
scans_per_submap = 100
overlap_scans = 10
max_active_submaps = 2

[map]
resolution = 0.05
initial_width = 20.0
initial_height = 20.0
log_odds_occupied = 0.9
log_odds_free = -0.7
log_odds_max = 50.0
log_odds_min = -50.0
occupied_threshold = 0.5
free_threshold = -0.5

[loop_closure]
enabled = false

[exploration]
# DISABLED for movement testing - we control via SetGoal only
# With unknown_is_obstacle=false, navigation works through unexplored areas
enabled = false
strategy = "frontier"
frontier_min_size = 3
scan_at_frontiers = true
max_frontier_distance = 5.0
replan_interval_ms = 5000
stuck_timeout_ms = 10000
recovery_distance = 0.3

[navigation]
enabled = true
update_rate_hz = 10.0
robot_radius = 0.18
max_linear_vel = 0.3
max_angular_vel = 0.5
waypoint_threshold = 0.15
goal_threshold = 0.08
heading_threshold = 0.15
unknown_is_obstacle = false  # Allow navigation through unexplored areas

[session]
state_path = "/tmp/dhruva_movement_session.json"
auto_load_last_map = false
EOF

(cd "$PROJECT_ROOT/dhruva-slam" && RUST_LOG=info cargo run --release -- --config "$DHRUVA_CONFIG") \
    > "$OUTPUT_DIR/dhruva.log" 2>&1 &
DHRUVA_PID=$!
echo "  PID: $DHRUVA_PID"
sleep 3

# Check if DhruvaSLAM started
if ! kill -0 $DHRUVA_PID 2>/dev/null; then
    echo "ERROR: DhruvaSLAM failed to start. Check $OUTPUT_DIR/dhruva.log"
    cat "$OUTPUT_DIR/dhruva.log" | tail -30
    exit 1
fi
echo "  DhruvaSLAM started successfully"

# 3. Wait for services to be ready
echo ""
echo "[3/5] Waiting for services to be ready..."
sleep 2

# 4. Run movement test
echo ""
echo "[4/5] Running movement test..."

# Build test command
TEST_CMD="$PYTHON $SCRIPT_DIR/scripts/test_movement.py --port 5557 --mode $MODE"
TEST_CMD="$TEST_CMD --output $OUTPUT_DIR/results.json"

case $MODE in
    linear_only)
        TEST_CMD="$TEST_CMD --distance $DISTANCE"
        ;;
    square)
        TEST_CMD="$TEST_CMD --size $SIZE"
        ;;
    waypoints)
        if [[ -z "$WAYPOINTS" ]]; then
            echo "ERROR: --waypoints required for waypoints mode"
            exit 1
        fi
        TEST_CMD="$TEST_CMD --waypoints '$WAYPOINTS'"
        ;;
esac

echo "  Command: $TEST_CMD"
echo ""

# Run test
eval $TEST_CMD
TEST_EXIT=$?

# 5. Collect logs
echo ""
echo "[5/5] Collecting results..."

# Kill processes before final analysis
echo "  Stopping processes..."
kill $SANGAM_PID $DHRUVA_PID 2>/dev/null || true
wait 2>/dev/null || true
unset SANGAM_PID DHRUVA_PID

# Check for pose errors in logs
echo ""
echo "  Checking logs for issues..."

SLAM_ERRORS=$(grep -c "ERROR\|WARN" "$OUTPUT_DIR/dhruva.log" 2>/dev/null || echo "0")
echo "  SLAM log warnings/errors: $SLAM_ERRORS"

# Extract key metrics from result
if [[ -f "$OUTPUT_DIR/results.json" ]]; then
    echo ""
    echo "  Test results saved to: $OUTPUT_DIR/results.json"
fi

# Print summary
echo ""
echo "=============================================="
echo "   Test Complete"
echo "=============================================="
echo "Results: $OUTPUT_DIR"
echo ""
echo "Files:"
echo "  - results.json     Test metrics and pose history"
echo "  - dhruva.log       SLAM process logs"
echo "  - sangamio.log     Simulator logs"

if [[ $TEST_EXIT -eq 0 ]]; then
    echo ""
    echo "Status: PASSED"
else
    echo ""
    echo "Status: FAILED (exit code $TEST_EXIT)"
fi

exit $TEST_EXIT
