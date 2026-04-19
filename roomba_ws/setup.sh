#!/usr/bin/env bash
# =============================================================================
# setup.sh — Tiered startup script for the roomba project
#
# Single entry point for starting the robot. Supports partial operation modes
# for incremental bring-up. Every mode first kills stale processes to ensure
# a fresh start.
#
# Usage:  ./setup.sh [MODE] [--dry-run] [--no-kill]
#
# Modes:
#   kill        Kill all roomba processes and tmux session, then exit
#   demo        Web UI only, no ROS2, no hardware — uses mock data (default)
#   web         Web UI + DB node only — ROS2 running but no hardware nodes
#   bt-test     Web + DB + bt_sim_node + joy_control_node — simulated controller
#   controller  Web + joy_linux_node + joy_control_node — real Xbox controller
#   draw-test   Draw on map with controller + save to DB — tests persistence pipeline
#   simulate-hw Simulated hardware — room generation, SLAM, fuzzy recon (no physical HW)
#   sensor-test LIDAR + SLAM + Web UI — verify sensor output on the map page
#   hardware    Hardware nodes only (sensors + motors) — no UI
#   full        Full system — all nodes + web UI
#   help        Print this message
#
# Development Stages (which mode to use):
#   Stage 1 – Web UI dev          →  demo
#   Stage 2 – Web + ROS2 bridge   →  web
#   Stage 3 – Simulated gamepad   →  bt-test
#   Stage 4 – Real Xbox testing   →  controller
#   Stage 4b – DB persistence test →  draw-test
#   Stage 4c – Simulated HW test  →  simulate-hw
#   Stage 5 – LIDAR bench test   →  sensor-test
#   Stage 6 – Motor + HW test    →  hardware
#   Stage 7 – Full integration    →  full
# =============================================================================

set -euo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-demo}"
DRY_RUN=false
NO_KILL=false
TMUX_SESSION="roomba"

# Parse flags
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        --no-kill) NO_KILL=true ;;
    esac
done

# Handle flag-only invocations
if [[ "$MODE" == "--dry-run" || "$MODE" == "--no-kill" ]]; then
    MODE="demo"
fi

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "${CYAN}[STEP]${NC}  $*"; }

# =============================================================================
# Kill — Clean slate before every start
# =============================================================================
# Patterns matching all roomba-related processes.  Extend this list as new
# nodes are added.
ROOMBA_PROC_PATTERNS=(
    "roomba_webui"
    "joy_control_node"
    "joy_linux_node"
    "joy_node"
    "bt_sim_node"
    "db_node"
    "ldlidar_node"
    "motor_controller"
    "recon_node"
    "sim_goal_follower"
    "sim_sensor_node"
    "sim_motor_node"
    "async_slam_toolbox_node"
)

kill_stale_processes() {
    log_info "Killing stale roomba processes..."

    # 1. Kill the tmux session (catches anything started by a previous run)
    if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
        tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
        log_step "Killed tmux session: $TMUX_SESSION"
    fi

    # 2. Kill individual processes by pattern (catches things started outside tmux)
    local killed=0
    for pattern in "${ROOMBA_PROC_PATTERNS[@]}"; do
        local pids
        pids=$(pgrep -f "$pattern" 2>/dev/null || true)
        if [[ -n "$pids" ]]; then
            for pid in $pids; do
                # Don't kill ourselves
                if [[ "$pid" != "$$" ]]; then
                    kill -9 "$pid" 2>/dev/null || true
                    killed=$((killed + 1))
                fi
            done
        fi
    done

    # 3. Kill anything still bound to webui port (80 or 5000)
    local port_pids
    port_pids=$(ss -tlnp 2>/dev/null | grep -E ':80 |:5000 ' | grep -oP 'pid=\K[0-9]+' | sort -u || true)
    if [[ -n "$port_pids" ]]; then
        for pid in $port_pids; do
            kill -9 "$pid" 2>/dev/null || true
            killed=$((killed + 1))
        done
    fi

    if (( killed > 0 )); then
        log_step "Killed $killed stale process(es)"
        sleep 1  # let ports release
    else
        log_info "No stale processes found."
    fi
}

# =============================================================================
# Help
# =============================================================================
print_help() {
    cat <<'EOF'
Usage: ./setup.sh [MODE] [--dry-run] [--no-kill]

Modes:
  kill        Kill all roomba processes and tmux session, then exit
  demo        Web UI only, no ROS2, no hardware — uses mock data (default)
  web         Web UI + DB node only — ROS2 running but no hardware nodes
  bt-test     Web + DB + bt_sim_node + joy_control_node — simulated controller
  controller  Web + joy_linux_node + joy_control_node — real Xbox controller
  draw-test   Draw on map with controller + save to DB — tests persistence pipeline
  simulate-hw Simulated hardware — room gen, SLAM, navigation, recon (no physical HW)
  sensor-test LIDAR + SLAM + Web UI — verify real sensor output on the map page
  hardware    Hardware nodes only (sensors + motors) — no UI
  full        Full system — all nodes + web UI
  help        Print this message

Options:
  --dry-run   Print what would be started without actually starting anything
  --no-kill   Skip the kill-stale-processes step (use if you want to layer)

Development Stages:
  Stage 1 – Web UI dev          →  demo
  Stage 2 – Web + ROS2 bridge   →  web
  Stage 3 – Simulated gamepad   →  bt-test
  Stage 4 – Real Xbox testing   →  controller
  Stage 4b – DB persistence test →  draw-test
  Stage 4c – Simulated HW test  →  simulate-hw
  Stage 5 – LIDAR bench test    →  sensor-test
  Stage 6 – Motor + HW test     →  hardware
  Stage 7 – Full integration    →  full

Examples:
  ./setup.sh kill              # Just kill everything and exit
  ./setup.sh demo              # Start web UI with mock data for development
  ./setup.sh web               # Start web UI + DB with ROS2
  ./setup.sh bt-test           # Test controller pipeline with simulated gamepad
  ./setup.sh controller        # Test with real Xbox controller
  ./setup.sh draw-test         # Draw on map with controller, save to PostgreSQL
  ./setup.sh simulate-hw       # Simulated room + SLAM + recon (no physical HW)
  ./setup.sh sensor-test       # LIDAR + SLAM + Web UI to see real sensor data
  ./setup.sh hardware          # LIDAR + motors (ESP32) for bench testing
  ./setup.sh full              # Full system launch
  ./setup.sh full --dry-run    # Show what full mode would start
  ./setup.sh web --no-kill     # Start web without killing existing processes
EOF
}

if [[ "$MODE" == "help" || "$MODE" == "-h" || "$MODE" == "--help" ]]; then
    print_help
    exit 0
fi

# =============================================================================
# Prerequisite Checks
# =============================================================================
check_python() {
    if ! python3 --version 2>/dev/null | grep -qE '3\.(1[1-9]|[2-9][0-9])'; then
        log_error "Python 3.11+ is required."
        return 1
    fi
}

check_flask() {
    if ! python3 -c "import flask" 2>/dev/null; then
        log_error "Flask is not installed. Run environment.sh first."
        return 1
    fi
    if ! python3 -c "import flask_socketio" 2>/dev/null; then
        log_error "Flask-SocketIO is not installed. Run environment.sh first."
        return 1
    fi
}

check_ros2() {
    # Always (re-)source the ROS2 underlay.  Even when ros2 is in PATH the
    # Python environment may be wrong (e.g. a venv hides system site-packages).
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        set +u
        # shellcheck disable=SC1091
        source /opt/ros/jazzy/setup.bash
        set -u
    fi
    if ! command -v ros2 &>/dev/null; then
        log_error "ROS2 is not installed. /opt/ros/jazzy/setup.bash not found."
        return 1
    fi
}

check_workspace_built() {
    if [[ ! -d "${SCRIPT_DIR}/install" ]]; then
        log_error "Workspace not built. Run: cd ${SCRIPT_DIR} && colcon build --symlink-install"
        return 1
    fi
    # Source the workspace overlay so ros2 run/launch can find our packages
    if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
        set +u
        # shellcheck disable=SC1091
        source "${SCRIPT_DIR}/install/setup.bash" 2>/dev/null || true
        set -u
    fi
}

check_lidar_serial() {
    if [[ ! -e /dev/ttyAMA0 ]]; then
        log_error "LIDAR serial port /dev/ttyAMA0 not found. Enable UART and disable serial console."
        return 1
    fi
}

check_i2c() {
    if [[ ! -e /dev/i2c-1 ]]; then
        log_error "I2C bus /dev/i2c-1 not found. Enable I2C in boot config (dtparam=i2c_arm=on)."
        return 1
    fi
}

check_xpadneo() {
    if ! lsmod | grep -q xpadneo; then
        log_warn "xpadneo module not loaded. Xbox controller may not work."
        # Not fatal — controller might not be needed yet
    fi
}

check_joy_package() {
    # Filesystem check — avoids ros2 CLI which breaks inside a venv
    if [[ ! -d /opt/ros/jazzy/share/joy ]]; then
        log_error "ROS2 joy package not found. Install: sudo apt install ros-jazzy-joy"
        return 1
    fi
    if [[ ! -d /opt/ros/jazzy/share/joy_linux ]]; then
        log_error "ROS2 joy_linux package not found. Install: sudo apt install ros-jazzy-joy"
        return 1
    fi
}

check_slam_toolbox() {
    if [[ ! -d /opt/ros/jazzy/share/slam_toolbox ]]; then
        log_error "slam_toolbox not found. Install: sudo apt install ros-jazzy-slam-toolbox"
        return 1
    fi
}

check_docker() {
    if ! command -v docker &>/dev/null; then
        log_error "docker is not installed. Run environment.sh to install it."
        return 1
    fi
    if ! docker info &>/dev/null && ! sudo docker info &>/dev/null; then
        log_error "Docker daemon is not running."
        return 1
    fi
}

ensure_db() {
    local docker_dir="${SCRIPT_DIR}/docker"
    if [[ ! -f "${docker_dir}/docker-compose.yaml" ]]; then
        log_error "docker/docker-compose.yaml not found"
        return 1
    fi
    if [[ ! -f "${docker_dir}/.env" ]]; then
        log_error "docker/.env not found — copy docker/.env.example and set credentials"
        return 1
    fi

    log_info "Starting PostgreSQL container..."
    cd "${docker_dir}"
    docker compose up -d 2>/dev/null || sudo docker compose up -d
    cd "${SCRIPT_DIR}"

    # Wait for PostgreSQL to accept connections (up to 30 seconds)
    local retries=30
    while ! (docker exec roomba_postgres pg_isready -U roomba &>/dev/null || \
             sudo docker exec roomba_postgres pg_isready -U roomba &>/dev/null); do
        retries=$((retries - 1))
        if [[ $retries -le 0 ]]; then
            log_error "PostgreSQL did not become ready in 30 seconds"
            return 1
        fi
        sleep 1
    done
    log_info "PostgreSQL is ready."

    # Export DB URL for child processes
    export ROOMBA_DB_URL="postgresql://roomba:$(grep POSTGRES_PASSWORD "${docker_dir}/.env" | cut -d= -f2)@localhost:5432/$(grep POSTGRES_DB "${docker_dir}/.env" | cut -d= -f2)"
}

run_checks() {
    local mode="$1"
    local failed=false

    log_info "Running prerequisite checks for mode: $mode"

    case "$mode" in
        kill)
            # No prerequisites needed for kill mode
            ;;
        demo)
            check_python || failed=true
            # Activate venv if available
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask || failed=true
            ;;
        web)
            check_python || failed=true
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask || failed=true
            check_ros2 || failed=true
            check_workspace_built || failed=true
            check_docker || failed=true
            ;;
        bt-test)
            check_python || failed=true
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask || failed=true
            check_ros2 || failed=true
            check_workspace_built || failed=true
            check_docker || failed=true
            ;;
        controller)
            check_python || failed=true
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask || failed=true
            check_ros2 || failed=true
            check_workspace_built || failed=true
            check_docker || failed=true
            check_xpadneo
            check_joy_package || failed=true
            ;;
        draw-test)
            check_python || failed=true
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask || failed=true
            check_ros2 || failed=true
            check_workspace_built || failed=true
            check_docker || failed=true
            check_xpadneo
            check_joy_package || failed=true
            ;;
        simulate-hw)
            check_python || failed=true
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask || failed=true
            check_ros2 || failed=true
            check_workspace_built || failed=true
            check_docker || failed=true
            check_slam_toolbox || failed=true
            ;;
        sensor-test)
            check_python || failed=true
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask || failed=true
            check_ros2 || failed=true
            check_workspace_built || failed=true
            check_docker || failed=true
            check_lidar_serial || failed=true
            check_slam_toolbox || failed=true
            ;;
        hardware)
            check_python || failed=true
            check_ros2 || failed=true
            check_workspace_built || failed=true
            check_lidar_serial || failed=true
            check_i2c || failed=true
            ;;
        full)
            check_python || failed=true
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask || failed=true
            check_ros2 || failed=true
            check_workspace_built || failed=true
            check_docker || failed=true
            check_lidar_serial || failed=true
            check_i2c || failed=true
            check_slam_toolbox || failed=true
            check_xpadneo
            check_joy_package || failed=true
            ;;
        *)
            log_error "Unknown mode: $mode"
            print_help
            exit 1
            ;;
    esac

    if $failed; then
        log_error "Prerequisite checks failed. Fix errors above and retry."
        exit 1
    fi

    log_info "All prerequisite checks passed for mode: $mode"
}

# =============================================================================
# Component Launchers
# =============================================================================

start_in_tmux() {
    local window_name="$1"
    shift
    local cmd="$*"

    if $DRY_RUN; then
        log_step "[DRY-RUN] Would start tmux window '$window_name': $cmd"
        return
    fi

    # Create session or add window with a plain bash shell, then send the
    # command via send-keys.  This avoids quoting issues with 'source' in
    # tmux's default /bin/sh.
    if ! tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
        tmux new-session -d -s "$TMUX_SESSION" -n "$window_name"
    else
        tmux new-window -t "$TMUX_SESSION" -n "$window_name"
    fi

    tmux send-keys -t "$TMUX_SESSION:$window_name" "$cmd" C-m

    log_step "Started [$window_name]: $cmd"
}

source_ros2_cmd() {
    # Prefix that sources ROS2 underlay + workspace overlay.
    # Explicitly deactivates any venv to avoid Python path pollution
    # that breaks ros2 run discovery of C++ nodes.
    echo "deactivate 2>/dev/null; unset VIRTUAL_ENV; source /opt/ros/jazzy/setup.bash && source ${SCRIPT_DIR}/install/setup.bash 2>/dev/null; "
}

venv_ros2_cmd() {
    # Prefix for Python nodes that need the venv (Flask, eventlet, etc.)
    # Activates venv AFTER ROS2 sourcing so PYTHONPATH includes both.
    local prefix=""
    if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
        prefix="source ${SCRIPT_DIR}/.venv/bin/activate && "
    fi
    echo "source /opt/ros/jazzy/setup.bash && source ${SCRIPT_DIR}/install/setup.bash 2>/dev/null && ${prefix}"
}

launch_webui_demo() {
    local venv_activate=""
    if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
        venv_activate="source ${SCRIPT_DIR}/.venv/bin/activate && "
    fi
    start_in_tmux "webui" "${venv_activate}cd ${SCRIPT_DIR}/src/roomba_webui && python3 -m roomba_webui.app"
}

launch_webui_ros() {
    # Flask/eventlet/socketio live in the venv, so we use python3 directly
    # rather than ros2 run (which uses system Python).
    start_in_tmux "webui" "$(venv_ros2_cmd)cd ${SCRIPT_DIR}/src/roomba_webui && python3 -m roomba_webui.app"
}

launch_db_node() {
    # Python node — needs venv for SQLAlchemy/psycopg2.
    # ros2 run follows the shebang (#!/usr/bin/python3) which
    # bypasses the venv, so launch via python3 -m instead.
    start_in_tmux "db_node" "$(venv_ros2_cmd)python3 -m roomba_db.db_node"
}

launch_bt_sim_node() {
    start_in_tmux "bt_sim" "$(source_ros2_cmd)ros2 run roomba_control bt_sim_node"
}

launch_joy_control_node() {
    start_in_tmux "joy_ctrl" "$(source_ros2_cmd)ros2 run roomba_control joy_control_node --ros-args --params-file ${SCRIPT_DIR}/config/controller.yaml"
}

launch_draw_node() {
    start_in_tmux "draw" "$(source_ros2_cmd)ros2 run roomba_control draw_node --ros-args --params-file ${SCRIPT_DIR}/config/controller.yaml"
}

launch_joy_node() {
    start_in_tmux "joy" "$(source_ros2_cmd)ros2 run joy_linux joy_linux_node --ros-args -p dev:=/dev/input/js0"
}

launch_lidar_node() {
    start_in_tmux "lidar" "$(source_ros2_cmd)ros2 launch ldlidar_stl_ros2 ld14p.launch.py"
}

launch_static_odom_tf() {
    # Publish a static identity odom→base_link TF for modes without a motor/odom node
    start_in_tmux "odom_tf" "$(source_ros2_cmd)ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link"
}

launch_motor_controller() {
    start_in_tmux "motors" "$(source_ros2_cmd)ros2 run roomba_hardware motor_controller"
}

launch_sim_motor_node() {
    start_in_tmux "sim_motor" "$(source_ros2_cmd)ros2 run roomba_hardware sim_motor_node --ros-args --params-file ${SCRIPT_DIR}/config/simulation.yaml"
}

launch_sim_sensor_node() {
    start_in_tmux "sim_sensor" "$(source_ros2_cmd)ros2 run roomba_hardware sim_sensor_node --ros-args --params-file ${SCRIPT_DIR}/config/simulation.yaml"
}

launch_slam_toolbox() {
    start_in_tmux "slam_tb" "$(source_ros2_cmd)ros2 launch slam_toolbox online_async_launch.py slam_params_file:=${SCRIPT_DIR}/config/slam_params.yaml use_sim_time:=false"
}

launch_recon_node() {
    start_in_tmux "recon" "$(source_ros2_cmd)ros2 run roomba_navigation recon_node --ros-args --params-file ${SCRIPT_DIR}/config/simulation.yaml"
}

launch_sim_goal_follower() {
    start_in_tmux "goal_fw" "$(source_ros2_cmd)ros2 run roomba_navigation sim_goal_follower --ros-args --params-file ${SCRIPT_DIR}/config/simulation.yaml"
}

launch_slam() {
    # Full SLAM stack — slam_toolbox only (LIDAR publishes /scan directly)
    start_in_tmux "slam_tb" "$(source_ros2_cmd)ros2 launch slam_toolbox online_async_launch.py slam_params_file:=${SCRIPT_DIR}/config/slam_params.yaml use_sim_time:=false"
}

launch_nav2() {
    start_in_tmux "nav2" "$(source_ros2_cmd)ros2 launch nav2_bringup navigation_launch.py params_file:=${SCRIPT_DIR}/config/nav2_params.yaml"
    start_in_tmux "recon" "$(source_ros2_cmd)ros2 run roomba_navigation recon_node"
}

# pigpiod removed — motors controlled via ESP32 coprocessor over I2C, not direct Pi GPIO

# =============================================================================
# Cleanup Handler
# =============================================================================
cleanup() {
    echo ""
    log_info "Caught signal — shutting down all components..."
    kill_stale_processes
    log_info "Shutdown complete."
    exit 0
}

trap cleanup SIGINT SIGTERM

# =============================================================================
# Mode Execution
# =============================================================================

# Handle kill mode immediately
if [[ "$MODE" == "kill" ]]; then
    kill_stale_processes
    log_info "All roomba processes killed. Exiting."
    exit 0
fi

# Kill stale processes before starting (unless --no-kill)
if ! $NO_KILL && ! $DRY_RUN; then
    kill_stale_processes
fi

run_checks "$MODE"

echo ""
echo "============================================"
echo "  ROOMBA — Starting in mode: $MODE"
if $DRY_RUN; then
    echo "  (DRY RUN — nothing will actually start)"
fi
echo "============================================"
echo ""

case "$MODE" in
    demo)
        log_info "Starting: Web UI (mock data only)"
        launch_webui_demo
        ;;
    web)
        log_info "Starting: DB node + Web UI (ROS2)"
        ensure_db
        launch_db_node
        sleep 1
        launch_webui_ros
        ;;
    bt-test)
        log_info "Starting: DB node + bt_sim_node + joy_control_node + Web UI"
        ensure_db
        launch_db_node
        sleep 1
        launch_bt_sim_node
        sleep 1
        launch_joy_control_node
        sleep 1
        launch_webui_ros
        ;;
    controller)
        log_info "Starting: joy_linux_node + joy_control_node + Web UI (real Xbox)"
        launch_joy_node
        sleep 1
        launch_joy_control_node
        sleep 1
        launch_webui_ros
        ;;
    draw-test)
        log_info "Starting: draw_node + joy_linux_node + DB + Web UI (draw & save)"
        ensure_db
        launch_joy_node
        sleep 1
        launch_draw_node
        sleep 1
        launch_db_node
        sleep 1
        launch_webui_ros
        ;;
    simulate-hw)
        log_info "Starting: Simulated HW — sim_motor + sim_sensor + SLAM + recon + DB + Web UI"
        ensure_db
        launch_sim_motor_node
        sleep 1
        launch_sim_sensor_node
        sleep 1
        launch_slam_toolbox
        sleep 3
        launch_sim_goal_follower
        sleep 1
        launch_recon_node
        sleep 1
        launch_joy_node
        sleep 1
        launch_joy_control_node
        sleep 1
        launch_db_node
        sleep 1
        launch_webui_ros
        ;;
    sensor-test)
        log_info "Starting: LIDAR + static TF + SLAM + DB + Web UI (sensor test)"
        ensure_db
        launch_lidar_node
        sleep 2
        launch_static_odom_tf
        sleep 1
        launch_slam_toolbox
        sleep 2
        launch_db_node
        sleep 1
        launch_webui_ros
        ;;
    hardware)
        log_info "Starting: LIDAR driver + Motor controller (ESP32 I2C)"
        launch_lidar_node
        sleep 1
        launch_motor_controller
        ;;
    full)
        log_info "Starting: Full system"
        launch_joy_node
        sleep 1
        launch_lidar_node
        sleep 1
        launch_motor_controller
        sleep 1
        launch_joy_control_node
        sleep 1
        launch_slam
        sleep 2
        launch_nav2
        sleep 1
        ensure_db
        launch_db_node
        sleep 1
        launch_webui_ros
        ;;
esac

if $DRY_RUN; then
    echo ""
    log_info "Dry run complete. No components were started."
    exit 0
fi

echo ""
log_info "All components started in tmux session: $TMUX_SESSION"
log_info "Attach with: tmux attach -t $TMUX_SESSION"
log_info "List windows: tmux list-windows -t $TMUX_SESSION"
log_info "Press Ctrl+C here to shut down all components."
echo ""

# Wait indefinitely until signal
while true; do
    sleep 1
done
