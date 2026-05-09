#!/usr/bin/env bash
# =============================================================================
# setup.sh — Tiered startup script for Recon-Platform-R2 (handheld scanner).
#
# Single entry point for starting the device. Supports partial operation modes
# for incremental bring-up. Every mode first kills stale processes to ensure
# a fresh start.
#
# Usage:  ./setup.sh [MODE] [--dry-run] [--no-kill]
#
# Modes:
#   kill         Kill all recon processes and tmux session, then exit
#   demo         Web UI only, no ROS2, no hardware — mock data (default)
#   web          Web UI + DB node only — ROS2 running but no hardware nodes
#   sensor-test  LIDAR + static TF + SLAM + Web UI — verify sensor on map page
#   help         Print this message
#
# Future stage targets (added as H2–H6 land):
#   imu-test     ESP32 bridge + imu_filter_madgwick + EKF (H3)
#   handheld     Full LIDAR + IMU + SLAM + DB + Web UI (H4+)
# =============================================================================

set -euo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RECON_WS="$SCRIPT_DIR"
MODE="${1:-demo}"
DRY_RUN=false
NO_KILL=false
TMUX_SESSION="recon"

for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        --no-kill) NO_KILL=true ;;
    esac
done

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
RECON_PROC_PATTERNS=(
    "recon_webui"
    "db_node"
    "ldlidar_node"
    "sim_sensor_node"
    "draw_node"
    "async_slam_toolbox_node"
)

kill_stale_processes() {
    log_info "Killing stale recon processes..."

    if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
        tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
        log_step "Killed tmux session: $TMUX_SESSION"
    fi

    local killed=0
    for pattern in "${RECON_PROC_PATTERNS[@]}"; do
        local pids
        pids=$(pgrep -f "$pattern" 2>/dev/null || true)
        if [[ -n "$pids" ]]; then
            for pid in $pids; do
                if [[ "$pid" != "$$" ]]; then
                    kill -9 "$pid" 2>/dev/null || true
                    killed=$((killed + 1))
                fi
            done
        fi
    done

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
        sleep 1
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
  kill         Kill all recon processes and tmux session, then exit
  demo         Web UI only, no ROS2, no hardware — mock data (default)
  web          Web UI + DB node only — ROS2 running but no hardware nodes
  sensor-test  LIDAR + static TF + SLAM + Web UI — verify sensor on map page
  help         Print this message

Options:
  --dry-run   Print what would be started without actually starting anything
  --no-kill   Skip the kill-stale-processes step (use if you want to layer)

Examples:
  ./setup.sh kill              # Just kill everything and exit
  ./setup.sh demo              # Start web UI with mock data
  ./setup.sh web               # Start web UI + DB with ROS2
  ./setup.sh sensor-test       # LIDAR + SLAM + Web UI to see real sensor data
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

    export RECON_DB_URL="postgresql://roomba:$(grep POSTGRES_PASSWORD "${docker_dir}/.env" | cut -d= -f2)@localhost:5432/$(grep POSTGRES_DB "${docker_dir}/.env" | cut -d= -f2)"
}

run_checks() {
    local mode="$1"
    local failed=false

    log_info "Running prerequisite checks for mode: $mode"

    case "$mode" in
        kill)
            ;;
        demo)
            check_python || failed=true
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

    if ! tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
        tmux new-session -d -s "$TMUX_SESSION" -n "$window_name"
    else
        tmux new-window -t "$TMUX_SESSION" -n "$window_name"
    fi

    tmux send-keys -t "$TMUX_SESSION:$window_name" "$cmd" C-m

    log_step "Started [$window_name]: $cmd"
}

source_ros2_cmd() {
    echo "deactivate 2>/dev/null; unset VIRTUAL_ENV; source /opt/ros/jazzy/setup.bash && source ${SCRIPT_DIR}/install/setup.bash 2>/dev/null; "
}

venv_ros2_cmd() {
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
    start_in_tmux "webui" "${venv_activate}cd ${SCRIPT_DIR}/src/recon_webui && python3 -m recon_webui.app"
}

launch_webui_ros() {
    start_in_tmux "webui" "$(venv_ros2_cmd)cd ${SCRIPT_DIR}/src/recon_webui && python3 -m recon_webui.app"
}

launch_db_node() {
    start_in_tmux "db_node" "$(venv_ros2_cmd)python3 -m recon_db.db_node"
}

launch_lidar_node() {
    start_in_tmux "lidar" "$(source_ros2_cmd)ros2 launch ldlidar_stl_ros2 ld14p.launch.py"
}

launch_static_odom_tf() {
    # Publish a static identity odom→base_link TF for modes without an odom node
    start_in_tmux "odom_tf" "$(source_ros2_cmd)ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link"
}

launch_slam_toolbox() {
    start_in_tmux "slam_tb" "$(source_ros2_cmd)ros2 launch slam_toolbox online_async_launch.py slam_params_file:=${SCRIPT_DIR}/config/slam_params.yaml use_sim_time:=false"
}

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

if [[ "$MODE" == "kill" ]]; then
    kill_stale_processes
    log_info "All recon processes killed. Exiting."
    exit 0
fi

if ! $NO_KILL && ! $DRY_RUN; then
    kill_stale_processes
fi

run_checks "$MODE"

echo ""
echo "============================================"
echo "  RECON — Starting in mode: $MODE"
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

while true; do
    sleep 1
done
