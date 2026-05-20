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

NO_ESP32=false
for arg in "$@"; do
    case "$arg" in
        --dry-run)  DRY_RUN=true ;;
        --no-kill)  NO_KILL=true ;;
        --no-esp32) NO_ESP32=true ;;
    esac
done

if [[ "$MODE" == "--dry-run" || "$MODE" == "--no-kill" || "$MODE" == "--no-esp32" ]]; then
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
    "esp32_uart_bridge"
    "imu_yaw_integrator"
    "ekf_node"                     # robot_localization executable
    "ekf_filter_node"              # robot_localization default node name
    "static_transform_publisher"   # both odom→base_link and base_link→imu_link
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

    # Free the webui port if something else is squatting on it. Default is
    # 80; WEBUI_PORT env var overrides (used by the smoke-test harness).
    local webui_port="${WEBUI_PORT:-80}"
    local port_pids
    port_pids=$(ss -tlnp 2>/dev/null | grep -E ":${webui_port} " | grep -oP 'pid=\K[0-9]+' | sort -u || true)
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
  imu-test     ESP32 UART bridge + EKF + Web UI — verify IMU on Stats page
  sensor-test  LIDAR + static TF + SLAM + ESP32 bridge (optional) + Web UI
                  diagnostic mode for sensor bring-up; supports --no-esp32
  full         **Everything.** LIDAR + ESP32 + IMU yaw + EKF + SLAM + DB +
                  Web UI. The mode for actual handheld scanning: walk around
                  and watch the map fill in + your trail on /map.
  help         Print this message

Options:
  --dry-run   Print what would be started without actually starting anything
  --no-kill   Skip the kill-stale-processes step (use if you want to layer)
  --no-esp32  In sensor-test, skip the ESP32 bridge launch (LIDAR-only)

Examples:
  ./setup.sh kill              # Just kill everything and exit
  ./setup.sh demo              # Web UI with mock data
  ./setup.sh web               # Web UI + DB with ROS2
  ./setup.sh imu-test          # ESP32 IMU + Stats page
  ./setup.sh sensor-test       # Diagnostic — LIDAR + SLAM + IMU
  ./setup.sh sensor-test --no-esp32   # As above but LIDAR-only
  ./setup.sh full              # Real scanning — everything live
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

check_esp32_serial() {
    # ESP32 enumerates as either /dev/ttyUSB0 (CP2102/CH340) or /dev/ttyACM0
    # (S3/C3 native USB). hardware.yaml's esp32.port pins the exact device.
    if [[ ! -e /dev/ttyUSB0 && ! -e /dev/ttyACM0 ]]; then
        log_error "ESP32 USB-Serial device not found. Plug the ESP32 into a Pi USB port."
        return 1
    fi
}

check_pyserial() {
    if ! python3 -c "import serial" 2>/dev/null; then
        log_error "pyserial not installed. Run 'pip install pyserial' inside the workspace venv."
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
    # The Postgres container is still named "roomba_postgres" (and the
    # database/user are still "roomba/roomba") deliberately — renaming
    # would orphan any historical scans already in the volume. The env
    # var RECON_DB_URL points the recon_db client at it. See
    # docs/STATUS.md §7 for the full rationale.
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
        imu-test)
            check_python || failed=true
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask || failed=true
            check_ros2 || failed=true
            check_workspace_built || failed=true
            check_esp32_serial || failed=true
            check_pyserial || failed=true
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
            # ESP32 is optional in sensor-test — only fail if --no-esp32 is
            # NOT passed AND the device is missing AND pyserial is missing.
            if ! $NO_ESP32; then
                check_esp32_serial || log_warn "ESP32 missing — pass --no-esp32 to silence this"
                check_pyserial      || failed=true
            fi
            ;;
        full)
            # Everything required — no optional fallbacks. This is the mode
            # used for actual scanning, so all the hardware must be present.
            check_python || failed=true
            if [[ -f "${SCRIPT_DIR}/.venv/bin/activate" ]]; then
                # shellcheck disable=SC1091
                source "${SCRIPT_DIR}/.venv/bin/activate"
            fi
            check_flask           || failed=true
            check_ros2            || failed=true
            check_workspace_built || failed=true
            check_docker          || failed=true
            check_lidar_serial    || failed=true
            check_slam_toolbox    || failed=true
            check_esp32_serial    || failed=true
            check_pyserial        || failed=true
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

launch_esp32_bridge() {
    # ROS params live in esp32_bridge.yaml (must be a pure ros-params YAML;
    # hardware.yaml keeps the LIDAR static config and isn't loadable via
    # --params-file). The bridge node is Python — venv must be active so
    # pyserial resolves.
    start_in_tmux "esp32" "$(venv_ros2_cmd)python3 -m recon_hardware.esp32_uart_bridge --ros-args --params-file ${SCRIPT_DIR}/config/esp32_bridge.yaml"
}

launch_imu_link_tf() {
    # H2.1: no enclosure yet — IMU sits at base_link. H6 replaces this
    # static identity with the actual mechanical offset.
    start_in_tmux "imu_tf" "$(source_ros2_cmd)ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link"
}

launch_imu_yaw_integrator() {
    # H3: integrates gyro Z from /imu/data_raw into a yaw quaternion on
    # /imu/data so slam_toolbox can use it as a scan-matching prior.
    start_in_tmux "imu_yaw" "$(venv_ros2_cmd)python3 -m recon_hardware.imu_yaw_integrator"
}

launch_ekf() {
    # H3: robot_localization ekf_node fuses /imu/data into a 2-D pose and
    # publishes /odom + odom→base_link TF (replacing the static identity
    # TF). yaw and yaw-rate only — accel is bias-corrupted on this chip.
    # The EKF defaults its odometry topic to /odometry/filtered; remap to
    # /odom so slam_toolbox + the web UI find it under the conventional name.
    start_in_tmux "ekf" "$(source_ros2_cmd)ros2 run robot_localization ekf_node --ros-args --params-file ${SCRIPT_DIR}/config/ekf.yaml -r /odometry/filtered:=/odom"
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
    imu-test)
        log_info "Starting: ESP32 bridge + yaw integrator + EKF + static IMU TF + DB + Web UI (IMU test)"
        ensure_db
        launch_esp32_bridge
        sleep 1
        launch_imu_yaw_integrator
        sleep 1
        launch_imu_link_tf
        sleep 1
        # EKF publishes /odom + odom→base_link from /imu/data — does its
        # job even without a LIDAR.
        launch_ekf
        sleep 1
        launch_db_node
        sleep 1
        launch_webui_ros
        ;;
    sensor-test)
        log_info "Starting: LIDAR + SLAM + ESP32 bridge + EKF + DB + Web UI (sensor test)"
        ensure_db
        launch_lidar_node
        sleep 2
        if $NO_ESP32; then
            # No IMU available → fall back to the legacy static identity
            # odom→base_link so slam_toolbox still has a TF chain.
            log_info "(--no-esp32: skipping ESP32 bridge, yaw integrator, EKF; using static odom→base_link)"
            launch_static_odom_tf
            sleep 1
        else
            # IMU pipeline supplies odom→base_link via EKF — no static TF.
            launch_esp32_bridge
            sleep 1
            launch_imu_yaw_integrator
            sleep 1
            launch_imu_link_tf
            sleep 1
            launch_ekf
            sleep 1
        fi
        launch_slam_toolbox
        sleep 2
        launch_db_node
        sleep 1
        launch_webui_ros
        ;;
    full)
        # Everything live — the actual scanning mode. No optional fallbacks
        # (no --no-esp32 path); if a piece is missing the prereq check
        # already failed. Order matters: bridge must be up before the yaw
        # integrator subscribes; integrator must be up before slam_toolbox
        # subscribes to /imu/data; EKF must be up before slam_toolbox tries
        # to look up odom→base_link.
        log_info "Starting: LIDAR + ESP32 bridge + yaw integrator + IMU TF + EKF + SLAM + DB + Web UI"
        ensure_db
        launch_esp32_bridge
        sleep 1
        launch_imu_yaw_integrator
        sleep 1
        launch_imu_link_tf
        sleep 1
        launch_ekf
        sleep 1
        launch_lidar_node
        sleep 2
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

# Mode-specific quick reference — the *most useful* window to look at first.
case "$MODE" in
    demo)
        log_info "Watch:  tmux attach -t $TMUX_SESSION  (only one window: 'webui')"
        log_info "Web UI: http://localhost:${WEBUI_PORT:-80}/  (mock data — no ROS2)"
        ;;
    web)
        log_info "Web UI: http://localhost:${WEBUI_PORT:-80}/"
        log_info "Useful windows: 'webui' (Flask logs), 'db_node' (save events)"
        ;;
    imu-test)
        log_info "Web UI: http://localhost:${WEBUI_PORT:-80}/stats  (live IMU + ESP32 link health)"
        log_info "Useful windows: 'esp32' (bridge logs), 'imu_yaw' (yaw → /imu/data), 'ekf' (/odom + TF), 'webui'"
        log_info "Check live:  ros2 topic hz /odom    (expect ~30 Hz)"
        ;;
    sensor-test)
        log_info "Web UI: http://localhost:${WEBUI_PORT:-80}/map    (live SLAM map)"
        log_info "        http://localhost:${WEBUI_PORT:-80}/stats  (link health + IMU)"
        log_info "Useful windows: 'lidar' (LD14P), 'slam_tb' (SLAM logs, watch for 'imu' init line)"
        log_info "                'esp32' + 'imu_yaw' + 'ekf' (if not --no-esp32 — IMU → /odom → SLAM)"
        ;;
    full)
        log_info "Web UI: http://localhost:${WEBUI_PORT:-80}/map    (live SLAM map + your trail)"
        log_info "        http://localhost:${WEBUI_PORT:-80}/stats  (full link health + live IMU)"
        log_info ""
        log_info "Walk-around recipe:"
        log_info "  1. Open /map in a phone/laptop browser on the same network."
        log_info "  2. Stand still for ~3 s while slam_toolbox does its first scan match."
        log_info "  3. Hold the device level and walk slowly (~0.5 m/s) along walls."
        log_info "  4. The red arrow = current pose. The faint blue line = trail."
        log_info "  5. Press SAVE on the ESP32 when finished — saves to PostgreSQL."
        log_info ""
        log_info "Useful tmux windows: 'slam_tb' (scan-match output) · 'ekf' (/odom rate)"
        log_info "                     'lidar' (driver) · 'esp32' (frame counts)"
        ;;
esac
log_info "Press Ctrl+C here to shut down all components."
echo ""

while true; do
    sleep 1
done
