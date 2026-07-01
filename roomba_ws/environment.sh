#!/usr/bin/env bash
# =============================================================================
# environment.sh — Fully idempotent environment reproduction script
# Installs every dependency needed to build and run the Recon-Platform-R2
# handheld scanner from a clean Ubuntu Server 24.04 LTS (ARM64) install.
#
# Running this script twice must produce the same result without errors.
# Every command is non-interactive.
#
# Modes:
#   ./environment.sh           — Full install + verify (default)
#   ./environment.sh --check   — Verify-only: skip all installs, just check
#                                 if the environment matches the project spec.
#                                 Use this on a device that was already set up.
#   ./environment.sh --help    — Show usage
#
# Key implementation notes:
#   - UART0 (PL011 / ttyAMA0) is dedicated to the LD14P LIDAR
#   - ESP32 I/O coprocessor (MPU-6050 IMU + 3 buttons) reaches the Pi via the
#     mini-UART; framing is custom binary (not micro-ROS).
#   - ROS2 sourcing uses set +u to handle unset bash variables
#   - Python venv uses --system-site-packages for rclpy access
# =============================================================================

set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_section() { echo -e "\n${BLUE}━━━ $* ━━━${NC}"; }

# Track pass/fail for final verification
declare -A VERIFY_RESULTS

# --- Argument parsing ---
MODE="install"
case "${1:-}" in
    --check|--verify|-c)
        MODE="check"
        ;;
    --help|-h)
        echo "Usage: $0 [--check|--verify|-c] [--help|-h]"
        echo ""
        echo "  (no args)       Full install + verification"
        echo "  --check, -c     Verify-only — check environment matches project spec"
        echo "  --help, -h      Show this help"
        exit 0
        ;;
    "")
        MODE="install"
        ;;
    *)
        log_error "Unknown argument: $1. Use --help for usage."
        exit 1
        ;;
esac

# =============================================================================
# INSTALL SECTIONS (skipped in --check mode)
# =============================================================================
if [[ "$MODE" == "install" ]]; then

# =============================================================================
# SECTION 0: Wait for apt locks (unattended-upgrades on fresh installs)
# =============================================================================
wait_for_apt_lock() {
    local max_wait=300  # 5 minutes
    local waited=0
    while fuser /var/lib/dpkg/lock-frontend &>/dev/null \
       || fuser /var/lib/apt/lists/lock &>/dev/null \
       || fuser /var/lib/dpkg/lock &>/dev/null; do
        if [[ $waited -eq 0 ]]; then
            log_warn "Waiting for apt lock (unattended-upgrades or another apt process)..."
        fi
        sleep 5
        waited=$((waited + 5))
        if [[ $waited -ge $max_wait ]]; then
            log_error "Timed out waiting for apt lock after ${max_wait}s. Kill the blocking process or retry later."
            exit 1
        fi
    done
    if [[ $waited -gt 0 ]]; then
        log_info "apt lock released after ${waited}s."
    fi
}

wait_for_apt_lock

# Ensure noble-updates is in apt sources (some cloud images ship without it).
# Without it, security-updated base packages (zlib1g, etc.) create version
# mismatches with their -dev counterparts still pinned to the base noble repo.
APT_SOURCES="/etc/apt/sources.list.d/ubuntu.sources"
if [[ -f "$APT_SOURCES" ]] && ! grep -q 'noble-updates' "$APT_SOURCES"; then
    log_info "Adding noble-updates to apt sources (required for dependency resolution)"
    sudo sed -i '/^Suites: noble$/s/$/ noble-updates/' "$APT_SOURCES"
fi

# =============================================================================
# SECTION 1: System Packages
# =============================================================================
log_info "=== Section 1: System Packages ==="

sudo apt-get update
sudo apt-get install -y \
    git \
    curl \
    wget \
    tmux \
    build-essential \
    cmake \
    python3-pip \
    python3-venv \
    python3-dev \
    software-properties-common \
    lsb-release \
    gnupg2 \
    ca-certificates \
    unzip \
    locales

# Ensure locale is set (required by ROS2)
sudo locale-gen en_US en_US.UTF-8 || true
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true
export LANG=en_US.UTF-8

log_info "System packages installed."

# =============================================================================
# SECTION 2: LIDAR UART Setup
# =============================================================================
log_info "=== Section 2: LIDAR UART Setup ==="

# The LD14P LIDAR connects to Pi 5 via UART0 (/dev/ttyAMA0 at 230400 baud).
# We must disable the serial console and ensure UART is enabled.

BOOT_CONFIG="/boot/firmware/config.txt"
if [[ -f "$BOOT_CONFIG" ]]; then
    # Enable UART0
    if ! grep -q "^enable_uart=1" "$BOOT_CONFIG"; then
        echo "enable_uart=1" | sudo tee -a "$BOOT_CONFIG" > /dev/null
        log_info "UART enabled in $BOOT_CONFIG (reboot required to take effect)."
    else
        log_info "UART already enabled in $BOOT_CONFIG."
    fi
    if ! grep -q "^dtoverlay=uart0" "$BOOT_CONFIG"; then
        echo "dtoverlay=uart0" | sudo tee -a "$BOOT_CONFIG" > /dev/null
        log_info "UART0 overlay added to $BOOT_CONFIG."
    else
        log_info "UART0 overlay already in $BOOT_CONFIG."
    fi
    # Move Bluetooth to mini-UART so PL011 (ttyAMA0) is free for LIDAR
    if ! grep -q "^dtoverlay=miniuart-bt" "$BOOT_CONFIG"; then
        echo "dtoverlay=miniuart-bt" | sudo tee -a "$BOOT_CONFIG" > /dev/null
        log_info "miniuart-bt overlay added — BT moved to mini-UART (reboot required)."
    else
        log_info "miniuart-bt overlay already in $BOOT_CONFIG."
    fi
else
    log_warn "$BOOT_CONFIG not found — not running on Pi? UART config skipped."
fi

# Remove kernel serial console from cmdline.txt (conflicts with LIDAR on ttyAMA0)
CMDLINE="/boot/firmware/cmdline.txt"
if [[ -f "$CMDLINE" ]] && grep -q 'console=serial0' "$CMDLINE"; then
    sudo sed -i 's/console=serial0,[0-9]* //' "$CMDLINE"
    log_info "Removed serial console from kernel command line (reboot required)."
else
    log_info "No serial console in kernel command line."
fi

# Disable serial console on /dev/ttyAMA0 (frees it for LIDAR)
if systemctl is-enabled serial-getty@ttyAMA0.service &>/dev/null; then
    sudo systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
    sudo systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
    sudo systemctl mask serial-getty@ttyAMA0.service 2>/dev/null || true
    log_info "Serial console disabled and masked on /dev/ttyAMA0."
else
    log_info "Serial console already disabled on /dev/ttyAMA0."
fi

# udev rule: ensure ttyAMA0 has correct group/permissions after boot
UDEV_RULE="/etc/udev/rules.d/99-lidar-uart.rules"
if [[ ! -f "$UDEV_RULE" ]]; then
    echo 'KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"' | sudo tee "$UDEV_RULE" > /dev/null
    log_info "udev rule created for /dev/ttyAMA0 permissions."
else
    log_info "udev rule for ttyAMA0 already exists."
fi

# Add user to dialout group for serial port access
if ! groups "$USER" | grep -q dialout; then
    sudo usermod -aG dialout "$USER"
    log_info "Added $USER to dialout group (log out and back in for effect)."
else
    log_info "$USER already in dialout group."
fi

log_info "LIDAR UART setup complete."

# =============================================================================
# SECTION 3: ROS2 Jazzy Jalisco
# =============================================================================
log_info "=== Section 3: ROS2 Jazzy Jalisco ==="

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    log_info "ROS2 Jazzy already installed, skipping."
else
    # Add ROS2 apt repository (only if no existing ros2 source)
    if [[ ! -f /etc/apt/sources.list.d/ros2.sources ]] && [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg

        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    fi

    sudo apt-get update
    sudo apt-get install -y ros-jazzy-ros-base
    log_info "ROS2 Jazzy installed."
fi

# Source ROS2 for the rest of this script
# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u

# =============================================================================
# SECTION 4: ROS2 Packages
# =============================================================================
log_info "=== Section 4: ROS2 Packages ==="

sudo apt-get install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-msgs \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    python3-colcon-common-extensions \
    python3-serial \
    libgtest-dev
# Note: several packages above are transitive deps of others in this list
# (msg packages via slam/sensor stack, libgtest-dev via ament-cmake-gtest).
# They are listed explicitly so a fresh install is always complete
# regardless of upstream dep changes.
#
# Notable choices:
#   * imu-filter-madgwick is intentionally NOT installed — we ship our own
#     imu_yaw_integrator node (gyro-Z → quaternion) that's cheaper and avoids
#     the magnetometer dependency. robot-localization handles the actual EKF.
#   * std-srvs is required for the slam_toolbox Pause/Resume services that
#     setup.sh exposes via the web UI start/stop buttons.
#   * python3-serial provides pyserial for esp32_uart_bridge. Installed via
#     apt so the system interpreter and the venv (--system-site-packages)
#     both see it without an extra pip step.

log_info "ROS2 packages installed."

# =============================================================================
# SECTION 5: Python Virtual Environment
# =============================================================================
log_info "=== Section 5: Python Virtual Environment ==="

if [[ ! -d "$VENV_DIR" ]]; then
    python3 -m venv "$VENV_DIR" --system-site-packages
    log_info "Python venv created at $VENV_DIR"
else
    log_info "Python venv already exists at $VENV_DIR"
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

if [[ -f "${SCRIPT_DIR}/requirements.txt" ]]; then
    pip install --upgrade pip
    pip install -r "${SCRIPT_DIR}/requirements.txt"
    log_info "Python dependencies installed from requirements.txt"
else
    log_warn "requirements.txt not found at ${SCRIPT_DIR}/requirements.txt — skipping pip install"
fi

# Verify critical packages are importable within the venv
if python3 -c "import eventlet" 2>/dev/null; then
    log_info "eventlet installed and importable in venv"
else
    log_warn "eventlet not importable — Flask-SocketIO will not work. Check requirements.txt"
fi

deactivate

# =============================================================================
# SECTION 6: WiFi Access Point & Networking
# =============================================================================
log_info "=== Section 6: WiFi Access Point & Networking ==="

# Install hostapd (WiFi AP daemon), dnsmasq (DNS+DHCP), iw (interface management)
sudo apt-get install -y hostapd dnsmasq iw rfkill

# Ubuntu masks hostapd after install — unmask it so we can start it later.
# ALSO disable it: hostapd.service must NOT auto-start at boot, because it would
# race ahead of recon-ap-start.sh and try to bind ap0 before that interface
# exists, failing with "Could not read interface ap0 flags: No such device" /
# "nl80211 driver initialization failed". recon-ap-start.sh creates ap0 first,
# then starts hostapd — so hostapd is started on-demand, exactly like dnsmasq.
sudo systemctl unmask hostapd 2>/dev/null || true
sudo systemctl disable hostapd 2>/dev/null || true

# dnsmasq auto-starts on install and conflicts with systemd-resolved (port 53).
# We don't need it running yet — recon-ap-start.sh will restart it after ap0 exists.
sudo systemctl stop dnsmasq 2>/dev/null || true
sudo systemctl disable dnsmasq 2>/dev/null || true

# --- hostapd configuration ---
HOSTAPD_CONF="/etc/hostapd/hostapd.conf"
if [[ ! -f "$HOSTAPD_CONF" ]] || ! grep -q "ssid=Recon" "$HOSTAPD_CONF" 2>/dev/null; then
    sudo tee "$HOSTAPD_CONF" > /dev/null <<'HOSTAPD_EOF'
interface=ap0
driver=nl80211
ssid=Recon
hw_mode=g
channel=1
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=recon123
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
HOSTAPD_EOF
    sudo chmod 600 "$HOSTAPD_CONF"
    log_info "hostapd.conf written (SSID: Recon, WPA2, mode 600)"
else
    log_info "hostapd.conf already configured."
fi

# --- dnsmasq configuration ---
DNSMASQ_CONF="/etc/dnsmasq.d/recon.conf"
# Get the current wlan0 IP for DNS resolution on LAN
# Note: || true guards against pipefail exiting the script if wlan0 has no IPv4
WLAN0_IP=$(ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -1 || true)
WLAN0_IP="${WLAN0_IP:-172.31.225.193}"
# Get default gateway for DNS forwarding
ROUTER_IP=$(ip route 2>/dev/null | grep default | awk '{print $3}' | head -1 || true)
ROUTER_IP="${ROUTER_IP:-172.31.225.213}"

# Rewrite recon.conf if it is missing OR if it still uses `bind-dynamic`.
#
# CRITICAL: dnsmasq applies ONE global bind mode to the whole daemon, and the
# system ships /etc/dnsmasq.d/ubuntu-fan with `bind-interfaces`. Mixing that
# with `bind-dynamic` here is fatal — dnsmasq exits with "cannot set
# --bind-interfaces and --bind-dynamic", which kills dnsmasq and cascades into
# recon-ap.service failing, so the AP comes up with NO DHCP server and clients
# associate but never get an IP. We MUST use `bind-interfaces` to stay
# consistent with ubuntu-fan.
#
# The boot race that `bind-dynamic` was meant to dodge ("unknown interface
# wlan0") does not apply: dnsmasq.service is disabled and only ever started by
# recon-ap-start.sh, which first creates ap0 from wlan0 — so both interfaces
# already exist before dnsmasq binds to them.
needs_rewrite=false
if [[ ! -f "$DNSMASQ_CONF" ]]; then
    needs_rewrite=true
elif grep -q '^bind-dynamic$' "$DNSMASQ_CONF" 2>/dev/null; then
    needs_rewrite=true
    log_warn "$DNSMASQ_CONF uses 'bind-dynamic' — rewriting with bind-interfaces (conflicts with ubuntu-fan)"
fi
if $needs_rewrite; then
    sudo tee "$DNSMASQ_CONF" > /dev/null <<DNSMASQ_EOF
# Recon AP — DHCP on ap0, DNS on both interfaces.
# bind-interfaces (NOT bind-dynamic): the system's /etc/dnsmasq.d/ubuntu-fan
# sets bind-interfaces, and dnsmasq forbids mixing it with bind-dynamic
# ("cannot set --bind-interfaces and --bind-dynamic"). dnsmasq is started
# on-demand by recon-ap-start.sh after ap0+wlan0 exist, so binding to named
# interfaces is safe (no boot-time "unknown interface wlan0" race).
interface=ap0
interface=wlan0
bind-interfaces
except-interface=lo

# DHCP only on hotspot (ap0), NOT on wlan0 (avoid conflicting with router)
dhcp-range=10.0.0.10,10.0.0.50,24h
no-dhcp-interface=wlan0

# DNS: resolve recon.local to Pi on both networks
address=/recon.local/10.0.0.1
address=/recon.local/${WLAN0_IP}
address=/gabi.local/10.0.0.1
address=/gabi.local/${WLAN0_IP}

# Forward other DNS queries
server=${ROUTER_IP}
server=8.8.8.8
DNSMASQ_EOF
    log_info "dnsmasq.conf written (DHCP on ap0, DNS on ap0+wlan0, bind-interfaces)"
    # Restart via recon-ap so ap0 is (re)created before dnsmasq binds to it.
    sudo systemctl restart recon-ap.service 2>/dev/null || log_warn "recon-ap restart failed — check 'systemctl status recon-ap'"
else
    log_info "dnsmasq.conf already up-to-date (bind-interfaces present)."
fi

# --- recon-ap systemd service + helper scripts ---
sudo tee /usr/local/bin/recon-ap-start.sh > /dev/null <<'AP_START_EOF'
#!/bin/bash
set -e

# Verify wlan0 exists before attempting to create AP
if ! ip link show wlan0 &>/dev/null; then
    echo "[recon-ap] ERROR: wlan0 not found — cannot create AP" >&2
    exit 1
fi

# Create virtual AP interface from wlan0
if ! ip link show ap0 &>/dev/null; then
    iw dev wlan0 interface add ap0 type __ap || {
        echo "[recon-ap] ERROR: failed to create ap0 from wlan0" >&2
        exit 1
    }
fi
ip addr flush dev ap0 2>/dev/null || true
ip addr add 10.0.0.1/24 dev ap0
ip link set ap0 up
# Restart services to pick up ap0
systemctl restart dnsmasq
systemctl restart hostapd
AP_START_EOF
sudo chmod +x /usr/local/bin/recon-ap-start.sh

sudo tee /usr/local/bin/recon-ap-stop.sh > /dev/null <<'AP_STOP_EOF'
#!/bin/bash
systemctl stop hostapd 2>/dev/null || true
ip link set ap0 down 2>/dev/null || true
iw dev ap0 del 2>/dev/null || true
AP_STOP_EOF
sudo chmod +x /usr/local/bin/recon-ap-stop.sh

# Remove legacy roomba-ap service if present (clean migration from old name)
if [[ -f /etc/systemd/system/roomba-ap.service ]]; then
    sudo systemctl disable --now roomba-ap.service 2>/dev/null || true
    sudo rm -f /etc/systemd/system/roomba-ap.service \
        /usr/local/bin/roomba-ap-start.sh \
        /usr/local/bin/roomba-ap-stop.sh
    sudo systemctl daemon-reload
    log_info "Legacy roomba-ap.service removed."
fi

# Remove legacy roomba dnsmasq config. This is checked independently of the
# service file above: the old roomba-ap.service can be gone while
# /etc/dnsmasq.d/roomba.conf survives, and its `bind-interfaces` directive
# fatally conflicts with recon.conf's `bind-dynamic` ("cannot set
# --bind-interfaces and --bind-dynamic"), which kills dnsmasq and cascades
# into recon-ap.service failing — leaving the AP with no DHCP server.
if [[ -f /etc/dnsmasq.d/roomba.conf ]]; then
    sudo rm -f /etc/dnsmasq.d/roomba.conf
    sudo systemctl restart dnsmasq 2>/dev/null || true
    log_info "Legacy /etc/dnsmasq.d/roomba.conf removed (conflicted with recon.conf)."
fi

RECON_AP_SERVICE="/etc/systemd/system/recon-ap.service"
if [[ ! -f "$RECON_AP_SERVICE" ]]; then
    sudo tee "$RECON_AP_SERVICE" > /dev/null <<'SERVICE_EOF'
[Unit]
Description=Recon WiFi Access Point (ap0)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/recon-ap-start.sh
ExecStop=/usr/local/bin/recon-ap-stop.sh

[Install]
WantedBy=multi-user.target
SERVICE_EOF
    sudo systemctl daemon-reload
    log_info "recon-ap.service created."
else
    log_info "recon-ap.service already exists."
fi

sudo systemctl enable recon-ap.service 2>/dev/null || true
sudo systemctl start recon-ap.service 2>/dev/null || true

# --- Bundle socket.io client library (no CDN dependency) ---
SOCKETIO_JS="${SCRIPT_DIR}/src/recon_webui/recon_webui/static/js/socket.io.min.js"
SOCKETIO_SHA256="ad52fc540680945fe7549c0f1b1126b54029dd7eb25f8ce2b079a6242c807011"
if [[ ! -f "$SOCKETIO_JS" ]]; then
    mkdir -p "$(dirname "$SOCKETIO_JS")"
    curl -sL "https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.4/socket.io.min.js" -o "$SOCKETIO_JS"
    if [[ -s "$SOCKETIO_JS" ]]; then
        ACTUAL_SHA=$(sha256sum "$SOCKETIO_JS" | awk '{print $1}')
        if [[ "$ACTUAL_SHA" == "$SOCKETIO_SHA256" ]]; then
            log_info "socket.io.min.js downloaded and verified (sha256 OK)"
        else
            log_warn "socket.io.min.js sha256 mismatch — expected ${SOCKETIO_SHA256:0:16}..., got ${ACTUAL_SHA:0:16}..."
        fi
    else
        log_warn "Failed to download socket.io.min.js — web UI will need CDN access"
    fi
else
    log_info "socket.io.min.js already bundled."
fi

# --- Allow Python to bind to port 80 without root ---
# The cap_net_bind_service capability allows binding to privileged ports (<1024).
# IMPORTANT: When a binary has Linux capabilities, the dynamic linker ignores
# LD_LIBRARY_PATH for security.  We must therefore register ROS2 library paths
# via ldconfig so the linker can still find them.
#
# GOTCHA: every `apt upgrade` of the python3.12 package strips this capability,
# which is what caused the live "PermissionError: [Errno 13]" failure on port 80.
# setup.sh now runs a preflight check that falls back to WEBUI_PORT=8080 when
# the cap is missing; re-running environment.sh is the way to restore port 80.
if ! getcap /usr/bin/python3.12 2>/dev/null | grep -q cap_net_bind_service; then
    sudo setcap cap_net_bind_service=ep /usr/bin/python3.12
    log_info "cap_net_bind_service set on /usr/bin/python3.12 (port 80 binding)"
else
    log_info "cap_net_bind_service already set on python3.12"
fi

# --- Register ROS2 library paths in ldconfig ---
# Required because cap_net_bind_service causes the linker to ignore LD_LIBRARY_PATH.
ROS2_LDCONFIG="/etc/ld.so.conf.d/ros2-jazzy.conf"
if [[ ! -f "$ROS2_LDCONFIG" ]]; then
    printf '%s\n' '/opt/ros/jazzy/lib' '/opt/ros/jazzy/lib/aarch64-linux-gnu' \
        | sudo tee "$ROS2_LDCONFIG" > /dev/null
    sudo ldconfig
    log_info "ROS2 library paths registered in ldconfig"
else
    log_info "ROS2 ldconfig entry already exists."
fi

log_info "WiFi AP & networking setup complete."

# =============================================================================
# SECTION 7: Docker & PostgreSQL
# =============================================================================
log_info "=== Section 7: Docker & PostgreSQL ==="

if ! command -v docker &>/dev/null; then
    sudo apt-get install -y docker.io docker-compose-v2
    sudo systemctl enable --now docker
    sudo usermod -aG docker "$USER"
    log_info "Docker installed. NOTE: log out and back in for group membership to take effect."
else
    log_info "Docker already installed."
fi

# Start the PostgreSQL container
DOCKER_DIR="${SCRIPT_DIR}/docker"
if [[ -f "${DOCKER_DIR}/docker-compose.yaml" ]]; then
    if [[ ! -f "${DOCKER_DIR}/.env" ]]; then
        if [[ -f "${DOCKER_DIR}/.env.example" ]]; then
            cp "${DOCKER_DIR}/.env.example" "${DOCKER_DIR}/.env"
            sed -i 's/POSTGRES_PASSWORD=CHANGE_ME/POSTGRES_PASSWORD=gabi/' "${DOCKER_DIR}/.env"
            chmod 600 "${DOCKER_DIR}/.env"
            log_info "docker/.env created from .env.example (review credentials in docker/.env)"
        else
            log_warn "docker/.env and .env.example not found — skipping PostgreSQL"
        fi
    fi
    if [[ -f "${DOCKER_DIR}/.env" ]]; then
        cd "${DOCKER_DIR}"
        docker compose up -d 2>/dev/null || sudo docker compose up -d
        cd "${SCRIPT_DIR}"
        log_info "PostgreSQL container started."
    fi
else
    log_warn "docker/docker-compose.yaml not found — skipping PostgreSQL setup"
fi

# =============================================================================
# SECTION 8: Workspace Build
# =============================================================================
log_info "=== Section 8: Workspace Build ==="

# ROS2 already sourced in Section 3 — no need to re-source
cd "$SCRIPT_DIR"

if [[ -d "src" ]]; then
    colcon build --symlink-install || log_warn "colcon build had warnings/errors — check output above"
    log_info "Workspace built."
else
    log_warn "src/ directory not found in $SCRIPT_DIR — skipping colcon build"
fi

# =============================================================================
# SECTION 9: Environment File (~/.bashrc)
# =============================================================================
log_info "=== Section 9: Environment File ==="

BASHRC="$HOME/.bashrc"
ROS2_SOURCE_LINE="source /opt/ros/jazzy/setup.bash"
WS_SOURCE_LINE="source ${SCRIPT_DIR}/install/setup.bash 2>/dev/null || true"
VENV_LINE="source ${VENV_DIR}/bin/activate"

append_if_missing() {
    local line="$1"
    local file="$2"
    if ! grep -qF "$line" "$file" 2>/dev/null; then
        echo "$line" >> "$file"
        log_info "Appended to $file: $line"
    fi
}

append_if_missing "# Recon-Platform-R2 environment" "$BASHRC"
append_if_missing "$ROS2_SOURCE_LINE" "$BASHRC"
append_if_missing "$WS_SOURCE_LINE" "$BASHRC"
append_if_missing "$VENV_LINE" "$BASHRC"
append_if_missing "export RECON_DB_URL=postgresql://roomba:gabi@localhost:5432/roomba" "$BASHRC"

log_info "Environment lines added to ~/.bashrc"

# =============================================================================
# SECTION 9.5: Auto-start systemd unit (recon-stack.service)
# =============================================================================
# Install + enable the unit that runs `setup.sh full` at boot. The unit file
# lives in the repo (roomba_ws/systemd/recon-stack.service) so it's tracked
# in git; we copy it to /etc/systemd/system/ and re-copy on every run so
# edits to the source are picked up.
#
# Disabling auto-start without uninstalling:
#   sudo systemctl disable recon-stack.service
#
# To remove the unit entirely:
#   sudo systemctl disable recon-stack.service
#   sudo rm /etc/systemd/system/recon-stack.service
#   sudo systemctl daemon-reload
log_info "=== Section 9.5: Auto-start systemd unit ==="

RECON_UNIT_SRC="${SCRIPT_DIR}/systemd/recon-stack.service"
RECON_UNIT_DST="/etc/systemd/system/recon-stack.service"

if [[ ! -f "$RECON_UNIT_SRC" ]]; then
    log_warn "Service unit template missing at $RECON_UNIT_SRC — skipping auto-start setup"
else
    # Re-copy on every run so source edits propagate. cmp avoids a needless
    # daemon-reload when there's no change.
    if [[ ! -f "$RECON_UNIT_DST" ]] || ! sudo cmp -s "$RECON_UNIT_SRC" "$RECON_UNIT_DST"; then
        sudo install -m 644 "$RECON_UNIT_SRC" "$RECON_UNIT_DST"
        sudo systemctl daemon-reload
        log_info "Installed/updated $RECON_UNIT_DST"
    else
        log_info "recon-stack.service already up-to-date at $RECON_UNIT_DST"
    fi

    # Enable for next boot (idempotent — no-op if already enabled).
    if ! systemctl is-enabled --quiet recon-stack.service; then
        sudo systemctl enable recon-stack.service
        log_info "Enabled recon-stack.service for auto-start at boot"
    else
        log_info "recon-stack.service already enabled."
    fi
fi

# =============================================================================
# SECTION 9.6: Sudoers rule for the hardware front-panel buttons
# =============================================================================
# The web UI (running as this user) restarts the stack on the START/STOP
# button and powers off the Pi on a SHUTDOWN long-press. Both need root. We
# grant a NOPASSWD rule scoped to exactly those two commands — nothing else.
#
# To remove: sudo rm /etc/sudoers.d/recon-buttons
log_info "=== Section 9.6: Sudoers rule for hardware buttons ==="

RECON_USER="$(id -un)"
SYSTEMCTL_BIN="$(command -v systemctl || echo /usr/bin/systemctl)"
SHUTDOWN_BIN="/usr/sbin/shutdown"
[[ -x "$SHUTDOWN_BIN" ]] || SHUTDOWN_BIN="$(command -v shutdown || echo /sbin/shutdown)"
SUDOERS_DST="/etc/sudoers.d/recon-buttons"
SUDOERS_TMP="$(mktemp)"

# Keep these command lines byte-for-byte in sync with webui.yaml's
# `buttons.restart_cmd` / `buttons.shutdown_cmd` — sudoers matches the exact
# argv the web UI invokes.
cat > "$SUDOERS_TMP" <<EOF
# Managed by roomba_ws/environment.sh — recon hardware buttons (START/STOP + SHUTDOWN).
${RECON_USER} ALL=(root) NOPASSWD: ${SYSTEMCTL_BIN} --no-block restart recon-stack.service
${RECON_USER} ALL=(root) NOPASSWD: ${SYSTEMCTL_BIN} restart recon-stack.service
${RECON_USER} ALL=(root) NOPASSWD: ${SYSTEMCTL_BIN} stop recon-stack.service
${RECON_USER} ALL=(root) NOPASSWD: ${SHUTDOWN_BIN} -h now
EOF

# Validate before installing — a malformed sudoers file can lock out sudo.
if sudo visudo -cf "$SUDOERS_TMP" >/dev/null 2>&1; then
    if [[ ! -f "$SUDOERS_DST" ]] || ! sudo cmp -s "$SUDOERS_TMP" "$SUDOERS_DST"; then
        sudo install -m 440 -o root -g root "$SUDOERS_TMP" "$SUDOERS_DST"
        log_info "Installed $SUDOERS_DST (restart recon-stack.service + shutdown for $RECON_USER)"
    else
        log_info "recon-buttons sudoers rule already up-to-date."
    fi
else
    log_warn "Generated sudoers file failed validation — skipping (START/STOP + SHUTDOWN buttons will need a password)."
fi
rm -f "$SUDOERS_TMP"

fi  # end of MODE == "install"

# =============================================================================
# SECTION 10: Post-Install Verification (ALWAYS runs — both install and check)
# =============================================================================
if [[ "$MODE" == "check" ]]; then
    log_info "Running in --check mode: skipping installation, verifying environment only."
fi
log_section "Environment Verification"

echo ""
echo "=== RECON ENVIRONMENT VERIFICATION ==="
echo ""

# --- Check / check_warn helpers ---
PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

check() {
    local name="$1"
    local cmd="$2"
    set +u +o pipefail
    if eval "$cmd" &>/dev/null; then
        echo -e "  [${GREEN}PASS${NC}] $name"
        VERIFY_RESULTS["$name"]="PASS"
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo -e "  [${RED}FAIL${NC}] $name"
        VERIFY_RESULTS["$name"]="FAIL"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
    set -u -o pipefail
}

check_warn() {
    local name="$1"
    local cmd="$2"
    set +u +o pipefail
    if eval "$cmd" &>/dev/null; then
        echo -e "  [${GREEN}PASS${NC}] $name"
        VERIFY_RESULTS["$name"]="PASS"
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo -e "  [${YELLOW}WARN${NC}] $name"
        VERIFY_RESULTS["$name"]="WARN"
        WARN_COUNT=$((WARN_COUNT + 1))
    fi
    set -u -o pipefail
}

# Checks that need root (e.g. reading mode-600 files). Uses `sudo -n` so we
# never block on a password prompt — if sudo isn't already cached, the check
# emits WARN (not FAIL) with a hint to run `sudo -v` first. This avoids the
# class of false-negative where every hostapd.conf check shows FAIL just
# because the running shell didn't have a fresh sudo timestamp.
check_sudo() {
    local name="$1"
    local cmd="$2"
    set +u +o pipefail
    if ! sudo -n true 2>/dev/null; then
        echo -e "  [${YELLOW}WARN${NC}] $name  (sudo not cached — run 'sudo -v' first to verify)"
        VERIFY_RESULTS["$name"]="WARN"
        WARN_COUNT=$((WARN_COUNT + 1))
        set -u -o pipefail
        return
    fi
    if eval "$cmd" &>/dev/null; then
        echo -e "  [${GREEN}PASS${NC}] $name"
        VERIFY_RESULTS["$name"]="PASS"
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo -e "  [${RED}FAIL${NC}] $name"
        VERIFY_RESULTS["$name"]="FAIL"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
    set -u -o pipefail
}

# ─── 1. Core Software ───────────────────────────────────────────────────────
log_section "1. Core Software"
check "ROS2 Jazzy installed"                "[[ -f /opt/ros/jazzy/setup.bash ]]"
check "ROS2 Jazzy sourceable"               "bash -c 'source /opt/ros/jazzy/setup.bash && command -v ros2' 2>/dev/null"
check "colcon available"                     "command -v colcon"
check "tmux installed"                       "command -v tmux"
check "cmake installed"                      "command -v cmake"
check "git installed"                        "command -v git"
check "curl installed"                       "command -v curl"
check "Python 3.11+ available"              "python3 -c 'import sys; assert sys.version_info >= (3,11)'"

# ─── 2. ROS2 Packages ───────────────────────────────────────────────────────
log_section "2. ROS2 Packages"
check "slam_toolbox package"                 "dpkg -l ros-jazzy-slam-toolbox 2>/dev/null | grep -q '^ii'"
check "robot_localization package"           "dpkg -l ros-jazzy-robot-localization 2>/dev/null | grep -q '^ii'"
check "std_srvs package"                     "dpkg -l ros-jazzy-std-srvs 2>/dev/null | grep -q '^ii'"
check "tf2_ros package"                      "dpkg -l ros-jazzy-tf2-ros 2>/dev/null | grep -q '^ii'"
check "tf2_msgs package"                     "dpkg -l ros-jazzy-tf2-msgs 2>/dev/null | grep -q '^ii'"
check "ament-cmake-gtest"                    "dpkg -l ros-jazzy-ament-cmake-gtest 2>/dev/null | grep -q '^ii'"
check "Google Test (libgtest-dev)"           "dpkg -l libgtest-dev 2>/dev/null | grep -q '^ii'"
check "python3-serial (apt)"                 "dpkg -l python3-serial 2>/dev/null | grep -q '^ii'"
check "pyserial importable (system)"         "python3 -c 'import serial' 2>/dev/null"

# ─── 3. Python Virtual Environment ──────────────────────────────────────────
log_section "3. Python Virtual Environment"
check "venv exists"                          "[[ -d '${VENV_DIR}' && -f '${VENV_DIR}/bin/activate' ]]"
check "venv has --system-site-packages"      "[[ -f '${VENV_DIR}/pyvenv.cfg' ]] && grep -q 'include-system-site-packages = true' '${VENV_DIR}/pyvenv.cfg'"
check "flask importable"                     "${VENV_DIR}/bin/python3 -c 'import flask' 2>/dev/null"
check "flask-socketio importable"            "${VENV_DIR}/bin/python3 -c 'import flask_socketio' 2>/dev/null"
check "eventlet importable"                  "${VENV_DIR}/bin/python3 -c 'import eventlet' 2>/dev/null"
check "sqlalchemy importable"                "${VENV_DIR}/bin/python3 -c 'import sqlalchemy' 2>/dev/null"
check "numpy importable"                     "${VENV_DIR}/bin/python3 -c 'import numpy' 2>/dev/null"
check "pyyaml importable"                    "${VENV_DIR}/bin/python3 -c 'import yaml' 2>/dev/null"
check "pytest importable"                    "${VENV_DIR}/bin/python3 -c 'import pytest' 2>/dev/null"
check "psycopg2 importable"                 "${VENV_DIR}/bin/python3 -c 'import psycopg2' 2>/dev/null"
check "alembic importable"                  "${VENV_DIR}/bin/python3 -c 'import alembic' 2>/dev/null"

# ─── 4. Docker & PostgreSQL ─────────────────────────────────────────────────
log_section "4. Docker & PostgreSQL"
check "docker installed"                     "command -v docker"
check "docker daemon running"                "docker info &>/dev/null || sudo docker info &>/dev/null"
check_warn "docker-compose available"        "docker compose version &>/dev/null || sudo docker compose version &>/dev/null"
check_warn "user in docker group"            "groups | grep -q docker"
check_warn "roomba_postgres container running" "docker ps --format '{{.Names}}' 2>/dev/null | grep -q roomba_postgres"
check_warn "PostgreSQL accepting connections" "docker exec roomba_postgres pg_isready -U roomba 2>/dev/null"

# ─── 5. LIDAR UART ─────────────────────────────────────────────────────────
log_section "5. LIDAR UART"
check_warn "/dev/ttyAMA0 exists"                 "[[ -e /dev/ttyAMA0 ]]"
check_warn "UART enabled in boot config"         "grep -q '^enable_uart=1' /boot/firmware/config.txt 2>/dev/null"
check_warn "UART0 overlay in boot config"        "grep -q '^dtoverlay=uart0' /boot/firmware/config.txt 2>/dev/null"
check_warn "miniuart-bt overlay in boot config"  "grep -q '^dtoverlay=miniuart-bt' /boot/firmware/config.txt 2>/dev/null"
check_warn "No serial console in cmdline.txt"    "! grep -q 'console=serial0' /boot/firmware/cmdline.txt 2>/dev/null"
check_warn "Serial console disabled"             "! systemctl is-enabled serial-getty@ttyAMA0.service 2>/dev/null"
check_warn "Serial console masked"               "systemctl is-enabled serial-getty@ttyAMA0.service 2>/dev/null | grep -q masked"
check_warn "ttyAMA0 udev rule exists"            "[[ -f /etc/udev/rules.d/99-lidar-uart.rules ]]"
check_warn "ttyAMA0 group is dialout"            "[[ \$(stat -c '%G' /dev/ttyAMA0 2>/dev/null) == 'dialout' ]]"

# ─── 6. WiFi Access Point & Networking ─────────────────────────────────────
log_section "6. WiFi Access Point & Networking"
check "hostapd installed"                    "command -v hostapd"
check "dnsmasq installed"                    "command -v dnsmasq"
check "iw installed"                         "command -v iw"
check "hostapd.conf exists"                  "[[ -f /etc/hostapd/hostapd.conf ]]"
check_sudo "hostapd.conf SSID is Recon"      "sudo -n grep -q '^ssid=Recon' /etc/hostapd/hostapd.conf"
check_sudo "hostapd.conf uses ap0"           "sudo -n grep -q '^interface=ap0' /etc/hostapd/hostapd.conf"
check_sudo "hostapd.conf WPA2 enabled"       "sudo -n grep -q '^wpa=2' /etc/hostapd/hostapd.conf"
check_sudo "hostapd.conf mode 600"           "[[ \$(sudo -n stat -c '%a' /etc/hostapd/hostapd.conf 2>/dev/null) == '600' ]]"
check "dnsmasq recon.conf exists"            "[[ -f /etc/dnsmasq.d/recon.conf ]]"
check "dnsmasq resolves recon.local"         "grep -q 'address=/recon.local/' /etc/dnsmasq.d/recon.conf 2>/dev/null"
check "dnsmasq DHCP range configured"        "grep -q 'dhcp-range=10.0.0.10' /etc/dnsmasq.d/recon.conf 2>/dev/null"
check "dnsmasq no-dhcp on wlan0"             "grep -q 'no-dhcp-interface=wlan0' /etc/dnsmasq.d/recon.conf 2>/dev/null"
check "dnsmasq uses bind-dynamic"            "grep -q '^bind-dynamic' /etc/dnsmasq.d/recon.conf 2>/dev/null"
check_warn "dnsmasq.service active"          "systemctl is-active dnsmasq"
check "recon-ap start script exists"         "[[ -x /usr/local/bin/recon-ap-start.sh ]]"
check "recon-ap stop script exists"          "[[ -x /usr/local/bin/recon-ap-stop.sh ]]"
check "recon-ap.service unit exists"         "[[ -f /etc/systemd/system/recon-ap.service ]]"
check "recon-ap.service enabled"             "systemctl is-enabled recon-ap 2>/dev/null | grep -q enabled"
check_warn "recon-ap service active"         "systemctl is-active recon-ap"
check_warn "AP interface ap0 exists"         "ip link show ap0 &>/dev/null"
check_warn "ap0 has IP 10.0.0.1"             "ip -4 addr show ap0 2>/dev/null | grep -q '10.0.0.1'"
check_warn "hostapd unmasked"                "! systemctl is-enabled hostapd 2>&1 | grep -q 'masked'"
check "dnsmasq standalone disabled"          "! systemctl is-enabled dnsmasq 2>/dev/null | grep -q '^enabled'"
check "python3.12 cap_net_bind_service"      "getcap /usr/bin/python3.12 2>/dev/null | grep -q cap_net_bind_service"
check "ROS2 ldconfig entry exists"           "[[ -f /etc/ld.so.conf.d/ros2-jazzy.conf ]]"
check "librcl_action.so in ldconfig cache"   "ldconfig -p 2>/dev/null | grep -q librcl_action"

# ─── 6b. Auto-start service ────────────────────────────────────────────────
log_section "6b. Auto-start (recon-stack.service)"
check "recon-stack.service template in repo" "[[ -f '${SCRIPT_DIR}/systemd/recon-stack.service' ]]"
check "recon-stack.service unit installed"   "[[ -f /etc/systemd/system/recon-stack.service ]]"
check "recon-stack.service enabled"          "systemctl is-enabled recon-stack 2>/dev/null | grep -q enabled"
check "recon-stack User=gabi"                "grep -q '^User=gabi' /etc/systemd/system/recon-stack.service 2>/dev/null"
check "recon-stack ExecStart uses setup.sh"  "grep -q 'setup.sh full' /etc/systemd/system/recon-stack.service 2>/dev/null"
check_warn "recon-stack.service in repo == /etc/" "cmp -s '${SCRIPT_DIR}/systemd/recon-stack.service' /etc/systemd/system/recon-stack.service"

# ─── 7. Workspace & Build ─────────────────────────────────────────────────
log_section "7. Workspace & Build"
check "src/ directory exists"                "[[ -d '${SCRIPT_DIR}/src' ]]"
check "recon_bringup package"                "[[ -d '${SCRIPT_DIR}/src/recon_bringup' ]]"
check "recon_control package"                "[[ -d '${SCRIPT_DIR}/src/recon_control' ]]"
check "recon_db package"                     "[[ -d '${SCRIPT_DIR}/src/recon_db' ]]"
check "recon_hardware package"               "[[ -d '${SCRIPT_DIR}/src/recon_hardware' ]]"
check "recon_webui package"                  "[[ -d '${SCRIPT_DIR}/src/recon_webui' ]]"
check_warn "workspace built (install/ exists)" "[[ -d '${SCRIPT_DIR}/install' ]]"
check_warn "All 5 packages in install/"      "[[ -d '${SCRIPT_DIR}/install/recon_bringup' ]] && [[ -d '${SCRIPT_DIR}/install/recon_control' ]] && [[ -d '${SCRIPT_DIR}/install/recon_db' ]] && [[ -d '${SCRIPT_DIR}/install/recon_hardware' ]] && [[ -d '${SCRIPT_DIR}/install/recon_webui' ]]"

# ─── 8. Config Files ─────────────────────────────────────────────────────
log_section "8. Config Files"
check "config/webui.yaml exists"             "[[ -f '${SCRIPT_DIR}/config/webui.yaml' ]]"
check "webui.yaml port is 80"                "grep -q 'port: 80' '${SCRIPT_DIR}/config/webui.yaml' 2>/dev/null"
check "webui.yaml host is 0.0.0.0"           "grep -q 'host:.*0.0.0.0' '${SCRIPT_DIR}/config/webui.yaml' 2>/dev/null"
check "config/hardware.yaml exists"          "[[ -f '${SCRIPT_DIR}/config/hardware.yaml' ]]"
check "config/slam_params.yaml exists"       "[[ -f '${SCRIPT_DIR}/config/slam_params.yaml' ]]"
check "config/simulation.yaml exists"        "[[ -f '${SCRIPT_DIR}/config/simulation.yaml' ]]"
check "config/ekf.yaml exists"               "[[ -f '${SCRIPT_DIR}/config/ekf.yaml' ]]"
check "config/esp32_bridge.yaml exists"      "[[ -f '${SCRIPT_DIR}/config/esp32_bridge.yaml' ]]"

# ─── 9. Web UI Assets ────────────────────────────────────────────────────
log_section "9. Web UI Assets"
WEBUI_DIR="${SCRIPT_DIR}/src/recon_webui/recon_webui"
check "socket.io.min.js bundled"             "[[ -s '${WEBUI_DIR}/static/js/socket.io.min.js' ]]"
check "base.html uses local socket.io"       "grep -q \"url_for('static'\" '${WEBUI_DIR}/templates/base.html' 2>/dev/null || grep -q 'url_for(\"static\"' '${WEBUI_DIR}/templates/base.html' 2>/dev/null"
check "base.html no CDN socket.io"           "! grep -q 'cdnjs.cloudflare.com' '${WEBUI_DIR}/templates/base.html' 2>/dev/null"
check "app.py exists"                        "[[ -f '${WEBUI_DIR}/app.py' ]]"
check "ros_bridge.py exists"                 "[[ -f '${WEBUI_DIR}/ros_bridge.py' ]]"
check "data_channels.py exists"              "[[ -f '${WEBUI_DIR}/data_channels.py' ]]"
check "mock_data.py exists"                  "[[ -f '${WEBUI_DIR}/mock_data.py' ]]"

# ─── 10. Shell Environment (.bashrc) ─────────────────────────────────────
log_section "10. Shell Environment"
check "~/.bashrc sources ROS2"               "grep -qF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc 2>/dev/null"
check "~/.bashrc sources workspace overlay"  "grep -qF 'install/setup.bash' ~/.bashrc 2>/dev/null"
check "~/.bashrc activates venv"             "grep -qF '${VENV_DIR}/bin/activate' ~/.bashrc 2>/dev/null"
check "~/.bashrc sets RECON_DB_URL"          "grep -qF 'RECON_DB_URL' ~/.bashrc 2>/dev/null"

# ─── 11. Test Skeletons ──────────────────────────────────────────────────
log_section "11. Test Skeletons"
TESTS_DIR="${SCRIPT_DIR}/tests"
check "test_draw_node.cpp"                   "[[ -f '${TESTS_DIR}/test_draw_node.cpp' ]]"
check "test_db_node.py"                      "[[ -f '${TESTS_DIR}/test_db_node.py' ]]"
check "test_recon_webui.py"                  "[[ -f '${TESTS_DIR}/test_recon_webui.py' ]]"
check "test_sim_sensor_node.cpp"             "[[ -f '${TESTS_DIR}/test_sim_sensor_node.cpp' ]]"
check "test_imu_yaw_integrator.py"           "[[ -f '${TESTS_DIR}/test_imu_yaw_integrator.py' ]]"
check "test_esp32_uart_bridge.py"            "[[ -f '${TESTS_DIR}/test_esp32_uart_bridge.py' ]]"
check "test_postprocess.py"                  "[[ -f '${TESTS_DIR}/test_postprocess.py' ]]"

# ─── Summary ─────────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
TOTAL=$((PASS_COUNT + FAIL_COUNT + WARN_COUNT))
echo -e "  Total checks: ${TOTAL}"
echo -e "  ${GREEN}PASS: ${PASS_COUNT}${NC}  │  ${RED}FAIL: ${FAIL_COUNT}${NC}  │  ${YELLOW}WARN: ${WARN_COUNT}${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if [[ $FAIL_COUNT -gt 0 ]]; then
    log_error "$FAIL_COUNT critical check(s) failed. Review output above."
    if [[ "$MODE" == "check" ]]; then
        log_info "Run './environment.sh' (without --check) to install missing components."
    fi
    exit 1
else
    if [[ $WARN_COUNT -gt 0 ]]; then
        log_info "All critical checks passed. $WARN_COUNT warning(s) — typically hardware-dependent."
    else
        log_info "All checks passed. Environment fully matches project spec."
    fi
    if [[ "$MODE" == "install" ]]; then
        log_info "Reboot if UART was newly enabled, then connect LIDAR."
    fi
fi
