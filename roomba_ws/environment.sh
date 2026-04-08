#!/usr/bin/env bash
# =============================================================================
# environment.sh — Fully idempotent environment reproduction script
# Installs every dependency needed to build and run the roomba project
# from a clean Ubuntu Server 24.04 LTS (ARM64) install.
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
#   - pigpio is built from source (not in Ubuntu 24.04 repos)
#   - ROS2 sourcing uses set +u to handle unset bash variables
#   - Python venv uses --system-site-packages for rclpy access
#   - joy_linux package (evdev-based) is used instead of SDL2 joy_node
#     to avoid haptic/force-feedback errors with xpadneo
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
# SECTION 2: I2C Tools
# =============================================================================
log_info "=== Section 2: I2C Tools ==="

sudo apt-get install -y i2c-tools libi2c-dev

# Enable I2C on Pi 5 if config.txt exists
BOOT_CONFIG="/boot/firmware/config.txt"
if [[ -f "$BOOT_CONFIG" ]]; then
    if ! grep -q "^dtparam=i2c_arm=on" "$BOOT_CONFIG"; then
        echo "dtparam=i2c_arm=on" | sudo tee -a "$BOOT_CONFIG" > /dev/null
        log_info "I2C enabled in $BOOT_CONFIG (reboot required to take effect)."
    else
        log_info "I2C already enabled in $BOOT_CONFIG."
    fi
else
    log_warn "$BOOT_CONFIG not found — not running on Pi? I2C config skipped."
fi

log_info "I2C tools installed."

# =============================================================================
# SECTION 3: pigpio (build from source — NOT available in Ubuntu 24.04 repos)
# =============================================================================
log_info "=== Section 3: pigpio (from source) ==="

# Note: Do NOT use 'apt install libpigpio-dev' — the package does not exist
# in Ubuntu 24.04 Noble. Must build from joan2937/pigpio on GitHub.
if command -v pigpiod &>/dev/null || [[ -f /usr/local/lib/libpigpio.so ]] || [[ -f /usr/lib/libpigpio.so ]]; then
    log_info "pigpio already installed, skipping build."
else
    PIGPIO_DIR="/tmp/pigpio_build"
    rm -rf "$PIGPIO_DIR"
    wget -q https://github.com/joan2937/pigpio/archive/master.zip -O /tmp/pigpio.zip
    unzip -q /tmp/pigpio.zip -d /tmp
    mv /tmp/pigpio-master "$PIGPIO_DIR"
    cd "$PIGPIO_DIR"
    make -j"$(nproc)"
    sudo make install
    sudo ldconfig
    cd "$SCRIPT_DIR"
    sudo rm -rf "$PIGPIO_DIR" /tmp/pigpio.zip
    log_info "pigpio built and installed from source."
fi

# Create systemd service for pigpiod
if ! systemctl list-unit-files 2>/dev/null | grep -q pigpiod; then
    sudo tee /etc/systemd/system/pigpiod.service > /dev/null <<'EOF'
[Unit]
Description=pigpio daemon
After=network.target

[Service]
ExecStart=/usr/local/bin/pigpiod -l
Type=forking

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
fi

sudo systemctl enable pigpiod 2>/dev/null || true
sudo systemctl start pigpiod 2>/dev/null || true
log_info "pigpiod service enabled and started."

# =============================================================================
# SECTION 4: ROS2 Jazzy Jalisco
# =============================================================================
log_info "=== Section 4: ROS2 Jazzy Jalisco ==="

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
# SECTION 5: ROS2 Packages
# =============================================================================
log_info "=== Section 5: ROS2 Packages ==="

# We use joy_linux_node (evdev-based) to avoid haptic errors with xpadneo.
# joy_linux is a separate package from joy on Jazzy.
sudo apt-get install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-core \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-util \
    ros-jazzy-joy \
    ros-jazzy-joy-linux \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-tf2-ros \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    python3-colcon-common-extensions \
    libgtest-dev
# Note: several packages above are transitive deps of others in this list
# (e.g. rclcpp/rclpy via ros-core, msg packages via nav2/slam/joy,
# libgtest-dev via ament-cmake-gtest). They are listed explicitly so a
# fresh install is always complete regardless of upstream dep changes.

log_info "ROS2 packages installed."

# =============================================================================
# SECTION 6: Python Virtual Environment
# =============================================================================
log_info "=== Section 6: Python Virtual Environment ==="

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
# SECTION 7: xpadneo (Xbox Controller Bluetooth Driver)
# =============================================================================
log_info "=== Section 7: xpadneo ==="

sudo apt-get install -y dkms

# Install kernel headers — try generic first, fall back to current
sudo apt-get install -y "linux-headers-$(uname -r)" 2>/dev/null \
    || sudo apt-get install -y linux-headers-generic 2>/dev/null \
    || log_warn "Could not install kernel headers — xpadneo DKMS may fail"

if ! dkms status 2>/dev/null | grep -q xpadneo; then
    XPADNEO_DIR="/tmp/xpadneo"
    if [[ -d "$XPADNEO_DIR" ]]; then
        rm -rf "$XPADNEO_DIR"
    fi
    git clone https://github.com/atar-axis/xpadneo.git "$XPADNEO_DIR"
    cd "$XPADNEO_DIR"
    sudo ./install.sh || log_warn "xpadneo install failed — may need matching kernel headers"
    cd "$SCRIPT_DIR"
    rm -rf "$XPADNEO_DIR"
    log_info "xpadneo installed."
else
    log_info "xpadneo already installed, skipping."
fi

# Configure xpadneo
XPADNEO_CONF="/etc/modprobe.d/xpadneo.conf"
if [[ ! -f "$XPADNEO_CONF" ]]; then
    sudo tee "$XPADNEO_CONF" > /dev/null <<'EOF'
options xpadneo trigger_rumble_damping=4
options xpadneo disable_ff=0
EOF
    log_info "xpadneo configuration written to $XPADNEO_CONF"
else
    log_info "xpadneo configuration already exists."
fi

# =============================================================================
# SECTION 8: bluez (Bluetooth stack)
# =============================================================================
log_info "=== Section 8: bluez ==="

sudo apt-get install -y bluez
sudo systemctl enable bluetooth 2>/dev/null || true
sudo systemctl start bluetooth 2>/dev/null || true

# Disable ERTM — Xbox controllers fail to create HID input with ERTM enabled
ERTM_CONF="/etc/modprobe.d/bluetooth-ertm.conf"
if [[ ! -f "$ERTM_CONF" ]]; then
    echo 'options bluetooth disable_ertm=Y' | sudo tee "$ERTM_CONF" > /dev/null
    log_info "ERTM disabled via $ERTM_CONF (reboot required for full effect)"
else
    log_info "ERTM config already exists."
fi
# Also disable at runtime (takes effect immediately, no reboot needed)
if [[ -w /sys/module/bluetooth/parameters/disable_ertm ]]; then
    echo 1 | sudo tee /sys/module/bluetooth/parameters/disable_ertm > /dev/null
fi

log_info "bluez installed and bluetooth service started."

# =============================================================================
# SECTION 9: WiFi Access Point & Networking
# =============================================================================
log_info "=== Section 9: WiFi Access Point & Networking ==="

# Install hostapd (WiFi AP daemon), dnsmasq (DNS+DHCP), iw (interface management)
sudo apt-get install -y hostapd dnsmasq iw rfkill

# Ubuntu masks hostapd after install — unmask it so we can start it later
sudo systemctl unmask hostapd 2>/dev/null || true

# dnsmasq auto-starts on install and conflicts with systemd-resolved (port 53).
# We don't need it running yet — roomba-ap-start.sh will restart it after ap0 exists.
sudo systemctl stop dnsmasq 2>/dev/null || true
sudo systemctl disable dnsmasq 2>/dev/null || true

# --- hostapd configuration ---
HOSTAPD_CONF="/etc/hostapd/hostapd.conf"
if [[ ! -f "$HOSTAPD_CONF" ]] || ! grep -q "ssid=Roomba" "$HOSTAPD_CONF" 2>/dev/null; then
    sudo tee "$HOSTAPD_CONF" > /dev/null <<'HOSTAPD_EOF'
interface=ap0
driver=nl80211
ssid=Roomba
hw_mode=g
channel=1
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=roomba123
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
HOSTAPD_EOF
    sudo chmod 600 "$HOSTAPD_CONF"
    log_info "hostapd.conf written (SSID: Roomba, WPA2, mode 600)"
else
    log_info "hostapd.conf already configured."
fi

# --- dnsmasq configuration ---
DNSMASQ_CONF="/etc/dnsmasq.d/roomba.conf"
# Get the current wlan0 IP for DNS resolution on LAN
# Note: || true guards against pipefail exiting the script if wlan0 has no IPv4
WLAN0_IP=$(ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -1 || true)
WLAN0_IP="${WLAN0_IP:-172.31.225.193}"
# Get default gateway for DNS forwarding
ROUTER_IP=$(ip route 2>/dev/null | grep default | awk '{print $3}' | head -1 || true)
ROUTER_IP="${ROUTER_IP:-172.31.225.213}"

if [[ ! -f "$DNSMASQ_CONF" ]]; then
    sudo tee "$DNSMASQ_CONF" > /dev/null <<DNSMASQ_EOF
# Roomba AP — DHCP on ap0, DNS on both interfaces
interface=ap0
interface=wlan0
bind-interfaces

# DHCP only on hotspot (ap0), NOT on wlan0 (avoid conflicting with router)
dhcp-range=10.0.0.10,10.0.0.50,24h
no-dhcp-interface=wlan0

# DNS: resolve roomba.local to Pi on both networks
address=/roomba.local/10.0.0.1
address=/roomba.local/${WLAN0_IP}
address=/gabi.local/10.0.0.1
address=/gabi.local/${WLAN0_IP}

# Forward other DNS queries
server=${ROUTER_IP}
server=8.8.8.8
DNSMASQ_EOF
    log_info "dnsmasq.conf written (DHCP on ap0, DNS on ap0+wlan0)"
else
    log_info "dnsmasq.conf already exists, skipping (delete to regenerate)."
fi

# --- roomba-ap systemd service + helper scripts ---
sudo tee /usr/local/bin/roomba-ap-start.sh > /dev/null <<'AP_START_EOF'
#!/bin/bash
set -e

# Verify wlan0 exists before attempting to create AP
if ! ip link show wlan0 &>/dev/null; then
    echo "[roomba-ap] ERROR: wlan0 not found — cannot create AP" >&2
    exit 1
fi

# Create virtual AP interface from wlan0
if ! ip link show ap0 &>/dev/null; then
    iw dev wlan0 interface add ap0 type __ap || {
        echo "[roomba-ap] ERROR: failed to create ap0 from wlan0" >&2
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
sudo chmod +x /usr/local/bin/roomba-ap-start.sh

sudo tee /usr/local/bin/roomba-ap-stop.sh > /dev/null <<'AP_STOP_EOF'
#!/bin/bash
systemctl stop hostapd 2>/dev/null || true
ip link set ap0 down 2>/dev/null || true
iw dev ap0 del 2>/dev/null || true
AP_STOP_EOF
sudo chmod +x /usr/local/bin/roomba-ap-stop.sh

ROOMBA_AP_SERVICE="/etc/systemd/system/roomba-ap.service"
if [[ ! -f "$ROOMBA_AP_SERVICE" ]]; then
    sudo tee "$ROOMBA_AP_SERVICE" > /dev/null <<'SERVICE_EOF'
[Unit]
Description=Roomba WiFi Access Point (ap0)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/roomba-ap-start.sh
ExecStop=/usr/local/bin/roomba-ap-stop.sh

[Install]
WantedBy=multi-user.target
SERVICE_EOF
    sudo systemctl daemon-reload
    log_info "roomba-ap.service created."
else
    log_info "roomba-ap.service already exists."
fi

sudo systemctl enable roomba-ap.service 2>/dev/null || true
sudo systemctl start roomba-ap.service 2>/dev/null || true

# --- Bundle socket.io client library (no CDN dependency) ---
SOCKETIO_JS="${SCRIPT_DIR}/src/roomba_webui/roomba_webui/static/js/socket.io.min.js"
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
# SECTION 10: Docker & PostgreSQL
# =============================================================================
log_info "=== Section 10: Docker & PostgreSQL ==="

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
# SECTION 11: Workspace Build
# =============================================================================
log_info "=== Section 11: Workspace Build ==="

# ROS2 already sourced in Section 4 — no need to re-source
cd "$SCRIPT_DIR"

if [[ -d "src" ]]; then
    colcon build --symlink-install || log_warn "colcon build had warnings/errors — check output above"
    log_info "Workspace built."
else
    log_warn "src/ directory not found in $SCRIPT_DIR — skipping colcon build"
fi

# =============================================================================
# SECTION 12: Environment File (~/.bashrc)
# =============================================================================
log_info "=== Section 12: Environment File ==="

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

append_if_missing "# Roomba project environment" "$BASHRC"
append_if_missing "$ROS2_SOURCE_LINE" "$BASHRC"
append_if_missing "$WS_SOURCE_LINE" "$BASHRC"
append_if_missing "$VENV_LINE" "$BASHRC"
append_if_missing "export ROOMBA_DB_URL=postgresql://roomba:gabi@localhost:5432/roomba" "$BASHRC"

log_info "Environment lines added to ~/.bashrc"

fi  # end of MODE == "install"

# =============================================================================
# SECTION 13: Post-Install Verification (ALWAYS runs — both install and check)
# =============================================================================
if [[ "$MODE" == "check" ]]; then
    log_info "Running in --check mode: skipping installation, verifying environment only."
fi
log_section "Environment Verification"

echo ""
echo "=== ROOMBA ENVIRONMENT VERIFICATION ==="
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
check "nav2 bringup package"                 "dpkg -l ros-jazzy-nav2-bringup 2>/dev/null | grep -q '^ii'"
check "nav2 bt-navigator"                    "dpkg -l ros-jazzy-nav2-bt-navigator 2>/dev/null | grep -q '^ii'"
check "nav2 controller"                      "dpkg -l ros-jazzy-nav2-controller 2>/dev/null | grep -q '^ii'"
check "nav2 costmap-2d"                      "dpkg -l ros-jazzy-nav2-costmap-2d 2>/dev/null | grep -q '^ii'"
check "nav2 lifecycle-manager"               "dpkg -l ros-jazzy-nav2-lifecycle-manager 2>/dev/null | grep -q '^ii'"
check "nav2 map-server"                      "dpkg -l ros-jazzy-nav2-map-server 2>/dev/null | grep -q '^ii'"
check "nav2 planner"                         "dpkg -l ros-jazzy-nav2-planner 2>/dev/null | grep -q '^ii'"
check "nav2 behaviors"                       "dpkg -l ros-jazzy-nav2-behaviors 2>/dev/null | grep -q '^ii'"
check "joy package"                          "dpkg -l ros-jazzy-joy 2>/dev/null | grep -q '^ii'"
check "joy_linux package"                    "dpkg -l ros-jazzy-joy-linux 2>/dev/null | grep -q '^ii'"
check "teleop-twist-joy package"             "dpkg -l ros-jazzy-teleop-twist-joy 2>/dev/null | grep -q '^ii'"
check "ament-cmake-gtest"                    "dpkg -l ros-jazzy-ament-cmake-gtest 2>/dev/null | grep -q '^ii'"
check "Google Test (libgtest-dev)"           "dpkg -l libgtest-dev 2>/dev/null | grep -q '^ii'"

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

# ─── 5. pigpio ───────────────────────────────────────────────────────────────
log_section "5. pigpio"
check "pigpio library installed"             "command -v pigpiod || [[ -f /usr/local/lib/libpigpio.so ]] || [[ -f /usr/lib/libpigpio.so ]]"
check "pigpiod systemd unit exists"          "systemctl list-unit-files 2>/dev/null | grep -q pigpiod"
check_warn "pigpiod service active"          "systemctl is-active pigpiod"

# ─── 6. I2C ─────────────────────────────────────────────────────────────────
log_section "6. I2C"
check "i2c-tools installed"                  "command -v i2cdetect"
check_warn "I2C bus /dev/i2c-1 accessible"   "[[ -e /dev/i2c-1 ]]"
check_warn "I2C enabled in boot config"      "grep -q '^dtparam=i2c_arm=on' /boot/firmware/config.txt 2>/dev/null"
check_warn "ESP32 detected on I2C (addr 0x42)" "i2cdetect -y 1 2>/dev/null | grep -q '42'"

# ─── 7. Xbox Controller (xpadneo + bluez) ───────────────────────────────────
log_section "7. Xbox Controller (xpadneo + bluez)"
check "bluez installed"                      "command -v bluetoothctl"
check_warn "bluetooth service active"        "systemctl is-active bluetooth"
check "dkms installed"                       "command -v dkms"
check "xpadneo DKMS registered"             "dkms status 2>/dev/null | grep -q xpadneo"
check_warn "xpadneo module loaded"           "lsmod | grep -q xpadneo"
check "xpadneo config exists"                "[[ -f /etc/modprobe.d/xpadneo.conf ]]"
check "ERTM disabled (modprobe)"             "[[ -f /etc/modprobe.d/bluetooth-ertm.conf ]]"
check "ERTM disabled (runtime)"              "[[ \$(cat /sys/module/bluetooth/parameters/disable_ertm 2>/dev/null) == Y ]]"

# ─── 8. WiFi Access Point & Networking ───────────────────────────────────────
log_section "8. WiFi Access Point & Networking"
check "hostapd installed"                    "command -v hostapd"
check "dnsmasq installed"                    "command -v dnsmasq"
check "iw installed"                         "command -v iw"
check "hostapd.conf exists"                  "[[ -f /etc/hostapd/hostapd.conf ]]"
check "hostapd.conf SSID is Roomba"          "sudo grep -q '^ssid=Roomba' /etc/hostapd/hostapd.conf 2>/dev/null"
check "hostapd.conf uses ap0"               "sudo grep -q '^interface=ap0' /etc/hostapd/hostapd.conf 2>/dev/null"
check "hostapd.conf WPA2 enabled"           "sudo grep -q '^wpa=2' /etc/hostapd/hostapd.conf 2>/dev/null"
check "hostapd.conf mode 600"               "[[ $(sudo stat -c '%a' /etc/hostapd/hostapd.conf 2>/dev/null) == '600' ]]"
check "dnsmasq roomba.conf exists"           "[[ -f /etc/dnsmasq.d/roomba.conf ]]"
check "dnsmasq resolves roomba.local"        "grep -q 'address=/roomba.local/' /etc/dnsmasq.d/roomba.conf 2>/dev/null"
check "dnsmasq DHCP range configured"        "grep -q 'dhcp-range=10.0.0.10' /etc/dnsmasq.d/roomba.conf 2>/dev/null"
check "dnsmasq no-dhcp on wlan0"             "grep -q 'no-dhcp-interface=wlan0' /etc/dnsmasq.d/roomba.conf 2>/dev/null"
check "roomba-ap start script exists"        "[[ -x /usr/local/bin/roomba-ap-start.sh ]]"
check "roomba-ap stop script exists"         "[[ -x /usr/local/bin/roomba-ap-stop.sh ]]"
check "roomba-ap.service unit exists"        "[[ -f /etc/systemd/system/roomba-ap.service ]]"
check "roomba-ap.service enabled"            "systemctl is-enabled roomba-ap 2>/dev/null | grep -q enabled"
check_warn "roomba-ap service active"        "systemctl is-active roomba-ap"
check_warn "AP interface ap0 exists"         "ip link show ap0 &>/dev/null"
check_warn "ap0 has IP 10.0.0.1"            "ip -4 addr show ap0 2>/dev/null | grep -q '10.0.0.1'"
check_warn "hostapd unmasked"               "! systemctl is-enabled hostapd 2>&1 | grep -q 'masked'"
check "dnsmasq standalone disabled"          "! systemctl is-enabled dnsmasq 2>/dev/null | grep -q '^enabled'"
check "python3.12 cap_net_bind_service"       "getcap /usr/bin/python3.12 2>/dev/null | grep -q cap_net_bind_service"
check "ROS2 ldconfig entry exists"            "[[ -f /etc/ld.so.conf.d/ros2-jazzy.conf ]]"
check "librcl_action.so in ldconfig cache"    "ldconfig -p 2>/dev/null | grep -q librcl_action"

# ─── 9. Workspace & Build ───────────────────────────────────────────────────
log_section "9. Workspace & Build"
check "src/ directory exists"                "[[ -d '${SCRIPT_DIR}/src' ]]"
check "roomba_bringup package"               "[[ -d '${SCRIPT_DIR}/src/roomba_bringup' ]]"
check "roomba_control package"               "[[ -d '${SCRIPT_DIR}/src/roomba_control' ]]"
check "roomba_db package"                    "[[ -d '${SCRIPT_DIR}/src/roomba_db' ]]"
check "roomba_hardware package"              "[[ -d '${SCRIPT_DIR}/src/roomba_hardware' ]]"
check "roomba_navigation package"            "[[ -d '${SCRIPT_DIR}/src/roomba_navigation' ]]"
check "roomba_webui package"                 "[[ -d '${SCRIPT_DIR}/src/roomba_webui' ]]"
check_warn "workspace built (install/ exists)" "[[ -d '${SCRIPT_DIR}/install' ]]"
check_warn "All 6 packages in install/"      "[[ -d '${SCRIPT_DIR}/install/roomba_bringup' ]] && [[ -d '${SCRIPT_DIR}/install/roomba_control' ]] && [[ -d '${SCRIPT_DIR}/install/roomba_db' ]] && [[ -d '${SCRIPT_DIR}/install/roomba_hardware' ]] && [[ -d '${SCRIPT_DIR}/install/roomba_navigation' ]] && [[ -d '${SCRIPT_DIR}/install/roomba_webui' ]]"

# ─── 10. Config Files ───────────────────────────────────────────────────────
log_section "10. Config Files"
check "config/webui.yaml exists"             "[[ -f '${SCRIPT_DIR}/config/webui.yaml' ]]"
check "webui.yaml port is 80"               "grep -q 'port: 80' '${SCRIPT_DIR}/config/webui.yaml' 2>/dev/null"
check "webui.yaml host is 0.0.0.0"          "grep -q 'host:.*0.0.0.0' '${SCRIPT_DIR}/config/webui.yaml' 2>/dev/null"
check "config/controller.yaml exists"        "[[ -f '${SCRIPT_DIR}/config/controller.yaml' ]]"
check "config/hardware.yaml exists"          "[[ -f '${SCRIPT_DIR}/config/hardware.yaml' ]]"
check "config/nav2_params.yaml exists"       "[[ -f '${SCRIPT_DIR}/config/nav2_params.yaml' ]]"
check "config/slam_params.yaml exists"       "[[ -f '${SCRIPT_DIR}/config/slam_params.yaml' ]]"
check "config/simulation.yaml exists"        "[[ -f '${SCRIPT_DIR}/config/simulation.yaml' ]]"

# ─── 11. Web UI Assets ─────────────────────────────────────────────────────
log_section "11. Web UI Assets"
WEBUI_DIR="${SCRIPT_DIR}/src/roomba_webui/roomba_webui"
check "socket.io.min.js bundled"             "[[ -s '${WEBUI_DIR}/static/js/socket.io.min.js' ]]"
check "base.html uses local socket.io"       "grep -q \"url_for('static'\" '${WEBUI_DIR}/templates/base.html' 2>/dev/null || grep -q 'url_for(\"static\"' '${WEBUI_DIR}/templates/base.html' 2>/dev/null"
check "base.html no CDN socket.io"           "! grep -q 'cdnjs.cloudflare.com' '${WEBUI_DIR}/templates/base.html' 2>/dev/null"
check "app.py exists"                        "[[ -f '${WEBUI_DIR}/app.py' ]]"
check "ros_bridge.py exists"                 "[[ -f '${WEBUI_DIR}/ros_bridge.py' ]]"
check "data_channels.py exists"              "[[ -f '${WEBUI_DIR}/data_channels.py' ]]"
check "mock_data.py exists"                  "[[ -f '${WEBUI_DIR}/mock_data.py' ]]"

# ─── 12. Shell Environment (.bashrc) ────────────────────────────────────────
log_section "12. Shell Environment"
check "~/.bashrc sources ROS2"               "grep -qF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc 2>/dev/null"
check "~/.bashrc sources workspace overlay"  "grep -qF 'install/setup.bash' ~/.bashrc 2>/dev/null"
check "~/.bashrc activates venv"             "grep -qF '${VENV_DIR}/bin/activate' ~/.bashrc 2>/dev/null"
check "~/.bashrc sets ROOMBA_DB_URL"         "grep -qF 'ROOMBA_DB_URL' ~/.bashrc 2>/dev/null"

# ─── 13. Test Skeletons ─────────────────────────────────────────────────────
log_section "13. Test Skeletons"
TESTS_DIR="${SCRIPT_DIR}/tests"
check "test_bt_sim_node.cpp"                 "[[ -f '${TESTS_DIR}/test_bt_sim_node.cpp' ]]"
check "test_joy_control_node.cpp"            "[[ -f '${TESTS_DIR}/test_joy_control_node.cpp' ]]"
check "test_motor_controller.cpp"            "[[ -f '${TESTS_DIR}/test_motor_controller.cpp' ]]"
check "test_esp32_sensor_node.cpp"           "[[ -f '${TESTS_DIR}/test_esp32_sensor_node.cpp' ]]"
check "test_slam_bridge_node.cpp"            "[[ -f '${TESTS_DIR}/test_slam_bridge_node.cpp' ]]"
check "test_recon_node.cpp"                  "[[ -f '${TESTS_DIR}/test_recon_node.cpp' ]]"
check "test_draw_node.cpp"                   "[[ -f '${TESTS_DIR}/test_draw_node.cpp' ]]"
check "test_db_node.py"                      "[[ -f '${TESTS_DIR}/test_db_node.py' ]]"
check "test_roomba_webui.py"                 "[[ -f '${TESTS_DIR}/test_roomba_webui.py' ]]"
check "test_sim_motor_node.cpp"               "[[ -f '${TESTS_DIR}/test_sim_motor_node.cpp' ]]"
check "test_sim_sensor_node.cpp"              "[[ -f '${TESTS_DIR}/test_sim_sensor_node.cpp' ]]"
check "test_sim_goal_follower.cpp"             "[[ -f '${TESTS_DIR}/test_sim_goal_follower.cpp' ]]"

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
        log_info "Reboot if I2C was newly enabled, then connect hardware."
    fi
fi
