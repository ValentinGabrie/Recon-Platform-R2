#!/usr/bin/env bash
# =============================================================================
# firmware/esp32/environment.sh — Idempotent provisioner for the ESP32 I/O
# hub firmware.
#
# Mirrors the style of roomba_ws/environment.sh. Running this script twice
# must produce the same result without errors.
#
# What it does (in order):
#   1. Verifies prerequisites (python3 venv, lsusb, sudo for udev).
#   2. Creates ~/.platformio-venv if absent and installs PlatformIO Core
#      + pyserial into it.
#   3. Adds the venv's bin/ to PATH via ~/.bashrc (guarded).
#   4. Installs the 99-platformio-udev.rules so non-root flashing works
#      reliably across CP210x / CH340 dev boards.
#   5. Confirms the user is in the dialout group; warns + offers to add
#      if not.
#   6. Detects which /dev/ttyUSB* / /dev/ttyACM* is the ESP32.
#   7. Builds the firmware (pio run).
#   8. Flashes the firmware (pio run -t upload) with auto-retry on the
#      flaky DTR/RTS auto-reset.
#   9. Forces a known-good hard reset via esptool (auto-reset post-upload
#      lies on some boards; without this you keep running the OLD image).
#  10. Runs a 6-second decoder capture and grades it: IMU frames present,
#      heartbeats steady, no CRC failures.
#
# Modes:
#   ./environment.sh             Full install + build + flash + verify (default)
#   ./environment.sh --check     Verify-only: don't install, don't flash; just
#                                check tools and do the 6-s capture grade.
#   ./environment.sh --no-flash  Install + build, skip flash + verify.
#   ./environment.sh --help      Print this help.
#
# Optional environment variables:
#   ESP32_PORT=/dev/ttyUSB0      Override port autodetection.
#   PIO_VENV=~/.platformio-venv  Override PlatformIO venv location.
# =============================================================================

set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIO_VENV="${PIO_VENV:-$HOME/.platformio-venv}"
DECODER="${SCRIPT_DIR}/tools/decode_serial.py"

log_info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $*"; }
log_section() { echo -e "\n${BLUE}━━━ $* ━━━${NC}"; }

# ---- Argument parsing -------------------------------------------------------
MODE="install"
case "${1:-}" in
    --check|--verify|-c) MODE="check" ;;
    --no-flash)          MODE="no-flash" ;;
    --help|-h)
        sed -n '2,28p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    "")                  MODE="install" ;;
    *)
        log_error "Unknown argument: $1 — use --help"
        exit 1
        ;;
esac

# ---- Counters for the final summary ----------------------------------------
PASS=0; FAIL=0; WARN=0
check()      { if eval "$2" &>/dev/null; then echo -e "  [${GREEN}PASS${NC}] $1"; PASS=$((PASS+1)); else echo -e "  [${RED}FAIL${NC}] $1"; FAIL=$((FAIL+1)); fi; }
check_warn() { if eval "$2" &>/dev/null; then echo -e "  [${GREEN}PASS${NC}] $1"; PASS=$((PASS+1)); else echo -e "  [${YELLOW}WARN${NC}] $1"; WARN=$((WARN+1)); fi; }

# =============================================================================
# SECTION 1: Prerequisites
# =============================================================================
log_section "1. Prerequisites"
check "python3 available"                 "command -v python3"
check "python3 venv module available"     "python3 -m venv --help"
check "lsusb available (for port detect)" "command -v lsusb"
check_warn "sudo available (for udev rules)" "command -v sudo"

if [[ $FAIL -gt 0 ]]; then
    log_error "Missing prerequisites — install python3 + python3-venv + usbutils and retry."
    exit 1
fi

# =============================================================================
# SECTION 2: PlatformIO Core venv
# =============================================================================
log_section "2. PlatformIO Core"

if [[ "$MODE" == "install" || "$MODE" == "no-flash" ]]; then
    if [[ ! -d "$PIO_VENV" ]]; then
        log_info "Creating PlatformIO venv at $PIO_VENV"
        python3 -m venv "$PIO_VENV"
    else
        log_info "PlatformIO venv already exists at $PIO_VENV"
    fi

    # shellcheck disable=SC1091
    source "$PIO_VENV/bin/activate"

    pip install --quiet --upgrade pip
    pip install --quiet -U platformio pyserial
    log_info "platformio + pyserial installed/updated in venv"
    deactivate
else
    log_info "--check mode: skipping pip install"
fi

# shellcheck disable=SC1091
[[ -f "$PIO_VENV/bin/activate" ]] && source "$PIO_VENV/bin/activate" || true

check "PlatformIO venv exists"            "[[ -d '$PIO_VENV' ]]"
check "pio command available"             "command -v pio"
check "pyserial importable"               "python3 -c 'import serial' 2>/dev/null"

# =============================================================================
# SECTION 3: PATH integration via ~/.bashrc
# =============================================================================
log_section "3. PATH integration"

if [[ "$MODE" == "install" || "$MODE" == "no-flash" ]]; then
    BASHRC="$HOME/.bashrc"
    PATH_LINE='export PATH="$HOME/.platformio-venv/bin:$PATH"  # PlatformIO Core'
    if ! grep -qF "$PIO_VENV/bin" "$BASHRC" 2>/dev/null; then
        echo "" >> "$BASHRC"
        echo "$PATH_LINE" >> "$BASHRC"
        log_info "Added PlatformIO venv to PATH in $BASHRC (sourced on next login)"
    else
        log_info "PlatformIO venv already on PATH in $BASHRC"
    fi
fi
check_warn "pio on PATH in ~/.bashrc"     "grep -qF '$PIO_VENV/bin' '$HOME/.bashrc' 2>/dev/null"

# =============================================================================
# SECTION 4: udev rules for non-root serial access
# =============================================================================
log_section "4. udev rules"

UDEV_RULE="/etc/udev/rules.d/99-platformio-udev.rules"
UDEV_URL="https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules"

if [[ "$MODE" == "install" && ! -f "$UDEV_RULE" ]]; then
    if command -v sudo &>/dev/null && sudo -n true 2>/dev/null; then
        log_info "Installing $UDEV_RULE (passwordless sudo)"
        sudo curl -fsSL "$UDEV_URL" -o "$UDEV_RULE"
        sudo udevadm control --reload-rules && sudo udevadm trigger
    else
        log_warn "udev rules not installed — sudo would prompt for a password."
        log_warn "To install manually:"
        log_warn "  sudo curl -fsSL $UDEV_URL -o $UDEV_RULE"
        log_warn "  sudo udevadm control --reload-rules && sudo udevadm trigger"
        log_warn "Not fatal if you're already in the 'dialout' group."
    fi
fi
check_warn "99-platformio-udev.rules installed" "[[ -f '$UDEV_RULE' ]]"
check_warn "user in 'dialout' group"             "groups | grep -qw dialout"

# =============================================================================
# SECTION 5: Detect the ESP32 USB port
# =============================================================================
log_section "5. Detect ESP32"

if [[ -n "${ESP32_PORT:-}" ]]; then
    log_info "Using ESP32_PORT override: $ESP32_PORT"
elif lsusb | grep -qiE 'cp210x|CP21'; then
    log_info "CP210x USB-Serial bridge detected"
    ESP32_PORT="$(ls /dev/ttyUSB* 2>/dev/null | head -1 || true)"
elif lsusb | grep -qiE 'ch340|qinheng'; then
    log_info "CH340 USB-Serial bridge detected"
    ESP32_PORT="$(ls /dev/ttyUSB* 2>/dev/null | head -1 || true)"
elif lsusb | grep -qiE 'espressif'; then
    log_info "Native ESP32 USB-OTG detected (S3 / C3 variant)"
    ESP32_PORT="$(ls /dev/ttyACM* 2>/dev/null | head -1 || true)"
else
    log_warn "No known ESP32 USB-Serial bridge in lsusb output."
    log_warn "Plug in the ESP32 (micro-USB → Pi USB-A) and re-run."
    ESP32_PORT=""
fi

check_warn "/dev/tty* device for ESP32 found" "[[ -n '$ESP32_PORT' && -e '$ESP32_PORT' ]]"
[[ -n "$ESP32_PORT" ]] && log_info "Port: $ESP32_PORT"

# =============================================================================
# SECTION 6: Build the firmware
# =============================================================================
log_section "6. Build firmware"

cd "$SCRIPT_DIR"

if [[ "$MODE" == "install" || "$MODE" == "no-flash" ]]; then
    if ! command -v pio &>/dev/null; then
        log_error "pio not on PATH after install — source $PIO_VENV/bin/activate manually and retry."
        exit 1
    fi
    log_info "Running: pio run"
    if ! pio run 2>&1 | tail -8; then
        log_error "pio run failed — see output above."
        exit 1
    fi
    log_info "Firmware built."
fi

check "build artefact exists" "[[ -f '$SCRIPT_DIR/.pio/build/esp32dev/firmware.bin' ]]"

# =============================================================================
# SECTION 7: Flash + force hard reset
# =============================================================================
if [[ "$MODE" == "install" ]]; then
    log_section "7. Flash firmware"

    if [[ -z "$ESP32_PORT" || ! -e "$ESP32_PORT" ]]; then
        log_error "ESP32 not detected — cannot flash. Plug it in and re-run."
        exit 1
    fi

    # The DTR/RTS auto-reset on cheap CP210x boards is flaky on first try
    # ("Failed to connect to ESP32: No serial data received"). Retry up
    # to 3 times before giving up.
    flashed=0
    for attempt in 1 2 3; do
        log_info "Upload attempt $attempt of 3 to $ESP32_PORT"
        if pio run -t upload --upload-port "$ESP32_PORT" 2>&1 | tail -6; then
            flashed=1
            break
        fi
        log_warn "Attempt $attempt failed — retrying after 2 s"
        sleep 2
    done
    [[ $flashed -eq 1 ]] || { log_error "Upload failed after 3 attempts."; exit 1; }
    log_info "Flash complete."

    # The "Hard resetting via RTS pin..." that esptool prints lies on
    # some boards — the chip keeps running its old in-memory image.
    # Force a clean reset by re-entering the bootloader via esptool and
    # exiting with --after hard_reset.
    log_section "8. Force hard reset"
    ESPTOOL="$HOME/.platformio/packages/tool-esptoolpy/esptool.py"
    if [[ -f "$ESPTOOL" ]]; then
        log_info "Running esptool chip_id to trigger a known-good reset"
        python3 "$ESPTOOL" --port "$ESP32_PORT" --after hard_reset chip_id 2>&1 | tail -3
        sleep 1
    else
        log_warn "esptool.py not found at $ESPTOOL — relying on pio's RTS reset (may not stick)"
    fi
fi

# =============================================================================
# SECTION 9: Verify — capture frames and grade
# =============================================================================
if [[ "$MODE" == "install" || "$MODE" == "check" ]]; then
    log_section "9. Verify (6 s capture)"

    if [[ -z "$ESP32_PORT" || ! -e "$ESP32_PORT" ]]; then
        log_warn "No port — skipping live capture"
    elif [[ ! -f "$DECODER" ]]; then
        log_warn "Decoder not found at $DECODER — skipping live capture"
    else
        LOG="$(mktemp)"
        log_info "Capturing for 6 s to $LOG"
        timeout 6 python3 "$DECODER" "$ESP32_PORT" --no-color --imu-rate 1 \
            > "$LOG" 2>&1 || true

        IMU_N=$(grep -c '] IMU' "$LOG" || true)
        HB_N=$(grep -c '] HEARTBEAT' "$LOG" || true)
        ST_N=$(grep -c '] STATUS' "$LOG" || true)
        CRC_N=$(grep -c '] CRC_FAIL' "$LOG" || true)
        BT_N=$(grep -c '] BUTTON' "$LOG" || true)
        IMU_OK=$(grep -c 'IMU_OK' "$LOG" || true)

        log_info "Capture summary:  IMU=$IMU_N  HEARTBEAT=$HB_N  STATUS=$ST_N  BUTTON=$BT_N  CRC_FAIL=$CRC_N"
        check "frames flowing (>=1 IMU print)"          "[[ $IMU_N -ge 1 ]]"
        check "heartbeat cadence (>=4 in 6 s window)"   "[[ $HB_N -ge 4 ]]"
        check "CRC failures below threshold (<=2)"      "[[ $CRC_N -le 2 ]]"
        check_warn "boot STATUS frame caught (IMU_OK)"  "[[ $IMU_OK -ge 1 ]]"

        rm -f "$LOG"
    fi
fi

# =============================================================================
# Summary
# =============================================================================
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
TOTAL=$((PASS + FAIL + WARN))
echo -e "  Total checks: $TOTAL"
echo -e "  ${GREEN}PASS: $PASS${NC}  │  ${RED}FAIL: $FAIL${NC}  │  ${YELLOW}WARN: $WARN${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if [[ $FAIL -gt 0 ]]; then
    log_error "$FAIL critical check(s) failed."
    [[ "$MODE" == "check" ]] && log_info "Run without --check to install + flash."
    exit 1
fi

if [[ "$MODE" == "install" ]]; then
    log_info "ESP32 provisioned and bench-validated."
    log_info "Watch live frames with:"
    log_info "  source $PIO_VENV/bin/activate && python3 $DECODER $ESP32_PORT"
elif [[ "$MODE" == "no-flash" ]]; then
    log_info "Toolchain + build OK. Re-run without --no-flash to flash + verify."
else
    log_info "Verify-only run complete."
fi
