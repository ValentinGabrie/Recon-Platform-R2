#!/usr/bin/env bash
# =============================================================================
# rebuild.sh — Quick rebuild script for the roomba workspace
#
# Rebuilds changed packages with colcon. Limits parallelism to avoid OOM
# on the Pi 5. Optionally rebuilds only specific packages.
#
# Usage:
#   ./rebuild.sh              # Rebuild all changed packages
#   ./rebuild.sh nav           # Rebuild roomba_navigation only
#   ./rebuild.sh hw            # Rebuild roomba_hardware only
#   ./rebuild.sh web           # Rebuild roomba_webui only
#   ./rebuild.sh ctrl          # Rebuild roomba_control only
#   ./rebuild.sh db            # Rebuild roomba_db only
#   ./rebuild.sh all           # Force rebuild everything
#   ./rebuild.sh clean         # Clean build/ install/ log/ and rebuild all
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${CYAN}[rebuild]${NC} $*"; }
log_ok()    { echo -e "${GREEN}[rebuild]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[rebuild]${NC} $*"; }
log_error() { echo -e "${RED}[rebuild]${NC} $*"; }

# Source ROS2
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
set -u

# Package short-name aliases
declare -A PKG_ALIASES=(
    [nav]=roomba_navigation
    [hw]=roomba_hardware
    [web]=roomba_webui
    [ctrl]=roomba_control
    [db]=roomba_db
    [bringup]=roomba_bringup
)

MODE="${1:-}"

case "$MODE" in
    clean)
        log_warn "Cleaning build/ install/ log/ directories..."
        rm -rf build/ install/ log/
        log_info "Rebuilding everything..."
        MAKEFLAGS="-j2" colcon build --symlink-install --executor sequential 2>&1
        ;;
    all)
        log_info "Rebuilding all packages..."
        MAKEFLAGS="-j2" colcon build --symlink-install --executor sequential 2>&1
        ;;
    "")
        log_info "Rebuilding changed packages..."
        MAKEFLAGS="-j2" colcon build --symlink-install --executor sequential 2>&1
        ;;
    -h|--help|help)
        head -16 "$0" | tail -12
        exit 0
        ;;
    *)
        # Resolve alias or use as-is
        PKG="${PKG_ALIASES[$MODE]:-$MODE}"
        if [[ ! -d "src/$PKG" ]]; then
            log_error "Unknown package: $MODE (resolved to $PKG)"
            log_info "Available: ${!PKG_ALIASES[*]} or full package names"
            exit 1
        fi
        log_info "Rebuilding $PKG..."
        MAKEFLAGS="-j2" colcon build --symlink-install --executor sequential \
            --packages-select "$PKG" 2>&1
        ;;
esac

RC=$?
if [[ $RC -eq 0 ]]; then
    log_ok "Build succeeded."
    # Re-source workspace overlay for the current shell
    set +u
    source install/setup.bash 2>/dev/null
    set -u
else
    log_error "Build failed (exit code $RC)"
fi

exit $RC
