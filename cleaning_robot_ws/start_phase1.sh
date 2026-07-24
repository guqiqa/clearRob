#!/bin/bash
# ============================================================================
# start_phase1.sh — Phase 1 chassis bringup for cleaning robot
# ============================================================================
# Prerequisites:
#   1. CAN bus setup:  sudo ip link set can0 type can bitrate 500000
#                      sudo ip link set up can0
#   2. Serial permissions: sudo chmod 666 /dev/ttyS* /dev/qmi8658_imu
#   3. ROS2 Humble installed and sourced
#
# Usage:
#   ./start_phase1.sh              # Real hardware mode
#   ./start_phase1.sh --sim        # Simulation mode (no hardware)
#   ./start_phase1.sh --help       # Show help
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# ---- Colors ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ---- Parse args ----
SIM_MODE="false"
while [[ $# -gt 0 ]]; do
    case $1 in
        --sim|-s) SIM_MODE="true"; shift ;;
        --help|-h)
            echo "Usage: $0 [--sim|-s] [--help|-h]"
            echo "  --sim, -s    Run in simulation mode (no hardware required)"
            echo "  --help, -h   Show this help"
            exit 0
            ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; exit 1 ;;
    esac
done

# ---- Check ROS2 ----
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ros2 not found. Source ROS2 setup.bash first.${NC}"
    exit 1
fi

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  Cleaning Robot — Phase 1 Chassis Bringup   ${NC}"
echo -e "${GREEN}============================================${NC}"
echo "  Simulation mode: ${YELLOW}${SIM_MODE}${NC}"
echo ""

# ---- Hardware setup (real mode) ----
if [ "$SIM_MODE" = "false" ]; then
    echo "--- Hardware Setup ---"

    # CAN bus
    echo -n "  CAN bus (can0)... "
    if ip link show can0 &>/dev/null; then
        echo -e "${GREEN}OK${NC} ($(ip -br link show can0 | awk '{print $2}'))"
    else
        echo -e "${RED}NOT FOUND${NC}"
        echo "    Run: sudo ip link set can0 type can bitrate 500000"
        echo "         sudo ip link set up can0"
    fi

    # IMU
    echo -n "  IMU (/dev/qmi8658_imu)... "
    if [ -e "/dev/qmi8658_imu" ]; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${YELLOW}NOT FOUND${NC} (will fail if simulate=false)"
    fi

    # RTK
    echo -n "  RTK (/dev/ttyS2)... "
    if [ -e "/dev/ttyS2" ]; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${YELLOW}NOT FOUND${NC} (will fail if simulate=false)"
    fi

    # Remote
    echo -n "  Remote (/dev/ttyS1)... "
    if [ -e "/dev/ttyS1" ]; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${YELLOW}NOT FOUND${NC} (will fail if simulate=false)"
    fi

    echo ""
fi

# ---- Build workspace (if needed) ----
if [ ! -d "$WS_ROOT/install" ]; then
    echo "--- Building workspace (first time) ---"
    cd "$WS_ROOT"
    # Use sequential executor for ARM64 4GB RAM safety
    colcon build --executor sequential --symlink-install
    echo ""
fi

# ---- Source workspace ----
echo "--- Sourcing workspace ---"
source "$WS_ROOT/install/setup.bash"

# ---- Launch ----
echo ""
echo "--- Launching Phase 1 nodes ---"
echo ""

ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:="$SIM_MODE"
