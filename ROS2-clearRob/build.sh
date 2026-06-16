#!/bin/bash
# Build and setup script for the cleaning robot ROS2 workspace
# Run from the ROS2-clearRob directory

set -e

echo "========================================"
echo " Cleaning Robot ROS2 Workspace Builder"
echo "========================================"
echo ""

# Check ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 environment not sourced."
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS2 Distribution: $ROS_DISTRO"
echo ""

# Install dependencies
echo "[1/4] Installing dependencies..."
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init 2>/dev/null || true
fi
rosdep update 2>/dev/null || true
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || echo "Some dependencies may need manual installation."
echo ""

# Build workspace
echo "[2/4] Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo ""

# Source setup
echo "[3/4] Sourcing setup..."
source install/setup.bash
echo ""

# Verify build
echo "[4/4] Verifying packages..."
colcon list
echo ""

echo "========================================"
echo " Build complete!"
echo ""
echo " To run the system:"
echo "   source install/setup.bash"
echo "   ros2 launch cleaning_robot_bringup cleaning_robot.launch.py"
echo ""
echo " For simulation:"
echo "   ros2 launch cleaning_robot_simulation simulation.launch.py"
echo "========================================"
