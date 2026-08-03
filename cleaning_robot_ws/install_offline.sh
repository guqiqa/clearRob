#!/bin/bash
# Phase 1 offline installer — no colcon, no internet required
# Installs all 7 packages via pip3 develop mode

SOURCE_ROS="/opt/ros/humble/setup.bash"
WS="/root/cleaning_robot_ws"

if [ ! -f "$SOURCE_ROS" ]; then
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble"
    exit 1
fi

source "$SOURCE_ROS"

echo "=== Phase 1 Offline Package Installer ==="
echo ""

for pkg_path in \
    src/cleaning_robot_common \
    src/cleaning_robot_bringup \
    src/cleaning_robot_drivers/chassis_driver \
    src/cleaning_robot_drivers/remote_driver \
    src/cleaning_robot_drivers/imu_driver \
    src/cleaning_robot_drivers/rtk_driver \
    src/master_bridge
do
    echo -n "  $pkg_path ... "
    cd "$WS/$pkg_path"
    if python3 setup.py develop --no-deps 2>/dev/null; then
        echo "OK"
    else
        echo "FAILED"
    fi

    # Create resource marker for ros2
    pkg_name=$(basename "$pkg_path")
    mkdir -p "$WS/install/$pkg_name/share/ament_index/resource_index/packages"
    touch "$WS/install/$pkg_name/share/ament_index/resource_index/packages/$pkg_name"
    mkdir -p "$WS/install/$pkg_name/share/$pkg_name"
    if [ -f "$WS/$pkg_path/package.xml" ]; then
        cp "$WS/$pkg_path/package.xml" "$WS/install/$pkg_name/share/$pkg_name/"
    fi
done

# Copy config and launch files
mkdir -p "$WS/install/cleaning_robot_bringup/share/cleaning_robot_bringup/launch"
mkdir -p "$WS/install/cleaning_robot_bringup/share/cleaning_robot_bringup/config"
cp "$WS/src/cleaning_robot_bringup/launch/"*.py "$WS/install/cleaning_robot_bringup/share/cleaning_robot_bringup/launch/" 2>/dev/null
cp "$WS/src/cleaning_robot_bringup/config/"*.yaml "$WS/install/cleaning_robot_bringup/share/cleaning_robot_bringup/config/" 2>/dev/null

echo ""
echo "=== Done! Launch with: ==="
echo "  cd $WS && source install/setup.bash && ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false"
echo ""
echo "But first, generate setup.bash:"
echo "  python3 -c \"from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('cleaning_robot_bringup'))\" 2>/dev/null"

# Create a simple setup.bash
cat > "$WS/install/setup.bash" << 'SETUPEOF'
#!/bin/bash
# Auto-generated Phase 1 setup
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for dir in "$WS_DIR"/install/*/; do
    [ -d "$dir" ] && export PATH="$dir/lib/$(basename "${dir%/}"):$PATH"
done
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
export PYTHONPATH="$WS_DIR/src:$WS_DIR/src/cleaning_robot_common:$WS_DIR/src/cleaning_robot_drivers/chassis_driver:$WS_DIR/src/cleaning_robot_drivers/remote_driver:$WS_DIR/src/cleaning_robot_drivers/imu_driver:$WS_DIR/src/cleaning_robot_drivers/rtk_driver:$WS_DIR/src/master_bridge:$WS_DIR/install/cleaning_robot_common/lib/python3.10/site-packages:$PYTHONPATH"
SETUPEOF

chmod +x "$WS/install/setup.bash"
echo "setup.bash created"
