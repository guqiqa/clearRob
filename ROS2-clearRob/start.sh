#!/bin/bash
# =============================================================================
# 智能环卫机器人控制系统 — 一键启动脚本
#
# 用法:
#   ./start.sh                    # 交互式菜单
#   ./start.sh build              # 仅构建
#   ./start.sh run                # 默认模拟模式运行
#   ./start.sh run_hardware       # 硬件模式运行
#   ./start.sh sim                # 仿真环境 (Gazebo)
#   ./start.sh multi robot_001    # 多机器人模式
#   ./start.sh check              # 环境检查
#   ./start.sh clean              # 清理工作空间
# =============================================================================

set -e

# ---- 颜色定义 ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ---- 路径 ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
SRC_DIR="$WORKSPACE_DIR/src"
BUILD_DIR="$WORKSPACE_DIR/build"
INSTALL_DIR="$WORKSPACE_DIR/install"
LOG_DIR="$WORKSPACE_DIR/logs"
PID_DIR="$WORKSPACE_DIR/.pids"

# ---- 默认参数 ----
ROS_DISTRO_DEFAULT="humble"

# =============================================================================
# 横幅
# =============================================================================
banner() {
    echo ""
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC}  ${BOLD}智能环卫机器人控制系统${NC}                                    ${CYAN}║${NC}"
    echo -e "${CYAN}║${NC}  Cleaning Robot Control System — ROS2 Humble                  ${CYAN}║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# =============================================================================
# 环境检查
# =============================================================================
check_env() {
    local errors=0

    echo -e "${BLUE}[检查]${NC} 系统环境..."

    # 检查操作系统
    if [ ! -f /etc/os-release ]; then
        echo -e "  ${RED}✗${NC} 无法检测操作系统"
        errors=$((errors + 1))
    else
        . /etc/os-release
        echo -e "  ${GREEN}✓${NC} 操作系统: $NAME $VERSION"
    fi

    # 检查 ROS2
    if [ -z "$ROS_DISTRO" ]; then
        # 尝试自动 source
        if [ -f "/opt/ros/$ROS_DISTRO_DEFAULT/setup.bash" ]; then
            source "/opt/ros/$ROS_DISTRO_DEFAULT/setup.bash"
            echo -e "  ${GREEN}✓${NC} ROS2 $ROS_DISTRO (自动加载)"
        else
            echo -e "  ${RED}✗${NC} ROS2 未安装或未 source"
            echo -e "    ${YELLOW}→${NC} 请运行: source /opt/ros/humble/setup.bash"
            errors=$((errors + 1))
        fi
    else
        echo -e "  ${GREEN}✓${NC} ROS2 $ROS_DISTRO"
    fi

    # 检查 Python
    if command -v python3 &>/dev/null; then
        local py_ver=$(python3 --version 2>&1 | awk '{print $2}')
        echo -e "  ${GREEN}✓${NC} Python $py_ver"
    else
        echo -e "  ${RED}✗${NC} Python3 未安装"
        errors=$((errors + 1))
    fi

    # 检查 colcon
    if command -v colcon &>/dev/null; then
        echo -e "  ${GREEN}✓${NC} colcon 已安装"
    else
        echo -e "  ${YELLOW}⚠${NC} colcon 未安装 (构建需要)"
    fi

    # 检查工作空间
    if [ -d "$SRC_DIR" ]; then
        local pkg_count=$(find "$SRC_DIR" -maxdepth 2 -name "package.xml" | wc -l)
        echo -e "  ${GREEN}✓${NC} 工作空间: $pkg_count 个包"
    else
        echo -e "  ${RED}✗${NC} src 目录不存在"
        errors=$((errors + 1))
    fi

    # 检查已构建状态
    if [ -f "$INSTALL_DIR/setup.bash" ]; then
        echo -e "  ${GREEN}✓${NC} 已构建 (可运行)"
    else
        echo -e "  ${YELLOW}⚠${NC} 未构建 (需先 build)"
    fi

    echo ""

    if [ $errors -gt 0 ]; then
        echo -e "${RED}环境检查失败: $errors 个错误${NC}"
        return 1
    else
        echo -e "${GREEN}环境检查通过${NC}"
        return 0
    fi
}

# =============================================================================
# 构建
# =============================================================================
do_build() {
    local clean_build="${1:-false}"

    echo -e "${BLUE}[构建]${NC} 开始构建工作空间..."

    # 确保 ROS2 已 source
    if [ -z "$ROS_DISTRO" ]; then
        source "/opt/ros/$ROS_DISTRO_DEFAULT/setup.bash" 2>/dev/null || {
            echo -e "${RED}错误: 请先 source ROS2 环境${NC}"
            return 1
        }
    fi

    cd "$WORKSPACE_DIR"

    # 清理
    if [ "$clean_build" = "true" ]; then
        echo -e "  ${YELLOW}→${NC} 清理旧构建..."
        rm -rf build install log
    fi

    # 创建日志目录
    mkdir -p "$LOG_DIR"

    # 安装依赖
    echo -e "  ${YELLOW}→${NC} 安装依赖..."
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init 2>/dev/null || true
    fi
    rosdep update 2>/dev/null || true
    rosdep install --from-paths src --ignore-src -r -y 2>&1 | tee "$LOG_DIR/rosdep.log" || {
        echo -e "  ${YELLOW}⚠${NC} 部分依赖需要手动安装"
    }

    # 构建
    echo -e "  ${YELLOW}→${NC} colcon 构建..."
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+ \
        2>&1 | tee "$LOG_DIR/build.log"

    if [ ${PIPESTATUS[0]} -eq 0 ]; then
        echo ""
        echo -e "${GREEN}构建成功!${NC}"

        # Source 构建结果
        source "$INSTALL_DIR/setup.bash"

        # 列出包
        echo ""
        echo -e "${CYAN}已构建包:${NC}"
        colcon list --base-paths "$SRC_DIR"
        echo ""
    else
        echo ""
        echo -e "${RED}构建失败, 请查看日志: $LOG_DIR/build.log${NC}"
        return 1
    fi
}

# =============================================================================
# 运行
# =============================================================================
do_run() {
    local mode="${1:-sim}"

    echo -e "${BLUE}[启动]${NC} 模式: $mode"

    # 确保已 source
    if [ -z "$ROS_DISTRO" ]; then
        source "/opt/ros/$ROS_DISTRO_DEFAULT/setup.bash" 2>/dev/null
    fi

    # 检查是否已构建
    if [ ! -f "$INSTALL_DIR/setup.bash" ]; then
        echo -e "${YELLOW}工作空间未构建, 自动构建...${NC}"
        do_build || return 1
    fi

    source "$INSTALL_DIR/setup.bash"

    # 创建日志和 PID 目录
    mkdir -p "$LOG_DIR" "$PID_DIR"

    case "$mode" in
        sim)
            echo -e "  ${GREEN}→${NC} 启动模拟模式 (全节点)..."
            echo ""
            ros2 launch cleaning_robot_bringup cleaning_robot.launch.py \
                sim_mode:=true \
                log_level:=info \
                2>&1 | tee "$LOG_DIR/run_sim_$(date +%Y%m%d_%H%M%S).log"
            ;;

        hardware)
            echo -e "  ${YELLOW}→${NC} 启动硬件模式 (需连接传感器/电机)..."
            echo ""
            ros2 launch cleaning_robot_bringup cleaning_robot.launch.py \
                sim_mode:=false \
                log_level:=info \
                2>&1 | tee "$LOG_DIR/run_hw_$(date +%Y%m%d_%H%M%S).log"
            ;;

        debug)
            echo -e "  ${YELLOW}→${NC} 启动调试模式 (DEBUG 日志级别)..."
            echo ""
            ros2 launch cleaning_robot_bringup cleaning_robot.launch.py \
                sim_mode:=true \
                log_level:=debug \
                2>&1 | tee "$LOG_DIR/run_debug_$(date +%Y%m%d_%H%M%S).log"
            ;;

        gazebo)
            echo -e "  ${GREEN}→${NC} 启动 Gazebo 仿真环境..."
            echo ""
            ros2 launch cleaning_robot_simulation simulation.launch.py \
                2>&1 | tee "$LOG_DIR/run_gazebo_$(date +%Y%m%d_%H%M%S).log"
            ;;

        gazebo_full)
            echo -e "  ${GREEN}→${NC} 启动 Gazebo + 全节点联调..."
            echo ""
            # 终端1: Gazebo
            ros2 launch cleaning_robot_simulation simulation.launch.py \
                2>&1 | tee "$LOG_DIR/run_gazebo_$(date +%Y%m%d_%H%M%S).log" &
            GAZEBO_PID=$!
            echo "$GAZEBO_PID" > "$PID_DIR/gazebo.pid"

            # 等待 Gazebo 就绪
            sleep 5

            # 终端2: 控制节点
            ros2 launch cleaning_robot_bringup cleaning_robot.launch.py \
                sim_mode:=false \
                2>&1 | tee "$LOG_DIR/run_full_$(date +%Y%m%d_%H%M%S).log" &
            CTRL_PID=$!
            echo "$CTRL_PID" > "$PID_DIR/control.pid"

            echo ""
            echo -e "${GREEN}Gazebo PID: $GAZEBO_PID${NC}"
            echo -e "${GREEN}控制节点 PID: $CTRL_PID${NC}"
            echo -e "${YELLOW}按 Ctrl+C 停止所有进程${NC}"

            # 等待
            wait $GAZEBO_PID $CTRL_PID 2>/dev/null || true
            ;;

        *)
            echo -e "${RED}未知模式: $mode${NC}"
            return 1
            ;;
    esac
}

# =============================================================================
# 多机器人模式
# =============================================================================
do_multi() {
    local robot_id="${1:-robot_001}"

    echo -e "${BLUE}[启动]${NC} 多机器人模式: $robot_id"
    echo ""

    if [ -z "$ROS_DISTRO" ]; then
        source "/opt/ros/$ROS_DISTRO_DEFAULT/setup.bash" 2>/dev/null
    fi

    if [ ! -f "$INSTALL_DIR/setup.bash" ]; then
        echo -e "${YELLOW}工作空间未构建, 自动构建...${NC}"
        do_build || return 1
    fi

    source "$INSTALL_DIR/setup.bash"
    mkdir -p "$LOG_DIR"

    ros2 launch cleaning_robot_bringup cleaning_robot.launch.py \
        robot_id:="$robot_id" \
        use_namespace:=true \
        sim_mode:=true \
        2>&1 | tee "$LOG_DIR/run_${robot_id}_$(date +%Y%m%d_%H%M%S).log"
}

# =============================================================================
# 节点状态监控
# =============================================================================
do_status() {
    if [ -z "$ROS_DISTRO" ]; then
        source "/opt/ros/$ROS_DISTRO_DEFAULT/setup.bash" 2>/dev/null
    fi
    if [ -f "$INSTALL_DIR/setup.bash" ]; then
        source "$INSTALL_DIR/setup.bash"
    fi

    echo -e "${BLUE}[状态]${NC} 系统运行状态"
    echo ""

    # ROS2 节点列表
    echo -e "${BOLD}ROS2 节点:${NC}"
    ros2 node list 2>/dev/null || echo -e "  ${YELLOW}无运行节点 (ros2 daemon 未启动?)${NC}"
    echo ""

    # Topic 列表
    echo -e "${BOLD}活跃 Topic (前20):${NC}"
    ros2 topic list 2>/dev/null | head -20 || echo "  ${YELLOW}无 Topic${NC}"
    echo ""

    # 心跳检查
    echo -e "${BOLD}心跳状态:${NC}"
    for node in control_gateway master_controller path_planner motion_controller localization_engine vision_detector cleaning_actuator lidar_perception fusion_engine; do
        if ros2 topic echo --once "system/heartbeat/$node" 2>/dev/null | grep -q "node_name"; then
            echo -e "  ${GREEN}●${NC} $node"
        else
            echo -e "  ${RED}○${NC} $node (离线)"
        fi
    done
    echo ""
}

# =============================================================================
# 停止所有进程
# =============================================================================
do_stop() {
    echo -e "${YELLOW}[停止]${NC} 停止所有机器人进程..."

    # 从 PID 文件停止
    if [ -d "$PID_DIR" ]; then
        for pidfile in "$PID_DIR"/*.pid; do
            if [ -f "$pidfile" ]; then
                local pid=$(cat "$pidfile")
                if kill -0 "$pid" 2>/dev/null; then
                    kill "$pid" 2>/dev/null && echo -e "  ${GREEN}✓${NC} 停止 PID $pid ($(basename "$pidfile"))"
                fi
                rm -f "$pidfile"
            fi
        done
    fi

    # 停止所有 ros2 进程
    pkill -f "ros2 launch" 2>/dev/null && echo -e "  ${GREEN}✓${NC} 停止 ros2 launch 进程" || true
    pkill -f "cleaning_robot" 2>/dev/null && echo -e "  ${GREEN}✓${NC} 停止 cleaning_robot 进程" || true

    echo -e "${GREEN}所有进程已停止${NC}"
}

# =============================================================================
# 清理
# =============================================================================
do_clean() {
    echo -e "${YELLOW}[清理]${NC} 清理工作空间..."

    cd "$WORKSPACE_DIR"

    rm -rf build install log
    rm -rf "$LOG_DIR" "$PID_DIR"

    echo -e "${GREEN}清理完成${NC}"
}

# =============================================================================
# 完整测试流程
# =============================================================================
do_test() {
    echo -e "${BLUE}[测试]${NC} 启动完整测试流程"
    echo ""

    # Step 1: 检查
    check_env || return 1

    # Step 2: 清理构建
    do_clean
    do_build true || return 1

    # Step 3: 启动模拟模式（后台 30s 冒烟测试）
    source "$INSTALL_DIR/setup.bash"
    mkdir -p "$LOG_DIR"

    echo -e "  ${YELLOW}→${NC} 冒烟测试 (30s)..."
    timeout 30 ros2 launch cleaning_robot_bringup cleaning_robot.launch.py \
        sim_mode:=true \
        2>&1 | tee "$LOG_DIR/smoke_test.log" || true

    echo ""
    echo -e "${GREEN}冒烟测试完成, 日志: $LOG_DIR/smoke_test.log${NC}"
}

# =============================================================================
# 交互式菜单
# =============================================================================
interactive_menu() {
    banner
    check_env || true

    echo -e "${BOLD}请选择操作:${NC}"
    echo ""
    echo -e "  ${GREEN}1${NC}) 构建工作空间"
    echo -e "  ${GREEN}2${NC}) 清理 + 重新构建"
    echo -e "  ${GREEN}3${NC}) 运行 (模拟模式)"
    echo -e "  ${GREEN}4${NC}) 运行 (硬件模式)"
    echo -e "  ${GREEN}5${NC}) 运行 (Gazebo 仿真)"
    echo -e "  ${GREEN}6${NC}) 运行 (Gazebo + 全节点联调)"
    echo -e "  ${GREEN}7${NC}) 调试模式运行 (DEBUG 日志)"
    echo -e "  ${GREEN}8${NC}) 多机器人模式"
    echo -e "  ${GREEN}9${NC}) 查看运行状态"
    echo -e "  ${GREEN}0${NC}) 停止所有进程"
    echo -e "  ${YELLOW}c${NC}) 清理工作空间"
    echo -e "  ${YELLOW}t${NC}) 完整测试流程"
    echo -e "  ${RED}q${NC}) 退出"
    echo ""

    read -p "请输入选项 [1-9, 0, c, t, q]: " choice

    case "$choice" in
        1) do_build ;;
        2) do_clean; do_build true ;;
        3) trap 'do_stop; exit 0' INT TERM; do_run sim ;;
        4) trap 'do_stop; exit 0' INT TERM; do_run hardware ;;
        5) trap 'do_stop; exit 0' INT TERM; do_run gazebo ;;
        6) trap 'do_stop; exit 0' INT TERM; do_run gazebo_full ;;
        7) trap 'do_stop; exit 0' INT TERM; do_run debug ;;
        8)
            read -p "输入机器人 ID [默认: robot_001]: " robot_id
            robot_id=${robot_id:-robot_001}
            trap 'do_stop; exit 0' INT TERM
            do_multi "$robot_id"
            ;;
        9) do_status ;;
        0) do_stop ;;
        c) do_clean ;;
        t) do_test ;;
        q) echo "退出"; exit 0 ;;
        *)
            echo -e "${RED}无效选项${NC}"
            ;;
    esac
}

# =============================================================================
# 帮助
# =============================================================================
show_help() {
    banner
    echo "用法: ./start.sh [命令] [参数]"
    echo ""
    echo -e "${BOLD}命令:${NC}"
    echo "  build            构建工作空间"
    echo "  rebuild          清理 + 重新构建"
    echo "  run              启动 (模拟模式, 默认)"
    echo "  run_hardware     启动 (硬件模式)"
    echo "  run_debug        启动 (调试模式)"
    echo "  sim              启动 Gazebo 仿真"
    echo "  sim_full         Gazebo + 全节点联调"
    echo "  multi <id>       多机器人模式 (指定 robot_id)"
    echo "  status           查看系统运行状态"
    echo "  stop             停止所有进程"
    echo "  clean            清理工作空间"
    echo "  test             完整测试流程 (构建+冒烟)"
    echo "  check            仅环境检查"
    echo "  help             显示此帮助"
    echo ""
    echo -e "${BOLD}示例:${NC}"
    echo "  ./start.sh                    # 交互式菜单"
    echo "  ./start.sh build              # 构建"
    echo "  ./start.sh run                # 运行模拟模式"
    echo "  ./start.sh multi robot_002    # 启动 robot_002"
    echo ""
}

# =============================================================================
# 主入口
# =============================================================================
main() {
    # 切换到工作空间目录
    cd "$WORKSPACE_DIR"

    case "${1:-menu}" in
        menu)
            interactive_menu
            ;;
        build)
            banner
            do_build false
            ;;
        rebuild)
            banner
            do_clean
            do_build true
            ;;
        run)
            banner
            echo -e "${YELLOW}按 Ctrl+C 停止${NC}"
            echo ""
            trap 'do_stop; exit 0' INT TERM
            do_run sim
            ;;
        run_hardware|hw)
            banner
            echo -e "${RED}警告: 硬件模式, 确保传感器和电机已连接!${NC}"
            echo ""
            read -p "确认继续? [y/N]: " confirm
            if [ "$confirm" != "y" ] && [ "$confirm" != "Y" ]; then
                echo "已取消"
                exit 0
            fi
            trap 'do_stop; exit 0' INT TERM
            do_run hardware
            ;;
        run_debug|debug)
            banner
            trap 'do_stop; exit 0' INT TERM
            do_run debug
            ;;
        sim|gazebo)
            banner
            trap 'do_stop; exit 0' INT TERM
            do_run gazebo
            ;;
        sim_full|gazebo_full)
            banner
            trap 'do_stop; exit 0' INT TERM
            do_run gazebo_full
            ;;
        multi)
            banner
            trap 'do_stop; exit 0' INT TERM
            do_multi "${2:-robot_001}"
            ;;
        status|st)
            do_status
            ;;
        stop)
            do_stop
            ;;
        clean)
            do_clean
            ;;
        test|smoke)
            banner
            do_test
            ;;
        check|env)
            banner
            check_env
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            echo -e "${RED}未知命令: $1${NC}"
            echo "运行 ./start.sh help 查看帮助"
            exit 1
            ;;
    esac
}

main "$@"
