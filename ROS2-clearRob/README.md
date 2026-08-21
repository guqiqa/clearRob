# Cleaning Robot Control System — ROS2 Workspace

## Overview

Based on the [需求文档 V2.5](../项目文档/需求文档_V2.5.md), this workspace implements a complete intelligent cleaning robot control system with 9 ROS2 nodes communicating via Topic/Service/Action.

## Architecture

```
control_gateway  →  master_controller  →  motion_controller
                         │                    (底盘驱动/里程计)
                         ├── path_planner
                         │    (全局覆盖/路径跟踪/避障)
                         ├── cleaning_actuator
                         │    (主刷/边刷/吸力/洒水)
                         ├── vision_detector
                         │    (YOLO 垃圾检测/清洁度)
                         ├── lidar_perception
                         │    (点云处理/障碍物/急停)
                         ├── localization_engine
                         │    (SLAM/建图/tf2)
                         └── fusion_engine
                              (视觉+LiDAR 后融合)
```

## Packages

| Package | Type | Language | Description |
|---------|------|----------|-------------|
| `cleaning_robot_interfaces` | ament_cmake | (IDL) | 24 msg + 7 srv + 3 action |
| `control_gateway` | ament_python | Python | 三源指令归一化与优先级裁决 |
| `master_controller` | ament_python | Python | 任务调度/状态机/运动仲裁 |
| `path_planner` | ament_python | Python | 路径规划/跟踪/避障重规划 |
| `motion_controller` | ament_python | Python | 底盘驱动/PID 控制/里程计 |
| `localization_engine` | ament_python | Python | SLAM 定位/建图/tf2 |
| `vision_detector` | ament_python | Python | 垃圾检测/清洁度/可通行区域 |
| `video_input` | ament_python | Python | 独立视频输入/RGB 与双目话题归一化 |
| `stereo_depth` | ament_python | Python | OpenCV 双目深度/SLAM 稀疏点云 |
| `cleaning_actuator` | ament_python | Python | 清扫策略执行/尘满检测 |
| `lidar_perception` | ament_python | Python | 点云预处理/障碍物/急停 |
| `fusion_engine` | ament_python | Python | 2D+3D 融合/避障决策 |
| `cleaning_robot_bringup` | ament_cmake | (config) | Launch + YAML config |
| `cleaning_robot_simulation` | ament_cmake | (assets) | Gazebo worlds + models |

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- colcon

## Build

```bash
cd ROS2-clearRob
source /opt/ros/humble/setup.bash
./build.sh
```

Or manually:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
source install/setup.bash

# Launch all nodes (simulation mode with simulated sensors)
ros2 launch cleaning_robot_bringup cleaning_robot.launch.py

# With Gazebo simulation
ros2 launch cleaning_robot_simulation simulation.launch.py

# Multi-robot mode
ros2 launch cleaning_robot_bringup cleaning_robot.launch.py robot_id:=robot_001 use_namespace:=true
ros2 launch cleaning_robot_bringup cleaning_robot.launch.py robot_id:=robot_002 use_namespace:=true
```

### Independent video input and stereo depth

The camera driver remains `S90cam-service.service`. The optional `video_input` package is the shared ROS2 image frontend: it republishes the configured camera source into stable RGB and left/right stereo topics. `vision_detector` consumes the RGB topic, while `stereo_depth` consumes the left/right topics for SLAM mapping only. Millimeter-wave radar remains the obstacle-avoidance source.

```bash
# Enable the shared video layer and SLAM depth frontend
ros2 launch cleaning_robot_bringup cleaning_robot.launch.py \
  sim_mode:=false use_video_input:=true use_stereo_depth:=true

# Check the depth frontend after deployment
curl http://127.0.0.1:8091/health
curl http://127.0.0.1:8091/latest
```

Set the real camera source topics in `src/video_input/config/video_input.yaml` before enabling the services. The current QSM602HA-GL board exposes a Hobot H.264 preview, but does not yet expose ROS2 left/right image topics; see `项目文档/双目测距与独立视频输入集成说明.md` for the deployment status and required calibration inputs. The current checkerboard target is `7x10` inner corners with `square_size_m: 0.02`.

## Control Interfaces

| Mode | Input | Example |
|------|-------|---------|
| RC Remote | `device/rc/raw` (RCRawSignal) | Short press → START, L1+R1 → ESTOP |
| Mobile App | `device/app/cmd` (AppCommand) | Map tap → SET_AREA |
| Voice | `device/voice/text` (String) | "开始清扫" → START |

Priority: RC > App > Voice

## Testing

All nodes include simulation modes for testing without hardware:

- `vision_detector`: `simulate_camera: true` — generates synthetic images + detections
- `lidar_perception`: `simulate_scans: true` — generates synthetic point clouds + obstacles
- `motion_controller`: Simulated PID motor response
- `localization_engine`: Simulated EKF-SLAM with controlled drift
