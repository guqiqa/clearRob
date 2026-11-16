# ROS 2 底盘控制接入（当前版本：V2.zip releases/20260921_134951_frozen_ros2_app）

## 启动
```bash
bash /root/ros2_bridge/start_chassis_ros2.sh
```
脚本会：停旧控制器 -> 起 SBUS 中位仿真（无接收机必需）-> 生成 config_emu.xml ->
带 ROS 2 环境启动控制器 -> 等待 /laifei_chassis_driver -> 自动开启 autonomous 模式。

## 接口
| 类型 | 名称 | 说明 |
|---|---|---|
| 订阅 | /cmd_vel (geometry_msgs/Twist) | autonomous 模式下的运动控制，300 ms 无指令自动停车 |
| 发布 | /odom (nav_msgs/Odometry) | 里程计 |
| 发布 | /tf | odom -> base_link |
| 服务 | /set_autonomous_mode (std_srvs/SetBool) | true=接受 /cmd_vel；false=切回 SBUS/APP 手动 |

## 控制
```bash
source /opt/ros/humble/setup.bash
python3 /root/ros2_bridge/chassis_cli.py forward 5
python3 /root/ros2_bridge/chassis_cli.py back 3
python3 /root/ros2_bridge/chassis_cli.py left 2
python3 /root/ros2_bridge/chassis_cli.py stop
python3 /root/ros2_bridge/chassis_cli.py status
# 或直接发布
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

## 停止
```bash
bash /root/ros2_bridge/stop_chassis_ros2.sh
```

## 说明
- 当前版本（09-21 ROS2 app 冻结版）的直线速度只取符号、档位固定 MID（110 RPM ≈1.15 m/s），
  线速度幅值不参与直线调速；转向幅值参与内侧轮比例。
- config.xml 已按实车修正：FL_ID=2 / RL_ID=1 / FR_ID=4 / RR_ID=3，Dir=-1/-1/-1/+1。
- 无遥控接收机时依赖 /root/ros2_bridge/sbus_emu.py 提供中性 SBUS 帧，仅为规避启动安全锁死。
