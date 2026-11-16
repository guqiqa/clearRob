# Device snapshot 2026-11-16

本目录是从设备 root@192.168.137.10 提取的脱敏快照，用于 GitHub 版本留档。

## 包含

- chassis_ros2_nav_v4/src：v4 底盘源码、config.xml 及历史配置备份
- chassis_ros2_nav_v4/scripts：构建/配置辅助脚本
- chassis_ros2_nav_v4/README_V4.md
- ros2_bridge：底盘测试、航向诊断、教学脚本（含 edu）
- MANIFEST.txt：设备、config/main.cpp/二进制/教学脚本的 SHA256

## 明确排除

- 实验记录（odometry.csv 等）
- 构建中间产物（build/install/log）
- 设备二进制（完整备份在 E 盘）
- netplan、WiFi PSK、systemd 运行配置等可能含敏感信息的系统文件
- nav_ws 运行时安装产物和数据集

## 完整备份

E:\小车备份（持续更新）\20261116_2024_device_snapshot

完整备份包含源码、脚本、配置、二进制、systemd/netplan 和快照 tar.gz，仅供本地保存，不要上传到公开仓库。
