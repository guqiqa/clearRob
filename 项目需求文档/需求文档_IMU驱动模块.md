# 需求文档 — IMU 驱动模块（imu_driver）

> 来源：智能环卫机器人控制系统需求文档 V3.0（落地版）
> 文档代号：需求文档_IMU驱动模块 V1.0
> 更新：2026-07-24

---

## 1. 模块定位

`imu_driver` 从 QMI8658 6 轴 IMU 传感器读取原始加速度和角速度数据，应用手册提供的校准系数，发布 ROS2 `sensor_msgs/Imu` 消息。

---

## 2. 硬件接口

### 2.1 设备

| 参数 | 值 |
|------|------|
| 型号 | QMI8658 (6 轴: 3 加速度 + 3 陀螺仪) |
| 设备路径 | `/dev/qmi8658_imu` |
| 波特率 | 460800 bps |
| 读取方式 | 阻塞读取 24 字节 struct |
| 数据格式 | 6 × int32 (little-endian): ax, ay, az, gx, gy, gz |

### 2.2 数据结构 (C struct, 手册提供)

```c
struct qmi8658_frame {
    int32_t ax;   // X 轴加速度原始值, /1000000.0 = m/s²
    int32_t ay;   // Y 轴加速度原始值
    int32_t az;   // Z 轴加速度原始值
    int32_t gx;   // X 轴角速度原始值, /1000000.0 = rad/s
    int32_t gy;   // Y 轴角速度原始值
    int32_t gz;   // Z 轴角速度原始值
};
// sizeof = 24 bytes
```

### 2.3 校准系数 (手册提供)

```python
ax = ax_raw / 1e6 + 0.320
ay = ay_raw / 1e6 + 0.027
az = (az_raw / 1e6 + 0.802) * 0.991
gx = gx_raw / 1e6 + 0.0265
gy = gy_raw / 1e6 - 0.0135
gz = gz_raw / 1e6 + 0.0132
```

---

## 3. ROS2 接口

### 3.1 Node 信息

| 项目 | 内容 |
|------|------|
| ROS2 Node | `imu_driver` |
| 语言 | Python 3.10+ |
| 依赖 | `rclpy`, `sensor_msgs` |

### 3.2 发布

| Topic | 消息类型 | 频率 | 说明 |
|-------|---------|------|------|
| `/imu/data` | `sensor_msgs/msg/Imu` | 200 Hz (硬编码, 与 IMU 数据率匹配) | 校准后的 6 轴数据 |

### 3.3 Service

| Service | 类型 | 说明 |
|---------|------|------|
| `imu/calibrate` | `std_srvs/srv/Trigger` | 触发零偏校准 (静止状态下采集 100 样本取均值) |

---

## 4. 功能需求

### 4.1 数据读取循环

```python
# 打开设备
fd = open("/dev/qmi8658_imu", O_RDONLY)

# 阻塞读取 (线程中运行, 不阻塞 ROS2 spin)
while running:
    data = os.read(fd, 24)  # 阻塞, 等待 IMU 产生新数据
    ax, ay, az, gx, gy, gz = struct.unpack('<iiiiii', data)
    # 应用校准 → 发布 Imu
```

### 4.2 Imu 消息填充

```python
imu_msg = Imu()
imu_msg.header.stamp = node.get_clock().now().to_msg()
imu_msg.header.frame_id = "imu_link"

# 加速度 (m/s²), 校准后
imu_msg.linear_acceleration.x = ax_calibrated
imu_msg.linear_acceleration.y = ay_calibrated
imu_msg.linear_acceleration.z = az_calibrated

# 角速度 (rad/s), 校准后
imu_msg.angular_velocity.x = gx_calibrated
imu_msg.angular_velocity.y = gy_calibrated
imu_msg.angular_velocity.z = gz_calibrated

# 协方差: 待标定, 暂设经验值
imu_msg.linear_acceleration_covariance[0] = 0.01   # ax 方差
imu_msg.linear_acceleration_covariance[4] = 0.01   # ay 方差
imu_msg.linear_acceleration_covariance[8] = 0.01   # az 方差
imu_msg.angular_velocity_covariance[0] = 0.001     # gx 方差
imu_msg.angular_velocity_covariance[4] = 0.001     # gy 方差
imu_msg.angular_velocity_covariance[8] = 0.001     # gz 方差

# 无姿态估计 (由 EKF 节点融合时计算)
imu_msg.orientation_covariance[0] = -1  # 无效
```

### 4.3 模拟模式

当 `simulate:=true` 时:
- 不打开 `/dev/qmi8658_imu`
- 发布零加速度 + 零角速度 (vectored: 重力加速度 [0, 0, -9.81])
- 日志 `[SIM] imu_driver running in simulated mode`

---

## 5. 配置参数

```yaml
imu_driver:
  ros__parameters:
    device: "/dev/qmi8658_imu"
    frame_id: "imu_link"
    publish_rate: 200             # 发布频率 (Hz)
    
    # 校准系数 (可由 calibrate service 更新)
    accel_bias_x: 0.320
    accel_bias_y: 0.027
    accel_bias_z: 0.802
    accel_scale_z: 0.991
    gyro_bias_x: 0.0265
    gyro_bias_y: -0.0135
    gyro_bias_z: 0.0132

    # 模拟
    simulate: false
```

---

## 6. 修改说明

| 日期 | 修改内容 |
|------|----------|
| 2026-07-24 | **V1.0 初始创建**：QMI8658 struct 解析 (24bytes, 6×int32)，手册校准系数，模拟模式 |
