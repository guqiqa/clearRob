# 需求文档 — RTK 驱动模块（rtk_driver）

> 来源：智能环卫机器人控制系统需求文档 V3.0（落地版）
> 文档代号：需求文档_RTK驱动模块 V1.0
> 更新：2026-07-24

---

## 1. 模块定位

`rtk_driver` 从 NTRIP 高精度 GNSS 模块读取 NMEA 0183 格式的定位数据，解析 GGA (位置) 和 RMC (速度/航向) 语句，发布 ROS2 `sensor_msgs/NavSatFix` 消息。

---

## 2. 硬件接口

| 参数 | 值 |
|------|------|
| 接收机 | NTRIP 高精度 GNSS |
| 设备路径 | `/dev/ttyS2` |
| 波特率 | 460800 bps |
| 协议 | NMEA 0183 标准 |
| 读取方式 | 行读取 (以 `\r\n` 分隔) |

### 2.1 NMEA 语句格式

**GGA (Global Positioning System Fix Data):**
```
$GPGGA,hhmmss.ss,ddmm.mmmm,N,dddmm.mmmm,E,quality,numSV,HDOP,alt,M,geoid,M,,*CS
```
关注字段: 时间 (field 1), 纬度 (2-3), 经度 (4-5), 定位质量 (6), 卫星数 (7)

**RMC (Recommended Minimum Navigation Information):**
```
$GPRMC,hhmmss.ss,A,ddmm.mmmm,N,dddmm.mmmm,E,speed_knots,course_deg,date,,*CS
```
关注字段: 有效标志 (2, A=有效), 速度 (7, 节), 航向 (8, 度)

---

## 3. ROS2 接口

### 3.1 Node 信息

| 项目 | 内容 |
|------|------|
| ROS2 Node | `rtk_driver` |
| 语言 | Python 3.10+ |
| 依赖 | `rclpy`, `sensor_msgs`, 可选 `pynmea2` |
| 备注 | 不需要 pynmea2 — NMEA 0183 格式简单, 手写解析器 ~30 行 |

### 3.2 发布

| Topic | 消息类型 | 频率 | 说明 |
|-------|---------|------|------|
| `/fix` | `sensor_msgs/msg/NavSatFix` | 与 GNSS 更新频率同步 (通常 1-10 Hz) | 经纬度 + 高度 + 协方差 |

### 3.3 发布 (辅助)

| Topic | 消息类型 | 频率 | 说明 |
|-------|---------|------|------|
| `/gps/status` | `sensor_msgs/msg/NavSatStatus` | 1 Hz | 定位质量 + 卫星数 |

---

## 4. 功能需求

### 4.1 NMEA 解析

```python
def parse_gga(sentence: str):
    """解析 $GPGGA → (lat, lon, alt, quality, num_sv)"""
    fields = sentence.split(',')
    # lat/lon 格式: ddmm.mmmm → 十进制度
    lat = float(fields[2][:2]) + float(fields[2][2:]) / 60.0
    if fields[3] == 'S': lat = -lat
    lon = float(fields[4][:3]) + float(fields[4][3:]) / 60.0
    if fields[5] == 'W': lon = -lon
    alt = float(fields[9]) if fields[9] else 0.0
    quality = int(fields[6])
    num_sv = int(fields[7])
    return lat, lon, alt, quality, num_sv
```

### 4.2 NavSatFix 填充

```python
fix = NavSatFix()
fix.header.stamp = node.get_clock().now().to_msg()
fix.header.frame_id = "gps_link"
fix.latitude = lat
fix.longitude = lon
fix.altitude = alt

# RTK float/fixed 模式的协方差不同
if quality >= 4:  # RTK fixed
    fix.position_covariance[0] = 0.01   # 1cm² 级别
    fix.position_covariance[4] = 0.01
    fix.position_covariance[8] = 0.04   # 高度稍差
elif quality >= 2:  # DGPS
    fix.position_covariance[0] = 1.0    # 米级
else:  # 单点
    fix.position_covariance[0] = 25.0   # ~5m 级别

fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
fix.status.status = quality  # 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
fix.status.service = NavSatStatus.SERVICE_GPS
```

### 4.3 模拟模式

当 `simulate:=true` 时:
- 不打开 `/dev/ttyS2`
- 发布固定坐标 (可配置的 `sim_lat`, `sim_lon`)
- 日志 `[SIM] rtk_driver running in simulated mode`

---

## 5. 配置参数

```yaml
rtk_driver:
  ros__parameters:
    device: "/dev/ttyS2"
    baudrate: 460800
    frame_id: "gps_link"
    
    # 模拟
    simulate: false
    sim_lat: 31.2304     # 上海示例坐标
    sim_lon: 121.4737
```

---

## 6. 与其他模块的交互

```
rtk_driver ──/fix──→ EKF 融合节点 (robot_localization) 或 二期 path_planner
             ──/gps/status──→ 监控
```

---

## 7. 修改说明

| 日期 | 修改内容 |
|------|----------|
| 2026-07-24 | **V1.0 初始创建**：NMEA 0183 GGA/RMC 解析，NavSatFix 发布，RTK 协方差区分，模拟模式 |
