# Phase 1 底盘控制系统 — 技术实现文档

> 版本: V1.0 | 日期: 2026-07-28 | 分支: `v8-all-datasets`
> 目标平台: 移远 QSM602HA-GL (D-Robotics X5, ARM64, Ubuntu 22.04)

---

## 一、系统架构

### 1.1 信号链路

```
C7mini遥控器 → SBUS 100Kbps(负逻辑)
    ↓
MCU(硬件反相) → UART(正逻辑) → port_bridge → /dev/ttyS1
    ↓
remote_driver(SBUS解码) → /joy (sensor_msgs/Joy)
    ↓
master_bridge(状态机+摇杆→差速) → /cmd_vel (geometry_msgs/Twist)
    ↓
chassis_driver(差速→CAN电流) → can0 (SocketCAN 500Kbps)
    ↓
001左后/002左前/003右后/004右前 (OID FOC电机)
```

### 1.2 节点拓扑

```
cleaning_robot_ws/
├── src/
│   ├── cleaning_robot_common/          # 共享模块 (纯Python库)
│   │   └── config.py                   # CAN ID/电机参数/Topic名/校准系数
│   │   └── kinematics.py               # 差速模型/里程计 (无状态纯函数)
│   │   └── can_protocol.py             # CAN帧构造/解析
│   ├── cleaning_robot_drivers/
│   │   ├── chassis_driver/             # CAN直驱5台FOC电机
│   │   ├── imu_driver/                 # QMI8658 6轴IMU (200Hz)
│   │   ├── rtk_driver/                 # NMEA 0183 GNSS
│   │   └── remote_driver/              # SBUS 16通道解码
│   ├── master_bridge/                  # STANDBY/MANUAL状态机
│   └── cleaning_robot_bringup/         # launch + YAML配置
```

---

## 二、关键技术实现

### 2.1 SBUS 信号解码

**挑战:** C7mini 遥控器使用标准 SBUS 协议，但信号经 MCU 硬件反相后通过 `port_bridge` 进程输出。`port_bridge` 是一个 Modem AT 命令桥接程序，需要通过特定 ioctl 序列切换到 SBUS 透传模式。

**实现方案 (最终采用):**

逆向 `RemoteControl_Test` 二进制（ARM64 ELF，含调试符号）：
- `SBUSReader::configureSerial()` — termios2 ioctl 序列，触发 port_bridge 切换
- `SBUSReader::sbus_parse()` — 16通道×11bit 位解包算法
- `SBUSReader::readFrame()` — 逐字节同步 (0x0F 帧头) + 25字节定长读取

**源码级复刻的关键代码 (remote_driver):**

```python
# port_bridge init — 逆向 RemoteControl_Test::configureSerial 反汇编
def init_port_bridge(device):
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
    # TCGETS2 → 修改 c_cflag → TCSETS2 → 设波特率 100000
    # 关键: buf[17+5]=0 (VMIN), buf[17+6]=0 (VTIME)
    # cflag &= ~0x200  (清除 CREAD 位)

# Vendor sbus_parse — 来自 SBUSReader.cpp 源码
def sbus_parse(buf):
    ch = [0]*16
    for i in range(16):
        byte_idx = 1 + (i*11)//8    # 整数除法
        bit_off = (i*11)%8
        raw = buf[byte_idx] | (buf[byte_idx+1]<<8) | (buf[byte_idx+2]<<16)
        ch[i] = (raw >> bit_off) & 0x07FF
    flags = buf[23]
    failsafe = (flags>>3)&1; frame_lost = (flags>>2)&1
    return ch, failsafe, frame_lost
```

**失败路径总结:**
1. ❌ 直接开 `/dev/ttyS1` 100000 8E2 标准串口 — port_bridge 未切换模式，输出为 AT 轮询数据
2. ❌ 批量 `buf.find(0x0F)` 搜索 — 0x0F 可出现在通道数据中，导致假同步帧
3. ❌ 固定 25 字节 stride — 帧间间隔不固定，偏移累积丢失同步
4. ✅ **逐字节读取 + 帧头同步** — 原生 SBUSReader 逻辑，稳定解码

### 2.2 CAN 总线电机控制

**硬件协议 (OID FOC 无刷电机):**

| CAN ID | 电机 | 安装方向 |
|:---:|------|:---:|
| 001 | 左后轮 | 反装 |
| 002 | 左前轮 | 反装 |
| 003 | 右后轮 | 正装 |
| 004 | 右前轮 | 正装 |
| 005 | 刀盘 | (二期) |

**CAN 帧格式:**

```
电流控制帧: [0x01, current_hi, current_lo]  (int16, 10mA/unit)
心跳帧:     [0x00]
查询帧:     [0x0F, 0x01] → 响应: [0x0F, 0x01, erpm_int32]
```

**电机参数:**

| 参数 | 值 |
|------|------|
| 极对数 | 10 (20极) |
| 减速比 | 5.2:1 |
| 轮半径 | 0.08m |
| 轮距 | 0.53m |
| 霍尔电角度 | 120° |
| 相电阻 | 0.8Ω |
| 相电感 | 1.6mH |

**里程计公式:**

```python
rpm_motor = erpm / 10          # 电转速 → 电机机械转速
rpm_wheel = rpm_motor / 5.2    # → 轮转速 (减速比)
rad_s = rpm_wheel * 2π / 60   # → rad/s
v = rad_s * 0.08               # → m/s
```

**左侧电机反转:** `left_motor_invert: True` — 因左侧电机反装，电流值取反

### 2.3 SWA 状态机

**问题历程:**

| 版本 | 逻辑 | 问题 |
|------|------|------|
| v1 | 边沿触发 (0→1 toggle) | 启动时 SWA 已在上端 → 不触发 |
| v2 | `_first_joy` 自动进 MANUAL | 上/下都触发，SWA 逻辑混乱 |
| v3 | 电平逻辑 (btn=1→MANUAL) | 按钮抖动，频繁切换 |
| **v4** | **电平 + 0.5s 去抖** | ✅ 稳定 |

**最终实现:**

```python
# master_bridge: SWA debounce — 0.5s stable required
if btn_start != self._swa_last:
    self._swa_last = btn_start; self._swa_changed = now
elif now - self._swa_changed > 0.5 and btn_start != self._swa_btn:
    self._swa_btn = btn_start
    self._state = STATE_MANUAL if btn_start else STATE_STANDBY
```

**SWA 三档映射 (C7mini):**

| 位置 | CH5值 | buttons[0] | 状态 |
|------|:---:|:---:|------|
| 上端 (UP) | 200 | 1 | MANUAL |
| 中位 (MID) | 1000 | 0 | STANDBY |
| 下端 (DOWN) | 1800 | 0 | STANDBY |

### 2.4 通道映射 (C7mini → /joy)

| 遥控器硬件 | SBUS CH | Joy 字段 | 功能 | 方向 |
|-----------|:---:|------|------|------|
| 右杆 左右 | CH1 | axes[0] | 转向 | steering_invert=True |
| 右杆 上下 | CH2 | — | (未用) | — |
| 左杆 前后 | CH3 | axes[1] | 油门 | forward=low→正速 |
| 左杆 左右 | CH4 | — | (未用) | — |
| SWA (3档) | CH5 | buttons[0] | MANUAL/STANDBY | <700=ON |
| SWB (3档) | CH6 | buttons[2] | ESTOP急停 | <700=ON |
| VR1 | CH7 | — | (未用) | — |
| VR2 | CH8 | — | (未用) | — |
| SWC (硬件) | — | — | 限位(非SBUS通道) | — |
| SWD (硬件回弹) | — | — | RF切换(非SBUS通道) | — |

**通道发现过程:**
- 用 GDB 反汇编 `RemoteControl_Test` 获取 `sbus_frame_t` 结构体和 `sbus_parse` 算法
- 编写逐帧dump工具 `verify_sbus.py`，逐一操作摇杆/开关确定各通道对应关系
- 发现 C7mini 行程为 1000-2000（非标准 SBUS 172-1811），但 port_bridge 将其重映射为标准范围

---

## 三、部署架构

### 3.1 systemd 服务

```ini
# /etc/systemd/system/phase1.service
[Unit]
Description=Phase 1 Chassis Control
[Service]
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && \
  source /root/cleaning_robot_ws/install/setup.bash && \
  exec ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false"
Restart=no
[Install]
WantedBy=multi-user.target
```

### 3.2 与原厂程序共存

```
原厂: LawnMower.service (enable, 开机自启)
我们: phase1.service    (disable, 手动启)

切换命令:
  systemctl stop LawnMower && systemctl start phase1    # 启用我们的
  systemctl stop phase1 && systemctl start LawnMower    # 恢复原厂
```

两套系统互斥使用 can0，通过 systemd 切换保证不冲突。

### 3.3 编译注意事项

- ARM64 4GB 环境: `colcon build --executor sequential --symlink-install`
- 修改 config.py 后需重编译 `cleaning_robot_common` + 依赖它的包
- Python 包全量编译约 60s

---

## 四、调试工具集

| 工具 | 路径 | 用途 |
|------|------|------|
| `verify_sbus.py` | PC: `cleaning_robot_ws/src/` | 逐字节 SBUS 帧dump+通道变化检测 |
| `swa_monitor.py` | PC: `cleaning_robot_ws/src/` | SWA 开关三档值精确测量 |
| `go.py` | PC: `cleaning_robot_ws/src/` | 一键启动底盘+键盘控制(W/S/A/D/Q) |
| `deploy_auto.sh` | PC: `cleaning_robot_ws/` | 设备端自动编译+启动脚本 |
| `calib.py` | PC: `cleaning_robot_ws/src/` | 全部16通道范围采集 |

---

## 五、已知问题与改进方向

| 问题 | 状态 | 解决方案 |
|------|:---:|------|
| CAN 缓冲区溢出 (`No buffer space`) | ⚠️ | 心跳 40ms 已缓解，未根除 |
| SBUS 帧率波动 (50-715f/10s) | ⚠️ | 逐字节解码效率，待优化 |
| 无速度闭环 | TODO | 二期: PID速度控制 ← 已提出 |
| 无自动导航 | TODO | 二期: path_planner + 毫米波雷达 |
| 无清扫控制 | TODO | 二期: CAN ID=005 刀盘电机 |

---

## 六、修改记录

| 日期 | 内容 |
|------|------|
| 2026-07-24 | Phase 1 初始实现：5节点 ROS2 架构，CAN 驱动，SBUS 原型解码 |
| 2026-07-25 | 电机参数修正(10对极/5.2:1)，模块化拆分为 cleaning_robot_common |
| 2026-07-26 | SBUS 解码调试：反汇编 RemoteControl_Test，逐字节同步算法，通道映射确认 |
| 2026-07-26 | SWA 状态机多次迭代：边沿→电平→去抖，最终稳定 |
| 2026-07-28 | CAN 心跳调整，转向反转，全链路键盘/遥控器双验证通过 |
