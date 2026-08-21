# RTK NTRIP 接入：执行计划与执行情况

## Summary

开发板暂无 SIM 卡（后续会加装），RTK 模块（移远 LG29xP 家族，无北斗 → LG290P(03)/LG580P(03)）暂时无法用自带 4G 连 NTRIP。本方案用**机器人现有 WiFi 拉取 RTK 差分改正（RTCM3）并注入模块 UART**（`/dev/ttyS2`），实现 RTK 固定解，撑到 SIM 到货。

技术路线 = 移远文档「通过计算机连接」的等价实现（QGNSS 在 PC 连 caster → 注入模块 UART），只是把客户端搬到机器人上：`rtk_driver` 内嵌 NTRIP 客户端，单进程同时读 NMEA、写 RTCM。

## Key Changes

- `rtk_driver/rtk_node.py`：新增内嵌 NTRIP 客户端（`ntrip_enable` 门控，默认关）——连接 caster、周期发 GGA（VRS 必需）、接收 RTCM3 写入模块 UART；修复 `$GP`→`$Gx` talker bug（`$GNGGA/$GNRMC`）。
- `phase1_params.yaml`：`rtk_driver` 段新增 NTRIP 参数块（host/port/mount/user/pwd/gga_interval/fallback 坐标）。
- 技术文档：`需求文档_RTK驱动模块.md` §5、`需求文档_参数与信号总表.md` §3、`需求文档_V3.0_实际平台.md` §2.7 补 NTRIP 配置与 WGS84 说明。

## Test Plan

- 纯函数单测：NTRIP 请求构造 / GGA 构造与校验和 / 经纬度 ddmm.mmmm 往返（PC）。
- 室内链路验证（无需卫星）：节点日志见 `ICY 200 OK` + `RTCM bytes injected` 持续增长 → 证明 WiFi 拉差分 + 串口注入链路通。
- 户外验收：模块出单点定位 → 客户端转发实时 GGA → caster 生成虚拟基站 → 差分注入 → GGA `quality=4`（固定）→ `/fix` `status=4`、`cov[0]=0.01`、差分龄期 0~2s。

## Assumptions

- 模块 UART 双向可写（已实测 `/dev/ttyS2` O_RDWR 成功），RTCM 可注入。
- caster `AUTO` 挂载点差分格式（RTCM3 MSM4：1074/1084/1094/1114）与模块支持表完全兼容。
- 默认端口 **8002 = WGS84**（模块默认坐标系）；8003 = CGCS2000，仅后续投影入地图才需处理。
- 测试账号 `qxr0014590` 有效期至 **2026-09-10 21:11**。
- 凭据明文存于 `phase1_params.yaml`，与仓库现有设备参数同级。

---

## 执行情况

| # | 事项 | 状态 | 说明 |
|---|---|---|---|
| 1 | 串口链路 + 10Hz `$GNGGA/$GNRMC`（5星座，`$GN*` talker） | ✅ 已实测 | 460800 波特，GPS/GLONASS/Galileo/QZSS/IRNSS，无北斗 |
| 2 | 模块型号与 RTCM3 注入能力确认 | ✅ 已确认 | LG29xP 家族；Quectel RTK 应用指导表2（1074/1084/1094/1114 等可注入） |
| 3 | caster 账号/挂载点/差分流验证 | ✅ 已实测 | 三挂载点 `200 OK`；发 GGA 后流 RTCM3 MSM4 |
| 4 | `/dev/ttyS2` 双向可写（注入前提） | ✅ 已实测 | O_RDWR 写入成功且仍读到 NMEA |
| 5 | `rtk_driver` $GN talker bug 修复 + 解析验证 | ✅ 已完成 | 15 项纯函数断言全过 |
| 6 | NTRIP 客户端节点实现（rtk_node.py 内嵌） | ✅ 已完成 | `ntrip_gga_interval` 类型修正（YAML 整数 10 ↔ 代码 INTEGER），2026-08-13 真机启动无错 |
| 7 | `phase1_params.yaml` NTRIP 参数块 | ✅ 已写入 | 参数块在 `rtk_driver:` 段，默认 `ntrip_enable: false` 不影响现状 |
| 8 | 部署到真机（192.168.0.28） | ✅ 已完成 | scp 覆盖 `rtk_node.py` + `phase1_params.yaml`，`--symlink-install` 免重编译，启动即生效 |
| 9 | 室内链路验证（NTRIP→串口注入） | ✅ 已实测 | `ICY 200 OK`；RTCM 字节 8207→114836 持续增长；注入期间 /fix 仍 10Hz 正常解析（读写互不干扰） |
| 10 | 户外 RTK 固定解验收（quality=4, cov=0.01） | ⏳ 待办 | 需移机器人到开阔处；命令见下方「复现」 |

---

## 已验证事实（实测记录）

### 1. 模块与串口（192.168.0.28 / 割草机1.1）
- `/dev/ttyS2` @460800，10Hz 输出 `$GNGGA/$GNRMC`，另见 `$GNVTG/$GNGLL/$GNGSA` + GSV（GP/GL/GA/GI/GQ）。
- 室内 0 卫星：GGA `quality=0, sats=00, HDOP=99.99`，RMC 状态 `V` → 定位需户外。
- 模块型号判断：5星座无北斗 → LG290P(03)/LG580P(03) 类；最终以实物标签为准。

### 2. NTRIP caster（机器人 WiFi 实测）
- `203.107.45.154:8002`、`203.107.45.154:8003`、`60.205.8.49:8003` 均 TCP 可达。
- 账号 `qxr0014590/5c3dbd6`：`/AUTO`、`/RTCM32_GGB`、`/RTCM30_GG` 握手均 `ICY 200 OK`。
- `/AUTO` 发 GGA 后 15s 收 90 帧 RTCM3：**1074(GPS)+1084(GLONASS)+1094(Galileo)+1114(QZSS) MSM4** + 1005/1012/1033 基站信息 → 与模块支持表兼容。
- VRS 行为：**不发 GGA 无数据，发 GGA 立即有差分流** → 客户端必须周期发 GGA（用模块实时位置）。

### 3. rtk_driver 修复（本次会话）
- talker bug：`startswith('$GPGGA')` → `line[3:6]=='GGA'`（兼容 `$GN/$GP/$BD`），`$GPRMC` 同理。
- 新增 NTRIP 客户端：内嵌于 rtk_driver，单进程持有 `/dev/ttyS2`（读 NMEA + 写 RTCM），避免双进程 termios 竞争。
- 参数类型坑：YAML `ntrip_gga_interval: 10` 是 INTEGER，若代码声明 DOUBLE 会在启动瞬间抛 `InvalidParameterTypeException`。**代码按整数声明（与 YAML 一致）**。

### 4. 室内链路验证（2026-08-13，真机）
- 起节点命令（需与 params 文件 + ntrip_enable 同时给）：
  ```
  cd /root/cleaning_robot_ws && source install/setup.bash
  ros2 run rtk_driver rtk_node --ros-args \
    --params-file /root/cleaning_robot_ws/src/cleaning_robot_bringup/config/phase1_params.yaml \
    -p ntrip_enable:=true
  ```
- 日志序列：`reading from /dev/ttyS2 @ 460800` → `NTRIP: client 203.107.45.154:8002 mount=AUTO` → **`NTRIP: connected — ICY 200 OK`** → `NTRIP: {N} RTCM bytes injected` 每 ~8s 涨 ~8KB（8207→114836，共约 1.7 分钟）。
- 注入期间 `ros2 topic echo /fix` 仍 10Hz 输出（`status:0`，室内 0 卫星）→ **读 NMEA 与写 RTCM 同串口互不干扰**。

---

## 后续（SIM 加装后）

若机器人硬件含 Quectel EVB MCU + 4G（EG25-G），可改用模块自带 NTRIP 客户端（文档 §2.3.4.2）：
`ntripclient --type SelfBuild --host 203.107.45.154 --port 8002 --user qxr0014590 --pwd 5c3dbd6 --mnt AUTO` + `ntrip --mode rover`。
本节点可留作 WiFi/4G 双通道之一。
