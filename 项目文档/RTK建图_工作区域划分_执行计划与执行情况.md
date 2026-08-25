# RTK 建图 + 工作区域划分：执行计划与执行情况

## Summary

平台**无 LiDAR**（毫米波雷达+视觉为避障传感器，不产生地图），要落地"建图 + 工作区域划分"最自然的路线是 **RTK 高精度定位**。本计划用机器人已有的 RTK 链路（`rtk_driver` 已发 `/fix`，NTRIP 差分注入已真机验证）实现：

> 沿边界手推一圈 → 采集 RTK 固定解轨迹 → 自动闭环成工作区多边形 → PC 侧划分子区域（可叠禁行区）→ 存 JSON 地图文件 → 供后续 `path_planner` 覆盖清扫使用。

### 可行性结论

| 能力 | 结论 | 说明 |
|---|---|---|
| 工作区**边界建图**（沿边手推圈定） | ✅ 可行 | RTK 固定解 ~1cm 级绝对坐标，天然适合边界圈定；现有 `/fix` + NTRIP 链路已通 |
| **子区域划分**（多边形切分/禁行区） | ✅ 可行 | PC 侧 shapely/matplotlib 工具，点选分割或 grid 自动切分 + 几何校验 |
| **环境障碍建图** | ❌ 不可行（本方案不承担） | RTK 只能定位机器人自身位置，**不能"看"环境**；运行时避障仍靠毫米波雷达+视觉（二期） |
| 覆盖路径/自主清扫 | 后续轮次 | Round 2 接 `path_planner`，本轮不含 |

**现状缺口**（探索确认，均需新建）：① `/fix` 目前无订阅者；② 全仓库无任何经纬度→平面投影代码；③ 无 `gps_link→base_link` TF（RTK 天线不在轮心，杠杆臂真实存在）；④ `path_planner` 区域只有硬编码矩形 `zone` 参数；⑤ 需求文档把"建图/区域管理"放**三期**（P3-04），本方案将其**提前**。

## Key Changes（计划，未实施）

### 1. 新包 `src/area_mapper/`（纯 Python 核心 + 薄 ROS 包装，仿 `path_planner/zigzag.py` 模式）

| 文件 | 内容 |
|---|---|
| `area_mapper/coords.py` | **横向墨卡托(Gauss-Krüger)投影器**：参数化 `Datum`（WGS84/CGCS2000）、`central_meridian_deg`、`scale_factor`（1.0=GK / 0.9996=UTM）、`origin`（原点局部米制坐标）。`forward`/`inverse` + ECEF/方位角辅助。纯 ~110 行，无第三方依赖（真机 aarch64 无网，不能装 pyproj） |
| `area_mapper/geometry.py` | 纯 Python 多边形工具（真机无 shapely）：`shoelace_area`/`perimeter`/`douglas_peucker`（保闭环）/`close_ring`/`snap_and_close` |
| `area_mapper/tracker.py` | **采集状态机（无 ROS，可单测）**：`feed_fix(lat,lon,status,cov0,yaw)` 质量门控+距离降采样（兼作低速过滤）；`start_boundary/start_no_go/stop_record`；`check_loop_closure` 自动闭环；杠杆臂校正（`/odom` yaw） |
| `area_mapper/map_io.py` | JSON 读写 + `construct_task_area_json`（对齐 TaskArea.msg 字段，Round 2 粘合线） |
| `area_mapper/collector_node.py` | ROS 节点 `area_mapper`（薄包装）：订 `/fix`、`/odom`、`/area_mapper/cmd`，发 `/area_mapper/status`，启动广播 `gps_link→base_link` 静态 TF。被动运行，无命令不动 |
| `area_mapper/tools/collect_cli.py` | 现场键盘助手：`b` 起/停边界、`n` 禁行区、`s` 保存 |
| `area_mapper/tools/sim_fix_pub.py` | 模拟 `/fix`（矩形 10Hz + 噪声 + status=4 + 假 `/odom`）→ 室内无卫星全链路测采集器 |
| `area_mapper/tools/area_editor.py` | **PC 侧**子区域划分：加载 raw → matplotlib 渲染 → 点选分割线 / `--grid` 自动切分 → 校验（合法性/面积/非重叠/面积和）→ 存终图 |
| `test/test_{coords,tracker,geometry,map_io}.py` | PC pytest（投影往返 vs pyproj、闭环/面积/门控、保闭环、JSON 往返） |

**触发交互**：`std_msgs/String` 命令话题 `/area_mapper/cmd`（START/STOP_BOUNDARY、START/STOP_NOGO、SAVE、DISCARD、RESET）+ CLI 单键流。**不复用 SBUS 按键**（SWA/SWB 已被 master_bridge 占用），与 master_bridge/remote_driver 解耦，不碰 `/chassis/intent`。

### 2. 坐标与坐标系

- 地图帧名 `map`；投影原点 = `START_BOUNDARY` 首个通过固定解（自动）或显式 `origin`。`local_xy` = 距原点米制（E=+x, N=+y）。
- **`datum` 必须与 `ntrip_port` 配套**：8002=WGS84 / 8003=CGCS2000。默认 8002 + WGS84。
- **中央子午线默认 120.0**（上海当地 3° 带标准中央子午线，CGCS2000 国家格网对齐；小地块也可设本区经度做本地 TM，误差等同）。
- **gps_link→base_link 静态 TF 本期就加**：杠杆臂（~10cm 级）是真实精度问题。采集时 `apply_lever_arm: true` 用 `/odom` yaw 软件校正（IMU 无 orientation，用 `/odom`）；现场 yaw 不可信则 `apply_lever_arm: false` 推天线贴线（整图平移一杠杆臂，Round 1 可接受）。

### 3. 地图文件 JSON（`maps/<map_id>.json`，schema 对齐 TaskArea.msg）

```json
{
  "schema_version": 1, "map_id": "area_20260813T1200", "frame": "map",
  "crs": { "datum": "WGS84", "projection": "transverse_mercator",
           "central_meridian_deg": 120.0, "scale_factor": 1.0,
           "origin": {"lat": 31.2304, "lon": 121.4737}, "origin_xy": {"x": 0.0, "y": 0.0} },
  "boundary": { "wgs84": [[31.2304,121.4737], "…"], "local_xy": [[0.0,0.0], "…"],
                "area_m2": 200.0, "perimeter_m": 60.0, "vertices": 4, "closed": true },
  "sub_areas": [ { "area_id": "sub_0", "wgs84": ["…"], "local_xy": ["…"],
                   "area_m2": 100.0, "cleaning_passes": 1, "overlap_ratio": 0.1 } ],
  "no_go": [ { "area_id": "ng_0", "wgs84": ["…"], "local_xy": ["…"], "area_m2": 4.0 } ],
  "meta": { "collector_version": "0.1.0", "ntrip_enabled": true,
            "quality_gate": 4, "mean_hdop": null, "notes": "" }
}
```

`sub_areas[i].local_xy` 即未来 `path_planner` 弓字形的 `[(x,y),…]` 角点输入（与现有 `zigzag.py` 输入形状一致）。

### 4. 现有文件改动（最小）

- `cleaning_robot_bringup/config/phase1_params.yaml`：新增 `area_mapper:` 参数块（datum/central_meridian/scale/origin/quality_gate/max_covariance/min_vertex_spacing/loop_close_radius/simplify_tolerance/min_boundary_points/apply_lever_arm/lever_arm_x/y + 话题与输出路径）。现有 `rtk_driver` NTRIP 块不动。
- `cleaning_robot_bringup/launch/phase1_chassis.launch.py`：加 `area_mapper` 节点（TimerAction 链，无条件启动——被动节点，命令才动）。
- `cleaning_robot_common/config.py`：加 `TOPIC_AREA_CMD`、`TOPIC_AREA_STATUS`、`FRAME_MAP`。
- `cleaning_robot_interfaces/msg/TaskArea.msg`：**本轮不建**（无消费者，JSON 已含同字段布局；Round 2 接 path_planner 时再建）。

## Test Plan（计划）

1. **PC 单测**：`pytest src/area_mapper/test/` —— 投影往返 vs pyproj（双 datum × 双 scale）、tracker 矩形走线闭环+面积±2%+门控拒收、geometry 保闭环、map_io 往返。
2. **室内模拟（无卫星）**：`sim_fix_pub` 喂合成矩形 → START→记录→自动闭环→SAVE 出合法地图；**负测试**：真机室内 `/fix`(status=0) 采集器保持 idle。
3. **户外验收（测试账号 2026-09-10 过期前）**：开 NTRIP，`/fix` 见 `status:4, cov[0]:0.01`；手推 10×20m 矩形 → SAVE → scp raw 到 PC → `area_editor.py` 校验形状+面积±5%、`--grid 2x2` 子区合法非重叠且面积和=总面积；`apply_lever_arm:false` 重采对比平移量≈杠杆臂。
4. **部署**：scp 新包+改后 yaml 到 `root@192.168.0.28`，`colcon build --symlink-install`，重启。

## Assumptions

- `status.status==4` 是可靠固定解判别（rtk_driver 把 GGA quality 直接映射；4=固定/5=浮点 cov 同为 0.01，cov 无法区分）。
- 手推速度慢，距离降采样（5cm）即足够低速过滤；`/fix` 未暴露速度。
- 现场笔记本 WiFi 可达（192.168.0.x，同 ROS_DOMAIN）跑 CLI。
- 真机无 shapely/matplotlib，多边形编辑全在 PC。

---

## 执行情况

> 本次为**计划与可行性评估**交付，实施未启动。表格随实施推进更新。

| # | 事项 | 状态 | 说明 |
|---|---|---|---|
| 0 | 可行性评估 + 方案设计 | ✅ 已完成 | 本计划落盘；结论：边界/区域划分可行，障碍建图不可行 |
| 1 | `coords.py` TM 投影器 + vs pyproj 单测 | ⏳ 待办 | WGS84+CGCS2000 × GK/UTM |
| 2 | `tracker.py` 采集状态机（门控/降采样/闭环/杠杆臂） | ⏳ 待办 | |
| 3 | `collector_node.py` + gps_link→base_link 静态 TF | ⏳ 待办 | |
| 4 | `area_editor.py` PC 分割工具（点选+grid+校验） | ⏳ 待办 | |
| 5 | yaml/launch/config.py 改动 | ⏳ 待办 | |
| 6 | 室内模拟（sim_fix_pub 全链路） | ⏳ 待办 | |
| 7 | 户外矩形验收 + 子区划分 | ⏳ 待办 | 2026-09-10 前 |
| 8 | 本文件执行情况表随推进更新 | ⏳ 待办 | |

### 近期实测（2026-08-25）

- 已在 `root@192.168.8.143` 上完成只读检查，板端可达。
- `/dev/ttyS2` 当前能连续输出 `GNGGA` / `GNRMC` 等 NMEA 报文，说明外接天线已收到卫星信号。
- 现场样本里可见 `quality=2`、`sats=31`、`HDOP≈0.66`，当前属于**有定位但尚未固定解**，还不是 `RTK fixed`。
- 当时未看到 `rtk_node` 进程在运行，因此 ` /fix` 话题还没有被这次现场检查直接验证。

## 关键决策（用户已确认）

1. **边界采集方式** = 沿边界手推一圈（沿边手推）。
2. **坐标系** = WGS84 与 CGCS2000 **都支持**，参数化（datum + 中央子午线 + scale），默认 WGS84。
3. **本期范围** = 边界 + 子区域划分（不含覆盖路径生成 / path_planner 集成 / EKF 融合）。

## Risks（如实）

1. **RTK 固定解户外可用性是最大弱环**：无 4G SIM 前差分靠 WiFi NTRIP，WiFi 覆盖限制作业半径；测试账号 **2026-09-10 过期**。树荫/屋檐遮挡会掉浮点/中断，质量门控丢样本 → 多边形可能有缺口需重走。建议尽快装 SIM 或用手机热点。
2. **RTK 建图≠障碍建图**：只有边界/禁行区（人工环绕采集），运行时避障仍是雷达+视觉（二期）。
3. **HDOP/差分龄期不在 NavSatFix**（rtk_driver 只发 lat/lon/alt/quality/cov）。本期仅 status+cov 门控；若浮点抖动明显需给 rtk_driver 加 `/gps/status`（记录为延期改动）。
4. **杠杆臂**：航向相关偏移（10cm 级），靠实测+`/odom` yaw 校正或 `apply_lever_arm:false` 推天线贴线缓解；odom yaw 有缓慢漂移，Round 1 在 10cm 级可接受。
5. **需求提前**：建图/区域管理在需求文档中是三期（P3-04）。本方案提前交付 RTK 多边形形式的"建图"，完整增量建图（雷达/LiDAR）仍在三期。
6. **`map` vs `/odom` 帧差**：本期地图存绝对 `map` 帧；现有 path_planner 用开机原点 `/odom` 死推算。Round 2 接覆盖时必须调和（map→odom 变换或任务开始 odom 复位）。

## 后续（Round 2，本期不做）

- `TaskArea.msg` 落地 + `path_planner` 接收 `sub_areas[i].local_xy` 生成弓字形覆盖路径（`zigzag.py` 输入形状已对齐）。
- `map→odom` 帧调和（或任务开始 odom 复位）。
- 可选：`rtk_driver` 增发 `/gps/status`（HDOP+卫星数）增强门控。
