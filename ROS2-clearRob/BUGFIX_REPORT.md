# Bug 修复报告 — 智能环卫机器人控制系统

> 日期：2026-06-17  
> 分支：初始版本修复  
> 状态：✅ 全部修复，9/9 节点仿真启动成功

---

## 概述

首次仿真测试中发现 **14 个 Bug**（4 个直接报错 + 10 个连锁 Bug），涉及 6 个 ROS2 节点。问题根因集中于三类：

| 类别 | 数量 | 说明 |
|------|------|------|
| ROS2 Humble 兼容性 | 2 | LifecycleState API 差异、topic 命名规则 |
| 消息字段名不符 | 10 | 代码使用的字段与 `.msg`/`.srv` 定义实际字段不一致 |
| Python 语法/命名错误 | 2 | 属性命名不一致、三元表达式逻辑错误 |

---

## 逐节点修复详情

### 1. `fusion_engine`（融合引擎）

| # | 文件 | 行 | 错误信息 | 原因 | 修复 |
|---|------|----|----------|------|------|
| 1.1 | [fusion_node.py](src/fusion_engine/fusion_engine/fusion_node.py#L140) | 140 | `InvalidTopicNameException: topic name token must not start with a number: 'fusion/3d_target'` | ROS2 不允许 topic 名以数字开头 | `fusion/3d_target` → `fusion/target_3d` |

**联动修复**：`master_node.py` L266 订阅同一 topic，同步修改。

---

### 2. `cleaning_actuator`（清扫执行器）

| # | 文件 | 行 | 错误信息 | 原因 | 修复 |
|---|------|----|----------|------|------|
| 2.1 | [cleaning_node.py](src/cleaning_actuator/cleaning_actuator/cleaning_node.py#L840) | 840 | `AttributeError: type object 'LifecycleState' has no attribute 'PRIMARY_STATE_UNCONFIGURED'` | ROS2 Humble 中 `LifecycleState` 是 NamedTuple，`PRIMARY_STATE_*` 常量在 `lifecycle_msgs.msg.State` 中 | 引入 `from lifecycle_msgs.msg import State as LifecycleStateMsg`，手动构造 State 对象 |
| 2.2 | 同上 | 648 | `AttributeError: 'tuple' object has no attribute 'id'`（连锁） | Humble 中 `_state_machine.current_state` 返回 tuple `(state_id, label)` 而非 State 对象 | `state.id` → `state[0]` |

**具体 diff**:

```python
# main() — 手动触发生命周期转换
# 修复前:
node.on_configure(LifecycleState.PRIMARY_STATE_UNCONFIGURED)

# 修复后:
from lifecycle_msgs.msg import State as LifecycleStateMsg
node.on_configure(LifecycleStateMsg(
    id=LifecycleStateMsg.PRIMARY_STATE_UNCONFIGURED, label='unconfigured'))

# _publish_heartbeat() — 获取当前状态
# 修复前:
state = self._state_machine.current_state
if state == LifecycleState.PRIMARY_STATE_ACTIVE: ...

# 修复后:
state = self._state_machine.current_state
if state[0] == LifecycleStateMsg.PRIMARY_STATE_ACTIVE: ...
```

---

### 3. `master_controller`（主控制器）— 共 6 处

| # | 文件 | 行 | 错误信息 | 原因 | 修复 |
|---|------|----|----------|------|------|
| 3.1 | [master_node.py](src/master_controller/master_controller/master_node.py#L51) | 37-51 | `ImportError: cannot import name 'GetStatus' from 'cleaning_robot_interfaces.msg'` | `GetStatus` 是 `.srv` 不是 `.msg` | 从 `cleaning_robot_interfaces.srv` 导入 |
| 3.2 | 同上 | 396 | `PlannerState` 字段不匹配（运行时） | 设置了不存在的 `.battery`/`.dust_full`/`.localization_confidence`；`.state` 应为 uint8 而非 string | 使用 `PlannerState.msg` 实际字段：`.state`(int) + `.state_text`(string) |
| 3.3 | 同上 | 271/308 | `RCLError: create_publisher() called for existing topic with incompatible type` | 同一 topic 上订阅用 `String`，发布用 `Heartbeat`，类型冲突 | 订阅也改为 `Heartbeat` 类型 |
| 3.4 | 同上 | 844-847 | `AttributeError: 'Heartbeat' object has no attribute 'module_name'` | Heartbeat.msg 字段是 `node_name` | `.module_name` → `.node_name`；`.state`/`.timestamp` → `.lifecycle_state`/`header.stamp` |
| 3.5 | 同上 | 927-930 | `Alert` 字段不匹配（连锁） | 使用了不存在的 `.code`/`.description`/`.timestamp` | 改用实际字段 `.alert_code`/`.alert_text`/`header.stamp` |
| 3.6 | 同上 | 404 | `AttributeError: 'FaultStatus' object has no attribute 'description'` | FaultStatus.msg 字段是 `fault_text` | `.description` → `.fault_text` |
| 3.7 | 同上 | 436-437 | `Fusion3DTarget` 字段不匹配（连锁） | `alert_level` 是 uint8 不能 `.upper()`；`target_type` 不存在 | 用整数比较 `>= 2` 判定 CRITICAL；`.target_type` → `.target_class` |

**Heartbeat.msg 实际定义**:
```
std_msgs/Header  header
string           node_name
uint8            lifecycle_state    # 1=Unconfigured 2=Inactive 3=Active 4=Finalized
uint8            error_code
```

---

### 4. `path_planner`（路径规划器）

| # | 文件 | 行 | 错误信息 | 原因 | 修复 |
|---|------|----|----------|------|------|
| 4.1 | [planner_node.py](src/path_planner/path_planner/planner_node.py#L279) | 279 | `AttributeError: 'PathPlannerNode' object has no attribute '_controller_rate'` | `_load_params()` 中用 `self.controller_rate`（无下划线），但 `__init__` 中用 `self._controller_rate` | 统一为 `self.controller_rate` |

---

### 5. `localization_engine`（定位引擎）

| # | 文件 | 行 | 错误信息 | 原因 | 修复 |
|---|------|----|----------|------|------|
| 5.1 | [localization_node.py](src/localization_engine/localization_engine/localization_node.py#L800-803) | 800 | `Heartbeat` 字段不存在（连锁） | 使用了 `.timestamp`/`.healthy` | 改用 `.header.stamp`/`.lifecycle_state`/`.error_code` |
| 5.2 | 同上 | 823-828 | `AttributeError: 'VisionStatus' object has no attribute 'header'` | VisionStatus.msg 没有 `.header`/`.is_active`/`.confidence` 等字段 | 改用实际字段 `.active_model`/`.status`/`.status_text` |
| 5.3 | 同上 | 263 | `TypeError: can only concatenate tuple (not "int") to tuple` | Bresenham 扫线算法三元表达式错误：`x, y = (x+sx if ... else x), (y+sy if ... else (x+sx, y+sy))` 错误赋值给 y | 重写为标准 Bresenham 算法 |

**VisionStatus.msg 实际定义**:
```
string           active_model
float32          avg_inference_ms
float32          fps
uint8            status              # 0=normal 1=degraded 2=fault
string           status_text
```

**Bresenham 修复 diff**:
```python
# 修复前（错误的单行三元表达式）:
x, y = x + sx if err > -dy else x, y + sy if err < dx else (x + sx, y + sy)
err += dy if err > -dy else -dx

# 修复后（标准 Bresenham）:
e2 = 2 * err
if e2 > -dy:
    err -= dy
    x += sx
if e2 < dx:
    err += dx
    y += sy
```

---

### 6. `motion_controller`（底盘运动控制器）

| # | 文件 | 行 | 错误信息 | 原因 | 修复 |
|---|------|----|----------|------|------|
| 6.1 | [motion_node.py](src/motion_controller/motion_controller/motion_node.py#L47) | 47 | `ModuleNotFoundError: No module named 'tf_transformations'` | `tf_transformations` 未安装（非 ROS2 Humble 默认包） | 实现本地 `euler_from_quaternion()` 和 `quaternion_from_euler()` 函数，替换所有 `tf_transformations.*` 调用（共 3 处） |

---

## 根因分析

所有 Bug 均属于**接口契约不一致**问题——代码中对消息/服务字段的命名和类型假设与 `.msg`/`.srv` IDL 定义存在偏差：

```
代码期望                           IDL 定义
─────────────────────────────────────────────────────
fusion/3d_target                   (以数字开头 → 违规)
hb.module_name / .state            node_name / lifecycle_state
alert.code / .description          alert_code / alert_text
FaultStatus.description            FaultStatus.fault_text
Fusion3DTarget.target_type         target_class
alert_level.upper()                uint8 (不能 .upper())
VisionStatus.header / .is_active   active_model / status
PlannerState.battery               (不存在)
GetStatus ∈ .msg                   GetStatus ∈ .srv
```

---

## 验证结果

```bash
# 构建
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 运行
./start.sh run
```

```
9/9 节点全部启动成功:
  ✅ motion_controller
  ✅ control_gateway
  ✅ lidar_perception
  ✅ vision_detector
  ✅ localization_engine
  ✅ path_planner
  ✅ fusion_engine
  ✅ cleaning_actuator
  ✅ master_controller

零崩溃，日志无 ERROR
```

---

## 涉及文件清单

| 文件 | 修改次数 | 变更类型 |
|------|----------|----------|
| [src/fusion_engine/fusion_engine/fusion_node.py](src/fusion_engine/fusion_engine/fusion_node.py) | 1 | topic 名修复 |
| [src/cleaning_actuator/cleaning_actuator/cleaning_node.py](src/cleaning_actuator/cleaning_actuator/cleaning_node.py) | 3 | LifecycleState 兼容 + current_state tuple |
| [src/master_controller/master_controller/master_node.py](src/master_controller/master_controller/master_node.py) | 10 | 导入修正 + 5 个消息字段对齐 |
| [src/path_planner/path_planner/planner_node.py](src/path_planner/path_planner/planner_node.py) | 1 | 属性命名统一 |
| [src/localization_engine/localization_engine/localization_node.py](src/localization_engine/localization_engine/localization_node.py) | 3 | Heartbeat + VisionStatus 字段 + Bresenham |
| [src/motion_controller/motion_controller/motion_node.py](src/motion_controller/motion_controller/motion_node.py) | 5 | tf_transformations 替换 |
