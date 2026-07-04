#!/usr/bin/env python3
"""
path_planner node: global coverage path planning, Pure Pursuit control,
local obstacle replanning, and multi-area TSP ordering.

Algorithm foundation:
  - Boustrophedon decomposition for coverage path generation
  - Pure Pursuit for smooth path following at 20 Hz
  - Reactive local replanning via TEB/detour logic
  - 2-opt TSP for multi-area ordering
"""

import math
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

from std_msgs.msg import Header
from geometry_msgs.msg import (
    Point, Point32, Polygon, Pose, PoseStamped, Twist, Vector3, Quaternion
)
from nav_msgs.msg import OccupancyGrid

from cleaning_robot_interfaces.msg import (
    TaskArea, TaskMode, Path, PlannerState, ObstacleList, FusionDecision, Heartbeat,
)
from cleaning_robot_interfaces.action import NavigateToGoal


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class PlannerStateEnum(IntEnum):
    IDLE = 0
    PLANNING = 1
    READY = 2
    EXECUTING = 3
    REPLANNING = 4
    BLOCKED = 5
    COMPLETED = 6


STATE_TEXT = {
    0: "IDLE",
    1: "PLANNING",
    2: "READY",
    3: "EXECUTING",
    4: "REPLANNING",
    5: "BLOCKED",
    6: "COMPLETED",
}


class Mode(IntEnum):
    CRUISE = 0
    CLEAN = 1
    RETURN = 2


class FusionAction(IntEnum):
    PASS = 0
    DETOUR = 1
    WAIT = 2
    SLOW = 3


# ---------------------------------------------------------------------------
# Geometry helpers (no external dependency on shapely)
# ---------------------------------------------------------------------------

def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _scale(a, s):
    return (a[0] * s, a[1] * s)


def _norm(a):
    return math.hypot(a[0], a[1])


def _normalize(a):
    n = _norm(a)
    if n < 1e-12:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


def _perp(a):
    """Right-hand perpendicular (rotate by -90 deg)."""
    return (a[1], -a[0])


def _polygon_area(poly):
    """Signed area of polygon (list of (x,y)).  Positive = CCW."""
    area = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return 0.5 * area


def _ensure_ccw(poly):
    """Return polygon vertices in CCW order."""
    if _polygon_area(poly) < 0:
        return list(reversed(poly))
    return list(poly)


def _point_in_polygon(pt, poly):
    """Ray-casting point-in-polygon test."""
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def _distance_point_to_segment(px, py, ax, ay, bx, by):
    """Shortest distance from point P to segment AB."""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    t = (apx * abx + apy * aby) / (abx * abx + aby * aby + 1e-15)
    if t < 0.0:
        return math.hypot(px - ax, py - ay)
    elif t > 1.0:
        return math.hypot(px - bx, py - by)
    else:
        proj_x = ax + t * abx
        proj_y = ay + t * aby
        return math.hypot(px - proj_x, proj_y - proj_y)


def _euler_from_quaternion(q: Quaternion):
    """Return (roll, pitch, yaw) from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


# ---------------------------------------------------------------------------
# Path Planner Node
# ---------------------------------------------------------------------------

class PathPlannerNode(Node):
    """Coverage path planner with Pure Pursuit + local replanning."""

    def __init__(self):
        super().__init__('path_planner')

        # ----- Parameters -----
        self.declare_parameter('cleaning_width', 0.8)
        self.declare_parameter('default_overlap_ratio', 0.1)
        self.declare_parameter('waypoint_spacing', 0.1)
        self.declare_parameter('min_turn_radius', 0.5)
        self.declare_parameter('lookahead_min', 0.5)
        self.declare_parameter('lookahead_max', 2.0)
        self.declare_parameter('lookahead_gain', 0.8)
        self.declare_parameter('k_cross_track', 0.8)
        self.declare_parameter('k_heading', 1.0)
        self.declare_parameter('target_velocity_cruise', 0.8)
        self.declare_parameter('target_velocity_clean', 0.3)
        self.declare_parameter('controller_rate', 20.0)
        self.declare_parameter('position_tolerance', 0.1)
        self.declare_parameter('heading_tolerance_deg', 5.0)
        self.declare_parameter('stop_velocity', 0.05)
        self.declare_parameter('teb_min_obstacle_dist', 0.3)
        self.declare_parameter('planning_timeout_s', 10.0)
        self.declare_parameter('max_replanning_attempts', 5)

        self._load_params()

        # ----- State -----
        self._state = PlannerStateEnum.IDLE
        self._mode = Mode.CRUISE
        self._current_pose: Pose | None = None
        self._current_yaw: float = 0.0
        self._current_speed: float = 0.0
        self._map: OccupancyGrid | None = None
        self._task_area: TaskArea | None = None
        self._active_path: list[tuple[float, float, float]] = []  # [(x, y, yaw), ...]
        self._path_seq: int = 0
        self._current_wp_idx: int = 0
        self._planner_attempts: int = 0
        self._fusion_decision: FusionDecision | None = None
        self._detour_generated: bool = False

        # ----- QoS profiles -----
        reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        reliable_volatile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        heartbeat_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ----- Subscribers -----
        self._sub_task_area = self.create_subscription(
            TaskArea, 'master/task/area', self._cb_task_area, reliable)

        self._sub_task_mode = self.create_subscription(
            TaskMode, 'master/task/mode', self._cb_task_mode, reliable)

        self._sub_pose = self.create_subscription(
            PoseStamped, 'localization/pose', self._cb_pose, sensor_qos)

        self._sub_map = self.create_subscription(
            OccupancyGrid, 'localization/map/occupancy', self._cb_map, reliable_volatile)

        self._sub_obstacle = self.create_subscription(
            ObstacleList, 'lidar/obstacle/list', self._cb_obstacle_list, sensor_qos)

        self._sub_fusion = self.create_subscription(
            FusionDecision, 'fusion/obstacle/decision', self._cb_fusion, reliable)

        # NOTE: vision/detect/drivable is NOT consumed here.
        # All visual obstacle data reaches path_planner through fusion_engine
        # via fusion/obstacle/decision — see the physical isolation contract.

        # ----- Publishers -----
        self._pub_path = self.create_publisher(Path, 'planner/output/path', reliable)
        self._pub_cmd = self.create_publisher(Twist, 'planner/cmd/maneuver', reliable)
        self._pub_state = self.create_publisher(PlannerState, 'planner/status/state', reliable)
        self._pub_heartbeat = self.create_publisher(
            Heartbeat, 'system/heartbeat/path_planner', heartbeat_qos)

        # ----- Action Server -----
        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'planner/navigate_to_goal',
            execute_callback=self._execute_navigate_to_goal,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

        # ----- Timers -----
        self._timer_control = self.create_timer(1.0 / self._controller_rate, self._control_loop)
        self._timer_state = self.create_timer(0.2, self._publish_state)
        self._timer_heartbeat = self.create_timer(1.0, self._publish_heartbeat)

        self.get_logger().info('path_planner node started (IDLE)')

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------

    def _load_params(self):
        self.cleaning_width = self.get_parameter('cleaning_width').value
        self.overlap_ratio = self.get_parameter('default_overlap_ratio').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.min_turn_radius = self.get_parameter('min_turn_radius').value
        self.lookahead_min = self.get_parameter('lookahead_min').value
        self.lookahead_max = self.get_parameter('lookahead_max').value
        self.lookahead_gain = self.get_parameter('lookahead_gain').value
        self.k_cross_track = self.get_parameter('k_cross_track').value
        self.k_heading = self.get_parameter('k_heading').value
        self.target_velocity_cruise = self.get_parameter('target_velocity_cruise').value
        self.target_velocity_clean = self.get_parameter('target_velocity_clean').value
        self.controller_rate = self.get_parameter('controller_rate').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.heading_tolerance = math.radians(
            self.get_parameter('heading_tolerance_deg').value)
        self.stop_velocity = self.get_parameter('stop_velocity').value
        self.teb_min_obstacle_dist = self.get_parameter('teb_min_obstacle_dist').value
        self.planning_timeout_s = self.get_parameter('planning_timeout_s').value
        self.max_replanning_attempts = self.get_parameter('max_replanning_attempts').value

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _cb_task_area(self, msg: TaskArea):
        self._task_area = msg
        self._planner_attempts = 0
        self.get_logger().info(f'Received task area: {msg.area_id}')
        self._transition(PlannerStateEnum.PLANNING)
        success = self._plan_global_path(msg)
        if success:
            self._transition(PlannerStateEnum.READY)
        else:
            self._transition(PlannerStateEnum.BLOCKED)

    def _cb_task_mode(self, msg: TaskMode):
        self._mode = Mode(msg.mode)
        self.get_logger().info(f'Mode changed: {msg.mode_name}')

    def _cb_pose(self, msg: PoseStamped):
        self._current_pose = msg.pose
        self._current_yaw = _euler_from_quaternion(msg.pose.orientation)

    def _cb_map(self, msg: OccupancyGrid):
        self._map = msg

    def _cb_obstacle_list(self, msg: ObstacleList):
        pass  # logged / used for context

    def _cb_fusion(self, msg: FusionDecision):
        self._fusion_decision = msg
        self._handle_fusion_decision(msg)

    # ------------------------------------------------------------------
    #  Global coverage path planning  (Boustrophedon decomposition)
    # ------------------------------------------------------------------

    def _plan_global_path(self, task_area: TaskArea) -> bool:
        """Run Boustrophedon coverage planning on the task polygon."""
        t0 = time.time()

        # 1. Extract polygon vertices from geometry_msgs/Polygon
        poly_pts = [(float(p.x), float(p.y)) for p in task_area.boundary.points]
        if len(poly_pts) < 3:
            self.get_logger().error('Task area polygon has fewer than 3 vertices.')
            return False

        poly_ccw = _ensure_ccw(poly_pts)
        overlap = task_area.overlap_ratio if task_area.overlap_ratio > 0 else self.overlap_ratio
        passes = task_area.cleaning_passes if task_area.cleaning_passes > 0 else 1
        strip_width = self.cleaning_width * (1.0 - overlap)

        # 2. Determine main sweep direction — align with polygon's longest edge
        direction = self._find_sweep_direction(poly_ccw)
        perp_dir = _perp(direction)

        # 3. Compute bounding box along sweep direction for strip generation
        proj_min = float('inf')
        proj_max = float('-inf')
        for p in poly_ccw:
            proj = _dot(p, perp_dir)
            proj_min = min(proj_min, proj)
            proj_max = max(proj_max, proj)

        # Generate coverage strips for each cleaning pass
        all_waypoints: list[tuple[float, float]] = []

        for pass_idx in range(passes):
            if pass_idx > 0:
                # Slightly offset subsequent passes
                perp_dir_pass = _scale(perp_dir, 1.0 + pass_idx * 0.02)
            else:
                perp_dir_pass = perp_dir

            strip_start = proj_min + strip_width * 0.5
            strip_waypoints: list[tuple[float, float]] = []
            left_to_right: list[tuple[float, float]] = []

            pos = strip_start
            strip_idx = 0
            while pos <= proj_max:
                # Sweep line: all points P such that dot(P, perp_dir) = pos
                intersections = self._polygon_line_intersection(poly_ccw, perp_dir_pass, pos)
                if len(intersections) >= 2:
                    # Sort intersections along sweep direction
                    intersections.sort(key=lambda pt: _dot(pt, direction))
                    # Take outermost pair (closest to polygon edges)
                    a = intersections[0]
                    b = intersections[-1]
                    if strip_idx % 2 == 0:
                        left_to_right = [a, b]
                    else:
                        left_to_right = [b, a]
                    strip_waypoints.extend(left_to_right)
                strip_idx += 1
                pos += strip_width

            # Add U-turn connections between adjacent strips
            connected = self._connect_strips(strip_waypoints, perp_dir_pass)
            all_waypoints.extend(connected)

        # 4. Add boundary contour path (perimeter coverage)
        boundary_wps = self._dense_interpolate(poly_ccw + [poly_ccw[0]], self.waypoint_spacing)
        all_waypoints = boundary_wps + all_waypoints

        # 5. Multi-region TSP ordering (single area → no-op here; stub for future)
        #    For a single polygon area, ordering is already correct.
        #    If multiple areas arrive, we would run nearest-neighbour + 2-opt.

        # 6. Waypoint smoothing with moving average filter
        all_waypoints = self._smooth_waypoints(all_waypoints, window=5)

        # 7. Resample to uniform spacing
        all_waypoints = self._dense_interpolate(all_waypoints, self.waypoint_spacing)

        # 8. Compute yaw at each waypoint from segment direction
        self._active_path = self._compute_yaws(all_waypoints)

        # 9. Publish the Path message
        self._publish_path()
        elapsed = (time.time() - t0) * 1000.0
        self.get_logger().info(
            f'Coverage path planned: {len(self._active_path)} waypoints, '
            f'{len(self._active_path)*self.waypoint_spacing:.1f}m in {elapsed:.0f}ms')
        return True

    def _find_sweep_direction(self, poly):
        """Return unit vector aligned with the polygon's longest edge."""
        best_len = 0.0
        best_dir = (1.0, 0.0)
        n = len(poly)
        for i in range(n):
            a = poly[i]
            b = poly[(i + 1) % n]
            vec = _sub(b, a)
            length = _norm(vec)
            if length > best_len:
                best_len = length
                best_dir = _normalize(vec)
        return best_dir

    def _polygon_line_intersection(self, poly, perp_dir, offset):
        """Return sorted list of intersection points of line  dot(p, perp_dir)=offset
        with the polygon edges.  The line is defined in homogeneous form:
            n·p = c,   where n = perp_dir, c = offset
        """
        pts = []
        n = len(poly)
        for i in range(n):
            a = poly[i]
            b = poly[(i + 1) % n]
            da = _dot(a, perp_dir) - offset
            db = _dot(b, perp_dir) - offset
            if da * db < 0:  # straddles the line
                t = da / (da - db + 1e-15)
                ix = a[0] + t * (b[0] - a[0])
                iy = a[1] + t * (b[1] - a[1])
                pts.append((ix, iy))
            elif abs(da) < 1e-9:
                if len(pts) == 0 or _norm(_sub((a[0], a[1]), pts[-1])) > 1e-9:
                    pts.append((a[0], a[1]))
            elif abs(db) < 1e-9:
                if len(pts) == 0 or _norm(_sub((b[0], b[1]), pts[-1])) > 1e-9:
                    pts.append((b[0], b[1]))
        return pts

    def _connect_strips(self, strip_wps, perp_dir):
        """Insert U-turn segments between adjacent (zigzag) strip waypoints."""
        if len(strip_wps) <= 2:
            return strip_wps
        result = [strip_wps[0]]
        i = 1
        while i < len(strip_wps) - 1:
            prev = strip_wps[i - 1]
            curr = strip_wps[i]
            nxt = strip_wps[i + 1]
            # Generate U-turn from prev->curr to curr->nxt with min turn radius
            turn_wps = self._generate_uturn(prev, curr, nxt, self.min_turn_radius)
            result.extend(turn_wps[1:])  # skip first point (duplicate of last added)
            i += 2
        result.append(strip_wps[-1])
        return result

    def _generate_uturn(self, p0, p1, p2, min_radius):
        """Generate smooth U-turn waypoints connecting (p0→p1) to (p1→p2)."""
        in_dir = _normalize(_sub(p1, p0))
        out_dir = _normalize(_sub(p2, p1))
        cross = _dot(in_dir, _perp(out_dir))
        # Determine turn direction (+1 = left, -1 = right)
        sign = 1.0 if cross > 0 else -1.0

        # Use a circular arc as a simple U-turn model
        arc_radius = max(min_radius, 0.3)
        center = _add(p1, _scale(_perp(in_dir), sign * arc_radius))

        # Angle from entry to exit
        angle_start = math.atan2(in_dir[1], in_dir[0]) - sign * math.pi / 2
        angle_end = math.atan2(out_dir[1], out_dir[0]) + sign * math.pi / 2

        if sign > 0:
            while angle_end < angle_start:
                angle_end += 2 * math.pi
        else:
            while angle_end > angle_start:
                angle_end -= 2 * math.pi

        n_pts = max(5, int(abs(angle_end - angle_start) * arc_radius / self.waypoint_spacing))
        result = []
        for j in range(n_pts + 1):
            angle = angle_start + (angle_end - angle_start) * j / n_pts
            x = center[0] + arc_radius * math.cos(angle)
            y = center[1] + arc_radius * math.sin(angle)
            result.append((x, y))
        return result

    def _smooth_waypoints(self, wps, window=5):
        """Moving average filter."""
        if len(wps) < window:
            return wps
        half = window // 2
        smoothed = []
        for i in range(len(wps)):
            x_sum, y_sum, cnt = 0.0, 0.0, 0
            for j in range(max(0, i - half), min(len(wps), i + half + 1)):
                x_sum += wps[j][0]
                y_sum += wps[j][1]
                cnt += 1
            smoothed.append((x_sum / cnt, y_sum / cnt))
        return smoothed

    def _dense_interpolate(self, wps, spacing):
        """Resample path to uniform spacing."""
        if len(wps) < 2:
            return wps
        result = [wps[0]]
        for i in range(1, len(wps)):
            prev = result[-1]
            curr = wps[i]
            seg_len = _norm(_sub(curr, prev))
            if seg_len < 1e-9:
                continue
            n_seg = max(1, int(seg_len / spacing))
            for j in range(1, n_seg + 1):
                t = j / n_seg
                x = prev[0] + t * (curr[0] - prev[0])
                y = prev[1] + t * (curr[1] - prev[1])
                result.append((x, y))
        return result

    def _compute_yaws(self, wps):
        """Assign yaw at each waypoint from the segment direction."""
        result = []
        for i in range(len(wps)):
            if i < len(wps) - 1:
                dx = wps[i + 1][0] - wps[i][0]
                dy = wps[i + 1][1] - wps[i][1]
            elif i > 0:
                dx = wps[i][0] - wps[i - 1][0]
                dy = wps[i][1] - wps[i - 1][1]
            else:
                dx, dy = 1.0, 0.0
            yaw = math.atan2(dy, dx)
            result.append((wps[i][0], wps[i][1], yaw))
        return result

    # ------------------------------------------------------------------
    #  Pure Pursuit path following (20 Hz control loop)
    # ------------------------------------------------------------------

    def _control_loop(self):
        """20 Hz Pure Pursuit + velocity control."""
        if self._state not in (PlannerStateEnum.EXECUTING, PlannerStateEnum.REPLANNING):
            # Publish zero velocity when idle
            if self._state == PlannerStateEnum.IDLE:
                self._pub_cmd.publish(self._zero_twist())
            return

        if self._current_pose is None or not self._active_path:
            return

        # --- Determine target velocity by mode ---
        if self._mode == Mode.CLEAN:
            target_linear = self.target_velocity_clean
        elif self._mode == Mode.RETURN:
            target_linear = self.target_velocity_cruise
        else:
            target_linear = self.target_velocity_cruise

        # SLOW override from fusion
        if self._fusion_decision is not None and self._fusion_decision.action == FusionAction.SLOW:
            target_linear *= self._fusion_decision.slow_ratio

        # --- Find lookahead point ---
        cx = self._current_pose.position.x
        cy = self._current_pose.position.y
        lookahead_dist = min(
            max(self.lookahead_min, self._current_speed * self.lookahead_gain),
            self.lookahead_max,
        )

        target_idx = self._find_lookahead_index(cx, cy, lookahead_dist)
        if target_idx < 0:
            return

        lx, ly, lyaw = self._active_path[target_idx]

        # --- Cross-track error ---
        # Find the nearest segment to the current pose
        xt_err = self._cross_track_error(cx, cy, target_idx)

        # --- Heading error ---
        # Path tangent vs current heading
        heading_err = self._normalize_angle(lyaw - self._current_yaw)

        # --- Pure Pursuit control law ---
        sin_he = math.sin(heading_err)
        angular = (2.0 * target_linear * sin_he / max(lookahead_dist, 0.01)
                   + self.k_cross_track * xt_err)

        # --- Limit angular by turn radius ---
        max_angular = target_linear / max(self.min_turn_radius, 0.01)
        angular = max(-max_angular, min(max_angular, angular))

        # --- Reduce linear velocity when turning sharply ---
        ratio = 1.0 - min(1.0, abs(angular) / max(max_angular, 0.01))
        linear = target_linear * ratio

        # --- Publish Twist ---
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._pub_cmd.publish(twist)

        # Update current speed estimate
        self._current_speed = linear

        # --- Arrival detection ---
        if not self._active_path:
            return
        final_wp = self._active_path[-1]
        dist_final = math.hypot(cx - final_wp[0], cy - final_wp[1])
        heading_err_final = self._normalize_angle(final_wp[2] - self._current_yaw)

        if (dist_final < self.position_tolerance
                and abs(heading_err_final) < self.heading_tolerance
                and abs(linear) < self.stop_velocity):
            self._transition(PlannerStateEnum.COMPLETED)
            self._pub_cmd.publish(self._zero_twist())
            self.get_logger().info('Arrived at final waypoint')

    def _find_lookahead_index(self, cx, cy, lookahead_dist):
        """Find first waypoint at lookahead distance from current position."""
        best_dist = float('inf')
        best_idx = -1
        for i in range(max(self._current_wp_idx, 0), len(self._active_path)):
            wx, wy, _ = self._active_path[i]
            d = math.hypot(wx - cx, wy - cy)
            if d >= lookahead_dist and d < best_dist:
                best_dist = d
                best_idx = i
        if best_idx < 0:
            # Fallback: farthest waypoint
            best_idx = len(self._active_path) - 1
        self._current_wp_idx = best_idx
        return best_idx

    def _cross_track_error(self, cx, cy, target_idx):
        """Compute shortest distance from current pose to the path segment ahead."""
        if target_idx <= 0 or target_idx >= len(self._active_path):
            return 0.0
        # Use the segment ending at target_idx
        ax, ay, _ = self._active_path[target_idx - 1]
        bx, by, _ = self._active_path[target_idx]
        return _distance_point_to_segment(cx, cy, ax, ay, bx, by)

    # ------------------------------------------------------------------
    #  Local obstacle replanning
    # ------------------------------------------------------------------

    def _handle_fusion_decision(self, decision: FusionDecision):
        action = FusionAction(decision.action)
        self.get_logger().info(f'Fusion decision: {action.name} (target={decision.target_id})')

        if action == FusionAction.DETOUR and self._state == PlannerStateEnum.EXECUTING:
            self._transition(PlannerStateEnum.REPLANNING)
            self._generate_detour(decision)
            self._transition(PlannerStateEnum.EXECUTING)

        elif action == FusionAction.WAIT:
            if self._state == PlannerStateEnum.EXECUTING:
                self._pub_cmd.publish(self._zero_twist())
                self.get_logger().info(
                    f'WAIT: pausing for {decision.wait_duration:.1f}s')

        elif action == FusionAction.SLOW:
            # Handled in control loop
            pass

        elif action == FusionAction.PASS:
            # Resume normal operation
            if self._state == PlannerStateEnum.REPLANNING:
                self._transition(PlannerStateEnum.EXECUTING)

    def _generate_detour(self, decision: FusionDecision):
        """Insert detour around obstacle via bezier intermediate waypoint."""
        if self._current_pose is None or not self._active_path:
            return

        cx = self._current_pose.position.x
        cy = self._current_pose.position.y

        # Offset point perpendicular to current path direction
        detour = decision.detour_offset if decision.detour_offset > 0 else 0.5
        # Place offset point to the left of current heading
        offset_dir = (-math.sin(self._current_yaw), math.cos(self._current_yaw))
        mid_x = cx + offset_dir[0] * detour
        mid_y = cy + offset_dir[1] * detour

        # Find a return point further ahead on the original path
        ahead_idx = min(self._current_wp_idx + 20, len(self._active_path) - 1)
        ret_x, ret_y, ret_yaw = self._active_path[ahead_idx]

        # Generate cubic bezier: current → offset_mid → return_point
        bezier_wps = self._cubic_bezier(
            (cx, cy), (mid_x, mid_y), (ret_x, ret_y), steps=15)

        # Build detour path
        detour_path = []
        for i, (bx, by) in enumerate(bezier_wps):
            if i < len(bezier_wps) - 1:
                dyaw = math.atan2(bezier_wps[i + 1][1] - by,
                                  bezier_wps[i + 1][0] - bx)
            else:
                dyaw = ret_yaw
            detour_path.append((bx, by, dyaw))

        # Splice: replace waypoints from current_wp_idx to ahead_idx
        before = self._active_path[:self._current_wp_idx]
        after = self._active_path[ahead_idx + 1:]
        self._active_path = before + detour_path + after
        self._detour_generated = True
        self.get_logger().info(f'Detour generated: {len(detour_path)} waypoints')

    def _cubic_bezier(self, p0, p1, p2, steps=20):
        """Simple cubic bezier with control point at p1."""
        # Use p1 as both control points for symmetry
        pts = []
        for i in range(steps + 1):
            t = i / steps
            x = (1 - t) ** 3 * p0[0] + 3 * (1 - t) ** 2 * t * p1[0] \
                + 3 * (1 - t) * t ** 2 * p1[0] + t ** 3 * p2[0]
            y = (1 - t) ** 3 * p0[1] + 3 * (1 - t) ** 2 * t * p1[1] \
                + 3 * (1 - t) * t ** 2 * p1[1] + t ** 3 * p2[1]
            pts.append((x, y))
        return pts

    # ------------------------------------------------------------------
    #  Multi-area TSP ordering
    # ------------------------------------------------------------------

    def _tsp_order_regions(self, regions: list, start_pose=None):
        """Order multiple region polygons using nearest-neighbour + 2-opt.

        Each region is a tuple: (region_id, centroid_x, centroid_y, vertices).
        Returns ordered list of regions.
        """
        if len(regions) <= 1:
            return regions

        # Nearest-neighbour initialization
        remaining = list(regions)
        ordered = []
        if start_pose:
            cur = (start_pose.position.x, start_pose.position.y)
        else:
            cur = None

        if cur is not None:
            # Start from nearest to current pose
            remaining.sort(key=lambda r: math.hypot(r[1] - cur[0], r[2] - cur[1]))
        ordered.append(remaining.pop(0))
        cur = (ordered[-1][1], ordered[-1][2])

        while remaining:
            remaining.sort(key=lambda r: math.hypot(r[1] - cur[0], r[2] - cur[1]))
            ordered.append(remaining.pop(0))
            cur = (ordered[-1][1], ordered[-1][2])

        # 2-opt improvement
        improved = True
        n = len(ordered)
        while improved:
            improved = False
            for i in range(1, n - 1):
                for j in range(i + 1, n):
                    old_cost = self._segment_length(ordered[i - 1], ordered[i]) \
                        + self._segment_length(ordered[j - 1], ordered[j])
                    if j == n - 1:
                        new_cost = self._segment_length(ordered[i - 1], ordered[j - 1]) \
                            + self._segment_length(ordered[i], ordered[j])
                    else:
                        new_cost = self._segment_length(ordered[i - 1], ordered[j - 1]) \
                            + self._segment_length(ordered[i], ordered[j])
                    if new_cost < old_cost - 1e-9:
                        ordered[i:j] = reversed(ordered[i:j])
                        improved = True

        return ordered

    @staticmethod
    def _segment_length(a, b):
        return math.hypot(a[1] - b[1], a[2] - b[2])

    # ------------------------------------------------------------------
    #  A* on occupancy grid (for navigate_to_goal)
    # ------------------------------------------------------------------

    def _astar_plan(self, start_pose, goal_pose, grid, timeout_s=10.0):
        """Simple A* on 2D occupancy grid."""
        if grid is None:
            return None

        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        width = grid.info.width
        height = grid.info.height
        data = grid.data  # list[int8], 0=free, 100=occupied, -1=unknown

        def world_to_grid(wx, wy):
            gx = int((wx - origin_x) / resolution)
            gy = int((wy - origin_y) / resolution)
            return gx, gy

        def grid_to_world(gx, gy):
            return (origin_x + gx * resolution + resolution / 2,
                    origin_y + gy * resolution + resolution / 2)

        start_gx, start_gy = world_to_grid(start_pose.position.x, start_pose.position.y)
        goal_gx, goal_gy = world_to_grid(goal_pose.position.x, goal_pose.position.y)

        # Bounds check
        if not (0 <= start_gx < width and 0 <= start_gy < height):
            return None
        if not (0 <= goal_gx < width and 0 <= goal_gy < height):
            return None

        # A* search
        import heapq

        def cost(gx, gy):
            idx = gy * width + gx
            if 0 <= idx < len(data):
                if data[idx] > 50:  # occupied or unknown
                    return float('inf')
                return 1.0 + (data[idx] / 100.0) * 10.0  # bias against high occupancy
            return float('inf')

        def heuristic(gx, gy):
            return math.hypot(gx - goal_gx, gy - goal_gy)

        open_set = []
        heapq.heappush(open_set, (heuristic(start_gx, start_gy), 0, start_gx, start_gy))
        came_from: dict = {}
        g_score: dict = {(start_gx, start_gy): 0.0}
        visited: set = set()

        t_start = time.time()
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            if time.time() - t_start > timeout_s:
                self.get_logger().warn('A* timeout')
                return None

            _, _, cx, cy = heapq.heappop(open_set)
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))

            if cx == goal_gx and cy == goal_gy:
                # Reconstruct path
                path = [(goal_gx, goal_gy)]
                cur = (goal_gx, goal_gy)
                while cur != (start_gx, start_gy):
                    cur = came_from[cur]
                    path.append(cur)
                path.reverse()
                return [grid_to_world(px, py) for px, py in path]

            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if (nx, ny) in visited:
                    continue
                step_cost = math.sqrt(dx * dx + dy * dy)
                if cost(nx, ny) >= float('inf'):
                    continue
                tentative_g = g_score[(cx, cy)] + step_cost * cost(nx, ny)
                if tentative_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + heuristic(nx, ny)
                    heapq.heappush(open_set, (f, tentative_g, nx, ny))

        return None  # No path found

    # ------------------------------------------------------------------
    #  Action Server: navigate_to_goal
    # ------------------------------------------------------------------

    def _handle_goal(self, goal_request):
        self.get_logger().info('Received navigate_to_goal request')
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle):
        self.get_logger().info('navigate_to_goal cancelled')
        self._transition(PlannerStateEnum.IDLE)
        return CancelResponse.ACCEPT

    def _execute_navigate_to_goal(self, goal_handle):
        goal = goal_handle.request
        feedback = NavigateToGoal.Feedback()
        result = NavigateToGoal.Result()

        if self._current_pose is None:
            result.success = False
            goal_handle.abort()
            return result

        # Plan with A*
        path_world = self._astar_plan(
            self._current_pose, goal.target_pose.pose, self._map, goal.timeout_s)

        if path_world is None:
            self.get_logger().error('navigate_to_goal: A* failed')
            result.success = False
            result.final_distance = float('inf')
            result.path_length = 0.0
            goal_handle.abort()
            return result

        # Convert to waypoints with yaw
        self._active_path = self._compute_yaws(path_world)
        total_length = len(path_world) * self.waypoint_spacing
        self._publish_path()
        self._transition(PlannerStateEnum.READY)
        self._transition(PlannerStateEnum.EXECUTING)

        rate = self.create_rate(5.0)
        start_time = time.time()
        timeout = goal.timeout_s if goal.timeout_s > 0 else float('inf')

        while rclpy.ok():
            if time.time() - start_time > timeout:
                result.success = False
                result.final_distance = float('inf')
                result.path_length = total_length
                goal_handle.abort()
                self.get_logger().warn('navigate_to_goal: timeout')
                break

            if goal_handle.is_cancel_requested:
                result.success = False
                goal_handle.canceled()
                self.get_logger().info('navigate_to_goal: cancelled')
                break

            # Compute feedback
            if self._current_pose and self._active_path:
                final_wp = self._active_path[-1]
                fx = self._current_pose.position.x
                fy = self._current_pose.position.y
                dist = math.hypot(fx - final_wp[0], fy - final_wp[1])
                feedback.progress = max(0.0, min(1.0, 1.0 - dist / max(total_length, 0.01)))
                feedback.distance_to_goal = dist
                feedback.estimated_remaining_s = dist / max(self._current_speed, 0.01)
                goal_handle.publish_feedback(feedback)

            if self._state == PlannerStateEnum.COMPLETED:
                result.success = True
                result.final_distance = feedback.distance_to_goal
                result.path_length = total_length
                goal_handle.succeed()
                self.get_logger().info('navigate_to_goal: succeeded')
                break

            rate.sleep()

        return result

    # ------------------------------------------------------------------
    #  State transitions
    # ------------------------------------------------------------------

    def _transition(self, next_state: PlannerStateEnum):
        if next_state == self._state:
            return
        prev = STATE_TEXT.get(self._state, "UNKNOWN")
        nxt = STATE_TEXT.get(next_state, "UNKNOWN")
        self.get_logger().info(f'State: {prev} → {nxt}')
        self._state = next_state

    # ------------------------------------------------------------------
    #  Publishers
    # ------------------------------------------------------------------

    def _publish_path(self):
        if not self._active_path:
            return
        msg = Path()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        msg.sequence = self._path_seq
        self._path_seq += 1
        msg.total_length = len(self._active_path) * self.waypoint_spacing
        for x, y, yaw in self._active_path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation = _quaternion_from_yaw(yaw)
            msg.waypoints.append(ps)
        self._pub_path.publish(msg)

    def _publish_state(self):
        msg = PlannerState()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='')
        msg.state = int(self._state)
        msg.state_text = STATE_TEXT.get(int(self._state), "UNKNOWN")
        self._pub_state.publish(msg)

    def _publish_heartbeat(self):
        msg = Heartbeat()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='')
        msg.node_name = 'path_planner'
        msg.lifecycle_state = 3  # Active
        msg.error_code = 0
        self._pub_heartbeat.publish(msg)

    # ------------------------------------------------------------------
    #  Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def _zero_twist() -> Twist:
        return Twist()

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
