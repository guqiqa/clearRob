#!/usr/bin/env python3
"""
LiDAR Perception Node for Cleaning Robot.

This node handles:
- Point cloud preprocessing (downsample, outlier removal, ground segmentation, range filtering)
- Obstacle detection and clustering (Euclidean clustering, tracking, velocity estimation)
- Emergency stop (ESTOP) triggering based on proximity and obstacle classification
- Simulated LiDAR scan generation for testing
- Mode-aware scanning strategies (CRUISE / CLEAN / RETURN)
"""

import math
import random
import struct
import time
import threading
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, Vector3
from cleaning_robot_interfaces.msg import (
    TaskMode,
    ObstacleItem,
    ObstacleList,
    MotionCommand,
    VisionStatus,
    Heartbeat,
)
from cleaning_robot_interfaces.srv import GetStatus, SetScanFreq


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

class ModeEnum(IntEnum):
    """Task mode enumeration matching TaskMode.msg."""
    CRUISE = 0
    CLEAN = 1
    RETURN = 2


class ClassificationEnum(IntEnum):
    """Obstacle classification enumeration matching ObstacleItem.msg."""
    UNKNOWN = 0
    SMALL = 1   # < 0.1 m
    MEDIUM = 2  # 0.1 ~ 0.5 m
    LARGE = 3   # > 0.5 m


class StatusEnum(IntEnum):
    """Module status enumeration matching VisionStatus.msg."""
    NORMAL = 0
    DEGRADED = 1
    FAULT = 2


class LifecycleEnum(IntEnum):
    """Lifecycle state enumeration matching Heartbeat.msg."""
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4


# PointCloud2 field layout (4 floats per point: x, y, z, intensity)
DEFAULT_POINT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]

POINT_STEP = 16  # 4 bytes * 4 fields


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def _make_pointcloud2(header, points):
    """Build a PointCloud2 message from a list of (x, y, z, intensity) tuples."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = DEFAULT_POINT_FIELDS
    msg.is_bigendian = False
    msg.point_step = POINT_STEP
    msg.row_step = POINT_STEP * len(points)
    msg.is_dense = True
    data = bytearray()
    for px, py, pz, pi in points:
        data.extend(struct.pack('<ffff', px, py, pz, pi))
    msg.data = bytes(data)
    return msg


def _unpack_pointcloud2(msg):
    """Unpack a PointCloud2 message into a list of (x, y, z, intensity) tuples."""
    points = []
    data = msg.data
    step = msg.point_step
    count = msg.width * (msg.height or 1)
    for i in range(count):
        offset = i * step
        x, y, z, i_val = struct.unpack_from('<ffff', data, offset)
        points.append((x, y, z, i_val))
    return points


def _euclidean_distance(p1, p2):
    """Euclidean distance between two (x, y, z) points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)


def _distance_from_origin(p):
    """Planar distance from robot origin (x, y, 0)."""
    return math.sqrt(p[0] ** 2 + p[1] ** 2)


def _classify_obstacle(max_dim):
    """Classify obstacle based on its largest bounding-box dimension."""
    if max_dim < 0.1:
        return ClassificationEnum.SMALL
    elif max_dim <= 0.5:
        return ClassificationEnum.MEDIUM
    else:
        return ClassificationEnum.LARGE


# ---------------------------------------------------------------------------
# Hungarian Algorithm (simplified, for obstacle association)
# ---------------------------------------------------------------------------

def _hungarian_assign(cost_matrix):
    """
    Greedy assignment for small matrices (simpler than full Hungarian).
    Returns list of (row, col) index pairs.
    For our use case (few obstacles), greedy is sufficient.
    """
    n_rows = len(cost_matrix)
    n_cols = len(cost_matrix[0]) if cost_matrix else 0
    if n_rows == 0 or n_cols == 0:
        return []

    assigned_rows = set()
    assigned_cols = set()
    pairs = []

    # Collect all (cost, row, col) and sort by cost ascending
    entries = []
    for r in range(n_rows):
        for c in range(n_cols):
            entries.append((cost_matrix[r][c], r, c))
    entries.sort(key=lambda e: e[0])

    for cost, r, c in entries:
        if r not in assigned_rows and c not in assigned_cols:
            pairs.append((r, c))
            assigned_rows.add(r)
            assigned_cols.add(c)

    return pairs


# ---------------------------------------------------------------------------
# LiDAR Perception Node
# ---------------------------------------------------------------------------

class LidarPerceptionNode(LifecycleNode):
    """Lifecycle-managed LiDAR perception node."""

    def __init__(self):
        super().__init__('lidar_perception')

        # ---- Parameters ----
        self.declare_parameter('scan_freq', 10.0)
        self.declare_parameter('horizontal_fov_deg', 360.0)
        self.declare_parameter('vertical_fov_deg', 30.0)
        self.declare_parameter('voxel_leaf_size', 0.05)
        self.declare_parameter('sor_mean_k', 20)
        self.declare_parameter('sor_std_dev_mul', 2.0)
        self.declare_parameter('ground_ransac_threshold', 0.05)
        self.declare_parameter('height_filter_min', 0.1)
        self.declare_parameter('height_filter_max', 3.0)
        self.declare_parameter('radius_filter_min', 0.3)
        self.declare_parameter('radius_filter_max', 30.0)
        self.declare_parameter('cluster_tolerance', 0.1)
        self.declare_parameter('cluster_min_points', 10)
        self.declare_parameter('cluster_max_points', 5000)
        self.declare_parameter('assoc_max_distance', 0.5)
        self.declare_parameter('estop_min_distance_cruise', 0.3)
        self.declare_parameter('estop_min_distance_clean', 0.3)
        self.declare_parameter('estop_min_distance_return', 0.5)
        self.declare_parameter('estop_large_obstacle_threshold', 1.0)
        self.declare_parameter('estop_large_obstacle_angle', 30.0)
        self.declare_parameter('estop_trigger_frames', 2)
        self.declare_parameter('estop_release_frames', 5)
        self.declare_parameter('estop_pointcloud_ratio_min', 0.3)
        self.declare_parameter('simulate_scans', True)

        # ---- State ----
        self._current_mode = ModeEnum.CRUISE
        self._scan_freq = 10.0
        self._fault_triggered = False
        self._sensor_timeout = 2.0
        self._last_sensor_time = 0.0

        # Point count for ratio degradation check
        self._prev_frame_point_count = 0
        self._prev_frame_time = 0.0

        # Noise degradation tracking
        self._noise_degrade_counter = 0
        self._last_point_count_raw = 0
        self._last_point_count_processed = 0

        # ESTOP state machine
        self._estop_active = False
        self._estop_trigger_counter = 0
        self._estop_release_counter = 0

        # Obstacle tracking state
        self._tracked_obstacles = []  # list of dicts: id, position, dimensions, dist, vel, cls, last_seen
        self._next_obstacle_id = 0

        # Timers
        self._sim_timer = None
        self._heartbeat_timer = None
        self._status_timer = None
        self._sensor_watchdog_timer = None

        # Node start time for uptime calculation
        self._start_time = time.time()

        # ---- Thread safety ----
        self._mode_lock = threading.Lock()

        self.get_logger().info('LidarPerceptionNode constructed.')

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the node: create pubs/subs/services."""
        self.get_logger().info('Configuring...')

        # Read parameters
        self._scan_freq = self.get_parameter('scan_freq').value
        self._simulate_scans = self.get_parameter('simulate_scans').value

        # QoS profiles
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(depth=10)
        periodic_qos = QoSProfile(depth=1)

        # Subscriptions
        self._sub_scan_raw = self.create_subscription(
            PointCloud2,
            'sensor/lidar/scan_raw',
            self._scan_raw_callback,
            sensor_qos,
        )
        self._sub_task_mode = self.create_subscription(
            TaskMode,
            'master/task/mode',
            self._task_mode_callback,
            reliable_qos,
        )

        # Publishers
        self._pub_pointcloud = self.create_publisher(
            PointCloud2, 'lidar/scan/pointcloud', reliable_qos,
        )
        self._pub_obstacle_list = self.create_publisher(
            ObstacleList, 'lidar/obstacle/list', reliable_qos,
        )
        self._pub_estop = self.create_publisher(
            MotionCommand, 'lidar/cmd/estop', reliable_qos,
        )
        self._pub_status = self.create_publisher(
            VisionStatus, 'lidar/status', periodic_qos,
        )
        self._pub_heartbeat = self.create_publisher(
            Heartbeat, 'system/heartbeat/lidar_perception', periodic_qos,
        )

        # Services
        cb_group = MutuallyExclusiveCallbackGroup()
        self._srv_get_status = self.create_service(
            GetStatus, 'lidar/get_status', self._get_status_callback, callback_group=cb_group,
        )
        self._srv_set_scan_freq = self.create_service(
            SetScanFreq, 'lidar/set_scan_freq', self._set_scan_freq_callback, callback_group=cb_group,
        )

        self.get_logger().info('Configured successfully.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the node: start timers."""
        self.get_logger().info('Activating...')

        # Reset state on activation
        self._estop_active = False
        self._estop_trigger_counter = 0
        self._estop_release_counter = 0
        self._fault_triggered = False
        self._noise_degrade_counter = 0
        self._start_time = time.time()

        # Timers
        sim_period = 1.0 / max(self._scan_freq, 0.1)
        heartbeat_period = 1.0
        status_period = 1.0
        watchdog_period = 0.5

        self._sim_timer = self.create_timer(sim_period, self._simulate_scan_timer_callback)
        self._heartbeat_timer = self.create_timer(heartbeat_period, self._heartbeat_callback)
        self._status_timer = self.create_timer(status_period, self._status_callback)
        self._sensor_watchdog_timer = self.create_timer(watchdog_period, self._sensor_watchdog_callback)

        self.get_logger().info(f'Activated. Scan freq: {self._scan_freq} Hz, Simulate: {self._simulate_scans}')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the node: stop timers."""
        self.get_logger().info('Deactivating...')
        self._destroy_timers()
        # Publish ESTOP release on deactivation
        self._publish_estop(False)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup the node: destroy pubs/subs/services."""
        self.get_logger().info('Cleaning up...')
        self._destroy_timers()
        self.destroy_subscription(self._sub_scan_raw)
        self.destroy_subscription(self._sub_task_mode)
        self.destroy_publisher(self._pub_pointcloud)
        self.destroy_publisher(self._pub_obstacle_list)
        self.destroy_publisher(self._pub_estop)
        self.destroy_publisher(self._pub_status)
        self.destroy_publisher(self._pub_heartbeat)
        self.destroy_client(self._srv_get_status)
        self.destroy_client(self._srv_set_scan_freq)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown the node."""
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle error transition."""
        self.get_logger().error('Error state entered.')
        self._destroy_timers()
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _destroy_timers(self):
        """Safely destroy all running timers."""
        for t in (self._sim_timer, self._heartbeat_timer, self._status_timer, self._sensor_watchdog_timer):
            if t is not None:
                self.destroy_timer(t)
        self._sim_timer = None
        self._heartbeat_timer = None
        self._status_timer = None
        self._sensor_watchdog_timer = None

    def _now(self):
        """Current time as float seconds (simplified monotonic)."""
        return self.get_clock().now().nanoseconds / 1e9

    def _make_header(self):
        """Build a standard message header."""
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'lidar_frame'
        return h

    def _get_effective_range(self):
        """Return (min_radius, max_radius) based on current mode."""
        with self._mode_lock:
            mode = self._current_mode
        if mode == ModeEnum.CLEAN:
            return (self.get_parameter('radius_filter_min').value, 10.0)
        else:  # CRUISE or RETURN
            return (self.get_parameter('radius_filter_min').value, 30.0)

    def _get_effective_scan_freq(self):
        """Return scan frequency based on current mode."""
        with self._mode_lock:
            mode = self._current_mode
        if mode == ModeEnum.CLEAN:
            return 20.0
        else:  # CRUISE or RETURN
            return self._scan_freq

    def _get_estop_params(self):
        """Return (min_safe_distance, ground_segmentation) based on current mode."""
        with self._mode_lock:
            mode = self._current_mode
        if mode == ModeEnum.CLEAN:
            min_safe = self.get_parameter('estop_min_distance_clean').value
            ground_seg = False
        elif mode == ModeEnum.RETURN:
            min_safe = self.get_parameter('estop_min_distance_return').value
            ground_seg = True
        else:  # CRUISE
            min_safe = self.get_parameter('estop_min_distance_cruise').value
            ground_seg = True
        return (min_safe, ground_seg)

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _scan_raw_callback(self, msg: PointCloud2):
        """Process incoming PointCloud2 data through the preprocessing pipeline."""
        self._last_sensor_time = self._now()

        # Unpack raw point cloud
        raw_points = _unpack_pointcloud2(msg)
        raw_count = len(raw_points)

        if raw_count == 0:
            return

        # ---- Point cloud preprocessing ----
        processed = self._preprocess_pointcloud(raw_points)

        # Check noise degradation
        processed_count = len(processed)
        noise_ratio = 1.0 - (processed_count / max(raw_count, 1))
        if noise_ratio > 0.5:
            self._noise_degrade_counter += 1
        else:
            self._noise_degrade_counter = max(0, self._noise_degrade_counter - 1)

        # Point count ratio check
        self._prev_frame_time = self._now()

        # Publish preprocessed point cloud
        header = self._make_header()
        pc_msg = _make_pointcloud2(header, processed)
        self._pub_pointcloud.publish(pc_msg)

        # ---- Obstacle detection and clustering ----
        min_safe_dist, ground_seg = self._get_estop_params()
        obstacles = self._detect_obstacles(processed, ground_seg)

        # Track obstacles across frames
        self._track_obstacles_across_frames(obstacles)

        # Build and publish obstacle list
        obs_list_msg = self._build_obstacle_list_msg(header)
        self._pub_obstacle_list.publish(obs_list_msg)

        # ---- ESTOP judgment ----
        estop_conditions = self._evaluate_estop_conditions(obs_list_msg, processed_count, raw_count)
        self._update_estop_state(estop_conditions)

        # Store for next frame
        self._prev_frame_point_count = processed_count

    def _task_mode_callback(self, msg: TaskMode):
        """Handle mode switch from master controller."""
        with self._mode_lock:
            self._current_mode = ModeEnum(msg.mode)
        self.get_logger().info(f'Mode switched to: {msg.mode_name} ({msg.mode})')

        # Update simulation timer rate on mode change
        if self._sim_timer is not None:
            eff_freq = self._get_effective_scan_freq()
            new_period = 1.0 / max(eff_freq, 0.1)
            self._sim_timer.timer_period_ns = int(new_period * 1e9)

    # ------------------------------------------------------------------
    # Preprocessing pipeline
    # ------------------------------------------------------------------

    def _preprocess_pointcloud(self, points):
        """Run the full preprocessing pipeline on raw points."""
        # 1. Downsample (simulated voxel grid)
        downsample_factor = 4
        points = [p for i, p in enumerate(points) if i % downsample_factor == 0]

        # 2. Outlier removal (simulated: remove extreme coordinates)
        sor_mul = self.get_parameter('sor_std_dev_mul').value
        points = self._remove_outliers(points, sor_mul)

        # 3. Ground segmentation (simulated RANSAC)
        ground_thresh = self.get_parameter('ground_ransac_threshold').value
        _, ground_seg = self._get_estop_params()
        if ground_seg:
            non_ground = [p for p in points if p[2] > (0.05 + ground_thresh)]
        else:
            non_ground = points
            # Still remove very low points that could be noise
            non_ground = [p for p in non_ground if p[2] > -0.5]

        # 4. Range filtering
        r_min, r_max = self._get_effective_range()
        h_min = self.get_parameter('height_filter_min').value
        h_max = self.get_parameter('height_filter_max').value
        filtered = []
        for p in non_ground:
            dist = _distance_from_origin(p)
            if r_min <= dist <= r_max and h_min <= p[2] <= h_max:
                filtered.append(p)

        return filtered

    def _remove_outliers(self, points, std_dev_mul):
        """Simulate Statistical Outlier Removal by filtering extreme coordinate values."""
        if len(points) < 3:
            return points

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]

        def _stats(values):
            n = len(values)
            mean = sum(values) / n
            var = sum((v - mean) ** 2 for v in values) / n
            std = math.sqrt(var)
            if std == 0:
                return mean, mean - 1.0, mean + 1.0
            lo = mean - std_dev_mul * std
            hi = mean + std_dev_mul * std
            return mean, lo, hi

        _, x_lo, x_hi = _stats(xs)
        _, y_lo, y_hi = _stats(ys)
        _, z_lo, z_hi = _stats(zs)

        filtered = [
            p for p in points
            if x_lo <= p[0] <= x_hi and y_lo <= p[1] <= y_hi and z_lo <= p[2] <= z_hi
        ]
        return filtered

    # ------------------------------------------------------------------
    # Obstacle detection and clustering
    # ------------------------------------------------------------------

    def _detect_obstacles(self, non_ground_points, ground_seg):
        """Cluster non-ground points into obstacle candidates."""
        if len(non_ground_points) < self.get_parameter('cluster_min_points').value:
            # With ground segmentation off, there will be many ground points,
            # but we filter by height, so fewer obstacles may still be valid
            return []

        cluster_tol = self.get_parameter('cluster_tolerance').value
        min_pts = self.get_parameter('cluster_min_points').value
        max_pts = self.get_parameter('cluster_max_points').value

        # Euclidean clustering (simple region-growing)
        clusters = self._euclidean_clustering(non_ground_points, cluster_tol)

        obstacles = []
        for cluster_pts in clusters:
            if len(cluster_pts) < min_pts or len(cluster_pts) > max_pts:
                continue

            # Centroid
            n = len(cluster_pts)
            cx = sum(p[0] for p in cluster_pts) / n
            cy = sum(p[1] for p in cluster_pts) / n
            cz = sum(p[2] for p in cluster_pts) / n

            # Bounding box dimensions
            xs = [p[0] for p in cluster_pts]
            ys = [p[1] for p in cluster_pts]
            zs = [p[2] for p in cluster_pts]
            dx = max(xs) - min(xs)
            dy = max(ys) - min(ys)
            dz = max(zs) - min(zs)

            # Distance from sensor origin
            dist = _distance_from_origin((cx, cy, cz))

            # Classification
            max_dim = max(dx, dy, dz)
            cls = _classify_obstacle(max_dim)

            obstacles.append({
                'position': (cx, cy, cz),
                'dimensions': (dx, dy, dz),
                'distance': dist,
                'classification': cls,
                'velocity': 0.0,
                'point_count': n,
            })

        return obstacles

    def _euclidean_clustering(self, points, tolerance):
        """Region-growing Euclidean clustering."""
        if not points:
            return []

        unvisited = set(range(len(points)))
        clusters = []

        while unvisited:
            seed_idx = unvisited.pop()
            seed_set = {seed_idx}
            cluster = [seed_idx]
            while seed_set:
                current = seed_set.pop()
                cp = points[current]
                for other in list(unvisited):
                    if _euclidean_distance(cp, points[other]) <= tolerance:
                        unvisited.remove(other)
                        seed_set.add(other)
                        cluster.append(other)
            clusters.append([points[i] for i in cluster])

        return clusters

    def _track_obstacles_across_frames(self, detections):
        """Track obstacles across frames using Hungarian matching."""
        assoc_max_dist = self.get_parameter('assoc_max_distance').value
        now = self._now()

        prev = self._tracked_obstacles

        if not detections:
            # No new detections: mark all tracked as lost
            self._tracked_obstacles = []
            return

        if not prev:
            # No prior tracks: assign new IDs to all detections
            for det in detections:
                det['id'] = self._next_obstacle_id
                self._next_obstacle_id += 1
                det['last_seen'] = now
            self._tracked_obstacles = detections
            return

        # Build cost matrix
        n_det = len(detections)
        n_trk = len(prev)
        cost = [[0.0] * n_trk for _ in range(n_det)]
        for d in range(n_det):
            for t in range(n_trk):
                cost[d][t] = _euclidean_distance(
                    detections[d]['position'], prev[t]['position']
                )

        # Hungarian assignment
        pairs = _hungarian_assign(cost)

        assigned_det = set()
        assigned_trk = set()
        for d, t in pairs:
            dist = cost[d][t]
            if dist <= assoc_max_dist:
                ppos = prev[t]['position']
                dpos = detections[d]['position']
                # Compute velocity from position delta
                dt = now - prev[t].get('last_seen', now - 0.1)
                vel = _euclidean_distance(dpos, ppos) / max(dt, 0.01)
                detections[d]['id'] = prev[t]['id']
                detections[d]['velocity'] = vel
                detections[d]['last_seen'] = now
                assigned_det.add(d)
                assigned_trk.add(t)

        # New detections (unassigned) get new IDs
        for d in range(n_det):
            if d not in assigned_det:
                detections[d]['id'] = self._next_obstacle_id
                self._next_obstacle_id += 1
                detections[d]['velocity'] = 0.0
                detections[d]['last_seen'] = now

        self._tracked_obstacles = detections

    def _build_obstacle_list_msg(self, header):
        """Build ObstacleList message from tracked obstacles."""
        msg = ObstacleList()
        msg.header = header

        for obs in self._tracked_obstacles:
            item = ObstacleItem()
            item.header = header
            item.obstacle_id = obs['id']
            item.position = Point(x=float(obs['position'][0]),
                                  y=float(obs['position'][1]),
                                  z=float(obs['position'][2]))
            item.dimensions = Vector3(x=float(obs['dimensions'][0]),
                                      y=float(obs['dimensions'][1]),
                                      z=float(obs['dimensions'][2]))
            item.distance = float(obs['distance'])
            item.velocity = float(obs['velocity'])
            item.classification = int(obs['classification'])
            msg.obstacles.append(item)

        return msg

    # ------------------------------------------------------------------
    # ESTOP logic
    # ------------------------------------------------------------------

    def _evaluate_estop_conditions(self, obs_list_msg, processed_count, raw_count):
        """Evaluate all ESTOP trigger conditions. Returns True if any triggers."""
        min_safe_dist, _ = self._get_estop_params()
        large_thresh = self.get_parameter('estop_large_obstacle_threshold').value
        large_angle = math.radians(self.get_parameter('estop_large_obstacle_angle').value)
        pc_ratio_min = self.get_parameter('estop_pointcloud_ratio_min').value

        # Condition 1: Any obstacle within min_safe_distance
        for obs in obs_list_msg.obstacles:
            if obs.distance < min_safe_dist:
                self.get_logger().warn(
                    f'ESTOP condition 1: obstacle {obs.obstacle_id} at {obs.distance:.2f}m < {min_safe_dist}m'
                )
                return True

        # Condition 2: Large obstacle within large_thresh AND angular position within +/- large_angle of forward
        for obs in obs_list_msg.obstacles:
            if obs.classification == ClassificationEnum.LARGE and obs.distance < large_thresh:
                # Compute angular position from forward (+x) direction
                angle = math.atan2(obs.position.y, obs.position.x)
                if abs(angle) <= large_angle:
                    self.get_logger().warn(
                        f'ESTOP condition 2: large obstacle {obs.obstacle_id} '
                        f'at {obs.distance:.2f}m, angle {math.degrees(angle):.1f} deg'
                    )
                    return True

        # Condition 3: Point cloud quality drop
        if self._prev_frame_point_count > 0:
            ratio = processed_count / self._prev_frame_point_count
            if ratio < pc_ratio_min:
                self.get_logger().warn(
                    f'ESTOP condition 3: point count ratio {ratio:.2f} < {pc_ratio_min}'
                )
                return True

        return False

    def _update_estop_state(self, conditions_met):
        """Update ESTOP state machine with debouncing."""
        trigger_frames = self.get_parameter('estop_trigger_frames').value
        release_frames = self.get_parameter('estop_release_frames').value

        if conditions_met:
            self._estop_trigger_counter += 1
            self._estop_release_counter = 0
            if self._estop_trigger_counter >= trigger_frames and not self._estop_active:
                self._estop_active = True
                self._publish_estop(True)
                self.get_logger().error(
                    f'ESTOP ACTIVATED after {trigger_frames} consecutive trigger frames.'
                )
        else:
            self._estop_release_counter += 1
            self._estop_trigger_counter = 0
            if self._estop_release_counter >= release_frames and self._estop_active:
                self._estop_active = False
                self._publish_estop(False)
                self.get_logger().info(
                    f'ESTOP RELEASED after {release_frames} consecutive safe frames.'
                )

    def _publish_estop(self, active: bool):
        """Publish ESTOP MotionCommand directly."""
        cmd = MotionCommand()
        cmd.header = self._make_header()
        cmd.estop = active
        cmd.linear_velocity = 0.0
        cmd.angular_velocity = 0.0
        self._pub_estop.publish(cmd)

    # ------------------------------------------------------------------
    # Simulated scan generation
    # ------------------------------------------------------------------

    def _simulate_scan_timer_callback(self):
        """Generate simulated LiDAR data when external sensor is absent."""
        if not self._simulate_scans:
            return

        # Check if we should use external sensor instead
        now = self._now()
        sensor_stale = (now - self._last_sensor_time) > self._sensor_timeout

        if not sensor_stale and self._last_sensor_time > 0:
            # External sensor is producing data; skip simulation
            return

        points = self._generate_simulated_scan()
        header = self._make_header()

        # Run the full pipeline on simulated data
        processed = self._preprocess_pointcloud(points)
        pc_msg = _make_pointcloud2(header, processed)
        self._pub_pointcloud.publish(pc_msg)

        # Obstacle detection
        _, ground_seg = self._get_estop_params()
        obstacles = self._detect_obstacles(processed, ground_seg)
        self._track_obstacles_across_frames(obstacles)
        obs_list_msg = self._build_obstacle_list_msg(header)
        self._pub_obstacle_list.publish(obs_list_msg)

        # ESTOP
        estop_meta = self._evaluate_estop_conditions(obs_list_msg, len(processed), len(points))
        self._update_estop_state(estop_meta)

        self._prev_frame_point_count = len(processed)

    def _generate_simulated_scan(self):
        """Generate a fake multi-layer LiDAR scan pattern with obstacles."""
        points = []
        h_fov = math.radians(self.get_parameter('horizontal_fov_deg').value)
        v_fov = math.radians(self.get_parameter('vertical_fov_deg').value)
        n_rings = 8  # simulate 8-layer LiDAR
        n_azimuth = 360  # points per ring
        r_min, r_max = self._get_effective_range()

        # Base cylindrical scan pattern
        for ring in range(n_rings):
            v_angle = -v_fov / 2.0 + (ring / (n_rings - 1)) * v_fov
            for i in range(n_azimuth):
                h_angle = -h_fov / 2.0 + (i / (n_azimuth - 1)) * h_fov
                r = random.uniform(r_min, r_max)
                x = r * math.cos(v_angle) * math.cos(h_angle)
                y = r * math.cos(v_angle) * math.sin(h_angle)
                z = r * math.sin(v_angle)
                # Add noise
                x += random.gauss(0, 0.02)
                y += random.gauss(0, 0.02)
                z += random.gauss(0, 0.02)
                intensity = random.uniform(0.1, 1.0)
                points.append((x, y, z, intensity))

        # Inject obstacle clusters at random positions
        n_obstacles = random.randint(1, 4)
        for _ in range(n_obstacles):
            angle = random.uniform(-h_fov / 2.0 * 0.8, h_fov / 2.0 * 0.8)
            dist = random.uniform(1.0, min(r_max, 20.0))
            obst_x = dist * math.cos(angle)
            obst_y = dist * math.sin(angle)
            size = random.choice([0.08, 0.15, 0.3, 0.6, 0.8])  # m
            n_obs_pts = random.randint(30, 200)

            for _ in range(n_obs_pts):
                ox = obst_x + random.gauss(0, size / 3.0)
                oy = obst_y + random.gauss(0, size / 3.0)
                oz = random.uniform(0.1, size)
                intensity = random.uniform(0.3, 1.0)
                points.append((ox, oy, oz, intensity))

        return points

    # ------------------------------------------------------------------
    # Sensor watchdog
    # ------------------------------------------------------------------

    def _sensor_watchdog_callback(self):
        """Check if external sensor has timed out."""
        now = self._now()
        if self._last_sensor_time > 0 and (now - self._last_sensor_time) > self._sensor_timeout:
            if not self._fault_triggered:
                self._fault_triggered = True
                self.get_logger().error('Sensor timeout: lidar/scan_raw > 2s stale.')
            # ESTOP on timeout when not in simulation mode
            if not self._simulate_scans:
                self._estop_active = True
                self._publish_estop(True)
        else:
            self._fault_triggered = False

    # ------------------------------------------------------------------
    # Periodic publishers
    # ------------------------------------------------------------------

    def _heartbeat_callback(self):
        """Publish heartbeat at 1Hz."""
        msg = Heartbeat()
        msg.header = self._make_header()
        msg.node_name = 'lidar_perception'

        # Map lifecycle state to heartbeat enum
        lc_state = self._state_machine.current_state
        if lc_state == LifecycleState.PRIMARY_STATE_UNCONFIGURED:
            msg.lifecycle_state = LifecycleEnum.UNCONFIGURED
        elif lc_state == LifecycleState.PRIMARY_STATE_INACTIVE:
            msg.lifecycle_state = LifecycleEnum.INACTIVE
        elif lc_state == LifecycleState.PRIMARY_STATE_ACTIVE:
            msg.lifecycle_state = LifecycleEnum.ACTIVE
        elif lc_state == LifecycleState.PRIMARY_STATE_FINALIZED:
            msg.lifecycle_state = LifecycleEnum.FINALIZED
        else:
            msg.lifecycle_state = LifecycleEnum.UNCONFIGURED

        msg.error_code = 0  # Normal
        self._pub_heartbeat.publish(msg)

    def _status_callback(self):
        """Publish module status at 1Hz."""
        msg = VisionStatus()
        msg.active_model = 'lidar_perception'

        if self._fault_triggered:
            msg.status = StatusEnum.FAULT
            msg.status_text = 'sensor_timeout'
        elif self._noise_degrade_counter >= 10:
            msg.status = StatusEnum.DEGRADED
            msg.status_text = 'high_noise_ratio'
        else:
            msg.status = StatusEnum.NORMAL
            msg.status_text = 'operational'

        msg.fps = self._get_effective_scan_freq()
        msg.avg_inference_ms = 0.0

        self._pub_status.publish(msg)

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _get_status_callback(self, request, response):
        """Handle lidar/get_status service."""
        response.node_name = 'lidar_perception'

        if self._fault_triggered:
            response.status = StatusEnum.FAULT
            response.status_text = 'sensor_timeout'
        elif self._noise_degrade_counter >= 10:
            response.status = StatusEnum.DEGRADED
            response.status_text = 'high_noise_ratio'
        else:
            response.status = StatusEnum.NORMAL
            response.status_text = 'operational'

        response.uptime_s = float(time.time() - self._start_time)
        return response

    def _set_scan_freq_callback(self, request, response):
        """Handle lidar/set_scan_freq service."""
        freq = request.frequency
        if 5.0 <= freq <= 20.0:
            self._scan_freq = freq
            self.set_parameters(
                [rclpy.parameter.Parameter('scan_freq', rclpy.Parameter.Type.DOUBLE, freq)]
            )
            # Update timer
            if self._sim_timer is not None:
                eff_freq = self._get_effective_scan_freq()
                self._sim_timer.timer_period_ns = int((1.0 / eff_freq) * 1e9)
            response.success = True
            response.message = f'Scan frequency set to {freq:.1f} Hz'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f'Invalid frequency {freq:.1f} Hz (range: 5~20 Hz)'
        return response


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = LidarPerceptionNode()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
