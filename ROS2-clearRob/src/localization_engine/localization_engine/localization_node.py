#!/usr/bin/env python3
"""
localization_engine.localization_node

SLAM localization engine for cleaning robot system.

Handles simulated SLAM, occupancy grid mapping, loop closure detection,
and tf2 coordinate transform broadcasting.

No real SLAM/scan-matcher library is used; instead, odometry + IMU are fused
with simulated drift and corrections to produce a realistic localization
pipeline suitable for integration testing.
"""

import math
import random
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Float32
from geometry_msgs.msg import (
    Pose,
    PoseWithCovarianceStamped,
    TransformStamped,
    Quaternion,
)
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# Custom interfaces – imported conditionally for environments where they
# might not yet be built.
try:
    from cleaning_robot_interfaces.srv import GetCurrentPose, RequestMap
    from cleaning_robot_interfaces.msg import Heartbeat, VisionStatus
    _HAS_CUSTOM_IFACES = True
except ImportError:
    GetCurrentPose = None
    RequestMap = None
    Heartbeat = None
    VisionStatus = None
    _HAS_CUSTOM_IFACES = False


# ---------------------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------------------

def euler_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
    """Return (roll, pitch, yaw) from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return (
        math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                   1.0 - 2.0 * (q.x * q.x + q.y * q.y)),
        math.asin(2.0 * (q.w * q.y - q.z * q.x)),
        math.atan2(siny_cosp, cosy_cosp),
    )


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Return geometry_msgs/Quaternion from Euler angles."""
    q = Quaternion()
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def pose_to_numpy(pose: Pose) -> np.ndarray:
    """Convert Pose to [x, y, yaw]."""
    _, _, yaw = euler_from_quaternion(pose.orientation)
    return np.array([pose.position.x, pose.position.y, yaw])


def numpy_to_pose(arr: np.ndarray) -> Pose:
    """Convert [x, y, yaw] to Pose."""
    p = Pose()
    p.position.x = float(arr[0])
    p.position.y = float(arr[1])
    p.position.z = 0.0
    p.orientation = quaternion_from_euler(0.0, 0.0, float(arr[2]))
    return p


# ---------------------------------------------------------------------------
# Occupancy grid helper
# ---------------------------------------------------------------------------

class OccupancyGridManager:
    """
    Maintains a 2-D occupancy grid map with dynamic expansion.
    """

    def __init__(self, resolution: float, init_size_x: float, init_size_y: float):
        self.resolution = resolution
        self.half_size_x = init_size_x / 2.0
        self.half_size_y = init_size_y / 2.0
        # origin offset maps continuous world coords -> grid indices
        self._origin_x = -self.half_size_x
        self._origin_y = -self.half_size_y
        self._reallocate()

    def _reallocate(self):
        w = max(1, int(self.half_size_x * 2.0 / self.resolution))
        h = max(1, int(self.half_size_y * 2.0 / self.resolution))
        self.width = w
        self.height = h
        self.data = np.full((h, w), -1, dtype=np.int8)  # -1 = unknown
        self.occupancy_hits = np.zeros((h, w), dtype=np.uint16)
        self.free_hits = np.zeros((h, w), dtype=np.uint16)
        self._update_bbox = None  # (x0,y0,x1,y1) in grid coords

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int((wx - self._origin_x) / self.resolution)
        gy = int((wy - self._origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = (gx + 0.5) * self.resolution + self._origin_x
        wy = (gy + 0.5) * self.resolution + self._origin_y
        return wx, wy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def ensure_size(self, min_wx: float, min_wy: float,
                    max_wx: float, max_wy: float):
        """Expand grid if needed to cover [min_wx,max_wx] x [min_wy,max_wy]."""
        needed = False
        while min_wx < self._origin_x:
            self.half_size_x += self.half_size_x * 0.5
            self._origin_x = -self.half_size_x
            needed = True
        while max_wx > self._origin_x + self.width * self.resolution:
            self.half_size_x += self.half_size_x * 0.5
            self._origin_x = -self.half_size_x
            needed = True
        while min_wy < self._origin_y:
            self.half_size_y += self.half_size_y * 0.5
            self._origin_y = -self.half_size_y
            needed = True
        while max_wy > self._origin_y + self.height * self.resolution:
            self.half_size_y += self.half_size_y * 0.5
            self._origin_y = -self.half_size_y
            needed = True
        if needed:
            self._reallocate()

    def _update_cell(self, gx: int, gy: int, occupied: bool):
        if not self.in_bounds(gx, gy):
            return
        if occupied:
            self.occupancy_hits[gy, gx] = min(100, self.occupancy_hits[gy, gx] + 1)
        else:
            self.free_hits[gy, gx] = min(100, self.free_hits[gy, gx] + 1)
        total = int(self.occupancy_hits[gy, gx]) + int(self.free_hits[gy, gx])
        if total > 0:
            ratio = int(self.occupancy_hits[gy, gx]) / total
            self.data[gy, gx] = int(np.clip(ratio * 100.0, 0, 100))
        # Track bbox of changed cells
        if self._update_bbox is None:
            self._update_bbox = [gx, gy, gx, gy]
        else:
            self._update_bbox[0] = min(self._update_bbox[0], gx)
            self._update_bbox[1] = min(self._update_bbox[1], gy)
            self._update_bbox[2] = max(self._update_bbox[2], gx)
            self._update_bbox[3] = max(self._update_bbox[3], gy)

    def integrate_point_cloud(self, origin: np.ndarray,  # [x,y,yaw]
                               points_world: np.ndarray):
        """
        Ray-trace from origin to each point; mark hits (occupied) and
        traversed cells (free).
        """
        ox, oy, oyaw = origin
        ogx, ogy = self.world_to_grid(ox, oy)
        # Ensure the grid covers our sensor origin and target points
        if len(points_world) > 0:
            self.ensure_size(
                min(ox, float(np.min(points_world[:, 0]))),
                min(oy, float(np.min(points_world[:, 1]))),
                max(ox, float(np.max(points_world[:, 0]))),
                max(oy, float(np.max(points_world[:, 1]))),
            )
        else:
            self.ensure_size(ox - 1.0, oy - 1.0, ox + 1.0, oy + 1.0)
        # Re-obtain origin grid coords after possible resize
        ogx, ogy = self.world_to_grid(ox, oy)
        for i in range(len(points_world)):
            px, py = points_world[i, 0], points_world[i, 1]
            pgx, pgy = self.world_to_grid(px, py)
            # Bresenham line from (ogx,ogy) -> (pgx,pgy)
            for gx, gy in _bresenham_line(ogx, ogy, pgx, pgy):
                self._update_cell(gx, gy, occupied=False)
            self._update_cell(pgx, pgy, occupied=True)

    def populate_dummy_scan(self, pose: np.ndarray, ranges: List[float],
                            angles: List[float]):
        """Project a set of ranges/angles (in robot frame) onto the grid."""
        x, y, yaw = pose
        points = []
        for r, a in zip(ranges, angles):
            if r <= 0.0 or r > 30.0:
                continue
            wx = x + r * math.cos(yaw + a)
            wy = y + r * math.sin(yaw + a)
            points.append((wx, wy))
        self.integrate_point_cloud(pose, np.array(points, dtype=np.float64))

    def fetch_and_reset_bbox(self) -> Optional[Tuple[int, int, int, int]]:
        b = self._update_bbox
        self._update_bbox = None
        return b

    def to_ros_msg(self, stamp, frame_id: str) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        # Flatten row-major
        msg.data = self.data.flatten().tolist()
        return msg


def _bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Yield grid cells along a line from (x0,y0) to (x1,y1), exclusive of start."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
        if (x, y) == (x1, y1):
            break
        cells.append((x, y))
    return cells


# ---------------------------------------------------------------------------
# Simulated sensor data (for testing without real hardware)
# ---------------------------------------------------------------------------

class SimulatedOdometry:
    """
    Generate fake odometry moving in a slow spiral to exercise the pipeline.
    """

    def __init__(self):
        self.t = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.omega = 0.0

    def step(self, dt: float):
        """Advance spiral: radius grows at ~0.15 m/s radial, 0.2 rad/s angular."""
        self.t += dt
        r = 0.15 * self.t
        omega = 0.2
        self.v = 0.15
        self.omega = omega
        self.yaw += omega * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

    def pose(self) -> np.ndarray:
        return np.array([self.x, self.y, self.yaw])


# ---------------------------------------------------------------------------
# Main Node
# ---------------------------------------------------------------------------

class LocalizationEngine(Node):
    """ROS2 Node: localization_engine — Lifecycle-managed simulated SLAM."""

    def __init__(self):
        super().__init__('localization_engine')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_init_size_x', 200.0)
        self.declare_parameter('map_init_size_y', 200.0)
        self.declare_parameter('map_publish_rate_max', 2.0)
        self.declare_parameter('slam_algorithm', 'simulated_ekf')
        self.declare_parameter('submap_size', 30.0)
        self.declare_parameter('max_submaps', 10)
        self.declare_parameter('loop_closure_enabled', True)
        self.declare_parameter('loop_closure_min_dist', 3.0)
        self.declare_parameter('loop_closure_min_interval_s', 30.0)
        self.declare_parameter('loop_closure_match_threshold', 0.75)
        self.declare_parameter('ekf_process_noise_xy', 0.05)
        self.declare_parameter('ekf_process_noise_yaw', 0.02)
        self.declare_parameter('ekf_odom_noise_xy', 0.02)
        self.declare_parameter('ekf_odom_noise_yaw', 0.01)
        self.declare_parameter('confidence_scan_match_weight', 0.5)
        self.declare_parameter('confidence_covariance_weight', 0.3)
        self.declare_parameter('confidence_imu_weight', 0.2)
        self.declare_parameter('max_covariance_trace', 0.5)
        self.declare_parameter('localization_rate', 50.0)
        self.declare_parameter('scan_match_rate', 20.0)

        # Derived shortcuts
        self.map_res = self.get_parameter('map_resolution').value
        self.loop_enabled = self.get_parameter('loop_closure_enabled').value
        self.loop_min_dist = self.get_parameter('loop_closure_min_dist').value
        self.loop_min_interval = self.get_parameter('loop_closure_min_interval_s').value

        # ------------------------------------------------------------------
        # State variables
        # ------------------------------------------------------------------
        # Pose in map frame [x, y, yaw]; start at origin
        self.map_pose = np.array([0.0, 0.0, 0.0])
        # Accumulated drift estimate
        self.accumulated_drift = np.array([0.0, 0.0, 0.0])
        self.total_distance_traveled = 0.0
        self.last_odom_pose: Optional[np.ndarray] = None  # previous odom [x,y,yaw]
        self.last_odom_stamp: Optional[Time] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_pointcloud_stamp: Optional[Time] = None
        self._pointcloud_timeout = Duration(seconds=2.0)

        # Covariance estimates
        self.pose_covariance = np.diag([0.01, 0.01, 0.001])  # 3x3
        self.imu_yaw_rate = 0.0
        self.odom_yaw_rate = 0.0
        self.scan_match_score = 0.8
        self.localization_confidence = 1.0

        # Loop closure history
        self.loop_history: deque = deque(maxlen=500)
        self.last_loop_closure_time: Optional[Time] = None

        # Occupancy grid
        self.occ_grid = OccupancyGridManager(
            self.map_res,
            self.get_parameter('map_init_size_x').value,
            self.get_parameter('map_init_size_y').value,
        )
        self.last_map_publish_time = self.get_clock().now()

        # Simulated odometry fallback
        self.sim_odom = SimulatedOdometry()
        self._use_sim_odom = True  # start in sim mode until real data arrives
        self._last_real_odom_time: Optional[Time] = None
        self._odom_timeout = Duration(seconds=0.5)

        # Map memory guard
        self._max_map_cells = 100_000_000

        # ------------------------------------------------------------------
        # QoS profiles
        # ------------------------------------------------------------------
        qos_sensor = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        qos_reliable = QoSProfile(depth=10)
        qos_tf_static = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ------------------------------------------------------------------
        # Callback groups (separate for services vs. timers/subs)
        # ------------------------------------------------------------------
        self._srv_cb = MutuallyExclusiveCallbackGroup()

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        self._sub_pc = self.create_subscription(
            PointCloud2, 'lidar/scan/pointcloud',
            self._cb_pointcloud, qos_sensor)
        self._sub_odom = self.create_subscription(
            Odometry, 'motion/odom',
            self._cb_odom, qos_reliable)
        self._sub_imu = self.create_subscription(
            Imu, 'sensor/imu/data',
            self._cb_imu, qos_sensor)

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, 'localization/pose', 10)
        self._pub_map = self.create_publisher(
            OccupancyGrid, 'localization/map/occupancy', qos_tf_static)
        self._pub_conf = self.create_publisher(
            Float32, 'localization/confidence', 10)
        self._pub_heartbeat = self.create_publisher(
            Heartbeat if _HAS_CUSTOM_IFACES else Float32,
            'system/heartbeat/localization_engine', 10)
        self._pub_status = self.create_publisher(
            VisionStatus if _HAS_CUSTOM_IFACES else Float32,
            'localization/status', 10)

        # ------------------------------------------------------------------
        # Services
        # ------------------------------------------------------------------
        if _HAS_CUSTOM_IFACES:
            self._srv_pose = self.create_service(
                GetCurrentPose, 'localization/get_current_pose',
                self._cb_get_current_pose, callback_group=self._srv_cb)
            self._srv_map = self.create_service(
                RequestMap, 'localization/request_map',
                self._cb_request_map, callback_group=self._srv_cb)

        # ------------------------------------------------------------------
        # tf2 broadcasters
        # ------------------------------------------------------------------
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_static_broadcaster = StaticTransformBroadcaster(self)

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        loc_rate = self.get_parameter('localization_rate').value
        self._timer_loc = self.create_timer(
            1.0 / loc_rate, self._cb_localization_tick)

        conf_rate = 5.0
        self._timer_conf = self.create_timer(
            1.0 / conf_rate, self._cb_confidence_tick)

        self._timer_heartbeat = self.create_timer(1.0, self._cb_heartbeat_tick)
        self._timer_status = self.create_timer(1.0, self._cb_status_tick)

        self._timer_map_publish = self.create_timer(
            1.0 / self.get_parameter('map_publish_rate_max').value,
            self._cb_map_publish_check)

        # odom -> base_link at 100 Hz (separate from localization loop)
        self._timer_odom_tf = self.create_timer(0.01, self._cb_odom_tf_tick)

        # Publish static transforms once at startup
        self._publish_static_transforms()

        self.get_logger().info('LocalizationEngine node started')

    # -------------------------------------------------------------------
    # Static transforms (base_link -> sensors, base_footprint)
    # -------------------------------------------------------------------

    def _make_static_tf(self, parent: str, child: str,
                        x: float, y: float, z: float,
                        roll: float = 0.0, pitch: float = 0.0,
                        yaw: float = 0.0) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = quaternion_from_euler(roll, pitch, yaw)
        return t

    def _publish_static_transforms(self):
        now = self.get_clock().now().to_msg()
        transforms = [
            self._make_static_tf('base_link', 'base_footprint', 0.0, 0.0, -0.15),
            self._make_static_tf('base_link', 'camera_link', 0.2, 0.0, 0.5),
            self._make_static_tf('base_link', 'lidar_link', 0.1, 0.0, 0.4),
            self._make_static_tf('base_link', 'imu_link', 0.0, 0.0, 0.1),
        ]
        for t in transforms:
            t.header.stamp = now
        self._tf_static_broadcaster.sendTransform(transforms)

    # -------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------

    def _cb_odom(self, msg: Odometry):
        now = self.get_clock().now()
        # Odometry jump guard (>2 m in one frame -> discard)
        cur_odom = pose_to_numpy(msg.pose.pose)
        if self.last_odom_pose is not None:
            jump = np.linalg.norm(cur_odom[:2] - self.last_odom_pose[:2])
            if jump > 2.0:
                self.get_logger().warn(
                    f'Odometry jump {jump:.2f}m > 2m, discarding frame',
                    throttle_duration_sec=5.0)
                return
        self.last_odom_pose = cur_odom
        self.last_odom_stamp = now
        self._last_real_odom_time = now
        self._use_sim_odom = False

        # Compute yaw rate from odometry
        if hasattr(msg, 'twist') and msg.twist.twist.angular.z != 0.0:
            self.odom_yaw_rate = msg.twist.twist.angular.z
        elif self.last_odom_stamp is not None:
            # fallback: differentiate pose
            pass

    def _cb_imu(self, msg: Imu):
        self.latest_imu = msg
        self.imu_yaw_rate = msg.angular_velocity.z

    def _cb_pointcloud(self, msg: PointCloud2):
        self.latest_pointcloud_stamp = self.get_clock().now()
        # The simulated point cloud processing is handled in the scan-match
        # timer; here we just record the timestamp for the timeout guard.
        # A real implementation would deserialize the point cloud and pass
        # points to the occupancy grid.
        pass

    def _cb_get_current_pose(self, request, response):
        response.pose = PoseWithCovarianceStamped()
        response.pose.header.stamp = self.get_clock().now().to_msg()
        response.pose.header.frame_id = 'map'
        response.pose.pose.pose = numpy_to_pose(self.map_pose)
        cov_flat = self.pose_covariance.flatten().tolist()
        response.pose.pose.covariance = cov_flat + [0.0] * (36 - len(cov_flat))
        response.valid = True
        return response

    def _cb_request_map(self, request, response):
        response.map = self.occ_grid.to_ros_msg(
            self.get_clock().now().to_msg(), 'map')
        response.success = True
        return response

    # -------------------------------------------------------------------
    # Periodic callbacks
    # -------------------------------------------------------------------

    def _cb_localization_tick(self):
        """Main localization loop: 50 Hz."""
        now = self.get_clock().now()
        dt = 0.02  # ~50 Hz

        # ---- Odometry source selection -----------------------------------
        if self._use_sim_odom:
            self.sim_odom.step(dt)
            odom_pose = self.sim_odom.pose()
        elif self._last_real_odom_time is not None:
            elapsed = (now - self._last_real_odom_time).nanoseconds * 1e-9
            if elapsed > 0.5:
                self.get_logger().info(
                    'Real odometry timeout, switching to simulated odometry',
                    throttle_duration_sec=10.0)
                self.sim_odom = SimulatedOdometry()
                self._use_sim_odom = True
                self.sim_odom.step(dt)
                odom_pose = self.sim_odom.pose()
            else:
                # Real odometry is active; odom_pose from last _cb_odom
                odom_pose = (
                    self.last_odom_pose
                    if self.last_odom_pose is not None
                    else self.sim_odom.pose()
                )
        else:
            # No real data yet — stay in sim mode
            self.sim_odom.step(dt)
            odom_pose = self.sim_odom.pose()
            self._use_sim_odom = True

        # ---- Drift model ------------------------------------------------
        delta_dist = 0.0
        if self.last_odom_pose is not None and not self._use_sim_odom:
            delta = odom_pose - self.last_odom_pose
            delta_dist = np.linalg.norm(delta[:2])
            self.total_distance_traveled += delta_dist

        # Accumulate drift: 0.1 m per 100 m traveled
        if delta_dist > 0:
            drift_gain = 0.1 / 100.0
            drift_angle = random.uniform(0, 2.0 * math.pi)
            self.accumulated_drift[0] += drift_gain * delta_dist * math.cos(drift_angle)
            self.accumulated_drift[1] += drift_gain * delta_dist * math.sin(drift_angle)
            self.accumulated_drift[2] += random.uniform(-0.002, 0.002) * delta_dist

        # Simulated scan-match correction (probabilistic)
        pc_age = 0.0
        if self.latest_pointcloud_stamp is not None:
            pc_age = (now - self.latest_pointcloud_stamp).nanoseconds * 1e-9
        if pc_age < 2.0:
            # Occasional correction: reduce drift by a random fraction
            if random.random() < 0.02:  # ~1 correction per second at 50 Hz
                correction_fraction = random.uniform(0.5, 0.95)
                self.accumulated_drift *= (1.0 - correction_fraction)
                self.scan_match_score = random.uniform(0.7, 0.95)
        else:
            # No point cloud -> confidence decays
            self.scan_match_score *= 0.98

        # ---- Loop closure (simulated) -----------------------------------
        if self.loop_enabled:
            self._check_loop_closure(now)

        # ---- Update map pose --------------------------------------------
        if not self._use_sim_odom and self.last_odom_pose is not None:
            self.map_pose = odom_pose + self.accumulated_drift
        else:
            self.map_pose = odom_pose

        # Update covariance (simple decay model)
        self.pose_covariance *= 0.9995
        self.pose_covariance += np.diag([1e-5, 1e-5, 1e-6])

        # Cap covariance trace
        max_trace = self.get_parameter('max_covariance_trace').value
        if np.trace(self.pose_covariance) > max_trace:
            scale = max_trace / np.trace(self.pose_covariance)
            self.pose_covariance *= scale

        # ---- Publish pose -----------------------------------------------
        self._publish_current_pose(now)

        # ---- Broadcast tf2 map->odom (50Hz) -------------------------------
        self._broadcast_map_odom_tf(now)

        # ---- Integrate dummy scan into occupancy grid ------------------
        self._integrate_dummy_scan_into_map()

        # ---- Record loop-closure history --------------------------------
        self.loop_history.append((now.nanoseconds * 1e-9, self.map_pose.copy()))

    def _check_loop_closure(self, now: Time):
        """Simulate loop closure when revisiting a location."""
        if self.last_loop_closure_time is not None:
            since_last = (now - self.last_loop_closure_time).nanoseconds * 1e-9
            if since_last < self.loop_min_interval:
                return
        now_sec = now.nanoseconds * 1e-9
        for hist_sec, hist_pose in self.loop_history:
            if now_sec - hist_sec < self.loop_min_interval:
                continue
            dist = np.linalg.norm(self.map_pose[:2] - hist_pose[:2])
            if dist < self.loop_min_dist:
                # Simulate loop closure: reduce accumulated drift by 90%
                self.accumulated_drift *= 0.1
                self.last_loop_closure_time = now
                self.scan_match_score = min(1.0, self.scan_match_score + 0.1)
                self.get_logger().info(
                    f'Loop closure detected! Distance to history: {dist:.2f}m. '
                    f'Drift reduced by 90%.')
                break

    def _publish_current_pose(self, now: Time):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose = numpy_to_pose(self.map_pose)
        cov_flat = self.pose_covariance.flatten().tolist()
        msg.pose.covariance = cov_flat + [0.0] * (36 - len(cov_flat))
        self._pub_pose.publish(msg)

    def _broadcast_map_odom_tf(self, now: Time):
        """Broadcast map->odom transform at 50 Hz (with drift correction)."""
        t_mo = TransformStamped()
        t_mo.header.stamp = now.to_msg()
        t_mo.header.frame_id = 'map'
        t_mo.child_frame_id = 'odom'
        t_mo.transform.translation.x = self.accumulated_drift[0]
        t_mo.transform.translation.y = self.accumulated_drift[1]
        t_mo.transform.translation.z = 0.0
        t_mo.transform.rotation = quaternion_from_euler(
            0.0, 0.0, self.accumulated_drift[2])
        self._tf_broadcaster.sendTransform(t_mo)

    def _cb_odom_tf_tick(self):
        """Broadcast odom->base_link transform at 100 Hz (pass-through)."""
        now = self.get_clock().now()
        t_ob = TransformStamped()
        t_ob.header.stamp = now.to_msg()
        t_ob.header.frame_id = 'odom'
        t_ob.child_frame_id = 'base_link'
        if self._use_sim_odom or self.last_odom_pose is None:
            ref_pose = self.sim_odom.pose()
            t_ob.transform.translation.x = ref_pose[0]
            t_ob.transform.translation.y = ref_pose[1]
            t_ob.transform.translation.z = 0.0
            t_ob.transform.rotation = quaternion_from_euler(0.0, 0.0, ref_pose[2])
        else:
            t_ob.transform.translation.x = self.last_odom_pose[0]
            t_ob.transform.translation.y = self.last_odom_pose[1]
            t_ob.transform.translation.z = 0.0
            t_ob.transform.rotation = quaternion_from_euler(
                0.0, 0.0, self.last_odom_pose[2])
        self._tf_broadcaster.sendTransform(t_ob)

    def _integrate_dummy_scan_into_map(self):
        """Feed simulated laser ranges (robot-relative) into the occupancy grid."""
        # Generate ~180 rays in a 270-degree FOV
        num_rays = 180
        angles = np.linspace(-3.0 * math.pi / 4.0, 3.0 * math.pi / 4.0, num_rays)
        rng = []
        for a in angles:
            # Simulate range: longer in open direction of motion
            base = 8.0 + 4.0 * math.cos(a)  # bias forward
            r = abs(random.gauss(base, 1.5))
            rng.append(min(r, 30.0))
        self.occ_grid.populate_dummy_scan(self.map_pose, rng, angles)

    def _cb_map_publish_check(self):
        """Publish occupancy grid at ≤2 Hz, only when the grid changed."""
        bbox = self.occ_grid.fetch_and_reset_bbox()
        if bbox is None:
            return  # No new data
        now = self.get_clock().now()
        elapsed = (now - self.last_map_publish_time).nanoseconds * 1e-9
        min_interval = 1.0 / self.get_parameter('map_publish_rate_max').value
        if elapsed < min_interval:
            return  # Rate-limited
        self.last_map_publish_time = now
        grid_msg = self.occ_grid.to_ros_msg(now.to_msg(), 'map')
        self._pub_map.publish(grid_msg)

        # Map memory guard
        total_cells = self.occ_grid.width * self.occ_grid.height
        if total_cells > self._max_map_cells:
            self.get_logger().warn(
                f'Map exceeds {self._max_map_cells} cells; '
                f'crop not implemented (guard warning).',
                throttle_duration_sec=30.0)

    def _cb_confidence_tick(self):
        """Compute and publish localization confidence at 5 Hz."""
        # scan_match_score (w1=0.5)
        w1 = self.get_parameter('confidence_scan_match_weight').value
        w2 = self.get_parameter('confidence_covariance_weight').value
        w3 = self.get_parameter('confidence_imu_weight').value
        max_cov = self.get_parameter('max_covariance_trace').value

        cov_trace = np.trace(self.pose_covariance)
        cov_term = 1.0 - min(1.0, cov_trace / max_cov)

        max_diff = 0.5  # rad/s
        if self.latest_imu is not None:
            imu_consistency = 1.0 - min(1.0,
                abs(self.imu_yaw_rate - self.odom_yaw_rate) / max_diff)
        else:
            imu_consistency = 0.5

        # Point cloud staleness penalty
        now = self.get_clock().now()
        if self.latest_pointcloud_stamp is not None:
            pc_age = (now - self.latest_pointcloud_stamp).nanoseconds * 1e-9
        else:
            pc_age = 999.0
        if pc_age > 2.0:
            # Rapid confidence decay when no point cloud
            self.scan_match_score *= 0.9
            self.scan_match_score = max(0.1, self.scan_match_score)

        confidence = (w1 * self.scan_match_score +
                      w2 * cov_term +
                      w3 * imu_consistency)
        confidence = max(0.0, min(1.0, confidence))
        self.localization_confidence = confidence

        msg = Float32()
        msg.data = float(confidence)
        self._pub_conf.publish(msg)

    def _cb_heartbeat_tick(self):
        now = self.get_clock().now()
        if _HAS_CUSTOM_IFACES:
            hb = Heartbeat()
            hb.header.stamp = now.to_msg()
            hb.header.frame_id = ''
            hb.node_name = 'localization_engine'
            hb.lifecycle_state = 3  # ACTIVE
            hb.error_code = (
                0 if self.localization_confidence > 0.3 else 2
            )
            self._pub_heartbeat.publish(hb)
        else:
            hb = Float32()
            hb.data = 1.0 if self.localization_confidence > 0.3 else 0.0
            self._pub_heartbeat.publish(hb)

    def _cb_status_tick(self):
        now = self.get_clock().now()
        if _HAS_CUSTOM_IFACES:
            status = VisionStatus()
            status.active_model = 'simulated_ekf'
            status.avg_inference_ms = 0.0
            status.fps = 0.0
            status.status = 0 if self.localization_confidence > 0.3 else 2
            status.status_text = (
                'normal' if self.localization_confidence > 0.3 else 'degraded')
            self._pub_status.publish(status)
        else:
            status = Float32()
            status.data = float(self.localization_confidence)
            self._pub_status.publish(status)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
