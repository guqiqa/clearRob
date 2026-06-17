#!/usr/bin/env python3
"""
fusion_engine ROS2 Node.

Fuses vision 2D detections with LiDAR 3D point clouds to produce:
- 3D target estimations (Fusion3DTarget)
- Obstacle avoidance decisions (FusionDecision)
- Status and heartbeat monitoring
"""

import math
import time
from typing import List, Optional, Tuple

import message_filters
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import PointCloud2

from cleaning_robot_interfaces.msg import (
    DetectionArray,
    Detection,
    ObstacleList,
    ObstacleItem,
    Fusion3DTarget,
    FusionDecision,
    VisionStatus,
    Heartbeat,
)
from cleaning_robot_interfaces.srv import GetStatus


# ---------------------------------------------------------------------------
# Alert level constants
# ---------------------------------------------------------------------------
ALERT_INFO = 0
ALERT_WARN = 1
ALERT_CRITICAL = 2

# Decision action constants
DECISION_PASS = 0
DECISION_DETOUR = 1
DECISION_WAIT = 2
DECISION_SLOW = 3

# Decision priority ordering (higher number = more conservative)
DECISION_PRIORITY = {
    DECISION_PASS: 0,
    DECISION_SLOW: 1,
    DECISION_DETOUR: 2,
    DECISION_WAIT: 3,
}

# Node status constants
STATUS_NORMAL = 0
STATUS_DEGRADED = 1
STATUS_FAULT = 2

# Heartbeat lifecycle states
LIFECYCLE_ACTIVE = 3

# Error codes
ERROR_NONE = 0
ERROR_RUNTIME = 2

# Obstacle classification
OBS_CLASS_UNKNOWN = 0
OBS_CLASS_SMALL = 1
OBS_CLASS_MEDIUM = 2
OBS_CLASS_LARGE = 3


class FusionEngineNode(Node):
    """Sensor fusion node for vision + LiDAR data."""

    def __init__(self) -> None:
        super().__init__('fusion_engine')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self._declare_parameters()

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._start_time: float = time.monotonic()
        self._last_vision_time: float = 0.0
        self._vision_alive_counter: float = 0.0
        self._last_lidar_pc_time: float = 0.0
        self._pure_lidar_mode: bool = False
        self._vision_recovery_start: Optional[float] = None

        # ------------------------------------------------------------------
        # QoS profiles
        # ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(depth=10)

        # ------------------------------------------------------------------
        # Message filter synchronizer (ApproximateTime)
        # Synchronizes vision/detect/list + lidar/scan/pointcloud + lidar/obstacle/list
        # ------------------------------------------------------------------
        self._vision_sub = message_filters.Subscriber(
            self, DetectionArray, 'vision/detect/list',
            qos_profile=sensor_qos,
        )
        self._lidar_pc_sub = message_filters.Subscriber(
            self, PointCloud2, 'lidar/scan/pointcloud',
            qos_profile=sensor_qos,
        )
        self._lidar_obs_sub = message_filters.Subscriber(
            self, ObstacleList, 'lidar/obstacle/list',
            qos_profile=sensor_qos,
        )

        tolerance = self.get_parameter('tolerance_window_ms').value / 1000.0
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._vision_sub, self._lidar_pc_sub, self._lidar_obs_sub],
            queue_size=10,
            slop=tolerance,
        )
        self._sync.registerCallback(self._synced_callback)

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._target_pub = self.create_publisher(
            Fusion3DTarget, 'fusion/target_3d', reliable_qos,
        )
        self._decision_pub = self.create_publisher(
            FusionDecision, 'fusion/obstacle/decision', reliable_qos,
        )
        self._status_pub = self.create_publisher(
            VisionStatus, 'fusion/status', 10,
        )

        # ------------------------------------------------------------------
        # Services
        # ------------------------------------------------------------------
        self._srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            GetStatus, 'fusion/get_status',
            self._handle_get_status,
            callback_group=self._srv_cb_group,
        )

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self._heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self._timeout_timer = self.create_timer(0.5, self._check_timeouts)

        # Heartbeat publisher
        self._heartbeat_pub = self.create_publisher(
            Heartbeat, 'system/heartbeat/fusion_engine', 10,
        )

        self.get_logger().info(
            f'fusion_engine started '
            f'(sync_tolerance={self.get_parameter("tolerance_window_ms").value}ms)'
        )

    # -----------------------------------------------------------------------
    # Parameter declaration
    # -----------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        p = self.declare_parameter

        # Synchronization
        p('tolerance_window_ms', 50)

        # Virtual camera intrinsics
        p('camera_fx', 600.0)
        p('camera_fy', 600.0)
        p('camera_cx', 640.0)
        p('camera_cy', 360.0)
        p('camera_width', 1280)
        p('camera_height', 720)

        # Physical extrinsic parameters
        p('camera_height_m', 0.5)
        p('T_camera_lidar_x', 0.1)
        p('T_camera_lidar_y', 0.0)
        p('T_camera_lidar_z', 0.3)

        # Depth / point cloud
        p('depth_range_min', 0.3)
        p('depth_range_max', 30.0)
        p('min_points_in_bbox', 5)

        # Visual-LiDAR association
        p('visual_lidar_assoc_max_dist', 1.0)
        p('visual_lidar_assoc_max_angle_deg', 10.0)

        # Thresholds
        p('large_trash_size_threshold', 0.3)
        p('water_area_detour_threshold', 0.5)
        p('detour_safety_margin', 0.5)
        p('pedestrian_default_wait_s', 5.0)
        p('pedestrian_rush_wait_s', 10.0)
        p('pedestrian_rush_velocity', 1.0)
        p('critical_distance', 3.0)

        # Timeouts
        p('vision_timeout_s', 2.0)
        p('vision_recovery_s', 3.0)

        # Output limits
        p('max_fusion_targets_per_frame', 10)

    # -----------------------------------------------------------------------
    # Synchronized callback
    # -----------------------------------------------------------------------
    def _synced_callback(
        self,
        vision_msg: DetectionArray,
        lidar_pc_msg: PointCloud2,
        obs_msg: ObstacleList,
    ) -> None:
        """Main fusion pipeline: called when vision + lidar PC + obstacles are
        approximately synchronized."""

        # Update timestamps
        now = time.monotonic()
        self._last_vision_time = now
        self._last_lidar_pc_time = now

        # If we were in degraded/pure-LiDAR mode, record recovery start
        if self._pure_lidar_mode:
            if self._vision_recovery_start is None:
                self._vision_recovery_start = now
            # Pure LiDAR mode: forward LiDAR-only decisions, skip fusion
            self._maybe_enter_pure_lidar_forward(obs_msg.obstacles)
            return

        # Normal mode: do the fusion
        self._process_fusion(vision_msg, obs_msg)

    # -----------------------------------------------------------------------
    # Fusion processing
    # -----------------------------------------------------------------------
    def _process_fusion(
        self,
        vision_msg: DetectionArray,
        obs_msg: ObstacleList,
    ) -> None:
        """Core fusion logic: match 2D detections to LiDAR obstacles, produce
        3D targets + decisions."""

        detections = vision_msg.detections
        obstacles = obs_msg.obstacles
        targets: List[Fusion3DTarget] = []
        decisions: List[FusionDecision] = []

        # Camera intrinsics
        fx = self.get_parameter('camera_fx').value
        fy = self.get_parameter('camera_fy').value
        cx = self.get_parameter('camera_cx').value
        cy = self.get_parameter('camera_cy').value
        cam_height = self.get_parameter('camera_height_m').value
        assoc_max_dist = self.get_parameter('visual_lidar_assoc_max_dist').value
        assoc_max_angle = math.radians(
            self.get_parameter('visual_lidar_assoc_max_angle_deg').value
        )

        large_thresh = self.get_parameter('large_trash_size_threshold').value
        water_thresh = self.get_parameter('water_area_detour_threshold').value
        safety_margin = self.get_parameter('detour_safety_margin').value

        used_obstacle_ids: set = set()

        for detection in detections:
            # ---------------------------------------------------------------
            # 2D -> angular sector estimate
            # ---------------------------------------------------------------
            bbox_cx_pix = (detection.x1 + detection.x2) / 2.0
            # Angular offset from principal point
            dx = bbox_cx_pix - cx
            ang_center = math.atan2(dx, fx)

            # ---------------------------------------------------------------
            # Find matching LiDAR obstacle
            # ---------------------------------------------------------------
            matched_obs: Optional[ObstacleItem] = None
            best_angle_diff = float('inf')

            for obs in obstacles:
                if obs.obstacle_id in used_obstacle_ids:
                    continue
                # Compute angular position of LiDAR obstacle relative to
                # camera principal axis
                obs_angle = math.atan2(
                    obs.position.y - self.get_parameter('T_camera_lidar_y').value,
                    obs.position.x - self.get_parameter('T_camera_lidar_x').value,
                )
                angle_diff = abs(obs_angle - ang_center)

                if angle_diff < assoc_max_angle and angle_diff < best_angle_diff:
                    # Extra check: distance consistency
                    match_dist = math.hypot(obs.position.x, obs.position.y)
                    est_dist = self._estimate_distance_from_area(
                        detection.pixel_area, cam_height
                    )
                    if abs(match_dist - est_dist) < assoc_max_dist:
                        best_angle_diff = angle_diff
                        matched_obs = obs

            # ---------------------------------------------------------------
            # Build 3D target
            # ---------------------------------------------------------------
            if matched_obs is not None:
                used_obstacle_ids.add(matched_obs.obstacle_id)
                position_3d = Point(
                    x=matched_obs.position.x,
                    y=matched_obs.position.y,
                    z=matched_obs.position.z,
                )
                dimensions = Vector3(
                    x=matched_obs.dimensions.x,
                    y=matched_obs.dimensions.y,
                    z=matched_obs.dimensions.z,
                )
                lidar_confidence = self._lidar_confidence_from_obs(matched_obs)
                fused_confidence = min(detection.confidence, lidar_confidence)
                lidar_matched = True
            else:
                # No LiDAR match -- estimate from pixel area
                est_z = self._estimate_distance_from_area(
                    detection.pixel_area, cam_height
                )
                position_3d = Point(x=est_z, y=0.0, z=0.0)
                dimensions = Vector3(x=0.1, y=0.1, z=0.1)
                fused_confidence = detection.confidence * 0.7
                lidar_matched = False

            # Alert level
            alert_level, alert_reason = self._determine_alert(
                detection, matched_obs,
            )

            target = Fusion3DTarget()
            target.header = Header()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = 'map'
            target.target_class = detection.class_name
            target.confidence = fused_confidence
            target.position_3d = position_3d
            target.dimensions = dimensions
            target.alert_level = alert_level
            target.alert_reason = alert_reason

            targets.append(target)

            # ---------------------------------------------------------------
            # Avoidance decision for this detection
            # ---------------------------------------------------------------
            decision = self._evaluate_decision(
                detection, matched_obs, lidar_matched
            )
            if decision is not None:
                decisions.append(decision)

        # -------------------------------------------------------------------
        # LiDAR-only obstacles (no vision match)
        # -------------------------------------------------------------------
        for obs in obstacles:
            if obs.obstacle_id in used_obstacle_ids:
                continue
            decision = self._evaluate_lidar_only_decision(obs)
            if decision is not None:
                decisions.append(decision)

        # -------------------------------------------------------------------
        # Publish targets (respect max per frame)
        # -------------------------------------------------------------------
        max_targets = self.get_parameter('max_fusion_targets_per_frame').value
        # Separate by alert level
        warn_crit_targets = [
            t for t in targets if t.alert_level >= ALERT_WARN
        ]
        info_targets = [t for t in targets if t.alert_level == ALERT_INFO]
        # Limit INFO targets
        limited_info = info_targets[:max_targets]
        limited_targets = warn_crit_targets + limited_info

        for target in limited_targets:
            self._target_pub.publish(target)

        if len(info_targets) > max_targets:
            self.get_logger().debug(
                f'Clipped INFO targets from {len(info_targets)} to '
                f'{max_targets}'
            )

        # -------------------------------------------------------------------
        # Merge decisions and publish the most conservative one
        # -------------------------------------------------------------------
        final_decision = self._merge_decisions(decisions, vision_msg.header)
        if final_decision is not None:
            self._decision_pub.publish(final_decision)

    # -----------------------------------------------------------------------
    # Distance estimation from pixel area (heuristic)
    # -----------------------------------------------------------------------
    @staticmethod
    def _estimate_distance_from_area(pixel_area: float, cam_height: float) -> float:
        """Rough depth estimate from bbox pixel area.

        Larger box -> closer object.  Clamp to a plausible range.
        """
        normalized = max(pixel_area / 10000.0, 0.001)
        estimated = cam_height / (normalized + 0.01)
        return float(max(0.3, min(estimated, 30.0)))

    # -----------------------------------------------------------------------
    # LiDAR confidence heuristic from point density
    # -----------------------------------------------------------------------
    @staticmethod
    def _lidar_confidence_from_obs(obs: ObstacleItem) -> float:
        """Estimate LiDAR detection confidence based on object size/class."""
        # Larger objects = more points = higher confidence
        size = obs.dimensions.x + obs.dimensions.y + obs.dimensions.z
        if size > 1.5:
            return 1.0
        if size > 0.5:
            return 0.9
        if size > 0.2:
            return 0.75
        if size > 0.1:
            return 0.6
        return 0.4

    # -----------------------------------------------------------------------
    # Alert level determination
    # -----------------------------------------------------------------------
    def _determine_alert(
        self,
        det: Detection,
        matched_obs: Optional[ObstacleItem],
    ) -> Tuple[int, str]:
        """Determine alert level for a fused target."""

        critical_dist = self.get_parameter('critical_distance').value

        if matched_obs is not None:
            dist = matched_obs.distance
            velocity = matched_obs.velocity
        else:
            dist = 999.0
            velocity = 0.0

        # CRITICAL: pedestrian/pet + fast approach + within critical distance
        if det.class_name in ('pedestrian_pet',) and velocity > 1.0 and dist < critical_dist:
            return (ALERT_CRITICAL,
                    f'pedestrian_pet fast approach v={velocity:.1f}m/s d={dist:.1f}m')

        # WARN: road_obstacle confirmed by LiDAR + within 5m
        if det.class_name == 'road_obstacle' and matched_obs is not None and dist < 5.0:
            return (ALERT_WARN,
                    f'road_obstacle lidar_confirmed d={dist:.1f}m')

        # WARN: large unknown object close by
        if matched_obs is not None and dist < 3.0 and matched_obs.classification >= OBS_CLASS_MEDIUM:
            return (ALERT_WARN,
                    f'large_object nearby d={dist:.1f}m cls={matched_obs.classification}')

        # Default: INFO
        return (ALERT_INFO, 'fused target')

    # -----------------------------------------------------------------------
    # Decision evaluation for vision-detected targets
    # -----------------------------------------------------------------------
    def _evaluate_decision(
        self,
        det: Detection,
        matched_obs: Optional[ObstacleItem],
        lidar_matched: bool,
    ) -> Optional[FusionDecision]:
        """Evaluate avoidance decision for a single detection."""

        decision = FusionDecision()

        if matched_obs is not None:
            obs_size = max(matched_obs.dimensions.x, matched_obs.dimensions.y,
                           matched_obs.dimensions.z)
            obs_dist = matched_obs.distance
            obs_velocity = matched_obs.velocity
        else:
            obs_size = 0.0
            obs_dist = 999.0
            obs_velocity = 0.0

        large_thresh = self.get_parameter('large_trash_size_threshold').value
        safety_margin = self.get_parameter('detour_safety_margin').value
        water_thresh = self.get_parameter('water_area_detour_threshold').value

        class_name = det.class_name

        # ---- road_obstacle ----
        if class_name == 'road_obstacle':
            if matched_obs is not None and (obs_size > 0.2 or obs_dist < 3.0):
                decision.action = DECISION_DETOUR
                decision.detour_offset = (obs_size / 2.0) + safety_margin
            else:
                decision.action = DECISION_SLOW
                decision.slow_ratio = 0.5
            return decision

        # ---- pedestrian_pet ----
        if class_name == 'pedestrian_pet':
            if matched_obs is not None:
                decision.action = DECISION_WAIT
                if obs_velocity > self.get_parameter('pedestrian_rush_velocity').value:
                    decision.wait_duration = self.get_parameter(
                        'pedestrian_rush_wait_s'
                    ).value
                else:
                    decision.wait_duration = self.get_parameter(
                        'pedestrian_default_wait_s'
                    ).value
                return decision
            # No LiDAR match but vision says pedestrian: wait anyway
            decision.action = DECISION_WAIT
            decision.wait_duration = self.get_parameter('pedestrian_default_wait_s').value
            return decision

        # ---- recyclable ----
        if class_name == 'recyclable':
            if matched_obs is not None and obs_size >= large_thresh:
                # Large cardboard box -- slow down, notify cleaning
                decision.action = DECISION_SLOW
                decision.slow_ratio = 0.3
            else:
                # Small recyclable: PASS (cleaning module handles)
                decision.action = DECISION_PASS
            return decision

        # ---- kitchen_waste, hazardous, other_waste, green_waste ----
        if class_name in ('kitchen_waste', 'hazardous', 'other_waste', 'green_waste'):
            decision.action = DECISION_PASS
            return decision

        # ---- stain ----
        if class_name == 'stain':
            if matched_obs is not None:
                projected_area = matched_obs.dimensions.x * matched_obs.dimensions.y
                if projected_area > water_thresh:
                    # Water puddle large enough to avoid
                    diameter = math.sqrt(projected_area)
                    decision.action = DECISION_DETOUR
                    decision.detour_offset = diameter + safety_margin
                else:
                    decision.action = DECISION_SLOW
                    decision.slow_ratio = 0.5
            else:
                # Stain with no LiDAR match: slow down
                decision.action = DECISION_SLOW
                decision.slow_ratio = 0.5
            return decision

        # Default: PASS unknown categories
        decision.action = DECISION_PASS
        return decision

    # -----------------------------------------------------------------------
    # LiDAR-only obstacle decision
    # -----------------------------------------------------------------------
    def _evaluate_lidar_only_decision(
        self, obs: ObstacleItem
    ) -> Optional[FusionDecision]:
        """Decision for LiDAR obstacles with no vision match."""

        decision = FusionDecision()
        decision.target_id = obs.obstacle_id

        obs_size = max(obs.dimensions.x, obs.dimensions.y, obs.dimensions.z)

        if obs_size <= 0.1:
            # Likely noise or small ground bump
            decision.action = DECISION_PASS
        else:
            # Conservative: detour around unknown obstacle
            decision.action = DECISION_DETOUR
            decision.detour_offset = (obs_size / 2.0) + self.get_parameter(
                'detour_safety_margin'
            ).value

        return decision

    # -----------------------------------------------------------------------
    # Decision merging
    # -----------------------------------------------------------------------
    def _merge_decisions(
        self,
        decisions: List[FusionDecision],
        header: Header,
    ) -> Optional[FusionDecision]:
        """Merge multiple decisions into one, keeping the most conservative.

        Priority: WAIT > DETOUR > SLOW > PASS
        """
        if not decisions:
            return None

        # Find the highest priority action
        best = max(decisions, key=lambda d: DECISION_PRIORITY.get(
            d.action, -1
        ))

        # Build merged decision
        merged = FusionDecision()
        merged.header = Header()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'map'
        merged.action = best.action
        merged.target_id = best.target_id
        merged.detour_offset = best.detour_offset
        merged.wait_duration = best.wait_duration
        merged.slow_ratio = best.slow_ratio

        return merged

    # -----------------------------------------------------------------------
    # Timeout handling
    # -----------------------------------------------------------------------
    def _check_timeouts(self) -> None:
        """Periodic check for sensor timeouts."""
        now = time.monotonic()
        vision_timeout = self.get_parameter('vision_timeout_s').value
        vision_recovery = self.get_parameter('vision_recovery_s').value

        vision_elapsed = now - self._last_vision_time if self._last_vision_time > 0 else 999.0
        lidar_elapsed = now - self._last_lidar_pc_time if self._last_lidar_pc_time > 0 else 999.0

        # Both timed out > 5s = FAULT
        if vision_elapsed > 5.0 and lidar_elapsed > 5.0:
            if not self._pure_lidar_mode:
                self.get_logger().warn(
                    'Both vision and LiDAR timed out >5s -- entering FAULT state'
                )
            self._pure_lidar_mode = False
            return

        # Vision timeout: enter pure LiDAR mode
        if vision_elapsed > vision_timeout and not self._pure_lidar_mode:
            self._pure_lidar_mode = True
            self._vision_recovery_start = None
            self.get_logger().warn(
                f'Vision timeout ({vision_elapsed:.1f}s): entering pure LiDAR mode'
            )

        # Vision recovery check
        if self._pure_lidar_mode and self._vision_recovery_start is not None:
            recovery_elapsed = now - self._vision_recovery_start
            if recovery_elapsed >= vision_recovery:
                self._pure_lidar_mode = False
                self._vision_recovery_start = None
                self.get_logger().info('Vision recovered: exiting pure LiDAR mode')

    # -----------------------------------------------------------------------
    # Pure LiDAR forward (called from sync callback when in pure_lidar_mode)
    #   Note: When the synchronizer still fires, but vision is stale,
    #   we check the flag and forward LiDAR-only decisions.
    # -----------------------------------------------------------------------
    def _maybe_enter_pure_lidar_forward(self, obstacles: List[ObstacleItem]) -> None:
        """If in pure-LiDAR mode, forward lidar obstacles as decisions."""
        decisions = []
        for obs in obstacles:
            d = FusionDecision()
            d.target_id = obs.obstacle_id
            obs_size = max(obs.dimensions.x, obs.dimensions.y, obs.dimensions.z)
            if obs.classification == OBS_CLASS_SMALL:
                d.action = DECISION_PASS
            else:
                # Medium/large: detour conservatively
                d.action = DECISION_DETOUR
                d.detour_offset = (obs_size / 2.0) + self.get_parameter(
                    'detour_safety_margin'
                ).value
            decisions.append(d)

        final = self._merge_decisions(decisions, Header())
        if final is not None:
            self._decision_pub.publish(final)

    # -----------------------------------------------------------------------
    # Status publisher (1Hz)
    # -----------------------------------------------------------------------
    def _publish_status(self) -> None:
        """Publish fusion/status at 1 Hz."""
        now = time.monotonic()

        status = VisionStatus()
        status.active_model = 'fusion_v1'
        status.fps = 0.0  # FPS tracking would require additional state
        status.avg_inference_ms = 0.0

        vision_timeout = self.get_parameter('vision_timeout_s').value
        vision_elapsed = now - self._last_vision_time if self._last_vision_time > 0 else 999.0
        lidar_elapsed = now - self._last_lidar_pc_time if self._last_lidar_pc_time > 0 else 999.0

        if vision_elapsed > 5.0 and lidar_elapsed > 5.0:
            status.status = STATUS_FAULT
            status.status_text = 'fault: both vision and lidar timed out'
        elif self._pure_lidar_mode:
            status.status = STATUS_DEGRADED
            status.status_text = 'vision_timeout: pure_lidar_mode'
        elif lidar_elapsed > self.get_parameter('vision_timeout_s').value:
            status.status = STATUS_DEGRADED
            status.status_text = 'lidar_timeout: vision_only_estimates'
        else:
            status.status = STATUS_NORMAL
            status.status_text = 'normal'

        self._status_pub.publish(status)

    # -----------------------------------------------------------------------
    # Heartbeat (1Hz)
    # -----------------------------------------------------------------------
    def _publish_heartbeat(self) -> None:
        """Publish system/heartbeat/fusion_engine at 1 Hz."""
        heartbeat = Heartbeat()
        heartbeat.header = Header()
        heartbeat.header.stamp = self.get_clock().now().to_msg()
        heartbeat.node_name = 'fusion_engine'
        heartbeat.lifecycle_state = LIFECYCLE_ACTIVE
        heartbeat.error_code = ERROR_NONE
        self._heartbeat_pub.publish(heartbeat)

    # -----------------------------------------------------------------------
    # GetStatus service
    # -----------------------------------------------------------------------
    def _handle_get_status(self, request, response):
        """Handle get_status service request."""
        now = time.monotonic()
        uptime = now - self._start_time

        response.node_name = 'fusion_engine'
        response.uptime_s = float(uptime)

        vision_elapsed = now - self._last_vision_time if self._last_vision_time > 0 else 999.0
        lidar_elapsed = now - self._last_lidar_pc_time if self._last_lidar_pc_time > 0 else 999.0

        if vision_elapsed > 5.0 and lidar_elapsed > 5.0:
            response.status = STATUS_FAULT
            response.status_text = 'fault: both vision and lidar timed out'
        elif self._pure_lidar_mode:
            response.status = STATUS_DEGRADED
            response.status_text = 'degraded: pure_lidar_mode'
        elif lidar_elapsed > self.get_parameter('vision_timeout_s').value:
            response.status = STATUS_DEGRADED
            response.status_text = 'degraded: lidar_timeout_vision_only'
        else:
            response.status = STATUS_NORMAL
            response.status_text = 'normal'

        return response


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusionEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
