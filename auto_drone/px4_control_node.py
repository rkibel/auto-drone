from __future__ import annotations

from collections import deque
from json import dumps
from math import asin, atan2, cos, sin, sqrt, tan
from time import perf_counter

import rclpy
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from auto_drone.interfaces import (
    CameraIntrinsics,
    MetricPose,
    VoxelGridSpec,
    enu_velocity_to_ned,
    px4_ned_pose_to_metric,
    yaw_enu_to_ned,
)
from auto_drone.coverage_planner import (
    DEFAULT_CLEARANCE_VOXELS,
    ExplorationBounds,
    MapCoverage,
    distance_squared,
    map_coverage,
    plan_frontier_route,
    trajectory_is_safe,
)
from auto_drone.mapping import (
    BeliefVolume,
    OCCUPIED_PROBABILITY_THRESHOLD,
    depth_image_to_range_frame,
    integrate_range_frame,
    rotate_local_offset_metric,
    stride_sample_offset,
    view_color_image,
    view_depth_image,
)
from auto_drone.rgbd import (
    DEPTH_STRIDE,
    MAX_MAPPING_DEPTH_M,
    camera_pose_from_body_pose,
    camera_pose_from_ground_truth_odometry,
    correct_ground_depths,
    exact_image_pairs,
    ground_truth_pose_from_odometry,
    image_timestamp_ns,
    is_ground_color,
    refine_camera_pose_from_ground,
)

STAGNATION_TIMEOUT_NS = 4_000_000_000
TARGET_ARRIVAL_DISTANCE_SQUARED = 4.0
ROUTE_WAYPOINT_ARRIVAL_DISTANCE_SQUARED = 1.0
INITIAL_SURVEY_ALTITUDE_M = 8.0
MAPPING_INTERVAL_NS = 90_000_000
IMAGE_BUFFER_SIZE = 12
GROUND_TRUTH_BUFFER_SIZE = 2_000
MAX_RVIZ_VOXELS = 16_000
MAX_DEPTH_POSE_TIME_DELTA_NS = 20_000_000
FALLBACK_VOXEL_COLOR = (0.15, 0.8, 0.2, 0.85)
MAX_EXPLORATION_SPEED_M_S = 1.0
MAX_EXPLORATION_ACCELERATION_M_S2 = 1.0
CONTROL_PERIOD_S = 0.05
EXPLORATION_HORIZONTAL_MARGIN_M = 3.0
EXPLORATION_VERTICAL_MARGIN_M = 1.0
SAFE_POSE_HISTORY_SIZE = 80
RETREAT_DISTANCE_M = 2.5


class Px4ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("auto_drone_px4_control")
        self.declare_parameter("enable_exploration", False)
        self.declare_parameter("force_arm", False)
        self.odometry: VehicleOdometry | None = None
        self.ground_truth_odometry: Odometry | None = None
        self.ground_truth_samples: deque[Odometry] = deque(maxlen=GROUND_TRUTH_BUFFER_SIZE)
        self.vehicle_status: VehicleStatus | None = None
        self.camera_info: CameraInfo | None = None
        self.depth_image_received = False
        self.color_image_received = False
        self.color_images: deque[Image] = deque(maxlen=IMAGE_BUFFER_SIZE)
        self.depth_images: deque[Image] = deque(maxlen=IMAGE_BUFFER_SIZE)
        self.grid = VoxelGridSpec(96, 112, 24, 0.5, -24.0, -16.0, 0.0)
        self.exploration_bounds = ExplorationBounds.from_grid(
            self.grid,
            margin_m=EXPLORATION_HORIZONTAL_MARGIN_M,
            vertical_margin_m=EXPLORATION_VERTICAL_MARGIN_M,
        )
        self.belief = BeliefVolume(self.grid.width, self.grid.height, self.grid.depth)
        self.last_mapping_ns = 0
        self.mapping_frame_count = 0
        self.last_mapped_cells = 0
        self.last_mapping_ms = 0.0
        self.color_matched_frame_count = 0
        self.color_skipped_depth_frame_count = 0
        self.occupied_voxel_count = 0
        self.elevated_ground_voxel_count = 0
        self.ground_pose_refined_frame_count = 0
        self.ground_pose_skipped_frame_count = 0
        self.last_ground_pose_correction_m = 0.0
        self.exploration_target: MetricPose | None = None
        self.exploration_route: deque[MetricPose] = deque()
        self.recent_targets: deque[MetricPose] = deque(maxlen=6)
        self.initial_climb_complete = False
        self.last_plan_ns = 0
        self.last_plan_ms = 0.0
        self.last_frontier_count = 0
        self.last_coverage_progress_ns = 0
        self.coverage = MapCoverage(0.0, 0.0)
        self.best_coverage_score = 0.0
        self.stalled_targets: list[MetricPose] = []
        self.unsafe_route_replan_count = 0
        self.completed_viewpoint_count = 0
        self.last_route_validation_mapping_frame = -1
        self.last_safe_pose_mapping_frame = -1
        self.safe_pose_history: deque[MetricPose] = deque(maxlen=SAFE_POSE_HISTORY_SIZE)
        self.retreat_target: MetricPose | None = None
        self.retreat_count = 0
        self.current_pose_clear = True
        self.offboard_request_count = 0
        self.setpoint_count = 0
        self.commanded_velocity = (0.0, 0.0, 0.0)
        px4_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odometry, px4_qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", self.on_vehicle_status, px4_qos)
        self.create_subscription(
            Odometry,
            "/model/x500_depth_0/odometry",
            self.on_ground_truth_odometry,
            image_qos,
        )
        self.create_subscription(CameraInfo, "/camera_info", self.on_camera_info, image_qos)
        self.create_subscription(Image, "/depth_camera", self.on_depth_image, image_qos)
        self.create_subscription(
            Image,
            "/world/forest/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
            self.on_color_image,
            image_qos,
        )
        self.mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", px4_qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos)
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_qos)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "~/occupied_voxels", 10)
        self.drone_pub = self.create_publisher(Marker, "~/drone", 10)
        self.camera_frustum_pub = self.create_publisher(Marker, "~/camera_frustum", 10)
        self.transform_broadcaster = StaticTransformBroadcaster(self)
        self.occupancy_transform = TransformStamped()
        self.occupancy_transform.header.frame_id = "map"
        self.occupancy_transform.child_frame_id = "occupancy_map"
        self.occupancy_transform.transform.rotation.w = 1.0
        self.create_timer(0.05, self.publish_hold)
        self.create_timer(0.05, self.publish_flight_markers)
        self.create_timer(0.25, self.publish_occupied_voxels)
        self.create_timer(1.0, self.publish_status)

    def on_odometry(self, message: VehicleOdometry) -> None:
        self.odometry = message

    def on_vehicle_status(self, message: VehicleStatus) -> None:
        self.vehicle_status = message

    def on_ground_truth_odometry(self, message: Odometry) -> None:
        self.ground_truth_odometry = message
        self.ground_truth_samples.append(message)
        self.map_synchronized_image()

    def on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def on_color_image(self, message: Image) -> None:
        self.color_image_received = True
        self.color_images.append(message)
        self.map_synchronized_image()

    def on_depth_image(self, message: Image) -> None:
        self.depth_image_received = True
        if len(self.depth_images) == self.depth_images.maxlen:
            self.color_skipped_depth_frame_count += 1
        self.depth_images.append(message)
        self.map_synchronized_image()

    def map_synchronized_image(self) -> None:
        if not self.depth_images or not self.color_images or self.camera_info is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_mapping_ns < MAPPING_INTERVAL_NS:
            return
        pair = self.newest_synchronized_image_pair()
        if pair is None:
            return
        message, color_image, pose = pair
        self.last_mapping_ns = now_ns
        selected_stamp = image_timestamp_ns(message)
        skipped_depths = sum(image_timestamp_ns(image) < selected_stamp for image in self.depth_images)
        self.color_skipped_depth_frame_count += skipped_depths
        while self.depth_images and image_timestamp_ns(self.depth_images[0]) <= selected_stamp:
            self.depth_images.popleft()
        while self.color_images and image_timestamp_ns(self.color_images[0]) <= selected_stamp:
            self.color_images.popleft()
        started = perf_counter()
        intrinsics = CameraIntrinsics(
            message.width,
            message.height,
            self.camera_info.k[0],
            self.camera_info.k[4],
            self.camera_info.k[2],
            self.camera_info.k[5],
        )
        depth_values = view_depth_image(
            message.data,
            message.width,
            message.height,
            message.encoding,
            message.step,
        )
        try:
            colors = view_color_image(
                color_image.data,
                color_image.width,
                color_image.height,
                color_image.encoding,
                color_image.step,
            )
        except ValueError as error:
            self.get_logger().warning(f"Ignoring color frame: {error}", throttle_duration_sec=5.0)
            return
        refined_pose = refine_camera_pose_from_ground(depth_values, colors, intrinsics, pose)
        if refined_pose is None:
            self.ground_pose_skipped_frame_count += 1
        else:
            self.last_ground_pose_correction_m = pose.z - refined_pose.z
            self.ground_pose_refined_frame_count += 1
            pose = refined_pose
        sample_offset = stride_sample_offset(self.mapping_frame_count, DEPTH_STRIDE)
        depth_values = correct_ground_depths(
            depth_values,
            colors,
            intrinsics,
            pose,
            sample_offset=sample_offset,
        )
        frame = depth_image_to_range_frame(
            depth_values,
            intrinsics,
            pose,
            self.grid,
            0.25,
            MAX_MAPPING_DEPTH_M,
            stride=DEPTH_STRIDE,
            colors=colors,
            sample_offset=sample_offset,
        )
        self.last_mapped_cells = len(integrate_range_frame(self.belief, frame))
        self.mapping_frame_count += 1
        self.color_matched_frame_count += 1
        self.last_mapping_ms = (perf_counter() - started) * 1_000.0

    def newest_synchronized_image_pair(self) -> tuple[Image, Image, MetricPose] | None:
        for depth_image, color_image in exact_image_pairs(self.depth_images, self.color_images):
            pose = self.matching_ground_truth_camera_pose(depth_image)
            if pose is not None:
                return depth_image, color_image, pose
        return None

    def matching_ground_truth_camera_pose(self, depth_image: Image) -> MetricPose | None:
        if not self.ground_truth_samples:
            return None
        depth_stamp = image_timestamp_ns(depth_image)
        odometry = min(
            self.ground_truth_samples,
            key=lambda sample: abs(image_timestamp_ns(sample) - depth_stamp),
        )
        if abs(image_timestamp_ns(odometry) - depth_stamp) > MAX_DEPTH_POSE_TIME_DELTA_NS:
            return None
        return camera_pose_from_ground_truth_odometry(odometry)

    def publish_hold(self) -> None:
        timestamp = self.px4_timestamp_us()
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.velocity = True
        self.mode_pub.publish(mode)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = [float("nan"), float("nan"), float("nan")]
        setpoint.velocity = list(self.exploration_velocity())
        setpoint.acceleration = [float("nan"), float("nan"), float("nan")]
        setpoint.yaw = self.exploration_yaw()
        self.setpoint_pub.publish(setpoint)
        self.setpoint_count += 1

    def exploration_enabled(self) -> bool:
        return bool(self.get_parameter("enable_exploration").value)

    def exploration_velocity(self) -> tuple[float, float, float]:
        if not self.exploration_enabled() or self.odometry is None:
            self.commanded_velocity = (0.0, 0.0, 0.0)
            return (0.0, 0.0, 0.0)
        if (
            self.setpoint_count >= 20
            and self.setpoint_count % 20 == 0
            and (
                self.vehicle_status is None
                or self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED
                or not self.vehicle_status.accepts_offboard_setpoints
            )
        ):
            self.request_offboard()
        pose = metric_pose_from_odometry(self.odometry)
        target = self.next_target(pose)
        if target is None:
            return self.rate_limited_velocity((0.0, 0.0, 0.0))
        dx = target.x - pose.x
        dy = target.y - pose.y
        dz = target.z - pose.z
        distance = (dx * dx + dy * dy + dz * dz) ** 0.5
        if distance < 0.01:
            return self.rate_limited_velocity((0.0, 0.0, 0.0))
        speed = min(MAX_EXPLORATION_SPEED_M_S, distance)
        return self.rate_limited_velocity(enu_velocity_to_ned(dx * speed / distance, dy * speed / distance, dz * speed / distance))

    def rate_limited_velocity(self, target_velocity: tuple[float, float, float]) -> tuple[float, float, float]:
        self.commanded_velocity = rate_limited_velocity(
            self.commanded_velocity,
            target_velocity,
            MAX_EXPLORATION_ACCELERATION_M_S2 * CONTROL_PERIOD_S,
        )
        return self.commanded_velocity

    def exploration_yaw(self) -> float:
        if not self.exploration_enabled() or self.odometry is None:
            return float("nan")
        pose = metric_pose_from_odometry(self.odometry)
        target = self.exploration_target
        if target is None or distance_squared(pose, target) <= 0.25:
            return float("nan")
        return yaw_enu_to_ned(atan2(target.y - pose.y, target.x - pose.x))

    def next_target(self, pose: MetricPose) -> MetricPose | None:
        bounds = self.exploration_bounds
        if not bounds.contains(pose):
            self.exploration_route.clear()
            self.exploration_target = bounds.clamp(pose)
            return self.exploration_target
        if self.mapping_frame_count != self.last_safe_pose_mapping_frame:
            self.last_safe_pose_mapping_frame = self.mapping_frame_count
            self.current_pose_clear = trajectory_is_safe(
                self.belief,
                self.grid,
                pose,
                pose,
                bounds,
                DEFAULT_CLEARANCE_VOXELS,
            )
            if self.current_pose_clear:
                self.safe_pose_history.append(pose)
        if not self.initial_climb_complete:
            climb_altitude = min(max(INITIAL_SURVEY_ALTITUDE_M, bounds.min_z), bounds.max_z)
            if abs(pose.z - climb_altitude) > 0.25:
                self.exploration_target = MetricPose(pose.x, pose.y, climb_altitude)
                return self.exploration_target
            self.initial_climb_complete = True
            self.exploration_target = None
        if self.retreat_target is not None:
            if distance_squared(pose, self.retreat_target) > ROUTE_WAYPOINT_ARRIVAL_DISTANCE_SQUARED:
                self.exploration_target = self.retreat_target
                return self.exploration_target
            self.retreat_target = None
            self.exploration_target = None
            self.last_plan_ns = 0
        now_ns = self.get_clock().now().nanoseconds
        if self.exploration_route and self.mapping_frame_count != self.last_route_validation_mapping_frame:
            self.last_route_validation_mapping_frame = self.mapping_frame_count
            if not trajectory_is_safe(
                self.belief,
                self.grid,
                pose,
                self.exploration_route[0],
                bounds,
                DEFAULT_CLEARANCE_VOXELS,
            ):
                self.exploration_route.clear()
                self.exploration_target = None
                self.last_plan_ns = 0
                self.unsafe_route_replan_count += 1
                if not self.current_pose_clear:
                    candidates = tuple(
                        candidate
                        for candidate in self.safe_pose_history
                        if trajectory_is_safe(
                            self.belief,
                            self.grid,
                            candidate,
                            candidate,
                            bounds,
                            DEFAULT_CLEARANCE_VOXELS,
                        )
                    )
                    self.retreat_target = select_backtrack_target(pose, candidates, RETREAT_DISTANCE_M)
                    if self.retreat_target is not None:
                        self.retreat_count += 1
                        self.exploration_target = self.retreat_target
                    return self.exploration_target
        route_completed = False
        while (
            self.exploration_route
            and distance_squared(pose, self.exploration_route[0]) <= ROUTE_WAYPOINT_ARRIVAL_DISTANCE_SQUARED
        ):
            if len(self.exploration_route) == 1:
                self.recent_targets.append(self.exploration_route[0])
                self.completed_viewpoint_count += 1
            self.exploration_route.popleft()
            route_completed = True
        if route_completed:
            self.exploration_target = None
            self.last_plan_ns = 0
        if self.exploration_route:
            self.exploration_target = self.exploration_route[0]
            if (
                len(self.exploration_route) == 1
                and distance_squared(pose, self.exploration_target) <= TARGET_ARRIVAL_DISTANCE_SQUARED
                and now_ns - self.last_coverage_progress_ns >= STAGNATION_TIMEOUT_NS
            ):
                if self.exploration_target not in self.stalled_targets:
                    self.stalled_targets.append(self.exploration_target)
                self.exploration_route.clear()
                self.exploration_target = None
            else:
                return self.exploration_target
        if now_ns - self.last_plan_ns < 1_000_000_000:
            return None
        self.coverage = map_coverage(self.belief)
        if self.last_coverage_progress_ns == 0 or self.coverage.score > self.best_coverage_score + 1e-4:
            self.best_coverage_score = self.coverage.score
            self.last_coverage_progress_ns = now_ns
            self.stalled_targets.clear()
        started = perf_counter()
        excluded_targets = tuple((*self.stalled_targets, *self.recent_targets))
        plan = plan_frontier_route(
            self.belief,
            self.grid,
            pose,
            bounds,
            excluded_targets=excluded_targets,
        )
        self.last_plan_ms = (perf_counter() - started) * 1_000.0
        self.last_frontier_count = plan.frontier_count
        self.last_plan_ns = now_ns
        if plan.waypoints:
            self.exploration_route = deque(plan.waypoints)
            self.exploration_target = self.exploration_route[0]
        return self.exploration_target

    def request_offboard(self) -> None:
        command = VehicleCommand()
        command.timestamp = self.px4_timestamp_us()
        command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        command.param1 = 1.0
        command.param2 = 6.0
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.from_external = True
        self.command_pub.publish(command)
        command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command.param1 = 1.0
        if bool(self.get_parameter("force_arm").value):
            command.param2 = 21196.0
            command.from_external = False
        self.command_pub.publish(command)
        self.offboard_request_count += 1

    def px4_timestamp_us(self) -> int:
        if self.odometry is not None:
            return self.odometry.timestamp
        return self.get_clock().now().nanoseconds // 1_000

    def publish_status(self) -> None:
        self.coverage = map_coverage(self.belief)
        status = String()
        status.data = dumps(
            {
                "camera_info_received": self.camera_info is not None,
                "depth_image_received": self.depth_image_received,
                "color_image_received": self.color_image_received,
                "color_matched_frame_count": self.color_matched_frame_count,
                "color_skipped_depth_frame_count": self.color_skipped_depth_frame_count,
                "buffered_depth_frame_count": len(self.depth_images),
                "buffered_color_frame_count": len(self.color_images),
                "known_ratio": round(self.coverage.known_ratio, 5),
                "floor_occupied_ratio": round(self.coverage.floor_occupied_ratio, 5),
                "last_mapped_cells": self.last_mapped_cells,
                "last_mapping_ms": round(self.last_mapping_ms, 2),
                "mapping_frame_count": self.mapping_frame_count,
                "odometry_received": self.odometry is not None,
                "ground_truth_odometry_received": self.ground_truth_odometry is not None,
                "ground_pose_refined_frame_count": self.ground_pose_refined_frame_count,
                "ground_pose_skipped_frame_count": self.ground_pose_skipped_frame_count,
                "last_ground_pose_correction_m": round(self.last_ground_pose_correction_m, 3),
                "armed": self.vehicle_status is not None
                and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                "occupied_voxel_count": self.occupied_voxel_count,
                "elevated_ground_voxel_count": self.elevated_ground_voxel_count,
                "target": target_payload(self.exploration_target),
                "frontier_count": self.last_frontier_count,
                "route_waypoint_count": len(self.exploration_route),
                "last_plan_ms": round(self.last_plan_ms, 2),
                "stalled_target_count": len(self.stalled_targets),
                "unsafe_route_replan_count": self.unsafe_route_replan_count,
                "completed_viewpoint_count": self.completed_viewpoint_count,
                "retreat_count": self.retreat_count,
                "retreat_target": target_payload(self.retreat_target),
                "current_pose_clear": self.current_pose_clear,
                "exploration_enabled": self.exploration_enabled(),
                "force_arm": bool(self.get_parameter("force_arm").value),
                "offboard_request_count": self.offboard_request_count,
                "setpoint_count": self.setpoint_count,
            }
        )
        self.status_pub.publish(status)

    def publish_occupied_voxels(self) -> None:
        self.transform_broadcaster.sendTransform(self.occupancy_transform)
        marker = Marker()
        marker.header.frame_id = "occupancy_map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tree_occupancy"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.grid.resolution
        marker.scale.y = self.grid.resolution
        marker.scale.z = self.grid.resolution
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = FALLBACK_VOXEL_COLOR
        self.elevated_ground_voxel_count = 0
        for z in range(self.belief.depth):
            for y in range(self.belief.height):
                for x in range(self.belief.width):
                    if self.belief.occupancy_probability(x, y, z) <= OCCUPIED_PROBABILITY_THRESHOLD:
                        continue
                    pose = self.grid.voxel_to_metric(x, y, z)
                    color = self.belief.colors.get(self.belief.index(x, y, z))
                    red, green, blue = color[:3] if color is not None else (38, 204, 51)
                    if z > 0 and color is not None and is_ground_color((red, green, blue)):
                        self.elevated_ground_voxel_count += 1
                    marker.points.append(Point(x=pose.x, y=pose.y, z=pose.z))
                    marker.colors.append(
                        ColorRGBA(r=red / 255.0, g=green / 255.0, b=blue / 255.0, a=0.85)
                    )
                    if len(marker.points) == MAX_RVIZ_VOXELS:
                        self.occupied_voxel_count = len(marker.points)
                        self.marker_pub.publish(MarkerArray(markers=[marker]))
                        return
        self.occupied_voxel_count = len(marker.points)
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_flight_markers(self) -> None:
        if self.ground_truth_odometry is not None:
            body_pose = ground_truth_pose_from_odometry(self.ground_truth_odometry)
            camera_pose = camera_pose_from_body_pose(body_pose)
        elif self.odometry is not None:
            body_pose = metric_pose_from_odometry(self.odometry)
            camera_pose = camera_pose_from_body_pose(body_pose)
        else:
            return
        drone = Marker()
        drone.header.frame_id = "map"
        drone.header.stamp = self.get_clock().now().to_msg()
        drone.ns = "drone"
        drone.id = 0
        drone.type = Marker.MESH_RESOURCE
        drone.action = Marker.ADD
        drone.mesh_resource = "file:///Users/ronki/Documents/GitHub/auto-drone/vendor/PX4-Autopilot/Tools/simulation/gz/models/x500_base/meshes/NXP-HGD-CF.dae"
        drone.mesh_use_embedded_materials = True
        drone.pose.position = Point(x=body_pose.x, y=body_pose.y, z=body_pose.z)
        drone.pose.orientation = quaternion_from_rpy(body_pose.roll, body_pose.pitch, body_pose.yaw)
        drone.scale.x = drone.scale.y = drone.scale.z = 1.0
        drone.color.a = 1.0
        self.drone_pub.publish(drone)

        frustum = Marker()
        frustum.header.frame_id = "map"
        frustum.header.stamp = drone.header.stamp
        frustum.ns = "camera_frustum"
        frustum.id = 0
        frustum.type = Marker.LINE_LIST
        frustum.action = Marker.ADD
        frustum.scale.x = 0.03
        frustum.color.r = 0.25
        frustum.color.g = 0.9
        frustum.color.b = 1.0
        frustum.color.a = 0.9
        frustum.points = camera_frustum_points(camera_pose)
        self.camera_frustum_pub.publish(frustum)


def main() -> None:
    rclpy.init()
    node = Px4ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


def metric_pose_from_odometry(message: VehicleOdometry) -> MetricPose:
    roll, pitch, yaw = attitude_enu_from_px4_quaternion(message.q)
    position = px4_ned_pose_to_metric(*message.position, 0.0)
    return MetricPose(float(position.x), float(position.y), float(position.z), roll, pitch, yaw)


def attitude_enu_from_px4_quaternion(q: list[float]) -> tuple[float, float, float]:
    w, x, y, z = q
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (x * y + z * w)
    r20 = 2.0 * (x * z - y * w)
    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return atan2(r21, r22), asin(r20), atan2(r00, r10)

def target_payload(target: MetricPose | None) -> list[float] | None:
    if target is None:
        return None
    return [round(target.x, 2), round(target.y, 2), round(target.z, 2)]


def select_backtrack_target(
    current: MetricPose,
    candidates: tuple[MetricPose, ...],
    minimum_distance_m: float,
) -> MetricPose | None:
    minimum_distance_squared = minimum_distance_m * minimum_distance_m
    for candidate in reversed(candidates):
        if distance_squared(current, candidate) >= minimum_distance_squared:
            return candidate
    return None


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    half_yaw = yaw / 2.0
    return Quaternion(
        x=sin(half_roll) * cos(half_pitch) * cos(half_yaw) - cos(half_roll) * sin(half_pitch) * sin(half_yaw),
        y=cos(half_roll) * sin(half_pitch) * cos(half_yaw) + sin(half_roll) * cos(half_pitch) * sin(half_yaw),
        z=cos(half_roll) * cos(half_pitch) * sin(half_yaw) - sin(half_roll) * sin(half_pitch) * cos(half_yaw),
        w=cos(half_roll) * cos(half_pitch) * cos(half_yaw) + sin(half_roll) * sin(half_pitch) * sin(half_yaw),
    )


def camera_frustum_points(pose: MetricPose) -> list[Point]:
    distance = 3.0
    half_width = distance * tan(1.274 / 2.0)
    half_height = half_width * 480.0 / 640.0
    origin = Point(x=float(pose.x), y=float(pose.y), z=float(pose.z))
    corners = [
        frustum_point(pose, distance, y, z)
        for y, z in ((-half_width, -half_height), (-half_width, half_height), (half_width, half_height), (half_width, -half_height))
    ]
    points = []
    for corner in corners:
        points.extend((origin, corner))
    for index, corner in enumerate(corners):
        points.extend((corner, corners[(index + 1) % len(corners)]))
    return points


def frustum_point(pose: MetricPose, x: float, y: float, z: float) -> Point:
    offset_x, offset_y, offset_z = rotate_local_offset_metric(x, y, z, pose.roll, pose.pitch, pose.yaw)
    return Point(x=float(pose.x + offset_x), y=float(pose.y + offset_y), z=float(pose.z + offset_z))


def rate_limited_velocity(
    current_velocity: tuple[float, float, float],
    target_velocity: tuple[float, float, float],
    maximum_change: float,
) -> tuple[float, float, float]:
    change = tuple(target - current for current, target in zip(current_velocity, target_velocity))
    magnitude = sqrt(sum(component * component for component in change))
    if magnitude <= maximum_change:
        return target_velocity
    scale = maximum_change / magnitude
    return tuple(current + component * scale for current, component in zip(current_velocity, change))
