from __future__ import annotations

from dataclasses import dataclass
from math import atan2

from auto_drone.autonomy3d import DiscoveryPlan, choose_discovery_plan
from auto_drone.core3d import BeliefVolume
from auto_drone.frames3d import enu_velocity_to_ned, px4_ned_pose_to_metric, yaw_enu_to_ned, yaw_rate_enu_to_ned
from auto_drone.interfaces3d import CameraIntrinsics, MetricPose, PoseSample, VoxelGridSpec, command_toward_pose
from auto_drone.mapping3d import integrate_range_frame
from auto_drone.pose_sources import Px4OdomPoseSource, RosSlamPoseSource
from auto_drone.rgbd_mapping import decode_depth_image, depth_image_to_range_frame


@dataclass
class AutonomyConfig:
    grid: VoxelGridSpec
    min_depth_m: float = 0.25
    max_depth_m: float = 8.0
    depth_stride: int = 8
    planning_period_sec: float = 1.0
    pose_timeout_sec: float = 0.5
    depth_timeout_sec: float = 0.5
    clearance_voxels: int = 1
    max_speed_mps: float = 1.2
    max_yaw_rate_rps: float = 0.8
    position_tolerance_m: float = 0.35


class ClosedLoopAutonomy:
    def __init__(self, config: AutonomyConfig):
        self.config = config
        self.belief = BeliefVolume(config.grid.width, config.grid.height, config.grid.depth)
        self.pose_sample: PoseSample | None = None
        self.intrinsics: CameraIntrinsics | None = None
        self.last_depth_stamp_sec = -1.0
        self.last_rgb_stamp_sec = -1.0
        self.last_plan_stamp_sec = -1.0
        self.plan = DiscoveryPlan(None, (), "not_planned")
        self.frame_count = 0

    def update_pose(self, sample: PoseSample) -> None:
        self.pose_sample = sample

    def update_camera_info(self, intrinsics: CameraIntrinsics) -> None:
        self.intrinsics = intrinsics

    def integrate_depth(self, depth_values: list[float], stamp_sec: float) -> int:
        if self.pose_sample is None or self.intrinsics is None:
            return 0
        frame = depth_image_to_range_frame(
            depth_values,
            self.intrinsics,
            self.pose_sample.pose,
            self.config.grid,
            self.config.min_depth_m,
            self.config.max_depth_m,
            self.config.depth_stride,
        )
        mapped = integrate_range_frame(self.belief, frame)
        self.last_depth_stamp_sec = stamp_sec
        self.frame_count += 1
        return len(mapped)

    def update_rgb_stamp(self, stamp_sec: float) -> None:
        self.last_rgb_stamp_sec = stamp_sec

    def ready(self, now_sec: float) -> tuple[bool, str | None]:
        if self.pose_sample is None:
            return False, "missing_pose"
        if self.intrinsics is None:
            return False, "missing_camera_info"
        if not self.pose_sample.is_fresh(now_sec, self.config.pose_timeout_sec):
            return False, "stale_pose"
        if now_sec - self.last_depth_stamp_sec > self.config.depth_timeout_sec:
            return False, "stale_depth"
        return True, None

    def update_plan_if_due(self, now_sec: float) -> DiscoveryPlan:
        if now_sec - self.last_plan_stamp_sec < self.config.planning_period_sec and self.plan.target is not None:
            return self.plan
        if self.pose_sample is None:
            self.plan = DiscoveryPlan(None, (), "missing_pose")
        else:
            self.plan = choose_discovery_plan(
                self.belief,
                self.config.grid,
                self.pose_sample.pose,
                self.config.clearance_voxels,
            )
        self.last_plan_stamp_sec = now_sec
        return self.plan

    def command(self):
        if self.pose_sample is None or self.plan.target is None:
            return None
        command_pose = self.plan.target
        if self.plan.path:
            next_voxel = self.plan.path[0]
            command_pose = self.config.grid.voxel_to_metric(*next_voxel, yaw=self.plan.target.yaw)
        return command_toward_pose(
            self.pose_sample.pose,
            command_pose,
            self.config.max_speed_mps,
            self.config.max_yaw_rate_rps,
            self.config.position_tolerance_m,
        )


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)


def main() -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image

    class Px4AutonomyNode(Node):
        def __init__(self):
            super().__init__("auto_drone_px4_autonomy")
            self.declare_parameter("depth_topic", "/camera/depth/image")
            self.declare_parameter("rgb_topic", "/camera/image")
            self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
            self.declare_parameter("px4_odom_topic", "/fmu/out/vehicle_odometry")
            self.declare_parameter("px4_status_topic", "/fmu/out/vehicle_status")
            self.declare_parameter("slam_pose_topic", "")
            self.declare_parameter("use_slam_pose", False)
            self.declare_parameter("width", 128)
            self.declare_parameter("height", 128)
            self.declare_parameter("depth", 32)
            self.declare_parameter("resolution", 0.25)
            self.declare_parameter("origin_x", -16.0)
            self.declare_parameter("origin_y", -16.0)
            self.declare_parameter("origin_z", 0.0)
            self.declare_parameter("min_depth_m", 0.25)
            self.declare_parameter("max_depth_m", 8.0)
            self.declare_parameter("depth_stride", 8)
            self.declare_parameter("planning_period_sec", 1.0)
            self.declare_parameter("pose_timeout_sec", 0.5)
            self.declare_parameter("depth_timeout_sec", 0.5)
            self.declare_parameter("clearance_voxels", 1)
            self.declare_parameter("max_speed_mps", 1.2)
            self.declare_parameter("max_yaw_rate_rps", 0.8)
            self.declare_parameter("position_tolerance_m", 0.35)
            self.declare_parameter("arm_and_offboard", True)
            self.declare_parameter("log_every", 10)

            grid = VoxelGridSpec(
                int(self.get_parameter("width").value),
                int(self.get_parameter("height").value),
                int(self.get_parameter("depth").value),
                float(self.get_parameter("resolution").value),
                float(self.get_parameter("origin_x").value),
                float(self.get_parameter("origin_y").value),
                float(self.get_parameter("origin_z").value),
            )
            config = AutonomyConfig(
                grid=grid,
                min_depth_m=float(self.get_parameter("min_depth_m").value),
                max_depth_m=float(self.get_parameter("max_depth_m").value),
                depth_stride=int(self.get_parameter("depth_stride").value),
                planning_period_sec=float(self.get_parameter("planning_period_sec").value),
                pose_timeout_sec=float(self.get_parameter("pose_timeout_sec").value),
                depth_timeout_sec=float(self.get_parameter("depth_timeout_sec").value),
                clearance_voxels=int(self.get_parameter("clearance_voxels").value),
                max_speed_mps=float(self.get_parameter("max_speed_mps").value),
                max_yaw_rate_rps=float(self.get_parameter("max_yaw_rate_rps").value),
                position_tolerance_m=float(self.get_parameter("position_tolerance_m").value),
            )
            self.autonomy = ClosedLoopAutonomy(config)
            self.log_every = int(self.get_parameter("log_every").value)
            self.use_slam_pose = bool(self.get_parameter("use_slam_pose").value)
            self.arm_and_offboard = bool(self.get_parameter("arm_and_offboard").value)
            self.setpoint_count = 0
            self.vehicle_status = None
            self.px4_pose_source = Px4OdomPoseSource()
            self.slam_pose_source = RosSlamPoseSource()

            self.create_subscription(Image, str(self.get_parameter("rgb_topic").value), self.on_rgb, 10)
            self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self.on_depth, 10)
            self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self.on_camera_info, 10)
            self.create_subscription(VehicleOdometry, str(self.get_parameter("px4_odom_topic").value), self.on_px4_odom, 10)
            self.create_subscription(VehicleStatus, str(self.get_parameter("px4_status_topic").value), self.on_vehicle_status, 10)
            slam_topic = str(self.get_parameter("slam_pose_topic").value)
            if slam_topic:
                self.create_subscription(PoseStamped, slam_topic, self.on_slam_pose, 10)

            self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
            self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
            self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
            self.create_timer(0.1, self.on_timer)
            self.get_logger().info("PX4 RGB-D autonomy node started")

        def stamp_sec(self, msg) -> float:
            return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

        def now_sec(self) -> float:
            now = self.get_clock().now().nanoseconds
            return now / 1e9

        def on_camera_info(self, msg: CameraInfo) -> None:
            self.autonomy.update_camera_info(CameraIntrinsics(msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5]))

        def on_depth(self, msg: Image) -> None:
            try:
                values = decode_depth_image(bytes(msg.data), msg.width, msg.height, msg.encoding, msg.step)
                self.autonomy.integrate_depth(values, self.stamp_sec(msg))
            except ValueError as exc:
                self.get_logger().warn(str(exc))

        def on_rgb(self, msg: Image) -> None:
            self.autonomy.update_rgb_stamp(self.stamp_sec(msg))

        def on_px4_odom(self, msg: VehicleOdometry) -> None:
            if self.use_slam_pose:
                return
            q = msg.q
            yaw_ned = yaw_from_quaternion(q[1], q[2], q[3], q[0])
            position = msg.position
            self.autonomy.update_pose(
                self.px4_pose_source.update(px4_ned_pose_to_metric(position[0], position[1], position[2], yaw_ned), self.now_sec())
            )

        def on_slam_pose(self, msg: PoseStamped) -> None:
            orientation = msg.pose.orientation
            position = msg.pose.position
            yaw = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            self.autonomy.update_pose(self.slam_pose_source.update(MetricPose(position.x, position.y, position.z, yaw=yaw), self.stamp_sec(msg)))

        def on_vehicle_status(self, msg: VehicleStatus) -> None:
            self.vehicle_status = msg

        def on_timer(self) -> None:
            now = self.now_sec()
            ready, reason = self.autonomy.ready(now)
            if not ready:
                if self.autonomy.frame_count % max(1, self.log_every) == 0:
                    self.get_logger().info(f"holding: {reason}")
                return
            plan = self.autonomy.update_plan_if_due(now)
            command = self.autonomy.command()
            if plan.target is None or command is None:
                self.get_logger().info(f"holding: {plan.stop_reason}")
                return

            mode = OffboardControlMode()
            mode.timestamp = int(now * 1_000_000)
            mode.position = False
            mode.velocity = True
            mode.acceleration = False
            mode.attitude = False
            mode.body_rate = False
            self.offboard_pub.publish(mode)

            setpoint = TrajectorySetpoint()
            setpoint.timestamp = mode.timestamp
            vx_ned, vy_ned, vz_ned = enu_velocity_to_ned(*command.velocity)
            setpoint.velocity = [float(vx_ned), float(vy_ned), float(vz_ned)]
            setpoint.yaw = float(yaw_enu_to_ned(command.pose.yaw))
            setpoint.yawspeed = float(yaw_rate_enu_to_ned(command.yaw_rate))
            self.setpoint_pub.publish(setpoint)
            self.setpoint_count += 1
            if self.arm_and_offboard and self.setpoint_count == 10:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.arm_and_offboard and self.setpoint_count == 50 and not self.vehicle_is_offboard_armed():
                self.get_logger().warn("PX4 has not reported armed offboard mode; check PX4 preflight and mode status")

            if self.autonomy.frame_count % max(1, self.log_every) == 0:
                self.get_logger().info(
                    f"frames={self.autonomy.frame_count} known={self.autonomy.belief.known_ratio():.3f} "
                    f"uncertainty={self.autonomy.belief.mean_uncertainty():.3f} "
                    f"target=({plan.target.x:.2f},{plan.target.y:.2f},{plan.target.z:.2f})"
                )

        def publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
            msg = VehicleCommand()
            msg.timestamp = int(self.now_sec() * 1_000_000)
            msg.param1 = float(param1)
            msg.param2 = float(param2)
            msg.command = command
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            self.vehicle_command_pub.publish(msg)

        def vehicle_is_offboard_armed(self) -> bool:
            if self.vehicle_status is None:
                return False
            armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            offboard = self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            return bool(armed and offboard)

    rclpy.init()
    node = Px4AutonomyNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
