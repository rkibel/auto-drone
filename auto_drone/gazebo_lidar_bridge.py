from __future__ import annotations

from math import atan2, cos, isfinite, sin

from auto_drone.core3d import BeliefVolume
from auto_drone.geometry3d import Pose3D, line_voxels
from auto_drone.mapping3d import integrate_range_frame
from auto_drone.sensing3d import RangeFrame, RangeMeasurement


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)


def pose_from_metric(
    x_meters: float,
    y_meters: float,
    z_meters: float,
    yaw: float,
    resolution: float,
    origin: tuple[int, int, int],
    sensor_z_offset_meters: float = 0.0,
) -> Pose3D:
    ox, oy, oz = origin
    return Pose3D(
        round(ox + x_meters / resolution),
        round(oy + y_meters / resolution),
        round(oz + (z_meters + sensor_z_offset_meters) / resolution),
        0.0,
        0.0,
        yaw,
    )


def scan_to_range_frame(
    ranges: list[float] | tuple[float, ...],
    angle_min: float,
    angle_increment: float,
    range_max: float,
    estimated_pose: Pose3D,
    resolution: float,
    max_range_voxels: int,
    residual: float = 0.05,
) -> RangeFrame:
    measurements = []
    for index, range_meters in enumerate(ranges):
        if not isfinite(range_meters) or range_meters <= 0.0:
            continue

        angle = angle_min + index * angle_increment
        clamped_range = min(range_meters, range_max)
        end_x = round(cos(angle) * clamped_range / resolution)
        end_y = round(sin(angle) * clamped_range / resolution)
        distance_voxels = min(max_range_voxels, max(1, round(clamped_range / resolution)))
        if end_x == 0 and end_y == 0:
            end_x = 1

        ray = tuple(line_voxels(0, 0, 0, end_x, end_y, 0)[1 : distance_voxels + 1])
        if not ray:
            continue

        hit = range_meters < range_max
        cells = ray if hit else ray[:distance_voxels]
        measurements.append(
            RangeMeasurement(
                ray=ray,
                cells=cells,
                hit=hit,
                hit_cell=cells[-1] if hit else None,
                range_voxels=min(len(cells), clamped_range / resolution),
                reprojection_error=residual,
            )
        )

    return RangeFrame(estimated_pose=estimated_pose, measurements=tuple(measurements))


class GazeboLidarMapper:
    def __init__(
        self,
        width: int,
        height: int,
        depth: int,
        resolution: float,
        origin: tuple[int, int, int],
        sensor_z_offset_meters: float = 0.0,
    ):
        self.belief = BeliefVolume(width, height, depth)
        self.resolution = resolution
        self.origin = origin
        self.sensor_z_offset_meters = sensor_z_offset_meters
        self.estimated_pose = Pose3D(*origin)
        self.frame_count = 0

    def update_pose(self, x_meters: float, y_meters: float, z_meters: float, yaw: float) -> Pose3D:
        self.estimated_pose = pose_from_metric(
            x_meters,
            y_meters,
            z_meters,
            yaw,
            self.resolution,
            self.origin,
            self.sensor_z_offset_meters,
        )
        return self.estimated_pose

    def integrate_scan(
        self,
        ranges: list[float] | tuple[float, ...],
        angle_min: float,
        angle_increment: float,
        range_max: float,
    ) -> set[tuple[int, int, int]]:
        frame = scan_to_range_frame(
            ranges,
            angle_min,
            angle_increment,
            range_max,
            self.estimated_pose,
            self.resolution,
            max(self.belief.width, self.belief.height),
        )
        mapped = integrate_range_frame(self.belief, frame)
        self.frame_count += 1
        return mapped


def main() -> None:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import LaserScan

    class GazeboLidarBridgeNode(Node):
        def __init__(self):
            super().__init__("auto_drone_gazebo_lidar_bridge")
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("width", 128)
            self.declare_parameter("height", 128)
            self.declare_parameter("depth", 16)
            self.declare_parameter("resolution", 0.25)
            self.declare_parameter("origin_x", 64)
            self.declare_parameter("origin_y", 64)
            self.declare_parameter("origin_z", 8)
            self.declare_parameter("sensor_z_offset_meters", 0.0)
            self.declare_parameter("log_every", 10)

            width = int(self.get_parameter("width").value)
            height = int(self.get_parameter("height").value)
            depth = int(self.get_parameter("depth").value)
            resolution = float(self.get_parameter("resolution").value)
            origin = (
                int(self.get_parameter("origin_x").value),
                int(self.get_parameter("origin_y").value),
                int(self.get_parameter("origin_z").value),
            )
            sensor_z_offset_meters = float(self.get_parameter("sensor_z_offset_meters").value)
            self.mapper = GazeboLidarMapper(width, height, depth, resolution, origin, sensor_z_offset_meters)
            self.log_every = int(self.get_parameter("log_every").value)

            scan_topic = str(self.get_parameter("scan_topic").value)
            odom_topic = str(self.get_parameter("odom_topic").value)
            self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
            self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
            self.get_logger().info(
                f"Mapping {scan_topic} + {odom_topic} into a {width}x{height}x{depth} belief volume"
            )

        def on_odom(self, msg: Odometry) -> None:
            orientation = msg.pose.pose.orientation
            position = msg.pose.pose.position
            yaw = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            self.mapper.update_pose(position.x, position.y, position.z, yaw)

        def on_scan(self, msg: LaserScan) -> None:
            self.mapper.integrate_scan(list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_max)
            if self.mapper.frame_count % self.log_every == 0:
                pose = self.mapper.estimated_pose
                self.get_logger().info(
                    f"frames={self.mapper.frame_count} known={self.mapper.belief.known_ratio():.3f} "
                    f"uncertainty={self.mapper.belief.mean_uncertainty():.3f} "
                    f"pose=({pose.x},{pose.y},{pose.z},{pose.yaw:.2f})"
                )

    rclpy.init()
    node = GazeboLidarBridgeNode()
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
