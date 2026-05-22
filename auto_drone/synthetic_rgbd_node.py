"""Synthetic RGB-D publisher for simulator environments without render sensors."""

from __future__ import annotations

from math import sin
from struct import pack


def depth_pattern(width: int, height: int, frame_index: int, near_m: float, far_m: float) -> list[float]:
    values: list[float] = []
    span = max(0.1, far_m - near_m)
    for v in range(height):
        y = (v / max(1, height - 1)) - 0.5
        for u in range(width):
            x = (u / max(1, width - 1)) - 0.5
            ripple = 0.5 + 0.5 * sin(frame_index * 0.07 + x * 5.0 + y * 3.0)
            corridor = abs(x) * 0.9 + abs(y) * 0.35
            depth = near_m + span * min(1.0, 0.35 + corridor + 0.25 * ripple)
            values.append(min(far_m, max(near_m, depth)))
    return values


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image

    class SyntheticRgbdNode(Node):
        def __init__(self):
            super().__init__("auto_drone_synthetic_rgbd")
            self.declare_parameter("rgb_topic", "/camera/image")
            self.declare_parameter("depth_topic", "/camera/depth/image")
            self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
            self.declare_parameter("width", 80)
            self.declare_parameter("height", 60)
            self.declare_parameter("rate_hz", 4.0)
            self.declare_parameter("near_m", 0.5)
            self.declare_parameter("far_m", 8.0)
            self.width = int(self.get_parameter("width").value)
            self.height = int(self.get_parameter("height").value)
            self.near_m = float(self.get_parameter("near_m").value)
            self.far_m = float(self.get_parameter("far_m").value)
            self.fx = float(self.width) * 0.85
            self.fy = float(self.height) * 0.85
            self.cx = (self.width - 1) * 0.5
            self.cy = (self.height - 1) * 0.5
            self.frame_index = 0
            self.depth_pub = self.create_publisher(Image, str(self.get_parameter("depth_topic").value), 10)
            self.rgb_pub = self.create_publisher(Image, str(self.get_parameter("rgb_topic").value), 10)
            self.info_pub = self.create_publisher(CameraInfo, str(self.get_parameter("camera_info_topic").value), 10)
            period = 1.0 / max(0.1, float(self.get_parameter("rate_hz").value))
            self.create_timer(period, self.publish_frame)
            self.get_logger().info("Synthetic RGB-D publisher started")

        def publish_frame(self) -> None:
            stamp = self.get_clock().now().to_msg()
            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = "synthetic_depth_camera"
            info.width = self.width
            info.height = self.height
            info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
            info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_pub.publish(info)

            values = depth_pattern(self.width, self.height, self.frame_index, self.near_m, self.far_m)
            depth = Image()
            depth.header = info.header
            depth.height = self.height
            depth.width = self.width
            depth.encoding = "32FC1"
            depth.is_bigendian = False
            depth.step = self.width * 4
            depth.data = b"".join(pack("<f", value) for value in values)
            self.depth_pub.publish(depth)

            rgb = Image()
            rgb.header = info.header
            rgb.height = self.height
            rgb.width = self.width
            rgb.encoding = "rgb8"
            rgb.is_bigendian = False
            rgb.step = self.width * 3
            rgb.data = bytes((36, 88, 142)) * (self.width * self.height)
            self.rgb_pub.publish(rgb)
            self.frame_index += 1

    rclpy.init()
    node = SyntheticRgbdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
