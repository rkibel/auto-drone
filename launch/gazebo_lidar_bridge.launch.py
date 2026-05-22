from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="auto_drone",
                executable="gazebo_lidar_bridge",
                name="auto_drone_gazebo_lidar_bridge",
                output="screen",
                parameters=[
                    {
                        "scan_topic": "/scan",
                        "odom_topic": "/odom",
                        "width": 128,
                        "height": 128,
                        "depth": 16,
                        "resolution": 0.25,
                        "origin_x": 64,
                        "origin_y": 64,
                        "origin_z": 8,
                        "sensor_z_offset_meters": 0.0,
                        "log_every": 10,
                    }
                ],
            )
        ]
    )
