from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="auto_drone_gazebo_topic_bridge",
                arguments=[
                    "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                    "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                ],
                output="screen",
            ),
            Node(
                package="auto_drone",
                executable="gazebo_lidar_bridge",
                name="auto_drone_gazebo_lidar_bridge",
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
                        "sensor_z_offset_meters": 0.98,
                        "log_every": 10,
                    }
                ],
                output="screen",
            ),
        ]
    )
