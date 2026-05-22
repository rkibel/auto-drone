from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("auto_drone")
    default_config = os.path.join(package_share, "config", "px4_autonomy.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/image"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            DeclareLaunchArgument("px4_odom_topic", default_value="/fmu/out/vehicle_odometry"),
            DeclareLaunchArgument("px4_status_topic", default_value="/fmu/out/vehicle_status"),
            DeclareLaunchArgument("slam_pose_topic", default_value=""),
            DeclareLaunchArgument("use_slam_pose", default_value="false"),
            Node(
                package="auto_drone",
                executable="px4_autonomy_node",
                name="auto_drone_px4_autonomy",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "rgb_topic": LaunchConfiguration("rgb_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "px4_odom_topic": LaunchConfiguration("px4_odom_topic"),
                        "px4_status_topic": LaunchConfiguration("px4_status_topic"),
                        "slam_pose_topic": LaunchConfiguration("slam_pose_topic"),
                        "use_slam_pose": ParameterValue(LaunchConfiguration("use_slam_pose"), value_type=bool),
                    },
                ],
            ),
        ]
    )
