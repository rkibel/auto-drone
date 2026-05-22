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
            DeclareLaunchArgument("telemetry_topic", default_value="~/status"),
            DeclareLaunchArgument("marker_topic", default_value="~/markers"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("enable_visualization", default_value="true"),
            DeclareLaunchArgument("visualization_period_sec", default_value="1.0"),
            DeclareLaunchArgument("depth_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="1.5"),
            DeclareLaunchArgument("require_rgb_frame", default_value="false"),
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
                        "telemetry_topic": LaunchConfiguration("telemetry_topic"),
                        "marker_topic": LaunchConfiguration("marker_topic"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "enable_visualization": ParameterValue(
                            LaunchConfiguration("enable_visualization"), value_type=bool
                        ),
                        "visualization_period_sec": ParameterValue(
                            LaunchConfiguration("visualization_period_sec"), value_type=float
                        ),
                        "depth_timeout_sec": ParameterValue(
                            LaunchConfiguration("depth_timeout_sec"), value_type=float
                        ),
                        "takeoff_altitude_m": ParameterValue(
                            LaunchConfiguration("takeoff_altitude_m"), value_type=float
                        ),
                        "require_rgb_frame": ParameterValue(
                            LaunchConfiguration("require_rgb_frame"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
