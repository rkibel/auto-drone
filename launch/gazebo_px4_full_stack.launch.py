from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("auto_drone")
    autonomy_launch = os.path.join(package_share, "launch", "gazebo_px4_autonomy.launch.py")

    px4_dir = LaunchConfiguration("px4_dir")
    px4_target = LaunchConfiguration("px4_target")
    px4_world = LaunchConfiguration("px4_gz_world")
    xrce_port = LaunchConfiguration("xrce_port")
    start_gz_server = LaunchConfiguration("start_gz_server")
    start_px4 = LaunchConfiguration("start_px4")
    start_xrce_agent = LaunchConfiguration("start_xrce_agent")
    start_camera_bridge = LaunchConfiguration("start_camera_bridge")
    standalone_gz = LaunchConfiguration("px4_gz_standalone")
    gz_ip = LaunchConfiguration("gz_ip")
    gz_partition = LaunchConfiguration("gz_partition")
    rgb_gz_topic = LaunchConfiguration("rgb_gz_topic")
    depth_gz_topic = LaunchConfiguration("depth_gz_topic")
    camera_info_gz_topic = LaunchConfiguration("camera_info_gz_topic")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    gz_env = [
        "export GZ_IP='",
        gz_ip,
        "'; export GZ_PARTITION='",
        gz_partition,
        "'; export PX4_GZ_WORLD='",
        px4_world,
        "'; export GZ_SIM_RESOURCE_PATH='",
        px4_dir,
        "/Tools/simulation/gz/models:",
        px4_dir,
        "/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}'; ",
    ]
    gz_server_cmd = [
        *gz_env,
        "exec gz sim -r -s '",
        px4_dir,
        "/Tools/simulation/gz/worlds/",
        px4_world,
        ".sdf'",
    ]
    px4_cmd = [
        *gz_env,
        "export PX4_GZ_STANDALONE='",
        standalone_gz,
        "'; cd '",
        px4_dir,
        "' && exec make px4_sitl '",
        px4_target,
        "'",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_dir", default_value=os.path.expanduser("~/PX4-Autopilot")),
            DeclareLaunchArgument("px4_target", default_value="gz_x500_depth"),
            DeclareLaunchArgument("px4_gz_world", default_value="default"),
            DeclareLaunchArgument("px4_gz_standalone", default_value="true"),
            DeclareLaunchArgument("start_gz_server", default_value="true"),
            DeclareLaunchArgument("start_px4", default_value="true"),
            DeclareLaunchArgument("start_xrce_agent", default_value="true"),
            DeclareLaunchArgument("start_camera_bridge", default_value="true"),
            DeclareLaunchArgument("gz_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("gz_partition", default_value="auto_drone_px4"),
            DeclareLaunchArgument("xrce_port", default_value="8888"),
            DeclareLaunchArgument(
                "rgb_gz_topic",
                default_value="/world/default/model/x500_depth_0/link/OakD-Lite/base_link/sensor/IMX214/image",
            ),
            DeclareLaunchArgument(
                "depth_gz_topic",
                default_value="/world/default/model/x500_depth_0/link/OakD-Lite/base_link/sensor/StereoOV7251/depth_image",
            ),
            DeclareLaunchArgument(
                "camera_info_gz_topic",
                default_value="/world/default/model/x500_depth_0/link/OakD-Lite/base_link/sensor/StereoOV7251/camera_info",
            ),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/image"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", xrce_port],
                output="screen",
                condition=IfCondition(start_xrce_agent),
            ),
            ExecuteProcess(
                cmd=["bash", "-lc", gz_server_cmd],
                output="screen",
                condition=IfCondition(start_gz_server),
            ),
            ExecuteProcess(
                cmd=["bash", "-lc", px4_cmd],
                output="screen",
                condition=IfCondition(start_px4),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="auto_drone_rgbd_bridge",
                output="screen",
                arguments=[
                    [rgb_gz_topic, "@sensor_msgs/msg/Image[gz.msgs.Image"],
                    [depth_gz_topic, "@sensor_msgs/msg/Image[gz.msgs.Image"],
                    [camera_info_gz_topic, "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
                    "--ros-args",
                    "-r",
                    [rgb_gz_topic, ":=", rgb_topic],
                    "-r",
                    [depth_gz_topic, ":=", depth_topic],
                    "-r",
                    [camera_info_gz_topic, ":=", camera_info_topic],
                ],
                condition=IfCondition(start_camera_bridge),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(autonomy_launch),
                launch_arguments={
                    "rgb_topic": rgb_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                }.items(),
            ),
        ]
    )
