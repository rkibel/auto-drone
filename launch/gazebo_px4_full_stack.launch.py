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
    rviz_config = os.path.join(package_share, "config", "px4_autonomy.rviz")
    default_bridge_config = os.path.join(package_share, "config", "gz_rgbd_bridge.yaml")

    px4_dir = LaunchConfiguration("px4_dir")
    px4_target = LaunchConfiguration("px4_target")
    px4_world = LaunchConfiguration("px4_gz_world")
    gz_world_file = LaunchConfiguration("gz_world_file")
    xrce_port = LaunchConfiguration("xrce_port")
    start_gz_server = LaunchConfiguration("start_gz_server")
    start_gz_client = LaunchConfiguration("start_gz_client")
    start_px4 = LaunchConfiguration("start_px4")
    start_xrce_agent = LaunchConfiguration("start_xrce_agent")
    start_camera_bridge = LaunchConfiguration("start_camera_bridge")
    start_synthetic_rgbd = LaunchConfiguration("start_synthetic_rgbd")
    start_rviz = LaunchConfiguration("start_rviz")
    standalone_gz = LaunchConfiguration("px4_gz_standalone")
    gz_ip = LaunchConfiguration("gz_ip")
    gz_partition = LaunchConfiguration("gz_partition")
    gz_render_engine = LaunchConfiguration("gz_render_engine")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    bridge_config = LaunchConfiguration("bridge_config")
    marker_topic = LaunchConfiguration("marker_topic")
    map_frame = LaunchConfiguration("map_frame")
    visualization_period_sec = LaunchConfiguration("visualization_period_sec")
    depth_timeout_sec = LaunchConfiguration("depth_timeout_sec")

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
        "/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}'; "
        "if [ -n \"${GZ_PREFIX:-}\" ]; then "
        "export GZ_RENDERING_PLUGIN_PATH=\"$GZ_PREFIX/lib/x86_64-linux-gnu/gz-rendering-8/engine-plugins:"
        "$GZ_PREFIX/lib/x86_64-linux-gnu/gz-rendering-7/engine-plugins:${GZ_RENDERING_PLUGIN_PATH:-}\"; "
        "export GZ_RENDERING_RESOURCE_PATH=\"$GZ_PREFIX/share/gz/gz-rendering8\"; "
        "export OGRE_RESOURCE_PATH=\"$GZ_PREFIX/share/gz/gz-rendering8/ogre/media:"
        "$GZ_PREFIX/share/gz/gz-rendering7/ogre/media:${OGRE_RESOURCE_PATH:-}\"; "
        "export OGRE_PLUGIN_DIR=\"$GZ_PREFIX/lib/x86_64-linux-gnu/OGRE-2.3/OGRE\"; "
        "export GZ_GUI_PLUGIN_PATH=\"$GZ_PREFIX/lib/x86_64-linux-gnu/gz-gui-8/plugins:"
        "$GZ_PREFIX/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui:${GZ_GUI_PLUGIN_PATH:-}\"; "
        "export GZ_SIM_GUI_PLUGIN_PATH=\"$GZ_PREFIX/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui:"
        "${GZ_SIM_GUI_PLUGIN_PATH:-}\"; fi; ",
    ]
    gz_server_cmd = [
        *gz_env,
        "render_args=(); if [ -n '",
        gz_render_engine,
        "' ]; then render_args=(--render-engine '",
        gz_render_engine,
        "'); fi; world_file='",
        gz_world_file,
        "'; if [ -z \"$world_file\" ]; then world_file='",
        px4_dir,
        "/Tools/simulation/gz/worlds/",
        px4_world,
        ".sdf'; fi; exec gz sim \"${render_args[@]}\" -r -s \"$world_file\"",
    ]
    gz_client_cmd = [
        *gz_env,
        "render_args=(); if [ -n '",
        gz_render_engine,
        "' ]; then render_args=(--render-engine '",
        gz_render_engine,
        "'); fi; gui_args=(); if [ -n \"${GZ_PREFIX:-}\" ]; then "
        "gui_args=(--gui-config \"$GZ_PREFIX/share/gz/gz-sim8/gui/gui.config\"); fi; "
        "exec gz sim \"${render_args[@]}\" -g \"${gui_args[@]}\"",
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
            DeclareLaunchArgument("gz_world_file", default_value=""),
            DeclareLaunchArgument("px4_gz_standalone", default_value="true"),
            DeclareLaunchArgument("start_gz_server", default_value="true"),
            DeclareLaunchArgument("start_gz_client", default_value="false"),
            DeclareLaunchArgument("start_px4", default_value="true"),
            DeclareLaunchArgument("start_xrce_agent", default_value="true"),
            DeclareLaunchArgument("start_camera_bridge", default_value="true"),
            DeclareLaunchArgument("start_synthetic_rgbd", default_value="false"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("gz_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("gz_partition", default_value="auto_drone_px4"),
            DeclareLaunchArgument("gz_render_engine", default_value=""),
            DeclareLaunchArgument("xrce_port", default_value="8888"),
            DeclareLaunchArgument("bridge_config", default_value=default_bridge_config),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/image"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            DeclareLaunchArgument("marker_topic", default_value="~/markers"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("visualization_period_sec", default_value="1.0"),
            DeclareLaunchArgument("depth_timeout_sec", default_value="0.5"),
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
                cmd=["bash", "-lc", gz_client_cmd],
                output="screen",
                condition=IfCondition(start_gz_client),
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
                arguments=["--ros-args", "-p", ["config_file:=", bridge_config]],
                condition=IfCondition(start_camera_bridge),
            ),
            Node(
                package="auto_drone",
                executable="synthetic_rgbd_node",
                name="auto_drone_synthetic_rgbd",
                output="screen",
                parameters=[
                    {
                        "rgb_topic": rgb_topic,
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                    }
                ],
                condition=IfCondition(start_synthetic_rgbd),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(autonomy_launch),
                launch_arguments={
                    "rgb_topic": rgb_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                    "marker_topic": marker_topic,
                    "map_frame": map_frame,
                    "visualization_period_sec": visualization_period_sec,
                    "depth_timeout_sec": depth_timeout_sec,
                }.items(),
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config],
                output="screen",
                condition=IfCondition(start_rviz),
            ),
        ]
    )
