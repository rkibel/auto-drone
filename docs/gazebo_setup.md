# Gazebo / ROS 2 Setup

This repository includes a Gazebo Fortress integration path for validating the simulator-facing mapper contract:

```text
Gazebo lidar + odometry -> ROS /scan + /odom -> RangeFrame -> BeliefVolume
```

The public setup intentionally avoids host-specific SSH, display, and credential details. Keep machine-local access instructions in ignored local notes, for example `docs/gazebo_setup.local.md`.

## Requirements

- Ubuntu 22.04 or compatible Linux environment
- ROS 2 Humble
- Gazebo Fortress / Ignition Gazebo 6
- `ros_gz_bridge`
- A working display for Gazebo GUI when using rendering-backed sensors such as GPU lidar

PX4-specific additions for the RGB-D autonomy path:

- PX4-Autopilot checkout with Gazebo simulation support and the `gz_x500_depth` target available
- `px4_msgs` built in the same ROS 2 workspace overlay as `auto_drone`, matching the PX4 checkout's message definitions
- Micro XRCE-DDS Agent available as `MicroXRCEAgent` and compatible with PX4's uXRCE-DDS client
- UDP port `8888` free on the simulator host for PX4 DDS traffic

## Build

From the ROS workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select auto_drone
source install/setup.bash
```

Expected executables:

```text
auto_drone gazebo_lidar_bridge
auto_drone headless_3d_runner
```

## Drone Reconstruction World

The current integration world is:

```text
worlds/drone_reconstruction_world.sdf
```

It is a self-contained SDF neighborhood scene rather than a downloaded asset pack. It uses compound primitives for trees, houses, fences, utility lines, parked vehicles, and inspection targets, so the world can be rebuilt and launched without external model downloads.

The world provides:

- `/scan` from an elevated GPU lidar
- `/odom` from the simulator motion model
- `/cmd_vel` for manual motion commands
- a quadrotor-shaped mapping proxy
- multi-height geometry for reconstruction and occlusion tests

The proxy is still not a full multirotor dynamics stack. It exists to validate the mapping/perception interface before introducing PX4, ArduPilot, or a richer mesh asset library.

## Run Gazebo

In a terminal with a display attached:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_IP=127.0.0.1
ign gazebo -r src/auto_drone/worlds/drone_reconstruction_world.sdf
```

If you are already in the package directory instead of the workspace root, use:

```bash
ign gazebo -r worlds/drone_reconstruction_world.sdf
```

In another terminal, confirm Gazebo transport topics:

```bash
source /opt/ros/humble/setup.bash
export IGN_IP=127.0.0.1
ign topic -l | grep -E '/scan|/odom|/cmd_vel'
```

## Run Bridge And Mapper

Start the ROS bridges and mapper:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_IP=127.0.0.1
ros2 launch auto_drone gazebo_lidar_demo.launch.py
```

The launch file bridges:

- `/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan`
- `/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry`
- `/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist`

It also starts `gazebo_lidar_bridge`, which converts scan rays into local sensor-frame voxel offsets and integrates them into the 3D belief volume using estimated odometry.

The lidar is mounted above the proxy base, so `gazebo_lidar_demo.launch.py` passes `sensor_z_offset_meters:=0.98`. That maps observed cells at the sensor height instead of the ground-contact proxy origin.

## Smoke Checks

Confirm ROS receives scan and odometry:

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
```

Move the proxy:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.45}, angular: {z: 0.25}}" -r 5
```

Stop it:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

The mapper log should periodically print frame count, known ratio, uncertainty, and estimated voxel pose. When the proxy moves, known ratio should increase as new scan rays are integrated.

If `/cmd_vel` publishes but the proxy does not move, check whether the world is paused:

```bash
export IGN_IP=127.0.0.1
timeout 2 ign topic -e -t /world/drone_reconstruction_world/stats
```

Unpause from the Gazebo GUI or through the world control service:

```bash
ign service -s /world/drone_reconstruction_world/control \
  --reqtype ignition.msgs.WorldControl \
  --reptype ignition.msgs.Boolean \
  --timeout 2000 \
  --req "pause: false"
```

## Future Asset Direction

The next visual-realism step is to add a controlled model asset pipeline:

- Gazebo Fuel models for common assets when network/model caching is acceptable
- checked-in lightweight meshes for repeatable reconstruction targets
- PX4 or ArduPilot SITL for realistic multirotor motion

Keep the mapper-facing topic contract stable while improving the simulator underneath it.

## PX4 RGB-D Autonomy Path

The first closed-loop multirotor path is centered on PX4 SITL and RGB-D mapping. It keeps the same mapper/planner core, but replaces manual `/cmd_vel` motion with PX4 offboard setpoints:

```text
RGB-D depth image + camera info + PX4 odometry or SLAM pose
-> RangeFrame
-> BeliefVolume
-> discovery waypoint
-> /fmu/in/offboard_control_mode + /fmu/in/trajectory_setpoint
```

The package installs:

- `launch/gazebo_px4_autonomy.launch.py`
- `config/px4_autonomy.yaml`
- `worlds/px4_reconstruction_world.sdf`
- `auto_drone px4_autonomy_node`

### Startup Runbook

Use the PX4-managed depth-camera model first. It is the lowest-friction path because PX4 owns model spawning, vehicle plugins, and the Gazebo/PX4 coupling:

```bash
# Terminal 1: PX4 SITL and Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth
```

Start the DDS agent in a second terminal. PX4 SITL normally starts its uXRCE-DDS client automatically for ROS 2, and the agent makes the `/fmu/*` topics visible to ROS:

```bash
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

Build and source the ROS workspace that contains `px4_msgs` and `auto_drone`:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select px4_msgs auto_drone
source install/setup.bash
```

If PX4's camera topics do not already match the repository defaults, add `ros_gz_bridge` remaps so the autonomy launch sees:

```text
/camera/image
/camera/depth/image
/camera/depth/camera_info
```

Then start the autonomy node:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch auto_drone gazebo_px4_autonomy.launch.py
```

For a single ROS launch entry point on a simulator host, use the composed launch. It starts Micro XRCE Agent, optionally starts a standalone Gazebo server, starts PX4 SITL, bridges the x500 depth-camera topics, and includes the autonomy node:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch auto_drone gazebo_px4_full_stack.launch.py \
  px4_dir:=~/PX4-Autopilot-v1.15 \
  px4_gz_standalone:=true \
  start_gz_server:=true
```

If PX4 or Gazebo are already supervised outside ROS launch, disable those processes with `start_px4:=false`, `start_gz_server:=false`, or `start_xrce_agent:=false`.

For repeatable host validation, use the smoke runner from the repository root:

```bash
PX4_DIR=~/PX4-Autopilot \
WORKSPACE_SETUP=$PWD/install/setup.bash \
scripts/px4_gazebo_autonomy_smoke.sh --dry-run
```

Remove `--dry-run` once PX4, `MicroXRCEAgent`, and the ROS workspace overlay are confirmed. The script starts the XRCE agent, starts `make px4_sitl gz_x500_depth`, and then launches `auto_drone` with cleanup traps for the child processes.

If Gazebo is installed into a user-writable prefix instead of `/usr`, set `GZ_PREFIX` so the script exports the required binary, library, Ruby command, system-plugin, physics-plugin, rendering-plugin, OGRE, and model paths. The runner also rewrites writable `GZ_PREFIX/share/gz/*.yaml` command metadata when extracted Debian packages still point at `/usr/lib/ruby/gz`. For rootless PX4/Gazebo validation on hosts like `noa`, use a fixed Gazebo transport endpoint and standalone server mode:

```bash
PX4_DIR=~/PX4-Autopilot-v1.15 \
WORKSPACE_SETUP=~/auto-drone-validation-ws/install/setup.bash \
GZ_PREFIX=~/gz_user_prefix/usr \
GZ_IP=127.0.0.1 \
GZ_PARTITION=auto_drone_px4 \
PX4_GZ_STANDALONE=1 \
START_GZ_SERVER=1 \
scripts/px4_gazebo_autonomy_smoke.sh
```

For local topic names that differ from the defaults, pass launch overrides rather than editing the config:

```bash
ros2 launch auto_drone gazebo_px4_autonomy.launch.py \
  rgb_topic:=/your/rgb/topic \
  depth_topic:=/your/depth/topic \
  camera_info_topic:=/your/camera_info/topic
```

Required topics:

```text
/camera/image
/camera/depth/image
/camera/depth/camera_info
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

Smoke checks before enabling motion:

```bash
ros2 topic echo /fmu/out/vehicle_odometry --once
ros2 topic echo /fmu/out/vehicle_status --once
ros2 topic echo /camera/depth/camera_info --once
ros2 topic hz /camera/depth/image
```

The autonomy log should show depth frames being integrated, a current pose, planned targets, and offboard/arming warnings when PX4 rejects mode changes. If `/fmu/out/*` topics are missing, check the XRCE agent and `px4_msgs` version before changing `auto_drone`.

By default the node uses PX4 odometry as the pose source. To use visual SLAM, publish `geometry_msgs/msg/PoseStamped` and set `slam_pose_topic` plus `use_slam_pose:=true` in `config/px4_autonomy.yaml` or as launch overrides. If PX4 does not report armed offboard mode after setpoints begin, the node logs a preflight/mode warning instead of silently failing.

Frame conventions:

- The mapper and planner use ENU metric coordinates.
- PX4 `VehicleOdometry` and `TrajectorySetpoint` are treated as NED at the node boundary.
- Depth image pixels are treated as camera optical frame samples and converted to local forward-left-up voxel rays before log-odds integration.
- Camera extrinsics are not estimated in v1; mount the RGB-D optical frame forward-facing with the vehicle, or publish a SLAM pose that already represents the sensor/body frame expected by the mapper.

### Custom Reconstruction World

`worlds/px4_reconstruction_world.sdf` is a repository-owned reconstruction arena, not a complete PX4 vehicle model. Treat it as the environment to merge into a PX4-supported Gazebo world or to load in PX4 standalone Gazebo mode once the host's PX4/Gazebo version is pinned.

The intended validation sequence is:

```bash
# Terminal 1: PX4 waits for an external Gazebo server
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_depth

# Terminal 2: start a Gazebo server with a PX4-compatible world/model setup
gz sim -r /path/to/px4_reconstruction_world_with_x500_depth.sdf
```

This custom-world flow is intentionally not baked into `gazebo_px4_autonomy.launch.py` yet. Keep PX4 spawning and Gazebo bridge wiring outside the `auto_drone` launch until the Ubuntu simulator host proves a repeatable model path, camera topic names, and world-file layout.

### External Assumptions

These assumptions cannot be validated from a macOS checkout and must be confirmed on the Ubuntu simulator host:

- PX4, Gazebo, and ROS 2 versions are mutually compatible. Current PX4 docs use the `gz_*` Gazebo targets, while this repository's lidar proxy docs still target Gazebo Fortress/Ignition naming.
- `make px4_sitl gz_x500_depth` publishes depth/RGB/camera-info topics that can be bridged or remapped to `/camera/image`, `/camera/depth/image`, and `/camera/depth/camera_info`.
- The PX4 uXRCE-DDS client connects to `MicroXRCEAgent udp4 -p 8888` and exposes `/fmu/out/vehicle_odometry` plus `/fmu/out/vehicle_status`.
- The Micro XRCE-DDS Agent version is compatible with the PX4 checkout's uXRCE-DDS client.
- The workspace `px4_msgs` package matches the PX4 SITL checkout. Mismatches can deserialize incorrectly even when topic names look correct.
- The RGB-D camera optical frame is forward-facing relative to the body frame expected by `auto_drone`, or a SLAM pose source compensates for that extrinsic.
