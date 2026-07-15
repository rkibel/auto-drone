# ROS 2 workflow

This is the current Mac-local ROS 2 workflow for the forest reconstruction
demo.

## Why this environment

The Mac runs Gazebo Harmonic and PX4 natively through Homebrew. ROS 2 Jazzy is
installed in the isolated Conda environment `auto-drone-ros` through RoboStack.
Keeping it separate avoids modifying the system Python or mixing ROS libraries
into the Homebrew Gazebo environment.

Always activate the environment before using ROS commands:

```bash
conda activate auto-drone-ros
```

The activation step sets `AMENT_PREFIX_PATH`, which ROS 2 uses to discover its
installed packages. Calling a `ros2` binary by absolute path without activation
does not set that variable.

## Build and run the controller baseline

From the repository root:

```bash
conda activate auto-drone-ros
colcon build --base-paths . vendor/px4_msgs --packages-select px4_msgs auto_drone
. install/setup.zsh
ros2 run auto_drone px4_control_node
```

`--symlink-install` is not currently compatible with this Conda environment's
editable-install tooling. A normal Colcon install is reliable for this small
Python package; rebuild after source edits.

Run the focused pure-Python tests in the same Conda environment:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

RoboStack's `launch_testing` plugin is not compatible with the environment's
pytest 9 release. The active test suite does not use launch testing, so
disabling automatic third-party plugin loading keeps the test command focused.

The controller subscribes to PX4 odometry and publishes stationary velocity
setpoints. It does not arm PX4 or enter offboard mode.

In a second activated terminal, verify ROS 2 communication with:

```bash
ros2 topic echo /auto_drone_px4_control/status std_msgs/msg/String --once
```

## Current behavior

The node publishes stationary velocity setpoints by default. It maps the
bridged depth stream at a capped rate, publishes tree-height occupancy markers
for RViz, and reports forest-specific coverage metrics. The low-coverage
planner remains an explicit simulator-only opt-in.

## PX4 message interface

The local PX4 checkout is v1.18. Its matching `px4_msgs` branch is installed
locally under `vendor/px4_msgs` and built alongside this package:

```bash
conda activate auto-drone-ros
colcon build --base-paths . vendor/px4_msgs --packages-select px4_msgs auto_drone
. install/setup.zsh
ros2 interface show px4_msgs/msg/VehicleOdometry
```

`px4_msgs` is ignored by Git because it is an upstream dependency, not project
source. PX4 and ROS must use matching generated message definitions; otherwise
the uXRCE-DDS bridge cannot safely interpret telemetry or offboard commands.

The matching Micro XRCE-DDS Agent 2.4.3 is also a local ignored dependency in
`vendor/Micro-XRCE-DDS-Agent`. Its build is separate from Colcon because it is
the UDP proxy between PX4's uXRCE-DDS client and the ROS 2 DDS graph.

Build the agent against the matching Fast DDS libraries already in the ROS 2
environment. This avoids its incompatible macOS super-build:

```bash
ROS_ENV="$HOME/miniconda3/envs/auto-drone-ros"
CMAKE_PREFIX_PATH="$ROS_ENV" cmake -S vendor/Micro-XRCE-DDS-Agent \
  -B vendor/Micro-XRCE-DDS-Agent/build-ros \
  -DCMAKE_BUILD_TYPE=Release \
  -DUAGENT_SUPERBUILD=OFF \
  -DUAGENT_USE_SYSTEM_FASTDDS=ON \
  -DUAGENT_USE_SYSTEM_FASTCDR=ON \
  -DUAGENT_LOGGER_PROFILE=OFF \
  -DUAGENT_CED_PROFILE=OFF \
  -DUAGENT_SOCKETCAN_PROFILE=OFF
cmake --build vendor/Micro-XRCE-DDS-Agent/build-ros --parallel 4
```

Start it before PX4:

```bash
DYLD_LIBRARY_PATH="$ROS_ENV/lib" \
vendor/Micro-XRCE-DDS-Agent/build-ros/MicroXRCEAgent udp4 -p 8888 -v 4
```

With PX4 SITL running, `ros2 topic list -t` exposes the `/fmu/in/...` and
`/fmu/out/...` topics. PX4 v1.18 publishes `VehicleStatus` as
`/fmu/out/vehicle_status_v4`; use that versioned topic instead of the older
unversioned name.

## PX4 controller check

The default `px4_control_node` subscribes to PX4 odometry and publishes
stationary velocity setpoints at 20 Hz. It does not arm PX4 or request offboard
mode:

```bash
conda activate auto-drone-ros
. install/setup.zsh
ros2 run auto_drone px4_control_node
```

In a second terminal, confirm that it receives telemetry and emits setpoints:

```bash
ros2 topic echo /auto_drone_px4_control/status std_msgs/msg/String --once
```

## Deterministic simulator reset

For every flight test, restart the **entire** simulator stack rather than only
the controller. A controller-only restart leaves the previous PX4/Gazebo
vehicle state in place, which can preserve an in-flight failsafe or a bad
initial pose.

The forest's fixed spawn is the level open point at `(0, 0, 0)` with zero
attitude. The new woodland-neighborhood scene keeps an eight-metre clear area
around it before the nearest obstacle. On its first run, Gazebo downloads the
CC0 OpenRobotics tree, bench, and barrel assets from Fuel; later runs use the
local cache. Launch PX4/Gazebo from its checkout with that pose explicitly set:

```bash
cd vendor/PX4-Autopilot
PX4_GZ_WORLD=forest PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
  make px4_sitl gz_x500_depth
```

Before starting the Gazebo bridge, Micro XRCE-DDS agent, or controller, verify
that Gazebo ground-truth odometry reports the expected stationary spawn pose
(within 5 cm in `z` after landing-gear settling):

```bash
gz topic -e -n 1 -t /model/x500_depth_0/odometry
```

Then start the bridge, agent, and controller. To reset again, stop those
processes first and stop PX4/Gazebo last; do not restart only
`px4_control_node` while a flight is active.

## Gazebo depth bridge

`ros-jazzy-ros-gz-bridge` is installed in the Conda environment and matches the
installed `ros_gz_interfaces` version. Do not build a second copy from source:
the source checkout required a newer interface version and added a macOS loader
conflict without helping this demo.

With Gazebo running, bridge the depth-camera topic into ROS 2:

```bash
conda activate auto-drone-ros
ros2 run ros_gz_bridge parameter_bridge \
  '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
  '/world/forest/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/model/x500_depth_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
```

Use `gz topic -l` to confirm the exact Gazebo topic names before starting the
bridge. The odometry stream is simulator ground truth used for mapping; PX4
odometry remains the control source. RGB and depth are fused only at identical
timestamps.

## Inspect the occupancy reconstruction

The controller maps at most eight depth frames per second and publishes a
capped occupancy marker list four times per second. It publishes the drone mesh
and camera frustum independently at 20 Hz. Start the committed RViz
configuration in an activated terminal:

```bash
rviz2 -d config/occupancy.rviz
```

It sets the fixed frame to `map` and subscribes displays to the occupied voxel,
drone, and camera-frustum topics. The controller publishes a static `map` to
`occupancy_map` transform, so RViz can resolve the marker frame. Floor hits are
integrated and rendered without a height or color mask.

The saved view is `Orbit`: left-drag rotates around the focal point, middle-drag
pans it, and the scroll wheel or right-drag zooms. The configuration starts at
20 m; edit `Distance` in the `Views` panel if you need a closer view. The
toolbar's camera icon selects `Move Camera`, which receives those gestures;
the target icon runs `Focus Camera`. Use the `Views` panel to choose another
camera controller without changing the map's fixed frame.

## Optional coverage planner

The default controller only holds position. The planner is an explicit simulator
opt-in. It climbs to 8 m, derives its safe bounds from the voxel grid, and uses
the belief map's free, unknown, and occupied states to select frontier routes.
It has no tree-reference input:

```bash
ros2 run auto_drone px4_control_node --ros-args -p enable_exploration:=true
```

The path check treats one occupied observation as safety evidence, inflates it
by 2 m, and revalidates the active segment whenever a new map frame is
integrated. It first routes only through confirmed-free cells, then uses
high-cost unknown cells only when that is the only way to reach a frontier. If
new canopy evidence encloses the current pose, the controller first backtracks
to a previously safe pose and then replans.

The current forest SITL reports a failed preflight check even after accepting
offboard setpoints. For simulator-only validation, add the separate explicit
override below. Do not use it for hardware.

```bash
ros2 run auto_drone px4_control_node --ros-args \
  -p enable_exploration:=true -p force_arm:=true
```

The same local PX4 instance defaults to Return when no GCS connection exists.
Disable that unrelated simulator action before the sweep:

```bash
vendor/PX4-Autopilot/build/px4_sitl_default/bin/px4-param set NAV_DLL_ACT 0
```

This setting and `force_arm` are simulator-only. The current sweep proves PX4
Offboard transport and live map growth, but has not yet demonstrated full route
completion or high tree coverage within 30 seconds.
