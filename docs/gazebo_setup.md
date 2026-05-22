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
