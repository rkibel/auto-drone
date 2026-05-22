# auto-drone

Headless autonomous drone exploration with active 3D voxel mapping, synthetic range sensing, noisy odometry, and a SLAM-ready transition pipeline.

## Run

Run the 3D voxel simulator:

```bash
python3 -m auto_drone.headless_3d_runner --max-steps 1500 --output-dir frames3d --video-path frames3d/exploration3d.mp4
```

For faster batch validation, render less often:

```bash
python3 -m auto_drone.headless_3d_runner --max-steps 300 --render-every 10000
```

After a ROS/colcon build, the console script is also available:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run auto_drone headless_3d_runner -- --max-steps 1500
```

## Gazebo Drone Reconstruction Demo

The Gazebo integration world is a drone-reconstruction proxy: a quadrotor-shaped inspection vehicle, elevated lidar, odometry, `/cmd_vel` control, and a self-contained neighborhood scene with trees, houses, fences, poles, wires, parked vehicles, and multi-height reconstruction obstacles. It keeps the current mapper-facing contract:

```text
Gazebo /scan + /odom -> RangeFrame -> BeliefVolume
```

Start the Gazebo world:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_IP=127.0.0.1
ign gazebo -r worlds/drone_reconstruction_world.sdf
```

Then start the bridge and mapper from another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_IP=127.0.0.1
ros2 launch auto_drone gazebo_lidar_demo.launch.py
```

Move the proxy vehicle with:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.45}, angular: {z: 0.25}}" -r 5
```

## PX4 RGB-D Autonomy Path

The PX4 v1 path adds the closed-loop simulator-facing pipeline:

```text
Gazebo RGB-D + PX4 odometry or SLAM pose
-> depth ray RangeFrame
-> BeliefVolume
-> discovery planner
-> PX4 offboard velocity setpoints
```

It is installed as:

```bash
ros2 launch auto_drone gazebo_px4_autonomy.launch.py
```

Recommended startup order on the Ubuntu simulator host:

```bash
# Terminal 1: PX4 + Gazebo depth-camera model
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth

# Terminal 2: PX4 DDS bridge
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888

# Terminal 3: auto_drone autonomy node
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch auto_drone gazebo_px4_autonomy.launch.py
```

The same sequence is encoded in `scripts/px4_gazebo_autonomy_smoke.sh` for repeatable host checks:

```bash
PX4_DIR=~/PX4-Autopilot \
WORKSPACE_SETUP=~/ros2_ws/install/setup.bash \
scripts/px4_gazebo_autonomy_smoke.sh
```

On hosts that use a user-local Gazebo install, point `GZ_PREFIX` at the extracted prefix. If PX4 needs Gazebo started separately, enable the standalone mode:

```bash
PX4_DIR=~/PX4-Autopilot-v1.15 \
WORKSPACE_SETUP=~/auto-drone-validation-ws/install/setup.bash \
GZ_PREFIX=~/gz_user_prefix/usr \
PX4_GZ_STANDALONE=1 \
START_GZ_SERVER=1 \
scripts/px4_gazebo_autonomy_smoke.sh
```

Use `--dry-run` first to confirm paths and commands without starting processes.

The launch expects PX4 SITL and the Gazebo/ROS bridge to provide:

- `/camera/image`
- `/camera/depth/image`
- `/camera/depth/camera_info`
- `/fmu/out/vehicle_odometry`
- `/fmu/out/vehicle_status`
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

Configuration lives in `config/px4_autonomy.yaml`. The package also installs `worlds/px4_reconstruction_world.sdf`, a PX4-oriented reconstruction arena with bounded obstacles and an RGB-D reference sensor. PX4 model spawning and bridge startup remain external so the mapper/planner is not tied to a specific PX4 checkout layout.

The autonomy core uses ENU metric coordinates internally. PX4 odometry and setpoints are converted at the ROS node boundary: PX4 NED position and yaw become internal ENU `MetricPose`, and internal velocity/yaw commands are converted back to PX4 NED `TrajectorySetpoint` fields. RGB-D depth pixels are interpreted in camera optical convention, then converted to local forward-left-up voxel rays before mapping.

See [docs/gazebo_setup.md](docs/gazebo_setup.md) for the fuller PX4/Gazebo runbook, topic checks, and host assumptions.

## Architecture

The active mapping loop is structured like a future simulator bridge:

```text
hidden voxel world
-> synthetic range/depth frame
-> noisy odometry estimate
-> residual-weighted mapper
-> keyframe pose graph
-> belief-volume planner
```

The mapper does not receive direct true occupancy or true pose. It receives range-frame measurements with local sensor-frame ray cells, hit/miss state, range, and a residual-like measurement error, then projects those cells from the estimated pose. Low-residual measurements update the belief volume more strongly; high-residual measurements leave more uncertainty.

Important modules:

- `common.py`: shared occupancy constants, angle math, clamping, color blending
- `geometry3d.py`: pose, orientation, cached 3D rays, voxel lines
- `sensing3d.py`: synthetic range/depth frame generation
- `interfaces3d.py`: metric pose, voxel-grid, camera, pose-source, and command target data contracts
- `frames3d.py`: PX4 NED/internal ENU and camera-frame conversion helpers
- `pose_sources.py`: PX4 odometry and ROS SLAM pose-source adapters
- `rgbd_mapping.py`: RGB-D/depth image conversion into local range-frame rays
- `odometry3d.py`: noisy estimated pose tracking
- `mapping3d.py`: range-frame integration into the belief volume
- `slam3d.py`: keyframes and relative-pose constraints
- `autonomy3d.py`: safe discovery planning on bounded belief volumes
- `px4_autonomy_node.py`: ROS 2/PX4 closed-loop autonomy node
- `core3d.py`: seeded voxel world, belief volume, active mapping loop, planner
- `render3d.py`: isometric voxel rendering
- `headless_3d_runner.py`: CLI runner and video generation

## Visualization

The 3D video uses a single isometric projection of the voxel cube.

- cube outline: world bounds
- blue marker: true drone pose
- green marker: estimated odometry pose, when it differs
- blue path: planned path
- yellow marker: target viewpoint
- cyan voxels/rays: currently visible sensor cone
- light-to-dark red: uncertainty
- near-white: confidently observed free space
- black: believed obstacle

The sensor is a bounded 3D cone. It casts cached rays inside the configured field of view out to `sensor_radius`, and rays stop at obstacles.

## Tests

Run the Python test suite:

```bash
python3 -m pytest -q
```

Run the ROS/colcon test path:

```bash
source /opt/ros/humble/setup.bash
colcon test --event-handlers console_direct+
```

The tests cover uncertainty updates, belief-only visibility occlusion, 3D reachability, cached sensor rays, bounded cone sensing, sensor frame generation, residual-weighted mapping, noisy odometry, pose graph constraints, and 6-DOF target pose generation.

For a fuller health check, use the repo-local Codex validation skill:

```bash
python3 .codex/skills/validate-auto-drone/scripts/validate_auto_drone.py --repo .
```

That validator runs compile checks, tests, no-render mapping efficiency smoke checks, render/video smoke checks, and ROS/colcon checks when available.

For dead-code cleanup and stale-reference checks:

```bash
python3 .codex/skills/cleanup-auto-drone/scripts/dead_code_audit.py --repo .
```

## Transition Plan

See [docs/transition_plan.md](docs/transition_plan.md) for the staged path toward simulator-based SLAM with RGB-D/lidar/odometry feeds.
