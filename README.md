# auto-drone

`auto-drone` is a Mac-local ROS 2 autonomy demo for PX4 SITL and Gazebo
Harmonic. Its target is a forest flight that reconstructs the occupied
environment.

## Active architecture

```text
PX4 SITL + Gazebo Harmonic
        │
        ├── uXRCE-DDS Agent → ROS 2 PX4 topics
        └── ros_gz_bridge → ROS 2 depth topics

auto_drone ROS 2 node
        → RGB-D occupancy map and frontier-based exploration
        → PX4 velocity setpoints
```

All project code is under `auto_drone/`:

- `mapping.py`, `rgbd.py`, `interfaces.py`: synchronized RGB-D mapping.
- `coverage_planner.py`: frontier selection and collision-safe routes.
- `forest_metrics.py`: forest benchmark coverage evaluation.
- `px4_control_node.py`: ROS 2/PX4 orchestration and RViz publishing.

## Build

```bash
conda activate auto-drone-ros
colcon build --base-paths . vendor/px4_msgs --packages-select px4_msgs auto_drone
. install/setup.zsh
```

See [docs/ros2-workflow.md](docs/ros2-workflow.md) for the native macOS ROS 2,
uXRCE-DDS Agent, and bridge setup. The project direction, measurements, and
decisions live in [docs/project-brief.md](docs/project-brief.md).
