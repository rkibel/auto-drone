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

The mapper does not receive direct true occupancy. It receives range-frame measurements with ray cells, hit/miss state, range, and a residual-like measurement error. Low-residual measurements update the belief volume more strongly; high-residual measurements leave more uncertainty.

Important modules:

- `common.py`: shared occupancy constants, angle math, clamping, color blending
- `geometry3d.py`: pose, orientation, cached 3D rays, voxel lines
- `sensing3d.py`: synthetic range/depth frame generation
- `odometry3d.py`: noisy estimated pose tracking
- `mapping3d.py`: range-frame integration into the belief volume
- `slam3d.py`: keyframes and relative-pose constraints
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
