# Transition Plan Toward Simulator-Based SLAM

## Goal

Move from direct voxel observation to a simulator-ready active mapping pipeline:

```text
true simulated world
-> sensor-like range/depth frames
-> noisy odometry / pose estimates
-> mapping from estimated pose
-> keyframes / pose graph
-> uncertainty from measurement residuals
-> active viewpoint planning
```

The eventual simulator bridge should replace only the data source, not the mapper or planner.

## Stages

1. **Synthetic range/depth frames**
   - Generate ray measurements with hit/miss, ray cells, range, and residual-like error.
   - Stop giving the mapper direct true occupancy.

2. **Noisy odometry**
   - Track true pose and estimated pose separately.
   - Use estimated pose when integrating sensor frames into the map.

3. **Range-frame mapping**
   - Carve free space along rays.
   - Mark hit cells as occupied.
   - Weight map updates by a confidence derived from reprojection/range error.
   - Treat measurement cells as local sensor-frame offsets, then project them from the estimated pose.

4. **SLAM skeleton**
   - Add keyframes, relative-pose constraints, residuals, and a pose graph.
   - Keep optimization simple for now; the shape matters because simulator/SLAM packages can plug in later.

5. **Simulator bridge**
   - Convert simulator RGB-D/lidar/odometry into the same sensor-frame and pose-estimate structures.
   - Keep the planner consuming `BeliefVolume` and pose estimates.

## Current Adapter Contract

`RangeFrame` is the simulator-facing contract. It carries an estimated pose and a sequence of range measurements. Each measurement contains local voxel offsets along a ray, whether the ray hit an obstacle, a measured range, and a residual-like error. Mapping must not depend on simulator ground truth; synthetic ground truth is only used to generate fake sensor returns and render debug visibility.

## Current Implementation Target

This milestone implements stages 1-4 with synthetic data, while preserving the current fast voxel planner and renderer.

## PX4 RGB-D Closed-Loop Milestone

The next implemented milestone adds the simulator-facing loop without replacing the lightweight voxel map:

```text
PX4 SITL + Gazebo RGB-D
-> depth image ray conversion
-> bounded BeliefVolume
-> safe discovery planning
-> PX4 offboard setpoints
```

The core data contracts are `MetricPose`, `VoxelGridSpec`, `SensorFrame3D`, `PoseSample`, and `CommandTarget`. The ROS node defaults to PX4 odometry as a pose source, and accepts a `PoseStamped` SLAM topic when `use_slam_pose` is enabled. RGB frames are subscribed and timestamped so visual SLAM can be connected without changing the mapper/planner boundary; depth frames provide deterministic free/occupied voxel evidence for v1.

Frame boundaries are explicit in v1: `frames3d.py` converts PX4 NED odometry/setpoints to and from the internal ENU map frame, and `rgbd_mapping.py` converts camera optical depth pixels into forward-left-up local voxel rays. Camera extrinsic calibration remains a simulator setup constraint rather than an estimated state.
