from math import inf, isclose, pi
from struct import pack

from auto_drone.autonomy3d import DiscoveryPlan, choose_discovery_plan, inflated_occupied_voxels, reachable_safe_voxels
from auto_drone.core3d import BeliefVolume
from auto_drone.frames3d import enu_velocity_to_ned, px4_ned_pose_to_metric, yaw_enu_to_ned, yaw_ned_to_enu, yaw_rate_enu_to_ned
from auto_drone.interfaces3d import CameraIntrinsics, MetricPose, PoseSample, VoxelGridSpec, command_toward_pose
from auto_drone.mapping3d import integrate_range_frame
from auto_drone.px4_autonomy_node import (
    AutonomyConfig,
    ClosedLoopAutonomy,
    telemetry_to_json,
    yaw_from_quaternion,
)
from auto_drone.pose_sources import Px4OdomPoseSource, RosSlamPoseSource
from auto_drone.rgbd_mapping import decode_depth_image, depth_image_to_range_frame, depth_pixel_to_local_voxel


def test_metric_pose_maps_to_voxel_and_back():
    grid = VoxelGridSpec(20, 20, 10, 0.5, -5.0, -5.0, 0.0)
    pose = MetricPose(0.0, -1.0, 1.5, yaw=0.2)
    voxel = grid.metric_to_voxel(pose)
    assert (voxel.x, voxel.y, voxel.z, voxel.yaw) == (10, 8, 3, 0.2)
    assert grid.voxel_to_metric(10, 8, 3, yaw=0.4) == MetricPose(0.0, -1.0, 1.5, yaw=0.4)


def test_depth_pixel_converts_to_local_camera_ray():
    intrinsics = CameraIntrinsics(width=4, height=4, fx=2.0, fy=2.0, cx=1.5, cy=1.5)
    assert depth_pixel_to_local_voxel(2, 2, 1.0, intrinsics, resolution=0.5) == (2, 0, 0)
    assert depth_pixel_to_local_voxel(3, 1, 1.0, intrinsics, resolution=0.5) == (2, -2, 0)


def test_rgbd_integration_marks_free_and_occupied_cells():
    grid = VoxelGridSpec(12, 8, 6, 0.5, -2.0, -2.0, 0.0)
    intrinsics = CameraIntrinsics(width=1, height=1, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    frame = depth_image_to_range_frame(
        [1.0],
        intrinsics,
        MetricPose(0.0, 0.0, 1.0),
        grid,
        min_depth_m=0.25,
        max_depth_m=4.0,
        stride=1,
    )
    belief = BeliefVolume(grid.width, grid.height, grid.depth)
    mapped = integrate_range_frame(belief, frame)
    assert (5, 4, 2) in mapped
    assert (6, 4, 2) in mapped
    assert belief.occupancy_probability(5, 4, 2) < 0.5
    assert belief.occupancy_probability(6, 4, 2) > 0.5


def test_rgbd_ignores_invalid_and_max_range_depths_safely():
    grid = VoxelGridSpec(12, 8, 6, 0.5, -2.0, -2.0, 0.0)
    intrinsics = CameraIntrinsics(width=3, height=1, fx=1.0, fy=1.0, cx=1.0, cy=0.0)
    frame = depth_image_to_range_frame(
        [inf, 0.1, 4.5],
        intrinsics,
        MetricPose(0.0, 0.0, 1.0),
        grid,
        min_depth_m=0.25,
        max_depth_m=4.0,
        stride=1,
    )
    assert len(frame.measurements) == 1
    assert not frame.measurements[0].hit


def test_depth_image_decoding_supports_float_and_uint16():
    float_data = pack("<ff", 1.25, 2.5)
    assert decode_depth_image(float_data, 2, 1, "32FC1", 8) == [1.25, 2.5]
    uint_data = pack("<HH", 1250, 2500)
    assert decode_depth_image(uint_data, 2, 1, "16UC1", 4) == [1.25, 2.5]


def test_frontier_planner_avoids_inflated_obstacles():
    grid = VoxelGridSpec(8, 8, 4, 1.0, 0.0, 0.0, 0.0)
    belief = BeliefVolume(grid.width, grid.height, grid.depth)
    for x in range(1, 6):
        belief.observe(x, 3, 2, occupied=False)
        belief.observe(x, 3, 2, occupied=False)
    for _ in range(5):
        belief.observe(3, 3, 2, occupied=True)

    inflated = inflated_occupied_voxels(belief, clearance_voxels=1)
    _, distances = reachable_safe_voxels((1, 3, 2), belief, inflated)
    assert (3, 3, 2) not in distances
    assert (4, 3, 2) not in distances

    plan = choose_discovery_plan(belief, grid, MetricPose(1.0, 3.0, 2.0), clearance_voxels=1)
    assert plan.target is not None
    target_voxel = grid.metric_to_voxel(plan.target)
    assert (target_voxel.x, target_voxel.y, target_voxel.z) not in inflated


def test_command_adapter_clamps_velocity_and_yaw_rate():
    command = command_toward_pose(
        MetricPose(0.0, 0.0, 0.0, yaw=0.0),
        MetricPose(3.0, 4.0, 0.0, yaw=pi),
        max_speed_mps=2.0,
        max_yaw_rate_rps=0.5,
        position_tolerance_m=0.1,
    )
    assert isclose(command.velocity[0], 1.2)
    assert isclose(command.velocity[1], 1.6)
    assert command.yaw_rate == -0.5
    assert not command.reached


def test_px4_ned_and_internal_enu_frame_conversions():
    pose = px4_ned_pose_to_metric(3.0, 2.0, -1.0, yaw_ned=0.0)
    assert pose == MetricPose(2.0, 3.0, 1.0, yaw=pi / 2.0)
    assert enu_velocity_to_ned(2.0, 3.0, 1.0) == (3.0, 2.0, -1.0)
    assert yaw_ned_to_enu(0.0) == pi / 2.0
    assert yaw_enu_to_ned(pi / 2.0) == 0.0
    assert yaw_rate_enu_to_ned(0.4) == -0.4


def test_pose_source_adapters_expose_latest_samples():
    px4_source = Px4OdomPoseSource()
    slam_source = RosSlamPoseSource()
    px4_sample = px4_source.update(MetricPose(1.0, 2.0, 3.0), 4.0)
    slam_sample = slam_source.update(MetricPose(3.0, 2.0, 1.0), 5.0, confidence=0.8)
    assert px4_source.latest() == px4_sample
    assert px4_sample.source == "px4_odom"
    assert slam_source.latest() == slam_sample
    assert slam_sample.source == "slam"
    assert slam_sample.confidence == 0.8


def test_closed_loop_autonomy_integrates_depth_plans_and_commands():
    grid = VoxelGridSpec(16, 10, 6, 0.5, -2.0, -2.0, 0.0)
    autonomy = ClosedLoopAutonomy(AutonomyConfig(grid=grid, depth_stride=1, planning_period_sec=0.0))
    autonomy.update_pose(PoseSample(MetricPose(0.0, 0.0, 1.0), stamp_sec=1.0))
    autonomy.update_camera_info(CameraIntrinsics(width=1, height=1, fx=1.0, fy=1.0, cx=0.0, cy=0.0))
    mapped_count = autonomy.integrate_depth([1.0], stamp_sec=1.1)
    ready, reason = autonomy.ready(now_sec=1.2)
    plan = autonomy.update_plan_if_due(now_sec=1.2)
    command = autonomy.command()
    assert mapped_count > 0
    assert ready, reason
    assert plan.target is not None
    assert command is not None


def test_closed_loop_autonomy_reports_safety_hold_reasons():
    grid = VoxelGridSpec(8, 8, 4, 0.5, -2.0, -2.0, 0.0)
    autonomy = ClosedLoopAutonomy(
        AutonomyConfig(grid=grid, pose_timeout_sec=0.5, depth_timeout_sec=0.5, min_altitude_m=0.5)
    )

    assert autonomy.ready(now_sec=1.0) == (False, "missing_pose")

    autonomy.update_pose(PoseSample(MetricPose(0.0, 0.0, 1.0), stamp_sec=1.0))
    assert autonomy.ready(now_sec=1.1) == (False, "missing_camera_info")

    autonomy.update_camera_info(CameraIntrinsics(width=1, height=1, fx=1.0, fy=1.0, cx=0.0, cy=0.0))
    assert autonomy.ready(now_sec=1.6) == (False, "stale_pose")

    autonomy.update_pose(PoseSample(MetricPose(0.0, 0.0, 1.0), stamp_sec=2.0))
    assert autonomy.ready(now_sec=2.1) == (False, "stale_depth")


def test_closed_loop_autonomy_reports_bounds_altitude_and_rgb_holds():
    grid = VoxelGridSpec(8, 8, 4, 0.5, -2.0, -2.0, 0.0)
    autonomy = ClosedLoopAutonomy(
        AutonomyConfig(
            grid=grid,
            pose_timeout_sec=1.0,
            depth_timeout_sec=1.0,
            min_altitude_m=0.5,
            max_altitude_m=1.5,
            require_rgb_frame=True,
        )
    )
    autonomy.update_camera_info(CameraIntrinsics(width=1, height=1, fx=1.0, fy=1.0, cx=0.0, cy=0.0))

    autonomy.update_pose(PoseSample(MetricPose(30.0, 0.0, 1.0), stamp_sec=1.0))
    assert autonomy.ready(now_sec=1.1) == (False, "pose_out_of_bounds")

    autonomy.update_pose(PoseSample(MetricPose(0.0, 0.0, 0.1), stamp_sec=1.0))
    assert autonomy.ready(now_sec=1.1) == (False, "below_min_altitude")

    autonomy.update_pose(PoseSample(MetricPose(0.0, 0.0, 1.6), stamp_sec=1.0))
    assert autonomy.ready(now_sec=1.1) == (False, "above_max_altitude")

    autonomy.update_pose(PoseSample(MetricPose(0.0, 0.0, 1.0), stamp_sec=1.0))
    autonomy.integrate_depth([1.0], stamp_sec=1.05)
    assert autonomy.ready(now_sec=1.1) == (False, "stale_rgb")
    autonomy.update_rgb_stamp(stamp_sec=1.08)
    assert autonomy.ready(now_sec=1.1) == (True, None)


def test_closed_loop_depth_integration_holds_until_pose_and_camera_info_available():
    grid = VoxelGridSpec(8, 8, 4, 0.5, -2.0, -2.0, 0.0)
    autonomy = ClosedLoopAutonomy(AutonomyConfig(grid=grid, depth_stride=1))

    assert autonomy.integrate_depth([1.0], stamp_sec=1.0) == 0
    assert autonomy.last_depth_stamp_sec == -1.0
    assert autonomy.frame_count == 0

    autonomy.update_pose(PoseSample(MetricPose(0.0, 0.0, 1.0), stamp_sec=1.0))
    assert autonomy.integrate_depth([1.0], stamp_sec=1.1) == 0
    assert autonomy.last_depth_stamp_sec == -1.0
    assert autonomy.frame_count == 0

    autonomy.update_camera_info(CameraIntrinsics(width=1, height=1, fx=1.0, fy=1.0, cx=0.0, cy=0.0))
    assert autonomy.integrate_depth([1.0], stamp_sec=1.2) > 0
    assert autonomy.last_depth_stamp_sec == 1.2
    assert autonomy.frame_count == 1


def test_closed_loop_command_tracks_next_path_voxel_before_final_target():
    grid = VoxelGridSpec(10, 10, 6, 1.0, 0.0, 0.0, 0.0)
    autonomy = ClosedLoopAutonomy(
        AutonomyConfig(grid=grid, max_speed_mps=1.0, max_yaw_rate_rps=1.0, position_tolerance_m=0.1)
    )
    autonomy.update_pose(PoseSample(MetricPose(1.0, 1.0, 1.0, yaw=0.0), stamp_sec=1.0))
    autonomy.plan = DiscoveryPlan(MetricPose(5.0, 1.0, 1.0, yaw=0.5), ((2, 1, 1), (5, 1, 1)), None)

    command = autonomy.command()

    assert command is not None
    assert command.pose == MetricPose(2.0, 1.0, 1.0, yaw=0.5)
    assert command.velocity == (1.0, 0.0, 0.0)
    assert command.yaw_rate == 0.5


def test_yaw_from_px4_quaternion_matches_expected_heading():
    assert isclose(yaw_from_quaternion(0.0, 0.0, 0.0, 1.0), 0.0)
    assert isclose(yaw_from_quaternion(0.0, 0.0, 2**0.5 / 2.0, 2**0.5 / 2.0), pi / 2.0)


def test_closed_loop_telemetry_serializes_status_for_ros_publication():
    grid = VoxelGridSpec(8, 8, 4, 0.5, -2.0, -2.0, 0.0)
    autonomy = ClosedLoopAutonomy(AutonomyConfig(grid=grid, min_altitude_m=0.5))
    telemetry = autonomy.telemetry(now_sec=1.0)
    payload = telemetry_to_json(telemetry)
    assert '"ready": false' in payload
    assert '"hold_reason": "missing_pose"' in payload
    assert '"known_ratio": 0.0' in payload
