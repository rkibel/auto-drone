from collections import deque
from math import isfinite, pi

import pytest
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import Image
from auto_drone.interfaces import CameraIntrinsics, MetricPose, VoxelGridSpec
from auto_drone.mapping import depth_image_to_range_frame
from auto_drone.px4_control_node import (
    attitude_enu_from_px4_quaternion,
    camera_frustum_points,
    metric_pose_from_odometry,
    rate_limited_velocity,
    select_backtrack_target,
)
from auto_drone.rgbd import (
    camera_pose_from_body_pose,
    camera_pose_from_ground_truth_odometry,
    correct_ground_depths,
    exact_image_pairs,
    ground_truth_pose_from_odometry,
    image_timestamp_ns,
    refine_camera_pose_from_ground,
)


def test_rate_limited_velocity_caps_the_change() -> None:
    assert rate_limited_velocity((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), 0.075) == (0.075, 0.0, 0.0)


def test_backtrack_selects_the_nearest_prior_pose_with_enough_separation() -> None:
    current = MetricPose(0.0, 0.0, 3.0)
    candidates = (
        MetricPose(-4.0, 0.0, 3.0),
        MetricPose(-2.6, 0.0, 3.0),
        MetricPose(-1.0, 0.0, 3.0),
    )

    assert select_backtrack_target(current, candidates, 2.5) == candidates[1]


def test_px4_identity_attitude_maps_to_enu_heading() -> None:
    assert attitude_enu_from_px4_quaternion([1.0, 0.0, 0.0, 0.0]) == pytest.approx((0.0, 0.0, pi / 2.0))


def test_camera_frustum_has_eight_line_segments() -> None:
    assert len(camera_frustum_points(MetricPose(0.0, 0.0, 0.0))) == 16


def test_camera_pose_uses_full_vehicle_attitude() -> None:
    odometry = VehicleOdometry()
    odometry.position = [0.0, 0.0, -2.0]
    odometry.q = [1.0, 0.0, 0.0, 0.0]

    pose = camera_pose_from_body_pose(metric_pose_from_odometry(odometry))

    assert (pose.x, pose.y, pose.z, pose.yaw) == pytest.approx((0.0, 0.13233, 2.26078, pi / 2.0))


def test_image_and_ground_truth_timestamps_use_nanoseconds() -> None:
    image = Image()
    image.header.stamp.sec = 2
    image.header.stamp.nanosec = 345
    odometry = Odometry()
    odometry.header.stamp.sec = 2
    odometry.header.stamp.nanosec = 1_000

    assert image_timestamp_ns(image) == 2_000_000_345
    assert image_timestamp_ns(odometry) == 2_000_001_000


def test_exact_image_pairs_returns_newest_matching_frames_first() -> None:
    depths = deque()
    colors = deque()
    for stamp in (1, 2, 3):
        depth = Image(width=640, height=480)
        depth.header.stamp.nanosec = stamp
        depths.append(depth)
    for stamp in (1, 3):
        color = Image(width=640, height=480)
        color.header.stamp.nanosec = stamp
        colors.append(color)

    pairs = exact_image_pairs(depths, colors)

    assert [image_timestamp_ns(depth) for depth, _ in pairs] == [3, 1]


def test_ground_truth_camera_pose_uses_enu_odometry() -> None:
    odometry = Odometry()
    odometry.pose.pose.position.x = 1.0
    odometry.pose.pose.position.y = 2.0
    odometry.pose.pose.position.z = 3.0
    odometry.pose.pose.orientation.w = 1.0

    assert ground_truth_pose_from_odometry(odometry) == MetricPose(1.0, 2.0, 3.0)
    pose = camera_pose_from_ground_truth_odometry(odometry)
    assert (pose.x, pose.y, pose.z) == pytest.approx((1.13233, 2.0, 3.26078))


def test_ground_plane_refines_camera_height_and_attitude() -> None:
    intrinsics = CameraIntrinsics(5, 5, 5.0, 5.0, 2.0, -1.0)
    depths = []
    for v in range(5):
        depths.extend([2.0 * intrinsics.fy / (v + 1)] * 5)
    colors = [(180, 196, 201)] * 25

    pose = refine_camera_pose_from_ground(
        depths,
        colors,
        intrinsics,
        MetricPose(1.0, 2.0, 2.4, roll=0.1, pitch=-0.1, yaw=0.7),
        stride=1,
    )

    assert (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw) == pytest.approx(
        (1.0, 2.0, 2.0, 0.0, 0.0, 0.7)
    )


def test_ground_depth_is_corrected_to_the_fitted_plane() -> None:
    intrinsics = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, -1.0)

    corrected = correct_ground_depths(
        [2.4],
        [(180, 196, 201)],
        intrinsics,
        MetricPose(0.0, 0.0, 2.0),
        stride=1,
    )

    assert corrected == pytest.approx([2.0])


def test_ground_depth_is_corrected_on_a_staggered_sample_phase() -> None:
    intrinsics = CameraIntrinsics(2, 2, 1.0, 1.0, 0.0, -1.0)
    corrected = correct_ground_depths(
        [2.4] * 4,
        [(180, 196, 201)] * 4,
        intrinsics,
        MetricPose(0.0, 0.0, 2.0),
        stride=2,
        sample_offset=(0, 0),
    )

    assert corrected[0] == pytest.approx(2.0)
    frame = depth_image_to_range_frame(
        corrected,
        intrinsics,
        MetricPose(2.0, 2.0, 2.0),
        VoxelGridSpec(10, 10, 10, 0.5, 0.0, 0.0, 0.0),
        0.25,
        8.0,
        stride=2,
        colors=[(180, 196, 201)] * 4,
        sample_offset=(0, 0),
    )
    assert frame.measurements[0].cells[-1][2] == 0


def test_ground_ray_without_a_mappable_plane_intersection_is_discarded() -> None:
    intrinsics = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 1.0)

    corrected = correct_ground_depths(
        [4.0],
        [(180, 196, 201)],
        intrinsics,
        MetricPose(0.0, 0.0, 2.0),
        stride=1,
    )

    assert not isfinite(corrected[0])
