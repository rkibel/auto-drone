from math import pi

from auto_drone.interfaces import CameraIntrinsics, MetricPose, VoxelGridSpec, VoxelPose
from auto_drone.mapping import (
    BeliefVolume,
    OCCUPIED_PROBABILITY_THRESHOLD,
    RangeFrame,
    RangeMeasurement,
    depth_image_to_range_frame,
    integrate_range_frame,
    stride_sample_offset,
    view_color_image,
    view_depth_image,
)


def test_ground_hit_updates_the_occupancy_map() -> None:
    belief = BeliefVolume(5, 5, 5)
    frame = RangeFrame(
        VoxelPose(1, 1, 1),
        (RangeMeasurement(((2, 1, 0), (3, 1, 0)), True),),
    )

    integrate_range_frame(belief, frame)
    integrate_range_frame(belief, frame)
    integrate_range_frame(belief, frame)

    assert belief.occupancy_probability(3, 1, 0) > OCCUPIED_PROBABILITY_THRESHOLD


def test_above_ground_hit_is_integrated() -> None:
    belief = BeliefVolume(5, 5, 5)
    frame = RangeFrame(
        VoxelPose(1, 1, 1),
        (RangeMeasurement(((2, 1, 2), (3, 1, 2)), True),),
    )

    integrate_range_frame(belief, frame)
    integrate_range_frame(belief, frame)
    integrate_range_frame(belief, frame)

    assert belief.occupancy_probability(3, 1, 2) > OCCUPIED_PROBABILITY_THRESHOLD


def test_single_occupied_observation_is_not_published_as_an_obstacle() -> None:
    belief = BeliefVolume(1, 1, 1)

    belief.observe(0, 0, 0, occupied=True)

    assert belief.occupancy_probability(0, 0, 0) <= OCCUPIED_PROBABILITY_THRESHOLD


def test_two_occupied_observations_are_not_published_as_an_obstacle() -> None:
    belief = BeliefVolume(1, 1, 1)

    belief.observe(0, 0, 0, occupied=True)
    belief.observe(0, 0, 0, occupied=True)

    assert belief.occupancy_probability(0, 0, 0) <= OCCUPIED_PROBABILITY_THRESHOLD


def test_image_views_handle_row_padding() -> None:
    assert view_color_image(bytes((3, 2, 1, 6, 5, 4, 0, 0)), 2, 1, "bgr8", 8).tolist() == [
        [1, 2, 3],
        [4, 5, 6],
    ]
    assert view_depth_image(
        bytes((0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 0, 0)), 2, 1, "32FC1", 12
    ).tolist() == [1.0, 2.0]


def test_depth_hit_carries_its_sampled_color() -> None:
    frame = depth_image_to_range_frame(
        [1.0],
        CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0),
        MetricPose(0.0, 0.0, 1.0),
        VoxelGridSpec(10, 10, 10, 1.0, 0.0, 0.0, 0.0),
        0.25,
        8.0,
        stride=1,
        colors=[(12, 34, 56)],
    )

    assert frame.measurements[0].hit_color == (12, 34, 56)


def test_staggered_stride_offsets_cover_every_pixel_phase() -> None:
    intrinsics = CameraIntrinsics(4, 4, 100.0, 100.0, 1.5, 1.5)
    colors = [(index, 0, 0) for index in range(16)]
    sampled_colors = set()

    for offset in ((0, 0), (1, 0), (0, 1), (1, 1)):
        frame = depth_image_to_range_frame(
            [2.0] * 16,
            intrinsics,
            MetricPose(2.0, 2.0, 2.0),
            VoxelGridSpec(10, 10, 10, 0.5, 0.0, 0.0, 0.0),
            0.25,
            8.0,
            stride=2,
            colors=colors,
            sample_offset=offset,
        )
        sampled_colors.update(measurement.hit_color for measurement in frame.measurements)

    assert sampled_colors == set(colors)


def test_stride_sample_offsets_cover_a_full_period_without_repeating() -> None:
    assert {stride_sample_offset(frame, 8) for frame in range(64)} == {
        (u, v) for v in range(8) for u in range(8)
    }


def test_depth_endpoint_is_voxelized_after_world_projection() -> None:
    frame = depth_image_to_range_frame(
        [2.0],
        CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0),
        MetricPose(0.49, 0.49, 1.0, yaw=pi / 4.0),
        VoxelGridSpec(10, 10, 10, 1.0, 0.0, 0.0, 0.0),
        0.25,
        8.0,
        stride=1,
    )

    assert frame.measurements[0].cells[-1] == (2, 2, 1)


def test_max_range_depth_does_not_create_an_occupied_hit() -> None:
    frame = depth_image_to_range_frame(
        [8.0],
        CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0),
        MetricPose(0.0, 0.0, 1.0),
        VoxelGridSpec(10, 10, 10, 1.0, 0.0, 0.0, 0.0),
        0.25,
        8.0,
        stride=1,
        colors=[(12, 34, 56)],
    )

    assert not frame.measurements[0].hit
    assert frame.measurements[0].hit_color is None


def test_color_running_mean_uses_all_observations() -> None:
    belief = BeliefVolume(1, 1, 1)
    for color in ((20, 30, 40), (1, 2, 3), (20, 30, 40), (20, 30, 40)):
        belief.observe_color(0, 0, 0, color)

    assert belief.colors[belief.index(0, 0, 0)] == (15.25, 23.0, 30.75, 4)
