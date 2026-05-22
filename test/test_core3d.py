from auto_drone.core3d import (
    ActiveMappingSim3D,
    BeliefVolume,
    Pose3D,
    VoxelWorld,
    reachable_voxels,
    sensor_rays3d,
    visible_voxels,
    visible_voxels_from_belief,
)
from auto_drone.mapping3d import confidence_from_reprojection_error, integrate_range_frame
from auto_drone.odometry3d import NoisyOdometry
from auto_drone.sensing3d import RangeFrame, RangeMeasurement, generate_range_frame, observed_cells_from_pose
from auto_drone.gazebo_lidar_bridge import GazeboLidarMapper, pose_from_metric, scan_to_range_frame
from auto_drone.slam3d import PoseGraph
from math import pi
from random import Random


def test_3d_observation_reduces_uncertainty():
    sim = ActiveMappingSim3D(width=20, height=20, depth=10, sensor_radius=5, sensor_noise=0.0, seed=4)
    before = sim.belief.mean_uncertainty()
    sim.step()
    assert sim.belief.mean_uncertainty() < before
    assert sim.visible_cells
    assert sim.last_range_frame is not None
    assert sim.last_mean_reprojection_error >= 0.0


def test_3d_reachable_voxels_include_vertical_motion():
    belief = BeliefVolume(8, 8, 6)
    for z in range(1, 5):
        belief.observe(3, 3, z, occupied=False)
        belief.observe(3, 3, z, occupied=False)
    _, distances = reachable_voxels((3, 3, 1), belief)
    assert (3, 3, 4) in distances
    assert distances[(3, 3, 4)] == 3


def test_3d_belief_visibility_stops_at_believed_obstacle():
    belief = BeliefVolume(12, 8, 6)
    for x in range(1, 10):
        belief.observe(x, 4, 3, occupied=False)
        belief.observe(x, 4, 3, occupied=False)
    for _ in range(5):
        belief.observe(5, 4, 3, occupied=True)
    visible = visible_voxels_from_belief(belief, Pose3D(2, 4, 3, 0.0, 0.0, 0.0), radius=8, fov=0.2)
    assert (5, 4, 3) in visible
    assert (7, 4, 3) not in visible


def test_3d_planner_generates_6d_target_pose():
    sim = ActiveMappingSim3D(width=20, height=20, depth=10, sensor_radius=5, sensor_noise=0.0, seed=5)
    for _ in range(4):
        result = sim.step()
    assert result.target is not None
    assert isinstance(result.target.roll, float)
    assert isinstance(result.target.pitch, float)
    assert isinstance(result.target.yaw, float)


def test_3d_sensor_rays_are_cached():
    sensor_rays3d.cache_clear()
    first = sensor_rays3d(5, 1.0, 0.0, 0.0)
    second = sensor_rays3d(5, 1.0, 0.0, 0.0)
    assert first is second
    assert sensor_rays3d.cache_info().hits >= 1


def test_3d_sensor_observes_bounded_cone():
    sim = ActiveMappingSim3D(width=18, height=18, depth=10, sensor_radius=4, sensor_fov_degrees=45, sensor_noise=0.0)
    sim.pose = Pose3D(8, 8, 5, 0.0, 0.0, 0.0)
    visible = visible_voxels(sim.world, sim.pose, sim.sensor_radius, sim.sensor_fov)
    assert (10, 8, 5) in visible
    assert (8, 10, 5) not in visible
    assert all((x - 8) ** 2 + (y - 8) ** 2 + (z - 5) ** 2 <= sim.sensor_radius**2 for x, y, z in visible)


def test_range_frame_is_sensor_like_not_direct_occupancy():
    world = VoxelWorld(18, 18, 10, obstacle_density=0.08, seed=7)
    pose = Pose3D(9, 9, 5, 0.0, 0.0, 0.0)
    frame = generate_range_frame(world, pose, pose, radius=5, fov=0.8, noise=0.0, random=Random(3))
    assert frame.measurements
    assert observed_cells_from_pose(pose, frame)
    assert not hasattr(frame, "true_pose")
    assert all(
        abs(dx) <= 5 and abs(dy) <= 5 and abs(dz) <= 5
        for measurement in frame.measurements
        for dx, dy, dz in measurement.cells
    )
    assert all(measurement.range_voxels >= 0.0 for measurement in frame.measurements)


def test_mapper_integrates_from_estimated_pose_without_true_pose():
    low_error = confidence_from_reprojection_error(0.05)
    high_error = confidence_from_reprojection_error(1.0)
    assert low_error > high_error

    frame = RangeFrame(
        estimated_pose=Pose3D(4, 2, 2),
        measurements=(
            RangeMeasurement(
                ray=((1, 0, 0), (2, 0, 0)),
                cells=((1, 0, 0), (2, 0, 0)),
                hit=True,
                hit_cell=(2, 0, 0),
                range_voxels=2.0,
                reprojection_error=0.05,
            ),
        ),
    )
    belief = BeliefVolume(8, 8, 6)
    mapped = integrate_range_frame(belief, frame)
    assert (5, 2, 2) in mapped
    assert (6, 2, 2) in mapped
    assert belief.occupancy_probability(5, 2, 2) < 0.5
    assert belief.occupancy_probability(6, 2, 2) > 0.5

    turned_frame = RangeFrame(
        estimated_pose=Pose3D(4, 2, 2, 0.0, 0.0, pi / 2.0),
        measurements=(
            RangeMeasurement(
                ray=((1, 0, 0), (2, 0, 0)),
                cells=((1, 0, 0), (2, 0, 0)),
                hit=True,
                hit_cell=(2, 0, 0),
                range_voxels=2.0,
                reprojection_error=0.05,
            ),
        ),
    )
    turned_belief = BeliefVolume(8, 8, 6)
    turned_mapped = integrate_range_frame(turned_belief, turned_frame)
    assert (4, 3, 2) in turned_mapped
    assert (4, 4, 2) in turned_mapped


def test_noisy_odometry_tracks_estimated_pose_separately():
    odom = NoisyOdometry(Random(1), translational_noise=0.0, rotational_noise=0.0, drift_per_step=0.0)
    start = Pose3D(4, 4, 4)
    odom.reset(start)
    estimated = odom.update(start, Pose3D(5, 4, 4, 0.0, 0.0, 0.3))
    assert estimated.x == 5
    assert estimated.y == 4
    assert estimated.z == 4
    assert estimated.yaw == 0.3


def test_pose_graph_adds_keyframes_and_constraints():
    graph = PoseGraph()
    first = graph.maybe_add_keyframe(0, Pose3D(1, 1, 1), 0.2, 10, min_step_gap=1)
    second = graph.maybe_add_keyframe(2, Pose3D(2, 1, 1, 0.0, 0.0, 0.1), 0.4, 12, min_step_gap=1)
    assert first is not None
    assert second is not None
    assert len(graph.keyframes) == 2
    assert len(graph.constraints) == 1
    assert 0.29 < graph.latest_residual() < 0.31


def test_gazebo_lidar_bridge_converts_scan_to_local_range_frame():
    pose = pose_from_metric(1.0, -0.5, 0.0, pi / 2.0, resolution=0.5, origin=(10, 10, 3))
    assert pose == Pose3D(12, 9, 3, 0.0, 0.0, pi / 2.0)
    elevated_pose = pose_from_metric(
        1.0,
        -0.5,
        0.0,
        pi / 2.0,
        resolution=0.5,
        origin=(10, 10, 3),
        sensor_z_offset_meters=1.0,
    )
    assert elevated_pose == Pose3D(12, 9, 5, 0.0, 0.0, pi / 2.0)

    frame = scan_to_range_frame(
        ranges=(1.0, 4.0),
        angle_min=0.0,
        angle_increment=pi / 2.0,
        range_max=4.0,
        estimated_pose=pose,
        resolution=0.5,
        max_range_voxels=8,
    )
    assert len(frame.measurements) == 2
    assert frame.measurements[0].hit
    assert frame.measurements[0].hit_cell == (2, 0, 0)
    assert not frame.measurements[1].hit


def test_gazebo_lidar_mapper_updates_belief_from_scan():
    mapper = GazeboLidarMapper(32, 32, 8, resolution=0.5, origin=(16, 16, 4), sensor_z_offset_meters=1.0)
    mapper.update_pose(0.0, 0.0, 0.0, 0.0)
    mapped = mapper.integrate_scan((1.0,), angle_min=0.0, angle_increment=1.0, range_max=4.0)
    assert (17, 16, 6) in mapped
    assert (18, 16, 6) in mapped
    assert mapper.belief.occupancy_probability(17, 16, 6) < 0.5
    assert mapper.belief.occupancy_probability(18, 16, 6) > 0.5
