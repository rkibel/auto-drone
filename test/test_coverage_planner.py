import pytest

from auto_drone.coverage_planner import (
    ExplorationBounds,
    frontier_clusters,
    map_coverage,
    plan_frontier_route,
    trajectory_is_safe,
)
from auto_drone.interfaces import MetricPose, VoxelGridSpec
from auto_drone.mapping import BeliefVolume


def test_unknown_trajectory_is_allowed_when_clear() -> None:
    grid = VoxelGridSpec(12, 12, 12, 1.0, 0.0, 0.0, 0.0)
    belief = BeliefVolume(12, 12, 12)
    start = MetricPose(1.0, 1.0, 3.0)
    target = MetricPose(5.0, 1.0, 3.0)

    assert trajectory_is_safe(belief, grid, start, target, ExplorationBounds(0, 11, 0, 11, 0, 11), 1)


def test_trajectory_rejects_clearance_collision() -> None:
    grid = VoxelGridSpec(12, 12, 12, 1.0, 0.0, 0.0, 0.0)
    belief = BeliefVolume(12, 12, 12)
    belief.log_odds[belief.index(3, 2, 3)] = 3.0

    assert not trajectory_is_safe(
        belief,
        grid,
        MetricPose(1.0, 1.0, 3.0),
        MetricPose(5.0, 1.0, 3.0),
        ExplorationBounds(0, 11, 0, 11, 0, 11),
        1,
    )


def test_single_occupied_observation_blocks_a_planning_trajectory() -> None:
    grid = VoxelGridSpec(12, 12, 12, 1.0, 0.0, 0.0, 0.0)
    belief = BeliefVolume(12, 12, 12)
    belief.observe(3, 2, 3, occupied=True)

    assert belief.occupancy_probability(3, 2, 3) < 0.9
    assert not trajectory_is_safe(
        belief,
        grid,
        MetricPose(1.0, 1.0, 3.0),
        MetricPose(5.0, 1.0, 3.0),
        ExplorationBounds(0, 11, 0, 11, 0, 11),
        1,
    )


def test_bounds_derive_a_safe_inset_from_the_map() -> None:
    grid = VoxelGridSpec(20, 30, 12, 0.5, -5.0, -7.0, 0.0)

    assert ExplorationBounds.from_grid(grid) == ExplorationBounds(-4.0, 3.5, -6.0, 6.5, 2.0, 4.5)


def test_bounds_support_a_larger_horizontal_braking_inset() -> None:
    grid = VoxelGridSpec(20, 30, 12, 0.5, -5.0, -7.0, 0.0)

    assert ExplorationBounds.from_grid(grid, margin_m=3.0, vertical_margin_m=1.0) == ExplorationBounds(
        -2.0,
        1.5,
        -4.0,
        4.5,
        2.0,
        4.5,
    )


def test_frontier_clusters_use_free_unknown_boundaries() -> None:
    grid = VoxelGridSpec(12, 12, 8, 1.0, 0.0, 0.0, 0.0)
    belief = BeliefVolume(12, 12, 8)
    for z in range(1, 5):
        for y in range(3, 7):
            for x in range(3, 7):
                belief.log_odds[belief.index(x, y, z)] = -2.0

    clusters = frontier_clusters(belief, grid, ExplorationBounds(1, 10, 1, 10, 2, 6))

    assert clusters
    assert sum(cluster.cell_count for cluster in clusters) > 0
    assert all(cluster.score > 0.0 for cluster in clusters)


def test_frontier_plan_returns_a_safe_route_without_tree_references() -> None:
    grid = VoxelGridSpec(16, 16, 8, 1.0, 0.0, 0.0, 0.0)
    belief = BeliefVolume(16, 16, 8)
    for z in range(1, 6):
        for y in range(2, 14):
            for x in range(2, 14):
                belief.log_odds[belief.index(x, y, z)] = -2.0
    belief.log_odds[belief.index(8, 8, 3)] = 3.0
    bounds = ExplorationBounds(1, 14, 1, 14, 2, 6)
    current = MetricPose(3.0, 3.0, 3.0)

    plan = plan_frontier_route(belief, grid, current, bounds)

    assert plan.frontier_count > 0
    assert plan.waypoints
    assert all(
        trajectory_is_safe(belief, grid, start, target, bounds, 2)
        for start, target in zip((current, *plan.waypoints), plan.waypoints)
    )


def test_frontier_plan_searches_past_recently_visited_top_clusters() -> None:
    grid = VoxelGridSpec(32, 32, 8, 1.0, 0.0, 0.0, 0.0)
    belief = BeliefVolume(32, 32, 8)
    for z in range(1, 6):
        for y in range(2, 30):
            for x in range(2, 30):
                belief.log_odds[belief.index(x, y, z)] = -2.0
    bounds = ExplorationBounds(1, 30, 1, 30, 2, 6)
    clusters = frontier_clusters(belief, grid, bounds)
    excluded = tuple(cluster.center for cluster in clusters[:4])

    plan = plan_frontier_route(
        belief,
        grid,
        MetricPose(16.0, 16.0, 3.0),
        bounds,
        excluded_targets=excluded,
    )

    assert len(clusters) > 4
    assert plan.waypoints
    assert all(
        (plan.waypoints[-1].x - target.x) ** 2 + (plan.waypoints[-1].y - target.y) ** 2 >= 16.0
        for target in excluded
    )


def test_map_coverage_reports_known_and_floor_occupied_cells() -> None:
    belief = BeliefVolume(4, 4, 4)
    for _ in range(3):
        belief.observe(1, 2, 0, occupied=True)

    coverage = map_coverage(belief)

    assert coverage.floor_occupied_ratio == pytest.approx(1.0 / 16.0)
    assert coverage.known_ratio > 0.0
