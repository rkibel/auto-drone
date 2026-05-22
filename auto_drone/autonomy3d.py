from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import atan2

from auto_drone.common import UNKNOWN
from auto_drone.core3d import BeliefVolume, neighbors6, neighbors26
from auto_drone.interfaces3d import MetricPose, VoxelGridSpec


@dataclass(frozen=True)
class DiscoveryPlan:
    target: MetricPose | None
    path: tuple[tuple[int, int, int], ...]
    stop_reason: str | None = None


def choose_discovery_plan(
    belief: BeliefVolume,
    grid: VoxelGridSpec,
    current_pose: MetricPose,
    clearance_voxels: int = 1,
    max_candidates: int = 32,
) -> DiscoveryPlan:
    start_pose = grid.metric_to_voxel(current_pose)
    start = (start_pose.x, start_pose.y, start_pose.z)
    if not belief.in_bounds(*start):
        return DiscoveryPlan(None, (), "pose_out_of_bounds")

    inflated = inflated_occupied_voxels(belief, clearance_voxels)
    came_from, distances = reachable_safe_voxels(start, belief, inflated)
    if not distances:
        return DiscoveryPlan(None, (), "no_safe_reachable_space")

    candidates = []
    for voxel, distance in distances.items():
        frontier = frontier_score(belief, voxel)
        if frontier <= 0.0:
            continue
        score = frontier / (1.0 + distance * 0.20)
        candidates.append((score, voxel))

    candidates.sort(reverse=True)
    if not candidates:
        return DiscoveryPlan(None, (), "no_discovery_frontier")

    best_voxel = candidates[:max_candidates][0][1]
    path = tuple(reconstruct_path(start, best_voxel, came_from))
    yaw = current_pose.yaw
    if path:
        nx, ny, _ = path[min(1, len(path) - 1)]
        yaw = atan2(ny - start[1], nx - start[0]) if (nx, ny) != (start[0], start[1]) else current_pose.yaw
    target = grid.voxel_to_metric(*best_voxel, yaw=yaw)
    return DiscoveryPlan(target, path, None)


def inflated_occupied_voxels(belief: BeliefVolume, clearance_voxels: int) -> set[tuple[int, int, int]]:
    inflated = set()
    for z in range(belief.depth):
        for y in range(belief.height):
            for x in range(belief.width):
                if belief.occupancy_probability(x, y, z) <= 0.65:
                    continue
                for dz in range(-clearance_voxels, clearance_voxels + 1):
                    for dy in range(-clearance_voxels, clearance_voxels + 1):
                        for dx in range(-clearance_voxels, clearance_voxels + 1):
                            cell = (x + dx, y + dy, z + dz)
                            if belief.in_bounds(*cell):
                                inflated.add(cell)
    return inflated


def reachable_safe_voxels(
    start: tuple[int, int, int],
    belief: BeliefVolume,
    inflated: set[tuple[int, int, int]],
) -> tuple[dict[tuple[int, int, int], tuple[int, int, int] | None], dict[tuple[int, int, int], int]]:
    if start in inflated:
        return {}, {}
    queue = deque([start])
    came_from: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start: None}
    distances = {start: 0}
    while queue:
        current = queue.popleft()
        for neighbor in neighbors6(*current):
            if neighbor in came_from or neighbor in inflated:
                continue
            if not belief.is_known_free(*neighbor):
                continue
            came_from[neighbor] = current
            distances[neighbor] = distances[current] + 1
            queue.append(neighbor)
    return came_from, distances


def frontier_score(belief: BeliefVolume, voxel: tuple[int, int, int]) -> float:
    score = 0.0
    for neighbor in neighbors26(*voxel):
        if belief.in_bounds(*neighbor) and belief.cell(*neighbor) == UNKNOWN:
            score += belief.uncertainty(*neighbor)
    return score


def reconstruct_path(
    start: tuple[int, int, int],
    goal: tuple[int, int, int],
    came_from: dict[tuple[int, int, int], tuple[int, int, int] | None],
) -> list[tuple[int, int, int]]:
    path = []
    current: tuple[int, int, int] | None = goal
    while current is not None and current != start:
        path.append(current)
        current = came_from.get(current)
    path.reverse()
    return path
