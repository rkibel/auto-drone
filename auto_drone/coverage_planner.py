from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from math import ceil, log, sqrt

import numpy as np

from auto_drone.interfaces import MetricPose, VoxelGridSpec
from auto_drone.mapping import BeliefVolume, OCCUPIED_PROBABILITY_THRESHOLD, line_voxels

FREE_LOG_ODDS_THRESHOLD = log(0.35 / 0.65)
OCCUPIED_LOG_ODDS_THRESHOLD = log(OCCUPIED_PROBABILITY_THRESHOLD / (1.0 - OCCUPIED_PROBABILITY_THRESHOLD))
PLANNING_OCCUPIED_PROBABILITY_THRESHOLD = 0.65
PLANNING_OCCUPIED_LOG_ODDS_THRESHOLD = log(
    PLANNING_OCCUPIED_PROBABILITY_THRESHOLD / (1.0 - PLANNING_OCCUPIED_PROBABILITY_THRESHOLD)
)
NAVIGATION_VOXEL_SIZE = 2
DEFAULT_CLEARANCE_VOXELS = 4
MAX_FRONTIER_CANDIDATES = 4
MAX_ASTAR_CANDIDATES = 2
RECENT_TARGET_EXCLUSION_DISTANCE_SQUARED = 16.0
UNKNOWN_TRAVERSAL_COST = 3.0
FLOOR_FRONTIER_WEIGHT = 6.0
SURFACE_FRONTIER_WEIGHT = 2.0
NAVIGATION_OFFSETS = tuple(
    (dx, dy, dz, sqrt(dx * dx + dy * dy + dz * dz))
    for dz in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dx in (-1, 0, 1)
    if not (dx == dy == dz == 0)
)


@dataclass(frozen=True)
class ExplorationBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float

    @classmethod
    def from_grid(
        cls,
        grid: VoxelGridSpec,
        margin_m: float = 1.0,
        minimum_flight_height_m: float = 2.0,
        vertical_margin_m: float | None = None,
    ) -> ExplorationBounds:
        vertical_margin = margin_m if vertical_margin_m is None else vertical_margin_m
        return cls(
            grid.origin_x + margin_m,
            grid.origin_x + (grid.width - 1) * grid.resolution - margin_m,
            grid.origin_y + margin_m,
            grid.origin_y + (grid.height - 1) * grid.resolution - margin_m,
            max(grid.origin_z + vertical_margin, minimum_flight_height_m),
            grid.origin_z + (grid.depth - 1) * grid.resolution - vertical_margin,
        )

    def contains(self, pose: MetricPose) -> bool:
        return (
            self.min_x <= pose.x <= self.max_x
            and self.min_y <= pose.y <= self.max_y
            and self.min_z <= pose.z <= self.max_z
        )

    def clamp(self, pose: MetricPose) -> MetricPose:
        return MetricPose(
            min(max(pose.x, self.min_x), self.max_x),
            min(max(pose.y, self.min_y), self.max_y),
            min(max(pose.z, self.min_z), self.max_z),
            yaw=pose.yaw,
        )


@dataclass(frozen=True)
class FrontierCluster:
    center: MetricPose
    score: float
    cell_count: int


@dataclass(frozen=True)
class FrontierPlan:
    waypoints: tuple[MetricPose, ...]
    frontier_count: int
    score: float = 0.0


@dataclass(frozen=True)
class MapCoverage:
    known_ratio: float
    floor_occupied_ratio: float

    @property
    def score(self) -> float:
        return self.known_ratio + self.floor_occupied_ratio * 0.25


def map_coverage(belief: BeliefVolume) -> MapCoverage:
    floor_cells = belief.width * belief.height
    floor_occupied = sum(
        belief.log_odds[belief.index(x, y, 0)] > OCCUPIED_LOG_ODDS_THRESHOLD
        for y in range(belief.height)
        for x in range(belief.width)
    )
    return MapCoverage(belief.known_ratio(), floor_occupied / floor_cells)


def frontier_clusters(
    belief: BeliefVolume,
    grid: VoxelGridSpec,
    bounds: ExplorationBounds,
) -> list[FrontierCluster]:
    odds = np.asarray(belief.log_odds).reshape((belief.depth, belief.height, belief.width))
    free = odds < FREE_LOG_ODDS_THRESHOLD
    unknown = (odds >= FREE_LOG_ODDS_THRESHOLD) & (odds <= OCCUPIED_LOG_ODDS_THRESHOLD)
    occupied = odds > OCCUPIED_LOG_ODDS_THRESHOLD
    surface_unknown = np.zeros_like(unknown)
    for offset in neighbor_offsets():
        source, target = shifted_slices(odds.shape, (offset[2], offset[1], offset[0]))
        surface_unknown[source] |= unknown[source] & occupied[target]
    unknown_faces = np.zeros_like(odds, dtype=np.int8)
    surface_faces = np.zeros_like(odds, dtype=np.int8)
    for offset in neighbor_offsets():
        source, target = shifted_slices(odds.shape, (offset[2], offset[1], offset[0]))
        faces = free[source] & unknown[target]
        unknown_faces[source] += faces
        surface_faces[source] += faces & surface_unknown[target]
    floor_faces = np.zeros_like(odds, dtype=np.int8)
    if belief.depth > 1:
        floor_faces[1] = free[1] & unknown[0]

    z, y, x = np.nonzero(unknown_faces)
    if len(x) == 0:
        return []
    bin_width = ceil(belief.width / 4)
    bin_height = ceil(belief.height / 4)
    keys = (z // 2) * bin_height * bin_width + (y // 4) * bin_width + x // 4
    bin_count = np.bincount(keys)
    total_x = np.bincount(keys, weights=x)
    total_y = np.bincount(keys, weights=y)
    total_z = np.bincount(keys, weights=z)
    total_unknown = np.bincount(keys, weights=unknown_faces[z, y, x])
    total_floor = np.bincount(keys, weights=floor_faces[z, y, x])
    total_surface = np.bincount(keys, weights=surface_faces[z, y, x])
    clusters = []
    for key in np.flatnonzero(bin_count):
        count = bin_count[key]
        center = grid.voxel_to_metric(
            round(total_x[key] / count),
            round(total_y[key] / count),
            round(total_z[key] / count),
        )
        center = MetricPose(center.x, center.y, min(max(center.z, bounds.min_z), bounds.max_z))
        if not bounds.contains(center):
            continue
        score = (
            total_unknown[key]
            + total_floor[key] * FLOOR_FRONTIER_WEIGHT
            + total_surface[key] * SURFACE_FRONTIER_WEIGHT
        )
        clusters.append(FrontierCluster(center, score, int(count)))
    return sorted(clusters, key=lambda cluster: cluster.score, reverse=True)


def plan_frontier_route(
    belief: BeliefVolume,
    grid: VoxelGridSpec,
    current: MetricPose,
    bounds: ExplorationBounds,
    clearance_voxels: int = DEFAULT_CLEARANCE_VOXELS,
    excluded_targets: tuple[MetricPose, ...] = (),
) -> FrontierPlan:
    clusters = frontier_clusters(belief, grid, bounds)
    if not clusters:
        return FrontierPlan((), 0)
    blocked = blocked_navigation_cells(belief, clearance_voxels)
    limits = navigation_limits(grid, bounds)
    start = metric_to_navigation_cell(current, grid)
    best: FrontierPlan | None = None
    astar_attempts = 0
    candidate_count = 0
    for cluster in clusters:
        if any(
            distance_squared(cluster.center, target) < RECENT_TARGET_EXCLUSION_DISTANCE_SQUARED
            for target in excluded_targets
        ):
            continue
        candidate_count += 1
        if candidate_count > MAX_FRONTIER_CANDIDATES:
            break
        goal = nearest_navigation_cell(metric_to_navigation_cell(cluster.center, grid), limits, blocked)
        if goal is None:
            continue
        target = navigation_cell_to_metric(goal, grid)
        if trajectory_is_safe(belief, grid, current, target, bounds, clearance_voxels):
            waypoints = (target,)
        else:
            if astar_attempts == MAX_ASTAR_CANDIDATES:
                continue
            astar_attempts += 1
            cells = navigation_path(belief, grid, start, goal, limits, blocked, allow_unknown=False)
            if not cells:
                cells = navigation_path(belief, grid, start, goal, limits, blocked, allow_unknown=True)
            if not cells:
                continue
            path = tuple(navigation_cell_to_metric(cell, grid) for cell in cells[1:])
            waypoints = simplify_path(belief, grid, current, path, bounds, clearance_voxels)
        if not waypoints:
            continue
        route_length = path_length(current, waypoints)
        score = cluster.score - route_length * 0.25
        if best is None or score > best.score:
            best = FrontierPlan(waypoints, sum(cluster.cell_count for cluster in clusters), score)
    return best if best is not None else FrontierPlan((), sum(cluster.cell_count for cluster in clusters))


def trajectory_is_safe(
    belief: BeliefVolume,
    grid: VoxelGridSpec,
    start: MetricPose,
    target: MetricPose,
    bounds: ExplorationBounds,
    clearance_voxels: int,
) -> bool:
    if not bounds.contains(target):
        return False
    start_voxel = grid.metric_to_voxel(start)
    target_voxel = grid.metric_to_voxel(target)
    for cell in line_voxels(start_voxel.x, start_voxel.y, start_voxel.z, target_voxel.x, target_voxel.y, target_voxel.z):
        if not belief.in_bounds(*cell) or not cell_has_clearance(belief, cell, clearance_voxels):
            return False
    return True


def simplify_path(
    belief: BeliefVolume,
    grid: VoxelGridSpec,
    current: MetricPose,
    path: tuple[MetricPose, ...],
    bounds: ExplorationBounds,
    clearance_voxels: int,
) -> tuple[MetricPose, ...]:
    simplified = []
    anchor = current
    next_index = 0
    while next_index < len(path):
        for index in range(len(path) - 1, next_index - 1, -1):
            if trajectory_is_safe(belief, grid, anchor, path[index], bounds, clearance_voxels):
                simplified.append(path[index])
                anchor = path[index]
                next_index = index + 1
                break
        else:
            return ()
    return tuple(simplified)


def blocked_navigation_cells(belief: BeliefVolume, clearance_voxels: int) -> set[tuple[int, int, int]]:
    blocked = set()
    clearance_cells = ceil(clearance_voxels / NAVIGATION_VOXEL_SIZE)
    for z in range(belief.depth):
        for y in range(belief.height):
            for x in range(belief.width):
                if not is_occupied(belief, x, y, z):
                    continue
                cell = (x // NAVIGATION_VOXEL_SIZE, y // NAVIGATION_VOXEL_SIZE, z // NAVIGATION_VOXEL_SIZE)
                for dz in range(-clearance_cells, clearance_cells + 1):
                    for dy in range(-clearance_cells, clearance_cells + 1):
                        for dx in range(-clearance_cells, clearance_cells + 1):
                            blocked.add((cell[0] + dx, cell[1] + dy, cell[2] + dz))
    return blocked


def navigation_path(
    belief: BeliefVolume,
    grid: VoxelGridSpec,
    start: tuple[int, int, int],
    goal: tuple[int, int, int],
    limits: tuple[tuple[int, int, int], tuple[int, int, int]],
    blocked: set[tuple[int, int, int]],
    allow_unknown: bool,
) -> tuple[tuple[int, int, int], ...]:
    if start in blocked or goal in blocked:
        return ()
    queue = [(navigation_distance(start, goal), 0.0, start)]
    previous: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start: None}
    costs = {start: 0.0}
    traversal_costs: dict[tuple[int, int, int], float | None] = {}
    while queue:
        _, cost, cell = heappop(queue)
        if cell == goal:
            return rebuild_path(previous, goal)
        if cost != costs[cell]:
            continue
        for neighbor, step_cost in navigation_neighbors(cell, limits):
            if neighbor in blocked:
                continue
            if neighbor not in traversal_costs:
                traversal_costs[neighbor] = navigation_cost(belief, grid, neighbor, allow_unknown)
            traversal_cost = traversal_costs[neighbor]
            if traversal_cost is None:
                continue
            next_cost = cost + step_cost * traversal_cost
            if next_cost >= costs.get(neighbor, float("inf")):
                continue
            costs[neighbor] = next_cost
            previous[neighbor] = cell
            heappush(queue, (next_cost + navigation_distance(neighbor, goal), next_cost, neighbor))
    return ()


def nearest_navigation_cell(
    cell: tuple[int, int, int],
    limits: tuple[tuple[int, int, int], tuple[int, int, int]],
    blocked: set[tuple[int, int, int]],
) -> tuple[int, int, int] | None:
    for radius in range(4):
        for dz in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    candidate = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                    if candidate not in blocked and navigation_cell_in_bounds(candidate, limits):
                        return candidate
    return None


def navigation_neighbors(
    cell: tuple[int, int, int],
    limits: tuple[tuple[int, int, int], tuple[int, int, int]],
):
    for dx, dy, dz, step_cost in NAVIGATION_OFFSETS:
        neighbor = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
        if navigation_cell_in_bounds(neighbor, limits):
            yield neighbor, step_cost


def navigation_cell_in_bounds(
    cell: tuple[int, int, int],
    limits: tuple[tuple[int, int, int], tuple[int, int, int]],
) -> bool:
    minimum, maximum = limits
    return (
        minimum[0] <= cell[0] <= maximum[0]
        and minimum[1] <= cell[1] <= maximum[1]
        and minimum[2] <= cell[2] <= maximum[2]
    )


def navigation_cost(
    belief: BeliefVolume,
    grid: VoxelGridSpec,
    cell: tuple[int, int, int],
    allow_unknown: bool,
) -> float | None:
    x = min(grid.width - 1, cell[0] * NAVIGATION_VOXEL_SIZE + NAVIGATION_VOXEL_SIZE // 2)
    y = min(grid.height - 1, cell[1] * NAVIGATION_VOXEL_SIZE + NAVIGATION_VOXEL_SIZE // 2)
    z = min(grid.depth - 1, cell[2] * NAVIGATION_VOXEL_SIZE + NAVIGATION_VOXEL_SIZE // 2)
    if is_free(belief, x, y, z):
        return 1.0
    return UNKNOWN_TRAVERSAL_COST if allow_unknown else None


def metric_to_navigation_cell(pose: MetricPose, grid: VoxelGridSpec) -> tuple[int, int, int]:
    voxel = grid.metric_to_voxel(pose)
    return (
        max(0, min(ceil(grid.width / NAVIGATION_VOXEL_SIZE) - 1, voxel.x // NAVIGATION_VOXEL_SIZE)),
        max(0, min(ceil(grid.height / NAVIGATION_VOXEL_SIZE) - 1, voxel.y // NAVIGATION_VOXEL_SIZE)),
        max(0, min(ceil(grid.depth / NAVIGATION_VOXEL_SIZE) - 1, voxel.z // NAVIGATION_VOXEL_SIZE)),
    )


def navigation_limits(
    grid: VoxelGridSpec,
    bounds: ExplorationBounds,
) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    return (
        metric_to_navigation_cell(MetricPose(bounds.min_x, bounds.min_y, bounds.min_z), grid),
        metric_to_navigation_cell(MetricPose(bounds.max_x, bounds.max_y, bounds.max_z), grid),
    )


def navigation_cell_to_voxel(cell: tuple[int, int, int], grid: VoxelGridSpec) -> tuple[int, int, int]:
    return tuple(
        min(size - 1, coordinate * NAVIGATION_VOXEL_SIZE + NAVIGATION_VOXEL_SIZE // 2)
        for coordinate, size in zip(cell, (grid.width, grid.height, grid.depth))
    )


def navigation_cell_to_metric(cell: tuple[int, int, int], grid: VoxelGridSpec) -> MetricPose:
    return grid.voxel_to_metric(*navigation_cell_to_voxel(cell, grid))


def rebuild_path(
    previous: dict[tuple[int, int, int], tuple[int, int, int] | None],
    goal: tuple[int, int, int],
) -> tuple[tuple[int, int, int], ...]:
    path = []
    cell: tuple[int, int, int] | None = goal
    while cell is not None:
        path.append(cell)
        cell = previous[cell]
    return tuple(reversed(path))


def navigation_distance(first: tuple[int, int, int], second: tuple[int, int, int]) -> float:
    return sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def path_length(current: MetricPose, path: tuple[MetricPose, ...]) -> float:
    total = 0.0
    previous = current
    for pose in path:
        total += sqrt(distance_squared(previous, pose))
        previous = pose
    return total


def distance_squared(first: MetricPose, second: MetricPose) -> float:
    return (first.x - second.x) ** 2 + (first.y - second.y) ** 2 + (first.z - second.z) ** 2


def cell_has_clearance(belief: BeliefVolume, cell: tuple[int, int, int], clearance_voxels: int) -> bool:
    x, y, z = cell
    for dz in range(-clearance_voxels, clearance_voxels + 1):
        for dy in range(-clearance_voxels, clearance_voxels + 1):
            for dx in range(-clearance_voxels, clearance_voxels + 1):
                neighbor = (x + dx, y + dy, z + dz)
                if belief.in_bounds(*neighbor) and is_occupied(belief, *neighbor):
                    return False
    return True


def is_free(belief: BeliefVolume, x: int, y: int, z: int) -> bool:
    return belief.log_odds[belief.index(x, y, z)] < FREE_LOG_ODDS_THRESHOLD


def is_occupied(belief: BeliefVolume, x: int, y: int, z: int) -> bool:
    return belief.log_odds[belief.index(x, y, z)] > PLANNING_OCCUPIED_LOG_ODDS_THRESHOLD


def neighbor_offsets() -> tuple[tuple[int, int, int], ...]:
    return ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))


def shifted_slices(
    shape: tuple[int, int, int],
    offset: tuple[int, int, int],
) -> tuple[tuple[slice, slice, slice], tuple[slice, slice, slice]]:
    source = []
    target = []
    for size, delta in zip(shape, offset):
        if delta >= 0:
            source.append(slice(0, size - delta))
            target.append(slice(delta, size))
        else:
            source.append(slice(-delta, size))
            target.append(slice(0, size + delta))
    return tuple(source), tuple(target)
