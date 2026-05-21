from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import pi
from random import Random

from auto_drone.common import FREE, OBSTACLE, UNKNOWN, angle_delta, clamp
from auto_drone.geometry3d import Pose3D, orientation_cost, orientation_to, sensor_rays3d
from auto_drone.mapping3d import integrate_range_frame
from auto_drone.odometry3d import NoisyOdometry
from auto_drone.sensing3d import RangeFrame, generate_range_frame
from auto_drone.slam3d import PoseGraph


@dataclass
class StepResult3D:
    step: int
    pose: Pose3D
    estimated_pose: Pose3D
    target: Pose3D | None
    path: list[tuple[int, int, int]]
    known_ratio: float
    mean_uncertainty: float
    mean_reprojection_error: float
    keyframe_count: int
    stop_reason: str | None
    done: bool


class VoxelWorld:
    def __init__(self, width: int, height: int, depth: int, obstacle_density: float, seed: int):
        self.width = width
        self.height = height
        self.depth = depth
        self.random = Random(seed)
        self.cells = [FREE] * (width * height * depth)
        self._generate(seed, obstacle_density)

    def index(self, x: int, y: int, z: int) -> int:
        return z * self.width * self.height + y * self.width + x

    def in_bounds(self, x: int, y: int, z: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth

    def cell(self, x: int, y: int, z: int) -> int:
        return self.cells[self.index(x, y, z)]

    def set_cell(self, x: int, y: int, z: int, value: int) -> None:
        self.cells[self.index(x, y, z)] = value

    def is_free(self, x: int, y: int, z: int) -> bool:
        return self.in_bounds(x, y, z) and self.cell(x, y, z) == FREE

    def _generate(self, seed: int, obstacle_density: float) -> None:
        for z in range(self.depth):
            for y in range(self.height):
                for x in range(self.width):
                    border = (
                        x == 0
                        or y == 0
                        or z == 0
                        or x == self.width - 1
                        or y == self.height - 1
                        or z == self.depth - 1
                    )
                    if border:
                        self.set_cell(x, y, z, OBSTACLE)

        # Deterministic blocky structures give occlusion without filling the airspace.
        block_count = max(4, int(self.width * self.height * self.depth * obstacle_density / 80))
        for _ in range(block_count):
            bx = self.random.randint(3, self.width - 8)
            by = self.random.randint(3, self.height - 8)
            bz = self.random.randint(1, max(1, self.depth - 5))
            bw = self.random.randint(2, 5)
            bh = self.random.randint(2, 5)
            bd = self.random.randint(2, 5)
            for z in range(bz, min(self.depth - 1, bz + bd)):
                for y in range(by, min(self.height - 1, by + bh)):
                    for x in range(bx, min(self.width - 1, bx + bw)):
                        self.set_cell(x, y, z, OBSTACLE)

        pillar_count = max(4, int(self.width * self.height * obstacle_density / 8))
        for _ in range(pillar_count):
            px = self.random.randint(3, self.width - 4)
            py = self.random.randint(3, self.height - 4)
            radius = self.random.choice([1, 1, 2])
            top = self.random.randint(self.depth // 2, self.depth - 2)
            for z in range(1, top):
                for y in range(py - radius, py + radius + 1):
                    for x in range(px - radius, px + radius + 1):
                        if self.in_bounds(x, y, z) and (x - px) ** 2 + (y - py) ** 2 <= radius * radius:
                            self.set_cell(x, y, z, OBSTACLE)

        cx, cy, cz = self.width // 2, self.height // 2, self.depth // 2
        for z in range(cz - 2, cz + 3):
            for y in range(cy - 3, cy + 4):
                for x in range(cx - 3, cx + 4):
                    if self.in_bounds(x, y, z):
                        self.set_cell(x, y, z, FREE)


class BeliefVolume:
    def __init__(self, width: int, height: int, depth: int):
        self.width = width
        self.height = height
        self.depth = depth
        self.log_odds = [0.0] * (width * height * depth)

    def index(self, x: int, y: int, z: int) -> int:
        return z * self.width * self.height + y * self.width + x

    def in_bounds(self, x: int, y: int, z: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth

    def occupancy_probability(self, x: int, y: int, z: int) -> float:
        odds = self.log_odds[self.index(x, y, z)]
        return 1.0 / (1.0 + pow(2.718281828, -odds))

    def cell(self, x: int, y: int, z: int) -> int:
        probability = self.occupancy_probability(x, y, z)
        if probability > 0.65:
            return OBSTACLE
        if probability < 0.35:
            return FREE
        return UNKNOWN

    def is_known_free(self, x: int, y: int, z: int) -> bool:
        return self.in_bounds(x, y, z) and self.occupancy_probability(x, y, z) < 0.45

    def observe(self, x: int, y: int, z: int, occupied: bool, weight: float = 1.0) -> None:
        idx = self.index(x, y, z)
        evidence = (0.85 if occupied else -0.85) * clamp(weight, 0.0, 1.0)
        self.log_odds[idx] = clamp(self.log_odds[idx] + evidence, -5.0, 5.0)

    def uncertainty(self, x: int, y: int, z: int) -> float:
        probability = self.occupancy_probability(x, y, z)
        return 1.0 - abs(probability - 0.5) * 2.0

    def known_ratio(self) -> float:
        known = sum(1 for odds in self.log_odds if abs(odds) >= 1.2)
        return known / len(self.log_odds)

    def mean_uncertainty(self) -> float:
        return sum(
            self.uncertainty(x, y, z)
            for z in range(self.depth)
            for y in range(self.height)
            for x in range(self.width)
        ) / len(self.log_odds)


class ActiveMappingSim3D:
    def __init__(
        self,
        width: int = 32,
        height: int = 32,
        depth: int = 12,
        obstacle_density: float = 0.14,
        sensor_radius: int = 6,
        sensor_fov_degrees: float = 75.0,
        sensor_noise: float = 0.06,
        seed: int = 11,
    ):
        self.random = Random(seed + 2000)
        self.world = VoxelWorld(width, height, depth, obstacle_density, seed)
        self.belief = BeliefVolume(width, height, depth)
        self.pose = Pose3D(width // 2, height // 2, depth // 2)
        self.odometry = NoisyOdometry(Random(seed + 3000))
        self.odometry.reset(self.pose)
        self.estimated_pose = self.pose
        self.pose_graph = PoseGraph()
        self.sensor_radius = sensor_radius
        self.sensor_fov = sensor_fov_degrees * pi / 180.0
        self.sensor_noise = sensor_noise
        self.step_count = 0
        self.target: Pose3D | None = None
        self.path: list[tuple[int, int, int]] = []
        self.visible_cells: set[tuple[int, int, int]] = set()
        self.mapped_cells: set[tuple[int, int, int]] = set()
        self.last_range_frame: RangeFrame | None = None
        self.last_mean_reprojection_error = 0.0

    def step(self) -> StepResult3D:
        previous_pose = self.pose
        self.observe()

        if not self.path:
            self.target, self.path = self.choose_target_and_path()

        if self.path:
            nx, ny, nz = self.path.pop(0)
            yaw, pitch = orientation_to(self.pose.x, self.pose.y, self.pose.z, nx, ny, nz)
            self.pose = Pose3D(nx, ny, nz, 0.0, pitch, yaw)
        elif self.target:
            self.pose = Pose3D(self.pose.x, self.pose.y, self.pose.z, self.target.roll, self.target.pitch, self.target.yaw)

        self.estimated_pose = self.odometry.update(previous_pose, self.pose)
        self.step_count += 1
        stop_reason = "no_useful_reachable_viewpoint" if self.target is None else None
        return StepResult3D(
            step=self.step_count,
            pose=self.pose,
            estimated_pose=self.estimated_pose,
            target=self.target,
            path=list(self.path),
            known_ratio=self.belief.known_ratio(),
            mean_uncertainty=self.belief.mean_uncertainty(),
            mean_reprojection_error=self.last_mean_reprojection_error,
            keyframe_count=len(self.pose_graph.keyframes),
            stop_reason=stop_reason,
            done=stop_reason is not None,
        )

    def observe(self) -> None:
        frame = generate_range_frame(
            self.world,
            self.pose,
            self.estimated_pose,
            self.sensor_radius,
            self.sensor_fov,
            self.sensor_noise,
            self.random,
        )
        self.last_range_frame = frame
        self.visible_cells = frame.true_visible_cells
        self.mapped_cells = integrate_range_frame(self.belief, frame)
        self.last_mean_reprojection_error = frame.mean_reprojection_error
        self.pose_graph.maybe_add_keyframe(
            self.step_count,
            self.estimated_pose,
            self.last_mean_reprojection_error,
            len(self.mapped_cells),
        )

    def choose_target_and_path(self) -> tuple[Pose3D | None, list[tuple[int, int, int]]]:
        current = (self.pose.x, self.pose.y, self.pose.z)
        came_from, distances = reachable_voxels(current, self.belief)
        if not distances:
            return None, []

        candidates = self.rank_candidate_voxels(distances)
        current_score = self.local_uncertainty_score(*current)
        if current_score > 0.0:
            candidates.append((current_score, current))
        candidates.sort(reverse=True)
        if not candidates or candidates[0][0] <= 0.0:
            return None, []

        shortlist = []
        for _, target in candidates[:24]:
            distance = distances[target]
            roll, pitch, yaw, gain = self.best_view_orientation(*target)
            turn_cost = orientation_cost(self.pose, roll, pitch, yaw)
            score = gain / (1.0 + distance + turn_cost * 1.5)
            shortlist.append((score, Pose3D(target[0], target[1], target[2], roll, pitch, yaw), gain))

        shortlist.sort(reverse=True, key=lambda item: item[0])
        best: tuple[float, Pose3D, list[tuple[int, int, int]]] | None = None
        for _, target_pose, gain in shortlist[:5]:
            target = (target_pose.x, target_pose.y, target_pose.z)
            path = reconstruct_path3d(current, target, came_from)
            path_gain = self.path_information_gain(path, sample_every=5)
            turn_cost = orientation_cost(self.pose, target_pose.roll, target_pose.pitch, target_pose.yaw)
            total_gain = gain + path_gain * 0.40
            score = total_gain / (1.0 + len(path) + turn_cost * 1.5)
            if best is None or score > best[0]:
                best = (score, target_pose, path)

        if best is None:
            return None, []
        return best[1], best[2]

    def rank_candidate_voxels(self, distances: dict[tuple[int, int, int], int]) -> list[tuple[float, tuple[int, int, int]]]:
        candidates = []
        for voxel, distance in distances.items():
            x, y, z = voxel
            frontier = self.frontier_score(x, y, z)
            uncertainty = self.belief.uncertainty(x, y, z)
            if frontier <= 0.0 and uncertainty < 0.55:
                continue
            score = (frontier * 3.5 + uncertainty) / (1.0 + distance * 0.16)
            candidates.append((score, voxel))
        return candidates

    def frontier_score(self, x: int, y: int, z: int) -> float:
        score = 0.0
        for nx, ny, nz in neighbors26(x, y, z):
            if self.belief.in_bounds(nx, ny, nz) and self.belief.cell(nx, ny, nz) == UNKNOWN:
                score += self.belief.uncertainty(nx, ny, nz)
        return score

    def local_uncertainty_score(self, x: int, y: int, z: int, radius: int = 2) -> float:
        score = 0.0
        radius_sq = radius * radius
        for cz in range(z - radius, z + radius + 1):
            for cy in range(y - radius, y + radius + 1):
                for cx in range(x - radius, x + radius + 1):
                    if not self.belief.in_bounds(cx, cy, cz):
                        continue
                    dist_sq = (cx - x) ** 2 + (cy - y) ** 2 + (cz - z) ** 2
                    if dist_sq > radius_sq:
                        continue
                    uncertainty = self.belief.uncertainty(cx, cy, cz)
                    if uncertainty >= 0.30:
                        score += uncertainty / (1.0 + dist_sq * 0.25)
        return score

    def best_view_orientation(self, x: int, y: int, z: int) -> tuple[float, float, float, float]:
        best = (0.0, 0.0, 0.0, -1.0)
        for pitch in (-pi / 6.0, 0.0, pi / 6.0):
            for yaw_idx in range(8):
                yaw = yaw_idx * pi / 4.0
                pose = Pose3D(x, y, z, 0.0, pitch, yaw)
                gain = sum(
                    self.belief.uncertainty(cx, cy, cz)
                    for cx, cy, cz in visible_voxels_from_belief(
                        self.belief, pose, self.sensor_radius, self.sensor_fov
                    )
                )
                if gain > best[3]:
                    best = (0.0, pitch, yaw, gain)
        return best

    def path_information_gain(self, path: list[tuple[int, int, int]], sample_every: int = 1) -> float:
        gain = 0.0
        seen = set()
        previous = (self.pose.x, self.pose.y, self.pose.z)
        for index, voxel in enumerate(path):
            if index % sample_every != 0 and index != len(path) - 1:
                previous = voxel
                continue
            yaw, pitch = orientation_to(*previous, *voxel)
            pose = Pose3D(voxel[0], voxel[1], voxel[2], 0.0, pitch, yaw)
            for cell in visible_voxels_from_belief(self.belief, pose, self.sensor_radius, self.sensor_fov):
                if cell in seen:
                    continue
                seen.add(cell)
                gain += self.belief.uncertainty(*cell)
            previous = voxel
        return gain


def neighbors6(x: int, y: int, z: int):
    yield x + 1, y, z
    yield x - 1, y, z
    yield x, y + 1, z
    yield x, y - 1, z
    yield x, y, z + 1
    yield x, y, z - 1


def neighbors26(x: int, y: int, z: int):
    for nz in range(z - 1, z + 2):
        for ny in range(y - 1, y + 2):
            for nx in range(x - 1, x + 2):
                if (nx, ny, nz) != (x, y, z):
                    yield nx, ny, nz


def reachable_voxels(
    start: tuple[int, int, int], belief: BeliefVolume
) -> tuple[dict[tuple[int, int, int], tuple[int, int, int] | None], dict[tuple[int, int, int], int]]:
    queue = deque([start])
    came_from: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start: None}
    distances = {start: 0}
    while queue:
        current = queue.popleft()
        for neighbor in neighbors6(*current):
            nx, ny, nz = neighbor
            if neighbor in came_from or not belief.is_known_free(nx, ny, nz):
                continue
            came_from[neighbor] = current
            distances[neighbor] = distances[current] + 1
            queue.append(neighbor)
    return came_from, distances


def reconstruct_path3d(
    start: tuple[int, int, int],
    goal: tuple[int, int, int],
    came_from: dict[tuple[int, int, int], tuple[int, int, int] | None],
) -> list[tuple[int, int, int]]:
    if goal == start or goal not in came_from:
        return []
    path = []
    current: tuple[int, int, int] | None = goal
    while current and current != start:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def visible_voxels(world: VoxelWorld, pose: Pose3D, radius: int, fov: float) -> set[tuple[int, int, int]]:
    cells = {(pose.x, pose.y, pose.z)}
    for ray in sensor_rays3d(radius, round(fov, 6), round(pose.pitch, 6), round(pose.yaw, 6)):
        for dx, dy, dz in ray:
            x = pose.x + dx
            y = pose.y + dy
            z = pose.z + dz
            if not world.in_bounds(x, y, z):
                break
            cells.add((x, y, z))
            if world.cell(x, y, z) == OBSTACLE:
                break
    return cells


def visible_voxels_from_belief(
    belief: BeliefVolume, pose: Pose3D, radius: int, fov: float
) -> set[tuple[int, int, int]]:
    cells = {(pose.x, pose.y, pose.z)}
    for ray in sensor_rays3d(radius, round(fov, 6), round(pose.pitch, 6), round(pose.yaw, 6)):
        for dx, dy, dz in ray:
            x = pose.x + dx
            y = pose.y + dy
            z = pose.z + dz
            if not belief.in_bounds(x, y, z):
                break
            cells.add((x, y, z))
            if belief.occupancy_probability(x, y, z) > 0.65:
                break
    return cells
