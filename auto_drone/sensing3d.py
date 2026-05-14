from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from random import Random

from auto_drone.common import OBSTACLE, clamp
from auto_drone.geometry3d import Pose3D, sensor_rays3d


@dataclass(frozen=True)
class RangeMeasurement:
    ray: tuple[tuple[int, int, int], ...]
    cells: tuple[tuple[int, int, int], ...]
    hit: bool
    hit_cell: tuple[int, int, int] | None
    range_voxels: float
    reprojection_error: float


@dataclass(frozen=True)
class RangeFrame:
    true_pose: Pose3D
    estimated_pose: Pose3D
    measurements: tuple[RangeMeasurement, ...]

    @property
    def true_visible_cells(self) -> set[tuple[int, int, int]]:
        cells = {(self.true_pose.x, self.true_pose.y, self.true_pose.z)}
        for measurement in self.measurements:
            cells.update(measurement.cells)
        return cells

    @property
    def mean_reprojection_error(self) -> float:
        if not self.measurements:
            return 0.0
        return sum(measurement.reprojection_error for measurement in self.measurements) / len(self.measurements)


def generate_range_frame(
    world,
    true_pose: Pose3D,
    estimated_pose: Pose3D,
    radius: int,
    fov: float,
    noise: float,
    random: Random,
) -> RangeFrame:
    measurements = []
    for ray in sensor_rays3d(radius, round(fov, 6), round(true_pose.pitch, 6), round(true_pose.yaw, 6)):
        cells = []
        hit_cell = None
        for dx, dy, dz in ray:
            x = true_pose.x + dx
            y = true_pose.y + dy
            z = true_pose.z + dz
            if not world.in_bounds(x, y, z):
                break
            cells.append((x, y, z))
            if world.cell(x, y, z) == OBSTACLE:
                hit_cell = (x, y, z)
                break

        if not cells:
            continue

        hit = hit_cell is not None
        if random.random() < noise:
            hit = not hit
            if not hit:
                hit_cell = None
            elif hit_cell is None:
                hit_cell = cells[-1]

        end = hit_cell if hit_cell else cells[-1]
        distance = sqrt((end[0] - true_pose.x) ** 2 + (end[1] - true_pose.y) ** 2 + (end[2] - true_pose.z) ** 2)
        distance_error = random.uniform(0.0, noise * 2.0)
        pose_error = pose_delta_magnitude(true_pose, estimated_pose) * 0.08
        grazing_error = 0.10 if hit and distance > radius * 0.75 else 0.0
        reprojection_error = clamp(distance_error + pose_error + grazing_error, 0.0, 1.5)
        measurements.append(
            RangeMeasurement(
                ray=ray,
                cells=tuple(cells),
                hit=hit,
                hit_cell=hit_cell,
                range_voxels=distance,
                reprojection_error=reprojection_error,
            )
        )

    return RangeFrame(true_pose=true_pose, estimated_pose=estimated_pose, measurements=tuple(measurements))


def pose_delta_magnitude(a: Pose3D, b: Pose3D) -> float:
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z) + abs(a.pitch - b.pitch) + abs(a.yaw - b.yaw)
