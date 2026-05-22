from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from random import Random

from auto_drone.common import OBSTACLE, clamp
from auto_drone.geometry3d import Pose3D, project_local_cells, rotate_local_offset, sensor_rays3d


@dataclass(frozen=True)
class RangeMeasurement:
    ray: tuple[tuple[int, int, int], ...]
    # Local sensor-frame cells observed along this ray, truncated at the hit or world boundary.
    cells: tuple[tuple[int, int, int], ...]
    hit: bool
    hit_cell: tuple[int, int, int] | None
    range_voxels: float
    reprojection_error: float


@dataclass(frozen=True)
class RangeFrame:
    estimated_pose: Pose3D
    measurements: tuple[RangeMeasurement, ...]

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
    for ray in sensor_rays3d(radius, round(fov, 6), 0.0, 0.0):
        cells = []
        hit_cell = None
        for dx, dy, dz in ray:
            ox, oy, oz = rotate_local_offset(dx, dy, dz, true_pose.pitch, true_pose.yaw)
            x = true_pose.x + ox
            y = true_pose.y + oy
            z = true_pose.z + oz
            if not world.in_bounds(x, y, z):
                break
            cells.append((dx, dy, dz))
            if world.cell(x, y, z) == OBSTACLE:
                hit_cell = (dx, dy, dz)
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
        distance = sqrt(end[0] ** 2 + end[1] ** 2 + end[2] ** 2)
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

    return RangeFrame(estimated_pose=estimated_pose, measurements=tuple(measurements))


def observed_cells_from_pose(pose: Pose3D, frame: RangeFrame) -> set[tuple[int, int, int]]:
    cells = {(pose.x, pose.y, pose.z)}
    for measurement in frame.measurements:
        cells.update(project_local_cells(pose, measurement.cells))
    return cells


def pose_delta_magnitude(a: Pose3D, b: Pose3D) -> float:
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z) + abs(a.pitch - b.pitch) + abs(a.yaw - b.yaw)
