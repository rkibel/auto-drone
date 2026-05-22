from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from math import atan2, cos, pi, sin, sqrt

from auto_drone.common import angle_delta


@dataclass(frozen=True)
class Pose3D:
    x: int
    y: int
    z: int
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


def orientation_to(x1: int, y1: int, z1: int, x2: int, y2: int, z2: int) -> tuple[float, float]:
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    yaw = atan2(dy, dx) if dx or dy else 0.0
    pitch = atan2(dz, max(1e-6, sqrt(dx * dx + dy * dy)))
    return yaw, pitch


def orientation_cost(pose: Pose3D, roll: float, pitch: float, yaw: float) -> float:
    return (
        abs(angle_delta(pose.yaw, yaw)) / (pi / 4.0)
        + abs(pose.pitch - pitch) / (pi / 6.0)
        + abs(pose.roll - roll) / (pi / 6.0)
    )


def forward_vector(yaw: float, pitch: float) -> tuple[float, float, float]:
    cp = cos(pitch)
    return cp * cos(yaw), cp * sin(yaw), sin(pitch)


def rotate_local_offset(dx: int, dy: int, dz: int, pitch: float, yaw: float) -> tuple[int, int, int]:
    cp = cos(pitch)
    sp = sin(pitch)
    cy = cos(yaw)
    sy = sin(yaw)

    pitched_x = cp * dx - sp * dz
    pitched_z = sp * dx + cp * dz
    world_x = cy * pitched_x - sy * dy
    world_y = sy * pitched_x + cy * dy
    return round(world_x), round(world_y), round(pitched_z)


def project_local_cells(pose: Pose3D, local_cells: tuple[tuple[int, int, int], ...]) -> list[tuple[int, int, int]]:
    cells = []
    for dx, dy, dz in local_cells:
        ox, oy, oz = rotate_local_offset(dx, dy, dz, pose.pitch, pose.yaw)
        cell = (pose.x + ox, pose.y + oy, pose.z + oz)
        if not cells or cells[-1] != cell:
            cells.append(cell)
    return cells


@lru_cache(maxsize=512)
def sensor_rays3d(
    radius: int, fov: float, pitch: float, yaw: float
) -> tuple[tuple[tuple[int, int, int], ...], ...]:
    forward = forward_vector(yaw, pitch)
    cos_limit = cos(fov / 2.0)
    endpoints = []
    inner_sq = max(1, (radius - 1) * (radius - 1))
    for dz in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                dist_sq = dx * dx + dy * dy + dz * dz
                if dist_sq > radius * radius:
                    continue
                boundary = abs(dx) == radius or abs(dy) == radius or abs(dz) == radius or dist_sq >= inner_sq
                if not boundary:
                    continue
                dist = sqrt(dist_sq)
                dot = (dx * forward[0] + dy * forward[1] + dz * forward[2]) / dist
                if dot >= cos_limit:
                    endpoints.append((atan2(dy, dx), atan2(dz, sqrt(dx * dx + dy * dy)), dx, dy, dz))

    rays = []
    seen = set()
    for _, _, dx, dy, dz in sorted(endpoints):
        ray = tuple(line_voxels(0, 0, 0, dx, dy, dz)[1:])
        if ray and ray not in seen:
            seen.add(ray)
            rays.append(ray)
    return tuple(rays)


def line_voxels(x1: int, y1: int, z1: int, x2: int, y2: int, z2: int) -> list[tuple[int, int, int]]:
    steps = max(abs(x2 - x1), abs(y2 - y1), abs(z2 - z1))
    if steps == 0:
        return [(x1, y1, z1)]
    cells = []
    for step in range(steps + 1):
        t = step / steps
        x = round(x1 + (x2 - x1) * t)
        y = round(y1 + (y2 - y1) * t)
        z = round(z1 + (z2 - z1) * t)
        cell = (x, y, z)
        if not cells or cells[-1] != cell:
            cells.append(cell)
    return cells
