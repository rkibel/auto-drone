from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, sin

import numpy as np

from auto_drone.common import clamp
from auto_drone.interfaces import CameraIntrinsics, MetricPose, VoxelGridSpec, VoxelPose

OCCUPIED_EVIDENCE = 0.85
FREE_EVIDENCE = -0.85
OCCUPIED_PROBABILITY_THRESHOLD = 0.9
MAX_COLOR_SAMPLES = 64
RAY_EVIDENCE_WEIGHT = 8.0 / 9.0


@dataclass(frozen=True)
class RangeMeasurement:
    cells: tuple[tuple[int, int, int], ...]
    hit: bool
    hit_color: tuple[int, int, int] | None = None


@dataclass(frozen=True)
class RangeFrame:
    estimated_pose: VoxelPose
    measurements: tuple[RangeMeasurement, ...]


class BeliefVolume:
    def __init__(self, width: int, height: int, depth: int):
        self.width = width
        self.height = height
        self.depth = depth
        self.log_odds = [0.0] * (width * height * depth)
        self.colors: dict[int, tuple[float, float, float, int]] = {}

    def index(self, x: int, y: int, z: int) -> int:
        return z * self.width * self.height + y * self.width + x

    def in_bounds(self, x: int, y: int, z: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth

    def occupancy_probability(self, x: int, y: int, z: int) -> float:
        odds = self.log_odds[self.index(x, y, z)]
        return 1.0 / (1.0 + pow(2.718281828, -odds))

    def observe(self, x: int, y: int, z: int, occupied: bool, weight: float = 1.0) -> None:
        idx = self.index(x, y, z)
        current = self.log_odds[idx]
        evidence_weight = clamp(weight, 0.0, 1.0)
        evidence = (OCCUPIED_EVIDENCE if occupied else FREE_EVIDENCE) * evidence_weight
        self.log_odds[idx] = clamp(current + evidence, -5.0, 5.0)

    def observe_color(self, x: int, y: int, z: int, color: tuple[int, int, int]) -> None:
        """Update a bounded running mean for an occupied voxel's RGB color."""
        idx = self.index(x, y, z)
        previous = self.colors.get(idx)
        if previous is None:
            self.colors[idx] = (*color, 1)
        else:
            sample_count = min(previous[3] + 1, MAX_COLOR_SAMPLES)
            weight = 1.0 / sample_count
            self.colors[idx] = (
                previous[0] + (color[0] - previous[0]) * weight,
                previous[1] + (color[1] - previous[1]) * weight,
                previous[2] + (color[2] - previous[2]) * weight,
                sample_count,
            )

    def known_ratio(self) -> float:
        known = sum(1 for odds in self.log_odds if abs(odds) >= 1.2)
        return known / len(self.log_odds)


def stride_sample_offset(frame_index: int, stride: int) -> tuple[int, int]:
    if frame_index < 0:
        raise ValueError("frame index must be non-negative")
    if stride <= 0:
        raise ValueError("stride must be positive")
    phase = frame_index % (stride * stride)
    return phase % stride, phase // stride


def integrate_range_frame(
    belief: BeliefVolume,
    frame: RangeFrame,
) -> set[tuple[int, int, int]]:
    mapped_cells = {(frame.estimated_pose.x, frame.estimated_pose.y, frame.estimated_pose.z)}
    for measurement in frame.measurements:
        if not measurement.cells:
            continue
        free_cells = measurement.cells[:-1] if measurement.hit else measurement.cells
        for cell in free_cells:
            if belief.in_bounds(*cell):
                belief.observe(*cell, occupied=False, weight=RAY_EVIDENCE_WEIGHT)
                mapped_cells.add(cell)

        if measurement.hit:
            hit_cell = measurement.cells[-1]
            if belief.in_bounds(*hit_cell):
                belief.observe(*hit_cell, occupied=True, weight=RAY_EVIDENCE_WEIGHT)
                if measurement.hit_color is not None:
                    belief.observe_color(*hit_cell, measurement.hit_color)
                mapped_cells.add(hit_cell)
    return mapped_cells


def depth_image_to_range_frame(
    depth_values: list[float] | tuple[float, ...],
    intrinsics: CameraIntrinsics,
    estimated_sensor_pose: MetricPose,
    grid: VoxelGridSpec,
    min_depth_m: float,
    max_depth_m: float,
    stride: int = 8,
    colors: list[tuple[int, int, int]] | tuple[tuple[int, int, int], ...] | None = None,
    sample_offset: tuple[int, int] | None = None,
) -> RangeFrame:
    if len(depth_values) != intrinsics.width * intrinsics.height:
        raise ValueError("depth value count does not match camera dimensions")
    if stride <= 0:
        raise ValueError("stride must be positive")
    if colors is not None and len(colors) != len(depth_values):
        raise ValueError("color value count does not match depth dimensions")
    u_offset, v_offset = sample_offset if sample_offset is not None else (stride // 2, stride // 2)
    if not 0 <= u_offset < stride or not 0 <= v_offset < stride:
        raise ValueError("sample offset must be inside one stride period")

    pose = grid.metric_to_voxel(estimated_sensor_pose)
    measurements = []
    for v in range(v_offset, intrinsics.height, stride):
        row = v * intrinsics.width
        for u in range(u_offset, intrinsics.width, stride):
            depth_m = depth_values[row + u]
            if not isfinite(depth_m) or depth_m < min_depth_m:
                continue
            hit = depth_m < max_depth_m
            ray_depth_m = clamp(depth_m, min_depth_m, max_depth_m)
            right_m = (u - intrinsics.cx) * ray_depth_m / intrinsics.fx
            down_m = (v - intrinsics.cy) * ray_depth_m / intrinsics.fy
            offset = rotate_local_offset_metric(
                ray_depth_m,
                -right_m,
                -down_m,
                estimated_sensor_pose.roll,
                estimated_sensor_pose.pitch,
                estimated_sensor_pose.yaw,
            )
            world_end = grid.metric_to_voxel(
                MetricPose(
                    estimated_sensor_pose.x + offset[0],
                    estimated_sensor_pose.y + offset[1],
                    estimated_sensor_pose.z + offset[2],
                )
            )
            cells = tuple(line_voxels(pose.x, pose.y, pose.z, world_end.x, world_end.y, world_end.z)[1:])
            if cells:
                measurements.append(
                    RangeMeasurement(
                        cells,
                        hit,
                        tuple(int(channel) for channel in colors[row + u]) if hit and colors is not None else None,
                    )
                )
    return RangeFrame(pose, tuple(measurements))


def view_depth_image(data: bytes, width: int, height: int, encoding: str, step: int) -> np.ndarray:
    encoding = encoding.upper()
    formats = {"32FC1": ("<f4", 4), "16UC1": ("<u2", 2)}
    if encoding not in formats:
        raise ValueError(f"unsupported depth image encoding: {encoding}")
    dtype, bytes_per_pixel = formats[encoding]
    if step < width * bytes_per_pixel or len(data) < step * height:
        raise ValueError("depth image data is smaller than its declared dimensions")
    values = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=data,
        strides=(step, bytes_per_pixel),
    )
    if encoding == "16UC1":
        values = values.astype(np.float32) / 1000.0
    return values.reshape(-1)


def view_color_image(data: bytes, width: int, height: int, encoding: str, step: int) -> np.ndarray:
    encoding = encoding.lower()
    channel_orders = {
        "rgb8": (0, 1, 2, 3),
        "bgr8": (2, 1, 0, 3),
        "rgba8": (0, 1, 2, 4),
        "bgra8": (2, 1, 0, 4),
    }
    if encoding not in channel_orders:
        raise ValueError(f"unsupported color image encoding: {encoding}")
    red, green, blue, channels = channel_orders[encoding]
    if step < width * channels or len(data) < step * height:
        raise ValueError("color image data is smaller than its declared dimensions")
    pixels = np.ndarray(
        shape=(height, width, channels),
        dtype=np.uint8,
        buffer=data,
        strides=(step, channels, 1),
    )
    return pixels[:, :, (red, green, blue)].reshape(-1, 3)


def rotate_local_offset_metric(
    dx: float,
    dy: float,
    dz: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    cr = cos(roll)
    sr = sin(roll)
    cp = cos(pitch)
    sp = sin(pitch)
    cy = cos(yaw)
    sy = sin(yaw)
    rolled_y = cr * dy - sr * dz
    rolled_z = sr * dy + cr * dz
    pitched_x = cp * dx - sp * rolled_z
    pitched_z = sp * dx + cp * rolled_z
    return cy * pitched_x - sy * rolled_y, sy * pitched_x + cy * rolled_y, pitched_z


def line_voxels(x1: int, y1: int, z1: int, x2: int, y2: int, z2: int) -> list[tuple[int, int, int]]:
    steps = max(abs(x2 - x1), abs(y2 - y1), abs(z2 - z1))
    if steps == 0:
        return [(x1, y1, z1)]
    cells = []
    for step in range(steps + 1):
        t = step / steps
        cell = (round(x1 + (x2 - x1) * t), round(y1 + (y2 - y1) * t), round(z1 + (z2 - z1) * t))
        if not cells or cells[-1] != cell:
            cells.append(cell)
    return cells
