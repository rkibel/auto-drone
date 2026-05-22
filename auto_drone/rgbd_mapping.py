from __future__ import annotations

from math import isfinite
from struct import unpack_from

from auto_drone.common import clamp
from auto_drone.geometry3d import line_voxels
from auto_drone.interfaces3d import CameraIntrinsics, MetricPose, VoxelGridSpec
from auto_drone.sensing3d import RangeFrame, RangeMeasurement


def depth_image_to_range_frame(
    depth_values: list[float] | tuple[float, ...],
    intrinsics: CameraIntrinsics,
    estimated_sensor_pose: MetricPose,
    grid: VoxelGridSpec,
    min_depth_m: float,
    max_depth_m: float,
    stride: int = 8,
    residual: float = 0.05,
) -> RangeFrame:
    if len(depth_values) != intrinsics.width * intrinsics.height:
        raise ValueError("depth value count does not match camera dimensions")
    if stride <= 0:
        raise ValueError("stride must be positive")

    pose = grid.metric_to_voxel(estimated_sensor_pose)
    max_range_voxels = max(1, round(max_depth_m / grid.resolution))
    measurements = []
    for v in range(stride // 2, intrinsics.height, stride):
        row = v * intrinsics.width
        for u in range(stride // 2, intrinsics.width, stride):
            depth_m = depth_values[row + u]
            if not isfinite(depth_m) or depth_m < min_depth_m:
                continue

            hit = depth_m < max_depth_m
            ray_depth_m = clamp(depth_m, min_depth_m, max_depth_m)
            local_end = depth_pixel_to_local_voxel(u, v, ray_depth_m, intrinsics, grid.resolution)
            if local_end[0] <= 0:
                continue

            ray = tuple(line_voxels(0, 0, 0, *local_end)[1 : max_range_voxels + 1])
            if not ray:
                continue
            cells = ray if hit else ray[:max_range_voxels]
            if not cells:
                continue

            measurements.append(
                RangeMeasurement(
                    ray=ray,
                    cells=cells,
                    hit=hit,
                    hit_cell=cells[-1] if hit else None,
                    range_voxels=min(len(cells), ray_depth_m / grid.resolution),
                    reprojection_error=residual,
                )
            )

    return RangeFrame(estimated_pose=pose, measurements=tuple(measurements))


def depth_pixel_to_local_voxel(
    u: int,
    v: int,
    depth_m: float,
    intrinsics: CameraIntrinsics,
    resolution: float,
) -> tuple[int, int, int]:
    right_m = (u - intrinsics.cx) * depth_m / intrinsics.fx
    down_m = (v - intrinsics.cy) * depth_m / intrinsics.fy
    return (
        round(depth_m / resolution),
        round(-right_m / resolution),
        round(-down_m / resolution),
    )


def decode_depth_image(data: bytes, width: int, height: int, encoding: str, step: int) -> list[float]:
    values = []
    encoding = encoding.upper()
    if encoding == "32FC1":
        item_size = 4
        for v in range(height):
            row_offset = v * step
            for u in range(width):
                values.append(unpack_from("<f", data, row_offset + u * item_size)[0])
        return values
    if encoding == "16UC1":
        item_size = 2
        for v in range(height):
            row_offset = v * step
            for u in range(width):
                values.append(unpack_from("<H", data, row_offset + u * item_size)[0] / 1000.0)
        return values
    raise ValueError(f"unsupported depth image encoding: {encoding}")
