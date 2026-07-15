from __future__ import annotations

from dataclasses import dataclass
from math import pi

from auto_drone.common import angle_delta


@dataclass(frozen=True)
class VoxelPose:
    x: int
    y: int
    z: int
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class MetricPose:
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class VoxelGridSpec:
    width: int
    height: int
    depth: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_z: float

    def metric_to_voxel(self, pose: MetricPose) -> VoxelPose:
        return VoxelPose(
            round((pose.x - self.origin_x) / self.resolution),
            round((pose.y - self.origin_y) / self.resolution),
            round((pose.z - self.origin_z) / self.resolution),
            pose.roll,
            pose.pitch,
            pose.yaw,
        )

    def voxel_to_metric(
        self,
        x: int,
        y: int,
        z: int,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> MetricPose:
        return MetricPose(
            self.origin_x + x * self.resolution,
            self.origin_y + y * self.resolution,
            self.origin_z + z * self.resolution,
            roll,
            pitch,
            yaw,
        )

    def in_bounds(self, x: int, y: int, z: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


def ned_position_to_enu(x_north: float, y_east: float, z_down: float) -> tuple[float, float, float]:
    return y_east, x_north, -z_down


def enu_velocity_to_ned(vx_east: float, vy_north: float, vz_up: float) -> tuple[float, float, float]:
    return vy_north, vx_east, -vz_up


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return angle_delta(pi / 2.0, yaw_ned)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    return angle_delta(pi / 2.0, yaw_enu)


def px4_ned_pose_to_metric(
    x_north: float,
    y_east: float,
    z_down: float,
    yaw_ned: float,
    roll: float = 0.0,
    pitch: float = 0.0,
) -> MetricPose:
    x_enu, y_enu, z_enu = ned_position_to_enu(x_north, y_east, z_down)
    return MetricPose(x_enu, y_enu, z_enu, roll, pitch, yaw_ned_to_enu(yaw_ned))
