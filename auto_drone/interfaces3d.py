from __future__ import annotations

from dataclasses import dataclass
from math import pi, sqrt

from auto_drone.common import clamp
from auto_drone.geometry3d import Pose3D


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

    def metric_to_voxel(self, pose: MetricPose) -> Pose3D:
        return Pose3D(
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
class SensorFrame3D:
    estimated_sensor_pose: MetricPose
    stamp_sec: float
    source: str


@dataclass(frozen=True)
class PoseSample:
    pose: MetricPose
    stamp_sec: float
    confidence: float = 1.0
    source: str = "odom"

    def is_fresh(self, now_sec: float, max_age_sec: float) -> bool:
        return now_sec - self.stamp_sec <= max_age_sec


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class CommandTarget:
    pose: MetricPose
    velocity: tuple[float, float, float]
    yaw_rate: float
    reached: bool


def command_toward_pose(
    current: MetricPose,
    target: MetricPose,
    max_speed_mps: float,
    max_yaw_rate_rps: float,
    position_tolerance_m: float,
) -> CommandTarget:
    dx = target.x - current.x
    dy = target.y - current.y
    dz = target.z - current.z
    distance = sqrt(dx * dx + dy * dy + dz * dz)
    reached = distance <= position_tolerance_m
    speed = 0.0 if reached else min(max_speed_mps, distance)
    if distance > 1e-6:
        velocity = (dx / distance * speed, dy / distance * speed, dz / distance * speed)
    else:
        velocity = (0.0, 0.0, 0.0)

    yaw_error = (target.yaw - current.yaw + pi) % (2.0 * pi) - pi
    yaw_rate = clamp(yaw_error, -max_yaw_rate_rps, max_yaw_rate_rps)
    return CommandTarget(target, velocity, yaw_rate, reached)
