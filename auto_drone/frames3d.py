from __future__ import annotations

from math import pi

from auto_drone.common import angle_delta
from auto_drone.interfaces3d import MetricPose


def ned_position_to_enu(x_north: float, y_east: float, z_down: float) -> tuple[float, float, float]:
    return y_east, x_north, -z_down


def enu_velocity_to_ned(vx_east: float, vy_north: float, vz_up: float) -> tuple[float, float, float]:
    return vy_north, vx_east, -vz_up


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return angle_delta(pi / 2.0, yaw_ned)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    return angle_delta(pi / 2.0, yaw_enu)


def yaw_rate_enu_to_ned(yaw_rate_enu: float) -> float:
    return -yaw_rate_enu


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
