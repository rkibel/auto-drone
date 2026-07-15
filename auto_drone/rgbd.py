from __future__ import annotations

from collections import deque
from math import asin, atan2, isfinite

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from auto_drone.interfaces import CameraIntrinsics, MetricPose
from auto_drone.mapping import rotate_local_offset_metric

DEPTH_STRIDE = 8
MAX_MAPPING_DEPTH_M = 19.0


def camera_pose_from_ground_truth_odometry(message: Odometry) -> MetricPose:
    return camera_pose_from_body_pose(ground_truth_pose_from_odometry(message))


def camera_pose_from_body_pose(body_pose: MetricPose) -> MetricPose:
    offset_x, offset_y, offset_z = rotate_local_offset_metric(
        0.13233,
        0.0,
        0.26078,
        body_pose.roll,
        body_pose.pitch,
        body_pose.yaw,
    )
    return MetricPose(
        body_pose.x + offset_x,
        body_pose.y + offset_y,
        body_pose.z + offset_z,
        roll=body_pose.roll,
        pitch=body_pose.pitch,
        yaw=body_pose.yaw,
    )


def image_timestamp_ns(message: Image | Odometry) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def exact_image_pairs(
    depth_images: deque[Image],
    color_images: deque[Image],
) -> list[tuple[Image, Image]]:
    colors_by_stamp = {image_timestamp_ns(image): image for image in color_images}
    pairs = []
    for depth_image in reversed(depth_images):
        color_image = colors_by_stamp.get(image_timestamp_ns(depth_image))
        if color_image is not None and (color_image.width, color_image.height) == (
            depth_image.width,
            depth_image.height,
        ):
            pairs.append((depth_image, color_image))
    return pairs


def ground_truth_pose_from_odometry(message: Odometry) -> MetricPose:
    position = message.pose.pose.position
    q = message.pose.pose.orientation
    roll = atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return MetricPose(float(position.x), float(position.y), float(position.z), roll, pitch, yaw)


def refine_camera_pose_from_ground(
    depth_values: list[float],
    colors: list[tuple[int, int, int]],
    intrinsics: CameraIntrinsics,
    pose: MetricPose,
    stride: int = DEPTH_STRIDE,
) -> MetricPose | None:
    points = []
    for v in range(stride // 2, intrinsics.height, stride):
        if v < intrinsics.cy:
            continue
        row = v * intrinsics.width
        for u in range(stride // 2, intrinsics.width, stride):
            depth_m = depth_values[row + u]
            if not isfinite(depth_m) or not 0.25 <= depth_m < MAX_MAPPING_DEPTH_M:
                continue
            if not is_ground_color(colors[row + u]):
                continue
            right_m = (u - intrinsics.cx) * depth_m / intrinsics.fx
            down_m = (v - intrinsics.cy) * depth_m / intrinsics.fy
            points.append((depth_m, -right_m, -down_m))
    if len(points) < 20:
        return None

    samples = np.asarray(points, dtype=float)
    normal = fitted_plane_normal(samples)
    if normal[2] < 0.0:
        normal = -normal
    height_m = -float(np.median(samples @ normal))
    residuals = np.abs(samples @ normal + height_m)
    inliers = samples[residuals <= max(0.08, float(np.median(residuals)) * 3.0)]
    if len(inliers) < 20:
        return None
    normal = fitted_plane_normal(inliers)
    if normal[2] < 0.0:
        normal = -normal
    height_m = -float(np.median(inliers @ normal))
    roll = atan2(float(normal[1]), float(normal[2]))
    pitch = asin(max(-1.0, min(1.0, -float(normal[0]))))
    if normal[2] < 0.8 or not 0.0 < height_m < 12.0:
        return None
    if abs(height_m - pose.z) > 2.0 or abs(roll - pose.roll) > 0.25 or abs(pitch - pose.pitch) > 0.25:
        return None
    return MetricPose(pose.x, pose.y, height_m, roll, pitch, pose.yaw)


def fitted_plane_normal(samples: np.ndarray) -> np.ndarray:
    center = samples.mean(axis=0)
    _, _, axes = np.linalg.svd(samples - center, full_matrices=False)
    return axes[-1]


def is_ground_color(color: tuple[int, int, int]) -> bool:
    red, green, blue = color
    return 145 <= red <= 215 and 155 <= green <= 225 and 165 <= blue <= 230 and blue > red and green >= red


def correct_ground_depths(
    depth_values: list[float],
    colors: list[tuple[int, int, int]],
    intrinsics: CameraIntrinsics,
    pose: MetricPose,
    stride: int = DEPTH_STRIDE,
    sample_offset: tuple[int, int] | None = None,
) -> list[float]:
    corrected = depth_values.copy()
    u_offset, v_offset = sample_offset if sample_offset is not None else (stride // 2, stride // 2)
    if not 0 <= u_offset < stride or not 0 <= v_offset < stride:
        raise ValueError("sample offset must be inside one stride period")
    for v in range(v_offset, intrinsics.height, stride):
        row = v * intrinsics.width
        for u in range(u_offset, intrinsics.width, stride):
            index = row + u
            measured_depth_m = depth_values[index]
            if not isfinite(measured_depth_m) or not is_ground_color(colors[index]):
                continue
            corrected[index] = float("nan")
            local_y = -(u - intrinsics.cx) / intrinsics.fx
            local_z = -(v - intrinsics.cy) / intrinsics.fy
            _, _, world_direction_z = rotate_local_offset_metric(
                1.0,
                local_y,
                local_z,
                pose.roll,
                pose.pitch,
                pose.yaw,
            )
            if world_direction_z >= -1e-6:
                continue
            plane_depth_m = -pose.z / world_direction_z
            if 0.25 <= plane_depth_m < MAX_MAPPING_DEPTH_M:
                corrected[index] = plane_depth_m
    return corrected
