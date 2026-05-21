from __future__ import annotations

from dataclasses import dataclass
from random import Random

from auto_drone.geometry3d import Pose3D


@dataclass
class NoisyOdometry:
    random: Random
    translational_noise: float = 0.04
    rotational_noise: float = 0.015
    drift_per_step: float = 0.015
    estimated_pose: Pose3D | None = None
    drift_x: float = 0.0
    drift_y: float = 0.0
    drift_z: float = 0.0

    def reset(self, pose: Pose3D) -> None:
        self.estimated_pose = pose
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_z = 0.0

    def update(self, previous_true_pose: Pose3D, true_pose: Pose3D) -> Pose3D:
        if self.estimated_pose is None:
            self.reset(previous_true_pose)

        self.drift_x += self.random.uniform(-self.drift_per_step, self.drift_per_step)
        self.drift_y += self.random.uniform(-self.drift_per_step, self.drift_per_step)
        self.drift_z += self.random.uniform(-self.drift_per_step, self.drift_per_step)

        dx = true_pose.x - previous_true_pose.x
        dy = true_pose.y - previous_true_pose.y
        dz = true_pose.z - previous_true_pose.z
        estimated = self.estimated_pose
        self.estimated_pose = Pose3D(
            round(estimated.x + dx + self.drift_x + self.random.uniform(-self.translational_noise, self.translational_noise)),
            round(estimated.y + dy + self.drift_y + self.random.uniform(-self.translational_noise, self.translational_noise)),
            round(estimated.z + dz + self.drift_z + self.random.uniform(-self.translational_noise, self.translational_noise)),
            true_pose.roll,
            true_pose.pitch + self.random.uniform(-self.rotational_noise, self.rotational_noise),
            true_pose.yaw + self.random.uniform(-self.rotational_noise, self.rotational_noise),
        )
        return self.estimated_pose
