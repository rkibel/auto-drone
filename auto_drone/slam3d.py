from __future__ import annotations

from dataclasses import dataclass, field

from auto_drone.geometry3d import Pose3D


@dataclass(frozen=True)
class Keyframe:
    id: int
    step: int
    estimated_pose: Pose3D
    mean_reprojection_error: float
    observed_count: int


@dataclass(frozen=True)
class PoseConstraint:
    source_id: int
    target_id: int
    relative_translation: tuple[int, int, int]
    relative_rotation: tuple[float, float, float]
    residual: float


@dataclass
class PoseGraph:
    keyframes: list[Keyframe] = field(default_factory=list)
    constraints: list[PoseConstraint] = field(default_factory=list)

    def maybe_add_keyframe(
        self,
        step: int,
        estimated_pose: Pose3D,
        mean_reprojection_error: float,
        observed_count: int,
        min_step_gap: int = 6,
    ) -> Keyframe | None:
        if self.keyframes and step - self.keyframes[-1].step < min_step_gap:
            return None

        keyframe = Keyframe(
            id=len(self.keyframes),
            step=step,
            estimated_pose=estimated_pose,
            mean_reprojection_error=mean_reprojection_error,
            observed_count=observed_count,
        )
        if self.keyframes:
            previous = self.keyframes[-1]
            self.constraints.append(
                PoseConstraint(
                    source_id=previous.id,
                    target_id=keyframe.id,
                    relative_translation=(
                        keyframe.estimated_pose.x - previous.estimated_pose.x,
                        keyframe.estimated_pose.y - previous.estimated_pose.y,
                        keyframe.estimated_pose.z - previous.estimated_pose.z,
                    ),
                    relative_rotation=(
                        keyframe.estimated_pose.roll - previous.estimated_pose.roll,
                        keyframe.estimated_pose.pitch - previous.estimated_pose.pitch,
                        keyframe.estimated_pose.yaw - previous.estimated_pose.yaw,
                    ),
                    residual=(previous.mean_reprojection_error + keyframe.mean_reprojection_error) / 2.0,
                )
            )
        self.keyframes.append(keyframe)
        return keyframe

    def latest_residual(self) -> float:
        if not self.constraints:
            return 0.0
        return self.constraints[-1].residual
