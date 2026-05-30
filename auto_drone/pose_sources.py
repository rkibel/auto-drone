from __future__ import annotations

from dataclasses import dataclass

from auto_drone.interfaces3d import MetricPose, PoseSample


class PoseSource:
    def latest(self) -> PoseSample | None:
        raise NotImplementedError


@dataclass
class Px4OdomPoseSource(PoseSource):
    sample: PoseSample | None = None

    def update(self, pose: MetricPose, stamp_sec: float, confidence: float = 1.0) -> PoseSample:
        self.sample = PoseSample(pose, stamp_sec, confidence, "px4_odom")
        return self.sample

    def latest(self) -> PoseSample | None:
        return self.sample


@dataclass
class RosSlamPoseSource(PoseSource):
    sample: PoseSample | None = None

    def update(self, pose: MetricPose, stamp_sec: float, confidence: float = 1.0) -> PoseSample:
        self.sample = PoseSample(pose, stamp_sec, confidence, "slam")
        return self.sample

    def latest(self) -> PoseSample | None:
        return self.sample
