from __future__ import annotations

from auto_drone.common import clamp
from auto_drone.sensing3d import RangeFrame


def integrate_range_frame(belief, frame: RangeFrame) -> set[tuple[int, int, int]]:
    mapped_cells = {(frame.estimated_pose.x, frame.estimated_pose.y, frame.estimated_pose.z)}
    for measurement in frame.measurements:
        confidence = confidence_from_reprojection_error(measurement.reprojection_error)
        estimated_cells = cells_from_estimated_pose(frame.estimated_pose, measurement.cells, frame.true_pose)
        if not estimated_cells:
            continue

        stop_index = len(estimated_cells)
        if measurement.hit and measurement.hit_cell is not None:
            stop_index = max(0, len(estimated_cells) - 1)

        for cell in estimated_cells[:stop_index]:
            if belief.in_bounds(*cell):
                belief.observe(*cell, occupied=False, weight=confidence)
                mapped_cells.add(cell)

        if measurement.hit and estimated_cells:
            hit_cell = estimated_cells[-1]
            if belief.in_bounds(*hit_cell):
                belief.observe(*hit_cell, occupied=True, weight=confidence)
                mapped_cells.add(hit_cell)

    return mapped_cells


def confidence_from_reprojection_error(error: float) -> float:
    return clamp(1.0 / (1.0 + error * 2.5), 0.20, 1.0)


def cells_from_estimated_pose(
    estimated_pose, true_cells: tuple[tuple[int, int, int], ...], true_pose
) -> list[tuple[int, int, int]]:
    dx = estimated_pose.x - true_pose.x
    dy = estimated_pose.y - true_pose.y
    dz = estimated_pose.z - true_pose.z
    return [(x + dx, y + dy, z + dz) for x, y, z in true_cells]
