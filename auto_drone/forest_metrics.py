from __future__ import annotations

from dataclasses import dataclass
from math import hypot

from auto_drone.interfaces import VoxelGridSpec
from auto_drone.mapping import BeliefVolume, OCCUPIED_PROBABILITY_THRESHOLD


TREE_REFERENCES = (
    (-20.0, -11.0), (-12.0, -10.0), (19.0, -11.0),
    (20.0, -3.0), (-20.0, -1.0), (19.0, 6.0),
    (-20.0, 13.0), (20.0, 17.0), (2.0, 19.0),
    (-10.0, 36.0), (-20.0, 36.0), (3.0, 28.0),
)
TREE_RADIUS_M = 3.0
HEIGHT_BANDS = 8
MIN_TREE_HEIGHT_M = 1.0


@dataclass(frozen=True)
class TreeCoverage:
    observed_trees: int
    tree_count: int
    observed_percentage: float
    reconstruction_percentage: float
    mean_observed_tree_quality: float
    tree_band_counts: tuple[int, ...]


def tree_coverage(belief: BeliefVolume, grid: VoxelGridSpec) -> TreeCoverage:
    bands = tree_height_bands(belief, grid)
    observed = [tree_bands for tree_bands in bands if tree_bands]
    observed_trees = len(observed)
    total_bands = sum(len(tree_bands) for tree_bands in bands)
    return TreeCoverage(
        observed_trees,
        len(TREE_REFERENCES),
        observed_trees / len(TREE_REFERENCES) * 100.0,
        total_bands / (len(TREE_REFERENCES) * HEIGHT_BANDS) * 100.0,
        (sum(len(tree_bands) for tree_bands in observed) / (observed_trees * HEIGHT_BANDS) * 100.0)
        if observed_trees
        else 0.0,
        tuple(len(tree_bands) for tree_bands in bands),
    )


def tree_height_bands(belief: BeliefVolume, grid: VoxelGridSpec) -> list[set[int]]:
    bands = [set() for _ in TREE_REFERENCES]
    for z in range(2, belief.depth):
        for y in range(belief.height):
            for x in range(belief.width):
                if belief.occupancy_probability(x, y, z) <= OCCUPIED_PROBABILITY_THRESHOLD:
                    continue
                pose = grid.voxel_to_metric(x, y, z)
                nearest_index, distance = nearest_tree(pose.x, pose.y)
                if distance <= TREE_RADIUS_M:
                    bands[nearest_index].add(min(HEIGHT_BANDS - 1, int(pose.z - MIN_TREE_HEIGHT_M)))
    return bands


def nearest_tree(x: float, y: float) -> tuple[int, float]:
    return min(enumerate(hypot(x - tx, y - ty) for tx, ty in TREE_REFERENCES), key=lambda item: item[1])
