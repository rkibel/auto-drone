import pytest

from auto_drone.forest_metrics import tree_coverage
from auto_drone.interfaces import VoxelGridSpec
from auto_drone.mapping import BeliefVolume


def test_tree_coverage_reports_observed_height_bands() -> None:
    grid = VoxelGridSpec(50, 60, 20, 1.0, -25.0, -16.0, 0.0)
    belief = BeliefVolume(50, 60, 20)
    belief.observe(5, 5, 2, occupied=True)
    belief.observe(5, 5, 2, occupied=True)
    belief.observe(5, 5, 2, occupied=True)
    belief.observe(5, 5, 4, occupied=True)
    belief.observe(5, 5, 4, occupied=True)
    belief.observe(5, 5, 4, occupied=True)

    coverage = tree_coverage(belief, grid)

    assert coverage.observed_trees == 1
    assert coverage.observed_percentage == pytest.approx(100.0 / 12.0)
    assert coverage.reconstruction_percentage == pytest.approx(2.0 / (12.0 * 8.0) * 100.0)
    assert coverage.mean_observed_tree_quality == 25.0
    assert coverage.tree_band_counts[0] == 2
