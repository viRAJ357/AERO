"""Unit tests for aero_exploration.frontier_utils.

Tests frontier detection on synthetic occupancy grids, cluster centroid calculation,
and snapping to free cells.
"""
import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'aero_exploration')))
from aero_exploration.frontier_utils import (
    find_frontiers,
    cluster_centroid,
    snap_to_free,
    score_frontier,
    world_to_grid,
    grid_to_world
)


def test_find_frontiers_synthetic():
    # 20x20 grid, left half free (0), right half unknown (-1)
    grid = np.full((20, 20), -1, dtype=np.int16)
    grid[:, :10] = 0

    # Column 9 is free and adjacent to column 10 (unknown) -> length 20 >= min_size 8
    clusters = find_frontiers(grid, free_max=25, min_size=8)
    assert len(clusters) == 1
    assert len(clusters[0]) == 20


def test_drop_small_clusters():
    # Only 4 frontier cells -> should be dropped when min_size=8
    grid = np.full((20, 20), -1, dtype=np.int16)
    grid[5:9, 5] = 0
    clusters = find_frontiers(grid, free_max=25, min_size=8)
    assert len(clusters) == 0


def test_cluster_centroid():
    cells = np.array([[0, 0], [0, 2], [2, 0], [2, 2]])
    r, c = cluster_centroid(cells)
    assert r == 1.0 and c == 1.0


def test_snap_to_free():
    # 5x5 grid with unknown (-1), and free (0) at center (2, 2)
    grid = np.full((5, 5), -1, dtype=np.int16)
    grid[2, 2] = 0

    # Query from (1, 1) should snap to (2, 2)
    snapped = snap_to_free(1, 1, grid, free_max=25, radius=2)
    assert snapped == (2, 2)


def test_grid_world_transforms():
    res = 0.05
    ox, oy = -10.0, -10.0

    r, c = world_to_grid(0.0, 0.0, ox, oy, res)
    wx, wy = grid_to_world(r, c, ox, oy, res)

    assert abs(wx - 0.0) <= res
    assert abs(wy - 0.0) <= res
