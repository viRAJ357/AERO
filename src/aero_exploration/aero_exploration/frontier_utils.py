"""Pure Python frontier detection utilities.

No ROS dependency - can be fully unit-tested with pytest.
"""
import math
from typing import List, Tuple, Optional

import numpy as np
from scipy import ndimage


def find_frontiers(
    grid: np.ndarray,
    free_max: int = 25,
    min_size: int = 8,
) -> List[np.ndarray]:
    """Find frontier clusters in an OccupancyGrid.

    A frontier cell is a free cell (occupancy in [0, free_max)) that is
    adjacent (8-connected) to at least one unknown cell (-1).

    Args:
        grid:     2-D int array from an OccupancyGrid.
                  Values: -1 = unknown, 0..100 = occupancy probability.
        free_max: Threshold below which a cell is considered free.
        min_size: Minimum cluster size (cells). Smaller clusters are dropped.

    Returns:
        List of arrays, each (N, 2) with (row, col) of cells in that cluster.
    """
    free = (grid >= 0) & (grid < free_max)
    unknown = grid == -1
    near_unknown = ndimage.binary_dilation(unknown, structure=np.ones((3, 3)))
    frontier = free & near_unknown
    labels, n = ndimage.label(frontier, structure=np.ones((3, 3)))
    clusters: List[np.ndarray] = []
    for k in range(1, n + 1):
        cells = np.argwhere(labels == k)   # (row, col) pairs
        if len(cells) >= min_size:
            clusters.append(cells)
    return clusters


def cluster_centroid(cells: np.ndarray) -> Tuple[float, float]:
    """Return the (row, col) centroid of a cluster."""
    return float(np.mean(cells[:, 0])), float(np.mean(cells[:, 1]))


def snap_to_free(
    row: int, col: int, grid: np.ndarray, free_max: int = 25, radius: int = 5
) -> Optional[Tuple[int, int]]:
    """Snap a centroid to the nearest free cell within radius.

    Returns (row, col) of nearest free cell, or None if none found.
    """
    r0 = max(0, row - radius)
    r1 = min(grid.shape[0], row + radius + 1)
    c0 = max(0, col - radius)
    c1 = min(grid.shape[1], col + radius + 1)
    sub = grid[r0:r1, c0:c1]
    free_mask = (sub >= 0) & (sub < free_max)
    indices = np.argwhere(free_mask)
    if len(indices) == 0:
        return None
    # Pick closest to centroid
    dists = np.linalg.norm(indices - np.array([row - r0, col - c0]), axis=1)
    best = indices[np.argmin(dists)]
    return int(best[0] + r0), int(best[1] + c0)


def score_frontier(
    cells: np.ndarray,
    centroid: Tuple[float, float],
    robot_row: float,
    robot_col: float,
    size_weight: float = 1.0,
    dist_weight: float = 0.5,
) -> float:
    """Score a frontier: large clusters near the robot score higher.

    Args:
        cells:        Cluster cells (N, 2).
        centroid:     (row, col) centroid.
        robot_row:    Robot position (rows).
        robot_col:    Robot position (cols).
        size_weight:  Weight for cluster size.
        dist_weight:  Penalty weight for distance.

    Returns:
        Score (higher = better).
    """
    size = len(cells)
    dist = math.sqrt(
        (centroid[0] - robot_row) ** 2 + (centroid[1] - robot_col) ** 2
    )
    return size_weight * size - dist_weight * dist


def world_to_grid(
    x: float, y: float, origin_x: float, origin_y: float, resolution: float
) -> Tuple[int, int]:
    """Convert world (x, y) to grid (row, col)."""
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return row, col


def grid_to_world(
    row: int, col: int, origin_x: float, origin_y: float, resolution: float
) -> Tuple[float, float]:
    """Convert grid (row, col) to world (x, y) at cell centre."""
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return x, y
