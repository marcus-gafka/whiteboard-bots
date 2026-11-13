"""Automatic (greedy nearest-neighbor) geometry ordering, plus point-count
reduction for the final drawn path."""

import numpy as np


def tsp_order_indices(start_xy, geometries):
    """Greedy nearest-neighbour draw order starting from the pen at
    `start_xy`. Returns [(index_into_geometries, reversed), ...]."""
    n = len(geometries)
    if n == 0:
        return []
    visited = np.zeros(n, dtype=bool)
    start = np.asarray(start_xy, dtype=float)

    starts = np.array([g[0] for g in geometries], dtype=float)
    ends = np.array([g[-1] for g in geometries], dtype=float)

    # First piece: whichever has an endpoint closest to the pen, oriented to
    # begin at the nearer end.
    d_start = np.linalg.norm(starts - start, axis=1)
    d_end = np.linalg.norm(ends - start, axis=1)
    current = int(np.argmin(np.minimum(d_start, d_end)))
    order = [(current, bool(d_start[current] > d_end[current]))]
    visited[current] = True

    # Each step scores every piece by its best of four endpoint pairings with
    # the previous piece, vectorized so DXFs with thousands of LINEs stay fast.
    for _ in range(n - 1):
        idx, rev = order[-1]
        prev_start, prev_end = (ends[idx], starts[idx]) if rev else (starts[idx], ends[idx])
        candidates = np.stack([
            np.linalg.norm(starts - prev_end, axis=1),    # end -> start
            np.linalg.norm(ends - prev_end, axis=1),      # end -> end (reverse next)
            np.linalg.norm(starts - prev_start, axis=1),  # start -> start
            np.linalg.norm(ends - prev_start, axis=1),    # start -> end (reverse next)
        ], axis=1)
        which = np.argmin(candidates, axis=1)
        dists = candidates[np.arange(n), which]
        dists[visited] = np.inf

        best = int(np.argmin(dists))
        order.append((best, bool(which[best] in (1, 3))))
        visited[best] = True

    return order


def tsp_geometry_order(robot, geometries):
    return [list(reversed(geometries[i])) if rev else geometries[i]
            for i, rev in tsp_order_indices(robot.START_XY, geometries)]


def reduce_points(points, n):
    """n points evenly spaced by index along `points`, keeping both ends."""
    points = np.array(points)
    if n >= len(points):
        return points.tolist()
    step = (len(points) - 1) / (n - 1)
    indices = np.unique(np.append(np.round(np.arange(n - 1) * step).astype(int), len(points) - 1))
    return points[indices].tolist()
