"""Automatic (greedy nearest-neighbor) geometry ordering, plus point-count
reduction for the final drawn path."""

import numpy as np

from paths.GeometryTransform import dist_points


def geometry_dist_to_start(robot, points):
    start = np.array(robot.START_XY)
    p_arr = np.array(points)
    dist_start = np.linalg.norm(p_arr[0] - start)
    dist_end = np.linalg.norm(p_arr[-1] - start)
    return min(dist_start, dist_end)


def geometry_distance(g1, g2):
    g1 = np.array(g1)
    g2 = np.array(g2)
    start = 0
    end = -1
    candidates = [
        (np.linalg.norm(g1[end] - g2[start]), False, False),
        (np.linalg.norm(g1[end] - g2[end]), False, True),
        (np.linalg.norm(g1[start] - g2[start]), True, False),
        (np.linalg.norm(g1[start] - g2[end]), True, True),
    ]
    return min(candidates, key=lambda x: x[0])


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
    reversed_ = bool(d_start[current] > d_end[current])
    order = [(current, reversed_)]
    visited[current] = True

    # Vectorized form of calling geometry_distance(prev, g) for every
    # unvisited g: same four candidate pairings, same tie-breaking (first
    # candidate in that order, then lowest index), but O(n) numpy per step
    # instead of O(n) Python calls — matters for DXFs with thousands of LINEs.
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
    points = np.array(points)
    if n >= len(points):
        return points.tolist()

    step = (len(points) - 1) / (n - 1)
    indices = np.unique(np.round(np.arange(n - 1) * step).astype(int))
    indices = np.append(indices, len(points) - 1)
    indices = np.unique(indices)

    reduced = points[indices]
    return reduced.tolist()
