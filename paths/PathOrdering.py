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


def tsp_geometry_order(robot, geometries):
    n = len(geometries)
    visited = np.zeros(n, dtype=bool)
    path = []

    distances_to_start = np.array([geometry_dist_to_start(robot, g) for g in geometries])
    current_index = np.argmin(distances_to_start)
    current_geometry = geometries[current_index]
    visited[current_index] = True

    dist_to_start_first = dist_points(robot.START_XY, current_geometry[0])
    dist_to_start_last = dist_points(robot.START_XY, current_geometry[-1])
    if dist_to_start_first > dist_to_start_last:
        current_geometry = list(reversed(current_geometry))

    path.append(current_geometry)

    # Vectorized form of calling geometry_distance(path[-1], g) for every
    # unvisited g: same four candidate pairings, same tie-breaking (first
    # candidate in that order, then lowest index), but O(n) numpy per step
    # instead of O(n) Python calls — matters for DXFs with thousands of LINEs.
    starts = np.array([g[0] for g in geometries], dtype=float)
    ends = np.array([g[-1] for g in geometries], dtype=float)

    for _ in range(n - 1):
        prev_start = np.asarray(path[-1][0], dtype=float)
        prev_end = np.asarray(path[-1][-1], dtype=float)
        candidates = np.stack([
            np.linalg.norm(starts - prev_end, axis=1),    # end -> start
            np.linalg.norm(ends - prev_end, axis=1),      # end -> end (reverse next)
            np.linalg.norm(starts - prev_start, axis=1),  # start -> start
            np.linalg.norm(ends - prev_start, axis=1),    # start -> end (reverse next)
        ], axis=1)
        which = np.argmin(candidates, axis=1)
        dists = candidates[np.arange(n), which]
        dists[visited] = np.inf

        best_index = int(np.argmin(dists))
        best_geom_reversed = bool(which[best_index] in (1, 3))

        next_geom = geometries[best_index]
        if best_geom_reversed:
            next_geom = list(reversed(next_geom))
        path.append(next_geom)
        visited[best_index] = True

    return path


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
