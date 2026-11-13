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

    for _ in range(n - 1):
        best_dist = float('inf')
        best_index = None
        best_geom_reversed = False

        for i in range(n):
            if visited[i]:
                continue
            dist, _, rev_next = geometry_distance(path[-1], geometries[i])
            if dist < best_dist:
                best_dist = dist
                best_index = i
                best_geom_reversed = rev_next

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
