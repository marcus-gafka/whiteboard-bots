"""Bounding-box and board-fitting transforms for extracted geometries."""

import numpy as np


def get_bounding_box(points):
    points = np.array(points)
    min_x, min_y = points.min(axis=0)
    max_x, max_y = points.max(axis=0)
    return min_x, max_x, min_y, max_y


def print_bounding_box(points):
    min_x, max_x, min_y, max_y = get_bounding_box(points)
    print(f"X: {min_x:.2f} to {max_x:.2f}")
    print(f"Y: {min_y:.2f} to {max_y:.2f}")


def dist_points(p1, p2):
    p1 = np.array(p1)
    p2 = np.array(p2)
    return np.linalg.norm(p1 - p2)


def scale_and_translate_geometries(robot, geometries):
    all_points = np.vstack([np.array(geometry) for geometry in geometries])
    min_x, max_x, min_y, max_y = get_bounding_box(all_points)
    width = max_x - min_x
    height = max_y - min_y

    scale_factor = (robot.DRAWABLE_WIDTH / width) if (height < robot.get_aspect_ratio() * width) else (robot.DRAWABLE_HEIGHT / height)
    tx = robot.DRAWABLE_WIDTH / 2 - ((min_x + max_x) / 2) * scale_factor
    ty = robot.DRAWABLE_HEIGHT / 2 - ((min_y + max_y) / 2) * scale_factor

    scaled_translated = []
    for geometry in geometries:
        pts = np.array(geometry)
        pts = pts * scale_factor
        pts[:, 0] += tx
        pts[:, 1] += ty
        scaled_translated.append(pts.tolist())

    return scaled_translated
