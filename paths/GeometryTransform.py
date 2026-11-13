"""Bounding-box and board-fitting transforms for extracted geometries."""

import numpy as np


def get_bounding_box(points):
    points = np.array(points)
    min_x, min_y = points.min(axis=0)
    max_x, max_y = points.max(axis=0)
    return min_x, max_x, min_y, max_y


def scale_and_translate_geometries(robot, geometries):
    """Uniformly scale the drawing to fill the robot's drawable area (limited
    by whichever side is tighter) and centre it there."""
    min_x, max_x, min_y, max_y = get_bounding_box(np.vstack([np.array(g) for g in geometries]))
    width, height = max_x - min_x, max_y - min_y

    board_aspect = robot.DRAWABLE_HEIGHT / robot.DRAWABLE_WIDTH
    scale = robot.DRAWABLE_WIDTH / width if height < board_aspect * width else robot.DRAWABLE_HEIGHT / height
    offset = np.array([robot.DRAWABLE_WIDTH / 2 - (min_x + max_x) / 2 * scale,
                       robot.DRAWABLE_HEIGHT / 2 - (min_y + max_y) / 2 * scale])

    return [(np.array(g) * scale + offset).tolist() for g in geometries]
