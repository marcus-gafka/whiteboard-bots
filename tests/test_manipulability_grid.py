import numpy as np
import pytest

from robots.ManipulabilityGrid import ManipulabilityGrid
from robots.TwoLink import TwoLink


def test_manipulability_grid_shape_and_range():
    robot = TwoLink()
    grid = ManipulabilityGrid(robot, resolution_x=5, resolution_y=4)

    assert grid.values.shape == (4, 5)
    assert grid.robot_x.shape == (4, 5)
    assert grid.robot_y.shape == (4, 5)
    assert np.all(grid.values >= 0.0)
    # At least the interior/reachable samples should be genuinely
    # manipulable (not every cell silently zeroed out).
    assert np.any(grid.values > 0.0)


def test_manipulability_grid_extent_is_robot_frame():
    robot = TwoLink()
    grid = ManipulabilityGrid(robot, resolution_x=3, resolution_y=3)
    xmin, xmax, ymin, ymax = grid.extent

    expected_xmin, expected_ymin = robot.apply_board_offset(0, 0)
    expected_xmax, expected_ymax = robot.apply_board_offset(
        robot.DRAWABLE_WIDTH, robot.DRAWABLE_HEIGHT
    )
    assert xmin == pytest.approx(expected_xmin)
    assert xmax == pytest.approx(expected_xmax)
    assert ymin == pytest.approx(expected_ymin)
    assert ymax == pytest.approx(expected_ymax)
