import os

import pytest

from Constants import Constants as c
from core import pipeline
from paths import GeometryTransform as geometry_transform
from paths import PathOrdering as path_ordering


def _dxf_bytes(name):
    with open(os.path.join(c.DXF_FOLDER, name), "rb") as f:
        return f.read()


def test_make_robot_known_and_unknown():
    assert pipeline.make_robot("FiveBar").name == "FiveBar"
    with pytest.raises(pipeline.PipelineError):
        pipeline.make_robot("Nope")


def test_read_dxf_rejects_garbage():
    with pytest.raises(pipeline.PipelineError):
        pipeline.read_dxf(b"this is not a dxf")


def test_full_pipeline_fits_drawing_inside_board():
    robot = pipeline.make_robot("Spooler")
    geometries, counts = pipeline.extract(pipeline.read_dxf(_dxf_bytes("box.dxf")))
    assert counts["lines"] == 4

    fitted = pipeline.fit_to_robot(robot, geometries)
    min_x, max_x, min_y, max_y = geometry_transform.get_bounding_box(pipeline.flatten(fitted))
    eps = 1e-6
    assert min_x >= -eps and max_x <= robot.DRAWABLE_WIDTH + eps
    assert min_y >= -eps and max_y <= robot.DRAWABLE_HEIGHT + eps

    ordered = path_ordering.tsp_geometry_order(robot, fitted)
    points = pipeline.reduce(pipeline.flatten(ordered), 50)
    assert len(points) == 50

    trajectory, commands = pipeline.build_trajectory(robot, points)
    assert len(trajectory) == len(commands) == 50


def test_reduce_none_keeps_everything_and_rejects_tiny_counts():
    points = [(i, 0.0) for i in range(10)]
    assert pipeline.reduce(points) == points
    with pytest.raises(pipeline.PipelineError):
        pipeline.reduce(points, 1)


@pytest.mark.parametrize("name", ["FiveBar", "TwoLink"])
def test_find_unreachable_flags_points_far_off_the_board(name):
    robot = pipeline.make_robot(name)
    assert pipeline.find_unreachable(robot, [(3.0, 3.0), (500.0, 500.0)]) == [1]


@pytest.mark.parametrize("name", list(pipeline.ROBOTS))
def test_robot_geometry_home_pose_puts_tip_at_start(name):
    bot = pipeline.make_robot(name)
    geo = pipeline.robot_geometry(bot)
    x0, y0, x1, y1 = geo["bounds"]
    assert x0 < x1 and y0 < y1
    # The drawable board lies inside the view bounds.
    assert x0 <= 0 and y0 <= 0 and x1 >= bot.DRAWABLE_WIDTH and y1 >= bot.DRAWABLE_HEIGHT
    assert geo["bases"] and geo["kind"] in ("rigid", "cable")
    home = geo["home"]
    assert len(home) % 4 == 0
    assert home[:2] == pytest.approx(list(bot.START_XY), abs=1e-3)
    # Every base point is an endpoint of some link.
    ends = {(round(home[i], 2), round(home[i + 1], 2)) for i in range(0, len(home), 2)}
    for bx, by in geo["bases"]:
        assert (round(bx, 2), round(by, 2)) in ends


def test_trajectory_poses_follow_the_pen_and_respect_the_cap():
    bot = pipeline.make_robot("FiveBar")
    path = [(1 + 8 * i / 99, 1 + 4 * i / 99) for i in range(100)]
    traj, _ = pipeline.build_trajectory(bot, path)

    poses, stride = pipeline.trajectory_poses(bot, traj)
    assert stride == 1 and len(poses) == 100
    for (x, y), pose in zip(path, poses):
        assert pose[0] == pytest.approx(x, abs=1e-2) and pose[1] == pytest.approx(y, abs=1e-2)

    poses, stride = pipeline.trajectory_poses(bot, traj, max_poses=30)
    assert stride == 4 and len(poses) == 26          # 0, 4, ..., 96, plus the last move
    assert poses[-1][:2] == pytest.approx(list(path[-1]), abs=1e-2)
    assert pipeline.trajectory_poses(bot, []) == ([], 1)


def test_pose_at_unreachable_point_is_none():
    assert pipeline.pose_at(pipeline.make_robot("FiveBar"), 500.0, 500.0) is None
