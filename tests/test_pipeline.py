import os

import pytest

from Constants import Constants as c
from core import pipeline
from paths import GeometryTransform as geometry_transform


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


def test_ordering_happens_in_dxf_units_and_any_robot_can_plan_it():
    geometries, counts = pipeline.extract(pipeline.read_dxf(_dxf_bytes("box.dxf")))
    assert counts["lines"] == 4
    ordered = pipeline.auto_order(geometries)
    # Ordering doesn't rescale: the pieces keep the DXF's own coordinates.
    assert pipeline.bounds(ordered) == pipeline.bounds(geometries)
    assert pipeline.flatten(ordered)[0] == pytest.approx(pipeline.drawing_start(geometries), abs=1e-9)

    for name in pipeline.ROBOTS:
        robot = pipeline.make_robot(name)
        p = pipeline.plan(robot, ordered, 50)
        min_x, max_x, min_y, max_y = geometry_transform.get_bounding_box(pipeline.flatten(p["fitted"]))
        eps = 1e-6
        assert min_x >= -eps and max_x <= robot.DRAWABLE_WIDTH + eps
        assert min_y >= -eps and max_y <= robot.DRAWABLE_HEIGHT + eps
        assert len(p["path"]) == len(p["trajectory"]) == len(p["commands"]) == 50
        assert p["unreachable"] == []


def test_plan_keeps_the_saved_order():
    # Two strokes ordered right-to-left on purpose; planning must not reorder them.
    pieces = [[(10.0, 0.0), (8.0, 0.0)], [(2.0, 0.0), (0.0, 2.0)]]
    p = pipeline.plan(pipeline.make_robot("Spooler"), pieces)
    xs = [x for x, _ in p["path"]]
    assert xs[0] > xs[1] > xs[2]


def test_extract_rejects_zero_size_drawing():
    import ezdxf
    doc = ezdxf.new()
    doc.modelspace().add_line((1, 1), (1, 1))
    with pytest.raises(pipeline.PipelineError):
        pipeline.extract(doc)


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


def test_parse_points_text_with_header_and_pieces():
    text = "x\ty\n0, 0\n1 0\n\n\n5;5\n6\t6\n7 7 extra columns ignored\n# comment\n"
    assert pipeline.parse_points(text) == [[(0, 0), (1, 0)], [(5, 5), (6, 6), (7, 7)]]


def test_parse_points_json_forms():
    assert pipeline.parse_points('{"meta": {}, "pieces": [[[0, 0], [1, 1]]]}') == [[(0, 0), (1, 1)]]
    assert pipeline.parse_points("[[0, 0], [2, 1]]") == [[(0, 0), (2, 1)]]
    assert pipeline.parse_points("[[[0, 0], [2, 1]], [[3, 3], [4, 4]]]") == [[(0, 0), (2, 1)], [(3, 3), (4, 4)]]


@pytest.mark.parametrize("bad", ["", "hello\nworld", "1 1\n1 1", "{not json", '{"x": 1}', "[[0, 0]]"])
def test_parse_points_rejects_unusable_input(bad):
    with pytest.raises(pipeline.PipelineError):
        pipeline.parse_points(bad)
