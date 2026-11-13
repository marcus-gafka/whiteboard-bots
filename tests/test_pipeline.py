import os

import pytest

from Constants import Constants as c
from core import pipeline


def _dxf_bytes(name):
    with open(os.path.join(c.DXF_FOLDER, name), "rb") as f:
        return f.read()


def test_make_robot_known_and_unknown():
    assert pipeline.make_robot("FiveBar").name == "FiveBar"
    with pytest.raises(pipeline.PipelineError):
        pipeline.make_robot("Nope")


def test_read_dxf_bytes_rejects_garbage():
    with pytest.raises(pipeline.PipelineError):
        pipeline.read_dxf_bytes(b"this is not a dxf")


def test_full_pipeline_fits_drawing_inside_board():
    robot = pipeline.make_robot("Spooler")
    extraction = pipeline.extract(pipeline.read_dxf_bytes(_dxf_bytes("box.dxf")))
    assert extraction.counts["lines"] == 4

    fitted = pipeline.fit_to_robot(robot, extraction.geometries)
    box = pipeline.bounding_box(fitted)
    eps = 1e-6
    assert box["min_x"] >= -eps and box["max_x"] <= robot.DRAWABLE_WIDTH + eps
    assert box["min_y"] >= -eps and box["max_y"] <= robot.DRAWABLE_HEIGHT + eps

    ordered = pipeline.auto_order(robot, fitted)
    points = pipeline.reduce(pipeline.flatten(ordered), 50)
    assert len(points) == 50

    trajectory, commands = pipeline.build_trajectory(robot, points)
    assert len(trajectory) == len(commands) == 50


def test_reduce_none_keeps_everything_and_rejects_tiny_counts():
    points = [(i, 0.0) for i in range(10)]
    assert pipeline.reduce(points) == points
    with pytest.raises(pipeline.PipelineError):
        pipeline.reduce(points, 1)


def test_find_unreachable_flags_points_far_off_the_board():
    robot = pipeline.make_robot("FiveBar")
    assert pipeline.find_unreachable(robot, [(5.0, 3.0), (500.0, 500.0)]) == [1]
