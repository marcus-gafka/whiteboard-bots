import math

import pytest

from paths import DxfExtraction as dxf


def test_line_to_points_endpoints():
    points = dxf.line_to_points((0, 0), (10, 0), spacing=1.0)
    assert points[0] == pytest.approx([0.0, 0.0])
    assert points[-1] == pytest.approx([10.0, 0.0])
    assert len(points) >= 10


def test_arc_to_points_endpoints_on_circle():
    points = dxf.arc_to_points((0, 0), radius=5.0, start_angle=0, end_angle=90, spacing=0.5)
    for x, y in (points[0], points[-1]):
        assert math.hypot(x, y) == pytest.approx(5.0, abs=1e-6)
    assert points[0] == pytest.approx([5.0, 0.0], abs=1e-6)
    assert points[-1] == pytest.approx([0.0, 5.0], abs=1e-6)


def test_polyline_to_points_concatenates_segments():
    vertices = [(0, 0), (10, 0), (10, 10)]
    points = dxf.polyline_to_points(vertices, spacing=1.0)
    assert points[0] == pytest.approx([0.0, 0.0])
    assert points[-1] == pytest.approx([10.0, 10.0])
