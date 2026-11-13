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


# ---------------------------------------------------------------------------
# Splines + unsupported-entity reporting
# ---------------------------------------------------------------------------

import ezdxf  # noqa: E402
import numpy as np  # noqa: E402


def test_resample_polyline_even_spacing_and_exact_ends():
    pts = dxf.resample_polyline([(0, 0), (1, 0), (1, 0), (1, 3)], spacing=0.5)
    assert pts[0] == pytest.approx([0, 0]) and pts[-1] == pytest.approx([1, 3])
    steps = np.linalg.norm(np.diff(np.array(pts), axis=0), axis=1)
    # Arc-length spacing is uniform; chords that cut the corner are slightly shorter.
    assert len(pts) == 9 and steps.max() == pytest.approx(0.5) and steps.min() > 0.35


def test_spline_extracted_on_curve_with_exact_endpoints():
    doc = ezdxf.new()
    ctrl = [(0, 0), (2, 4), (4, 0)]
    spline = doc.modelspace().add_open_spline(ctrl, degree=2)
    geometries, counts = dxf.extract_geometries(doc, spacing=0.05)

    assert counts["splines"] == 1 and counts["skipped"] == {}
    pts = np.array(geometries[0])
    assert pts[0] == pytest.approx([0, 0]) and pts[-1] == pytest.approx([4, 0])

    # Quadratic Bezier with these control points: the apex is (2, 2).
    assert pts[:, 1].max() == pytest.approx(2.0, abs=0.01)
    true_curve = np.array([(v.x, v.y) for v in spline.construction_tool().approximate(2000)])
    worst = max(np.min(np.linalg.norm(true_curve - p, axis=1)) for p in pts)
    assert worst < 0.01

    steps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    # Sampled at spacing * 2. Like line_to_points, the segment count rounds
    # down, so steps can be a hair over the target.
    assert steps.max() == pytest.approx(0.1, rel=0.01)
    assert steps.min() > 0.08                 # evenly, not bunched up


def test_closed_spline_returns_to_its_start():
    doc = ezdxf.new()
    # A fit-point spline whose last fit point repeats the first is a loop.
    doc.modelspace().add_spline([(0, 0), (4, 0), (4, 4), (0, 4), (0, 0)], degree=3)
    geometries, counts = dxf.extract_geometries(doc, spacing=0.05)
    pts = geometries[0]
    assert counts["splines"] == 1
    assert pts[0] == pytest.approx(pts[-1], abs=1e-6)


def test_unsupported_entities_are_counted_not_silently_dropped():
    doc = ezdxf.new()
    msp = doc.modelspace()
    msp.add_line((0, 0), (1, 0))
    msp.add_ellipse((0, 0), major_axis=(2, 0), ratio=0.5)
    msp.add_text("hi")
    msp.add_text("there")
    geometries, counts = dxf.extract_geometries(doc, spacing=0.1)
    assert len(geometries) == 1 and counts["lines"] == 1
    assert counts["skipped"] == {"ELLIPSE": 1, "TEXT": 2}


def test_m_sample_now_includes_its_splines_and_they_connect():
    from Constants import Constants as c
    import os
    doc = ezdxf.readfile(os.path.join(c.DXF_FOLDER, "m.dxf"))
    geometries, counts = dxf.extract_geometries(doc, dxf.INITIAL_SPACING)
    assert counts["splines"] == 4 and counts["lines"] == 44 and counts["skipped"] == {}
    assert len(geometries) == 48
    # Geometries follow entity order; splines are interleaved with the lines.
    kinds = [e.dxftype() for e in doc.modelspace()]
    lines = [g for g, k in zip(geometries, kinds) if k == "LINE"]
    splines = [g for g, k in zip(geometries, kinds) if k == "SPLINE"]
    # Each spline's ends should meet a neighbouring LINE (within the DXF's own ~1e-4 gaps).
    ends = [g[i] for g in lines for i in (0, -1)]
    for spline_pts in splines:
        for end in (spline_pts[0], spline_pts[-1]):
            assert min(np.hypot(end[0] - x, end[1] - y) for x, y in ends) < 1e-3
