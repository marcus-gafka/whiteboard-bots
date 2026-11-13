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
    msp.add_point((5, 5))
    msp.add_text("hi")
    msp.add_text("there")
    geometries, counts = dxf.extract_geometries(doc, spacing=0.1)
    assert len(geometries) == 1 and counts["lines"] == 1
    assert counts["skipped"] == {"POINT": 1, "TEXT": 2}


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


# ---------------------------------------------------------------------------
# Bulges, closed polylines, ellipses, mirrored (-Z extrusion) entities.
# Checked against ezdxf's own path geometry as an independent reference.
# ---------------------------------------------------------------------------

from ezdxf import path as ezpath  # noqa: E402


def _reference(entity, spacing=0.002):
    flat = [(v.x, v.y) for v in ezpath.make_path(entity).flattening(0.0002)]
    return np.array(dxf.resample_polyline(flat, spacing))


def _assert_matches_reference(pts, entity, tol=0.005):
    pts, ref = np.array(pts), _reference(entity)
    on_curve = max(np.min(np.linalg.norm(ref - p, axis=1)) for p in pts)
    covered = max(np.min(np.linalg.norm(pts - r, axis=1)) for r in ref[::5])
    assert on_curve < tol, f"sampled point {on_curve:.4f} off the true curve"
    # Spacing between our samples is <= 0.06, so every part of the curve is within ~0.03.
    assert covered < 0.035, f"part of the true curve is {covered:.4f} from any sample (missing segment?)"


def test_bulge_arc_direction_and_shape():
    # bulge 1 = half circle; positive sweeps CCW, so going (0,0)->(2,0) it dips below.
    pts = np.array(dxf.bulge_arc_to_points((0, 0), (2, 0), 1.0, 0.05))
    assert pts[0] == pytest.approx([0, 0]) and pts[-1] == pytest.approx([2, 0])
    assert np.allclose(np.linalg.norm(pts - [1, 0], axis=1), 1.0)
    assert pts[:, 1].min() == pytest.approx(-1.0, abs=1e-3) and pts[:, 1].max() <= 1e-9
    neg = np.array(dxf.bulge_arc_to_points((0, 0), (2, 0), -1.0, 0.05))
    assert neg[:, 1].max() == pytest.approx(1.0, abs=1e-3)


@pytest.mark.parametrize("closed", [False, True])
@pytest.mark.parametrize("mirrored", [False, True])
def test_lwpolyline_with_bulges_matches_ezdxf(closed, mirrored):
    doc = ezdxf.new()
    attribs = {"extrusion": (0, 0, -1)} if mirrored else {}
    pl = doc.modelspace().add_lwpolyline(
        [(0, 0, 0), (3, 0, 0.5), (3, 2, -0.3), (1, 3, 0), (0, 2, 0.8)],
        format="xyb", close=closed, dxfattribs=attribs)
    geometries, counts = dxf.extract_geometries(doc, spacing=0.02)
    assert counts["polylines"] == 1
    _assert_matches_reference(geometries[0], pl)
    if closed:  # closing segment drawn, curved by the last vertex's bulge
        assert geometries[0][-1] == pytest.approx(geometries[0][0])


def test_2d_polyline_with_bulges_matches_ezdxf():
    doc = ezdxf.new()
    pl = doc.modelspace().add_polyline2d([(0, 0), (4, 0), (4, 3)], format="xy", close=True)
    pl.vertices[0].dxf.bulge = 0.4
    pl.vertices[2].dxf.bulge = -0.6
    geometries, _ = dxf.extract_geometries(doc, spacing=0.02)
    _assert_matches_reference(geometries[0], pl)


def test_straight_polyline_segments_unchanged_and_corners_exact():
    vertices = [(0, 0), (1, 0), (1, 1)]
    assert dxf.polyline_to_points(vertices, 0.1) == (
        dxf.line_to_points(vertices[0], vertices[1], 0.1) + dxf.line_to_points(vertices[1], vertices[2], 0.1))
    closed = dxf.polyline_to_points(vertices, 0.1, closed=True)
    assert closed[-1] == pytest.approx([0, 0])
    assert [1.0, 1.0] in closed and [1.0, 0.0] in closed


@pytest.mark.parametrize("params", [(0, 2 * np.pi), (0.3, 2.0), (5.5, 1.0)])
def test_ellipse_full_and_partial_match_ezdxf(params):
    doc = ezdxf.new()
    el = doc.modelspace().add_ellipse((1, 2), major_axis=(3, 1), ratio=0.4,
                                      start_param=params[0], end_param=params[1])
    geometries, counts = dxf.extract_geometries(doc, spacing=0.02)
    assert counts["ellipses"] == 1 and counts["skipped"] == {}
    _assert_matches_reference(geometries[0], el)


@pytest.mark.parametrize("kind", ["arc", "circle"])
def test_mirrored_arc_and_circle_match_ezdxf(kind):
    doc = ezdxf.new()
    msp = doc.modelspace()
    ext = {"extrusion": (0, 0, -1)}
    e = (msp.add_arc((2, 1), 1.5, 20, 160, dxfattribs=ext) if kind == "arc"
         else msp.add_circle((2, 1), 1.5, dxfattribs=ext))
    geometries, _ = dxf.extract_geometries(doc, spacing=0.02)
    _assert_matches_reference(geometries[0], e)


def test_polyline_meshes_are_reported_as_skipped():
    doc = ezdxf.new()
    doc.modelspace().add_polymesh(size=(2, 2))
    geometries, counts = dxf.extract_geometries(doc, spacing=0.1)
    assert geometries == [] and counts["skipped"] == {"POLYLINE (mesh)": 1}


def _chunks(n, per_item, max_elems=2_000_000):
    """Slices over range(n) sized so each temporary holds about max_elems
    floats (~16 MB) — unbounded broadcasting here once used ~13 GB."""
    step = max(1, max_elems // max(1, per_item))
    return (slice(i, i + step) for i in range(0, n, step))


def _min_dists(a, b):
    """For each point in a, distance to the nearest point in b."""
    return np.concatenate([np.linalg.norm(a[s, None, :] - b[None, :, :], axis=2).min(axis=1)
                           for s in _chunks(len(a), 2 * len(b))])


def _dists_to_polyline(a, poly):
    """For each point in a, distance to the nearest segment of polyline poly."""
    p0, d = poly[:-1], np.diff(poly, axis=0)
    dd = np.maximum((d * d).sum(axis=1), 1e-300)
    out = []
    for s in _chunks(len(a), 4 * len(d)):
        rel = a[s, None, :] - p0[None, :, :]
        t = np.clip((rel * d).sum(axis=2) / dd, 0.0, 1.0)
        out.append(np.linalg.norm(rel - t[..., None] * d, axis=2).min(axis=1))
    return np.concatenate(out)


@pytest.mark.parametrize("name", ["gompei.dxf", "heart.dxf", "r.dxf", "wpi_text.dxf", "volleyball.dxf"])
def test_sample_polylines_match_ezdxf_geometry(name):
    from Constants import Constants as c
    import os
    doc = ezdxf.readfile(os.path.join(c.DXF_FOLDER, name))
    spacing = dxf.INITIAL_SPACING
    geometries, _ = dxf.extract_geometries(doc, spacing)
    for g, e in list(zip(geometries, doc.modelspace()))[:2]:
        pts = np.array(g)
        outline = np.array([(v.x, v.y) for v in ezpath.make_path(e).flattening(spacing / 100)])
        # Every sampled point lies on the true outline...
        sample = pts[:: max(1, len(pts) // 400)]
        assert _dists_to_polyline(sample, outline).max() < spacing / 5
        # ...and the whole outline (including the closing segment) is covered.
        ref = np.array(dxf.resample_polyline(outline, spacing))
        assert _min_dists(ref[:: max(1, len(ref) // 400)], pts).max() < spacing * 2
