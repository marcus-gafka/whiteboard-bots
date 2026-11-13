"""Converts DXF entities into sampled point-list geometries."""

import math

import numpy as np
from ezdxf.math import Vec3

INITIAL_SPACING = 0.02


def line_to_points(start, end, spacing):
    start = np.array(start)
    end = np.array(end)
    vec = end - start
    length = np.linalg.norm(vec)
    num_segments = max(int(length / spacing), 1)
    t = np.linspace(0, 1, num_segments + 1)
    points = start + np.outer(t, vec)
    return points.tolist()


def arc_to_points(center, radius, start_angle, end_angle, spacing):
    center = np.array(center)
    start_rad = math.radians(start_angle)
    end_rad = math.radians(end_angle)
    if end_rad < start_rad:
        end_rad += 2 * math.pi
    arc_length = radius * (end_rad - start_rad)
    num_segments = max(int(arc_length / spacing), 1)
    angles = np.linspace(start_rad, end_rad, num_segments + 1)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    points = np.stack((x, y), axis=-1)
    return points.tolist()


def bulge_arc_to_points(start, end, bulge, spacing):
    """Points along a polyline arc segment. `bulge` is tan(sweep/4); positive
    sweeps counter-clockwise from start to end. Both ends are kept exactly."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    chord_vec = end - start
    chord = np.linalg.norm(chord_vec)
    if chord == 0:
        return [start.tolist(), end.tolist()]
    sweep = 4.0 * math.atan(bulge)
    radius = chord / (2.0 * abs(math.sin(sweep / 2.0)))
    # Centre sits on the chord's perpendicular bisector, on the left of the
    # travel direction for a CCW (positive) sweep under half a turn.
    mid = (start + end) / 2.0
    left = np.array([-chord_vec[1], chord_vec[0]]) / chord
    center = mid + left * (chord / 2.0) / math.tan(sweep / 2.0)
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    num_segments = max(int(abs(sweep) * radius / spacing), 1)
    angles = a0 + np.linspace(0.0, sweep, num_segments + 1)
    points = np.column_stack((center[0] + radius * np.cos(angles), center[1] + radius * np.sin(angles)))
    points[0], points[-1] = start, end
    return points.tolist()


def polyline_to_points(vertices, spacing, bulges=None, closed=False, arc_spacing=None):
    """Samples each segment separately so vertices (corners) are kept exactly.
    bulges[i] curves the segment that starts at vertex i (0 = straight).
    closed adds the segment from the last vertex back to the first."""
    vertices = np.asarray(vertices, dtype=float)
    n = len(vertices)
    bulges = [0.0] * n if bulges is None else list(bulges)
    arc_spacing = spacing if arc_spacing is None else arc_spacing

    segments = [(i, i + 1) for i in range(n - 1)]
    if closed and n >= 2 and (bulges[-1] or np.linalg.norm(vertices[-1] - vertices[0]) > 0):
        segments.append((n - 1, 0))

    points = []
    for i, j in segments:
        if bulges[i]:
            points.extend(bulge_arc_to_points(vertices[i], vertices[j], bulges[i], arc_spacing))
        else:
            points.extend(line_to_points(vertices[i], vertices[j], spacing=spacing))
    return points


# Max deviation of the flattened spline from the true curve, as a fraction of
# the sampling spacing — small enough that resampling, not flattening,
# dominates the error.
SPLINE_TOLERANCE_FRACTION = 0.05


def resample_polyline(vertices, spacing):
    """Evenly spaced points along a polyline by arc length, keeping both
    endpoints exactly. Used for curves that ezdxf has already flattened,
    where the vertices are dense and uneven."""
    pts = np.asarray(vertices, dtype=float)[:, :2]
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    pts = pts[np.concatenate([[True], seg > 0])]   # drop repeated vertices
    if len(pts) < 2:
        return pts.tolist()
    cum = np.concatenate([[0.0], np.cumsum(seg[seg > 0])])
    num_segments = max(int(cum[-1] / spacing), 1)
    s = np.linspace(0.0, cum[-1], num_segments + 1)
    return np.column_stack((np.interp(s, cum, pts[:, 0]), np.interp(s, cum, pts[:, 1]))).tolist()


def curve_to_points(entity, spacing):
    """SPLINE or ELLIPSE (both WCS): flatten finely with ezdxf, then resample evenly."""
    flattened = [(v.x, v.y) for v in entity.flattening(spacing * SPLINE_TOLERANCE_FRACTION)]
    return resample_polyline(flattened, spacing)


def _to_wcs(entity, points, elevation=0.0):
    """ARC/CIRCLE/polyline coordinates are in the entity's OCS. With the usual
    +Z extrusion that's plain XY; a -Z extrusion (what CAD "mirror" produces)
    flips X, which ignoring would draw the shape mirrored."""
    ext = entity.dxf.get("extrusion", (0.0, 0.0, 1.0))
    if abs(ext[0]) < 1e-12 and abs(ext[1]) < 1e-12 and ext[2] > 0:
        return points
    ocs = entity.ocs()
    return [[v.x, v.y] for v in ocs.points_to_wcs(Vec3(x, y, elevation) for x, y in points)]


def _polyline_parts(e):
    """(vertices_xy, bulges, closed, elevation) for LWPOLYLINE / 2D-3D POLYLINE,
    or None for POLYLINE meshes (not drawable as a single stroke)."""
    if e.dxftype() == "LWPOLYLINE":
        pts = list(e.get_points("xyb"))
        return [(x, y) for x, y, _ in pts], [b for _, _, b in pts], e.closed, e.dxf.elevation
    if e.is_poly_face_mesh or e.is_polygon_mesh:
        return None
    vs = list(e.vertices)
    verts = [(v.dxf.location.x, v.dxf.location.y) for v in vs]
    # 3D polylines are already WCS and have no bulges.
    bulges = [0.0] * len(vs) if e.is_3d_polyline else [v.dxf.get("bulge", 0.0) for v in vs]
    elevation = vs[0].dxf.location.z if vs and e.is_2d_polyline else 0.0
    return verts, bulges, e.is_closed, elevation


def extract_geometries(doc, spacing):
    """Returns (geometries, counts). counts maps 'lines', 'circles', 'arcs',
    'polylines', 'splines', 'ellipses' to how many of each were converted,
    and 'skipped' to {dxftype: count} for entities that aren't supported."""
    msp = doc.modelspace()
    geometries = []
    counts = {"lines": 0, "circles": 0, "arcs": 0, "polylines": 0, "splines": 0, "ellipses": 0,
              "skipped": {}}

    def skip(kind):
        counts["skipped"][kind] = counts["skipped"].get(kind, 0) + 1

    for e in msp:
        kind = e.dxftype()
        if kind == 'LINE':
            start = (e.dxf.start.x, e.dxf.start.y)
            end = (e.dxf.end.x, e.dxf.end.y)
            geometries.append(line_to_points(start, end, spacing))
            counts["lines"] += 1

        elif kind in ('CIRCLE', 'ARC'):
            center = (e.dxf.center.x, e.dxf.center.y)
            radius = e.dxf.radius
            a0, a1 = (0, 360) if kind == 'CIRCLE' else (e.dxf.start_angle, e.dxf.end_angle)
            pts = arc_to_points(center, radius, a0, a1, spacing * 2)
            geometries.append(_to_wcs(e, pts, e.dxf.center.z))
            counts["circles" if kind == 'CIRCLE' else "arcs"] += 1

        elif kind in ('LWPOLYLINE', 'POLYLINE'):
            parts = _polyline_parts(e)
            if parts is None:
                skip("POLYLINE (mesh)")
                continue
            vertices, bulges, closed, elevation = parts
            # Straight runs keep their historical density; curved (bulge)
            # segments use the same density as ARC entities.
            pts = polyline_to_points(vertices, spacing * 3, bulges=bulges, closed=closed,
                                     arc_spacing=spacing * 2)
            geometries.append(_to_wcs(e, pts, elevation))
            counts["polylines"] += 1

        elif kind in ('SPLINE', 'ELLIPSE'):
            # Same density as arcs: smooth curves.
            geometries.append(curve_to_points(e, spacing * 2))
            counts["splines" if kind == 'SPLINE' else "ellipses"] += 1

        else:
            skip(kind)

    return geometries, counts
