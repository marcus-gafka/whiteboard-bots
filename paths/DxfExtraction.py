"""Converts DXF entities into sampled point-list geometries."""

import math

import numpy as np

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


def polyline_to_points(vertices, spacing):
    points = []
    vertices = np.array(vertices)
    for i in range(len(vertices) - 1):
        points.extend(line_to_points(vertices[i], vertices[i + 1], spacing=spacing))
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


def spline_to_points(spline, spacing):
    flattened = [(v.x, v.y) for v in spline.flattening(spacing * SPLINE_TOLERANCE_FRACTION)]
    return resample_polyline(flattened, spacing)


def extract_geometries(doc, spacing):
    """Returns (geometries, counts). counts maps 'lines', 'circles', 'arcs',
    'polylines', 'splines' to how many of each were converted, and
    'skipped' to {dxftype: count} for entities that aren't supported."""
    msp = doc.modelspace()
    geometries = []
    counts = {"lines": 0, "circles": 0, "arcs": 0, "polylines": 0, "splines": 0, "skipped": {}}

    for e in msp:
        kind = e.dxftype()
        if kind == 'LINE':
            start = (e.dxf.start.x, e.dxf.start.y)
            end = (e.dxf.end.x, e.dxf.end.y)
            geometries.append(line_to_points(start, end, spacing))
            counts["lines"] += 1

        elif kind == 'CIRCLE':
            center = (e.dxf.center.x, e.dxf.center.y)
            radius = e.dxf.radius
            geometries.append(arc_to_points(center, radius, 0, 360, spacing * 2))
            counts["circles"] += 1

        elif kind == 'ARC':
            center = (e.dxf.center.x, e.dxf.center.y)
            radius = e.dxf.radius
            geometries.append(arc_to_points(center, radius, e.dxf.start_angle, e.dxf.end_angle, spacing * 2))
            counts["arcs"] += 1

        elif kind in ('LWPOLYLINE', 'POLYLINE'):
            vertices = []
            if kind == 'LWPOLYLINE':
                vertices = [(pt[0], pt[1]) for pt in e]
            else:  # POLYLINE
                for v in e.vertices:
                    loc = v.dxf.location
                    vertices.append((loc.x, loc.y))
            geometries.append(polyline_to_points(vertices, spacing * 3))
            counts["polylines"] += 1

        elif kind == 'SPLINE':
            # Same density as arcs: both are smooth curves.
            geometries.append(spline_to_points(e, spacing * 2))
            counts["splines"] += 1

        else:
            counts["skipped"][kind] = counts["skipped"].get(kind, 0) + 1

    return geometries, counts
