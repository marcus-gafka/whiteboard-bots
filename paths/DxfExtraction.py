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


def extract_geometries(doc, spacing):
    msp = doc.modelspace()
    geometries = []
    lines, circles, arcs, polylines = 0, 0, 0, 0

    for e in msp:
        if e.dxftype() == 'LINE':
            start = (e.dxf.start.x, e.dxf.start.y)
            end = (e.dxf.end.x, e.dxf.end.y)
            geometries.append(line_to_points(start, end, spacing))
            lines += 1

        elif e.dxftype() == 'CIRCLE':
            center = (e.dxf.center.x, e.dxf.center.y)
            radius = e.dxf.radius
            geometries.append(arc_to_points(center, radius, 0, 360, spacing * 2))
            circles += 1

        elif e.dxftype() == 'ARC':
            center = (e.dxf.center.x, e.dxf.center.y)
            radius = e.dxf.radius
            geometries.append(arc_to_points(center, radius, e.dxf.start_angle, e.dxf.end_angle, spacing * 2))
            arcs += 1

        elif e.dxftype() in ('LWPOLYLINE', 'POLYLINE'):
            vertices = []
            if e.dxftype() == 'LWPOLYLINE':
                vertices = [(pt[0], pt[1]) for pt in e]
            else:  # POLYLINE
                for v in e.vertices:
                    loc = v.dxf.location
                    vertices.append((loc.x, loc.y))
            geometries.append(polyline_to_points(vertices, spacing * 3))
            polylines += 1

    return geometries, lines, circles, arcs, polylines
