"""UI-agnostic processing pipeline: DXF -> ordered board points -> trajectory.

Every function here is pure with respect to I/O (no input()/print(), no
plotting), so the same code backs the CLI in main.py and the web app.
Failures are raised as PipelineError with a message fit to show a user.
"""

import io
from dataclasses import dataclass, field

import ezdxf
from ezdxf import recover

from paths import DxfExtraction as dxf_extraction
from paths import GeometryTransform as geometry_transform
from paths import PathOrdering as path_ordering
from robots.FiveBar import FiveBar
from robots.Spooler import Spooler
from robots.TwoLink import TwoLink

ROBOTS = {cls.name: cls for cls in (FiveBar, Spooler, TwoLink)}


class PipelineError(Exception):
    """A user-facing processing failure (bad file, nothing drawable, ...)."""


@dataclass
class Extraction:
    geometries: list            # list of point lists, in DXF units
    counts: dict = field(default_factory=dict)  # entity kind -> count, plus 'skipped': {dxftype: n}


def make_robot(name):
    try:
        return ROBOTS[name]()
    except KeyError:
        raise PipelineError(f"Unknown robot '{name}'. Choose one of: {', '.join(ROBOTS)}")


def read_dxf_file(path):
    try:
        return ezdxf.readfile(path)
    except (IOError, ezdxf.DXFError) as e:
        raise PipelineError(f"Could not read DXF '{path}': {e}")


def read_dxf_bytes(data):
    """Parse an uploaded DXF. Uses ezdxf's recover mode, which tolerates the
    minor structural errors common in files exported by other tools."""
    try:
        doc, _auditor = recover.read(io.BytesIO(data))
    except ezdxf.DXFError as e:
        raise PipelineError(f"Not a readable DXF file: {e}")
    return doc


def extract(doc, spacing=dxf_extraction.INITIAL_SPACING):
    geometries, counts = dxf_extraction.extract_geometries(doc, spacing)
    # Drop degenerate geometries (fewer than 2 points can't be drawn as a stroke).
    geometries = [g for g in geometries if len(g) >= 2]
    if not geometries:
        skipped = ", ".join(sorted(counts["skipped"])) or "none"
        raise PipelineError("No drawable LINE, CIRCLE, ARC, POLYLINE or SPLINE entities found in the DXF "
                            f"(unsupported entities present: {skipped}).")
    return Extraction(geometries, counts)


def bounding_box(geometries):
    min_x, max_x, min_y, max_y = geometry_transform.get_bounding_box(flatten(geometries))
    return {"min_x": float(min_x), "max_x": float(max_x), "min_y": float(min_y), "max_y": float(max_y)}


def fit_to_robot(robot, geometries):
    box = bounding_box(geometries)
    if box["max_x"] - box["min_x"] <= 0 and box["max_y"] - box["min_y"] <= 0:
        raise PipelineError("Drawing has zero size; nothing to scale onto the board.")
    return geometry_transform.scale_and_translate_geometries(robot, geometries)


def auto_order(robot, geometries):
    return path_ordering.tsp_geometry_order(robot, geometries)


def flatten(geometries):
    return [tuple(pt) for g in geometries for pt in g]


def reduce(points, n=None):
    """Down-sample to n points (None = keep all)."""
    if n is None or n >= len(points):
        return [tuple(p) for p in points]
    if n < 2:
        raise PipelineError("Point count must be at least 2.")
    return [tuple(p) for p in path_ordering.reduce_points(points, n)]


def build_trajectory(robot, points):
    """Runs IK + step/timing calculation for each point in order.
    Returns (trajectory, commands):
    trajectory: list of (x, y, theta1_deg, theta2_deg)
    commands: list of (steps1, steps2, interval_ms)
    """
    trajectory, commands = [], []
    current_t1, current_t2 = robot.ik(robot.START_X, robot.START_Y)
    for x, y in points:
        t1, t2 = robot.ik(x, y)
        s1, s2, interval = robot.calc_steps(current_t1, current_t2, t1, t2)
        trajectory.append((x, y, t1, t2))
        commands.append((s1, s2, interval))
        current_t1, current_t2 = t1, t2
    return trajectory, commands


def find_unreachable(robot, points):
    """Indices of points the robot's IK can't solve (e.g. outside its reach)."""
    bad = []
    for i, (x, y) in enumerate(points):
        try:
            robot.ik(x, y)
        except (ValueError, ZeroDivisionError):
            bad.append(i)
    return bad
