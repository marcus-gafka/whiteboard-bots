"""Web-app processing pipeline: DXF -> board-fitted, ordered points ->
trajectory, plus the linkage poses the preview draws.

Pure with respect to I/O (no HTTP, no files); failures are raised as
PipelineError with a message fit to show a user.
"""

import io

import ezdxf
from ezdxf import recover

from paths import DxfExtraction as dxf_extraction
from paths import GeometryTransform as geometry_transform
from paths import PathOrdering as path_ordering
from robots.FiveBar import FiveBar
from robots.Spooler import Spooler
from robots.TwoLink import TwoLink

ROBOTS = {cls.name: cls for cls in (FiveBar, Spooler, TwoLink)}

# Longest per-move pose list sent for display; longer trajectories are
# sampled with a stride (the drawn path itself is never decimated).
MAX_POSES = 4000


class PipelineError(Exception):
    """A user-facing processing failure (bad file, nothing drawable, ...)."""


def make_robot(name):
    try:
        return ROBOTS[name]()
    except KeyError:
        raise PipelineError(f"Unknown robot '{name}'. Choose one of: {', '.join(ROBOTS)}")


def read_dxf(data):
    """Parse DXF bytes. Uses ezdxf's recover mode, which tolerates the minor
    structural errors common in files exported by other tools."""
    try:
        doc, _auditor = recover.read(io.BytesIO(data))
    except ezdxf.DXFError as e:
        raise PipelineError(f"Not a readable DXF file: {e}")
    return doc


def extract(doc, spacing=dxf_extraction.INITIAL_SPACING):
    """Returns (geometries, counts); geometries are point lists in DXF units."""
    geometries, counts = dxf_extraction.extract_geometries(doc, spacing)
    # Fewer than 2 points can't be drawn as a stroke.
    geometries = [g for g in geometries if len(g) >= 2]
    if not geometries:
        skipped = ", ".join(sorted(counts["skipped"])) or "none"
        raise PipelineError("No drawable LINE, CIRCLE, ARC, POLYLINE, SPLINE or ELLIPSE entities found in "
                            f"the DXF (unsupported entities present: {skipped}).")
    return geometries, counts


def fit_to_robot(robot, geometries):
    min_x, max_x, min_y, max_y = geometry_transform.get_bounding_box(flatten(geometries))
    if max_x - min_x <= 0 and max_y - min_y <= 0:
        raise PipelineError("Drawing has zero size; nothing to scale onto the board.")
    return geometry_transform.scale_and_translate_geometries(robot, geometries)


def flatten(geometries):
    return [tuple(pt) for g in geometries for pt in g]


def reduce(points, n=None):
    """Down-sample to n points (None = keep all)."""
    if n is None or n >= len(points):
        return [tuple(p) for p in points]
    if n < 2:
        raise PipelineError("Point count must be at least 2.")
    return [tuple(p) for p in path_ordering.reduce_points(points, n)]


def find_unreachable(robot, points):
    """Indices of points the robot's IK can't solve (e.g. outside its reach)."""
    bad = []
    for i, (x, y) in enumerate(points):
        try:
            robot.ik(x, y)
        except (ValueError, ZeroDivisionError):
            bad.append(i)
    return bad


def build_trajectory(robot, points):
    """Runs IK + step/timing calculation for each point in order.
    Returns (trajectory, commands):
    trajectory: list of (x, y, theta1_deg, theta2_deg)
    commands: list of (steps1, steps2, interval_ms)
    """
    trajectory, commands = [], []
    current_t1, current_t2 = robot.ik(*robot.START_XY)
    for x, y in points:
        t1, t2 = robot.ik(x, y)
        commands.append(robot.calc_steps(current_t1, current_t2, t1, t2))
        trajectory.append((x, y, t1, t2))
        current_t1, current_t2 = t1, t2
    return trajectory, commands


def link_pose(robot, theta1_deg, theta2_deg, nd=3):
    """The linkage at the given joint angles as flat board-frame segments:
    [x1, y1, x2, y2, ...], one quadruple per link. The first link starts at
    the pen tip. Returns None if the pose can't be evaluated."""
    try:
        links = robot.get_links(theta1_deg, theta2_deg)
    except (ValueError, ZeroDivisionError, TypeError):
        return None
    flat = []
    for seg in links:
        for p in seg:
            if p is None:
                return None
            bx, by = robot.apply_inverse_board_offset(float(p[0]), float(p[1]))
            flat += [round(bx, nd), round(by, nd)]
    return flat


def pose_at(robot, x, y):
    """Linkage pose with the pen at board point (x, y), or None if unreachable."""
    try:
        return link_pose(robot, *robot.ik(x, y))
    except (ValueError, ZeroDivisionError):
        return None


def robot_geometry(robot):
    """Static description of the mechanism for display, in board coordinates:
    the view bounds (the robot's plot bounds), fixed base points, link kind,
    and the pose at the home position."""
    x0, y0 = robot.apply_inverse_board_offset(robot.MIN_PLOT_X, robot.MIN_PLOT_Y)
    x1, y1 = robot.apply_inverse_board_offset(robot.MAX_PLOT_X, robot.MAX_PLOT_Y)
    return {
        "bounds": [float(x0), float(y0), float(x1), float(y1)],
        "bases": [list(map(float, robot.apply_inverse_board_offset(*b))) for b in robot.get_bases()],
        "kind": robot.LINK_KIND,
        "home": pose_at(robot, *robot.START_XY),
    }


def trajectory_poses(robot, trajectory, max_poses=MAX_POSES):
    """Linkage pose after each move, sampled every `stride` moves (pose j is
    the pose after move min(j * stride, n - 1)). Returns (poses, stride)."""
    n = len(trajectory)
    if not n:
        return [], 1
    stride = max(1, -(-n // max_poses))
    idx = list(range(0, n, stride))
    if idx[-1] != n - 1:
        idx.append(n - 1)
    return [link_pose(robot, trajectory[i][2], trajectory[i][3]) for i in idx], stride
