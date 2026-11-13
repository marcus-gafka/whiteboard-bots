"""Processing pipeline, in two robot-independent / robot-specific halves:

1. Trajectory generation (no robot): DXF -> pieces in the DXF's own units ->
   ordered, oriented pieces. Ordering starts from drawing_start().
2. Robot planning: ordered pieces -> fitted to a robot's board -> reduced
   path -> IK + (steps1, steps2, interval) commands, plus the linkage poses
   the preview draws.

Pure with respect to I/O (no HTTP, no files); failures are raised as
PipelineError with a message fit to show a user.
"""

import io
import json
import re

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


# ---------------------------------------------------------------------------
# 1. Trajectory generation (robot-independent, DXF units)
# ---------------------------------------------------------------------------

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
    min_x, max_x, min_y, max_y = geometry_transform.get_bounding_box(flatten(geometries))
    if max_x - min_x <= 0 and max_y - min_y <= 0:
        raise PipelineError("Drawing has zero size; nothing to draw.")
    return geometries, counts


def bounds(geometries):
    """[min_x, min_y, max_x, max_y] of all points."""
    min_x, max_x, min_y, max_y = geometry_transform.get_bounding_box(flatten(geometries))
    return [float(min_x), float(min_y), float(max_x), float(max_y)]


def drawing_start(geometries):
    """Where the pen starts while ordering, without knowing the robot: the
    drawing's bottom-left bounding-box corner. Every robot's home is its
    board's bottom-left corner and drawings are centred on the board, so this
    is where home ends up relative to the drawing on any robot."""
    x0, y0, _, _ = bounds(geometries)
    return (x0, y0)


def auto_order(geometries):
    return path_ordering.tsp_geometry_order(drawing_start(geometries), geometries)


def flatten(geometries):
    return [tuple(pt) for g in geometries for pt in g]


def parse_points(text):
    """Pieces from an imported points file, already in draw order.

    - JSON: a saved trajectory ({"pieces": [...]}), a list of pieces, or a
      single list of [x, y] points.
    - Text (.txt / .csv): one point per line, x and y as the first two
      columns (comma, tab, space or semicolon separated). Lines that don't
      start with two numbers (headers, comments) are skipped; a blank line
      starts a new piece. The web app's steps .txt export also works.
    """
    text = text.strip()
    if text[:1] in "[{":
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                data = data["pieces"]
            if data and isinstance(data[0][0], (int, float)):
                data = [data]
            pieces = [[(float(p[0]), float(p[1])) for p in piece] for piece in data]
        except (ValueError, KeyError, IndexError, TypeError) as e:
            raise PipelineError(f"Not a usable points JSON file ({type(e).__name__}).")
    else:
        pieces = [[]]
        for line in text.splitlines():
            if not line.strip():
                if pieces[-1]:
                    pieces.append([])
                continue
            parts = re.split(r"[,\s;]+", line.strip())
            try:
                pieces[-1].append((float(parts[0]), float(parts[1])))
            except (ValueError, IndexError):
                continue
    pieces = [p for p in pieces if len(p) >= 2]
    if not pieces:
        raise PipelineError("No points found. Expected lines of 'x, y' (blank lines separate pieces).")
    x0, y0, x1, y1 = bounds(pieces)
    if x1 - x0 <= 0 and y1 - y0 <= 0:
        raise PipelineError("The points have zero extent; nothing to draw.")
    return pieces


# ---------------------------------------------------------------------------
# 2. Robot planning
# ---------------------------------------------------------------------------

def make_robot(name):
    try:
        return ROBOTS[name]()
    except KeyError:
        raise PipelineError(f"Unknown robot '{name}'. Choose one of: {', '.join(ROBOTS)}")


def fit_to_robot(robot, pieces):
    """Scale the pieces (any units) to fill the robot's drawable board, centred."""
    return geometry_transform.scale_and_translate_geometries(robot, pieces)


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


def plan(robot, pieces, points=None):
    """Ordered pieces (any units) -> everything needed to preview or run them
    on `robot`. Commands are only built when every point is reachable."""
    fitted = fit_to_robot(robot, pieces)
    all_points = flatten(fitted)
    path = reduce(all_points, points)
    unreachable = find_unreachable(robot, path)
    trajectory, commands = ([], []) if unreachable else build_trajectory(robot, path)
    return {"fitted": fitted, "total_points": len(all_points), "path": path,
            "unreachable": unreachable, "trajectory": trajectory, "commands": commands}


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
