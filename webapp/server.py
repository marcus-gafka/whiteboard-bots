"""FastAPI backend for the whiteboard-bot web UI.

Run from the repo root:
    .venv/bin/uvicorn webapp.server:app --reload
then open http://127.0.0.1:8000

The web app is the only front end. Three pages, one per workflow step:

1. Trajectory generator (/): load a DXF, order its pieces (auto, or manual
   with cuts and flips) and save the ordered path as a trajectory, kept in
   the DXF's own units so any robot can draw it at its own size.
2. Robot preview (/preview): pick a saved trajectory (or import a points
   file) and a robot. The path is fitted to that robot's board and turned
   into steps1/steps2/interval commands, previewed with the moving linkage.
   Nothing is sent to a robot from here.
3. Live (/live): the only place a trajectory is sent to the robot, over USB
   serial (or the simulated port), with run controls and status.

Drawings being ordered live in memory (a single local user is the expected
setup); saved trajectories are JSON files in TRAJECTORY_DIR.
"""

import os
import threading
import uuid
from collections import OrderedDict
from typing import Literal

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from serial.tools import list_ports as serial_list_ports

from Constants import Constants as c
from comms.SerialComms import SerialComms
from comms.SimulatedComms import SimulatedComms
from core import pipeline
from core import trajectory_store as store
from core.runner import RobotRunner, RunnerBusy
from paths import ManualOrder as manual_order
from paths import PathOrdering as path_ordering

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
TRAJECTORY_DIR = c.TRAJECTORY_FOLDER
MAX_UPLOAD_BYTES = 20 * 1024 * 1024
MAX_DRAWINGS = 16

# Cap on points sent for piece outlines (display only; never affects what is
# saved or drawn). Dense DXFs sample to hundreds of thousands of points.
MAX_DISPLAY_POINTS = 20000

app = FastAPI(title="Whiteboard Bots")


# User-facing errors raised below the HTTP layer, mapped to status codes once
# here instead of in a try/except around every call.
def _error_handler(status, message=None):
    def handle(_request, exc):
        return JSONResponse({"detail": message or str(exc)}, status_code=status)
    return handle


app.add_exception_handler(pipeline.PipelineError, _error_handler(400))
app.add_exception_handler(manual_order.ManualOrderError, _error_handler(400))
app.add_exception_handler(store.TrajectoryNotFound, _error_handler(404, "Trajectory not found."))
app.add_exception_handler(RunnerBusy, _error_handler(409))


def _stride(total_points):
    return max(1, -(-total_points // MAX_DISPLAY_POINTS))  # ceil


def _round_pts(points, nd=4):
    return [[round(float(x), nd), round(float(y), nd)] for x, y in points]


async def _read_upload(file):
    data = await file.read(MAX_UPLOAD_BYTES + 1)
    if len(data) > MAX_UPLOAD_BYTES:
        raise pipeline.PipelineError("File is larger than 20 MB.")
    return data


def _name(requested, default):
    return (requested or "").strip()[:80] or default


def _display(points, stride):
    """Uniformly decimate a piece for display, always keeping both ends."""
    if stride > 1:
        points = points[::stride] + ([points[-1]] if (len(points) - 1) % stride else [])
    return _round_pts(points)


# ---------------------------------------------------------------------------
# 1. Trajectory generator: drawings in DXF units
# ---------------------------------------------------------------------------

class Drawing:
    def __init__(self, source, counts, geometries):
        self.id = uuid.uuid4().hex[:12]
        self.source = source
        self.counts = counts
        self.geometries = geometries          # DXF units, original entity order
        self.bounds = pipeline.bounds(geometries)
        self.start = pipeline.drawing_start(geometries)
        self.stride = _stride(sum(len(g) for g in geometries))
        self.manual = None                    # ManualOrderState, created on demand
        self.lock = threading.Lock()


DRAWINGS = OrderedDict()
DRAWINGS_LOCK = threading.Lock()


def _store_drawing(drawing):
    with DRAWINGS_LOCK:
        DRAWINGS[drawing.id] = drawing
        while len(DRAWINGS) > MAX_DRAWINGS:
            DRAWINGS.popitem(last=False)


def _get_drawing(drawing_id):
    with DRAWINGS_LOCK:
        drawing = DRAWINGS.get(drawing_id)
        if drawing:
            DRAWINGS.move_to_end(drawing_id)
    if drawing is None:
        raise HTTPException(404, "Drawing not found. It may have expired; load it again.")
    return drawing


@app.get("/api/samples")
def list_samples():
    if not os.path.isdir(c.DXF_FOLDER):
        return []
    return sorted(f for f in os.listdir(c.DXF_FOLDER) if f.lower().endswith(".dxf"))


@app.post("/api/drawings")
async def create_drawing(sample: str | None = Form(None), file: UploadFile | None = File(None)):
    """Upload a DXF (or name a bundled sample) and extract its pieces."""
    if file is not None:
        data = await _read_upload(file)
        source = os.path.splitext(os.path.basename(file.filename or "upload"))[0]
    elif sample:
        # basename() keeps the lookup inside DXF_FOLDER.
        safe = os.path.basename(sample)
        path = os.path.join(c.DXF_FOLDER, safe)
        if not os.path.isfile(path):
            raise pipeline.PipelineError(f"Sample '{safe}' not found.")
        source = os.path.splitext(safe)[0]
        with open(path, "rb") as f:
            data = f.read()
    else:
        raise pipeline.PipelineError("Upload a DXF file or choose a sample.")
    geometries, counts = pipeline.extract(pipeline.read_dxf(data))

    drawing = Drawing(source, counts, geometries)
    _store_drawing(drawing)
    return {
        "drawing": drawing.id,
        "source": source,
        "counts": counts,
        "bounds": drawing.bounds,
        "start": list(drawing.start),
        "piece_count": len(geometries),
        "total_points": sum(len(g) for g in geometries),
    }


class OrderRequest(BaseModel):
    mode: Literal["auto", "manual"] = "auto"


def _ordered(drawing, mode):
    with drawing.lock:
        if mode == "manual":
            if drawing.manual is None or not drawing.manual.is_complete:
                raise HTTPException(400, "Place every piece before using the manual order.")
            return drawing.manual.ordered_path()
    return pipeline.auto_order(drawing.geometries)


@app.post("/api/drawings/{drawing_id}/order")
def order(drawing_id: str, req: OrderRequest):
    """The ordered pieces (display-decimated) for previewing an order."""
    drawing = _get_drawing(drawing_id)
    return {"mode": req.mode, "pieces": [_display(p, drawing.stride) for p in _ordered(drawing, req.mode)]}


class ManualAction(BaseModel):
    action: Literal["place", "cut", "flip", "undo", "reset", "autofinish"]
    piece: int | None = None
    reverse: bool = False
    point: tuple[float, float] | None = None


def _manual_state(drawing, include_pieces=None):
    """include_pieces: None = every live piece, else an iterable of ids."""
    st = drawing.manual
    ids = st.all_pieces.keys() if include_pieces is None else include_pieces
    return {
        "pool": sorted(st.pool_ids),
        "placed": [[pid, rev] for pid, rev in st.placed_order],
        "complete": st.is_complete,
        "can_undo": bool(st.history),
        "pen": list(st.pen_position(drawing.start)),
        "pieces": {str(pid): _display(st.all_pieces[pid], drawing.stride) for pid in ids},
        "roots": {str(pid): st.root_of[pid] for pid in ids},
        "cuttable": [pid for pid in st.pool_ids if manual_order.cuttable(st.all_pieces[pid])],
    }


@app.get("/api/drawings/{drawing_id}/manual")
def manual_get(drawing_id: str):
    drawing = _get_drawing(drawing_id)
    with drawing.lock:
        drawing.manual = drawing.manual or manual_order.ManualOrderState(drawing.geometries)
        return _manual_state(drawing)


@app.post("/api/drawings/{drawing_id}/manual")
def manual_act(drawing_id: str, act: ManualAction):
    drawing = _get_drawing(drawing_id)
    with drawing.lock:
        st = drawing.manual = drawing.manual or manual_order.ManualOrderState(drawing.geometries)
        new_ids = []
        if act.action == "place":
            st.place(_need(act.piece, "piece"), act.reverse)
        elif act.action == "flip":
            st.flip(_need(act.piece, "piece"))
        elif act.action == "cut":
            point = _need(act.point, "point")
            new_ids = list(st.cut_at_point(_need(act.piece, "piece"), point))
        elif act.action == "undo":
            st.undo()
        elif act.action == "reset":
            drawing.manual = manual_order.ManualOrderState(drawing.geometries)
            return _manual_state(drawing)
        elif act.action == "autofinish":
            pool = sorted(st.pool_ids)
            seq = path_ordering.tsp_order_indices(
                st.pen_position(drawing.start), [st.all_pieces[pid] for pid in pool])
            st.place_many([(pool[i], rev) for i, rev in seq])
        return _manual_state(drawing, include_pieces=new_ids)


def _need(value, name):
    if value is None:
        raise HTTPException(422, f"'{name}' is required for this action.")
    return value


# ---------------------------------------------------------------------------
# Saved trajectories
# ---------------------------------------------------------------------------

class SaveRequest(BaseModel):
    drawing: str
    mode: Literal["auto", "manual"] = "auto"
    name: str | None = None


@app.post("/api/trajectories")
def save_trajectory(req: SaveRequest):
    drawing = _get_drawing(req.drawing)
    pieces = _ordered(drawing, req.mode)
    name = _name(req.name, f"{drawing.source} ({req.mode})")
    return store.save(TRAJECTORY_DIR, name=name, source=drawing.source, mode=req.mode,
                      pieces=pieces, bounds=drawing.bounds)


@app.post("/api/trajectories/import")
async def import_trajectory(file: UploadFile = File(...), name: str | None = Form(None)):
    """Save an uploaded points file (see pipeline.parse_points) as a trajectory."""
    data = await _read_upload(file)
    pieces = pipeline.parse_points(data.decode("utf-8", errors="replace"))
    source = os.path.splitext(os.path.basename(file.filename or "points"))[0]
    return store.save(TRAJECTORY_DIR, name=_name(name, f"{source} (imported)"), source=source, mode="imported",
                      pieces=pieces, bounds=pipeline.bounds(pieces))


@app.get("/api/trajectories")
def list_trajectories():
    return store.list_meta(TRAJECTORY_DIR)


@app.delete("/api/trajectories/{tid}")
def delete_trajectory(tid: str):
    store.delete(TRAJECTORY_DIR, tid)
    return {"deleted": tid}


# ---------------------------------------------------------------------------
# 2. Robot preview: fit a trajectory to a robot and compute its commands
# ---------------------------------------------------------------------------

def _board(bot):
    return {"width": bot.DRAWABLE_WIDTH, "height": bot.DRAWABLE_HEIGHT, "start": list(bot.START_XY),
            "robot": pipeline.robot_geometry(bot)}


@app.get("/api/robots")
def list_robots():
    return [{"name": name, **_board(cls())} for name, cls in pipeline.ROBOTS.items()]


class PlanRequest(BaseModel):
    robot: str
    points: int | None = None


def _plan(tid, robot_name, points):
    """Returns (meta, robot, plan)."""
    meta, pieces = store.load(TRAJECTORY_DIR, tid)
    bot = pipeline.make_robot(robot_name)
    return meta, bot, pipeline.plan(bot, pieces, points)


@app.post("/api/trajectories/{tid}/plan")
def plan_trajectory(tid: str, req: PlanRequest):
    meta, bot, p = _plan(tid, req.robot, req.points)
    stride = _stride(p["total_points"])
    poses, pose_stride = pipeline.trajectory_poses(bot, p["trajectory"])
    return {
        "trajectory": meta,
        "robot": bot.name,
        "board": _board(bot),
        "total_points": p["total_points"],
        "pieces": [_display(g, stride) for g in p["fitted"]],
        "path": _round_pts(p["path"]),
        "unreachable": p["unreachable"],
        "rows": [
            [round(x, 3), round(y, 3), round(t1, 3), round(t2, 3), s1, s2, iv]
            for (x, y, t1, t2), (s1, s2, iv) in zip(p["trajectory"], p["commands"])
        ],
        "duration_ms": sum(cmd[2] for cmd in p["commands"]),
        "poses": poses,
        "pose_stride": pose_stride,
    }


# ---------------------------------------------------------------------------
# Sending to the robot
# ---------------------------------------------------------------------------

SIMULATED_PORT = "simulated"
RUNNER = RobotRunner()


def _open_comms(port, baud):
    if port == SIMULATED_PORT:
        return SimulatedComms
    return lambda: SerialComms(port, baud)


@app.get("/api/ports")
def list_ports():
    """USB serial devices (legacy /dev/ttyS* entries without hardware info
    are hidden), plus a simulated port for trying the flow without a robot."""
    found = [
        {"device": p.device, "description": p.description if p.description != "n/a" else p.device}
        for p in sorted(serial_list_ports.comports(), key=lambda p: p.device)
        if p.hwid and p.hwid != "n/a"
    ]
    return found + [{"device": SIMULATED_PORT, "description": "Simulated (no robot)"}]


class PortRequest(BaseModel):
    port: str
    baud: int = c.DEFAULT_SERIAL_BAUD


class RunRequest(PortRequest):
    trajectory: str
    robot: str
    points: int | None = None


@app.get("/api/run")
def run_status():
    return RUNNER.status()


@app.post("/api/run")
def run_start(req: RunRequest):
    if not RUNNER.status()["at_home"]:
        raise HTTPException(409, "The pen isn't at home (the last run didn't finish). "
                                 "Return it home or confirm it's there before starting.")
    meta, bot, p = _plan(req.trajectory, req.robot, req.points)
    if p["unreachable"]:
        raise HTTPException(400, f"{len(p['unreachable'])} point(s) are outside {bot.name}'s reach.")
    run_meta = {"kind": "draw", "trajectory": meta["id"], "name": meta["name"], "robot": bot.name,
                "points": req.points, "port": req.port}
    RUNNER.start(p["commands"], [(t1, t2) for _, _, t1, t2 in p["trajectory"]],
                 _open_comms(req.port, req.baud), bot.name, run_meta)
    return RUNNER.status()


@app.post("/api/run/pause")
def run_pause():
    RUNNER.pause()
    return RUNNER.status()


@app.post("/api/run/resume")
def run_resume():
    RUNNER.resume()
    return RUNNER.status()


@app.post("/api/run/stop")
def run_stop():
    RUNNER.stop()
    return RUNNER.status()


@app.post("/api/run/home")
def run_home(req: PortRequest):
    """Move from the last recorded position straight back to home. The pen
    can't lift, so this draws a line on the board."""
    pos = RUNNER.position()
    if pos is None:
        raise HTTPException(400, "The pen is already at home.")
    robot_name, t1, t2 = pos
    bot = pipeline.make_robot(robot_name)
    home = bot.ik(*bot.START_XY)
    command = bot.calc_steps(t1, t2, *home)
    RUNNER.start([command], [home], _open_comms(req.port, req.baud), robot_name,
                 {"kind": "home", "robot": robot_name, "port": req.port}, ends_at_home=True)
    return RUNNER.status()


@app.post("/api/run/mark-home")
def run_mark_home():
    RUNNER.mark_home()
    return RUNNER.status()


# ---------------------------------------------------------------------------
# Pages
# ---------------------------------------------------------------------------

@app.get("/")
def generator_page():
    return FileResponse(os.path.join(STATIC_DIR, "generator.html"))


@app.get("/preview")
def preview_page():
    return FileResponse(os.path.join(STATIC_DIR, "preview.html"))


@app.get("/live")
def live_page():
    return FileResponse(os.path.join(STATIC_DIR, "live.html"))


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
