"""FastAPI backend for the whiteboard-bot web UI.

Run from the repo root:
    .venv/bin/uvicorn webapp.server:app --reload
then open http://127.0.0.1:8000

This is the only front end: every trajectory is generated, previewed and
sent to the robot (over USB serial, or the simulated port) through it.

Flow: POST /api/jobs (upload or sample) creates a job holding the board-fitted
geometry. The client then either asks for an auto-ordered trajectory, or
builds a manual order through /manual actions and asks for a trajectory from
that, then starts a run. Jobs live in memory only (a single local user is the
expected setup).
"""

import os
import threading
import uuid
from collections import OrderedDict
from typing import Literal

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from serial.tools import list_ports as serial_list_ports

from Constants import Constants as c
from comms.SerialComms import SerialComms
from comms.SimulatedComms import SimulatedComms
from core import pipeline
from core.runner import RobotRunner, RunnerBusy
from paths import ManualOrder as manual_order
from paths import PathOrdering as path_ordering

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
MAX_UPLOAD_BYTES = 20 * 1024 * 1024
MAX_JOBS = 16

# Cap on points sent for piece outlines (display only; the actual drawn path
# is controlled by the user's point count). Dense DXFs sample to hundreds of
# thousands of points, which would bloat responses.
MAX_DISPLAY_POINTS = 20000

app = FastAPI(title="Whiteboard Bots")


class Job:
    def __init__(self, robot, source, counts, geometries):
        self.id = uuid.uuid4().hex[:12]
        self.robot = robot
        self.source = source
        self.counts = counts
        self.geometries = geometries          # board-fitted, original DXF order
        total = sum(len(g) for g in geometries)
        self.stride = max(1, -(-total // MAX_DISPLAY_POINTS))  # ceil
        self.manual = None                    # ManualOrderState, created on demand
        self.lock = threading.Lock()


JOBS = OrderedDict()
JOBS_LOCK = threading.Lock()


def _store(job):
    with JOBS_LOCK:
        JOBS[job.id] = job
        while len(JOBS) > MAX_JOBS:
            JOBS.popitem(last=False)


def _get(job_id):
    with JOBS_LOCK:
        job = JOBS.get(job_id)
        if job:
            JOBS.move_to_end(job_id)
    if job is None:
        raise HTTPException(404, "Job not found. It may have expired; process the drawing again.")
    return job


def _round_pts(points, nd=4):
    return [[round(float(x), nd), round(float(y), nd)] for x, y in points]


def _display(points, stride):
    """Uniformly decimate a piece for display, always keeping both ends."""
    if stride > 1:
        points = points[::stride] + ([points[-1]] if (len(points) - 1) % stride else [])
    return _round_pts(points)


def _board(bot):
    return {"width": bot.DRAWABLE_WIDTH, "height": bot.DRAWABLE_HEIGHT, "start": list(bot.START_XY),
            "robot": pipeline.robot_geometry(bot)}


@app.get("/api/robots")
def list_robots():
    return [{"name": name, **_board(cls())} for name, cls in pipeline.ROBOTS.items()]


@app.get("/api/samples")
def list_samples():
    if not os.path.isdir(c.DXF_FOLDER):
        return []
    return sorted(f for f in os.listdir(c.DXF_FOLDER) if f.lower().endswith(".dxf"))


@app.post("/api/jobs")
async def create_job(
    robot: str = Form(...),
    sample: str | None = Form(None),
    file: UploadFile | None = File(None),
):
    """Upload a DXF (or name a bundled sample); fit it to the robot's board."""
    try:
        bot = pipeline.make_robot(robot)

        if file is not None:
            data = await file.read(MAX_UPLOAD_BYTES + 1)
            if len(data) > MAX_UPLOAD_BYTES:
                raise pipeline.PipelineError("File is larger than 20 MB.")
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
        fitted = pipeline.fit_to_robot(bot, geometries)
    except pipeline.PipelineError as e:
        raise HTTPException(400, str(e))

    job = Job(bot, source, counts, fitted)
    _store(job)
    return {
        "job": job.id,
        "source": source,
        "robot": bot.name,
        "board": _board(bot),
        "counts": counts,
        "piece_count": len(fitted),
    }


# ---------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------

class TrajectoryRequest(BaseModel):
    mode: Literal["auto", "manual"] = "auto"
    points: int | None = None


def _compute(job, mode, points):
    """Ordered pieces -> reduced path -> IK trajectory. Returns a dict of the
    intermediate results; raises HTTPException on user errors."""
    bot = job.robot
    with job.lock:
        if mode == "manual":
            if job.manual is None or not job.manual.is_complete:
                raise HTTPException(400, "Place every piece before generating a manual-order trajectory.")
            ordered = job.manual.ordered_path()
        else:
            ordered = path_ordering.tsp_geometry_order(bot, job.geometries)
    try:
        all_points = pipeline.flatten(ordered)
        path = pipeline.reduce(all_points, points)
    except pipeline.PipelineError as e:
        raise HTTPException(400, str(e))
    unreachable = pipeline.find_unreachable(bot, path)
    traj, commands = ([], []) if unreachable else pipeline.build_trajectory(bot, path)
    return {"ordered": ordered, "all_points": all_points, "path": path,
            "unreachable": unreachable, "trajectory": traj, "commands": commands}


@app.post("/api/jobs/{job_id}/trajectory")
def trajectory(job_id: str, req: TrajectoryRequest):
    job = _get(job_id)
    bot = job.robot
    r = _compute(job, req.mode, req.points)
    return {
        "mode": req.mode,
        "points_requested": req.points,
        "source": job.source,
        "robot": bot.name,
        "board": _board(bot),
        "total_points": len(r["all_points"]),
        "pieces": [_display(g, job.stride) for g in r["ordered"]],
        "path": _round_pts(r["path"]),
        "unreachable": r["unreachable"],
        "rows": [
            [round(x, 3), round(y, 3), round(t1, 3), round(t2, 3), s1, s2, iv]
            for (x, y, t1, t2), (s1, s2, iv) in zip(r["trajectory"], r["commands"])
        ],
        "duration_ms": sum(cmd[2] for cmd in r["commands"]),
        **dict(zip(("poses", "pose_stride"), pipeline.trajectory_poses(bot, r["trajectory"]))),
    }


# ---------------------------------------------------------------------------
# Manual ordering
# ---------------------------------------------------------------------------

class ManualAction(BaseModel):
    action: Literal["place", "cut", "undo", "reset", "autofinish"]
    piece: int | None = None
    reverse: bool = False
    point: tuple[float, float] | None = None


def _manual_state(job, include_pieces=None):
    """include_pieces: None = every live piece, else an iterable of ids."""
    st = job.manual
    ids = st.all_pieces.keys() if include_pieces is None else include_pieces
    pen = st.pen_position(job.robot.START_XY)
    return {
        "pool": sorted(st.pool_ids),
        "placed": [[pid, rev] for pid, rev in st.placed_order],
        "complete": st.is_complete,
        "can_undo": bool(st.history),
        "pen": list(pen),
        "pose": pipeline.pose_at(job.robot, *pen),
        "pieces": {str(pid): _display(st.all_pieces[pid], job.stride) for pid in ids},
        "roots": {str(pid): st.root_of[pid] for pid in ids},
        "cuttable": [pid for pid in st.pool_ids if manual_order.cuttable(st.all_pieces[pid])],
    }


@app.get("/api/jobs/{job_id}/manual")
def manual_get(job_id: str):
    job = _get(job_id)
    with job.lock:
        job.manual = job.manual or manual_order.ManualOrderState(job.geometries)
        return _manual_state(job)


@app.post("/api/jobs/{job_id}/manual")
def manual_act(job_id: str, act: ManualAction):
    job = _get(job_id)
    with job.lock:
        st = job.manual = job.manual or manual_order.ManualOrderState(job.geometries)
        new_ids = []
        try:
            if act.action == "place":
                st.place(_need(act.piece, "piece"), act.reverse)
            elif act.action == "cut":
                point = _need(act.point, "point")
                new_ids = list(st.cut_at_point(_need(act.piece, "piece"), point))
            elif act.action == "undo":
                st.undo()
            elif act.action == "reset":
                job.manual = manual_order.ManualOrderState(job.geometries)
                return _manual_state(job)
            elif act.action == "autofinish":
                pool = sorted(st.pool_ids)
                order = path_ordering.tsp_order_indices(
                    st.pen_position(job.robot.START_XY), [st.all_pieces[pid] for pid in pool])
                st.place_many([(pool[i], rev) for i, rev in order])
        except manual_order.ManualOrderError as e:
            raise HTTPException(400, str(e))
        return _manual_state(job, include_pieces=new_ids)


def _need(value, name):
    if value is None:
        raise HTTPException(422, f"'{name}' is required for this action.")
    return value


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
    job: str
    mode: Literal["auto", "manual"] = "auto"
    points: int | None = None


@app.get("/api/run")
def run_status():
    return RUNNER.status()


@app.post("/api/run")
def run_start(req: RunRequest):
    job = _get(req.job)
    if not RUNNER.status()["at_home"]:
        raise HTTPException(409, "The pen isn't at home (the last run didn't finish). "
                                 "Return it home or confirm it's there before starting.")
    r = _compute(job, req.mode, req.points)
    if r["unreachable"]:
        raise HTTPException(400, f"{len(r['unreachable'])} point(s) are outside {job.robot.name}'s reach.")
    meta = {"kind": "draw", "job": job.id, "mode": req.mode, "points": req.points,
            "robot": job.robot.name, "source": job.source, "port": req.port}
    try:
        RUNNER.start(r["commands"], [(t1, t2) for _, _, t1, t2 in r["trajectory"]],
                     _open_comms(req.port, req.baud), job.robot.name, meta)
    except RunnerBusy as e:
        raise HTTPException(409, str(e))
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
    try:
        RUNNER.start([command], [home], _open_comms(req.port, req.baud), robot_name,
                     {"kind": "home", "robot": robot_name, "port": req.port}, ends_at_home=True)
    except RunnerBusy as e:
        raise HTTPException(409, str(e))
    return RUNNER.status()


@app.post("/api/run/mark-home")
def run_mark_home():
    try:
        RUNNER.mark_home()
    except RunnerBusy as e:
        raise HTTPException(409, str(e))
    return RUNNER.status()


@app.get("/")
def index():
    return FileResponse(os.path.join(STATIC_DIR, "index.html"))


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
