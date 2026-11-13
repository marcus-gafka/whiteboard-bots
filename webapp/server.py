"""FastAPI backend for the whiteboard-bot web UI.

Run from the repo root:
    .venv/bin/uvicorn webapp.server:app --reload
then open http://127.0.0.1:8000

Flow: POST /api/jobs (upload or sample) creates a job holding the board-fitted
geometry. The client then either asks for an auto-ordered trajectory, or
builds a manual order through /manual actions and asks for a trajectory from
that. Jobs live in memory only (a single local user is the expected setup).
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

from Constants import Constants as c
from core import pipeline
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
    def __init__(self, robot, source, counts, source_bbox, geometries):
        self.id = uuid.uuid4().hex[:12]
        self.robot = robot
        self.source = source
        self.counts = counts
        self.source_bbox = source_bbox
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
    return {"width": bot.DRAWABLE_WIDTH, "height": bot.DRAWABLE_HEIGHT, "start": list(bot.START_XY)}


@app.get("/api/robots")
def list_robots():
    return [{"name": name, **_board(cls)} for name, cls in pipeline.ROBOTS.items()]


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
            doc = pipeline.read_dxf_bytes(data)
        elif sample:
            # basename() keeps the lookup inside DXF_FOLDER.
            safe = os.path.basename(sample)
            path = os.path.join(c.DXF_FOLDER, safe)
            if not os.path.isfile(path):
                raise pipeline.PipelineError(f"Sample '{safe}' not found.")
            source = os.path.splitext(safe)[0]
            doc = pipeline.read_dxf_file(path)
        else:
            raise pipeline.PipelineError("Upload a DXF file or choose a sample.")

        extraction = pipeline.extract(doc)
        source_bbox = pipeline.bounding_box(extraction.geometries)
        fitted = pipeline.fit_to_robot(bot, extraction.geometries)
    except pipeline.PipelineError as e:
        raise HTTPException(400, str(e))

    job = Job(bot, source, extraction.counts, source_bbox, fitted)
    _store(job)
    return {
        "job": job.id,
        "source": source,
        "robot": bot.name,
        "board": _board(bot),
        "counts": job.counts,
        "source_bbox": source_bbox,
        "piece_count": len(fitted),
        "total_points": sum(len(g) for g in fitted),
    }


# ---------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------

class TrajectoryRequest(BaseModel):
    mode: Literal["auto", "manual"] = "auto"
    points: int | None = None


@app.post("/api/jobs/{job_id}/trajectory")
def trajectory(job_id: str, req: TrajectoryRequest):
    job = _get(job_id)
    bot = job.robot
    with job.lock:
        if req.mode == "manual":
            if job.manual is None or not job.manual.is_complete:
                raise HTTPException(400, "Place every piece before generating a manual-order trajectory.")
            ordered = job.manual.ordered_path()
        else:
            ordered = pipeline.auto_order(bot, job.geometries)

    try:
        all_points = pipeline.flatten(ordered)
        path = pipeline.reduce(all_points, req.points)
    except pipeline.PipelineError as e:
        raise HTTPException(400, str(e))

    unreachable = pipeline.find_unreachable(bot, path)
    traj, commands = ([], []) if unreachable else pipeline.build_trajectory(bot, path)

    return {
        "mode": req.mode,
        "source": job.source,
        "robot": bot.name,
        "board": _board(bot),
        "total_points": len(all_points),
        "pieces": [_display(g, job.stride) for g in ordered],
        "path": _round_pts(path),
        "unreachable": unreachable,
        "rows": [
            [round(x, 3), round(y, 3), round(t1, 3), round(t2, 3), s1, s2, iv]
            for (x, y, t1, t2), (s1, s2, iv) in zip(traj, commands)
        ],
        "duration_ms": sum(cmd[2] for cmd in commands),
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
    return {
        "pool": sorted(st.pool_ids),
        "placed": [[pid, rev] for pid, rev in st.placed_order],
        "complete": st.is_complete,
        "can_undo": bool(st.history),
        "pen": list(st.pen_position(job.robot.START_XY)),
        "pieces": {str(pid): _display(st.all_pieces[pid], job.stride) for pid in ids},
        "roots": {str(pid): st.root_of[pid] for pid in ids},
        "cuttable": [pid for pid in st.pool_ids if manual_order.cuttable(st.all_pieces[pid])],
    }


@app.get("/api/jobs/{job_id}/manual")
def manual_get(job_id: str):
    job = _get(job_id)
    with job.lock:
        if job.manual is None:
            job.manual = manual_order.ManualOrderState(job.geometries)
        return _manual_state(job)


@app.post("/api/jobs/{job_id}/manual")
def manual_act(job_id: str, act: ManualAction):
    job = _get(job_id)
    with job.lock:
        if job.manual is None:
            job.manual = manual_order.ManualOrderState(job.geometries)
        st = job.manual
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
                job.manual = st = manual_order.ManualOrderState(job.geometries)
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


@app.get("/")
def index():
    return FileResponse(os.path.join(STATIC_DIR, "index.html"))


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
