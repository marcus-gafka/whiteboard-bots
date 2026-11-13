"""FastAPI backend for the whiteboard-bot web UI.

Run from the repo root:
    .venv/bin/uvicorn webapp.server:app --reload
then open http://127.0.0.1:8000
"""

import os

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from Constants import Constants as c
from core import pipeline

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
MAX_UPLOAD_BYTES = 20 * 1024 * 1024

app = FastAPI(title="Whiteboard Bots")


# Cap on points returned for the piece outlines (display only; the actual
# drawn path is controlled by the user's point count). Dense DXFs sample to
# hundreds of thousands of points, which would bloat the response.
MAX_DISPLAY_POINTS = 20000


def _round_pts(points, nd=4):
    return [[round(float(x), nd), round(float(y), nd)] for x, y in points]


def _thin_for_display(geometries, budget=MAX_DISPLAY_POINTS):
    """Uniformly decimate each piece, always keeping its first and last point."""
    total = sum(len(g) for g in geometries)
    if total <= budget:
        return geometries
    stride = -(-total // budget)  # ceil
    return [g[::stride] + ([g[-1]] if (len(g) - 1) % stride else []) for g in geometries]


@app.get("/api/robots")
def list_robots():
    return [
        {
            "name": name,
            "width": cls.DRAWABLE_WIDTH,
            "height": cls.DRAWABLE_HEIGHT,
            "start": list(cls.START_XY),
        }
        for name, cls in pipeline.ROBOTS.items()
    ]


@app.get("/api/samples")
def list_samples():
    if not os.path.isdir(c.DXF_FOLDER):
        return []
    return sorted(f for f in os.listdir(c.DXF_FOLDER) if f.lower().endswith(".dxf"))


@app.post("/api/process")
async def process(
    robot: str = Form(...),
    points: int | None = Form(None),
    sample: str | None = Form(None),
    file: UploadFile | None = File(None),
):
    """Upload a DXF (or name a bundled sample) and get back the board-fitted,
    auto-ordered drawing plus the IK trajectory for the chosen robot."""
    try:
        bot = pipeline.make_robot(robot)

        if file is not None:
            data = await file.read(MAX_UPLOAD_BYTES + 1)
            if len(data) > MAX_UPLOAD_BYTES:
                raise pipeline.PipelineError("File is larger than 20 MB.")
            source_name = os.path.splitext(file.filename or "upload")[0]
            doc = pipeline.read_dxf_bytes(data)
        elif sample:
            # basename() keeps the lookup inside DXF_FOLDER.
            safe = os.path.basename(sample)
            path = os.path.join(c.DXF_FOLDER, safe)
            if not os.path.isfile(path):
                raise pipeline.PipelineError(f"Sample '{safe}' not found.")
            source_name = os.path.splitext(safe)[0]
            doc = pipeline.read_dxf_file(path)
        else:
            raise pipeline.PipelineError("Upload a DXF file or choose a sample.")

        extraction = pipeline.extract(doc)
        source_box = pipeline.bounding_box(extraction.geometries)
        fitted = pipeline.fit_to_robot(bot, extraction.geometries)
        ordered = pipeline.auto_order(bot, fitted)
        all_points = pipeline.flatten(ordered)
        path = pipeline.reduce(all_points, points)

        unreachable = pipeline.find_unreachable(bot, path)
        trajectory, commands = ([], []) if unreachable else pipeline.build_trajectory(bot, path)
    except pipeline.PipelineError as e:
        raise HTTPException(status_code=400, detail=str(e))

    return {
        "source": source_name,
        "robot": bot.name,
        "board": {"width": bot.DRAWABLE_WIDTH, "height": bot.DRAWABLE_HEIGHT, "start": list(bot.START_XY)},
        "counts": extraction.counts,
        "source_bbox": source_box,
        "total_points": len(all_points),
        "pieces": [_round_pts(g) for g in _thin_for_display(ordered)],
        "path": _round_pts(path),
        "unreachable": unreachable,
        "rows": [
            [round(x, 3), round(y, 3), round(t1, 3), round(t2, 3), s1, s2, iv]
            for (x, y, t1, t2), (s1, s2, iv) in zip(trajectory, commands)
        ],
        "duration_ms": sum(cmd[2] for cmd in commands),
    }


@app.get("/")
def index():
    return FileResponse(os.path.join(STATIC_DIR, "index.html"))


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
