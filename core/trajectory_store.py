"""Saved trajectories: a drawing's ordered, oriented pieces in the source DXF's
own units (robot-independent), one JSON file per trajectory:

    {"meta": {id, name, source, mode, created, piece_count, point_count, bounds},
     "pieces": [[[x, y], ...], ...]}

Any robot can draw a saved trajectory; pipeline.plan() scales it to the board.
"""

import json
import os
import re
import time
import uuid

_ID = re.compile(r"^[0-9a-f]{12}$")
_meta_cache = {}  # path -> (mtime, meta); listing must not re-parse big files


class TrajectoryNotFound(KeyError):
    pass


def _path(folder, tid):
    if not _ID.match(tid or ""):
        raise TrajectoryNotFound(tid)
    return os.path.join(folder, f"{tid}.json")


def save(folder, *, name, source, mode, pieces, bounds):
    os.makedirs(folder, exist_ok=True)
    tid = uuid.uuid4().hex[:12]
    pieces = [[[round(float(x), 6), round(float(y), 6)] for x, y in p] for p in pieces]
    meta = {"id": tid, "name": name, "source": source, "mode": mode, "created": time.time(),
            "piece_count": len(pieces), "point_count": sum(len(p) for p in pieces), "bounds": bounds}
    path = _path(folder, tid)
    with open(path + ".tmp", "w") as f:
        json.dump({"meta": meta, "pieces": pieces}, f, separators=(",", ":"))
    os.replace(path + ".tmp", path)
    return meta


def load(folder, tid):
    """Returns (meta, pieces)."""
    try:
        with open(_path(folder, tid)) as f:
            data = json.load(f)
    except FileNotFoundError:
        raise TrajectoryNotFound(tid)
    return data["meta"], data["pieces"]


def list_meta(folder):
    """Metadata of every saved trajectory, newest first."""
    if not os.path.isdir(folder):
        return []
    metas = []
    for fname in os.listdir(folder):
        path = os.path.join(folder, fname)
        if not fname.endswith(".json") or not _ID.match(fname[:-5]):
            continue
        mtime = os.path.getmtime(path)
        cached = _meta_cache.get(path)
        if not cached or cached[0] != mtime:
            with open(path) as f:
                cached = _meta_cache[path] = (mtime, json.load(f)["meta"])
        metas.append(cached[1])
    return sorted(metas, key=lambda m: m["created"], reverse=True)


def delete(folder, tid):
    path = _path(folder, tid)
    try:
        os.remove(path)
    except FileNotFoundError:
        raise TrajectoryNotFound(tid)
    _meta_cache.pop(path, None)
