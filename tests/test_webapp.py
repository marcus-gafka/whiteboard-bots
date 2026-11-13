import os
import time

import pytest

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from Constants import Constants as c  # noqa: E402
from webapp import server  # noqa: E402

client = TestClient(server.app)


@pytest.fixture(autouse=True)
def trajectory_dir(tmp_path, monkeypatch):
    """Saved trajectories go to a temp folder, never the real one."""
    monkeypatch.setattr(server, "TRAJECTORY_DIR", str(tmp_path))
    return tmp_path


def _drawing(sample="box.dxf"):
    res = client.post("/api/drawings", data={"sample": sample})
    assert res.status_code == 200, res.text
    return res.json()


def _act(drawing_id, **body):
    return client.post(f"/api/drawings/{drawing_id}/manual", json=body)


def _save(drawing_id, mode="auto", name=None):
    res = client.post("/api/trajectories", json={"drawing": drawing_id, "mode": mode, "name": name})
    assert res.status_code == 200, res.text
    return res.json()


def _plan(tid, robot="Spooler", points=None):
    return client.post(f"/api/trajectories/{tid}/plan", json={"robot": robot, "points": points})


def test_pages_served():
    assert "Trajectory Generator" in client.get("/").text
    assert "Robot Preview" in client.get("/preview").text
    live = client.get("/live").text
    assert "No robot connected" in live and "Send to robot" in live
    assert "Send to robot" not in client.get("/preview").text


def test_robots_and_samples_listed():
    names = [r["name"] for r in client.get("/api/robots").json()]
    assert set(names) == {"FiveBar", "Spooler", "TwoLink"}
    assert "box.dxf" in client.get("/api/samples").json()


def test_drawing_is_kept_in_dxf_units():
    d = _drawing("gompei.dxf")
    x0, y0, x1, y1 = d["bounds"]
    assert d["start"] == [x0, y0]
    # gompei.dxf is far larger than any robot board: nothing was rescaled.
    assert max(x1 - x0, y1 - y0) > 20
    pieces = client.post(f"/api/drawings/{d['drawing']}/order", json={"mode": "auto"}).json()["pieces"]
    assert len(pieces) == d["piece_count"]
    # The order begins at the piece end nearest the start corner.
    gap = lambda p: ((p[0] - d["start"][0]) ** 2 + (p[1] - d["start"][1]) ** 2) ** 0.5
    assert gap(pieces[0][0]) <= min(gap(e) for p in pieces for e in (p[0], p[-1])) + 1e-3


def test_uploaded_file_creates_drawing():
    with open(os.path.join(c.DXF_FOLDER, "box.dxf"), "rb") as f:
        res = client.post("/api/drawings", files={"file": ("box.dxf", f, "application/dxf")})
    assert res.status_code == 200, res.text
    assert res.json()["source"] == "box" and res.json()["piece_count"] == 4


def test_create_drawing_rejects_bad_input():
    assert client.post("/api/drawings").status_code == 400
    assert client.post("/api/drawings", data={"sample": "../Constants.py"}).status_code == 400
    res = client.post("/api/drawings", files={"file": ("x.dxf", b"garbage", "application/dxf")})
    assert res.status_code == 400


def test_unknown_ids_are_404():
    assert client.get("/api/drawings/doesnotexist/manual").status_code == 404
    assert _plan("0123456789ab").status_code == 404
    assert _plan("../../etc").status_code == 404
    assert client.delete("/api/trajectories/0123456789ab").status_code == 404


def test_save_once_then_plan_on_every_robot(trajectory_dir):
    d = _drawing("gompei.dxf")
    meta = _save(d["drawing"], name="Gompei auto")
    assert meta["name"] == "Gompei auto" and meta["mode"] == "auto"
    assert meta["point_count"] == d["total_points"]
    assert (trajectory_dir / f"{meta['id']}.json").is_file()
    assert [t["id"] for t in client.get("/api/trajectories").json()] == [meta["id"]]

    for robot in ("FiveBar", "Spooler", "TwoLink"):
        res = _plan(meta["id"], robot, 300)
        assert res.status_code == 200, res.text
        body = res.json()
        assert body["robot"] == robot and body["trajectory"]["id"] == meta["id"]
        assert len(body["path"]) == len(body["rows"]) == 300 and body["unreachable"] == []
        # Fitted onto this robot's board.
        xs, ys = [p[0] for p in body["path"]], [p[1] for p in body["path"]]
        assert min(xs) >= -1e-3 and max(xs) <= body["board"]["width"] + 1e-3
        assert min(ys) >= -1e-3 and max(ys) <= body["board"]["height"] + 1e-3
        # Linkage poses for the preview, one per move.
        assert body["pose_stride"] == 1 and len(body["poses"]) == 300
        assert body["poses"][-1][:2] == pytest.approx(body["path"][-1], abs=1e-2)


def test_default_name_and_delete():
    meta = _save(_drawing()["drawing"])
    assert meta["name"] == "box (auto)"
    assert client.delete(f"/api/trajectories/{meta['id']}").status_code == 200
    assert client.get("/api/trajectories").json() == []


def test_manual_order_cut_place_and_save():
    d = _drawing()
    did = d["drawing"]
    state = client.get(f"/api/drawings/{did}/manual").json()
    assert sorted(state["pool"]) == [0, 1, 2, 3] and state["pen"] == d["start"]

    # Can't save a manual order until everything is placed.
    res = client.post("/api/trajectories", json={"drawing": did, "mode": "manual"})
    assert res.status_code == 400

    # Cut piece 0 at its midpoint -> two new pieces, sent back to the client.
    pts = state["pieces"]["0"]
    mid = pts[len(pts) // 2]
    state = _act(did, action="cut", piece=0, point=mid).json()
    assert 0 not in state["pool"] and len(state["pool"]) == 5
    assert set(state["pieces"]) == {"4", "5"}

    state = _act(did, action="place", piece=5, reverse=True).json()
    assert state["placed"] == [[5, True]]
    # Reversed second half ends where the first half was cut.
    assert state["pen"] == pytest.approx(pts[len(pts) // 2 + 1], abs=1e-3)
    assert _act(did, action="place", piece=5).status_code == 400  # already placed

    # Flip reverses a placed piece's direction (and is undoable).
    state = _act(did, action="flip", piece=5).json()
    assert state["placed"] == [[5, False]]
    assert _act(did, action="flip", piece=1).status_code == 400   # not placed yet
    state = _act(did, action="undo").json()
    assert state["placed"] == [[5, True]]

    state = _act(did, action="autofinish").json()
    assert state["complete"] and state["pool"] == []
    state = _act(did, action="undo").json()          # auto-finish undoes as one step
    assert state["placed"] == [[5, True]]
    _act(did, action="autofinish")

    meta = _save(did, mode="manual")
    assert meta["mode"] == "manual" and meta["piece_count"] == 5
    body = _plan(meta["id"], "TwoLink", 200).json()
    assert len(body["pieces"]) == 5 and len(body["rows"]) == 200

    state = _act(did, action="reset").json()
    assert sorted(state["pool"]) == [0, 1, 2, 3] and state["placed"] == []


def test_manual_errors_are_400():
    did = _drawing()["drawing"]
    assert _act(did, action="undo").status_code == 400          # nothing to undo
    assert _act(did, action="place", piece=99).status_code == 400
    assert _act(did, action="cut", piece=0).status_code == 422  # missing point


def test_import_points_then_plan():
    text = "x,y\n0,0\n10,0\n10,5\n\n2,2\n4,3\n"
    res = client.post("/api/trajectories/import", files={"file": ("square.csv", text, "text/csv")})
    assert res.status_code == 200, res.text
    meta = res.json()
    assert meta["name"] == "square (imported)" and meta["mode"] == "imported"
    assert meta["piece_count"] == 2 and meta["point_count"] == 5
    body = _plan(meta["id"], "FiveBar").json()
    assert len(body["rows"]) == 5 and body["unreachable"] == []

    res = client.post("/api/trajectories/import", files={"file": ("x.txt", "nothing here", "text/plain")})
    assert res.status_code == 400


def test_plan_rejects_unknown_robot():
    meta = _save(_drawing()["drawing"])
    assert _plan(meta["id"], "Nope").status_code == 400


def _wait_run(states, timeout=10):
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = client.get("/api/run").json()
        if s["state"] in states:
            return s
        time.sleep(0.05)
    raise AssertionError(client.get("/api/run").json())


def test_ports_always_include_simulated():
    assert {"device": "simulated", "description": "Simulated (no robot)"} in client.get("/api/ports").json()


def test_simulated_run_stop_then_home_gate():
    server.RUNNER.mark_home()
    tid = _save(_drawing()["drawing"])["id"]
    run = {"trajectory": tid, "robot": "TwoLink", "points": 40, "port": "simulated"}

    res = client.post("/api/run", json=run)
    assert res.status_code == 200, res.text
    s = res.json()
    assert s["total"] == 40 and s["meta"]["trajectory"] == tid and s["meta"]["robot"] == "TwoLink"
    _wait_run({"running"})
    # A second start while running is refused.
    assert client.post("/api/run", json=run).status_code == 409

    client.post("/api/run/stop")
    s = _wait_run({"stopped", "done"})
    if s["state"] == "stopped":
        assert not s["at_home"]
        # Can't start a new drawing until the pen is home.
        assert client.post("/api/run", json=run).status_code == 409
        assert client.post("/api/run/home", json={"port": "simulated"}).status_code == 200
        s = _wait_run({"done"})
        assert s["at_home"] and s["meta"]["kind"] == "home"
    assert client.post("/api/run/home", json={"port": "simulated"}).status_code == 400


def test_run_unknown_trajectory_is_404():
    server.RUNNER.mark_home()
    res = client.post("/api/run", json={"trajectory": "0123456789ab", "robot": "TwoLink", "port": "simulated"})
    assert res.status_code == 404
