import os

import pytest

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from Constants import Constants as c  # noqa: E402
from webapp.server import app  # noqa: E402

client = TestClient(app)


def _job(robot="Spooler", sample="box.dxf"):
    res = client.post("/api/jobs", data={"robot": robot, "sample": sample})
    assert res.status_code == 200, res.text
    return res.json()


def _act(job_id, **body):
    return client.post(f"/api/jobs/{job_id}/manual", json=body)


def test_robots_and_samples_listed():
    names = [r["name"] for r in client.get("/api/robots").json()]
    assert set(names) == {"FiveBar", "Spooler", "TwoLink"}
    assert "box.dxf" in client.get("/api/samples").json()


def test_auto_trajectory_from_sample():
    job = _job(sample="gompei.dxf")
    res = client.post(f"/api/jobs/{job['job']}/trajectory", json={"mode": "auto", "points": 300})
    assert res.status_code == 200, res.text
    body = res.json()
    assert len(body["path"]) == len(body["rows"]) == 300
    assert body["unreachable"] == []
    # Robot linkage for the simulation: static geometry plus one pose per move.
    assert body["board"]["robot"]["home"] and body["board"]["robot"]["bases"]
    assert body["pose_stride"] == 1 and len(body["poses"]) == 300
    assert body["poses"][-1][:2] == pytest.approx(body["path"][-1], abs=1e-2)


def test_uploaded_file_creates_job():
    with open(os.path.join(c.DXF_FOLDER, "box.dxf"), "rb") as f:
        res = client.post("/api/jobs", data={"robot": "TwoLink"},
                          files={"file": ("box.dxf", f, "application/dxf")})
    assert res.status_code == 200, res.text
    assert res.json()["source"] == "box"
    assert res.json()["piece_count"] == 4


def test_create_job_rejects_bad_input():
    assert client.post("/api/jobs", data={"robot": "Spooler"}).status_code == 400
    assert client.post("/api/jobs", data={"robot": "Nope", "sample": "box.dxf"}).status_code == 400
    assert client.post("/api/jobs", data={"robot": "Spooler", "sample": "../Constants.py"}).status_code == 400
    res = client.post("/api/jobs", data={"robot": "Spooler"},
                      files={"file": ("x.dxf", b"garbage", "application/dxf")})
    assert res.status_code == 400


def test_unknown_job_is_404():
    assert client.get("/api/jobs/doesnotexist/manual").status_code == 404


def test_manual_place_cut_undo_and_generate():
    job = _job()
    jid = job["job"]
    state = client.get(f"/api/jobs/{jid}/manual").json()
    assert sorted(state["pool"]) == [0, 1, 2, 3]
    assert set(state["pieces"]) == {"0", "1", "2", "3"}

    # Manual trajectory refused until everything is placed.
    assert client.post(f"/api/jobs/{jid}/trajectory", json={"mode": "manual"}).status_code == 400

    # Cut piece 0 at its midpoint -> two new pieces, sent back to the client.
    pts = state["pieces"]["0"]
    mid = pts[len(pts) // 2]
    state = _act(jid, action="cut", piece=0, point=mid).json()
    assert 0 not in state["pool"] and len(state["pool"]) == 5
    assert set(state["pieces"]) == {"4", "5"}

    state = _act(jid, action="place", piece=5, reverse=True).json()
    assert state["placed"] == [[5, True]]
    # Reversed second half ends where the first half was cut.
    assert state["pen"] == pytest.approx(pts[len(pts) // 2 + 1], abs=1e-3)
    # The linkage pose follows the pen.
    assert state["pose"][:2] == pytest.approx(state["pen"], abs=1e-2)

    assert _act(jid, action="place", piece=5).status_code == 400  # already placed

    state = _act(jid, action="autofinish").json()
    assert state["complete"] and state["pool"] == []

    # Auto-finish undoes as one step.
    state = _act(jid, action="undo").json()
    assert state["placed"] == [[5, True]]

    state = _act(jid, action="autofinish").json()
    res = client.post(f"/api/jobs/{jid}/trajectory", json={"mode": "manual", "points": 200})
    assert res.status_code == 200, res.text
    body = res.json()
    assert body["mode"] == "manual" and len(body["pieces"]) == 5 and len(body["rows"]) == 200

    state = _act(jid, action="reset").json()
    assert sorted(state["pool"]) == [0, 1, 2, 3] and state["placed"] == []


def test_manual_errors_are_400():
    jid = _job()["job"]
    assert _act(jid, action="undo").status_code == 400          # nothing to undo
    assert _act(jid, action="place", piece=99).status_code == 400
    assert _act(jid, action="cut", piece=0).status_code == 422  # missing point


def test_index_served():
    assert "Whiteboard Bots" in client.get("/").text


def _wait_run(states, timeout=10):
    import time
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
    from webapp import server
    server.RUNNER.mark_home()
    jid = _job(robot="TwoLink", sample="box.dxf")["job"]

    res = client.post("/api/run", json={"job": jid, "mode": "auto", "points": 40, "port": "simulated"})
    assert res.status_code == 200, res.text
    assert res.json()["total"] == 40
    _wait_run({"running"})
    # A second start while running is refused.
    assert client.post("/api/run", json={"job": jid, "port": "simulated"}).status_code == 409

    client.post("/api/run/stop")
    s = _wait_run({"stopped", "done"})
    if s["state"] == "stopped":
        assert not s["at_home"]
        # Can't start a new drawing until the pen is home.
        assert client.post("/api/run", json={"job": jid, "port": "simulated"}).status_code == 409
        assert client.post("/api/run/home", json={"port": "simulated"}).status_code == 200
        s = _wait_run({"done"})
        assert s["at_home"] and s["meta"]["kind"] == "home"
    assert client.post("/api/run/home", json={"port": "simulated"}).status_code == 400


def test_manual_run_requires_complete_order():
    from webapp import server
    server.RUNNER.mark_home()
    jid = _job()["job"]
    client.get(f"/api/jobs/{jid}/manual")
    res = client.post("/api/run", json={"job": jid, "mode": "manual", "port": "simulated"})
    assert res.status_code == 400
