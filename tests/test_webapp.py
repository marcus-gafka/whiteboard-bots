import os

import pytest

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from Constants import Constants as c  # noqa: E402
from webapp.server import app  # noqa: E402

client = TestClient(app)


def test_robots_and_samples_listed():
    names = [r["name"] for r in client.get("/api/robots").json()]
    assert set(names) == {"FiveBar", "Spooler", "TwoLink"}
    assert "box.dxf" in client.get("/api/samples").json()


def test_process_sample_returns_trajectory():
    res = client.post("/api/process", data={"robot": "Spooler", "sample": "gompei.dxf", "points": "300"})
    assert res.status_code == 200, res.text
    body = res.json()
    assert len(body["path"]) == 300
    assert len(body["rows"]) == 300
    assert body["unreachable"] == []


def test_process_uploaded_file():
    with open(os.path.join(c.DXF_FOLDER, "box.dxf"), "rb") as f:
        res = client.post("/api/process", data={"robot": "TwoLink"},
                          files={"file": ("box.dxf", f, "application/dxf")})
    assert res.status_code == 200, res.text
    assert res.json()["source"] == "box"


def test_process_rejects_bad_input():
    assert client.post("/api/process", data={"robot": "Spooler"}).status_code == 400
    assert client.post("/api/process", data={"robot": "Nope", "sample": "box.dxf"}).status_code == 400
    res = client.post("/api/process", data={"robot": "Spooler", "sample": "../Constants.py"})
    assert res.status_code == 400
    res = client.post("/api/process", data={"robot": "Spooler"},
                      files={"file": ("x.dxf", b"garbage", "application/dxf")})
    assert res.status_code == 400


def test_index_served():
    assert "Whiteboard Bots" in client.get("/").text
