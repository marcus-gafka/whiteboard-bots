# Whiteboard Bots

Turns DXF drawings into stepper-motor trajectories for three drawing robots
(FiveBar, Spooler, TwoLink).

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Web app

```bash
.venv/bin/uvicorn webapp.server:app --reload
```

Open http://127.0.0.1:8000, then upload a DXF (or pick a sample from `.DXFS/`), choose a robot
and point count, and press **Process**. You can preview the ordered path, play it back
at the robot's real timing, and download the trajectory `.txt`. The CLI's "saved TXT" mode
can replay that file.

## CLI

```bash
.venv/bin/python main.py
```

## Layout

| Path | Purpose |
|---|---|
| `core/pipeline.py` | UI-agnostic pipeline (DXF → fitted, ordered points → trajectory) shared by CLI and web app |
| `paths/` | DXF sampling, board fitting, ordering, manual ordering, file I/O |
| `robots/` | Kinematics for each robot |
| `comms/` | Serial / network / dry-run transports |
| `webapp/` | FastAPI server and static frontend |

## Tests

```bash
.venv/bin/python -m pytest -q
```
