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
and press **Load drawing**.

- **Auto order**: nearest-neighbour ordering. Preview the path, play it back at the robot's
  real timing, and download the trajectory `.txt` (the CLI's "saved TXT" mode can replay it).
- **Manual order**: build the draw order yourself.
  - **Place** (`P`): click a piece to draw it next. It starts from the end nearest your click.
  - **Cut** (`C`): click a point on a piece to split it in two there.
  - **Undo** (`Ctrl+Z`), **Reset**, and **Auto-finish rest**, which orders whatever is left
    from the current pen position and undoes as one step.
  - **Generate trajectory** when every piece is placed. The sidebar shows pen-down travel
    between pieces so you can compare against the auto order.

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
