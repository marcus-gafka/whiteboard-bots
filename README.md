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

### Sending to the robot (USB serial)

Once a trajectory preview is showing, use **Send to robot** in the sidebar. Pick the port
(only USB serial devices are listed; ↻ rescans) and baud rate (default 9600), then
**Start drawing**. Pause and Stop take effect after the move in progress.

- The protocol is unchanged: one `steps1 steps2 interval` line per move, and the firmware
  replies `DONE`. If no `DONE` arrives within the move's duration plus 5 s, the run stops
  with an error.
- Every drawing assumes the pen starts at home (★). After a run ends or is stopped, the app
  blocks the next one until you **Return pen home** (moves straight back, which draws a line
  because the pen can't lift) or confirm **It's at home**.
- **Simulated (no robot)** runs the same flow in real time without hardware.
- **Linux:** your user needs serial access:
  `sudo usermod -aG dialout $USER`, then log out and back in.

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
