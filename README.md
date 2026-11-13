# Whiteboard Bots

Turns DXF drawings into stepper-motor trajectories for three drawing robots
(FiveBar, Spooler, TwoLink). Everything (generating, previewing and sending a
trajectory to the robot) goes through the web app.

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Web app

```bash
.venv/bin/uvicorn webapp.server:app --reload
```

Open http://127.0.0.1:8000. The app has one page per step:

**1 · Trajectory generator** (`/`): upload a DXF (or pick a sample from `.DXFS/`) and press
**Load drawing**. Ordering happens in the DXF's own units, with no robot chosen yet, so one
trajectory can later be drawn by any robot at any size. The pen starts at the drawing's
bottom-left corner (★), which is where every robot's home lands once the drawing is fitted.

Chevrons (›) along each piece show the direction the pen draws it.

- **Auto order**: nearest-neighbour ordering. **Edit this order** copies it into the manual
  editor so you can flip or reorder pieces.
- **Manual order**: build the draw order yourself.
  - **Place** (`P`): click a piece to draw it next. It starts from the end nearest your click.
  - **Cut** (`C`): click a point on a piece to split it in two there.
  - **Flip** (`F`): click a placed piece to reverse the direction it is drawn in.
  - **Undo** (`Ctrl+Z`), **Reset**, and **Auto-finish rest**, which orders whatever is left
    from the current pen position and undoes as one step.
- **Save trajectory** (any time in Auto, once every piece is placed in Manual). Saved
  trajectories are JSON files in `.TRAJECTORIES/` and survive server restarts.

**2 · Robot preview** (`/preview`): pick a saved trajectory, or **Import points…** from a
`.txt` / `.csv` (x, y per line; a blank line starts a new piece; header lines are skipped) or
a trajectory `.json`. Imports are saved as trajectories too. Choose a robot and a point
count: the drawing is scaled to fill that robot's board and turned into
`steps1 steps2 interval` commands. Watch it draw at the robot's real timing (the pen and
linkage are interpolated within each move, so playback is smooth at any speed and the
slider scrubs by time) and download the commands as a tab-separated `.txt`. ✕ deletes the
selected saved trajectory. Nothing is sent to a robot from this page.

**3 · Live** (`/live`): the only place a trajectory is sent to the robot (below). The live
view of the robot's state isn't built yet; it shows "No robot connected".

### Sending to the robot (USB serial)

On the Live page, pick the trajectory, robot and point count, then use **Send to robot**. Pick the port
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

## Layout

| Path | Purpose |
|---|---|
| `core/pipeline.py` | Processing pipeline (DXF → fitted, ordered points → trajectory, linkage poses) |
| `core/trajectory_store.py` | Saved trajectories (`.TRAJECTORIES/*.json`) |
| `core/runner.py` | Background sender that streams a trajectory to the robot |
| `paths/` | DXF sampling, board fitting, automatic and manual ordering |
| `robots/` | Kinematics for each robot |
| `comms/` | Wire protocol, USB serial transport, simulated robot |
| `webapp/` | FastAPI server and static frontend |

## Tests

```bash
.venv/bin/python -m pytest -q
```
