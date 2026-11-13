"""Background sender: streams a trajectory's step commands to the robot.

Independent of any web framework so it can be tested with fake comms. One
runner drives one physical robot; only one run can be active at a time.

Position tracking: every trajectory assumes the pen starts at the robot's
home (START_XY). The runner records the joint angles reached after each
acknowledged move, so after a stop or error the caller can tell the robot is
*not* home and either send it home or have the user confirm they moved it.
"""

import threading
import time

from comms.CommsBase import CommsError, CommsTimeout

ACTIVE_STATES = {"connecting", "running", "paused", "stopping"}


class RunnerBusy(Exception):
    pass


class RobotRunner:
    # Seconds of slack on top of each move's own duration before declaring
    # the robot unresponsive.
    ACK_MARGIN_S = 5.0

    def __init__(self):
        self._lock = threading.Lock()
        self._thread = None
        self._pause = threading.Event()
        self._stop = threading.Event()
        self._status = {"state": "idle", "index": 0, "total": 0, "error": None, "meta": {}}
        # (robot_name, theta1, theta2) after the last acknowledged move, or
        # None when the pen is (assumed to be) at home.
        self._position = None

    # ------------------------------------------------------------- queries
    def status(self):
        with self._lock:
            s = dict(self._status)
            s["meta"] = dict(s["meta"])
            s["at_home"] = self._position is None
            s["position_robot"] = self._position[0] if self._position else None
            return s

    @property
    def active(self):
        with self._lock:
            return self._status["state"] in ACTIVE_STATES

    def position(self):
        with self._lock:
            return self._position

    # ------------------------------------------------------------- control
    def start(self, commands, angles, open_comms, robot_name, meta=None, ends_at_home=False):
        """commands: [(steps1, steps2, interval_ms)]; angles: [(t1, t2)] after
        each command. open_comms: zero-arg callable returning a CommsBase.
        ends_at_home: the final command returns the pen home (position is
        cleared on success instead of recorded)."""
        if len(commands) != len(angles):
            raise ValueError("commands and angles must be the same length")
        with self._lock:
            if self._status["state"] in ACTIVE_STATES:
                raise RunnerBusy("The robot is already running.")
            self._pause.clear()
            self._stop.clear()
            self._status = {"state": "connecting", "index": 0, "total": len(commands),
                            "error": None, "meta": dict(meta or {})}
            self._thread = threading.Thread(
                target=self._run, args=(list(commands), list(angles), open_comms, robot_name, ends_at_home),
                name="robot-runner", daemon=True)
            self._thread.start()

    def pause(self):
        with self._lock:
            if self._status["state"] == "running":
                self._pause.set()
                self._status["state"] = "paused"

    def resume(self):
        with self._lock:
            if self._status["state"] == "paused":
                self._pause.clear()
                self._status["state"] = "running"

    def stop(self):
        """Stops after the move in progress finishes (the protocol has no
        mid-move abort)."""
        with self._lock:
            if self._status["state"] in ACTIVE_STATES:
                self._stop.set()
                self._pause.clear()
                self._status["state"] = "stopping"

    def mark_home(self):
        """User confirms the pen is physically at home."""
        with self._lock:
            if self._status["state"] in ACTIVE_STATES:
                raise RunnerBusy("Can't change position while the robot is running.")
            self._position = None

    def wait(self, timeout=None):
        t = self._thread
        if t:
            t.join(timeout)

    # ------------------------------------------------------------- worker
    def _set(self, **kw):
        with self._lock:
            self._status.update(kw)

    def _run(self, commands, angles, open_comms, robot_name, ends_at_home):
        comms = None
        try:
            comms = open_comms()
            comms.prepare()
            if self._stop.is_set():
                self._set(state="stopped")
                return
            self._set(state="running")

            for i, (s1, s2, interval) in enumerate(commands):
                while self._pause.is_set() and not self._stop.is_set():
                    time.sleep(0.05)
                if self._stop.is_set():
                    self._set(state="stopped")
                    return
                comms.send_steps(s1, s2, interval, timeout=interval / 1000 + self.ACK_MARGIN_S)
                with self._lock:
                    self._status["index"] = i + 1
                    last = i == len(commands) - 1
                    self._position = None if (ends_at_home and last) else (robot_name, *angles[i])

            self._set(state="done")
        except CommsTimeout as e:
            self._set(state="error", error=f"{e} Check the robot, then return it home or confirm its position.")
        except CommsError as e:
            self._set(state="error", error=str(e))
        except Exception as e:  # unexpected: surface rather than leave the UI hanging
            self._set(state="error", error=f"{type(e).__name__}: {e}")
        finally:
            if comms is not None:
                try:
                    comms.close()
                except Exception:
                    pass
