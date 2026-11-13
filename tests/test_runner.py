import os
import sys
import threading
import time

import pytest

from comms.CommsBase import CommsBase, CommsError
from core.runner import RobotRunner


class FakeComms(CommsBase):
    """Acks every command after `delay` seconds; can be told to go silent."""

    def __init__(self, delay=0.0, silent_after=None):
        self.sent, self.delay, self.silent_after = [], delay, silent_after
        self.closed = False

    def _transmit(self, command):
        self.sent.append(command.strip())

    def _read_ack(self):
        time.sleep(self.delay)
        if self.silent_after is not None and len(self.sent) > self.silent_after:
            time.sleep(0.01)
            return ""
        return "DONE"

    def close(self):
        self.closed = True


def _wait_for(runner, states, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if runner.status()["state"] in states:
            return runner.status()
        time.sleep(0.01)
    raise AssertionError(f"runner stuck in {runner.status()}")


CMDS = [(10, -5, 0), (3, 3, 0), (-1, 0, 0)]
ANGLES = [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]


def test_runs_all_commands_and_records_position():
    comms = FakeComms()
    r = RobotRunner()
    r.start(CMDS, ANGLES, lambda: comms, "TwoLink", {"job": "x"})
    r.wait(5)
    s = r.status()
    assert s["state"] == "done" and s["index"] == 3 and s["meta"] == {"job": "x"}
    assert comms.sent == ["10 -5 0", "3 3 0", "-1 0 0"] and comms.closed
    assert r.position() == ("TwoLink", 5.0, 6.0)
    assert s["at_home"] is False


def test_home_run_clears_position():
    r = RobotRunner()
    r.start([(1, 1, 0)], [(0.0, 0.0)], lambda: FakeComms(), "TwoLink", ends_at_home=True)
    r.wait(5)
    assert r.status()["state"] == "done" and r.position() is None


def test_pause_resume_and_stop():
    comms = FakeComms(delay=0.05)
    r = RobotRunner()
    r.start([(1, 1, 0)] * 40, [(0.0, float(i)) for i in range(40)], lambda: comms, "TwoLink")
    _wait_for(r, {"running"})
    r.pause()
    time.sleep(0.15)
    paused_at = r.status()["index"]
    time.sleep(0.15)
    assert r.status()["state"] == "paused" and r.status()["index"] == paused_at

    r.resume()
    time.sleep(0.12)
    assert r.status()["index"] > paused_at

    r.stop()
    r.wait(5)
    s = r.status()
    assert s["state"] == "stopped" and 0 < s["index"] < 40
    assert r.position() == ("TwoLink", 0.0, float(s["index"] - 1))


def test_only_one_run_at_a_time_and_mark_home():
    r = RobotRunner()
    r.start([(1, 1, 0)] * 20, [(0.0, 0.0)] * 20, lambda: FakeComms(delay=0.05), "TwoLink")
    with pytest.raises(Exception):
        r.start(CMDS, ANGLES, lambda: FakeComms(), "TwoLink")
    with pytest.raises(Exception):
        r.mark_home()
    r.stop()
    r.wait(5)
    r.mark_home()
    assert r.status()["at_home"]


def test_unresponsive_robot_times_out_with_error():
    r = RobotRunner()
    r.ACK_MARGIN_S = 0.2
    r.start(CMDS, ANGLES, lambda: FakeComms(silent_after=1), "TwoLink")
    r.wait(5)
    s = r.status()
    assert s["state"] == "error" and "No 'DONE'" in s["error"]
    assert s["index"] == 1 and r.position() == ("TwoLink", 1.0, 2.0)


def test_connection_failure_is_reported():
    def boom():
        raise CommsError("/dev/ttyUSB9 not found. Is the robot plugged in?")

    r = RobotRunner()
    r.start(CMDS, ANGLES, boom, "TwoLink")
    r.wait(5)
    s = r.status()
    assert s["state"] == "error" and "not found" in s["error"] and r.status()["at_home"]


# ---------------------------------------------------------------------------
# Real SerialComms over a pseudo-terminal acting as the robot firmware.
# ---------------------------------------------------------------------------

@pytest.mark.skipif(sys.platform == "win32", reason="needs POSIX pty")
def test_serial_comms_against_fake_firmware():
    from comms.SerialComms import SerialComms

    master, slave = os.openpty()
    received = []

    def firmware():
        buf = b""
        while len(received) < 3:
            buf += os.read(master, 1024)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                received.append(line.decode())
                os.write(master, b"DONE\r\n")

    t = threading.Thread(target=firmware, daemon=True)
    t.start()
    try:
        r = RobotRunner()
        r.start(CMDS, ANGLES, lambda: SerialComms(os.ttyname(slave), 9600, reset_delay=0), "TwoLink")
        r.wait(5)
        assert r.status()["state"] == "done", r.status()
        assert received == ["10 -5 0", "3 3 0", "-1 0 0"]
    finally:
        os.close(master)
        os.close(slave)


def test_serial_comms_missing_port_message():
    from comms.SerialComms import SerialComms

    with pytest.raises(CommsError, match="not found|Could not open"):
        SerialComms("/dev/does-not-exist-123", 9600)
