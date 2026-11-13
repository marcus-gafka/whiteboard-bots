import time

import serial

from comms.CommsBase import CommsBase, CommsError


class SerialComms(CommsBase):
    # Opening the port toggles DTR, which resets most Arduinos; give the
    # bootloader time to hand over before the first command.
    RESET_DELAY_S = 2.0

    def __init__(self, port, baud, read_timeout=0.5, reset_delay=RESET_DELAY_S):
        self.port = port
        self.reset_delay = reset_delay
        try:
            # read_timeout makes readline() return periodically so callers'
            # ack deadlines are honoured instead of blocking forever.
            self.ser = serial.Serial(port, baud, timeout=read_timeout)
        except serial.SerialException as e:
            raise CommsError(self._explain(e)) from e

    def _explain(self, e):
        msg = str(e)
        if "Permission denied" in msg or getattr(e, "errno", None) == 13:
            return (f"Permission denied opening {self.port}. On Linux, add your user to the "
                    f"'dialout' group (sudo usermod -aG dialout $USER) and log out and back in.")
        if "No such file" in msg or getattr(e, "errno", None) == 2:
            return f"{self.port} not found. Is the robot plugged in?"
        return f"Could not open {self.port}: {msg}"

    def _transmit(self, command):
        try:
            self.ser.write(command.encode())
        except serial.SerialException as e:
            raise CommsError(f"Lost connection to {self.port}: {e}") from e

    def _read_ack(self):
        try:
            return self.ser.readline().decode(errors="replace").strip()
        except serial.SerialException as e:
            raise CommsError(f"Lost connection to {self.port}: {e}") from e

    def prepare(self):
        time.sleep(self.reset_delay)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()
