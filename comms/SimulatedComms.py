import time

from comms.CommsBase import CommsBase
from Constants import Constants as c


class SimulatedComms(CommsBase):
    """Stand-in robot for the web app's "Simulated (no robot)" port: acks
    each move after its real duration, through the same protocol path as
    serial, so a run behaves exactly like one on hardware."""

    def __init__(self):
        self._pending_ms = 0

    def _transmit(self, command):
        self._pending_ms = int(command.split()[2])

    def _read_ack(self):
        time.sleep(self._pending_ms / 1000)
        return c.ACK_TOKEN
