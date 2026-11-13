import abc
import time

from Constants import Constants as c


class CommsError(Exception):
    """Transport-level failure talking to the robot (message is user-facing)."""


class CommsTimeout(CommsError):
    """The robot didn't acknowledge a command in time."""


class CommsBase(abc.ABC):
    """Wire protocol for sending step commands to a plotter: one
    "steps1 steps2 interval\\n" line per move, then block until the robot
    replies with the ack token. Subclasses implement only the transport."""

    def send_steps(self, steps1: int, steps2: int, interval: int, timeout: float | None = None) -> None:
        """Send one move and block until the robot acks it.

        timeout: seconds to wait for the ack before raising CommsTimeout
        (None = wait forever). _read_ack() must return periodically (e.g. ""
        on a read timeout) for the deadline to be checked.
        """
        command = f"{steps1} {steps2} {interval}\n"
        self._transmit(command)
        deadline = None if timeout is None else time.monotonic() + timeout
        while self._read_ack() != c.ACK_TOKEN:
            if deadline is not None and time.monotonic() > deadline:
                raise CommsTimeout(f"No '{c.ACK_TOKEN}' from the robot within {timeout:.1f}s of sending '{command.strip()}'.")

    @abc.abstractmethod
    def _transmit(self, command: str) -> None:
        """Send the raw command string over this backend's transport."""

    @abc.abstractmethod
    def _read_ack(self) -> str:
        """Block for and return the next whitespace-stripped line of input."""

    def prepare(self) -> None:
        """Run once after connecting, before any commands are sent."""

    def close(self) -> None:
        pass
