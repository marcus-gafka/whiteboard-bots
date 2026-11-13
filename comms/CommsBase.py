import abc

from Constants import Constants as c


class CommsBase(abc.ABC):
    """Common interface + wire protocol for sending step commands to a plotter.

    Subclasses implement only the transport (_transmit/_read_ack); the
    protocol itself ("steps1 steps2 interval\\n", then block for an ack
    token) is defined once here via the template method send_steps().
    """

    def send_steps(self, steps1: int, steps2: int, interval: int) -> None:
        command = f"{steps1} {steps2} {interval}\n"
        self._transmit(command)
        while self._read_ack() != c.ACK_TOKEN:
            pass

    @abc.abstractmethod
    def _transmit(self, command: str) -> None:
        """Send the raw command string over this backend's transport."""

    @abc.abstractmethod
    def _read_ack(self) -> str:
        """Block for and return the next whitespace-stripped line of input."""

    def prepare(self) -> None:
        """Optional post-connect settling step, run once before any commands
        are sent. Default no-op; backends that need it (e.g. giving a serial
        connection time to reset) override this instead of callers branching
        on which backend is in use."""
        pass

    def close(self) -> None:
        pass
