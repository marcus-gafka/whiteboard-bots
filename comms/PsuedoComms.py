from comms.CommsBase import CommsBase
import time

class PsuedoComms(CommsBase):
    """No-hardware simulation stub: logs commands and (optionally) sleeps to
    mimic motion time, but never actually waits on an external ack, so it
    overrides send_steps() directly rather than going through CommsBase's
    real-protocol template method. _transmit/_read_ack are still defined
    (trivially) since CommsBase declares them abstract.
    """

    def __init__(self, simulate_delay=False, verbose=True):
        """
        simulate_delay: if True, sleep for 'interval' to mimic motion time
        verbose: if True, print commands to terminal
        """
        self.simulate_delay = simulate_delay
        self.verbose = verbose
        self.command_count = 0

    def send_steps(self, steps1, steps2, interval):
        self.command_count += 1

        if self.verbose:
            print(
                f"[PsuedoComms] CMD {self.command_count}: "
                f"steps1={steps1}, steps2={steps2}, interval={interval}"
            )

        # Simulate the robot taking time to execute the move
        if self.simulate_delay and interval > 0:
            time.sleep(interval / 1000)

    def _transmit(self, command):
        pass

    def _read_ack(self):
        return ""

    def close(self):
        if self.verbose:
            print(f"[PsuedoComms] Closed after {self.command_count} commands")
