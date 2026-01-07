from comms.CommsBase import CommsBase
import time

class PsuedoComms(CommsBase):
    def __init__(self, simulate_delay=False, verbose=True):
        """
        simulate_delay: if True, sleep for 'interval' to mimic motion time
        verbose: if True, print commands to terminal
        """
        self.simulate_delay = simulate_delay
        self.verbose = verbose
        self.command_count = 0

        #if self.verbose:
            #print("[PsuedoComms] Initialized (no hardware, no network)")

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

    def close(self):
        if self.verbose:
            print(f"[PsuedoComms] Closed after {self.command_count} commands")
