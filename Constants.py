import os


_BASE = os.path.dirname(os.path.abspath(__file__))


class Constants:
    # Bundled sample drawings offered in the web app.
    DXF_FOLDER = os.path.join(_BASE, ".DXFS")
    # Trajectories saved by the generator page (robot-independent, DXF units).
    TRAJECTORY_FOLDER = os.path.join(_BASE, ".TRAJECTORIES")

    # Wire protocol: the firmware replies with this line after each move.
    ACK_TOKEN = "DONE"
    DEFAULT_SERIAL_BAUD = 9600
