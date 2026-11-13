import os


class Constants:
    # Bundled sample drawings offered in the web app.
    DXF_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".DXFS")

    # Wire protocol: the firmware replies with this line after each move.
    ACK_TOKEN = "DONE"
    DEFAULT_SERIAL_BAUD = 9600
