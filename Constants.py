import os

class Constants:
    _BASE = os.path.dirname(os.path.abspath(__file__))
    TXT_FOLDER = os.path.join(_BASE, ".TXTS")
    DXF_FOLDER = os.path.join(_BASE, ".DXFS")

    # Comms wire protocol
    ACK_TOKEN = "DONE"

    # Serial backend defaults
    DEFAULT_SERIAL_PORT = "COM3"
    DEFAULT_SERIAL_BAUD = 9600

    # Network backend defaults
    DEFAULT_NETWORK_IP = "192.168.1.190"
    DEFAULT_NETWORK_PORT = 5000
