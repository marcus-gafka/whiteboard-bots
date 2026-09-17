import os

class Constants:
    _BASE = os.path.dirname(os.path.abspath(__file__))
    TXT_FOLDER = os.path.join(_BASE, ".TXTS")
    DXF_FOLDER = os.path.join(_BASE, ".DXFS")
