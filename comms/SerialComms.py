from comms.CommsBase import CommsBase
from Constants import Constants as c
import serial, time

class SerialComms(CommsBase):
    def __init__(self, port=None, baud=None):
        self.ser = serial.Serial(port or c.DEFAULT_SERIAL_PORT, baud or c.DEFAULT_SERIAL_BAUD)
        time.sleep(2)
        self.ser.flush()

    def _transmit(self, command):
        self.ser.write(command.encode())

    def _read_ack(self):
        return self.ser.readline().decode().strip()

    def prepare(self):
        time.sleep(2)
        self.ser.flush()

    def close(self):
        self.ser.close()
