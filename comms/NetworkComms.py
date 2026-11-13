from comms.CommsBase import CommsBase
from Constants import Constants as c
import socket, time

class NetworkComms(CommsBase):
    def __init__(self, ip=None, port=None):
        self.ip = ip or c.DEFAULT_NETWORK_IP
        self.port = port or c.DEFAULT_NETWORK_PORT
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))
        self.sock.settimeout(5)

    def _transmit(self, command):
        self.sock.sendall(command.encode())

    def _read_ack(self):
        try:
            return self.sock.recv(1024).decode().strip()
        except socket.timeout:
            return ""

    def prepare(self):
        time.sleep(0.5)

    def close(self):
        self.sock.close()
