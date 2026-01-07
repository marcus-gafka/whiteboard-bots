from comms.CommsBase import CommsBase
import socket, time

class NetworkComms(CommsBase):
    NETWORK_IP = "192.168.1.190"
    NETWORK_PORT = 5000

    def __init__(self, ip=None, port=None):
        self.ip = ip or self.NETWORK_IP
        self.port = port or self.NETWORK_PORT
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))
        self.sock.settimeout(5)

    def send_steps(self, steps1, steps2, interval):
        cmd = f"{steps1} {steps2} {interval}\n"
        self.sock.sendall(cmd.encode())
        while True:
            try:
                data = self.sock.recv(1024).decode().strip()
            except socket.timeout:
                continue
            if data == "DONE":
                break

    def close(self):
        self.sock.close()
