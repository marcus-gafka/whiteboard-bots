from comms.CommsBase import CommsBase
import serial, time

class SerialComms(CommsBase):
    def __init__(self, port="COM3", baud=9600):
        self.ser = serial.Serial(port, baud)
        time.sleep(2)
        self.ser.flush()

    def send_steps(self, steps1, steps2, interval):
        cmd = f"{steps1} {steps2} {interval}\n"
        self.ser.write(cmd.encode())
        while True:
            if self.ser.readline().decode().strip() == "DONE":
                break

    def close(self):
        self.ser.close()
