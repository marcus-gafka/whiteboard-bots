class CommsBase:
    def send_steps(self, steps1, steps2, interval):
        raise NotImplementedError

    def close(self):
        pass
