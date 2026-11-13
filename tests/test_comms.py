import pytest

from comms.CommsBase import CommsBase
from comms.SimulatedComms import SimulatedComms


def test_commsbase_cannot_be_instantiated_directly():
    with pytest.raises(TypeError):
        CommsBase()


def test_commsbase_subclass_missing_abstract_methods_cannot_be_instantiated():
    class Incomplete(CommsBase):
        def _transmit(self, command):
            pass
        # _read_ack intentionally not implemented

    with pytest.raises(TypeError):
        Incomplete()


class FakeTransportComms(CommsBase):
    """In-memory stand-in for a real transport, to test the shared
    send_steps() template method without touching serial/sockets."""

    def __init__(self, acks):
        self.sent = []
        self._acks = iter(acks)

    def _transmit(self, command):
        self.sent.append(command)

    def _read_ack(self):
        return next(self._acks)


def test_send_steps_formats_command_and_waits_for_ack_token():
    comms = FakeTransportComms(acks=["", "not yet", "DONE"])
    comms.send_steps(5, -3, 120)
    assert comms.sent == ["5 -3 120\n"]


def test_prepare_defaults_to_a_noop():
    comms = FakeTransportComms(acks=["DONE"])
    comms.prepare()  # must not raise


def test_simulated_comms_acks_after_the_move_duration():
    import time
    comms = SimulatedComms()
    t = time.monotonic()
    comms.send_steps(5, -3, 50, timeout=1)
    assert time.monotonic() - t >= 0.05
