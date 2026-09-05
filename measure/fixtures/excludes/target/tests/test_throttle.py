from hopper.middleware.throttle import Throttle


class Req:
    remote_addr = "10.0.0.1"


def test_zero_disables():
    assert Throttle(0).check(Req()) is None
