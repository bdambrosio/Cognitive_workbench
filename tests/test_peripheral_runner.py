"""Phase 2.2 lifecycle test for PeripheralRunner."""
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))

from peripheral_runner import PeripheralRunner  # noqa: E402
from utils.peripheral_loader import load_peripherals  # noqa: E402


def _peripherals():
    return load_peripherals(str(_SRC_DIR / "peripherals"))


def test_inert_stub_loads_and_stops_cleanly():
    """A peripheral with no Peripheral class should load, idle, and stop."""
    peripherals = _peripherals()
    meta = peripherals["_test_loadable"]
    runner = PeripheralRunner(meta, character_name="Test")
    runner.start()
    time.sleep(0.1)
    assert runner.is_alive(), "thread should be alive after start"
    runner.stop(timeout=2.0)
    assert not runner.is_alive(), "thread should have stopped after stop()"


def test_lifecycle_peripheral_runs_and_stops():
    """Peripheral.run() should tick until stop_event is set, then exit cleanly."""
    peripherals = _peripherals()
    meta = peripherals["_test_runner_lifecycle"]
    runner = PeripheralRunner(meta, character_name="Test")
    runner.start()
    time.sleep(0.3)  # ~6 ticks at 0.05s period
    runner.stop(timeout=2.0)

    assert not runner.is_alive(), "thread did not stop"
    inst = runner._peripheral_instance
    assert inst is not None, "peripheral was not instantiated"
    assert inst.stop_called, "stop() was not invoked on the peripheral"
    assert inst.stopped_cleanly, "run() did not exit via stop_event path"
    assert inst.ticks > 0, f"expected at least one tick; got {inst.ticks}"


def test_register_tool_stub_mode_does_not_crash():
    """register_tool with no executor wired should log and continue."""
    peripherals = _peripherals()
    meta = peripherals["_test_loadable"]
    runner = PeripheralRunner(meta, character_name="Test")
    runner.register_tool("body.fake", lambda *a, **kw: None)
    # No assertion — success is "did not raise".


def test_publish_sense_event_stub_mode_does_not_crash():
    peripherals = _peripherals()
    meta = peripherals["_test_loadable"]
    runner = PeripheralRunner(meta, character_name="Test")
    runner.publish_sense_event("alert", "body.test", {"x": 1, "y": 2})
    # No assertion — success is "did not raise".


def test_register_tool_with_callback_invokes_it():
    """When a callback is provided, register_tool forwards to it."""
    peripherals = _peripherals()
    meta = peripherals["_test_loadable"]
    received = []
    runner = PeripheralRunner(
        meta,
        character_name="Test",
        register_tool_fn=lambda name, fn: received.append((name, fn)),
    )
    sentinel = lambda: None
    runner.register_tool("body.set_velocity", sentinel)
    assert received == [("body.set_velocity", sentinel)], received


def test_publish_sense_event_with_callback_invokes_it():
    peripherals = _peripherals()
    meta = peripherals["_test_loadable"]
    received = []
    runner = PeripheralRunner(
        meta,
        character_name="Test",
        publish_sense_fn=lambda d, k, p: received.append((d, k, p)),
    )
    runner.publish_sense_event("alert", "body.estop_fired", {"reason": "x"})
    assert received == [("alert", "body.estop_fired", {"reason": "x"})], received


if __name__ == "__main__":
    test_inert_stub_loads_and_stops_cleanly()
    test_lifecycle_peripheral_runs_and_stops()
    test_register_tool_stub_mode_does_not_crash()
    test_publish_sense_event_stub_mode_does_not_crash()
    test_register_tool_with_callback_invokes_it()
    test_publish_sense_event_with_callback_invokes_it()
    print("OK")
