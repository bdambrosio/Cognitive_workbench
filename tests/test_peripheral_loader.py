"""Phase 2.1 loader-plumbing test for peripheral_loader."""
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))

from utils.peripheral_loader import load_peripherals  # noqa: E402


def test_loads_stub_peripheral():
    peripherals_dir = _SRC_DIR / "peripherals"
    peripherals = load_peripherals(str(peripherals_dir))
    assert "_test_loadable" in peripherals, (
        f"Expected _test_loadable peripheral; got {sorted(peripherals.keys())}"
    )
    p = peripherals["_test_loadable"]
    assert p["type"] == "peripheral"
    assert p["python_file"].endswith("peripheral.py")
    assert p["config"] == {}
    assert p["registered_tools"] == []
    assert p["registered_alerts"] == []
    assert p["description"], "description should be non-empty"


def test_missing_dir_returns_empty():
    peripherals = load_peripherals("/nonexistent/path/to/peripherals")
    assert peripherals == {}


if __name__ == "__main__":
    test_loads_stub_peripheral()
    test_missing_dir_returns_empty()
    print("OK")
