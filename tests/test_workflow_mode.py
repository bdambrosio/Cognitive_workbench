"""workflow_mode suppresses the companion machinery, and says what it touched."""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.workflow import apply_workflow_mode          # noqa: E402


def _loop():
    return SimpleNamespace(discourse_enabled=True, orientation_enabled=True,
                           reflection_enabled=True, attribution_enabled=True,
                           substrate_enabled=True, embodiment_enabled=True)


def test_workflow_mode_switches_off_all_six():
    loop = _loop()
    touched = apply_workflow_mode(loop, {"workflow_mode": True})
    assert not loop.discourse_enabled and not loop.orientation_enabled
    assert not loop.reflection_enabled and not loop.attribution_enabled
    assert not loop.substrate_enabled and not loop.embodiment_enabled
    assert len(touched) == 6 and any("attribution" in t for t in touched)


def test_explicit_scenario_setting_wins_and_off_touches_nothing():
    loop = _loop()
    touched = apply_workflow_mode(
        loop, {"workflow_mode": True, "attribution": {"enabled": True}})
    assert loop.attribution_enabled and not loop.reflection_enabled
    assert len(touched) == 5
    loop = _loop()
    assert apply_workflow_mode(loop, {}) == [] and loop.attribution_enabled
