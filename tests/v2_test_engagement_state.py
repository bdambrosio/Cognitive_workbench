"""Which intake and run are current: explicit if set and not cancelled, else
most recent not cancelled; a new intake clears the explicit choice; marks
never delete. Added 2026-09-03 with workflowsv2/engagement_state.py."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2 import engagement_state as st                  # noqa: E402


def _run(eng, name, intake):
    d = eng / "merged" / name
    d.mkdir(parents=True)
    (d / "meta.json").write_text(json.dumps({"intake": intake}))
    return d


def test_intakes_current_by_recency_explicit_and_cancel(tmp_path):
    eng = tmp_path / "e"; eng.mkdir()
    assert st.intakes(eng) == [] and st.current_intake(eng) is None
    a = st.new_intake(eng)
    (eng / "intakes" / "2027-01-01T00-00-00Z").mkdir()
    assert st.current_intake(eng) == "2027-01-01T00-00-00Z"       # most recent
    st.set_current_intake(eng, a)
    assert st.current_intake(eng) == a                             # explicit
    st.cancel_intake(eng, a)
    assert st.current_intake(eng) == "2027-01-01T00-00-00Z"       # explicit cancelled → recency
    assert (eng / "intakes" / a).is_dir()                          # a mark, not a deletion
    (eng / "intakes" / a / st.BLOCKS_FILE).write_text("thresholds: |\n  walks\n")
    assert st.intake_blocks(eng, a)["thresholds"].strip() == "walks"
    assert st.intake_blocks(eng, "2027-01-01T00-00-00Z") == {}


def test_runs_pin_an_intake_and_current_follows_the_same_rule(tmp_path):
    eng = tmp_path / "e"; eng.mkdir()
    _run(eng, "2026-01-01T00-00-00Z_a", "I1")
    r2 = _run(eng, "2026-01-02T00-00-00Z_b", "I1")
    _run(eng, "2026-01-03T00-00-00Z_c", None)          # pre-pin / no-intake run
    assert [r.name for r in st.runs_for(eng, "I1")] == [
        "2026-01-01T00-00-00Z_a", "2026-01-02T00-00-00Z_b"]
    assert st.current_run(eng, "I1").name == r2.name
    assert st.current_run(eng, None).name == "2026-01-03T00-00-00Z_c"
    assert st.current_run(eng, "I9") is None
    st.set_current_run(eng, "2026-01-01T00-00-00Z_a")
    assert st.current_run(eng, "I1").name == "2026-01-01T00-00-00Z_a"
    st.cancel_run(eng, "2026-01-01T00-00-00Z_a")
    assert st.current_run(eng, "I1").name == r2.name
    text = st.status(eng)
    assert "CURRENT" in text and "cancelled" in text
    state = json.loads((eng / "state.json").read_text())
    assert state["cancelled"]["runs"] == ["2026-01-01T00-00-00Z_a"]
