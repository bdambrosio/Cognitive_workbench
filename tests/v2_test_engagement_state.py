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


def test_new_engagement_writes_a_stub_and_refuses_twice(tmp_path):
    eng = st.new_engagement(tmp_path / "e")
    text = (eng / "engagement.yaml").read_text()
    assert "target: target" in text and "claim_sources: []" in text
    import pytest
    with pytest.raises(SystemExit):
        st.new_engagement(tmp_path / "e")


def test_new_engagement_clones_a_local_checkout(tmp_path):
    import shutil, subprocess
    if not shutil.which("git"):
        return
    src = tmp_path / "src"; src.mkdir()
    subprocess.run(["git", "-C", str(src), "init", "-q"], check=True)
    (src / "README.md").write_text("claims\n")
    subprocess.run(["git", "-C", str(src), "add", "-A"], check=True)
    subprocess.run(["git", "-C", str(src), "-c", "user.email=t@t", "-c", "user.name=t",
                    "commit", "-q", "-m", "x"], check=True)
    eng = st.new_engagement(tmp_path / "e", clone=str(src))
    assert (eng / "target" / "README.md").read_text() == "claims\n"
    assert (eng / "target" / ".git").exists()


def test_stages_are_marks_with_who_and_when(tmp_path):
    eng = st.new_engagement(tmp_path / "e", client_emails=["A@x.test", " b@x.test "])
    assert st.client_emails(eng) == ["a@x.test", "b@x.test"]
    assert st.claim_sources(eng) == []
    assert st.stage_value(eng, "created") == "done"
    assert st.stage(eng, "letter") is None
    st.set_stage(eng, "letter", "accepted", by="a@x.test")
    m = st.stage(eng, "letter")
    assert m["value"] == "accepted" and m["by"] == "a@x.test" and m["at"].endswith("Z")
    import pytest
    with pytest.raises(SystemExit):
        st.set_stage(eng, "billing", "done")
    assert st.summary(eng)["stages"]["letter"]["value"] == "accepted"
    assert "letter  accepted" in st.status(eng)


def test_jobs_lock_and_finish(tmp_path):
    eng = st.new_engagement(tmp_path / "e")
    assert st.running_job(eng) is None
    j = st.add_job(eng, "enumerate", by="p@x.test", log="l")
    assert st.running_job(eng)["id"] == j["id"]
    import pytest
    with pytest.raises(SystemExit):
        st.add_job(eng, "chain")
    st.update_job(eng, j["id"], steps=[{"label": "a", "pid": 1, "exit": None}])
    done = st.finish_job(eng, j["id"], 0)
    assert done["state"] == "done" and done["ended"] and st.running_job(eng) is None
    j2 = st.add_job(eng, "chain")
    failed = st.finish_job(eng, j2["id"], 2, "review exited 2")
    assert failed["state"] == "failed" and failed["error"] == "review exited 2"
    assert [x["kind"] for x in st.jobs(eng)] == ["enumerate", "chain"]
    assert st.summary(eng)["job"] is None


def test_update_engagement_keeps_comments_and_other_keys(tmp_path):
    eng = tmp_path / "e"; eng.mkdir()
    (eng / "engagement.yaml").write_text(
        "# the practice's note\n\ntarget: target\n\nclaim_sources:\n  - README.md\n\n\nretention: keep\n\n"
        "# THE BUYER IS ILLUSTRATIVE.\ntransaction: |\n  Client: someone\n  Subject: code\n\nthresholds: |\n  Paying for: code\n")
    cfg = st.update_engagement(eng, claim_sources=["README.md", "docs/a.md"], client_emails=["c@x.test"], target=None)
    text = (eng / "engagement.yaml").read_text()
    assert "# the practice's note" in text and "# THE BUYER IS ILLUSTRATIVE." in text
    assert "transaction: |\n  Client: someone\n  Subject: code\n" in text
    assert "claim_sources:\n  - README.md\n  - docs/a.md\n" in text
    assert text.rstrip().endswith("client_emails:\n  - c@x.test")
    assert cfg["claim_sources"] == ["README.md", "docs/a.md"] and cfg["client_emails"] == ["c@x.test"]
    assert cfg["target"] == "target" and cfg["transaction"].startswith("Client: someone")
    cfg = st.update_engagement(eng, claim_sources=[], target="targets/x")
    text = (eng / "engagement.yaml").read_text()
    assert "claim_sources: []\n" in text and "target: targets/x\n" in text and text.count("target:") == 1
