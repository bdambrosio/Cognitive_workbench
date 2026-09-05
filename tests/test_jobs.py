"""The site's job runner: steps run in order as subprocesses, the job record
and the stage follow the exit, the practice is mailed, and a second job is
refused while one runs. Real runners are replaced by shell commands."""
import os
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2 import engagement_state as st                  # noqa: E402
from client_ui import jobs, mail                                # noqa: E402


@pytest.fixture(autouse=True)
def _env(monkeypatch):
    monkeypatch.setenv("MAIL_DRY_RUN", "1")
    monkeypatch.setenv("PRACTICE_EMAILS", "practice@example.test")
    mail.sent.clear()
    yield
    jobs._steps_for = None


def _eng(tmp_path):
    return st.new_engagement(tmp_path / "e", client_emails=["c@example.test"])


def _fake(*cmds):
    def steps(eng_dir, kind, model, ts):
        for i, c in enumerate(cmds):
            yield (f"step {i + 1}", ["sh", "-c", c])
    return steps


def test_steps_run_in_order_and_the_job_is_recorded_done(tmp_path):
    eng = _eng(tmp_path)
    jobs._steps_for = _fake("echo one", "echo two")
    j = jobs.start(eng, "enumerate", by="p@example.test")
    j.wait(30)
    rec = st.jobs(eng)[0]
    assert rec["state"] == "done" and rec["exit"] == 0 and rec["error"] is None
    assert [s["label"] for s in rec["steps"]] == ["step 1", "step 2"]
    assert [s["exit"] for s in rec["steps"]] == [0, 0]
    log = Path(rec["log"]).read_text()
    assert "one" in log and "two" in log and "=== done" in log
    assert st.stage_value(eng, "enumeration") == "done"
    assert st.running_job(eng) is None
    assert mail.sent[-1]["to"] == ["practice@example.test"]
    assert "finished" in mail.sent[-1]["subject"]


def test_a_failing_step_ends_the_job_failed_and_skips_the_rest(tmp_path):
    eng = _eng(tmp_path)
    jobs._steps_for = _fake("echo first", "exit 3", "echo never")
    j = jobs.start(eng, "chain", by="p@example.test")
    j.wait(30)
    rec = st.jobs(eng)[0]
    assert rec["state"] == "failed" and rec["exit"] == 3
    assert rec["error"] == "step 2 exited 3"
    assert len(rec["steps"]) == 2
    assert "never" not in Path(rec["log"]).read_text()
    assert st.stage_value(eng, "chain") == "failed"
    assert "failed" in mail.sent[-1]["subject"]
    # The client is never mailed about a failure.
    assert all(r["to"] == ["practice@example.test"] for r in mail.sent)


def test_a_second_job_is_refused_while_one_runs(tmp_path):
    eng = _eng(tmp_path)
    release = tmp_path / "go"
    jobs._steps_for = _fake(f"while [ ! -f {release} ]; do sleep 0.05; done")
    j = jobs.start(eng, "enumerate")
    assert st.stage_value(eng, "enumeration") == "running"
    with pytest.raises(jobs.JobRunning):
        jobs.start(eng, "chain")
    with pytest.raises(jobs.JobRunning):
        jobs.start(eng, "enumerate")
    release.write_text("")
    j.wait(30)
    assert st.running_job(eng) is None
    assert len(st.jobs(eng)) == 1


def test_unknown_kind_and_unready_engagement(tmp_path):
    eng = _eng(tmp_path)
    with pytest.raises(ValueError):
        jobs.start(eng, "sweep")
    # No claim sources: refused before any record is made.
    with pytest.raises(SystemExit):
        jobs.start(eng, "enumerate")
    assert st.jobs(eng) == []


def test_real_step_lists_name_the_runners(tmp_path):
    eng = _eng(tmp_path)
    (eng / "engagement.yaml").write_text(
        "target: target\nclaim_sources: [README.md, docs/llms.txt]\nclient_emails: []\n")
    steps = jobs.enumerate_steps(eng, "m.yaml", "T")
    assert [s[0] for s in steps] == ["enumerate README.md", "enumerate docs/llms.txt"]
    argv = steps[1][1]
    assert argv[1].endswith("workflowsv2/claims_audit/runner.py")
    assert "--enumerate-only" in argv and "enum_e_docs_llms_txt_T" in argv
    chain = jobs.Chain(eng, "m.yaml", "T")
    assert chain.check() == "no frozen surface for README.md, docs/llms.txt"
    (eng / "surface").mkdir()
    for s in ("readme_md", "docs_llms_txt"):
        (eng / "surface" / f"{s}.surface.json").write_text("{}")
    assert chain.check() is None
    first = next(chain.steps())
    assert first[0] == "audit README.md" and "--surface" in first[1]
