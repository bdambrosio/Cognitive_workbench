"""The single-claim test after delivery: what is refused, what the two
runners are asked, what the record holds, and that the delivered run stays
the current run. The runners are faked; no model is called.

    python3 -m pytest tests/v2_test_supplement.py -q
"""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2 import engagement_state as state                     # noqa: E402
from workflowsv2.claims_audit import supplement                        # noqa: E402

SRC = "docs/CLI.md"
CLAIMS = [
    {"id": 1, "quote": "The CLI lists links.", "lines": [3, 3], "statement": "The CLI can list links.",
     "about": "target", "tier": 1, "tier_basis": "bears on the decision"},
    {"id": 2, "quote": "It reads a config file.", "lines": [5, 5], "statement": "The CLI reads a config file.",
     "about": "target", "tier": 2, "tier_basis": "reference detail"},
    {"id": 3, "quote": "It reads a config file.", "lines": [5, 5], "statement": "The file is TOML.",
     "about": "target", "tier": 3, "tier_basis": "minor", "implied_by": 1},
]


def _json(path: Path, obj) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj))


@pytest.fixture
def eng(tmp_path, monkeypatch):
    """An engagement with a delivered run, a frozen surface and no job."""
    d = tmp_path / "e1"
    delivered_audit = d / "runs" / "2026-09-01T00-00-00Z_audit_e1_docs_cli_md_T"
    _json(delivered_audit / "run_meta.json", {"model_config": "measure/models/the_engagements.yaml"})
    merged = d / "merged" / "2026-09-01T01-00-00Z_chain_T"
    _json(merged / "meta.json", {"intake": None, "runs": [{"dir": str(delivered_audit), "claim_source": SRC}]})
    (merged / "report.md").write_text("# report\n")
    _json(d / "surface" / "docs_cli_md.surface.json", {"claim_source": SRC, "claims": CLAIMS})
    _json(d / "state.json", {})
    monkeypatch.setattr(state, "ENGAGEMENTS", tmp_path)
    return d


def _fake_runners(monkeypatch, eng, calls, audit_exit=0, review_exit=0, verdict="real", holds=True):
    """`_step` replaced: records each command and writes what the real
    runner would leave behind."""
    def step(argv, log):
        calls.append(argv)
        if "workflowsv2/claims_audit/runner.py" in argv[1]:
            if audit_exit == 0:
                world = argv[argv.index("--world") + 1]
                run = eng / "runs" / f"2026-09-21T00-00-00Z_{world}"
                surface = json.loads(Path(argv[argv.index("--surface") + 1]).read_text())
                tested = [c["id"] for c in surface["claims"] if c.get("tier") not in (2, 3)]
                _json(run / "run_meta.json", {"resolved_model": "served/model", "resolved_temperature": 1.0, "top_p": 0.95})
                _json(run / "findings.json", {"findings": [
                    {"claim_id": i, "adjudication": {"verdict": verdict},
                     "evidence": [{"form": "citation", "document": "src/cli.rs", "lines": [10, 12],
                                   "quote": "fn load_config()", "shows": "The CLI loads a config file."}]}
                    for i in tested]})
                (run / "issues.jsonl").write_text(json.dumps(
                    {"stage": "claims_audit", "code": "surface_check", "severity": "blocking", "text": "a problem"}) + "\n")
            return audit_exit
        run = Path(argv[argv.index("--run") + 1])
        if review_exit == 0:
            ids = [f["claim_id"] for f in json.loads((run / "findings.json").read_text())["findings"]]
            _json(run / "review" / "outcomes.json", {
                "derived": {"outcomes": {str(i): {"holds": holds, "adverse_observations": [] if holds else ["evidence_supports"]}
                                         for i in ids}},
                "standings": {"ran": True, "per_finding": {str(i): {"standing": "control"} for i in ids}}})
        return review_exit
    monkeypatch.setattr(supplement, "_step", step)


def test_the_one_claim_surface_loses_its_tier_and_carries_a_parent_as_listed():
    surface = {"claim_source": SRC, "claims": CLAIMS}
    one = supplement.one_claim_surface(surface, SRC, 2)
    assert [c["id"] for c in one["claims"]] == [2] and "tier" not in one["claims"][0]
    assert CLAIMS[1]["tier"] == 2                                     # the frozen surface is not changed
    with_parent = supplement.one_claim_surface(surface, SRC, 3)["claims"]
    assert [c["id"] for c in with_parent] == [1, 3]
    assert with_parent[0]["tier"] == 2 and "delivered report" in with_parent[0]["tier_basis"]   # the parent is not tested again
    from workflowsv2.claims_audit import schemas
    tested, listed = schemas.split_by_tier(with_parent)
    assert [c["id"] for c in tested] == [3] and [c["id"] for c in listed] == [1]
    for cid, why in ((1, "tested it"), (9, "no claim 9")):
        with pytest.raises(supplement.Refused, match=why):
            supplement.one_claim_surface(surface, SRC, cid)


def test_run_asks_the_two_runners_as_the_chain_does_and_records_what_they_found(eng, monkeypatch):
    calls = []
    _fake_runners(monkeypatch, eng, calls)
    before = state.current_run(eng, None)
    out = supplement.run(eng, SRC, 2, by="Bruce")
    audit, review = calls
    world = audit[audit.index("--world") + 1]
    assert world.startswith("supp_e1_docs_cli_md_2_")
    assert audit[2:] == ["--engagement", "e1", "--world", world, "--claim-source", SRC,
                         "--surface", str(out / "surface.json"), "--model", "measure/models/the_engagements.yaml"]
    assert review[2:] == ["--run", str(eng / "runs" / f"2026-09-21T00-00-00Z_{world}"),
                          "--model", "measure/models/the_engagements.yaml", "--world", f"review_{world}"]
    assert "--temperature" not in audit + review                      # temperature resolves per model, never here

    rec = json.loads((out / "supplement.json").read_text())
    assert rec["tier"] == 2 and rec["tier_basis"] == "reference detail" and rec["claim"]["statement"].startswith("The CLI reads")
    assert rec["delivered_run"] == before.name and rec["model_is_delivered"] and rec["approved"] is None
    assert rec["finding"]["adjudication"]["verdict"] == "real" and rec["resolved_temperature"] == 1.0
    assert rec["review"] == {"holds": True, "adverse_observations": [], "standing": "control"}
    assert [p["text"] for p in rec["run_problems"]] == ["a problem"] and rec["error"] is None
    text = (out / "supplement.md").read_text()
    assert "**Verdict:** real" in text and "**Independent check:** holds" in text
    assert "`src/cli.rs:10-12`" in text and "NOT APPROVED" in text and "PROBLEM" in text

    assert state.current_run(eng, None) == before                     # the delivered run is still the current run
    assert sorted(p.name for p in (eng / "merged").iterdir()) == [before.name]


def test_what_is_refused(eng, monkeypatch):
    calls = []
    _fake_runners(monkeypatch, eng, calls)
    monkeypatch.setattr(state, "running_job", lambda d: {"kind": "chain"})
    with pytest.raises(supplement.Refused, match="chain job is running"):
        supplement.run(eng, SRC, 2, by="Bruce")
    monkeypatch.setattr(state, "running_job", lambda d: None)
    with pytest.raises(supplement.Refused, match="no frozen surface"):
        supplement.run(eng, "README.md", 2, by="Bruce")
    supplement.run(eng, SRC, 2, by="Bruce")
    with pytest.raises(supplement.Refused, match="already tested after delivery"):
        supplement.run(eng, SRC, 2, by="Bruce")
    assert len(calls) == 2                                            # only the one test ran
    (eng / "merged" / "2026-09-01T01-00-00Z_chain_T" / "report.md").unlink()
    with pytest.raises(supplement.Refused, match="no current run with a report"):
        supplement.run(eng, SRC, 3, by="Bruce")


def test_another_model_is_recorded_as_not_the_delivered_one(eng, monkeypatch):
    calls = []
    _fake_runners(monkeypatch, eng, calls)
    out = supplement.run(eng, SRC, 2, by="Bruce", model="measure/models/another.yaml")
    rec = json.loads((out / "supplement.json").read_text())
    assert calls[0][-1] == "measure/models/another.yaml" and not rec["model_is_delivered"]
    assert "NOT the model the delivered audit ran on" in (out / "supplement.md").read_text()


def test_a_failed_audit_is_recorded_the_review_is_not_run_and_the_test_can_be_run_again(eng, monkeypatch):
    calls = []
    _fake_runners(monkeypatch, eng, calls, audit_exit=3)
    out = supplement.run(eng, SRC, 2, by="Bruce")
    rec = json.loads((out / "supplement.json").read_text())
    assert len(calls) == 1 and rec["error"] == "the audit exited 3" and rec["audit_run"] is None
    with pytest.raises(supplement.Refused, match="did not finish"):
        supplement.approve(eng, out.name, by="Bruce")
    _fake_runners(monkeypatch, eng, calls)
    monkeypatch.setattr(state, "stamp", lambda: "2026-09-22T00-00-00Z")          # a second directory
    assert supplement.run(eng, SRC, 2, by="Bruce") != out                        # a failed test does not block another


def test_approve_records_who_and_when_and_a_check_that_does_not_hold_is_shown(eng, monkeypatch):
    _fake_runners(monkeypatch, eng, [], verdict="contradicted", holds=False)
    out = supplement.run(eng, SRC, 2, by="Bruce")
    rec = supplement.approve(eng, out.name, by="Bruce D'Ambrosio")
    assert rec["approved"]["by"] == "Bruce D'Ambrosio" and rec["approved"]["at"]
    text = (out / "supplement.md").read_text()
    assert "does not hold (evidence_supports); retest: control" in text and "Approved by Bruce D'Ambrosio" in text
    run_dir = Path(rec["audit_run"])
    (run_dir / "review" / "outcomes.json").write_text(json.dumps({
        "derived": {"outcomes": {"2": {"holds": False, "adverse_observations": ["verdict_calibration"]}}},
        "standings": {"ran": False, "reason": "nothing qualified and nothing sampled"}}))
    assert supplement.result(run_dir, 2)["review"]["standing"] == "not retested (nothing qualified and nothing sampled)"
    assert [r["dir"] for r in supplement.records(eng)] == [out.name]
    with pytest.raises(supplement.Refused, match="no supplement"):
        supplement.approve(eng, "nothing", by="Bruce")
