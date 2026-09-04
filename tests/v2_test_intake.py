"""The intake form: schema, check, ledger, the engagement blocks, the fill
loop with a stubbed emission, and --finish on a temporary engagement."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.intake import schemas as sch                  # noqa: E402
from workflowsv2.intake import runner as rn                    # noqa: E402


def test_empty_form_matches_schema_and_checks_as_empty():
    f = sch.empty_form()
    req = sch.intake_schema()["required"]
    assert set(req) == set(f)
    c = sch.check_intake(f)
    assert c["complete"] == [] and c["filled"] == 0 and c["total"] == 17
    assert c["emptiest"] == "identify"          # all equal: the first in §2 order
    assert sch.ledger(c).startswith("[form: 0 of 17 fields filled; still empty — identify (")


def test_check_finds_the_emptiest_slot_and_completeness():
    f = sch.empty_form()
    for k in sch.SLOTS["identify"]:
        f["identify"][k] = "x"
    f["situation"]["subject"] = "the product"
    c = sch.check_intake(f)
    assert c["complete"] == ["identify"]
    assert c["emptiest"] in ("background", "assessment", "recommendation")
    assert "identify" not in c["empty"] and c["empty"]["situation"] == ["price", "structure", "timetable"]
    full = {s: {k: "v" for k in fs} for s, fs in sch.SLOTS.items()}
    full.update(open_questions=[], notes=[])
    assert sch.ledger(sch.check_intake(full)) == "[form: every slot is filled]"


def test_engagement_blocks_assemble_only_what_was_said():
    f = sch.empty_form()
    f["identify"]["client"] = "Acme"
    f["situation"]["price"] = "$480k, 12x MRR"
    f["assessment"]["walk_away"] = "backups not daily"
    b = sch.engagement_blocks(f)
    assert b["transaction"] == "Client: Acme\nPrice and basis: $480k, 12x MRR"
    assert b["thresholds"] == "Would end the deal: backups not daily"
    assert sch.engagement_blocks(sch.empty_form()) == {"transaction": "", "thresholds": ""}


def test_fill_form_with_a_stubbed_emission_updates_or_keeps():
    calls = []

    def good(system, user, schema, max_tokens):
        calls.append(user)
        f = sch.empty_form(); f["identify"]["client"] = "Acme"
        return {"obj": f, "parse": "parsed", "parse_error": None}

    prev = sch.empty_form()
    res = rn.fill_form(good, "METHOD", [("client", "We are Acme"), ("Jill", "Noted.")], prev, 100)
    assert res["updated"] and res["form"]["identify"]["client"] == "Acme"
    assert "client: We are Acme" in calls[0] and "INTAKE.md" in calls[0]

    def bad(system, user, schema, max_tokens):
        return {"obj": None, "parse": "unparseable", "parse_error": "cut"}
    res = rn.fill_form(bad, "METHOD", [], prev, 100)
    assert not res["updated"] and res["form"] is prev and res["parse_error"] == "cut"


def test_finish_writes_the_intake_blocks_and_a_brief_once(tmp_path):
    eng = tmp_path / "e"; eng.mkdir()
    (eng / "engagement.yaml").write_text("# header\ntarget: t\nclaim_sources: [a.md]\n")
    idir = eng / "intakes" / "2026-09-03T00-00-00Z"; idir.mkdir(parents=True)
    f = sch.empty_form()
    f["identify"]["client"] = "Acme"; f["assessment"]["walk_away"] = "no backups"
    f["background"]["target"] = "FlowMetrics"; f["recommendation"]["scope"] = "doc2"
    res = rn.finish(eng, idir, f)
    assert res["written"] == ["transaction", "thresholds", "brief.md"]
    import yaml
    blocks = yaml.safe_load((idir / "blocks.yaml").read_text())
    assert blocks["transaction"].strip() == "Client: Acme"
    assert blocks["thresholds"].strip() == "Would end the deal: no backups"
    # engagement.yaml is not touched
    assert (eng / "engagement.yaml").read_text() == "# header\ntarget: t\nclaim_sources: [a.md]\n"
    assert "The target: FlowMetrics" in (eng / "brief.md").read_text()
    meta = json.loads((idir / "intake_meta.json").read_text())
    assert meta["written"] == res["written"] and meta["check"]["filled"] == 4
    # a second finish rewrites the blocks from the form and leaves the brief
    f["assessment"]["walk_away"] = "changed"
    (eng / "brief.md").write_text("edited by hand")
    res = rn.finish(eng, idir, f)
    assert res["written"] == ["transaction", "thresholds"]
    assert yaml.safe_load((idir / "blocks.yaml").read_text())["thresholds"].strip() == "Would end the deal: changed"
    assert (eng / "brief.md").read_text() == "edited by hand"


def test_intake_session_refuses_a_missing_engagement(tmp_path, monkeypatch):
    import pytest
    from workflowsv2.intake.session import IntakeSession
    monkeypatch.setattr(rn, "ENGAGEMENTS", tmp_path)
    with pytest.raises(SystemExit, match="the engagement comes first"):
        IntakeSession("nope")


def test_finish_records_the_conclusion_opt_in(tmp_path):
    eng = tmp_path / "e"; eng.mkdir()
    idir = eng / "intakes" / "I1"; idir.mkdir(parents=True)
    f = sch.empty_form(); f["identify"]["client"] = "Acme"
    res = rn.finish(eng, idir, f, conclusion=True)
    assert "conclusion" in res["written"]
    import yaml
    assert yaml.safe_load((idir / "blocks.yaml").read_text())["conclusion"] is True
    res = rn.finish(eng, idir, f)
    assert "conclusion" not in yaml.safe_load((idir / "blocks.yaml").read_text())
