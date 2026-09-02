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


def test_finish_writes_blocks_and_brief_and_leaves_existing_alone(tmp_path):
    eng = tmp_path / "e"; eng.mkdir()
    (eng / "engagement.yaml").write_text("# header\ntarget: t\nclaim_sources: [a.md]\n")
    f = sch.empty_form()
    f["identify"]["client"] = "Acme"; f["assessment"]["walk_away"] = "no backups"
    f["background"]["target"] = "FlowMetrics"; f["recommendation"]["scope"] = "doc2"
    res = rn.finish(eng, f)
    assert res["written"] == ["transaction", "thresholds", "brief.md"] and res["skipped"] == []
    import yaml
    cfg = yaml.safe_load((eng / "engagement.yaml").read_text())
    assert cfg["target"] == "t" and cfg["transaction"].strip() == "Client: Acme"
    assert cfg["thresholds"].strip() == "Would end the deal: no backups"
    assert "# header" in (eng / "engagement.yaml").read_text()
    assert "The target: FlowMetrics" in (eng / "brief.md").read_text()
    meta = json.loads((eng / "intake_meta.json").read_text())
    assert meta["written"] == res["written"] and meta["check"]["filled"] == 4
    # a second finish leaves the present blocks and the brief alone
    f["assessment"]["walk_away"] = "changed"
    res = rn.finish(eng, f)
    assert res["written"] == [] and res["skipped"] == ["transaction", "thresholds"]
    assert yaml.safe_load((eng / "engagement.yaml").read_text())["thresholds"].strip() == "Would end the deal: no backups"
