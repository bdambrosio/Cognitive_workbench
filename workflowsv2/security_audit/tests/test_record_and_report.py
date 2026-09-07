"""The security audit's record and document, without a model: citation
resolution and the form checks, the computed conclusion, the assembled
document and its slots, change-since across two runs, and the prose check.

Run by path: python3 -m pytest workflowsv2/security_audit/tests -q
"""
import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.security_audit import record, report          # noqa: E402


def _collection(d: Path, unauthorised=()):
    c = d / "collection"
    c.mkdir(parents=True)
    (c / "sockets.txt").write_text(
        "Netid State  Local Address:Port\n"
        "tcp   LISTEN 0.0.0.0:22\n"
        "tcp   LISTEN 127.0.0.1:631\n", encoding="utf-8")
    (c / "firewall.txt").write_text(
        "Status: active\nTo   Action  From\n22   ALLOW   Anywhere\n", encoding="utf-8")
    outcomes = {"sockets": {"outcome": "completed", "lines": 3},
                "firewall": {"outcome": "completed", "lines": 3},
                "apparmor": {"outcome": "not run", "why": "not authorised by the engagement"}}
    for k in unauthorised:
        outcomes[k] = {"outcome": "unauthorised", "why": "sudo -n exited 1"}
        (c / f"{k}.txt").write_text(f"# {k}: unauthorised\n", encoding="utf-8")
    (c / "outcomes.json").write_text(json.dumps(outcomes), encoding="utf-8")
    return c


def _cite(art, a, b, q):
    return {"artifact": art, "lines": [a, b], "quote": q}


def _surface():
    return [{"kind": "socket", "identity": "socket tcp 0.0.0.0:22", "reach": "lan_unauthenticated",
             "description": "ssh on every interface", "cite": _cite("sockets.txt", 2, 2, "0.0.0.0:22")},
            {"kind": "socket", "identity": "socket tcp 127.0.0.1:631", "reach": "other",
             "description": "cups on loopback", "cite": _cite("sockets.txt", 3, 3, "127.0.0.1:631")}]


def _findings(disp="confirmed"):
    return {"findings": [
        {"title": "ssh reachable from the LAN", "element": "S1", "disposition": disp,
         "exposure": {"cite": _cite("sockets.txt", 2, 2, "LISTEN 0.0.0.0:22"), "text": "listens on all interfaces"},
         "path": [{"cite": _cite("firewall.txt", 3, 3, "22   ALLOW   Anywhere"), "text": "the firewall allows it"}],
         "consequence": {"cite": _cite("sockets.txt", 2, 2, "0.0.0.0:22"), "text": "a login prompt to anyone on the LAN"},
         "assessment": "password guessing from the LAN", "remedy_locus": "/etc/ssh/sshd_config ListenAddress"}],
        "examined": ["S1", "S2"],
        "limitations": [{"text": "the collection is one snapshot"}],
        "gaps": [{"unknown": "whether password auth is on", "why": "sshd_config is not in the probe set",
                  "settles": "sshd -T | grep passwordauthentication", "element": "S1"}]}


def _write_run(d: Path, name: str, surface, findings, unauthorised=()):
    run = d / name
    run.mkdir(parents=True)
    _collection(run, unauthorised)
    frozen = record.freeze(surface)
    (run / "surface.json").write_text(json.dumps({"elements": frozen}), encoding="utf-8")
    (run / "findings.json").write_text(json.dumps(findings), encoding="utf-8")
    (run / "run_meta.json").write_text(json.dumps({"hosts": ["localhost"], "captured_at_utc": "2026-09-07T20-00-00Z"}),
                                       encoding="utf-8")
    return run


def test_citations_resolve_or_say_why(tmp_path):
    c = _collection(tmp_path)
    assert record.resolve_cite(_cite("sockets.txt", 2, 2, "0.0.0.0:22"), c) is None
    assert record.resolve_cite(_cite("collection/sockets.txt", 2, 3, "LISTEN 0.0.0.0:22 tcp"), c) is None  # flattened span
    assert "does not exist" in record.resolve_cite(_cite("nope.txt", 1, 1, "x"), c)
    assert "outside" in record.resolve_cite(_cite("sockets.txt", 2, 9, "x"), c)
    assert record.resolve_cite(_cite("sockets.txt", 1, 1, "nothing like this"), c).startswith("quote not found")
    assert "not a collection file name" in record.resolve_cite(_cite("../etc/passwd", 1, 1, "x"), c)
    assert "empty quote" in record.resolve_cite(_cite("sockets.txt", 1, 1, ""), c)
    # punctuation drift on the first line: loose; real text at the wrong line: relocated and corrected
    assert record.resolve_cite(_cite("sockets.txt", 2, 2, "LISTEN 0.0.0.0 22"), c) == record.LOOSE
    moved = _cite("sockets.txt", 1, 1, "127.0.0.1:631")
    assert record.resolve_cite(moved, c) == record.RELOCATED and moved["lines"] == [3, 3]
    assert "appears at" in record.resolve_cite(_cite("sockets.txt", 1, 1, "LISTEN"), c)   # ambiguous: not moved


def test_surface_and_findings_checks(tmp_path):
    c = _collection(tmp_path)
    frozen = record.freeze(_surface())
    assert [e["label"] for e in frozen] == ["S1", "S2"]
    assert record.check_surface({"elements": _surface()}, c)["ok"]
    # a repeat is the same way in: dropped at the freeze, labels stay dense
    twice = record.freeze(_surface() + [dict(_surface()[0], description="again")])
    assert [e["label"] for e in twice] == ["S1", "S2"] and twice[0]["description"] == "ssh on every interface"
    dup = {"elements": _surface() + [dict(_surface()[0])]}
    assert any("repeats" in p for p in record.check_surface(dup, c)["problems"])
    ok = record.check_findings(_findings(), frozen, c)
    assert ok["ok"], ok["problems"]
    bad = _findings("uncertain")
    bad["gaps"] = []
    bad["findings"][0]["element"] = "S9"
    bad["findings"][0]["exposure"]["cite"]["quote"] = "nothing like this at all"
    r = record.check_findings(bad, frozen, c)
    assert not r["ok"]
    assert any("not on the frozen surface" in p for p in r["problems"])
    assert any("no gap naming" in p for p in r["problems"])
    assert any("quote not found" in p for p in r["problems"])
    assert bad["findings"][0]["citation_problems"]              # written onto the finding
    assert "surface_lines" and "S1" in record.surface_lines(frozen)


def test_conclusion_is_computed_from_dispositions_and_grants():
    frozen = record.freeze(_surface())
    fs = _findings()["findings"]
    c = record.conclusion(fs, frozen, {})
    assert c["conclusion"] == "Exposed" and c["confirmed_without_credentials"] == 1
    frozen[0]["reach"] = "authenticated"
    assert record.conclusion(fs, frozen, {})["conclusion"] == "Weak"
    c = record.conclusion([], frozen, {"apparmor": {"outcome": "unauthorised"}})
    assert c["conclusion"].startswith("Hardened") and c["caveat_required"] and c["withheld_grants"] == ["apparmor"]
    assert record.conclusion([], frozen, {})["caveat_required"] is False


def test_document_assembles_with_markers_then_prose(tmp_path):
    run_dir = _write_run(tmp_path, "2026-09-07T20-00-00Z_sec_3", _surface(), _findings(), unauthorised=("listener_owners",))
    run = record.load_run(run_dir)
    doc = report.assemble(run, None, None)
    for h in ("# Security review — localhost", "## Probes", "## Conclusion", "## How to read a finding",
              "## Findings", "## The attack surface", "## Gap map", "## Limitations"):
        assert h in doc, h
    assert "## Change since the previous review" not in doc
    for f in report.FIELDS:
        if f != "change_note":
            assert f"[[{f}]]" in doc
    assert "[[change_note]]" not in doc
    assert "**Exposed.**" in doc and "`listener_owners`" in doc          # computed, with the withheld grant
    assert "### S1 — ssh reachable from the LAN — [confirmed]" in doc
    assert "`collection/firewall.txt:3`" in doc and "the firewall allows it" in doc
    assert "**Remedy locus.** /etc/ssh/sshd_config ListenAddress" in doc
    assert "every citation resolves" in doc
    assert "| S2 | socket | `socket tcp 127.0.0.1:631` | other |" in doc
    assert "*Settled by:* `sshd -T | grep passwordauthentication`" in doc
    prose = {f: f"<{f}>" for f in report.FIELDS}
    prose["change_note"] = ""
    doc2 = report.assemble(run, prose, None)
    assert "[[" not in doc2 and "<summary>" in doc2 and "<limitations>" in doc2
    # the whole stage without a model writes the skeleton and the report
    out = report.run(run_dir, None)
    assert out.is_file() and (run_dir / "report_skeleton.md").is_file()
    assert "[[summary]]" in out.read_text()


def test_change_since_compares_by_identity_not_label(tmp_path):
    first = _write_run(tmp_path, "2026-09-06T00-00-00Z_sec_1", _surface(), _findings())
    later_surface = [_surface()[1], _surface()[0],
                     {"kind": "account", "identity": "account deploy", "reach": "authenticated",
                      "description": "a new login", "cite": _cite("sockets.txt", 1, 1, "Netid")}]
    later = _write_run(tmp_path, "2026-09-07T00-00-00Z_sec_2", later_surface, {"findings": [], "examined": [], "limitations": [], "gaps": []},
                       unauthorised=())
    assert record.previous_run(later) == first
    delta = report.change_since(record.load_run(later), record.load_run(first))
    assert [e["identity"] for e in delta["new_elements"]] == ["account deploy"]
    assert delta["gone_elements"] == []
    assert [i for i, _ in delta["resolved_candidates"]] == ["socket tcp 0.0.0.0:22"]   # label moved S1->S2, identity matched
    doc = report.assemble(record.load_run(later), None, record.load_run(first))
    assert "## Change since the previous review" in doc and "[[change_note]]" in doc
    assert "not resolved" in doc and "enumerated this time and not last time" in doc


def test_prose_check(tmp_path):
    run_dir = _write_run(tmp_path, "2026-09-07T20-00-00Z_sec_3", _surface(), _findings())
    run = record.load_run(run_dir)
    good = {f: f"about S1 and S2" for f in report.FIELDS}
    good["change_note"] = ""
    assert report.check_prose(good, run, has_change=False)["ok"]
    bad = dict(good)
    bad["summary"] = "S7 is exposed\n=== REPORT ==="
    bad["change_note"] = "there was change"
    r = report.check_prose(bad, run, has_change=False)
    assert any("S7" in p for p in r["problems"])
    assert any("===" in p for p in r["problems"])
    assert any("no such section" in p for p in r["problems"])
    assert report.prose_schema()["required"] == list(report.FIELDS)
