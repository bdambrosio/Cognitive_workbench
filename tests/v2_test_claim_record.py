"""The consultation's `claim` action: one claim's record served whole, each
citation resolved against the materials at call time, the report's lines
that carry the claim, and registration on a built loop."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.claims_audit import record, schemas             # noqa: E402

SRC = "line one\nApache License\n  Version 2.0\nfour\nfive\nsix\n"
REPORT = """# Claims review — t

## What the review showed

### README.md, claim 2 — materiality: material

> "two"

**Verdict:** contradicted

## Claims that hold

| claim source | id | claim | evidence |
|---|---|---|---|
| README.md | 1 | one | `LICENSE` lines 2–3 |

## Appendix — every claim and its verdict

| claim source | id | verdict |
|---|---|---|
| README.md | 1 | real |
| README.md | 2 | contradicted |
"""


def _merged():
    return {"findings": [
        {"claim_source": "README.md", "claim_id": 1, "quote": "one", "lines": [1, 1],
         "statement": "s1", "about": "target",
         "adjudication": {"verdict": "real"},
         "evidence": [{"form": "citation", "document": "LICENSE", "lines": [2, 3],
                       "quote": "Apache License\n  Version 2.0", "shows": "it"}],
         "review": {"outcome": "holds", "adverse_observations": []}},
        {"claim_source": "README.md", "claim_id": 2, "quote": "two", "lines": [2, 2],
         "statement": "s2", "about": "target",
         "adjudication": {"verdict": "contradicted", "gap": "no such thing"},
         "correction": "Verdict unchanged; gap reworded.",
         "evidence": [{"form": "citation", "document": "LICENSE", "lines": [4, 5],
                       "quote": "Apache License", "shows": "moved"},
                      {"form": "citation", "document": "LICENSE", "lines": [4, 5],
                       "quote": "nowhere text", "shows": "gone"},
                      {"form": "citation", "document": "missing.txt", "lines": [1, 1],
                       "quote": "x", "shows": "absent file"},
                      {"form": "search", "kind": "lexical", "performed": "grep x",
                       "result": "nothing", "candidates": ["LICENSE"]}],
         "review": {"outcome": "does_not_hold", "adverse_observations": ["verdict_calibration"]}},
    ], "questions": [{"claim_source": "README.md", "claim_id": 2, "question": "Where is it?"}]}


def _materiality():
    return {"ratings": [{"claim_source": "README.md", "claim_id": 2, "materiality": "material",
                         "basis": "B", "agreement": "2 of 2", "borderline": False,
                         "samples": [{"materiality": "material", "basis": "B1"},
                                     {"materiality": "material", "basis": "B2"}]}],
            "exposures": []}


@pytest.fixture
def rec(tmp_path):
    target = tmp_path / "target"; target.mkdir()
    (target / "LICENSE").write_text(SRC)
    (target / "README.md").write_text("one\ntwo\n")
    merged_dir = tmp_path / "merged"; merged_dir.mkdir()
    (merged_dir / "materiality.json").write_text(json.dumps(_materiality()))
    (merged_dir / "report.md").write_text(REPORT)
    return record.ClaimRecord(_merged(), merged_dir, target)


def test_quote_at_statuses():
    body = SRC.splitlines()
    assert schemas.quote_at(body, 2, 3, "Apache License\n  Version 2.0") == ("exact", None)
    assert schemas.quote_at(body, 1, 3, "line one\nVersion 2.0") == ("joined", None)
    assert schemas.quote_at(body, 2, 3, "2|Apache License\n3|  Version 2.0") == (
        "prefixed", "Apache License\n  Version 2.0")
    assert schemas.quote_at(body, 4, 5, "Apache License") == ("elsewhere", None)
    assert schemas.quote_at(body, 4, 5, "nowhere text") == ("missing", "nowhere text")


def test_record_text_holds_the_finding_and_resolves_citations(rec):
    t = rec.text(("README.md", 1))
    assert "README.md #1 — verdict: real" in t
    assert "check: holds" in t
    assert "the quote is at LICENSE:2-3" in t
    assert "rating: none recorded" in t
    assert "line 15, a row of the table under 'Claims that hold'" in t
    assert "line 21, a row of the table under 'Appendix — every claim and its verdict'" in t


def test_record_text_shows_current_lines_when_a_quote_moved(rec):
    t = rec.text(("README.md", 2))
    assert "correction after the check:" in t
    assert "check: does_not_hold (observations: verdict_calibration)" in t
    assert "the quote is in the file but not at those lines. Lines 4-5 of LICENSE read now:" in t
    assert "        four\n        five" in t
    assert "the quote is not in the file; first part not found: 'nowhere text'" in t
    assert "document 'missing.txt' is not in the materials" in t
    assert "search (lexical)" in t and "candidates: LICENSE" in t
    assert "rating: materiality material (agreement 2 of 2, not borderline)" in t
    assert "each rater's basis (2):" in t and "2. material: B2" in t
    assert "lines 5-10, the finding's section under 'What the review showed'" in t
    assert "- Where is it?" in t


def test_invoke_ids_and_source_rules(rec):
    ok = rec.invoke({"ids": [1, 2]})
    assert ok["status"] == "ok" and "README.md #1" in ok["text"] and "README.md #2" in ok["text"]
    assert rec.invoke({"ids": 1})["status"] == "ok"           # a bare id is accepted
    assert "no claim 9" in rec.invoke({"ids": [9]})["text"]
    assert "at most 4" in rec.invoke({"ids": [1, 2, 1, 2, 1]})["text"]
    assert "must be a list" in rec.invoke({})["text"]
    # Two sources sharing an id need `source`.
    rec.findings[("OTHER.md", 1)] = rec.findings[("README.md", 1)]
    rec.sources = ["OTHER.md", "README.md"]
    assert "give `source`" in rec.invoke({"ids": [1]})["text"]
    assert rec.invoke({"ids": [1], "source": "OTHER.md"})["status"] == "ok"


def test_register_puts_the_action_on_a_loop(rec):
    from chat.tools import ToolsMixin
    loop = SimpleNamespace(_discovered_tools={}, _tool_module_cache={})
    record.register(loop, rec)
    meta = loop._discovered_tools["claim"]
    entry = ToolsMixin._render_discovered_tool_entry(loop, "claim", meta)
    assert '"tool": "claim", "ids": <...>, "source": <...>' in entry
    mod = loop._tool_module_cache["claim"]
    res = mod.react_invoke({"ids": [2]}, character_name="Jill", backend=None, logger=None)
    assert res["status"] == "ok" and "README.md #2" in res["text"]
