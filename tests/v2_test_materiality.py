"""The merge is mechanical and the ratings check is arithmetic.

Two synthetic runs: one reviewed with a finding that does not hold and a
citation problem, one unreviewed; one quote restated across both.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.audit_materiality import merge as mg           # noqa: E402
from workflowsv2.audit_materiality import schemas as ms         # noqa: E402


def _run(tmp_path, name, source, claims, findings, meta_problems=(),
         outcomes=None):
    d = tmp_path / name
    d.mkdir()
    (d / "claims.json").write_text(json.dumps(
        {"claim_source": source, "claims": claims}))
    (d / "findings.json").write_text(json.dumps(
        {"claim_source": source, "findings": findings,
         "unclaimed": [{"note": "seen", "evidence": {"form": "citation"}}],
         "questions": ["who?"]}))
    (d / "run_meta.json").write_text(json.dumps(
        {"world": name, "resolved_model": "m", "harness_rev": "abc",
         "output_check": {"ok": not meta_problems,
                          "problems": list(meta_problems)}}))
    if outcomes is not None:
        (d / "review").mkdir()
        (d / "review" / "outcomes.json").write_text(json.dumps(outcomes))
    return d


def _fixture(tmp_path):
    cite = [{"form": "citation", "document": "d.md", "lines": [1, 1],
             "quote": "x", "shows": "y"}]
    a = _run(tmp_path, "a", "doc1.md",
             [{"id": 1, "quote": "Backups run daily", "lines": [3, 3], "statement": "s"},
              {"id": 2, "quote": "Low churn", "lines": [4, 4], "statement": "s"}],
             [{"claim_id": 1, "adjudication": {"verdict": "contradicted", "gap": "g"}, "evidence": cite},
              {"claim_id": 2, "adjudication": {"verdict": "real"}, "evidence": cite}],
             meta_problems=["finding 1 evidence 1: quote is not at d.md:1-1; x",
                            "claim 3 in the frozen surface has no finding"],
             outcomes={"derived": {"outcomes": {
                 "1": {"holds": False, "adverse_observations": ["evidence_supports"]},
                 "2": {"holds": True, "adverse_observations": []}}},
                 "standings": {"per_finding": {"1": {"standing": "does not stand"}}}})
    b = _run(tmp_path, "b", "doc9.md",
             [{"id": 1, "quote": "Backups run daily", "lines": [9, 9], "statement": "s"}],
             [{"claim_id": 1, "adjudication": {"verdict": "partial", "gap": "g"}, "evidence": cite}])
    return a, b


def test_merge_carries_review_outcome_problems_and_restatements(tmp_path):
    a, b = _fixture(tmp_path)
    m = mg.merge([a, b])
    assert m["figures"]["runs"] == 2 and m["figures"]["findings"] == 3
    assert m["figures"]["reviewed"] == 2 and m["figures"]["unreviewed"] == 1
    f1 = next(f for f in m["findings"] if f["claim_source"] == "doc1.md" and f["claim_id"] == 1)
    assert f1["review"]["outcome"] == "does_not_hold"
    assert f1["review"]["adverse_observations"] == ["evidence_supports"]
    assert f1["review"]["retest"]["standing"] == "does not stand"
    assert f1["citation_problems"] == ["finding 1 evidence 1: quote is not at d.md:1-1; x"]
    f9 = next(f for f in m["findings"] if f["claim_source"] == "doc9.md")
    assert f9["review"] == {"outcome": "unreviewed"} and f9["citation_problems"] == []
    # the claim-level problem stays with the run, not a finding
    assert m["figures"]["run_problems"] == {"doc1.md": ["claim 3 in the frozen surface has no finding"]}
    assert [x["in"] for x in m["figures"]["restated_quotes"]] == [[
        {"claim_source": "doc1.md", "claim_id": 1, "verdict": "contradicted"},
        {"claim_source": "doc9.md", "claim_id": 1, "verdict": "partial"}]]
    assert len(m["unclaimed"]) == 2 and m["questions"][0]["claim_source"] == "doc1.md"


def test_merge_refuses_an_unfinished_run(tmp_path):
    d = tmp_path / "x"; d.mkdir()
    (d / "claims.json").write_text("{}")
    try:
        mg.merge([d])
    except FileNotFoundError as e:
        assert "findings.json" in str(e)
    else:
        raise AssertionError("merged a run with no findings")


def test_ratings_check_wants_every_rateable_finding_once(tmp_path):
    a, b = _fixture(tmp_path)
    m = mg.merge([a, b])
    ok = {"ratings": [
        {"claim_source": "doc1.md", "claim_id": 1, "materiality": "decisive", "basis": "b"},
        {"claim_source": "doc9.md", "claim_id": 1, "materiality": "material", "basis": "b"}]}
    res = ms.check_ratings(ok, m)
    assert res["ok"], res["problems"]
    assert res["figures"] == {"rateable": 2, "rated": 2,
                              "materiality": {"decisive": 1, "material": 1}}
    bad = {"ratings": [
        {"claim_source": "doc1.md", "claim_id": 1, "materiality": "decisive", "basis": "b"},
        {"claim_source": "doc1.md", "claim_id": 1, "materiality": "material", "basis": "b"},
        {"claim_source": "doc1.md", "claim_id": 2, "materiality": "material", "basis": ""},
        {"claim_source": "nope.md", "claim_id": 7, "materiality": "material", "basis": "b"}]}
    res = ms.check_ratings(bad, m)
    text = "\n".join(res["problems"])
    assert "rated twice" in text
    assert "`real` finding is not rated" in text
    assert "no basis" in text
    assert "no such finding" in text
    assert "finding doc9.md#1 was not rated" in text


import pytest  # noqa: E402


@pytest.mark.parametrize("modname", [
    "workflowsv2.audit_materiality.runner",
    "workflowsv2.claims_audit.runner",
    "workflowsv2.audit_review.runner",
    "workflowsv2.emit",
])
def test_every_name_the_runner_uses_at_run_time_resolves(modname):
    """Copied from the review's test and widened to every v2 runner on
    2026-09-02, after consolidating `_emit` into emit.py sliced
    `_merge_emissions` out of the audit runner with it: a NameError at
    adjudication cost a GLM run, and only the review runner had this test."""
    import ast
    import builtins
    import importlib
    import inspect
    mod = importlib.import_module(modname)

    src = ast.parse(inspect.getsource(mod))
    problems = []
    for fn in [n for n in src.body if isinstance(n, ast.FunctionDef)]:
        local = set()
        for node in ast.walk(fn):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                local.update(a.asname or a.name.split(".")[0] for a in node.names)
            elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                local.add(node.id)
            elif isinstance(node, ast.arg):
                local.add(node.arg)
            elif isinstance(node, ast.ExceptHandler) and node.name:
                local.add(node.name)
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                local.add(node.name)
        for node in ast.walk(fn):
            if not (isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)):
                continue
            n = node.id
            if n in local or hasattr(mod, n) or hasattr(builtins, n):
                continue
            problems.append(f"{fn.name}: {n} (line {node.lineno})")
    assert not problems, "; ".join(sorted(set(problems)))


def test_render_puts_decisive_first_and_marks_real(tmp_path):
    from workflowsv2.audit_materiality.runner import render, _rating_text
    a, b = _fixture(tmp_path)
    m = mg.merge([a, b])
    ratings = {"ratings": [
        {"claim_source": "doc1.md", "claim_id": 1, "materiality": "material", "basis": "b1"},
        {"claim_source": "doc9.md", "claim_id": 1, "materiality": "decisive", "basis": "b9"}],
        "figures": {"material": 1, "decisive": 1}}
    md = render(m, ratings)
    rows = [l for l in md.splitlines() if l.startswith("| ")][1:]
    assert rows[0].startswith("| decisive | doc9.md")
    assert rows[1].startswith("| material | doc1.md | 1")
    assert "does_not_hold (evidence_supports)" in rows[1] and "1 problem(s)" in rows[1]
    assert rows[2].startswith("| real | doc1.md | 2")
    text = _rating_text(m["findings"][0])
    assert text.startswith("--- doc1.md claim 1") and "review    : does_not_hold" in text
    assert "citation problems:" in text
