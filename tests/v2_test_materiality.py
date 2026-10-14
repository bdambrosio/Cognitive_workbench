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
         "questions": [{"claim_id": 1, "question": "who?"}]
                      if source == "doc1.md" else ["who?"]}))
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
    search = [{"form": "search", "kind": "lexical", "performed": "p", "result": "r",
               "candidates": []},
              {"form": "search", "kind": "structural", "performed": "p", "result": "r",
               "candidates": ["x.py"]}]
    b = _run(tmp_path, "b", "doc9.md",
             [{"id": 1, "quote": "Backups run daily", "lines": [9, 9], "statement": "s"},
              {"id": 2, "quote": "Handoff to a human", "lines": [10, 10], "statement": "s"}],
             [{"claim_id": 1, "adjudication": {"verdict": "partial", "gap": "g"}, "evidence": cite},
              {"claim_id": 2, "adjudication": {"verdict": "unverifiable",
                                               "unresolved_because": "not_examined"},
               "evidence": search}])
    return a, b


def test_merge_carries_review_outcome_problems_and_restatements(tmp_path):
    a, b = _fixture(tmp_path)
    m = mg.merge([a, b])
    assert m["figures"]["runs"] == 2 and m["figures"]["findings"] == 4
    assert m["figures"]["reviewed"] == 2 and m["figures"]["unreviewed"] == 2
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
    assert len(m["unclaimed"]) == 2
    # The object form since 2026-09-06 keeps its claim_id; a string from an
    # older run merges without one.
    by_src = {q["claim_source"]: q for q in m["questions"]}
    assert by_src["doc1.md"] == {"claim_source": "doc1.md", "claim_id": 1, "question": "who?"}
    assert by_src["doc9.md"] == {"claim_source": "doc9.md", "question": "who?"}


def test_merge_keeps_who_asked_a_question_source_statement(tmp_path):
    """The report sets a statement the buyer asked apart from the seller's
    claims by `asked_by`; the merge is what hands it on."""
    cite = [{"form": "citation", "document": "d.md", "lines": [1, 1],
             "quote": "x", "shows": "y"}]
    q = _run(tmp_path, "q", "questions/buyer.md",
             [{"id": 1, "quote": "Passwords are hashed", "lines": [2, 2],
               "statement": "Passwords are hashed", "asked_by": "buyer"}],
             [{"claim_id": 1, "adjudication": {"verdict": "contradicted", "gap": "g"},
               "evidence": cite}])
    m = mg.merge([q])
    assert [f.get("asked_by") for f in m["findings"]] == ["buyer"]


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
    fix = {"change": "c", "band": "one_place"}
    ok = {"ratings": [
        {"claim_source": "doc1.md", "claim_id": 1, "materiality": "decisive", "basis": "b", "remedy": fix,
         "in_short": "i"},
        {"claim_source": "doc9.md", "claim_id": 1, "materiality": "material", "basis": "b",
         "remedy": {"change": "", "band": "none"}, "in_short": "i"}],
        "exposures": [
        {"claim_source": "doc9.md", "claim_id": 2, "exposure": "decisive", "basis": "b"}]}
    res = ms.check_ratings(ok, m)
    assert res["ok"], res["problems"]
    assert res["figures"] == {"rateable": 2, "rated": 2,
                              "materiality": {"decisive": 1, "material": 1},
                              "exposable": 1, "exposed": 1,
                              "exposure": {"decisive": 1}}
    bad = {"ratings": [
        {"claim_source": "doc1.md", "claim_id": 1, "materiality": "decisive", "basis": "b",
         "remedy": {"change": "c", "band": "two_weeks"}},
        {"claim_source": "doc1.md", "claim_id": 1, "materiality": "material", "basis": "b"},
        {"claim_source": "doc1.md", "claim_id": 2, "materiality": "material", "basis": ""},
        {"claim_source": "nope.md", "claim_id": 7, "materiality": "material", "basis": "b"},
        # unverifiable rated for materiality: the defect the split exists for
        {"claim_source": "doc9.md", "claim_id": 2, "materiality": "decisive", "basis": "b"}],
        "exposures": [
        {"claim_source": "doc9.md", "claim_id": 1, "exposure": "material", "basis": "b"}]}
    res = ms.check_ratings(bad, m)
    text = "\n".join(res["problems"])
    assert "rated twice" in text
    assert "`real` finding is not rated" in text
    assert "no basis" in text
    assert "no such finding" in text
    assert "finding doc9.md#1 was not rated for materiality" in text
    assert "materiality doc9.md#2: this finding is rated for exposure" in text
    assert "exposure doc9.md#1: this finding is rated for materiality" in text
    assert "finding doc9.md#2 was not rated for exposure" in text
    # a gap carries a remedy whose band is one of MATERIALITY §5a's
    assert "materiality doc1.md#1: band 'two_weeks' is not one of §5a's" in text
    assert "materiality doc1.md#2: no remedy" in text
    assert "materiality doc1.md#1: no in_short (MATERIALITY §5b)" in text


def test_observation_groups_hold_every_observation_exactly_once():
    fix = {"change": "c", "band": "setting"}
    row = lambda members: {"members": members, "materiality": "material", "basis": "b", "remedy": fix,
                           "in_short": "i"}
    assert ms.check_observations({"observations": [row([1, 3]), row([2])]}, 3)["ok"]
    res = ms.check_observations({"observations": [row([1, 2]), row([2, 4])]}, 3)
    assert set(res["problems"]) == {"observation group 2: observation 2 is also in group 1",
                                    "observation group 2: no observation 4",
                                    "observation 3 is in no group"}


def test_a_remedy_survives_combining_its_samples():
    s = [{"claim_source": "a", "claim_id": 1, "materiality": "material", "basis": "x",
          "remedy": {"change": "c", "band": "design"}},
         {"claim_source": "a", "claim_id": 1, "materiality": "material", "basis": "y",
          "remedy": {"change": "d", "band": "setting"}}]
    assert ms.combine("materiality", s)["remedy"] == {"change": "c", "band": "design"}


import pytest  # noqa: E402


@pytest.mark.parametrize("modname", [
    "workflowsv2.audit_materiality.runner",
    "workflowsv2.audit_report.runner",
    "workflowsv2.claims_audit.runner",
    "workflowsv2.claims_audit.continuation",
     "workflowsv2.claims_audit.post_session", "workflowsv2.intake.session",
    "workflowsv2.intake.runner",
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
        "exposures": [
        {"claim_source": "doc9.md", "claim_id": 2, "exposure": "decisive", "basis": "e9"}],
        "figures": {"materiality": {"material": 1, "decisive": 1},
                    "exposure": {"decisive": 1}}}
    md = render(m, ratings)
    assert "Materiality: decisive 1, material 1. Exposure: decisive 1." in md
    rows = [l for l in md.splitlines()
            if l.startswith("| ") and not l.startswith("| materiality")
            and not l.startswith("| exposure")]
    # table 1: what the audit showed, decisive first
    assert rows[0].startswith("| decisive | doc9.md | 1")
    assert rows[1].startswith("| material | doc1.md | 1")
    assert "does_not_hold (evidence_supports)" in rows[1] and "1 problem(s)" in rows[1]
    # table 2: unsettled, with its disposition; never among the materiality rows
    assert rows[2].startswith("| decisive | doc9.md | 2 | unverifiable | not_examined |")
    # table 3: real
    assert rows[3].startswith("| real | doc1.md | 2")
    assert md.index("## What the audit showed") < md.index("## Unsettled claims") \
        < md.index("## Claims that hold")
    text = _rating_text(m["findings"][0])
    assert text.startswith("--- doc1.md claim 1") and "review    : does_not_hold" in text
    assert "citation problems:" in text


def test_load_engagement_returns_thresholds(tmp_path, monkeypatch):
    from workflowsv2.claims_audit import runner as rn
    d = tmp_path / "eng"; d.mkdir()
    (d / "engagement.yaml").write_text(
        "target: t\nclaim_sources: [a.md]\ntransaction: |\n  buys it\n"
        "thresholds: |\n  walks if backups are not daily\n")
    (d / "brief.md").write_text("brief")
    monkeypatch.setattr(rn, "ENGAGEMENTS", tmp_path)
    e = rn.load_engagement("eng")
    assert e["transaction"].strip() == "buys it"
    assert e["thresholds"].strip() == "walks if backups are not daily"
    (d / "engagement.yaml").write_text("target: t\nclaim_sources: [a.md]\n")
    assert rn.load_engagement("eng")["thresholds"] is None
    assert rn.load_engagement("eng")["intake_id"] is None
    # the target defaults to <engagement>/target when it exists there
    (d / "target").mkdir()
    (d / "engagement.yaml").write_text("claim_sources: [a.md]\n")
    assert rn.load_engagement("eng")["target"] == (d / "target").resolve()


def test_load_engagement_prefers_the_intake_blocks(tmp_path, monkeypatch):
    """An intake's blocks.yaml beats engagement.yaml; an explicit intake is
    honoured; an unknown one is refused; an unfinished intake falls back."""
    from workflowsv2.claims_audit import runner as rn
    from workflowsv2 import engagement_state as st
    d = tmp_path / "eng"; d.mkdir()
    (d / "engagement.yaml").write_text(
        "target: t\nclaim_sources: [a.md]\nthresholds: |\n  yaml says\n")
    (d / "brief.md").write_text("brief")
    monkeypatch.setattr(rn, "ENGAGEMENTS", tmp_path)
    a = st.new_intake(d)
    e = rn.load_engagement("eng")
    assert e["intake_id"] == a and e["thresholds"].strip() == "yaml says"   # unfinished → fallback
    (st.intake_dir(d, a) / st.BLOCKS_FILE).write_text(
        "transaction: |\n  buys it\nthresholds: |\n  intake says\n")
    e = rn.load_engagement("eng")
    assert e["thresholds"].strip() == "intake says" and e["transaction"].strip() == "buys it"
    assert rn.load_engagement("eng", intake=a)["intake_id"] == a
    assert rn.load_engagement("eng")["conclusion"] is False
    (st.intake_dir(d, a) / st.BLOCKS_FILE).write_text(
        "thresholds: |\n  intake says\nconclusion: true\n")
    assert rn.load_engagement("eng")["conclusion"] is True
    import pytest
    with pytest.raises(SystemExit):
        rn.load_engagement("eng", intake="nope")


def test_combine_two_agree_majority_and_borderline():
    from workflowsv2.audit_materiality import schemas as ms
    def r(v, b="b"):
        return {"claim_source": "d.md", "claim_id": 1, "materiality": v, "basis": b}
    c = ms.combine("materiality", [r("material", "first"), r("material", "second")])
    assert (c["materiality"], c["agreement"], c["borderline"], c["basis"]) == ("material", "2 of 2", False, "first")
    c = ms.combine("materiality", [r("material"), r("not_material"), r("material"), r("material"), r("material")])
    assert (c["materiality"], c["agreement"], c["borderline"]) == ("material", "4 of 5", False)
    c = ms.combine("materiality", [r("not_material", "n1"), r("material"), r("material"), r("not_material"), r("not_material")])
    assert (c["materiality"], c["agreement"], c["borderline"], c["basis"]) == ("not_material", "3 of 5", True, "n1")
    assert len(c["samples"]) == 5
    c = ms.combine("materiality", [r("material"), r("decisive"), r("material"), r("decisive"), r("not_material")])
    assert (c["materiality"], c["agreement"], c["borderline"]) == ("material", "2 of 5", True)   # tie: sampled first


def test_contested_finds_disagreements_and_one_sided_ratings():
    from workflowsv2.audit_materiality import schemas as ms
    a = [{"claim_source": "d", "claim_id": 1, "exposure": "material"},
         {"claim_source": "d", "claim_id": 2, "exposure": "not_material"}]
    b = [{"claim_source": "d", "claim_id": 1, "exposure": "material"},
         {"claim_source": "d", "claim_id": 2, "exposure": "material"},
         {"claim_source": "d", "claim_id": 3, "exposure": "material"}]
    assert ms.contested("exposure", a, b) == ["d#2", "d#3"]


def test_the_rating_call_is_given_the_reliance_statement_when_there_is_one(monkeypatch):
    from workflowsv2.audit_materiality import runner
    seen = []
    def fake_emit(loop, system, user, schema, max_tokens, salvage=None):
        seen.append(user)
        return {"obj": {"ratings": [], "exposures": []}, "raw": "{}", "parse": "parsed",
                "parse_error": None, "finish": "stop", "attempts": [], "response_format_dropped": []}
    monkeypatch.setattr(runner, "emit", fake_emit)
    monkeypatch.setattr(runner, "_rating_text", lambda f: "finding text")
    f = {"claim_source": "README.md", "claim_id": 1}
    runner.rate(object(), "method", "Client: a buyer", [f], [], 100, 0,
                thresholds="Paying for: privacy", reliance="- Hit count only — depends")
    runner.rate(object(), "method", "Client: a buyer", [f], [], 100, 0, thresholds="Paying for: privacy")
    assert "The reliance statement:\n\n- Hit count only — depends\n\nRate these 1 findings" in seen[0]
    assert "Paying for: privacy" in seen[0] and "The reliance statement" not in seen[1]


def test_ratings_schema_limits_claim_source_to_the_sources_given():
    from workflowsv2.audit_materiality import schemas as ms
    item = ms.ratings_schema(["claim_sources/site/privacy.md"])["properties"]["ratings"]["items"]
    assert item["properties"]["claim_source"] == {"enum": ["claim_sources/site/privacy.md"]}
    assert ms.ratings_schema()["properties"]["exposures"]["items"]["properties"][
        "claim_source"] == {"type": "string"}


def _gaps():
    def f(cid, asked=None):
        return {"claim_source": "a.md", "claim_id": cid, "quote": "q", "lines": [cid, cid],
                "statement": "s", "asked_by": asked,
                "adjudication": {"verdict": "contradicted", "gap": f"gap {cid}"},
                "evidence": [], "review": {"outcome": "holds"}, "citation_problems": []}
    merged = {"findings": [f(1), f(2), f(3, asked="buyer")], "unclaimed": []}
    ratings = {"ratings": [{"claim_source": "a.md", "claim_id": c, "materiality": "material", "basis": "b",
                            "remedy": {"change": "c", "band": "setting"}, "in_short": f"plain {c}"}
                           for c in (1, 2, 3)], "exposures": []}
    return merged, ratings


def _scripted(monkeypatch, answers):
    """runner.emit replaced: each call takes the next answer; returns the prompts."""
    from workflowsv2.audit_materiality import runner as mr
    seen = []
    def fake(loop, method, user, schema, max_tokens):
        seen.append(user)
        return {"obj": answers.pop(0), "parse": "parsed", "response_format_dropped": [], "parse_error": None}
    monkeypatch.setattr(mr, "emit", fake)
    return mr, seen


def test_an_in_short_that_fails_the_check_is_written_again_and_dropped_if_it_fails_twice(monkeypatch):
    merged, ratings = _gaps()
    ok = lambda k: {"key": k, "verdict": "holds", "words": "", "why": ""}
    bad = lambda k, v="adds": {"key": k, "verdict": v, "words": "w", "why": "y"}
    mr, seen = _scripted(monkeypatch, [
        {"checks": [ok("a.md#1"), bad("a.md#2"), bad("a.md#3", "misattributes")]},
        {"rewrites": [{"key": "a.md#2", "in_short": "plain 2, again"},
                      {"key": "a.md#3", "in_short": "plain 3, again"}]},
        {"checks": [ok("a.md#2"), bad("a.md#3")]}])
    res = mr.check_in_short(None, "m", ratings, merged, 100)
    by = {r["claim_id"]: r for r in ratings["ratings"]}
    assert by[1]["in_short"] == "plain 1" and by[2]["in_short"] == "plain 2, again"
    assert "in_short" not in by[3] and by[3]["in_short_dropped"].startswith("failed the check again")
    assert res["rewritten"] == ["a.md#2", "a.md#3"] and res["dropped"] == ["a.md#3"]
    assert ms.check_ratings(ratings, merged)["ok"]          # a dropped account is not a missing one
    # the check is told who made the statement, and only the rewritten are checked again
    assert "the buyer asked this; the seller did not say it" in seen[0]
    assert "a.md#1" not in seen[2]


def test_a_check_that_misses_a_sentence_twice_withdraws_every_in_short(monkeypatch):
    merged, ratings = _gaps()
    partial = {"checks": [{"key": "a.md#1", "verdict": "holds", "words": "", "why": ""}]}
    mr, _ = _scripted(monkeypatch, [partial, partial])
    res = mr.check_in_short(None, "m", ratings, merged, 100)
    assert res["dropped"] == ["a.md#1", "a.md#2", "a.md#3"]
    assert all("in_short" not in r for r in ratings["ratings"])
