"""METHOD §8's candidate rule, and the runner pieces that chase it.

Added 2026-09-02. The ChatterMate GLM run ended with seventy-two
`unverifiable` findings whose searches named files; the adjudication had been
handed 2 of 40 evidence traces, and nothing checked either fact.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.claims_audit import schemas as sch             # noqa: E402
from workflowsv2.claims_audit.runner import (                     # noqa: E402
    chase_message, compact_trace, gathered_evidence, replace_findings)


def _corpus(tmp_path):
    d = tmp_path / "t"
    (d / "app").mkdir(parents=True)
    (d / "app" / "a.py").write_text("x = 1\n")
    (d / "app" / "b.py").write_text("y = 2\n")
    (d / "c.md").write_text("claim\n")
    return d


def _unv(cid, because, candidates):
    return {"claim_id": cid,
            "adjudication": {"verdict": "unverifiable", "unresolved_because": because},
            "evidence": [
                {"form": "search", "kind": "lexical", "performed": "p", "result": "r",
                 "candidates": []},
                {"form": "search", "kind": "structural", "performed": "p", "result": "r",
                 "candidates": candidates}]}


def test_candidate_files_resolves_and_subtracts_what_was_read(tmp_path):
    docs = sch.corpus_index(_corpus(tmp_path))
    fs = [_unv(1, "not_examined", ["app/a.py", "b.py", "nope.py"]),
          {"claim_id": 2, "adjudication": {"verdict": "real"}, "evidence": []}]
    out = sch.candidate_files(fs, docs, read={"app/a.py"})
    assert set(out) == {1}
    assert out[1]["named"] == ["app/a.py", "app/b.py"]
    assert out[1]["unopened"] == ["app/b.py"]
    assert out[1]["unresolved"] == ["document 'nope.py' is not in the materials"]
    # no read set: nothing is called unopened
    assert sch.candidate_files(fs, docs, read=None)[1]["unopened"] == []
    # a directory of the corpus is recorded, not an obligation or a problem;
    # a dotfile resolves (lstrip once ate the dot)
    (tmp_path / "t" / ".gitmodules").write_text("[submodule]\n")
    docs = sch.corpus_index(tmp_path / "t")
    out = sch.candidate_files([_unv(1, "not_examined", ["app/", "./.gitmodules", "nope/"])],
                              docs, read=set())
    assert out[1]["directories"] == ["app"]
    assert out[1]["named"] == [".gitmodules"] and out[1]["unopened"] == [".gitmodules"]
    assert out[1]["unresolved"] == ["document 'nope/' is not in the materials"]


def test_check_output_enforces_the_disposition_against_the_read_set(tmp_path):
    corpus = _corpus(tmp_path)
    frozen = [{"id": 1}, {"id": 2}, {"id": 3}]
    obj = {"findings": [
        _unv(1, "not_in_the_materials", ["app/a.py"]),   # named, unopened: wrong
        _unv(2, "not_examined", ["app/b.py"]),           # named, opened: wrong
        _unv(3, "not_examined", ["app/a.py"])]}          # named, unopened: right
    res = sch.check_output(obj, corpus, "c.md", frozen, read={"app/b.py"})
    text = "\n".join(res["problems"])
    assert "finding 1: searches named app/a.py and the run did not open it" in text
    assert "finding 2: `not_examined` but every candidate" in text
    assert "finding 3" not in text
    assert res["figures"]["not_examined"] == 2
    assert res["figures"]["unopened_candidates"] == ["app/a.py"]
    # without a read set the dispositions are taken as recorded
    res = sch.check_output(obj, corpus, "c.md", frozen)
    assert not [p for p in res["problems"] if "not_examined" in p or "did not open" in p]


def _trace(query, body, answer):
    return ("=" * 80 + "\n[inspect_external] t exit=respond iters=1\n" + "=" * 80
            + f"\nQuery: {query}\n\n--- iter 1 ---\nACTION:\n{{}}\nOBSERVATION:\n"
            + body + "\n\nFINAL ANSWER:\n" + answer)


def test_compact_trace_keeps_query_and_answer():
    t = _trace("read a.py", "1|x = 1\n" * 50, "a.py:1 says x = 1")
    c = compact_trace(t)
    assert c.startswith("=" * 80) and "Query: read a.py" in c
    assert "FINAL ANSWER:\na.py:1 says x = 1" in c and "OBSERVATION" not in c
    assert compact_trace("no answer marker") == "no answer marker"


def test_gathered_evidence_represents_every_request(tmp_path):
    d = tmp_path / "traces"
    d.mkdir()
    big = "1|x\n" * 2000
    for i in range(3):
        (d / f"inspect_external_2026-09-02T00-00-0{i}Z.txt").write_text(
            _trace(f"q{i}", big, f"answer {i}"))
    full = gathered_evidence(d, budget=10 ** 6)
    assert full["form"] == "full" and full["included"] == 3 and full["omitted"] == 0
    assert "OBSERVATION" in full["text"]
    compact = gathered_evidence(d, budget=5000)
    assert compact["form"] == "compact" and compact["included"] == 3
    assert compact["omitted"] == 0
    assert all(f"answer {i}" in compact["text"] for i in range(3))
    assert "OBSERVATION" not in compact["text"]
    cut = gathered_evidence(d, budget=400)
    assert cut["form"] == "compact" and cut["included"] < 3 and cut["omitted"] >= 1


def test_chase_message_names_claims_and_files():
    frozen = [{"id": 2, "quote": "Handoff to a human"}, {"id": 5, "quote": "Widget themes"}]
    msg = chase_message({5: ["frontend/widget.ts"], 2: ["app/agents/transfer.py",
                                                        "app/services/routing.py"]}, frozen)
    assert msg.index("claim 2. Handoff to a human") < msg.index("claim 5. Widget themes")
    assert "      app/agents/transfer.py\n      app/services/routing.py" in msg
    assert "Do not write findings" in msg


def test_replace_findings_swaps_only_the_wanted_claims():
    obj = {"findings": [{"claim_id": 1, "adjudication": {"verdict": "unverifiable"}},
                        {"claim_id": 2, "adjudication": {"verdict": "real"}},
                        {"claim_id": 3, "adjudication": {"verdict": "unverifiable"}}]}
    again = {"obj": {"findings": [
        {"claim_id": 1, "adjudication": {"verdict": "contradicted", "gap": "g"}},
        {"claim_id": 2, "adjudication": {"verdict": "partial", "gap": "no"}},
        {"claim_id": 9, "adjudication": {"verdict": "real"}}]}}
    n = replace_findings(obj, again, wanted={1, 3})
    assert n == 1
    assert [f["adjudication"]["verdict"] for f in obj["findings"]] == \
        ["contradicted", "real", "unverifiable"]
    assert len(obj["findings"]) == 3
