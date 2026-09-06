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
    _merge_emissions, chase_message, compact_trace, evidence_batches,
    gathered_evidence, previous_adjudications, replace_findings, trace_claims,
    untagged_message)


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
    # a claim of absence rests on its searches (METHOD §8, 2026-09-05): its
    # candidates are the same obligation whatever the verdict
    real = dict(_unv(3, None, ["app/b.py"]), adjudication={"verdict": "real"})
    assert sch.candidate_files([real], docs, read={"app/a.py"})[3]["unopened"] == ["app/b.py"]
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
    # a `real` resting on searches alone: both kinds, every candidate opened
    real = dict(_unv(4, None, ["app/a.py"]), adjudication={"verdict": "real"})
    res = sch.check_output({"findings": [real]}, corpus, "c.md", [{"id": 4}], read={"app/b.py"})
    assert "verdict 'real' rests on searches that named app/a.py" in "\n".join(res["problems"])
    real["evidence"] = real["evidence"][:1]          # lexical only
    res = sch.check_output({"findings": [real]}, corpus, "c.md", [{"id": 4}], read={"app/a.py"})
    assert "missing structural" in "\n".join(res["problems"])
    cited = dict(real, evidence=[{"form": "citation", "document": "app/a.py", "lines": [1, 1],
                                  "quote": "x = 1", "shows": "s"}])
    assert sch.check_output({"findings": [cited]}, corpus, "c.md", [{"id": 4}], read=set())["ok"]
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
    files = sorted(d.glob("*.txt"))
    full = gathered_evidence(files, budget=10 ** 6)
    assert full["form"] == "full" and full["included"] == 3 and full["omitted"] == 0
    assert "OBSERVATION" in full["text"]
    compact = gathered_evidence(files, budget=5000)
    assert compact["form"] == "compact" and compact["included"] == 3
    assert compact["omitted"] == 0
    assert all(f"answer {i}" in compact["text"] for i in range(3))
    assert "OBSERVATION" not in compact["text"]
    cut = gathered_evidence(files, budget=400)
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


def test_replace_findings_keeps_the_readjudication_questions_and_incompletion():
    obj = {"findings": [{"claim_id": 1, "adjudication": {"verdict": "unverifiable"}}],
           "questions": ["q0"], "unclaimed": [{"note": "u0", "evidence": {}}]}
    again = {"obj": {"findings": [], "questions": ["q1"],
                     "unclaimed": [{"note": "u1", "evidence": {}}],
                     "not_completed": "batch could not be read"}}
    assert replace_findings(obj, again, wanted={1}) == 0
    assert obj["questions"] == ["q0", "q1"]
    assert [u["note"] for u in obj["unclaimed"]] == ["u0", "u1"]
    assert obj["not_completed"] == "batch could not be read"


def test_previous_adjudications_shows_verdict_and_disposition():
    obj = {"findings": [
        {"claim_id": 1, "adjudication": {"verdict": "unverifiable",
                                         "unresolved_because": "not_examined"}},
        {"claim_id": 2, "adjudication": {"verdict": "partial", "gap": "g"}}]}
    text = previous_adjudications(obj, [1, 2, 3])
    assert "claim 1: verdict unverifiable; unresolved_because: not_examined" in text
    assert "claim 2: verdict partial; gap: g" in text
    assert "claim 3: verdict None" in text


def test_merge_emissions_carries_not_completed_from_any_batch():
    def part(obj):
        return {"obj": obj, "raw": "", "parse": "parsed", "finish": "stop",
                "attempts": [], "response_format_dropped": [], "evidence": {}}
    first = part({"claim_source": "d.md", "findings": [{"claim_id": 1}]})
    second = part({"claim_source": "d.md", "findings": [],
                   "not_completed": "batch 2 unreadable"})
    merged = _merge_emissions([first, second])["obj"]
    assert merged["not_completed"] == "batch 2 unreadable"
    assert [f["claim_id"] for f in merged["findings"]] == [1]
    clean = _merge_emissions([first, part({"claim_source": "d.md",
                                           "findings": [{"claim_id": 2}]})])
    assert "not_completed" not in clean["obj"] and clean["batch_defects"] == []
    # A batch that sets the field beside its findings has misused it
    # (METHOD §13): reported as a defect, its text not carried.
    misused = part({"claim_source": "d.md", "findings": [{"claim_id": 2}],
                    "not_completed": "All claims were attempted."})
    m = _merge_emissions([first, misused])
    assert "not_completed" not in m["obj"]
    assert len(m["batch_defects"]) == 1 and m["batch_defects"][0].startswith("batch 2")
    alone = _merge_emissions([misused])
    assert "not_completed" not in alone["obj"] and len(alone["batch_defects"]) == 1
    assert [f["claim_id"] for f in alone["obj"]["findings"]] == [2]


def test_trace_claims_reads_the_prefix():
    assert trace_claims("=" * 80 + "\n[x]\n" + "=" * 80
                        + "\nQuery: [claims 5, 2, 5] read a.py\n") == [2, 5]
    assert trace_claims("Query: read a.py\n") == []


def test_evidence_batches_walk_claims_in_order_within_size_and_budget():
    from pathlib import PurePosixPath as P
    index = {P("t1"): {"claims": [1, 2], "chars": 500, "trimmed": 100},
             P("t2"): {"claims": [2, 3], "chars": 500, "trimmed": 100},
             P("t3"): {"claims": [4], "chars": 500, "trimmed": 100},
             P("t4"): {"claims": [], "chars": 500, "trimmed": 100}}
    b = evidence_batches([1, 2, 3, 4, 5], index, batch=10, budget=10_000)
    assert [x["claims"] for x in b] == [[1, 2, 3, 4, 5]]
    assert b[0]["traces"] == [P("t1"), P("t2"), P("t3")]
    assert b[0]["untagged"] == [5]
    # the budget closes a batch before the next claim's traces overflow it;
    # sized on the FULL form, so trimming stays the exception
    b = evidence_batches([1, 2, 3, 4], index, batch=10, budget=1_250)
    assert [x["claims"] for x in b] == [[1, 2, 3], [4]]
    b = evidence_batches([1, 2, 3, 4], index, batch=10, budget=250)
    assert [x["claims"] for x in b] == [[1], [2], [3], [4]]
    # the size cap closes it too
    b = evidence_batches([1, 2, 3, 4], index, batch=2, budget=10_000)
    assert [x["claims"] for x in b] == [[1, 2], [3, 4]]
    # a claim over budget alone is a batch of one, never dropped
    b = evidence_batches([1, 2], index, batch=10, budget=400)
    assert [x["claims"] for x in b] == [[1], [2]]
    assert b[0]["traces"] == [P("t1")]


TRACE = ("=" * 80 + "\n[inspect_external] t exit=respond iters=3\n" + "=" * 80
         + "\nQuery: [claims 3] where is the guard\n\n"
         "--- iter 1 ---\nACTION:\n{\n  \"thought\": \"look\",\n  \"tool\": \"grep\",\n  \"pattern\": \"guard\"\n}\n"
         "OBSERVATION:\nOK: app/g.py:40: def guard():\napp/z.py:7: guard = None\n\n"
         "--- iter 2 ---\nACTION:\n{\n  \"tool\": \"read\",\n  \"file\": \"app/g.py\"\n}\n"
         "OBSERVATION:\nOK: 1|import x\n" + "".join(f"{n}|line {n}\n" for n in range(2, 101))
         + "\n--- iter 3 ---\nACTION:\n{\n  \"tool\": \"read\",\n  \"file\": \"app/h.py\"\n}\n"
         "OBSERVATION:\nOK: 1|nothing cited\n2|here\n\n"
         "FINAL ANSWER:\nThe guard is at g.py:40-42 and app/g.py:90.")


def test_trim_trace_keeps_cited_lines_with_a_band_and_grep_whole():
    from workflowsv2.claims_audit.runner import trim_trace
    out = trim_trace(TRACE, band=3)
    assert "Query: [claims 3] where is the guard" in out
    assert "app/g.py:40: def guard():" in out              # a hit in a cited file
    assert "app/z.py:7" not in out and "1 hit(s) in files the answer does not cite" in out
    assert '"thought"' not in out
    kept = [int(l.split("|")[0]) for l in out.splitlines() if "|" in l and l.split("|")[0].isdigit()]
    assert kept == list(range(37, 46)) + list(range(87, 94))
    assert "… 41 line(s) not cited" in out                 # the gap between ranges
    assert "line(s) of app/g.py not cited, not shown" in out
    assert "read app/h.py; the answer cites no line of it" in out
    assert out.endswith("FINAL ANSWER:\nThe guard is at g.py:40-42 and app/g.py:90.")
    assert len(out) < len(TRACE)
    assert trim_trace("no answer") == "no answer"


def test_gathered_evidence_tries_full_then_trimmed_then_compact(tmp_path):
    f = tmp_path / "inspect_external_2026-09-02T00-00-00Z.txt"
    f.write_text(TRACE)
    full = gathered_evidence([f], budget=10 ** 6)
    assert full["form"] == "full" and "99|line 99" in full["text"]
    trimmed = gathered_evidence([f], budget=len(TRACE) - 1)
    assert trimmed["form"] == "trimmed" and "42|line 42" in trimmed["text"]
    assert "60|line 60" not in trimmed["text"]   # outside the band of 40-42 and 90
    compact = gathered_evidence([f], budget=400)
    assert compact["form"] == "compact" and "OBSERVATION" not in compact["text"]


def test_untagged_message_names_each_claim_with_its_statement():
    frozen = [{"id": 4, "quote": "Docker Pulls", "statement": "Images are pulled.", "lines": [4, 4]},
              {"id": 9, "quote": "Shopify support", "statement": "Shopify is supported.", "lines": [9, 9]}]
    msg = untagged_message([9, 4], frozen)
    assert "No evidence request has been filed" in msg
    assert msg.index("9. [[9, 9]] Shopify support") < msg.index("4. [[4, 4]] Docker Pulls")
    assert "statement: Shopify is supported." in msg and "`claims` field" in msg


def test_files_matched_reads_search_hits_not_reads(tmp_path):
    """A grep observation's `path:line:` lines name files the auditor saw;
    a read's `NN|` lines and a listing do not count; a path that is not in
    the target is dropped."""
    from workflowsv2.claims_audit.runner import files_matched, files_read
    tgt = tmp_path / "t"; (tgt / ".github").mkdir(parents=True)
    (tgt / "a.py").write_text("x\n"); (tgt / ".github" / "ci.yml").write_text("on: push\n")
    tr = tmp_path / "traces"; tr.mkdir()
    (tr / "inspect_external_1.txt").write_text(
        'ACTION:\n{"tool": "grep", "pattern": "x"}\nOBSERVATION:\n'
        'OK: a.py:1:x\n./.github/ci.yml:1:on: push\nnope.py:3:zzz\n'
        'ACTION:\n{"tool": "read", "file": "a.py"}\nOBSERVATION:\nOK: 1|x\n'
        'ACTION:\n{"tool": "list"}\nOBSERVATION:\nOK: ./\na.py\t2 bytes\n')
    assert files_matched(tr, tgt) == [".github/ci.yml", "a.py"]
    assert set(files_read(tr, tgt)) == {"a.py"}


def test_a_candidate_in_a_submodule_is_outside_not_unresolved(tmp_path):
    from workflowsv2.claims_audit import schemas as sch
    docs = sch.corpus_index(_corpus(tmp_path))
    fs = [_unv(1, "outside_the_materials", ["backend/app/enterprise", "app/a.py"]),
          _unv(2, "not_in_the_materials", ["backend/app/enterprise/x.py"])]
    out = sch.candidate_files(fs, docs, read={"app/a.py"},
                              submodules=["backend/app/enterprise"])
    assert out[1]["outside"] == ["backend/app/enterprise"] and out[1]["unresolved"] == []
    assert out[2]["outside"] == ["backend/app/enterprise/x.py"]
    # without the submodule list the same candidate is unresolved, as before
    assert sch.candidate_files(fs[:1], docs, read=set())[1]["unresolved"]


def test_check_output_requires_a_question_to_name_a_frozen_claim(tmp_path):
    corpus = _corpus(tmp_path)
    real = {"claim_id": 4, "adjudication": {"verdict": "real"},
            "evidence": [{"form": "citation", "document": "app/a.py",
                          "lines": [1, 1], "quote": "x = 1", "shows": "s"}]}
    obj = {"findings": [real],
           "questions": [{"claim_id": 4, "question": "since when?"},
                         {"claim_id": 9, "question": "orphan"},
                         "a bare string"]}
    res = sch.check_output(obj, corpus, "c.md", [{"id": 4}], read={"app/a.py"})
    qs = [p for p in res["problems"] if p.startswith("question")]
    assert len(qs) == 2 and qs[0].startswith("question 2") and qs[1].startswith("question 3")


def test_a_seller_claim_needs_no_searches_to_be_unverifiable(tmp_path):
    corpus = _corpus(tmp_path)
    bare = {"claim_id": 5,
            "adjudication": {"verdict": "unverifiable",
                             "unresolved_because": "outside_the_materials"},
            "evidence": [{"form": "citation", "document": "app/a.py",
                          "lines": [1, 1], "quote": "x = 1", "shows": "s"}]}
    res = sch.check_output({"findings": [bare]}, corpus, "c.md",
                           [{"id": 5, "about": "seller"}], read={"app/a.py"})
    assert not any("needs a lexical" in p for p in res["problems"])
    res = sch.check_output({"findings": [bare]}, corpus, "c.md",
                           [{"id": 5, "about": "target"}], read={"app/a.py"})
    assert any("needs a lexical" in p for p in res["problems"])


def test_a_binary_candidate_is_present_but_not_readable(tmp_path):
    corpus = _corpus(tmp_path)
    (corpus / "wordpress").mkdir()
    (corpus / "wordpress" / "plugin.zip").write_bytes(b"PK\x03\x04\x00\x00binary")
    docs = sch.corpus_index(corpus)
    view = sch.corpus_view(corpus)
    assert "wordpress/plugin.zip" in view["binary_skipped"]
    f = _unv(7, "outside_the_materials", ["wordpress/plugin.zip", "wordpress"])
    cands = sch.candidate_files([f], docs, {"app/a.py"}, view["submodules"], view["binary_skipped"])
    c = cands[7]
    assert c["unreadable"] == ["wordpress/plugin.zip"] and c["unresolved"] == []
    assert c["directories"] == ["wordpress"] and c["unopened"] == []
    res = sch.check_output({"findings": [f]}, corpus, "c.md", [{"id": 7}], read={"app/a.py"})
    assert any("present_but_not_readable" in p for p in res["problems"])
    assert not any("not in the materials" in p for p in res["problems"])
    ok = _unv(7, "present_but_not_readable", ["wordpress/plugin.zip"])
    res = sch.check_output({"findings": [ok]}, corpus, "c.md", [{"id": 7}], read={"app/a.py"})
    assert not any("candidate" in p for p in res["problems"])
