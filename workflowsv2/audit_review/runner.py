#!/usr/bin/env python3
"""Review one finished claims audit against the evidence it cites.

    python3 workflowsv2/audit_review/runner.py --run <audit run directory>
    python3 workflowsv2/audit_review/runner.py --run <dir> \
            --model measure/models/grok_4p6.yaml

A review has three parts, in this order:

  1. STATISTICS. Every mechanical property of the audit's output, recomputed
     here from `claims.json`, `findings.json` and the materials. Written to
     `review/statistics.json` for the reviewer to read.
  2. GATHER. The reviewer opens the documents it needs, for as long as it
     keeps yielding.
  3. EMIT. Schema-constrained calls producing the observations REVIEW.md §6
     defines. The outcome — whether each finding holds — is derived from those
     observations here, not written by the reviewer.

Then the retest: the adverse observations that qualify, plus a sample of
findings that hold, given to a second reviewer that has not seen the first.

WHY THE STATISTICS ARE RECOMPUTED AND NOT READ FROM `run_meta.json`. The audit
records its own checks there. A review that reads them is checking the audit's
account of itself, which is the one thing it must not do. Same code, run over
the artifacts by a different process, is not the same as trusting a figure.

WHAT IS MECHANICAL HERE AND WHAT IS NOT. Typed output moved most of the old
review into arithmetic: whether a document exists, whether a line range is real,
whether a quote appears where it says. All of that is settled before the
reviewer starts, and REVIEW.md §4 tells it so. What is left is the five
judgements in §5, of which evidence RELEVANCE is the one nothing mechanical can
reach — a citation can resolve, quote the document exactly, and be about
something else.

ONE REVIEW PER RUN. The runner refuses if `<run>/review/` already holds a
review: a reviewer whose `inspect` can see a previous one is not independent
of it.

CHOOSE THE REVIEWER FOR QUALITY, NOT FOR DISTANCE FROM THE AUDITOR. Bruce's
call, 2026-08-29: the intra-model effect is much smaller than the difference
between a strong reviewer and a weak one, so a model reviewing its own audit is
still worth more than a weaker model's independent review. Run the best
available and say which it was.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import random
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

from chat.workflow import load_workflow                        # noqa: E402
from utils.json_utils import repair_json_string                # noqa: E402
from workflowsv2 import issues                                 # noqa: E402
from workflowsv2.audit_review import schemas                   # noqa: E402
from workflowsv2.claims_audit import schemas as audit_schemas  # noqa: E402
from workflowsv2.turns import last_exit_reason, latest_reply   # noqa: E402

SCENARIO = HERE / "scenario.yaml"
REVIEW_PATH = "workflowsv2/audit_review/method/REVIEW.md"
SOURCE = "User"
CONTINUE = "continue"

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("audit_review.run")
logger.setLevel(logging.INFO)


# ---------------------------------------------------------------------------
# Part one: the statistics
# ---------------------------------------------------------------------------

def unclaimed_spans(claims: Sequence[Dict[str, Any]],
                    src: Path) -> List[Dict[str, Any]]:
    """Lines of the claim source no claim covers.

    FOR A PERSON, NOT FOR THE REVIEWER. REVIEW.md §7: judging whether an
    unenumerated sentence contains a seller assertion is the enumeration task
    over again, and a model that missed it once will miss it twice. This
    reports where to look; a human decides.

    Blank lines and markdown headings are dropped — an unclaimed heading is
    not a missed claim, and leaving them in buries the real gaps.
    """
    if not src.is_file():
        return []
    lines = src.read_text(encoding="utf-8", errors="replace").splitlines()
    covered = set()
    for c in claims:
        ln = c.get("lines")
        if isinstance(ln, list) and len(ln) == 2 \
                and all(isinstance(n, int) for n in ln):
            covered.update(range(max(1, ln[0]), min(len(lines), ln[1]) + 1))
    out, run_start = [], None
    for n in range(1, len(lines) + 2):
        text = lines[n - 1].strip() if n <= len(lines) else ""
        skip = n > len(lines) or n in covered or not text \
            or text.startswith("#") or set(text) <= set("*-_= ")
        if skip:
            if run_start is not None:
                out.append({"lines": [run_start, n - 1],
                            "text": "\n".join(lines[run_start - 1:n - 1])})
                run_start = None
        elif run_start is None:
            run_start = n
    return out


def statistics(run: Path, corpus: Path, claim_source: str) -> Dict[str, Any]:
    """Every mechanical property of the audit's output, recomputed here."""
    claims = json.loads((run / "claims.json").read_text())
    findings = json.loads((run / "findings.json").read_text())
    frozen = claims.get("claims") or []
    surface = audit_schemas.check_surface(claims, corpus, claim_source)
    output = audit_schemas.check_output(findings, corpus, claim_source, frozen)

    docs = audit_schemas.corpus_index(corpus)
    ev_per, cited_docs, no_citation = [], set(), []
    cited_by: Dict[str, List[Any]] = {}
    for f in findings.get("findings") or []:
        ev = f.get("evidence") or []
        ev_per.append(len(ev))
        cites = [e for e in ev if e.get("form") == "citation"]
        for e in cites:
            key, _ = audit_schemas.resolve_document(docs, e.get("document"))
            name = key or e.get("document")
            cited_docs.add(name)
            cited_by.setdefault(name, []).append(f.get("claim_id"))
        if not cites and (f.get("adjudication") or {}).get("verdict") \
                != "unverifiable":
            no_citation.append(f.get("claim_id"))
    ev_per.sort()
    corpus_docs = set(docs)
    # THE RECORD CHECK'S ARITHMETIC. REVIEW.md §7 asks whether the record
    # shows each cited document being opened. `files_read` in run_meta.json is
    # that record, derived from the evidence traces by the audit runner; a
    # cited document absent from it was never opened by any evidence request.
    # Computed here, not asked of the reviewer, because it is a set difference.
    meta = json.loads((run / "run_meta.json").read_text()) \
        if (run / "run_meta.json").is_file() else {}
    opened = sorted((meta.get("files_read") or {}).keys())
    never_opened = {d: sorted(set(cited_by[d]), key=lambda x: (x is None, x))
                    for d in sorted(cited_docs - set(opened)) if d}

    verdicts = (output.get("figures") or {}).get("verdicts") or {}
    return {
        "claim_source": claim_source,
        "surface_check": surface,
        "output_check": output,
        # THE AUTHORITATIVE FIGURES. Bruce's call: the review reports, the
        # audit does not score itself. Every count here is recomputed from
        # claims.json, findings.json and the materials — never read from the
        # audit's run_meta.json — and `adverse` uses the one definition, in
        # claims_audit/schemas.py, rather than a rule the reader has to infer.
        "counts": {
            "claims": len(frozen),
            "findings": len(findings.get("findings") or []),
            "verdicts": verdicts,
            "adverse_findings": sum(verdicts.get(v, 0)
                                    for v in audit_schemas.ADVERSE_VERDICTS),
            "adverse_verdicts": list(audit_schemas.ADVERSE_VERDICTS),
            # Reported beside `adverse_findings`, never inside it: an
            # unsettled claim is a limit of the materials, not a defect in
            # the target. A reader who wants to weigh it can.
            "unverifiable": verdicts.get("unverifiable", 0)},
        "evidence_per_finding": {
            "min": ev_per[0] if ev_per else 0,
            "median": ev_per[len(ev_per) // 2] if ev_per else 0,
            "max": ev_per[-1] if ev_per else 0},
        # A finding resting on no citation, whose verdict is not
        # `unverifiable`, has asserted an outcome from nothing readable.
        "findings_without_a_citation": no_citation,
        "documents_cited": sorted(d for d in cited_docs if d),
        "documents_never_cited": sorted(corpus_docs - cited_docs),
        # The audit's own record of what it opened, and the cited documents
        # that record does not show being opened, each with the claims whose
        # findings cite it. REVIEW.md §7.
        "documents_opened": opened,
        "documents_cited_but_never_opened": never_opened,
        # For a person, per REVIEW.md §7.
        "unclaimed_spans": unclaimed_spans(frozen, corpus / claim_source),
    }


# ---------------------------------------------------------------------------
# The reviewer
# ---------------------------------------------------------------------------

def build_config(run: Path, world: str, model_path: Optional[Path],
                 target: Path) -> Tuple[str, Dict[str, Any]]:
    """Same shape and the same reasons as the audit runner's: the model config
    REPLACES `llm_config` rather than merging into it, so a stale field from
    the scenario cannot survive into a model that never declared one."""
    from launcher import parse_characters                      # noqa: E402
    scenario = yaml.safe_load(SCENARIO.read_text(encoding="utf-8")) or {}
    scen_llm = dict(scenario.get("llm_config") or {})
    if model_path:
        doc = yaml.safe_load(Path(model_path).read_text(encoding="utf-8")) or {}
        llm = dict(doc.get("llm_config") or {})
        if not llm:
            raise SystemExit(f"{model_path}: no llm_config block")
        for ch in (scenario.get("characters") or {}).values():
            if isinstance(ch, dict) and ch.get("mode") == "chat":
                ch["llm_config"] = dict(llm)
        scen_llm.update(llm)
    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world
    chars = parse_characters(scenario, scen_llm, world_cfg,
                             scenario.get("setting", ""),
                             scenario.get("alt_llm_config") or {})
    chat = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat) != 1:
        raise SystemExit(f"expected 1 chat character, found {len(chat)}")
    name, cfg = chat[0]
    cfg["autonomy_enabled"] = False
    cfg["external_repo"] = str(target)
    cfg["inspect_repo"] = str(run)
    return name, cfg


def _emit(loop, system: str, user: str, schema: Dict[str, Any],
          max_tokens: int) -> Dict[str, Any]:
    """One schema-constrained call, parsed, with what could have degraded it.

    The audit runner carries the same function for the same reasons: an action
    payload is unconstrained by design, so the emission answers under its own
    schema outside the ReAct loop; and an empty completion with finish=stop is
    frequent enough on the local route to need a retry that `react.py` has and
    a bare call does not.
    """
    before = set(getattr(loop.backend, "_param_drops", set()))
    messages = [{"role": "system", "content": system},
                {"role": "user", "content": user}]
    attempts, raw = [], ""
    for _ in range(2):
        raw = loop.backend.chat(messages, max_tokens=max_tokens,
                                response_schema=schema)
        attempts.append({
            "chars": len(raw or ""),
            "finish": getattr(loop.backend, "last_finish_reason", None),
            "reasoning_chars": getattr(loop.backend,
                                       "last_reasoning_chars", None)})
        if (raw or "").strip():
            break
    dropped = sorted(set(getattr(loop.backend, "_param_drops", set())) - before)
    obj, how, err = None, None, None
    try:
        obj, how = json.loads(raw), "parsed"
    except Exception as e:                                     # noqa: BLE001
        err = f"{type(e).__name__}: {e}"
        repaired = repair_json_string(raw)
        obj, how = (repaired, "repaired") if isinstance(repaired, dict) \
            else (None, "unparseable")
    return {"raw": raw, "obj": obj, "parse": how, "parse_error": err,
            "attempts": attempts, "response_format_dropped": dropped}


def _finding_text(f: Dict[str, Any], claims: Dict[int, Dict[str, Any]]) -> str:
    """One finding, and the claim it adjudicates, as the reviewer reads it."""
    c = claims.get(f.get("claim_id")) or {}
    adj = f.get("adjudication") or {}
    out = [f"--- claim {f.get('claim_id')}",
           f"    quote     : {c.get('quote')}",
           f"    lines     : {c.get('lines')}",
           f"    statement : {c.get('statement')}",
           f"    verdict   : {adj.get('verdict')}"]
    if adj.get("gap"):
        out.append(f"    gap       : {adj['gap']}")
    if adj.get("unresolved_because"):
        out.append(f"    unresolved_because: {adj['unresolved_because']}")
    for i, e in enumerate(f.get("evidence") or [], 1):
        out.append(f"    evidence {i} ({e.get('form')}): "
                   + json.dumps({k: v for k, v in e.items() if k != "form"},
                                ensure_ascii=False)[:1500])
    return "\n".join(out)


#: Lines shown on each side of a cited range. Enough to see a heading, the
#: sentence before and the sentence after; not enough to hand over the file.
CONTEXT_LINES = 8
#: Characters of cited material per finding. A finding with many citations
#: gets its first ones in full and a note that the rest were cut.
MATERIALS_BUDGET = 8_000


def _cited_materials(f: Dict[str, Any], docs: Dict[str, List[str]],
                     context: int = CONTEXT_LINES,
                     budget: int = MATERIALS_BUDGET) -> str:
    """The lines each citation of one finding points at, with context.

    WHY THE EMISSION NEEDS THIS. `_emit` is a two-message call with no tools
    and no history: nothing the reviewer opened in its reading legs reaches it.
    Without this, REVIEW.md §5 checks 2 and 3 were judged from the auditor's
    own `quote` and `shows`, and both v2 reviews on 2026-09-01 passed every
    finding — including two whose quote appears nowhere in the document cited.
    This is the review-side half of what the scenario promises: citations are
    resolved before the review starts, and the reviewer's judgement is spent
    on what the resolved lines mean.

    A citation that does not resolve is shown as such, with the reason, so the
    reviewer sees the same fact the statistics record.
    """
    refs: List[Tuple[str, Any, Any]] = []
    for e in f.get("evidence") or []:
        if not isinstance(e, dict):
            continue
        if e.get("form") == "citation":
            refs.append((e.get("document"), e.get("lines"), e.get("quote")))
        elif e.get("form") == "derived":
            for b in e.get("basis") or []:
                if isinstance(b, dict):
                    refs.append((b.get("document"), b.get("lines"),
                                 b.get("quote")))
    out, used = [], 0
    for doc, lines, _quote in refs:
        key, why = audit_schemas.resolve_document(docs, doc)
        if key is None:
            block = f"    [{doc}:{lines}] does not resolve: {why}"
        elif (not isinstance(lines, list) or len(lines) != 2
              or not all(isinstance(n, int) for n in lines)
              or lines[0] < 1 or lines[1] < lines[0]):
            block = f"    [{doc}:{lines}] does not resolve: not a line range"
        elif lines[1] > len(docs[key]):
            block = (f"    [{key}:{lines[0]}-{lines[1]}] does not resolve: "
                     f"the file has {len(docs[key])} lines")
        else:
            body = docs[key]
            lo, hi = lines
            a, b = max(1, lo - context), min(len(body), hi + context)
            rows = [f"    {n:>5}{'>' if lo <= n <= hi else ' '}|{body[n - 1]}"
                    for n in range(a, b + 1)]
            block = (f"    [{key}:{lo}-{hi}] lines {a}-{b}, the cited range "
                     f"marked with >\n" + "\n".join(rows))
        if used + len(block) > budget:
            out.append(f"    [{len(refs) - len(out)} further citation(s) not "
                       f"shown: over the materials budget]")
            break
        out.append(block)
        used += len(block)
    return "\n".join(out)


def emit_parts(loop, method_text: str, stats: Dict[str, Any],
               frozen: Sequence[Dict[str, Any]],
               findings: Sequence[Dict[str, Any]],
               max_tokens: int, batch: int = 0,
               docs: Optional[Dict[str, List[str]]] = None) -> Dict[str, Any]:
    """The review, in one call or in batches.

    BATCHING IS A RUNNER DECISION, NOT A SCHEMA CHANGE. `schemas.py` carries
    one schema per part precisely so a batch is the same call over ten items
    instead of forty. `batch=0` asks for everything at once, which is the
    default because nothing has yet truncated; a review of two hundred
    findings is what the flag is for.
    """
    claims_by_id = {c.get("id"): c for c in frozen}
    docs = docs or {}
    facts = json.dumps({k: v for k, v in stats.items()
                        if k != "unclaimed_spans"},
                       ensure_ascii=False, indent=1)[:20_000]
    head = ("The audit under review is in the run directory. Its claim surface "
            "and findings follow, and the statistics the client's process "
            "computed are:\n\n" + facts + "\n\n")
    parts, calls = [], []

    def ask(what: str, body: str, schema) -> None:
        out = _emit(loop, method_text, head + body, schema, max_tokens)
        calls.append({"part": what,
                      **{k: v for k, v in out.items() if k not in ("raw", "obj")}})
        if out["obj"] is not None:
            parts.append(out["obj"])
        else:
            logger.warning("%s did not parse: %s", what, out["parse_error"])

    groups = ([list(frozen)] if not batch
              else [list(frozen)[i:i + batch]
                    for i in range(0, len(frozen), batch)])
    for n, g in enumerate(groups, 1):
        body = ("Check the fidelity of these claims, per REVIEW §5 check 1 — "
                "does each `statement` faithfully render its `quote`?\n\n"
                + json.dumps(g, ensure_ascii=False, indent=1)
                + "\n\nEmit `claim_checks` for exactly these claims.")
        ask(f"claim_checks[{n}/{len(groups)}]", body,
            schemas.claim_checks_schema())

    groups = ([list(findings)] if not batch
              else [list(findings)[i:i + batch]
                    for i in range(0, len(findings), batch)])
    for n, g in enumerate(groups, 1):
        body = ("Review these findings, per REVIEW §5 checks 2 to 5. Record "
                "the four observations for each; the outcome is derived from "
                "them and is not yours to write. Under each finding, the "
                "client's process has placed the lines its citations point "
                "at, with the lines around them.\n\n"
                + "\n\n".join(_finding_text(f, claims_by_id)
                                + "\n    cited material:\n"
                                + _cited_materials(f, docs) for f in g)
                + "\n\nEmit `finding_reviews` for exactly these findings.")
        ask(f"finding_reviews[{n}/{len(groups)}]", body,
            schemas.finding_reviews_schema())

    never = stats.get("documents_cited_but_never_opened") or {}
    ask("record_check",
        "You have checked every finding. Report the one statement REVIEW §7 "
        "asks for: whether the audit's record bears out the work the findings "
        "claim. The client's process read the record; these are its facts.\n\n"
        "Documents the record shows the audit opened:\n"
        + ("\n".join(f"  {d}" for d in stats.get("documents_opened") or [])
           or "  (none)")
        + "\n\nDocuments the findings cite:\n"
        + ("\n".join(f"  {d}" for d in stats.get("documents_cited") or [])
           or "  (none)")
        + "\n\nCited documents the record does not show being opened, with "
          "the claims whose findings cite them:\n"
        + ("\n".join(f"  {d}: claims {', '.join(str(c) for c in cs)}"
                      for d, cs in never.items()) or "  (none)"),
        schemas.record_check_schema())

    return {"obj": schemas.merge_parts(parts), "calls": calls,
            "batched": bool(batch)}


# ---------------------------------------------------------------------------
# The retest
# ---------------------------------------------------------------------------

def retest(run: Path, world: str, model_path: Optional[Path], target: Path,
           claims_by_id: Dict[int, Dict[str, Any]],
           findings_by_id: Dict[int, Dict[str, Any]],
           candidates: Sequence[int], sampled: Sequence[int],
           max_tokens: int,
           docs: Optional[Dict[str, List[str]]] = None) -> Dict[str, Any]:
    """A second reviewer, on the findings that qualify and on a sample.

    BLIND. It is given the finding and the materials and nothing else — not the
    first reviewer's observations, not its exception, and not whether this
    finding was one it failed or one it passed. It runs in its own world, and
    the first review is not on disk when it starts.

    THE TWO SETS ARE NOT SYMMETRIC, and REVIEW.md §9 says why. A disagreement
    about a finding the first reviewer FAILED marks that finding borderline: a
    reviewer asserting a defect in finished work carries the higher standard,
    because one wrong exception puts a defect on the record against an audit
    that does not carry it. A disagreement about a SAMPLED finding that held
    changes nothing about that finding — it counts toward the control and
    nowhere else. Sampling exists because retesting only exceptions can catch a
    reviewer that is too harsh and can never catch one that is too lax.
    """
    subjects = list(dict.fromkeys(list(candidates) + list(sampled)))
    if not subjects:
        return {"ran": False, "reason": "nothing qualified and nothing sampled"}
    from chat.chat_loop import ChatLoop                        # noqa: E402
    name, cfg = build_config(run, world, model_path, target)
    loop = ChatLoop(character_name=name, character_config=cfg)
    method_text = load_workflow(REPO / REVIEW_PATH)
    out: Dict[str, Any] = {"ran": True, "world": world,
                           "resolved_model": loop.backend.resolved_model(),
                           "results": {}}
    try:
        for cid in subjects:
            f = findings_by_id.get(cid)
            if f is None:
                continue
            body = ("Review this one finding, per REVIEW §5 checks 2 to 5. You "
                    "have not seen any other review of it. Under the finding, "
                    "the client's process has placed the lines its citations "
                    "point at, with the lines around them.\n\n"
                    + _finding_text(f, claims_by_id)
                    + "\n    cited material:\n"
                    + _cited_materials(f, docs or {})
                    + "\n\nEmit `finding_reviews` for this finding only.")
            r = _emit(loop, method_text, body,
                      schemas.finding_reviews_schema(), max_tokens)
            rows = ((r.get("obj") or {}).get("finding_reviews") or [])
            row = next((x for x in rows if x.get("claim_id") == cid),
                       rows[0] if rows else None)
            out["results"][cid] = {
                "parse": r["parse"],
                "observations": ({k: row.get(k) for k in schemas.OBSERVATIONS}
                                 if row else None),
                "exception": (row or {}).get("exception"),
                "materials_show": (row or {}).get("materials_show")}
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("retest executor shutdown failed: %s", e)
    return out


def standings(first: Dict[str, Any], second: Dict[str, Any],
              candidates: Sequence[int],
              sampled: Sequence[int]) -> Dict[str, Any]:
    """What the retest made of each subject, per REVIEW.md §9."""
    if not second.get("ran"):
        return {"ran": False, "reason": second.get("reason"),
                "note": "a retest that did not run is not one that disagreed"}
    firsts = {r.get("claim_id"): r for r in first.get("finding_reviews") or []}
    rows, agree_ex, dis_ex, agree_s, dis_s = {}, 0, 0, 0, 0
    for cid, got in (second.get("results") or {}).items():
        obs = got.get("observations")
        mine = firsts.get(cid)
        if obs is None or mine is None:
            rows[cid] = {"standing": "not retested",
                         "why": "the retest produced nothing readable"}
            continue
        same = all(obs.get(n) == mine.get(n) for n in schemas.OBSERVATIONS)
        # BOTH SETS OF OBSERVATIONS ARE KEPT. The first version recorded only
        # `agreed`, so a control that disagreed on 2 of 3 (Qwen, 2026-09-02)
        # could not say what the second reviewer had seen differently.
        both = {"first": {n: mine.get(n) for n in schemas.OBSERVATIONS},
                "second": obs}
        if cid in candidates:
            rows[cid] = {"standing": "stands" if same else "does not stand",
                         "set": "exception", **both}
            agree_ex += same
            dis_ex += (not same)
        else:
            # A sampled finding that holds is a control, not a correction:
            # the disagreement is counted and the finding is untouched.
            rows[cid] = {"standing": "control", "agreed": same, "set": "held",
                         **both}
            agree_s += same
            dis_s += (not same)
    return {"ran": True, "per_finding": rows,
            "exceptions": {"agreed": agree_ex, "disagreed": dis_ex},
            "held_sample": {"agreed": agree_s, "disagreed": dis_s},
            # The comparison REVIEW §9 exists for: if the second reviewer
            # disagrees about findings the first passed at a rate near the rate
            # it disagrees about ones the first failed, the review is not
            # discriminating and its numbers should not be used.
            "discriminating": None if not (agree_s + dis_s) and not (agree_ex + dis_ex)
            else {"disagreement_on_exceptions":
                  round(dis_ex / (agree_ex + dis_ex), 2) if (agree_ex + dis_ex) else None,
                  "disagreement_on_held":
                  round(dis_s / (agree_s + dis_s), 2) if (agree_s + dis_s) else None}}


BRIEF = """You are reviewing the claims audit in this run directory, per REVIEW.md.

Under `inspect`: the audit's frozen claim surface (`claims.json`), its findings
(`findings.json`), the statistics the client's process computed
(`review/statistics.json`), and the auditor's working record. The materials that
audit examined are under `inspect_external`.

Every mechanical property of the output has been checked already, per REVIEW §4.
Each cited document exists, each line range is inside its file, and each quoted
span was found at the lines it names. Do not re-check those; the statistics
record what they found. Your subject is the citations that do resolve, where
the only remaining question is what they mean.

Read the findings and the materials now, and the working record last, per
REVIEW §3. Work in as many legs as you need: end a leg with `yield` and I will
say continue, or with `respond` when you have read what you need.

Do not write the review in a leg. When you stop reading I will ask you for it,
under the schema in REVIEW §8.
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, help="a finished audit run directory")
    ap.add_argument("--model", help="YAML with an llm_config block")
    ap.add_argument("--retest-model", help="the second reviewer; defaults to --model")
    ap.add_argument("--world", help="fresh world name; defaults from the run")
    ap.add_argument("--max-turns", type=int, default=12)
    ap.add_argument("--batch", type=int, default=0,
                    help="items per emission call; 0 asks for each part at once")
    ap.add_argument("--held-sample", type=int, default=3,
                    help="findings that hold, retested as a control (REVIEW §9)")
    ap.add_argument("--seed", type=int, default=0, help="for the held sample")
    args = ap.parse_args()

    run = Path(args.run).resolve()
    for needed in ("claims.json", "findings.json", "run_meta.json"):
        if not (run / needed).is_file():
            raise SystemExit(f"{run}: no {needed} — not a finished audit run")
    out = run / "review"
    if (out / "review.json").is_file():
        raise SystemExit(f"{out}: already reviewed. A reviewer whose `inspect` "
                         f"can see a previous review is not independent of it.")
    out.mkdir(exist_ok=True)

    meta = json.loads((run / "run_meta.json").read_text())
    target = Path(meta.get("external_repo") or ".")
    claims_doc = json.loads((run / "claims.json").read_text())
    findings_doc = json.loads((run / "findings.json").read_text())
    claim_source = claims_doc.get("claim_source") or ""
    frozen = claims_doc.get("claims") or []
    findings = findings_doc.get("findings") or []
    world = args.world or f"review_{run.name[-40:]}"

    # ---- part one: the statistics, recomputed here -------------------------
    stats = statistics(run, target, claim_source)
    docs = audit_schemas.corpus_index(target)
    (out / "statistics.json").write_text(
        json.dumps(stats, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    logger.info("statistics: %d claims, %d findings, audit checks %s/%s, "
                "%d unclaimed span(s)", len(frozen), len(findings),
                "ok" if stats["surface_check"]["ok"] else "FAIL",
                "ok" if stats["output_check"]["ok"] else "FAIL",
                len(stats["unclaimed_spans"]))

    from chat.chat_loop import ChatLoop                        # noqa: E402
    name, cfg = build_config(run, world, Path(args.model) if args.model else None,
                             target)
    loop = ChatLoop(character_name=name, character_config=cfg)
    method_text = load_workflow(REPO / REVIEW_PATH)
    max_tokens = int((cfg.get("chat") or {}).get("react_max_tokens", 32768))
    legs, error, emission = [], None, None
    t0 = datetime.datetime.now(datetime.timezone.utc)

    try:
        # ---- part two: the reviewer reads ---------------------------------
        text, reason = BRIEF, "opening brief"
        for i in range(args.max_turns):
            loop._process_user_turn(source=SOURCE, text=text, close=False)
            reply = latest_reply(loop, SOURCE)
            exit_reason = last_exit_reason(world, name)
            legs.append({"leg": i + 1, "sent": reason,
                         "exit_reason": exit_reason, "reply_chars": len(reply)})
            logger.info("leg %d: exit=%s chars=%d", i + 1, exit_reason, len(reply))
            if exit_reason in ("llm_error", "crashed"):
                error = f"turn {i + 1} ended {exit_reason} — review is not valid"
                break
            if exit_reason == "max_iters":
                error = f"turn {i + 1} hit max_iters — review is not valid"
                break
            if exit_reason != "yield":
                break
            text, reason = CONTINUE, "reviewer yielded — continue reading"

        # ---- part three: the observations ---------------------------------
        if not error:
            emission = emit_parts(loop, method_text, stats, frozen, findings,
                                  max_tokens, batch=args.batch, docs=docs)
            for c in emission["calls"]:
                if c["response_format_dropped"]:
                    error = (f"the route dropped "
                             f"{', '.join(c['response_format_dropped'])} on "
                             f"{c['part']} — the review was not constrained")
    except Exception as e:                                     # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("review failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)

    obj = (emission or {}).get("obj") or {"claim_checks": [],
                                          "finding_reviews": [],
                                          "record_check": ""}
    check = schemas.check_review(obj, frozen, findings)
    for problem in check["problems"]:
        issues.note(out, stage="audit_review", code="review_check",
                    text=problem, severity="blocking")
    derived = schemas.derive_outcomes(obj)

    # ---- the retest, blind, before the review is on disk --------------------
    held = [cid for cid, o in derived["outcomes"].items() if o["holds"]]
    random.Random(args.seed).shuffle(held)
    sampled = held[:max(0, args.held_sample)]
    second = retest(run, f"{world}_retest",
                    Path(args.retest_model or args.model)
                    if (args.retest_model or args.model) else None,
                    target, {c.get("id"): c for c in frozen},
                    {f.get("claim_id"): f for f in findings},
                    derived["retest_candidates"], sampled, max_tokens,
                    docs=docs) \
        if not error else {"ran": False, "reason": "the review did not complete"}
    stand = standings(obj, second, derived["retest_candidates"], sampled)

    # The second reviewer's answers in full, including parse state and any
    # exception text — the standings summarise them, and did not keep them.
    (out / "retest.json").write_text(
        json.dumps(second, indent=1, ensure_ascii=False, default=str) + "\n",
        encoding="utf-8")
    (out / "review.json").write_text(
        json.dumps(obj, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "outcomes.json").write_text(
        json.dumps({"derived": derived, "standings": stand}, indent=1,
                   ensure_ascii=False, default=str) + "\n", encoding="utf-8")
    wall = round((datetime.datetime.now(datetime.timezone.utc) - t0).total_seconds(), 1)
    (out / "review_meta.json").write_text(json.dumps({
        "reviewed_run": run.name, "world": world,
        "model_config": args.model, "resolved_model": loop.backend.resolved_model(),
        "batch": args.batch, "held_sample": len(sampled),
        "legs": legs, "wall_clock_s": wall,
        "emission": [{k: v for k, v in c.items() if k != "raw"}
                     for c in (emission or {}).get("calls", [])],
        "review_check": check, "retest": {k: v for k, v in second.items()
                                          if k != "results"},
        "error": error,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(legs)} legs, {wall}s, error={error}")
    print(f"observations: {check['figures']}")
    print(f"retest: {stand.get('exceptions')} on exceptions, "
          f"{stand.get('held_sample')} on the held sample")
    print(f"review: {out}/review.json, outcomes.json")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
