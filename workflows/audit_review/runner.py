#!/usr/bin/env python3
"""Review a finished claims audit against the evidence it cites.

    python3 workflows/audit_review/runner.py --run <audit run directory>
    python3 workflows/audit_review/runner.py --run <dir> --model measure/models/grok_4p6.yaml

WHY. METHOD §12a: "There is no independent review of the report, and that is a
decision... Revisit it before the first engagement where a finding moves real
money." And §12 step 6b, on the audit checking its own citations: "This catches
a citation pointing at nothing. It will not catch one pointing convincingly at
the wrong line." This catches that one.

WHAT IS MECHANICAL HERE AND WHAT IS NOT. Two jobs are done before the reviewer
starts, because they have answers that do not depend on judgement:

  conformance   the method-conformance criteria that need no answer key —
                markers, closed vocabularies, a claim surface that closed
  citations     every cited line, fetched, and every quoted span, looked for.
                Whether doc4:16-19 EXISTS is a file operation, and it decides
                one of the two verdicts that fail a report. Resolving it here
                means the reviewer cannot hallucinate a line it never fetched,
                and spends its judgement on whether the line SUPPORTS the claim.
  scheme        whether those integers can be line numbers at all. A reference
                past the end of its document proves they are not — see §4.0.
                This does not decide admissibility; it is what the reviewer
                decides it on.

An LLM asked whether `=== LIMITATIONS ===` appears is slower, dearer and less
reliable than a substring test. The workflow's value is the checks with no
mechanical form.

ONE REVIEW PER RUN. The runner refuses if <run>/review/ exists: a reviewer whose
`inspect` can see a previous review is not independent of it.

NOT A RE-AUDIT. The subject is the report, not the business — REVIEW.md §2, §10.
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent          # workflows/audit_review
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

import yaml                                                    # noqa: E402

SCENARIO = HERE / "scenario.yaml"
SOURCE = "User"

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("audit_review")
logger.setLevel(logging.INFO)

# The second deliverable is ASKED FOR, not detected — the same rule the audit
# runner arrived at. Its opening sentence is a stable sentinel: harness text,
# emitted verbatim, so matching it is safe in a way matching model output is not.
SUMMARY_REQUEST = ("The review is received. Now the review summary, per §8. "
                   "The whole reply is the summary — no preamble, no "
                   "restatement of the review.")
CONTINUE = "continue"

# Report-side citation forms, from what models actually emit across 29 reports.
_CITE = re.compile(
    r"\b([A-Za-z][A-Za-z0-9_.\-/]*)\s*:\s*(?:lines?\s*)?(\d+)(?:\s*[-–]\s*(\d+))?")
_LOOKS_LIKE_A_FILE = re.compile(r"\.[A-Za-z][A-Za-z0-9]{0,4}$")
_DOCN = re.compile(r"^doc(\d)", re.I)
_FINDING = re.compile(r"^\s*\**\s*Finding\s+(\d+)[^\n\[]*\[([^\]]+)\]", re.M | re.I)

# A quoted span in the report. Both quote characters, because a report that has
# been through a model may carry either. The 12-character floor drops `"active"`
# and other single-word quoting, which is not a citation of anything.
_QUOTE = re.compile(r"[\"“]([^\"”\n]{12,})[\"”]")
_MIN_QUOTE_CHARS = 12


_NOT_TEXT = re.compile(r"[^a-z0-9 ]+")
_SEGMENT = re.compile(r"(?<=[.;:])\s+|\n+")
_MIN_SEGMENT_CHARS = 8


def _flatten(s: str) -> str:
    """Lowercased alphanumerics and single spaces, and nothing else.

    Everything discarded here is a difference that is not a citation failure:
    markdown emphasis and bullet markers (`*   **Status:**` against `Status:`),
    the newline where a quote spans two source lines, a re-typed apostrophe,
    and the punctuation a model adds when it joins two bullets into one
    sentence. What survives is the words, in order.
    """
    return " ".join(_NOT_TEXT.sub(" ", s.lower()).split())


def resolve_quotes(report: str, target: Path) -> List[Dict[str, Any]]:
    """Every quoted span in the report, looked for in the materials.

    THE SECOND HALF OF THE CITATION SURFACE. `_CITE` sees `docN:NN` and nothing
    else, so a report citing its evidence by section name — "(doc4, Backups
    section)" — resolved to zero entries and the reviewer read that absence as
    absence of evidence. On 2026-08-26 that produced "supported 1 of 15" on a
    report whose evidence quotes were, every one, verbatim correct.

    A quote resolves by file operation exactly as a line reference does, so it
    decides `[broken citation]` without judgement. It also survives the boundary
    a line number does not: the inspect subagent reformats numbered content into
    prose before the auditor ever sees it (code_subagent.py:407 numbers it;
    subagent.py:252 is the model's own answer, unnumbered).

    SEGMENTS, NOT SPANS. Reports quote composites — four bullets of doc4's
    Backups section joined into one sentence, sometimes reordered. Every fact in
    such a quote can be verbatim while the span as typed appears nowhere. Whole-
    span matching reports those as fabrication, so each quote is split and its
    segments resolved separately. A quote resolves when all of them do.

    KNOWN NOISE. The extractor takes quoted spans, and a report that writes
    `"Streamlined" is not a fair description of "not present."` yields the prose
    between the two quoted words. Such a span resolves to nothing, correctly,
    and still counts in the denominator. The ratio is a signal for §4.0, not a
    verdict.
    """
    if not target.is_dir():
        return []
    flat = {f.name: _flatten(f.read_text(errors="replace"))
            for f in sorted(target.rglob("*")) if f.is_file()}

    body = _FINDING.sub("", report)     # titles quote the claim; they are headings
    out: List[Dict[str, Any]] = []
    seen = set()
    for m in _QUOTE.finditer(body):
        q = m.group(1).strip()
        if len(q) < _MIN_QUOTE_CHARS or q in seen:
            continue
        seen.add(q)
        segs = [s for s in (_flatten(x) for x in _SEGMENT.split(q))
                if len(s) >= _MIN_SEGMENT_CHARS]
        if not segs:
            continue
        whole = _flatten(q)
        hits, missing, docs_hit = 0, [], []
        for s in segs:
            for name, text in flat.items():
                if s in text:
                    hits += 1
                    if name not in docs_hit:
                        docs_hit.append(name)
                    break
            else:
                missing.append(s)
        contiguous = any(whole in text for text in flat.values())
        rec: Dict[str, Any] = {
            "quote": q[:300], "resolved": hits == len(segs),
            "how": "contiguous" if contiguous
                   else "segments" if hits == len(segs)
                   else "partial" if hits else "miss",
            "segments": len(segs), "segments_found": hits,
            "documents": docs_hit,
        }
        if missing:
            rec["missing"] = [s[:160] for s in missing]
        out.append(rec)
    return out


def scheme(citations: List[Dict[str, Any]],
           quotes: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Whether the report's `docN:NN` references are line numbers at all.

    Decidable by file operation, and only in the negative: **an integer that
    exceeds its document's line count proves the reference is not a line
    number.** `doc2:13` against a five-line doc2 is not a broken line reference;
    it is claim 13 of the 13 that report enumerated for doc2.

    This does not say what the scheme IS. Nothing mechanical can. It says the
    declared one does not hold, which is what REVIEW.md §4.0 needs in order to
    stop rather than report a ratio computed over a misread.
    """
    over = [c for c in citations
            if not c.get("resolved") and " lines" in str(c.get("why", ""))]
    consistent: Optional[bool] = None
    if citations:
        consistent = not over

    # PER DOCUMENT, because the rate is what distinguishes the two cases. One
    # stray reference in forty is a typo. Four of the seven references to one
    # five-line document are a different coordinate system, and only the second
    # is grounds to stop. Measured across 29 reports on 2026-08-26: six carry at
    # least one over-length reference, and they range from 1-in-42 to 4-in-7.
    by_doc: Dict[str, Dict[str, Any]] = {}
    for c in citations:
        key = str(c.get("cited", "")).split(":")[0].strip().lower() or "?"
        d = by_doc.setdefault(key, {"refs": 0, "exceeding": 0, "max_cited": 0})
        d["refs"] += 1
        d["max_cited"] = max(d["max_cited"], int(c.get("line") or 0))
        if c in over:
            d["exceeding"] += 1
            d["file_lines"] = str(c.get("why", ""))

    return {
        "line_refs": len(citations),
        "line_refs_resolving": sum(1 for c in citations if c.get("resolved")),
        "line_refs_exceeding_file_length": len(over),
        "by_document": {k: v for k, v in sorted(by_doc.items())
                        if v["exceeding"]},
        "quotes": len(quotes),
        "quotes_resolving": sum(1 for q in quotes if q.get("resolved")),
        "quotes_contiguous": sum(1 for q in quotes
                                 if q.get("how") == "contiguous"),
        "consistent_with_line_numbering": consistent,
    }


def resolve_citations(report: str, target: Path) -> Dict[str, Any]:
    """Every citation in the report, fetched from the materials.

    THE REVIEWER DOES NOT FETCH ITS OWN. Two reasons, and the second is the
    one that matters. A model asked to quote a line it has not read will
    sometimes produce a plausible line; handing it the bytes removes that
    possibility entirely. And `[broken citation]` is one of the two verdicts
    that fail a report, so it should be decided by a file operation rather than
    by judgement that was measured flipping on identical text.

    Matching is deliberately the same shape as measure/fixtures/dataroom/
    overlap.py: a document reference is `docN` or something with a file
    extension, so "12:30" and "Active customers: 120" are not citations.
    """
    files = {f.name: f for f in target.rglob("*") if f.is_file()} \
        if target.is_dir() else {}
    by_docn = {}
    for name, f in files.items():
        m = _DOCN.match(name)
        if m:
            by_docn.setdefault(f"doc{m.group(1)}", f)

    out: List[Dict[str, Any]] = []
    seen = set()
    for m in _CITE.finditer(report):
        tok, lo, hi = m.group(1), int(m.group(2)), m.group(3)
        base = tok.rsplit("/", 1)[-1]
        dm = _DOCN.match(base)
        if not (dm or _LOOKS_LIKE_A_FILE.search(base)):
            continue
        key = (base, lo, hi)
        if key in seen:
            continue
        seen.add(key)
        f = by_docn.get(f"doc{dm.group(1)}") if dm else files.get(base)
        rec: Dict[str, Any] = {"cited": m.group(0).strip(), "document": base,
                               "line": lo, "through": int(hi) if hi else lo}
        if f is None:
            rec["resolved"] = False
            rec["why"] = "no such document in the materials"
        else:
            lines = f.read_text(errors="replace").splitlines()
            hi_i = min(rec["through"], lo + 40)
            if lo < 1 or lo > len(lines):
                rec["resolved"] = False
                rec["why"] = f"{f.name} has {len(lines)} lines"
            else:
                rec["resolved"] = True
                rec["document"] = f.name
                rec["text"] = "\n".join(
                    f"{n}: {lines[n-1]}" for n in range(lo, min(hi_i, len(lines)) + 1))
        out.append(rec)
    broken = [c for c in out if not c["resolved"]]
    quotes = resolve_quotes(report, target)
    return {"citations": out, "total": len(out), "broken": len(broken),
            "quotes": quotes, "scheme": scheme(out, quotes)}


def conformance(run: Path) -> Dict[str, Any]:
    """The method-conformance checks that need no answer key.

    Reused from the fixture scorer rather than reimplemented — these are the
    five criteria that hold for ANY engagement, including a paying client's
    where no answer key exists or ever will. Recall against a key stays in
    measure/, which is where the key is.
    """
    sys.path.insert(0, str(REPO / "measure" / "fixtures" / "dataroom"))
    import score                                               # noqa: E402
    report = (run / "report.md").read_text(errors="replace") \
        if (run / "report.md").is_file() else ""
    gap = (run / "gap_map.md").read_text(errors="replace") \
        if (run / "gap_map.md").is_file() else ""
    vc = score.verdict_conformance(report)
    rec = score.recommendation_of(report)
    el = score.gap_map_elements(gap) if gap else {}
    return {
        "recommendation": rec,
        "§6 verdicts only": not vc["off_vocabulary"],
        "off_vocabulary": vc["off_vocabulary"],
        "limitations statement": bool(score._LIMITS_RE.search(report)),
        "gap map present": bool(gap.strip()),
        "§15 elements missing": [k for k, ok in el.items() if not ok],
        "findings": [{"n": int(n), "verdict": v.strip()}
                     for n, v in _FINDING.findall(report)],
    }


def build_config(run: Path, world: str, model_path: Optional[Path],
                 target: Path) -> Tuple[str, Dict[str, Any]]:
    """Same shape as the audit runner's, and the same reasons: the model config
    REPLACES the llm_config block rather than merging into it, and per-session
    paths are set here rather than by editing the committed scenario."""
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


BRIEF = """You are reviewing the claims audit in this run directory, per REVIEW.md.

The report and the Gap Map are under `inspect`, with the auditor's working
record. The materials that audit examined are under `inspect_external`.

Citations have been resolved for you as far as a file operation reaches.
`review/citations.json` holds three things: every `docN:NN` reference with the
text of the line it names; every quoted span in the report, looked for in the
materials; and a `scheme` block saying whether those integers can be line
numbers at all.

Do not re-fetch what it has already resolved, and do not assess support for a
finding whose citation is broken. **It is not the whole of the materials.** A
reference it does not cover — an evidence document named by section rather than
by line — is absent from it because no regex could fetch it, not because the
document says nothing. Read anything it does not cover under `inspect_external`.

Settle admissibility first, per §4.0. If the report's citation scheme cannot be
established, say so and stop: do not enumerate, do not check findings, do not
report a supported ratio.

Otherwise enumerate the findings and close the review surface as the method
says, then check every one. Work in as many legs as you need — end a leg with
`yield` and I will say continue. End with the review; I will ask for the summary
after it.
"""


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True,
                    help="the audit run directory to review")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block. Need not be the model "
                         "that produced the audit — arguably should not be")
    ap.add_argument("--world", default=None)
    ap.add_argument("--max-turns", type=int, default=25)
    args = ap.parse_args()

    run = args.run.resolve()
    meta_p = run / "run_meta.json"
    if not meta_p.is_file():
        raise SystemExit(f"{run}: no run_meta.json — not a run directory")
    if not (run / "report.md").is_file():
        raise SystemExit(f"{run}: no report.md to review")
    out = run / "review"
    if out.exists():
        raise SystemExit(
            f"{out} already exists. One review per run: a reviewer whose "
            f"`inspect` can see a previous review is not independent of it. "
            f"Move or delete it to review again.")

    meta = json.loads(meta_p.read_text(encoding="utf-8"))
    target = Path(meta.get("external_repo") or "").resolve()
    if not target.is_dir():
        raise SystemExit(f"target {target} does not exist — citations cannot "
                         f"be resolved, and a review without them is opinion")
    out.mkdir(parents=True)

    report = (run / "report.md").read_text(encoding="utf-8", errors="replace")
    cites = resolve_citations(report, target)
    (out / "citations.json").write_text(json.dumps(cites, indent=2), encoding="utf-8")
    conf = conformance(run)
    (out / "conformance.json").write_text(json.dumps(conf, indent=2), encoding="utf-8")
    logger.info("resolved %d citations, %d broken; %d quotes, %d resolving; "
                "%d findings parsed", cites["total"], cites["broken"],
                cites["scheme"]["quotes"], cites["scheme"]["quotes_resolving"],
                len(conf["findings"]))

    world = args.world or f"review_{run.name.split('_', 1)[-1]}"
    if (REPO / "scenarios" / world).exists():
        raise SystemExit(f"world '{world}' already exists — pass --world")

    name, cfg = build_config(run, world, args.model, target)
    from chat.chat_loop import ChatLoop                        # noqa: E402
    from chat.model_params import TOP_P                        # noqa: E402
    from workflows.claims_audit.runner import (                # noqa: E402
        latest_reply, last_exit_reason, git_rev)
    loop = ChatLoop(character_name=name, character_config=cfg)
    resolved_model = loop.backend.resolved_model()
    logger.info("reviewing %s with %s", run.name, resolved_model)

    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    t0, legs, error, summary_sent, review_text = time.time(), [], None, False, ""
    try:
        text = BRIEF
        for i in range(args.max_turns):
            loop._process_user_turn(source=SOURCE, text=text, close=False)
            reply = latest_reply(loop, SOURCE)
            exit_reason = last_exit_reason(world, name)
            legs.append({"leg": i + 1, "exit_reason": exit_reason,
                         "reply_chars": len(reply)})
            logger.info("leg %d: exit=%s chars=%d", i + 1, exit_reason, len(reply))
            if exit_reason in ("llm_error", "crashed"):
                error = f"turn {i + 1} ended {exit_reason} — review is not valid"
                break
            if summary_sent:
                break
            if exit_reason == "yield":
                text = (CONTINUE + f"\n\n[review state, recorded by the client's "
                        f"process — leg {i + 2} of {args.max_turns}, "
                        f"{(time.time() - t0) / 60:.0f} min elapsed.]")
                continue
            review_text, text, summary_sent = reply, SUMMARY_REQUEST, True
    except Exception as e:                                     # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("review failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)

    final = latest_reply(loop, SOURCE)
    if summary_sent:
        (out / "review.md").write_text((review_text or "").rstrip() + "\n",
                                       encoding="utf-8")
        (out / "summary.md").write_text(final.strip() + "\n", encoding="utf-8")
    elif final:
        (out / "review.md").write_text(final, encoding="utf-8")
        logger.warning("no summary leg — review summary not produced")

    (out / "review_meta.json").write_text(json.dumps({
        "reviewed_run": run.name,
        "world": world,
        "model_config": str(args.model) if args.model else "(scenario default)",
        "resolved_model": resolved_model,
        "top_p": TOP_P,
        "target": str(target),
        "target_rev": meta.get("target_rev"),
        "harness_rev": git_rev(REPO),
        "citations_total": cites["total"],
        "citations_broken": cites["broken"],
        "scheme": cites["scheme"],
        "findings_parsed": len(conf["findings"]),
        "legs": legs,
        "wall_clock_s": round(time.time() - t0, 1),
        "error": error,
        "captured_at_utc": ts,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(legs)} legs, {round(time.time() - t0, 1)}s, error={error}")
    sch = cites["scheme"]
    print(f"citations: {cites['total']} line refs, {cites['broken']} broken; "
          f"{sch['quotes']} quotes, {sch['quotes_resolving']} resolving")
    if sch["consistent_with_line_numbering"] is False:
        print(f"scheme: NOT line numbers — "
              f"{sch['line_refs_exceeding_file_length']} references exceed "
              f"their document's line count. See REVIEW.md §4.0.")
    print(f"review: {out}/review.md, summary.md")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
