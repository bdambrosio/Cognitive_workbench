"""Merge the reviewed runs of one engagement, then rate each finding's
materiality.

    python3 workflowsv2/audit_materiality/runner.py --engagement dataroom-fixture \\
        --run <doc1 run> --run <doc2 run> --run <doc9 run> \\
        --model measure/models/or_glm53flash.yaml [--batch 20] [--label x]

Two parts, in order:

  1. MERGE (merge.py). Mechanical: the named runs' findings, each with its
     review outcome and the citation problems the output check recorded,
     concatenated. No deduplication — the claim surface is human-owned.
  2. RATE. Schema-constrained calls over batches of rateable findings, per
     MATERIALITY.md. No tools, no legs: the finding and the transaction are
     the whole input.

Output, under engagements/<name>/merged/<ts>_<label>/: merged.json,
materiality.json, materiality.md (for a person: decisive first), meta.json,
issues.jsonl.

WHY THE ENGAGEMENT AND NOT ONLY THE RUNS. The transaction — buyer, purpose,
price basis, structure — is an engagement fact (`transaction:` in
engagement.yaml) and the scale in MATERIALITY.md §3 is relative to it.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("audit_materiality.run")
logger.setLevel(logging.INFO)

from chat.workflow import load_workflow                        # noqa: E402
from workflowsv2 import issues                                 # noqa: E402
from workflowsv2.audit_materiality import merge as merging     # noqa: E402
from workflowsv2.audit_materiality import schemas              # noqa: E402
# The review's rendering of a finding, so the rater reads the same text the
# reviewer did. The review runner configures logging at import without
# force=True, which is a no-op after the basicConfig above.
from workflowsv2.audit_review.runner import _finding_text      # noqa: E402
from workflowsv2.claims_audit.runner import load_engagement    # noqa: E402
from workflowsv2.emit import emit                              # noqa: E402

SCENARIO = HERE / "scenario.yaml"
METHOD_PATH = "workflowsv2/audit_materiality/method/MATERIALITY.md"
STAGE = "audit_materiality"


def build_config(out: Path, world: str, model_path: Optional[Path]
                 ) -> Tuple[str, Dict[str, Any]]:
    """Same shape and the same reasons as the review runner's: the model config
    REPLACES `llm_config` rather than merging into it."""
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
    cfg["external_repo"] = str(out)
    cfg["inspect_repo"] = str(out)
    return name, cfg


def _rating_text(f: Dict[str, Any]) -> str:
    """One merged finding as the rater reads it: the review's rendering, then
    the review outcome and any citation problem, per MATERIALITY.md §2."""
    claim = {f["claim_id"]: {"quote": f.get("quote"), "lines": f.get("lines"),
                             "statement": f.get("statement")}}
    body = _finding_text({"claim_id": f["claim_id"],
                          "adjudication": f.get("adjudication") or {},
                          "evidence": f.get("evidence") or []}, claim)
    body = body.replace(f"--- claim {f['claim_id']}",
                        f"--- {f['claim_source']} claim {f['claim_id']}", 1)
    rv = f.get("review") or {}
    line = rv.get("outcome", "unreviewed")
    if rv.get("adverse_observations"):
        line += " (" + ", ".join(rv["adverse_observations"]) + ")"
    body += f"\n    review    : {line}"
    if f.get("citation_problems"):
        body += "\n    citation problems: " + " | ".join(f["citation_problems"])
    return body


def rate(loop, method_text: str, transaction: Optional[str],
         findings: List[Dict[str, Any]], max_tokens: int, batch: int
         ) -> Dict[str, Any]:
    """The ratings, in batches. Same runner-decides-the-batch pattern as the
    review's emit_parts: one schema per part, concatenated by merge_parts."""
    head = ("The transaction, as the engagement states it:\n\n"
            + (transaction.strip() if transaction else
               "(The engagement states nothing about the transaction. Rate "
               "against a buyer paying a price that assumes every claim "
               "holds, per MATERIALITY \u00a72.)")
            + "\n\n")
    groups = ([findings] if not batch
              else [findings[i:i + batch] for i in range(0, len(findings), batch)])
    parts, calls = [], []
    for n, g in enumerate(groups, 1):
        body = (f"Rate these {len(g)} findings, per MATERIALITY \u00a73 to \u00a75. "
                f"Each carries its claim source and claim id; return both as "
                f"given.\n\n" + "\n\n".join(_rating_text(f) for f in g)
                + "\n\nEmit `ratings` for exactly these findings.")
        out = emit(loop, method_text, head + body, schemas.ratings_schema(),
                   max_tokens)
        calls.append({"part": f"ratings[{n}/{len(groups)}]", "findings": len(g),
                      **{k: v for k, v in out.items() if k not in ("raw", "obj")}})
        if out["obj"] is not None:
            parts.append(out["obj"])
        else:
            logger.warning("batch %d/%d did not parse: %s", n, len(groups),
                           out["parse_error"])
    return {"obj": schemas.merge_parts(parts), "calls": calls}


def render(merged: Dict[str, Any], ratings: Dict[str, Any]) -> str:
    """materiality.md: what a person reads. Decisive first."""
    by_key = {f"{r.get('claim_source')}#{r.get('claim_id')}": r
              for r in ratings.get("ratings") or []}
    order = {m: i for i, m in enumerate(reversed(schemas.MATERIALITY))}
    rows = []
    for f in merged["findings"]:
        k = f"{f['claim_source']}#{f['claim_id']}"
        r = by_key.get(k)
        m = r.get("materiality") if r else ("not rated" if schemas.rateable(f)
                                            else "real")
        rows.append((order.get(m, 99), f, r, m))
    rows.sort(key=lambda x: (x[0], x[1]["claim_source"], x[1]["claim_id"]))
    fig = merged["figures"]
    lines = ["# Materiality", "",
             f"{fig['runs']} run(s), {fig['findings']} findings, "
             f"{fig['reviewed']} reviewed, {fig['unreviewed']} unreviewed. "
             f"Ratings: " + ", ".join(f"{k} {v}" for k, v in
                                       sorted((ratings.get('figures') or {}).items()))
             + ".", ""]
    if fig.get("restated_quotes"):
        lines += ["## Restated across claim sources", ""]
        for x in fig["restated_quotes"]:
            lines.append("- " + "; ".join(f"{i['claim_source']} #{i['claim_id']} "
                                          f"{i['verdict']}" for i in x["in"])
                         + f" — \"{(x['quote'] or '')[:120]}\"")
        lines.append("")
    lines += ["| rating | source | id | verdict | review | citations | claim | basis |",
              "|---|---|---|---|---|---|---|---|"]
    for _, f, r, m in rows:
        rv = f.get("review") or {}
        review = rv.get("outcome", "unreviewed")
        if rv.get("adverse_observations"):
            review += " (" + ", ".join(rv["adverse_observations"]) + ")"
        cites = f"{len(f.get('citation_problems') or [])} problem(s)" \
            if f.get("citation_problems") else "ok"
        q = (f.get("quote") or "").replace("|", "\\|").replace("\n", " ")
        b = ((r or {}).get("basis") or "").replace("|", "\\|").replace("\n", " ")
        lines.append(f"| {m} | {f['claim_source']} | {f['claim_id']} | "
                     f"{(f.get('adjudication') or {}).get('verdict')} | {review} | "
                     f"{cites} | {q[:160]} | {b} |")
    if merged.get("questions"):
        lines += ["", "## Questions for the seller", ""]
        lines += [f"- ({q['claim_source']}) {q['question']}"
                  for q in merged["questions"]]
    if merged.get("unclaimed"):
        lines += ["", "## Unclaimed observations", ""]
        lines += [f"- ({u['claim_source']}) {u.get('note')}"
                  for u in merged["unclaimed"]]
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True,
                    help="engagement name under claims_audit/engagements/")
    ap.add_argument("--run", action="append", required=True, type=Path,
                    help="a reviewed audit run directory; repeat per claim source")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--batch", type=int, default=20,
                    help="findings per rating call; 0 rates all at once")
    ap.add_argument("--label", default=None,
                    help="suffix for the output directory (default: model stem)")
    ap.add_argument("--max-tokens", type=int, default=None)
    args = ap.parse_args()

    eng = load_engagement(args.engagement)
    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    label = args.label or (args.model.stem if args.model else "default")
    out = eng["dir"] / "merged" / f"{ts}_{label}"
    out.mkdir(parents=True, exist_ok=False)
    fh = logging.FileHandler(out / "run.log", encoding="utf-8")
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(fh)

    # ---- part one: merge ----------------------------------------------------
    merged = merging.merge(args.run)
    merged["engagement"] = eng["name"]
    (out / "merged.json").write_text(
        json.dumps(merged, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    fig = merged["figures"]
    logger.info("merged %d run(s): %d findings, %d reviewed, %d unreviewed",
                fig["runs"], fig["findings"], fig["reviewed"], fig["unreviewed"])
    for r in merged["runs"]:
        if not r["reviewed"]:
            issues.note(out, stage=STAGE, code="unreviewed_run",
                        text=f"{r['claim_source']}: no review — its findings "
                             f"are rated with outcome `unreviewed`",
                        severity="check")
    for x in fig["restated_quotes"]:
        issues.note(out, stage=STAGE, code="restated_quote",
                    text="one quote in several claim sources: "
                         + "; ".join(f"{i['claim_source']} #{i['claim_id']} "
                                     f"{i['verdict']}" for i in x["in"]),
                    severity="check")
    if not eng.get("transaction"):
        issues.note(out, stage=STAGE, code="no_transaction",
                    text="engagement.yaml has no `transaction:` block; rated "
                         "against a buyer paying a price that assumes every "
                         "claim holds", severity="check")

    # ---- part two: rate -----------------------------------------------------
    rateable = [f for f in merged["findings"] if schemas.rateable(f)]
    from chat.chat_loop import ChatLoop                        # noqa: E402
    world = f"materiality_{ts}_{label}"[:60]
    name, cfg = build_config(out, world, args.model)
    loop = ChatLoop(character_name=name, character_config=cfg)
    method_text = load_workflow(REPO / METHOD_PATH)
    max_tokens = args.max_tokens or int((cfg.get("chat") or {})
                                        .get("react_max_tokens", 32768))
    t0 = datetime.datetime.now(datetime.timezone.utc)
    error, result = None, {"obj": {"ratings": []}, "calls": []}
    try:
        result = rate(loop, method_text, eng.get("transaction"), rateable,
                      max_tokens, args.batch)
        for c in result["calls"]:
            if c["response_format_dropped"]:
                error = (f"the route dropped "
                         f"{', '.join(c['response_format_dropped'])} on "
                         f"{c['part']} — the ratings were not constrained")
            elif c["parse"] == "unparseable":
                error = f"{c['part']} did not parse"
    except Exception as e:                                     # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("rating failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)

    ratings = result["obj"]
    check = schemas.check_ratings(ratings, merged)
    for p in check["problems"]:
        issues.note(out, stage=STAGE, code="ratings_check", text=p,
                    severity="blocking")
    ratings["figures"] = check["figures"]["materiality"]

    # A consequential rating on a finding the review did not uphold, or whose
    # citation failed the mechanical check, is for a person to read.
    by_key = {f"{f['claim_source']}#{f['claim_id']}": f for f in merged["findings"]}
    for r in ratings.get("ratings") or []:
        f = by_key.get(f"{r.get('claim_source')}#{r.get('claim_id')}")
        if not f or r.get("materiality") == "not_material":
            continue
        rv = f.get("review") or {}
        if rv.get("outcome") == "does_not_hold" or f.get("citation_problems"):
            why = []
            if rv.get("outcome") == "does_not_hold":
                why.append("review: does not hold ("
                           + ", ".join(rv.get("adverse_observations") or []) + ")")
            if f.get("citation_problems"):
                why.append(f"{len(f['citation_problems'])} citation problem(s)")
            issues.note(out, stage=STAGE, code="rated_but_not_upheld",
                        text=f"{r['claim_source']} #{r['claim_id']} rated "
                             f"{r['materiality']}; " + "; ".join(why),
                        severity="check")

    (out / "materiality.json").write_text(
        json.dumps(ratings, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "materiality.md").write_text(render(merged, ratings), encoding="utf-8")
    wall = round((datetime.datetime.now(datetime.timezone.utc) - t0).total_seconds(), 1)
    (out / "meta.json").write_text(json.dumps({
        "engagement": eng["name"], "world": world,
        "model_config": str(args.model) if args.model else None,
        "resolved_model": loop.backend.resolved_model(),
        "runs": [{k: r[k] for k in ("dir", "claim_source", "harness_rev",
                                    "resolved_model", "reviewed")}
                 for r in merged["runs"]],
        "batch": args.batch, "rateable": len(rateable),
        "calls": result["calls"], "ratings_check": check,
        "wall_clock_s": wall, "error": error,
        "captured_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(result['calls'])} call(s), {wall}s, error={error}")
    print(f"merged: {fig['findings']} findings from {fig['runs']} run(s); "
          f"rateable {len(rateable)}")
    print(f"ratings: {check['figures']}")
    print(f"ratings check: {'clean' if check['ok'] else str(len(check['problems'])) + ' problem(s)'}")
    print(f"out: {out}/materiality.md")
    return 1 if (error or not check["ok"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
