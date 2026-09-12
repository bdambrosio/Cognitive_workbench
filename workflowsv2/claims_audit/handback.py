#!/usr/bin/env python3
"""Hand claims back to adjudication after the review pointed at uncited lines.

    python3 workflowsv2/claims_audit/handback.py --run <audit run dir> \
        --model measure/models/fw_glm53flash.yaml [--world NAME] [--batch 10]

The review's adverse-recall check (audit_review/runner.py, `adverse_recall`)
reads the evidence requests filed under each finding rated `real` and names,
per claim, lines that are in the audit's own record and not cited. This step
re-adjudicates exactly those claims, the way the audit's chase does (METHOD
§10): one schema-constrained call per batch, no tools, the full evidence
requests filed under the claims, the previous adjudication in view, and a
note. The note names the LOCATION the review pointed at and nothing else. The
review's reading of what the lines mean stays out of the prompt, so the
auditor judges the lines rather than answering the reviewer (Bruce,
2026-09-12).

WHY A NEW RUN DIRECTORY. The audit has finished when the review finds the
hits, and its findings.json is the record the review judged. The rewritten
findings go into a copy, `<run>_handback_<ts>`, with `review/` left out: the
new text has not been reviewed, and the review runner is run on the copy next.
The merge stage pins a run by directory, so the copy is a run like any other.

WHY NO AGENT. Adjudication never used one: gathering is the agent's job, and
the lines the review points at are already in the requests filed under the
claim. `evidence_batches` sizes batches on the full form, so the pointed-at
lines are in front of the call; the trimmed form is what would drop them.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import shutil
import sys
from pathlib import Path
from typing import Any, Dict, List

REPO = Path(__file__).resolve().parents[2]
for p in (REPO / "src", REPO):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from chat.workflow import load_workflow                          # noqa: E402
from workflowsv2.claims_audit import runner as audit             # noqa: E402

logger = logging.getLogger("handback")


def hits_from_review(run: Path) -> List[Dict[str, Any]]:
    recall = run / "review" / "adverse_recall.json"
    if not recall.is_file():
        raise SystemExit(f"{recall} not found: run the review first")
    rows = json.loads(recall.read_text(encoding="utf-8")).get("rows") or []
    return [r for r in rows if r.get("uncited_adverse") and r.get("claim_id")]


def note_for(obj: Dict[str, Any], hits: List[Dict[str, Any]], ids: List[int]) -> str:
    where = {h["claim_id"]: h.get("where") or "" for h in hits}
    lines = ["The review read the evidence requests filed under the claims "
             "below and found, in each, lines that are in your own record "
             "and that the finding does not cite. Nothing new has been "
             "gathered. Adjudicate these claims again on the evidence "
             "below. For each, either cite the lines named and give the "
             "verdict METHOD §6 requires with them cited, or cite them "
             "and say in `shows` why they do not bear on the claim. The "
             "adjudication each carries now:", "",
             audit.previous_adjudications(obj, ids), "",
             "The lines the review named, per claim:"]
    for cid in ids:
        lines.append(f"  claim {cid}: {where.get(cid, '(not named)')}")
    lines += ["", "Where the verdict changes, say in `correction` what "
                  "changed and why, in one line (METHOD §10)."]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, help="a reviewed audit run directory")
    ap.add_argument("--model", help="YAML with an llm_config block")
    ap.add_argument("--world", help="fresh world name; defaults from the run")
    ap.add_argument("--batch", type=int, default=10,
                    help="claims per adjudication call at most (default 10)")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    run = Path(args.run).resolve()
    meta = json.loads((run / "run_meta.json").read_text(encoding="utf-8"))
    frozen = json.loads((run / "claims.json").read_text(encoding="utf-8")).get("claims") or []
    obj = json.loads((run / "findings.json").read_text(encoding="utf-8"))
    hits = hits_from_review(run)
    if not hits:
        print("no uncited-adverse hits in the review; nothing to hand back")
        return 0
    ids = sorted({int(h["claim_id"]) for h in hits})
    by_id = {c.get("id"): c for c in frozen}
    missing = [c for c in ids if c not in by_id]
    if missing:
        raise SystemExit(f"hit claim(s) not on the frozen surface: {missing}")

    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    out = run.parent / f"{run.name}_handback_{ts}"
    shutil.copytree(run, out, ignore=shutil.ignore_patterns("review", "findings.partial.json"))
    target = Path(meta["external_repo"])
    claim_source = obj.get("claim_source") or (meta.get("surface") or {}).get("claim_source")
    src_doc = target / claim_source
    traces_dir = out / "working_record" / "inspect_traces"
    excludes = list(meta.get("evidence_excludes") or [])
    budget = int(meta.get("evidence_budget") or audit.EVIDENCE_BUDGET)

    world = args.world or f"handback_{run.name[-40:]}_{ts}"
    name, cfg = audit.build_config(world, Path(args.model) if args.model else None,
                                   None, None, None, target, excludes)
    cfg["subagent_map"] = False
    from chat.chat_loop import ChatLoop                        # noqa: E402
    loop = ChatLoop(character_name=name, character_config=cfg)
    method_text = load_workflow(REPO / audit.METHOD_PATH)
    max_tokens = int((cfg.get("chat") or {}).get("react_max_tokens", 32768))
    logger.info("hand-back of %d claim(s) %s from %s -> %s; model %s",
                len(ids), ids, run.name, out.name, loop.backend.resolved_model())

    index = audit.trace_index(traces_dir)
    batches = audit.evidence_batches(ids, index, args.batch, budget)
    log = []
    replaced = 0
    try:
        for bi, b in enumerate(batches, 1):
            logger.info("batch %d/%d: claims %s, %d trace(s)", bi, len(batches),
                        b["claims"], len(b["traces"]))
            again = audit.emit_findings(
                loop, method_text=method_text, claim_source=src_doc,
                frozen=[by_id[c] for c in b["claims"]], traces=b["traces"],
                max_tokens=max_tokens, evidence_budget=budget,
                note=note_for(obj, hits, b["claims"]))
            n = audit.replace_findings(obj, again, set(b["claims"]))
            replaced += n
            log.append({"claims": b["claims"], "traces": len(b["traces"]),
                        "parse": again.get("parse"), "finish": again.get("finish"),
                        "evidence": again.get("evidence"), "replaced": n})
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)

    (out / "findings.json").write_text(
        json.dumps(obj, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    checks = audit.post_run_checks(
        obj, target, claim_source, frozen, out,
        read=set(audit.files_read(traces_dir, target)), excludes=excludes)
    meta["hand_back"] = {
        "source_run": run.name, "review_hits": ids, "batches": log,
        "replaced": replaced, "world": world,
        "model_config": args.model, "resolved_model": loop.backend.resolved_model(),
        "captured_at_utc": ts,
    }
    meta["output_check"] = checks
    (out / "run_meta.json").write_text(json.dumps(meta, indent=2, default=str) + "\n",
                                       encoding="utf-8")
    changed = [(f["claim_id"], f["adjudication"].get("verdict"),
                (f.get("correction") or "")[:80])
               for f in obj.get("findings") or [] if f.get("claim_id") in ids]
    print(f"\nhand-back: {replaced} of {len(ids)} finding(s) replaced; "
          f"output check problems={len(checks.get('problems') or [])}")
    for cid, v, corr in changed:
        print(f"  claim {cid}: {v}" + (f"  correction: {corr}" if corr else ""))
    print(f"\nrun dir: {out}\nnext: python3 workflowsv2/audit_review/runner.py "
          f"--run {out} --model {args.model or '<model yaml>'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
