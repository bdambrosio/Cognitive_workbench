#!/usr/bin/env python3
"""Does the claim-division rule change how many claims a model enumerates?

Enumeration on doc1 went from 20 claims under v1 to 33-36 under v2, and that
was attributed to the freeze — v2 enumerates before it adjudicates, so the
model is not choosing what counts as a claim with the verdicts in view. The
attribution is confounded: the DIVISION RULE changed in the same edit, and it
changed toward more claims.

  v1  "Do not combine separate assertions merely because they concern the same
       subject, and do not split a single assertion into artificial subclaims."
      Two prohibitions, no test, no example.

  v2  "Split assertions apart where different evidence could give them
       different verdicts. Keep them together where they stand or fall on the
       same evidence." Plus one worked example, which splits.

  v3  v2's test, with the anti-split prohibition restored and a second worked
      example that keeps two assertions together.

WHAT THIS HOLDS FIXED, AND WHY THAT IS THE WHOLE DESIGN. Running v1's METHOD
against v2's would compare two documents that differ in format, freeze, schema
and vocabulary at once, and attribute the result to whichever one you had in
mind. So every arm here is v2's METHOD with ONE PARAGRAPH swapped. Same model,
same claim source, same schema, same phase-one prompt the runner sends. The
only thing that varies is the rule.

REPLICATES ARE NOT OPTIONAL. Three v2 runs on doc1 enumerated 36, 33 and 30 —
a 20% spread with the text held constant. An arm run once measures that spread
and nothing else.

    python3 measure/claim_granularity/run.py --model measure/models/local_qwen38flashnext.yaml
    python3 measure/claim_granularity/run.py --reps 5 --doc doc2_tech_stack_description_as_provided_by_se.md
"""
from __future__ import annotations

import argparse
import json
import statistics as st
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

from chat.workflow import load_workflow                        # noqa: E402
from utils.json_utils import repair_json_string                # noqa: E402
from workflowsv2.claims_audit import schemas                   # noqa: E402

METHOD = REPO / "workflowsv2/claims_audit/method/METHOD.md"
CORPUS = REPO / "measure/fixtures/dataroom/corpus"

# The paragraph as it stands in METHOD §5. Swapped for each arm's text.
CURRENT = """**Where to divide the text into claims.** Split assertions apart where different evidence could give them different verdicts. Keep them together where they stand or fall on the same evidence. **Do not split one assertion into artificial parts**, and do not combine separate assertions because they share a subject. Both are faults, and the second is not worse than the first.

> "Backups run daily with 30-day retention" is **two** claims. The schedule and the retention period are settled by different evidence, and one can hold while the other fails. Sharing a sentence is not a reason to combine.
>
> "Platform-level redundancy and automatic failover" is **one** claim. Both stand or fall on whether replicas exist, and one line of the infrastructure config settles both. Sharing a subject is not, by itself, a reason to combine — but being settled by the same evidence is."""

ARMS = {
    # v1's rule, verbatim from workflows/claims_audit/method/METHOD.md §12.2.
    "v1_prohibitions": """**Where to divide the text into claims.** A claim is one seller assertion that can be evaluated as true or false. Do not combine separate assertions merely because they concern the same subject, and do not split a single assertion into artificial subclaims.""",
    # v2 as first shipped: the test, and one example, which splits.
    "v2_test_one_example": """**Where to divide the text into claims.** Split assertions apart where different evidence could give them different verdicts. Keep them together where they stand or fall on the same evidence.

Sharing a subject is not a reason to combine: "backups run daily" and "backups are retained 30 days" are settled by different evidence and are two claims.""",
    # v3: the test, the prohibition restored, and an example each way.
    "v3_symmetric": CURRENT,
}


def prompt(doc: str) -> str:
    """Phase one's user message, as `claims_audit/runner.py:emit_surface` sends
    it. Copied rather than imported because the runner builds it around a live
    ChatLoop; if that message changes, this must be re-checked against it."""
    lines = (CORPUS / doc).read_text(encoding="utf-8").splitlines()
    numbered = "\n".join(f"{n}|{t}" for n, t in enumerate(lines, 1))
    return (f"The claim source for this run is `{doc}`. Its text, with the "
            f"line numbers your citations refer to:\n\n{numbered}\n\n"
            f"Emit the claim surface now, per METHOD §5 and §13. You "
            f"have gathered no evidence and formed no verdicts; enumerate "
            f"every assertion the seller makes about the target.")


def method_for(arm: str) -> str:
    """v2's METHOD with one paragraph swapped, then stripped for the prompt."""
    raw = METHOD.read_text(encoding="utf-8")
    if CURRENT not in raw:
        raise SystemExit(
            "METHOD §5's division paragraph has changed since this probe "
            "was written. Update CURRENT to match it before trusting any arm — "
            "a swap that silently did nothing would report the same text three "
            "times as three arms.")
    swapped = REPO / "measure/claim_granularity/_arm.md"
    swapped.write_text(raw.replace(CURRENT, ARMS[arm]), encoding="utf-8")
    try:
        return load_workflow(swapped)
    finally:
        swapped.unlink(missing_ok=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="measure/models/local_qwen38flashnext.yaml")
    ap.add_argument("--doc", default="doc1_seller_listing_description.md")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--max-tokens", type=int, default=32768)
    ap.add_argument("--out", default="measure/claim_granularity/results.jsonl")
    args = ap.parse_args()

    doc = yaml.safe_load(Path(args.model).read_text(encoding="utf-8")) or {}
    llm = dict(doc.get("llm_config") or {})
    if not llm:
        raise SystemExit(f"{args.model}: no llm_config block")
    from chat.backend import _ChatBackend                       # noqa: E402
    import os
    # Same construction as measure/regrade.py's. The model yaml names an env
    # var for its key rather than the key; a local server names none.
    key = llm.get("api_key")
    backend = _ChatBackend(
        server=llm.get("server", "local"),
        model=llm.get("model", ""),
        base_url=llm.get("vllm_url") or llm.get("base_url", ""),
        is_reasoning=llm.get("is_reasoning_model"),
        api_key=(os.environ.get(key) if key and key.isupper() else key),
        reasoning_effort=llm.get("reasoning_effort"),
        extra_body=llm.get("extra_body"))
    user = prompt(args.doc)
    src = CORPUS / args.doc
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = []

    for arm in ARMS:
        system = method_for(arm)
        for rep in range(1, args.reps + 1):
            t0 = time.time()
            raw = backend.chat([{"role": "system", "content": system},
                                {"role": "user", "content": user}],
                               max_tokens=args.max_tokens,
                               response_schema=schemas.surface_schema())
            dt = round(time.time() - t0, 1)
            try:
                obj = json.loads(raw)
            except Exception:                                  # noqa: BLE001
                obj = repair_json_string(raw)
            claims = (obj or {}).get("claims") or []
            check = (schemas.check_surface(obj, CORPUS, args.doc)
                     if isinstance(obj, dict) else
                     {"ok": False, "problems": ["unparseable"]})
            # Span in lines, so a rise in the count can be told from the same
            # text carved finer: more claims over the same lines is division,
            # more lines covered is reach.
            covered = set()
            for c in claims:
                ln = c.get("lines")
                if isinstance(ln, list) and len(ln) == 2 \
                        and all(isinstance(n, int) for n in ln):
                    covered.update(range(ln[0], ln[1] + 1))
            row = {"arm": arm, "rep": rep, "doc": args.doc,
                   "model": backend.resolved_model(),
                   "claims": len(claims), "lines_covered": len(covered),
                   "check_ok": check["ok"],
                   "problems": len(check.get("problems") or []),
                   "secs": dt, "chars": len(raw or "")}
            rows.append(row)
            with out.open("a") as fh:
                fh.write(json.dumps(row) + "\n")
            print(f"  {arm:22s} rep{rep}  claims={len(claims):3d}  "
                  f"lines={len(covered):3d}  check={'ok' if check['ok'] else 'FAIL'}"
                  f"  {dt}s")

    print(f"\n{args.doc}, {len(src.read_text().splitlines())} lines, "
          f"{backend.resolved_model()}\n")
    print(f"{'arm':24s} {'claims':>18s}  {'lines covered':>14s}")
    for arm in ARMS:
        cs = [r["claims"] for r in rows if r["arm"] == arm]
        ls = [r["lines_covered"] for r in rows if r["arm"] == arm]
        if not cs:
            continue
        print(f"{arm:24s} {st.median(cs):6.0f} ({min(cs)}-{max(cs)})   "
              f"{st.median(ls):6.0f} ({min(ls)}-{max(ls)})")
    print("\nMedian and range over replicates. Three runs of the SAME text on "
          "doc1 gave 36, 33 and 30, so an arm whose range overlaps another's "
          "has not been shown to differ.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
