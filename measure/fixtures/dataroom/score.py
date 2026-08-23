#!/usr/bin/env python3
"""Score a dataroom run against the answer key.

    python3 measure/fixtures/dataroom/score.py --world dataroom_1 --agent Jill
    python3 measure/fixtures/dataroom/score.py --world dataroom_1 --dry-run

Reports a VECTOR, and reports the highest difficulty tier reached rather
than a pass/fail — a fixture that everyone passes is not an instrument, and
the ladder is what stops this one saturating.

WHAT IS MECHANICAL AND WHAT IS NOT. Which documents were opened, how many
iterations ran, whether the word ceiling held, whether the key was read —
all read straight off the trace, no model involved. Deciding whether a
paragraph in the memo *is* finding P2 is a judgement about meaning, so it
runs through ONE LLM call on the pinned grader, held constant across runs
for the same reason `regrade.py` pins it: if the arm under test also did the
matching, a difference in matching quality would surface as the metric.

THE KEY IS READABLE BY THE AGENT. `inspect` is geofenced to the repo root
(176b4029), so `answer_key.md` sits inside reach. Rather than hide it, this
scorer looks for it in the trace and marks the run INVALID if it was opened.
That is a third outcome, not a zero: a run that read the answers measured
nothing, and reporting it as a bad score would be worse than reporting
nothing.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from measure.trace import load_turns                            # noqa: E402

CORPUS_DOCS = 9

# TWO DIFFERENT AXES, and conflating them was the first bug in this file.
#
# Axis 1 — PROVENANCE of the finding (the answer key's own grouping):
#   planted / derived / supported-but-unplanted. Tier 3 is credited and
#   NEVER penalised, because entries 4-5 of the original exchange were
#   discovered rather than seeded — an agent finding something real that
#   nobody planted is being correct, and a grader that calls that a false
#   positive is measuring the wrong thing.
TIER1 = ["P1", "P2", "P3"]                       # planted, must-find
TIER2 = ["F1", "F2"]                             # derived, planted
TIER3 = ["B1", "B2", "B3", "B4", "B5", "B6", "B7"]   # unplanted, supported

# Axis 2 — DIFFICULTY of noticing it (the README's ladder). This is the one
# that keeps the fixture from saturating: report the highest rung reached
# rather than pass/fail, and retire the fixture when every arm tops out.
#   stated         a fact sitting in one document
#   cross_document a seller claim contradicted by another document
#   derived        true only after arithmetic on stated figures
LADDER = {
    "stated": ["B3", "B4", "B5"],
    "cross_document": ["P1", "P2", "P3", "B1", "B2", "B6"],
    "derived": ["F1", "F2", "B7"],
}
RUNGS = ["stated", "cross_document", "derived"]

# From the key. Placement is scored against this, because for a one-pager
# the forced choice about what earns space IS the measurement.
SEVERITY = ["P2", "F2", "P3", "P1", "F1", "B3", "B2", "B1", "B7", "B5",
            "B4", "B6"]

FINDINGS_BRIEF = """\
P1 vocabulary_gap — seller claims "redundancy and automatic failover" (Doc 9);
   Doc 4 shows a single standard-1x dyno with Postgres on the SAME dyno, no
   read replicas. A dyno restart is not failover.
P2 operational_degradation — seller claims "daily automated database backups"
   (Doc 9); Doc 4 records backup failures for the last 21 days, last success
   2026-07-30, alerting none configured.
P3 structural_omission — seller claims all integrations are standard with no
   lock-in and never names DataEnrich.io (Docs 1/2/9); Doc 8 shows
   DataEnrich.io at $400/mo, 40% of features depend on it, 90-day termination,
   no fallback.
F1 derived — Stripe MRR $16,000 (Doc 5) vs three enterprise contracts at
   $8,000/mo paid by wire and not in Stripe (Doc 7): ~60% of revenue is
   unverifiable from the processor; wire revenue is 150% of Stripe MRR.
   (28% is NOT this finding.)
F2 derived — 30-day retention against 21 days of failures puts the last
   recoverable backup (2026-07-30) at total loss around 2026-08-29.
B1 "comprehensive test coverage" vs 12 unit tests all in test/utils/, 0
   integration, 0 payment-path, no CI/CD, no staging, no review.
B2 "we monitor uptime at 99.9%" vs Doc 4 "Uptime Monitor: None", while two
   customers hold 99.9% SLAs with credits.
B3 key person — dave is 73% of commits, 0 in 30 days, last commit 2026-07-30;
   DNS managed personally by dave with no secondary.
B4 SLA heterogeneity — ShopStream at 99.5% where the other two are 99.9%.
B5 legal — 15 accounts with no recorded ToS acceptance; 8 pilots expiring Q3
   2026 with no payment obligation.
B6 "scale horizontally, adding dynos is one-click" vs the co-located database,
   which adding dynos does not scale.
B7 concentration — 3 customers are ~60% of revenue; GlobalMart renews first
   (2026-11-01), inside the 90-day non-renewal notice window.
"""

_MATCH_SYS = """\
You are scoring a due-diligence memo against a fixed answer key. You are not \
judging quality, only identity: which of the key's findings does each of the \
memo's findings correspond to?

Here is the key:

{findings}

Read the memo. For EACH distinct finding the memo reports, in the order the \
memo presents them, output one entry:
- "id": the key id it matches (P1 P2 P3 F1 F2 B1..B7), or "unsupported" if \
the memo asserts something the data room does not establish, or "other" if \
it is a real observation that matches no key entry.
- "quote": a short verbatim span from the memo identifying that finding.

Match on SUBSTANCE, not wording. A memo saying "the database shares a dyno \
with the app so restarting it is not real failover" is P1 whatever words it \
uses. Do NOT match on a topic being mentioned in passing — the memo must \
actually report it as a finding. If the memo mentions backups only to say \
they exist, that is not P2.

A finding may match only one id. Do not invent ids.

Output ONLY: {{"findings": [{{"id": "...", "quote": "..."}}]}}"""


GAP_MARK = "=== GAP MAP ==="


def read_memo(world: str, agent: str) -> Optional[str]:
    """The report is the last substantial reply, with the Gap Map removed.

    The Gap Map repeats the top findings by design, so matching over both
    would count them twice and inflate recall. It is reported separately —
    presence and length — rather than scored."""
    turns = [t for t in load_turns(world, agent) if t.produced_chars > 400]
    if not turns:
        return None
    return str(turns[-1].raw.get("raw_response") or "")


def split_deliverables(reply: str) -> tuple:
    """(report, gap_map_or_None). Split on the marker the brief specifies."""
    if GAP_MARK in reply:
        head, _, tail = reply.partition(GAP_MARK)
        return head.rstrip(), tail.strip()
    return reply, None


def trace_facts(world: str, agent: str) -> Dict[str, Any]:
    """Everything computable without a model."""
    turns = load_turns(world, agent)
    log = "\n".join(t.working_log for t in turns)
    docs = set(re.findall(r'doc(\d)_[a-z0-9_]+\.md', log))
    return {
        "turns": len(turns),
        "iterations": sum(t.iterations for t in turns),
        "docs_opened": sorted(int(d) for d in docs),
        "exit_reasons": {},
        # The run is void if the answers were in reach and got read.
        "read_answer_key": "answer_key" in log,
    }


def word_count(memo: str) -> int:
    return len(re.findall(r"\S+", memo))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True)
    ap.add_argument("--agent", default="Jill")
    ap.add_argument("--dry-run", action="store_true",
                    help="mechanical metrics only; no LLM call, no cost")
    args = ap.parse_args()

    facts = trace_facts(args.world, args.agent)
    memo = read_memo(args.world, args.agent)

    print(f"\n=== dataroom — {args.world}/{args.agent} ===")
    if facts["read_answer_key"]:
        print("  MEASUREMENT INVALID — the trace shows answer_key was opened.")
        print("  This is not a zero. The run measured nothing; discard it.")
        return 2
    if not memo:
        print("  no memo found (no reply over 400 chars)")
        return 1

    report, gap_map = split_deliverables(memo)
    memo = report          # findings are matched over the report only
    wc = word_count(report)
    print(f"  turns {facts['turns']}   iterations {facts['iterations']}")
    if gap_map is None:
        print("  Gap Map            ABSENT — the brief asked for one")
    else:
        print(f"  Gap Map            present, {word_count(gap_map)} words")
    print(f"  corpus docs opened  {len(facts['docs_opened'])}/{CORPUS_DOCS}  "
          f"{facts['docs_opened']}")
    print(f"  report length       {wc} words  "
          f"({'within' if wc <= 2000 else 'OVER'} the 2,000 guide)")

    if args.dry_run:
        print("  dry run — no finding match attempted.")
        return 0

    from measure.regrade import make_grader
    _, llm = make_grader()
    raw = llm([
        {"role": "system",
         "content": _MATCH_SYS.format(findings=FINDINGS_BRIEF)},
        {"role": "user", "content": memo},
    ])
    # repair_json_string already returns the parsed dict, not a string —
    # json.loads on its result raises TypeError. Guard shape the way the
    # retired bench/common.py did, for the same reason.
    from utils.json_utils import repair_json_string
    parsed = repair_json_string(raw) if isinstance(raw, str) else raw
    if not isinstance(parsed, dict) or "findings" not in parsed:
        print(f"  match pass returned no usable JSON: {str(raw)[:200]}")
        return 1
    got = parsed["findings"]
    if not isinstance(got, list):
        print(f"  match pass 'findings' was {type(got).__name__}, not a list")
        return 1

    order = [f.get("id") for f in got]
    found = set(order)
    t1 = [x for x in TIER1 if x in found]
    t2 = [x for x in TIER2 if x in found]
    t3 = [x for x in TIER3 if x in found]
    unsupported = order.count("unsupported")

    # Difficulty ladder — the anti-saturation measure. Highest rung with at
    # least one finding on it, reported alongside the count on every rung so
    # a single lucky hit is visible as one.
    rungs = {r: [x for x in LADDER[r] if x in found] for r in RUNGS}
    top = next((r for r in reversed(RUNGS) if rungs[r]), None)
    print(f"\n  ladder rung reached {top or 'none'}")
    for r in RUNGS:
        print(f"      {r:15} {len(rungs[r])}/{len(LADDER[r])}  {rungs[r]}")

    print(f"\n  Tier 1 (must-find)  {len(t1)}/3  {t1}")
    print(f"  Tier 2 (derived)    {len(t2)}/2  {t2}")
    print(f"  Tier 3 (credit)     {len(t3)}    {t3}")
    print(f"  unsupported claims  {unsupported}"
          f"   (only these are penalised)")

    ranked = [x for x in order if x in SEVERITY]
    if ranked:
        top = ranked[0]
        print(f"\n  memo's top finding  {top}  "
              f"(key ranks it #{SEVERITY.index(top) + 1} of {len(SEVERITY)})")
        ideal = [x for x in SEVERITY if x in found]
        print(f"  placement           memo order {ranked[:6]}")
        print(f"                      key order  {ideal[:6]}")
    print("\n  n=1 is anecdote. Run the fixture at least three times per arm.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
