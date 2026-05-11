#!/usr/bin/env python3
"""Frontier-model meta-task labeller for discourse-reflection pairs.

For each pair_NN.json under bench/discourse_reflect/pairs/, run a
Sonnet meta-task pass that emits gold annotations: what the dialog
window genuinely added / modified / made obsolete / left unchanged.

The labeller is given prior_discourse_state + dialog_window +
user_input_at_prior. It is NOT shown production_new_discourse_state
(would anchor the labeller on what production did rather than on
what the dialog supports).

This is meta-task labelling, not system-replay: the prompt asks
"what should be in the gold set?", not "produce the new discourse
state." The output is downstream-judging substrate, not a candidate
production output.

Provenance metadata is stamped onto each pair file (judge model,
prompt version, timestamp). Re-runs overwrite gold + provenance.

Usage:
    CLAUDE_API_KEY=... python bench/discourse_reflect/label_gold.py \\
        --pairs-dir bench/discourse_reflect/pairs \\
        --model claude-sonnet-4-6
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"
sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import _ChatBackend  # noqa: E402

logger = logging.getLogger("bench.discourse_reflect.label_gold")

DEFAULT_MODEL = "claude-sonnet-4-6"
ANTHROPIC_BASE_URL = "https://api.anthropic.com"
API_KEY_ENV = "CLAUDE_API_KEY"

PROMPT_VERSION = "meta_task_v1"

META_TASK_SYSTEM = """\
You are a labeller building a test case for a discourse-extraction \
system. Your job is NOT to produce the system's output. Your job is to \
declare the ground truth for what a correct update SHOULD contain, given \
the inputs the system saw.

The system being tested takes:
  - a PRIOR DISCOURSE STATE (a structured summary of agreements and \
decisions established earlier in the conversation), and
  - a DIALOG WINDOW (recent user/agent exchanges)
and produces an updated discourse state that should:
  (a) add agreements and decisions newly established by the dialog,
  (b) modify prior items the dialog refined or partially superseded,
  (c) remove prior items the dialog made obsolete,
  (d) preserve prior items the dialog did not affect,
  (e) keep wording concise, faithful, and self-contained.

Each agreement/decision in the discourse state carries one of two \
provenance tags:
  (this segment)        — established in the new dialog window
  (established earlier) — carried forward from prior conversation

You will produce a JSON object with these fields:

{
  "should_add_agreements": [
    {"item": "<short paraphrase of an agreement the dialog established>",
     "evidence": "<quote or close paraphrase from the dialog supporting it>"}
  ],
  "should_add_decisions": [
    {"decision": "<Adopted/Rejected/Added: short description>",
     "evidence": "<quote or close paraphrase from the dialog>"}
  ],
  "should_modify": [
    {"prior_item": "<short paraphrase of the prior-state item>",
     "modification": "<how the dialog refines, narrows, or partially supersedes it>",
     "evidence": "<quote or close paraphrase from the dialog>"}
  ],
  "should_remove": [
    {"prior_item": "<short paraphrase of the prior-state item>",
     "reason": "<why the dialog made this obsolete; e.g., 'superseded by new policy', 'parties explicitly retracted', 'topic abandoned and now stale'>"}
  ],
  "should_carry_over_unchanged": [
    {"prior_item": "<short paraphrase of a prior-state item>",
     "why_at_risk": "<why a sloppy update might silently re-word, drop, or misattribute it; e.g., 'closely related to a new agreement on X', 'wording invites re-rolling'>"}
  ],
  "labeller_notes": "<free-form notes: ambiguities, judgment calls, points where the dialog is unclear about what was truly agreed>"
}

Critical guidelines:

1. ANCHOR ON WHAT THE DIALOG SAYS. An agreement is something the parties \
genuinely converged on, not something one party asserted unilaterally. \
A decision is a discrete choice that closes a question. Casual \
acknowledgements ('interesting', 'I see') are not agreements.

2. BE CONSERVATIVE. If you're unsure whether the dialog truly \
established an item, leave it out and note the ambiguity. False positives \
in the gold set produce false-negative scores against the system.

3. should_carry_over_unchanged is NOT a full re-listing of every prior \
item. Only enumerate prior items that are *at risk* of being silently \
dropped, re-worded, or merged in a careless update — typically items \
topically adjacent to new agreements you've identified.

4. Provenance tags in the prior state are informational. Do not \
regenerate the full state. Do not attempt to re-render the section \
headers. Just produce the JSON above.

5. Output ONLY the JSON object. No prose before or after. No markdown \
code fences.
"""

META_TASK_USER_TEMPLATE = """\
PRIOR DISCOURSE STATE:
=========================
{prior_state}
=========================

DIALOG WINDOW (most recent exchange last; prior context above it):
=========================
{dialog_block}
=========================

The user's most recent input (the one that triggered the update):
{user_input}

Produce the gold-annotation JSON now.\
"""


def _format_dialog(dialog_window: List[Dict[str, str]]) -> str:
    if not dialog_window:
        return "(empty)"
    lines: List[str] = []
    for entry in dialog_window:
        src = entry.get("source", "?")
        text = (entry.get("text") or "").strip()
        if not text:
            continue
        lines.append(f"[{src}] {text}")
    return "\n\n".join(lines)


def _strip_code_fences(s: str) -> str:
    s = s.strip()
    if s.startswith("```"):
        s = re.sub(r"^```(?:json)?\s*", "", s)
        s = re.sub(r"\s*```\s*$", "", s)
    return s.strip()


def _parse_gold_json(raw: str) -> Optional[Dict[str, Any]]:
    raw = _strip_code_fences(raw)
    try:
        return json.loads(raw)
    except json.JSONDecodeError as e:
        logger.warning(f"json parse failed: {e}; raw head: {raw[:200]!r}")
        return None


def _label_one(backend: _ChatBackend, pair: Dict[str, Any]
               ) -> Optional[Dict[str, Any]]:
    """Run one meta-task labelling call. Returns the gold dict on
    success, None on failure."""
    user_msg = META_TASK_USER_TEMPLATE.format(
        prior_state=pair["prior_discourse_state"],
        dialog_block=_format_dialog(pair["dialog_window"]),
        user_input=pair["user_input_at_prior"],
    )
    messages = [
        {"role": "system", "content": META_TASK_SYSTEM},
        {"role": "user", "content": user_msg},
    ]
    try:
        raw = backend.chat(messages, max_tokens=4000, temperature=0.2)
    except Exception as e:
        logger.warning(f"backend.chat raised: {e}")
        return None
    gold = _parse_gold_json(raw)
    if gold is None:
        return None
    # Schema validation: every top-level field present.
    required = (
        "should_add_agreements", "should_add_decisions", "should_modify",
        "should_remove", "should_carry_over_unchanged", "labeller_notes",
    )
    for k in required:
        if k not in gold:
            logger.warning(f"gold missing required field: {k}")
            return None
    return gold


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Frontier meta-task labeller for discourse pairs.")
    parser.add_argument("--pairs-dir", type=Path, required=True,
                        help="Directory containing pair_NN.json files.")
    parser.add_argument("--model", type=str, default=DEFAULT_MODEL,
                        help=f"Anthropic model (default {DEFAULT_MODEL}).")
    parser.add_argument("--limit", type=int, default=0,
                        help="Process at most N pairs (0 = all). Useful "
                             "for sanity-checking the prompt on a few "
                             "before committing to the full run.")
    parser.add_argument("--skip-existing", action="store_true",
                        help="Skip pairs that already have non-empty gold "
                             "(e.g., from a partial prior run).")
    args = parser.parse_args()

    pairs_dir = args.pairs_dir.resolve()
    if not pairs_dir.is_dir():
        parser.error(f"pairs dir not found: {pairs_dir}")
    if not os.environ.get(API_KEY_ENV):
        parser.error(f"{API_KEY_ENV} not set — required for the labeller.")

    pair_paths = sorted(pairs_dir.glob("pair_*.json"))
    if not pair_paths:
        parser.error(f"no pair_*.json files in {pairs_dir}")
    if args.limit > 0:
        pair_paths = pair_paths[:args.limit]
    logger.info(f"labelling {len(pair_paths)} pairs with {args.model}")

    backend = _ChatBackend(
        server="anthropic",
        model=args.model,
        base_url=ANTHROPIC_BASE_URL,
        is_reasoning=False,
        api_key=API_KEY_ENV,
    )

    n_done = 0
    n_skipped = 0
    n_failed = 0
    failed_ids: List[str] = []
    for path in pair_paths:
        pair = json.loads(path.read_text(encoding="utf-8"))
        pid = pair.get("id", path.stem)
        existing_gold = pair.get("gold") or {}
        # "Non-empty" = at least one of the list fields has entries OR
        # labeller_notes is non-empty. Skip if so under --skip-existing.
        if args.skip_existing:
            list_fields = ("should_add_agreements", "should_add_decisions",
                           "should_modify", "should_remove",
                           "should_carry_over_unchanged")
            has_content = any((existing_gold.get(k) or [])
                              for k in list_fields) or bool(
                (existing_gold.get("labeller_notes") or "").strip())
            if has_content:
                n_skipped += 1
                logger.info(f"  {pid}: skip (existing gold)")
                continue

        logger.info(f"  {pid}: labelling...")
        gold = _label_one(backend, pair)
        if gold is None:
            n_failed += 1
            failed_ids.append(pid)
            logger.warning(f"  {pid}: FAILED (no gold written)")
            continue

        pair["gold"] = gold
        pair["gold_provenance"] = {
            "source": "frontier_meta_task",
            "model": args.model,
            "prompt_version": PROMPT_VERSION,
            "ts": datetime.now(timezone.utc).isoformat(),
        }
        path.write_text(
            json.dumps(pair, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        n_done += 1
        n_added = len(gold.get("should_add_agreements") or []) + len(
            gold.get("should_add_decisions") or [])
        n_mod = len(gold.get("should_modify") or [])
        n_rem = len(gold.get("should_remove") or [])
        n_car = len(gold.get("should_carry_over_unchanged") or [])
        logger.info(
            f"  {pid}: done — add={n_added} mod={n_mod} rem={n_rem} "
            f"carry={n_car}")

    print()
    print(f"=== Labelling summary ===")
    print(f"labelled:  {n_done}")
    print(f"skipped:   {n_skipped}")
    print(f"failed:    {n_failed}  {failed_ids if failed_ids else ''}")


if __name__ == "__main__":
    main()
