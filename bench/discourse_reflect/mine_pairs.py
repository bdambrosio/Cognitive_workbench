#!/usr/bin/env python3
"""Mine snapshot pairs for discourse-reflection scoring.

Each pair captures one production discourse-update event:
    (prior_discourse_state, dialog_window, new_discourse_state)

Source: scenarios/<world>/<character>/memory/reasoning_trace.jsonl, which
records per-turn `discourse_state`, `user_input`, and `working_log`.

How a pair maps to the production reflection call:
- prior_discourse_state = `discourse_state` field at user turn T_i
  (this is what the prompt sees at the START of T_i, i.e., the result
  of the prior reflection).
- dialog_window = the recent non-autonomous exchange leading up to and
  including T_i (approximating what `_build_dialog(limit=20)` returned
  to the discourse pass).
- new_discourse_state = `discourse_state` field at the next non-
  autonomous user turn T_{i+1} (this is what the post-T_i reflection
  wrote and what T_{i+1}'s prompt then saw).

Autonomous turns skip reflection (chat_loop.py:5027-5031), so they're
filtered out — both as pair endpoints and as dialog members.

Pairs where prior_state == new_state (no-op reflections) are dropped:
nothing for the labeller to score.

Output: pair_NN.json files under bench/discourse_reflect/pairs/ plus
manifest.json. Each pair file leaves the gold-label slots empty for
manual filling in v0.1.

Usage:
    python bench/discourse_reflect/mine_pairs.py \\
        --trace scenarios/jill_chat/Jill/memory/reasoning_trace.jsonl \\
        --out bench/discourse_reflect/pairs \\
        --n 20
"""

from __future__ import annotations

import argparse
import json
import logging
import re
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger("bench.discourse_reflect.mine_pairs")


def _load_trace(path: Path) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                out.append(json.loads(ln))
            except json.JSONDecodeError as e:
                logger.warning(f"skipping malformed jsonl line: {e}")
    return out


def _extract_jill_response(entry: Dict[str, Any]) -> str:
    """Use the trace's `raw_response` field as Jill's spoken text for
    the exchange. Returns empty string if absent (silent autonomous-
    style turn — those are filtered from the dialog window anyway)."""
    return str(entry.get("raw_response") or "").strip()


def _segment_range(state: str) -> Optional[str]:
    """Pull the 'segment X-Y' marker from a discourse state string for
    pair metadata. Returns None if not found."""
    if not state:
        return None
    m = re.search(r"DISCOURSE STATE \(segment ([\d\-]+)\)", state)
    return m.group(1) if m else None


def _agreement_count(state: str) -> int:
    """Rough count of agreement entries by counting the per-item
    provenance tags that the prompt requires on every agreement
    (`(this segment)` or `(established earlier)`). Format-agnostic
    — works for both paragraph-form (old prompt) and bullet-form
    (new prompt) outputs. Used for interestingness ranking and
    labeller-facing metadata; not used for scoring."""
    if not state:
        return 0
    m = re.search(r"CURRENT AGREEMENTS:\s*\n(.*?)(?:\n[A-Z][A-Z ]+:|\Z)",
                  state, re.DOTALL)
    if not m:
        return 0
    body = m.group(1)
    return len(re.findall(r"\((?:this segment|established earlier)\)",
                          body))


def build_pairs(trace: List[Dict[str, Any]],
                dialog_window_size: int = 20) -> List[Dict[str, Any]]:
    """Walk the trace; for each consecutive pair of non-autonomous
    user turns, build a snapshot pair. Drops no-op reflections
    (state unchanged) and pairs missing either side's discourse_state."""
    user_indices: List[int] = [
        i for i, e in enumerate(trace)
        if not e.get("autonomous") and e.get("source") == "User"
    ]
    pairs: List[Dict[str, Any]] = []
    for k, idx_i in enumerate(user_indices[:-1]):
        idx_next = user_indices[k + 1]
        e_i = trace[idx_i]
        e_next = trace[idx_next]
        prior = (e_i.get("discourse_state") or "").strip()
        new_ = (e_next.get("discourse_state") or "").strip()
        if not prior or not new_:
            continue
        if prior == new_:
            # No-op reflection — skip; nothing to score.
            continue

        # Dialog window: the last `dialog_window_size` non-autonomous
        # exchanges up to and including T_i. Each exchange contributes
        # two store entries (user + jill), so window/2 trace entries
        # ≈ window store entries.
        window_back = dialog_window_size // 2
        # Walk back from idx_i collecting non-autonomous user turns.
        prior_user_indices: List[int] = []
        j = idx_i
        while j >= 0 and len(prior_user_indices) < window_back:
            ej = trace[j]
            if not ej.get("autonomous") and ej.get("source") == "User":
                prior_user_indices.append(j)
            j -= 1
        prior_user_indices.reverse()
        dialog: List[Dict[str, str]] = []
        for j in prior_user_indices:
            ej = trace[j]
            uin = (ej.get("user_input") or "").strip()
            jresp = _extract_jill_response(ej)
            if uin:
                dialog.append({"source": "User", "text": uin})
            if jresp:
                dialog.append({"source": "Jill", "text": jresp})

        pairs.append({
            "trace_index_prior": idx_i,
            "trace_index_new": idx_next,
            "ts_prior": e_i.get("ts"),
            "ts_new": e_next.get("ts"),
            "prior_segment": _segment_range(prior),
            "new_segment": _segment_range(new_),
            "prior_agreement_count": _agreement_count(prior),
            "new_agreement_count": _agreement_count(new_),
            "user_input_at_prior": (e_i.get("user_input") or "").strip(),
            "dialog_window": dialog,
            "prior_discourse_state": prior,
            "production_new_discourse_state": new_,
        })
    return pairs


def stratified_sample(pairs: List[Dict[str, Any]], n: int
                      ) -> List[Dict[str, Any]]:
    """Sample n pairs across the full conversation depth, with light
    bias toward pairs where the agreement count actually changed
    (more interesting for labellers — pure reorderings are less
    informative)."""
    if not pairs:
        return []
    if len(pairs) <= n:
        return pairs

    # Partition into "delta-meaningful" (count changed) and "delta-zero"
    # (count same; may still be content updates but lower yield).
    delta = [p for p in pairs
             if p["prior_agreement_count"] != p["new_agreement_count"]]
    zero = [p for p in pairs
            if p["prior_agreement_count"] == p["new_agreement_count"]]

    # Allocate: prefer delta-meaningful when available, fill rest from zero.
    n_delta = min(len(delta), max(int(n * 0.7), n - len(zero)))
    n_zero = n - n_delta
    sampled: List[Dict[str, Any]] = []
    for bucket, take in ((delta, n_delta), (zero, n_zero)):
        if not bucket or take <= 0:
            continue
        # Even stride across the bucket so we span conversation depth.
        if take >= len(bucket):
            sampled.extend(bucket)
        else:
            stride = len(bucket) / take
            picks = [bucket[int(i * stride)] for i in range(take)]
            sampled.extend(picks)
    sampled.sort(key=lambda p: p["trace_index_prior"])
    return sampled


def write_pairs(pairs: List[Dict[str, Any]], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest: List[Dict[str, Any]] = []
    for i, p in enumerate(pairs, start=1):
        pair_id = f"pair_{i:02d}"
        # Add empty gold-label slots for manual filling. Per the
        # update-aware scoring axes (add-recall, add-precision,
        # carryover-preservation, prune-correctness, update-correctness,
        # wording), the labeller's job is to declare what the correct
        # update should look like, then a separate scoring pass compares
        # production_new_discourse_state to that gold.
        record = {
            "id": pair_id,
            **p,
            "gold": {
                "should_add_agreements": [],
                "should_add_decisions": [],
                "should_modify": [],
                "should_remove": [],
                "should_carry_over_unchanged": [],
                "labeller_notes": "",
            },
        }
        path = out_dir / f"{pair_id}.json"
        path.write_text(json.dumps(record, indent=2, ensure_ascii=False),
                        encoding="utf-8")
        manifest.append({
            "id": pair_id,
            "trace_index_prior": p["trace_index_prior"],
            "trace_index_new": p["trace_index_new"],
            "ts_prior": p["ts_prior"],
            "prior_segment": p["prior_segment"],
            "new_segment": p["new_segment"],
            "prior_agreement_count": p["prior_agreement_count"],
            "new_agreement_count": p["new_agreement_count"],
            "user_input_preview": p["user_input_at_prior"][:120],
        })
    (out_dir.parent / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Mine discourse-reflection snapshot pairs.")
    parser.add_argument("--trace", type=Path, required=True,
                        help="Path to reasoning_trace.jsonl")
    parser.add_argument("--out", type=Path, required=True,
                        help="Output dir for pair JSON files")
    parser.add_argument("--n", type=int, default=20,
                        help="Number of pairs to sample (default 20)")
    parser.add_argument("--window", type=int, default=20,
                        help="Dialog-window size to reconstruct "
                             "(approximates _build_dialog(limit=N))")
    args = parser.parse_args()

    trace_path = args.trace.resolve()
    if not trace_path.is_file():
        parser.error(f"trace file not found: {trace_path}")

    logger.info(f"loading {trace_path}")
    trace = _load_trace(trace_path)
    logger.info(f"loaded {len(trace)} trace entries "
                f"({sum(1 for e in trace if not e.get('autonomous')
                       and e.get('source') == 'User')} non-autonomous user)")

    pairs = build_pairs(trace, dialog_window_size=args.window)
    logger.info(f"built {len(pairs)} eligible snapshot pairs (after "
                f"dropping no-ops and missing-state turns)")

    sampled = stratified_sample(pairs, n=args.n)
    logger.info(f"sampled {len(sampled)} pairs")

    write_pairs(sampled, args.out.resolve())
    logger.info(f"wrote {len(sampled)} pair files to {args.out}")
    logger.info(f"manifest at {args.out.parent / 'manifest.json'}")


if __name__ == "__main__":
    main()
