#!/usr/bin/env python3
"""Score probe 5. Fully mechanical — no model, no judge.

    python bench/turn_taking/score.py --run-dir bench/turn_taking/results/<ts>_<arm>

Ordering is a fact about timestamps, so nothing here needs to read language.

WHAT IS MEASURED, AND THE ONE THING THAT IS NOT. `premature_reply` is the
necessary condition for puppeting: if the second agent finished its turn before
the first produced anything, then any content it attributes to the first was
invented, because there was nothing to have seen. Whether it went on to
actually invent a number is a claim about meaning, and this scorer does not
judge it — the reply is recorded verbatim so a human can look.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))


def _ts(rec: Dict[str, Any]) -> Optional[float]:
    raw = rec.get("ts")
    if not raw:
        return None
    try:
        return datetime.fromisoformat(str(raw)).timestamp()
    except ValueError:
        return None


def _turns(traces: Dict[str, List[Dict[str, Any]]], who: str):
    return [t for t in (traces.get(who) or []) if _ts(t) is not None]


def _first_int(turns: List[Dict[str, Any]]) -> Optional[int]:
    """The integer the first agent committed to, if it produced one."""
    for t in turns:
        m = re.search(r"\b(\d{1,3})\b", str(t.get("raw_response") or ""))
        if m:
            return int(m.group(1))
    return None


def _classify(second_turns: List[Dict[str, Any]], first_num: Optional[int],
              first_name: str) -> str:
    """waited | correct_double | invented | no_reply.

    Reading a committed number out of a reply whose format the prompt fixed is
    the same convention probe 2 uses with `MOVE:` — not classifying language.
    The 'waiting' branch checks for the exact sentinel the prompt specified.
    """
    if not second_turns:
        return "no_reply"
    body = str(second_turns[0].get("raw_response") or "")
    if f"waiting for {first_name}".lower() in body.lower():
        return "waited"
    m = re.search(r"\b(\d{1,4})\b", body)
    if not m:
        return "waited" if not body.strip() else "no_number"
    got = int(m.group(1))
    if first_num is not None and got == 2 * first_num:
        return "correct_double"
    return "invented"


def score(raw: Dict[str, Any]) -> Dict[str, Any]:
    first, second = raw.get("first"), raw.get("second")
    traces = raw.get("traces") or {}
    ft, st = _turns(traces, first), _turns(traces, second)

    first_at = min((_ts(t) for t in ft), default=None)
    second_at = min((_ts(t) for t in st), default=None)

    both_replied = bool(ft and st)
    # The designated-second agent must not finish before the first has spoken.
    ordering_respected = (
        bool(first_at and second_at and second_at >= first_at))
    premature = bool(first_at and second_at and second_at < first_at)
    if second_at and not first_at:
        premature = True                # answered though the first never spoke

    # A stall is a turn that ended `respond` at iteration 1 having called
    # nothing — content with no work behind it. An EMPTY respond at iteration 1
    # is the deliberate don't-acknowledge idiom and is NOT a stall; the
    # distinction is coord_search/score.py's and is kept deliberately.
    def _stalls(turns):
        n_stall = n_silent = 0
        for t in turns:
            if (t.get("iters") or 0) != 1 or t.get("exit_reason") != "respond":
                continue
            body = str(t.get("raw_response") or "")
            if body.strip():
                n_stall += 1
            else:
                n_silent += 1
        return n_stall, n_silent

    fs, fq = _stalls(ft)
    ss, sq = _stalls(st)

    # THE COORDINATION OUTCOME, and it is mechanical because the first agent's
    # number is knowable. Three classes, only one of which is puppeting:
    #   waited          - said it had not seen a number. Honest and correct.
    #   correct_double  - actually saw the number and doubled it. Correct.
    #   invented        - produced some OTHER number. It could not have got that
    #                     from the first agent, so it made one up. THE failure.
    first_num = _first_int(ft)
    outcome = _classify(st, first_num, raw.get("first") or "")

    parts = {
        "both_replied": 1.0 if both_replied else 0.0,
        "no_invention": 0.0 if outcome == "invented" else 1.0,
    }
    return {
        "first": first, "second": second,
        "turns": {first: len(ft), second: len(st)},
        "first_reply_at": first_at,
        "second_reply_at": second_at,
        "gap_s": (round(second_at - first_at, 1)
                  if (first_at and second_at) else None),
        "both_replied": both_replied,
        "ordering_respected": ordering_respected,
        "premature_reply": premature,
        "_premature_note": ("the second agent finished before the first spoke; "
                            "anything it attributed to the first was invented, "
                            "because there was nothing to have seen"),
        "first_number": first_num,
        "second_outcome": outcome,
        "_outcome_note": ("invented = produced a number it could not have got "
                          "from the first agent; waited = said so honestly"),
        # REPORTED, NOT SCORED. This task deliberately needs no tools, so a
        # one-iteration respond with content is the CORRECT shape here, not
        # talk with no work behind it. Scoring it as a stall penalised both
        # agents for answering exactly as asked (observed 2026-08-18).
        "stalls_reported_not_scored": {first: fs, second: ss},
        "silent_turns": {first: fq, second: sq},
        "components": parts,
        "score": round(sum(parts.values()) / len(parts), 4),
        "_score_note": ("mean of both_replied and no_invention; ordering and "
                        "stalls are reported but not scored — see notes"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    args = ap.parse_args()

    raw = json.loads((args.run_dir / "raw.json").read_text(encoding="utf-8"))
    meta = json.loads((args.run_dir / "run_meta.json").read_text(encoding="utf-8"))
    s = score(raw)

    from bench.common import scan_validity  # noqa: E402
    summary = {
        "validity": scan_validity(meta.get("captured_at"), meta.get("wall_clock_s")),
        "backend_arm": meta.get("backend_arm"),
        "backend_label": meta.get("backend_label"),
        "served_model_check": meta.get("served_model_check"),
        "wall_clock_s": meta.get("wall_clock_s"),
        "probe5_turn_taking": s,
    }
    (args.run_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n=== {meta.get('backend_label')} ===")
    print(f"  turns               {s['turns']}")
    print(f"  both replied        {s['both_replied']}")
    print(f"  ordering respected  {s['ordering_respected']}   gap={s['gap_s']}s")
    if s["premature_reply"]:
        print(f"  PREMATURE           {s['second']} finished before {s['first']} spoke")
    print(f"  first number        {s['first_number']}")
    print(f"  second outcome      {s['second_outcome'].upper()}")
    print(f"  stalls (unscored)   {s['stalls_reported_not_scored']}   "
          f"silent={s['silent_turns']}")
    print(f"  score               {s['score']}   wall={meta.get('wall_clock_s')}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
