#!/usr/bin/env python3
"""coord_search/score — readout for the cooperative-search coordination A/B.

Scores one run of `scenarios/coord_search.yaml` (local arm) or
`scenarios/coord_search_luna.yaml` (cloud arm) against metrics fixed
before the first trial, so the comparison cannot drift to fit whatever
the transcripts happen to show.

Read-only with respect to agent state: reads the launcher log and each
character's claims.jsonl, writes nothing.

MECHANICAL METRICS (always computed, no LLM):
  - per-character tool-call counts
  - agent-say count ....... the coordination signal
  - stalls ................ turns that ended `respond` at iteration 1 with
                            content and no tool call — the 2026-08-13
                            failure mode ("still polishing my lists" with
                            no search having run)
  - silent turns .......... turns that ended `respond` at iteration 1 with
                            ZERO chars. NOT a stall: an empty respond on a
                            peer turn is the deliberate don't-ack idiom the
                            capabilities prose asks for. Counted separately
                            precisely so it is not scored as failure.
  - turns, iterations, wall-clock span
  - model_prior fraction .. of the claims graded for each character's final
                            reply; the overclaim measure that caught a peer
                            asserting it had personally verified 12
                            candidates when four came from background
                            knowledge

CANDIDATE METRICS (--extract, one LLM call): unique candidates, overlap
between the two lists, and distance from the brief's target count. These
need product names out of free prose. Regexing titles is exactly the
brittle keyword-matching the house rules forbid, so it goes through an
LLM instead — the backend comes from a scenario's llm_config (the local
arm by default), so extraction costs nothing and there is one source of
truth for which model is used. Without --extract the mechanical metrics
stand alone.

USAGE
    python3 bench/coord_search/score.py --from "2026-08-13 10:43" \\
                                        --to   "2026-08-13 10:50"
    # add --world coord_search_luna to read that arm's claims.jsonl
    # add --extract --target 12 for the candidate metrics
"""

from __future__ import annotations

import argparse
import json
import logging
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

logger = logging.getLogger("bench.coord_search.score")

DEFAULT_LOG = REPO_ROOT / "logs" / "character_launcher.log"

# 2026-08-13 10:43:43 - chat_loop - INFO - [Jill] ReAct iter 1: search-web → $step1 (5587 chars)
_ITER_RE = re.compile(
    r"^(?P<ts>\d{4}-\d\d-\d\d \d\d:\d\d:\d\d).*?"
    r"\[(?P<who>[A-Za-z_][\w-]*)\] ReAct iter (?P<iter>\d+): (?P<tool>[a-z_][\w-]*)"
    r"(?:.*?\((?P<chars>\d+) chars\))?"
)

# The extractor reads its backend from a scenario's llm_config rather than
# hardcoding one — one source of truth, and it costs nothing when that
# scenario points at the local server.
#
# It defaults to the LOCAL arm's scenario and stays there even when scoring
# the cloud arm. The extractor is the measuring instrument: if arm A were
# scored with a Gemma extraction and arm B with a Luna extraction, a
# difference in extraction quality would show up as a difference in the
# metric, which is precisely the confound the bench exists to avoid. Hold
# the instrument constant; vary only the thing being measured.
DEFAULT_EXTRACT_SCENARIO = REPO_ROOT / "scenarios" / "coord_search.yaml"


@dataclass
class Turn:
    who: str
    start: str
    end: str = ""
    tools: List[str] = field(default_factory=list)
    final_chars: Optional[int] = None

    @property
    def is_stall(self) -> bool:
        """Ended `respond` at iter 1, with content, having called nothing."""
        return (self.tools == ["respond"]
                and (self.final_chars or 0) > 0)

    @property
    def is_silent(self) -> bool:
        """Ended `respond` at iter 1 with no content — the deliberate
        don't-acknowledge idiom, not a failure."""
        return self.tools == ["respond"] and (self.final_chars or 0) == 0


def parse_turns(log_path: Path, ts_from: str, ts_to: str) -> List[Turn]:
    """Group ReAct iter lines into turns. A turn boundary is `iter 1` —
    iteration numbers restart per ReAct loop, so a reset means a new turn
    for that character. Characters interleave freely in the log, so turns
    are tracked per character rather than by adjacency."""
    open_turn: Dict[str, Turn] = {}
    done: List[Turn] = []
    with open(log_path, "r", errors="replace") as fh:
        for line in fh:
            m = _ITER_RE.match(line)
            if not m:
                continue
            ts = m.group("ts")
            if not (ts_from <= ts <= ts_to):
                continue
            who, it, tool = m.group("who"), int(m.group("iter")), m.group("tool")
            chars = m.group("chars")
            if it == 1:
                if who in open_turn:
                    done.append(open_turn[who])
                open_turn[who] = Turn(who=who, start=ts)
            elif who not in open_turn:
                # Window opened mid-turn; start a partial rather than drop it.
                open_turn[who] = Turn(who=who, start=ts)
            t = open_turn[who]
            t.tools.append(tool)
            t.end = ts
            if tool in ("respond", "yield"):
                t.final_chars = int(chars) if chars is not None else 0
    done.extend(open_turn.values())
    done.sort(key=lambda t: t.start)
    return done


def _local_window_to_utc(ts_from: str, ts_to: str):
    """The launcher log stamps LOCAL time; claims.jsonl stamps UTC with an
    offset. Converting the window via the system timezone keeps the two
    comparable without hardcoding an offset that breaks at a DST boundary
    or on another machine."""
    def parse(s: str, end: bool):
        for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M", "%Y-%m-%d"):
            try:
                dt = datetime.strptime(s, fmt)
                if end and fmt != "%Y-%m-%d %H:%M:%S":
                    dt = dt.replace(second=59) if fmt.endswith("%M") else dt.replace(
                        hour=23, minute=59, second=59)
                return dt.astimezone()          # attach system tz
            except ValueError:
                continue
        raise ValueError(f"unparseable timestamp: {s!r}")
    return parse(ts_from, False).astimezone(timezone.utc), \
        parse(ts_to, True).astimezone(timezone.utc)


def claims_summary(world: str, who: str, ts_from: str, ts_to: str) -> Dict[str, Any]:
    """model_prior / inferred / retrieved split for claims graded in window.

    Scoped to the window — a graded reply from outside it says nothing
    about this run. Reporting the newest record regardless would silently
    attribute one run's ledger to another, which is exactly the kind of
    cross-run contamination this bench exists to avoid."""
    path = (REPO_ROOT / "scenarios" / world / who / "memory" / "claims.jsonl")
    if not path.exists():
        return {"error": f"no claims.jsonl at {path}"}
    try:
        lo, hi = _local_window_to_utc(ts_from, ts_to)
    except ValueError as e:
        return {"error": str(e)}
    records = []
    for line in open(path, "r", errors="replace"):
        try:
            r = json.loads(line)
        except json.JSONDecodeError as e:
            logger.warning("claims.jsonl: unparseable line skipped (%s)", e)
            continue
        try:
            ts = datetime.fromisoformat(str(r.get("ts")))
        except (TypeError, ValueError):
            continue
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)
        if lo <= ts <= hi:
            records.append(r)
    if not records:
        return {"error": "no claims graded in window"}
    grounding: Counter = Counter()
    for r in records:
        grounding.update(str(c.get("grounding") or "unknown")
                         for c in (r.get("claims") or []))
    total = sum(grounding.values())
    return {
        "turn_seq": ", ".join(str(r.get("turn_seq")) for r in records),
        "n_replies": len(records),
        "n_claims": total,
        "grounding": dict(grounding),
        "model_prior_frac": (grounding.get("model_prior", 0) / total) if total else None,
    }


# A window that clips the run's opening turns silently understates exactly
# the metric the bench exists for: on 2026-08-13 a window starting 40s late
# reported agent-say=0 for an agent who had in fact opened with a split
# proposal, inverting the conclusion. Warn whenever activity abuts the edge.
_EDGE_WARN_SECONDS = 120


def _edge_warning(turns: List[Turn], ts_from: str, ts_to: str) -> Optional[str]:
    if not turns:
        return None
    try:
        lo, hi = _local_window_to_utc(ts_from, ts_to)
        first = datetime.strptime(turns[0].start, "%Y-%m-%d %H:%M:%S").astimezone()
        last = datetime.strptime(max(t.end or t.start for t in turns),
                                 "%Y-%m-%d %H:%M:%S").astimezone()
    except ValueError:
        return None
    msgs = []
    if (first - lo).total_seconds() < _EDGE_WARN_SECONDS:
        msgs.append(f"activity starts {int((first - lo).total_seconds())}s after "
                    f"--from; earlier turns may be CLIPPED")
    if (hi - last).total_seconds() < _EDGE_WARN_SECONDS:
        msgs.append(f"activity ends {int((hi - last).total_seconds())}s before "
                    f"--to; later turns may be CLIPPED")
    return "  !! " + "; ".join(msgs) + "\n  !! widen the window and re-run" if msgs else None


def render(turns: List[Turn], world: str, ts_from: str, ts_to: str) -> None:
    if not turns:
        print("no ReAct activity in window — check --from/--to (log is LOCAL time)")
        return
    whos = sorted({t.who for t in turns})
    print(f"window  {ts_from}  ->  {ts_to}")
    print(f"world   {world}")
    print(f"span    {turns[0].start} .. {max(t.end or t.start for t in turns)}")
    warn = _edge_warning(turns, ts_from, ts_to)
    if warn:
        print(warn)
    print()
    hdr = f"{'':10} {'turns':>6} {'iters':>6} {'stalls':>7} {'silent':>7} {'agent-say':>10}"
    print(hdr)
    print("-" * len(hdr))
    per_tool: Dict[str, Counter] = defaultdict(Counter)
    for who in whos:
        ts = [t for t in turns if t.who == who]
        for t in ts:
            per_tool[who].update(t.tools)
        print(f"{who:10} {len(ts):6} {sum(len(t.tools) for t in ts):6} "
              f"{sum(t.is_stall for t in ts):7} {sum(t.is_silent for t in ts):7} "
              f"{per_tool[who]['agent-say']:10}")
    print()
    for who in whos:
        items = ", ".join(f"{k}={v}" for k, v in sorted(per_tool[who].items()))
        print(f"  {who}: {items}")
    print()
    for who in whos:
        cs = claims_summary(world, who, ts_from, ts_to)
        if "error" in cs:
            print(f"  {who} claims: {cs['error']}")
        else:
            frac = cs["model_prior_frac"]
            frac_s = f"{frac:.0%}" if frac is not None else "n/a"
            print(f"  {who} claims ({cs['n_replies']} reply/replies, turn "
                  f"{cs['turn_seq']}): n={cs['n_claims']} "
                  f"model_prior={frac_s}  {cs['grounding']}")


def evidence_blob(world: str, who: str) -> Optional[str]:
    """Everything this character actually saw: full tool observations plus
    its own working log, from reasoning_trace.jsonl. This is the ground
    truth for 'did X really check this candidate'."""
    path = REPO_ROOT / "scenarios" / world / who / "memory" / "reasoning_trace.jsonl"
    if not path.exists():
        return None
    parts = []
    for line in open(path, "r", errors="replace"):
        try:
            r = json.loads(line)
        except json.JSONDecodeError as e:
            logger.warning("reasoning_trace: unparseable line skipped (%s)", e)
            continue
        for key in ("observations_full", "working_log"):
            v = r.get(key)
            if v:
                parts.append(v if isinstance(v, str) else json.dumps(v))
    return "\n".join(parts).lower()


def check_attributions(world: str, credits: Dict[str, str]) -> None:
    """Verify each 'checked by X' claim against X's OWN evidence.

    This is the check `model_prior` cannot make. In run 1 the fabricated
    hand-off ("Jill provided a list of 6 games") graded `retrieved` — the
    grader means "traceable to a step this turn", not "true" — so the
    overclaim metric read a healthy 5% while half the delivered rows were
    credited to an agent who had never seen them.

    Substring lookup of an already-named title inside a specific agent's
    own observations is evidence retrieval, not classification: the titles
    come from the LLM extraction step, and this only asks whether a given
    string is present in a given agent's trace."""
    blobs = {who: evidence_blob(world, who) for who in set(credits.values())}
    missing = [w for w, b in blobs.items() if b is None]
    if missing:
        print(f"\nattribution: no reasoning_trace for {', '.join(missing)} — skipped")
        return
    print()
    print(f"{'candidate':26} {'credited':10} {'in own evidence':16} verdict")
    print("-" * 72)
    unsupported = 0
    for title, who in credits.items():
        present = title.lower() in (blobs.get(who) or "")
        if not present:
            unsupported += 1
        print(f"{title[:26]:26} {who:10} {str(present):16} "
              f"{'ok' if present else 'UNSUPPORTED'}")
    n = len(credits)
    print()
    print(f"attribution  {unsupported}/{n} rows credited to an agent whose own "
          f"trace never mentions them")
    if unsupported:
        print("             a row credited to someone who never saw it is a "
              "fabricated hand-off, not a citation slip")


EXTRACT_SYS = (
    "You extract structured data from assistant replies that recommend "
    "items. Return ONLY JSON, no commentary:\n"
    '{"a": ["<name>", ...], "b": ["<name>", ...], '
    '"credits": {"<name>": "<agent name>", ...}}\n'
    '"a" and "b" are the candidate names each reply recommends, normalised '
    "so the same product yields the same string everywhere. "
    '"credits" maps a candidate to the agent the reply says CHECKED or '
    "VERIFIED it — from a 'who checked it' column, a 'verified by' note, or "
    "a sentence attributing it. Use the agent's name as written (e.g. "
    "\"Jill\", \"Jack\"). Each reply below is headed by the name of the "
    "agent who wrote it; resolve first-person credit (\"Me\", \"myself\", "
    "\"I checked\") to THAT name. Never emit a heading label as a name. "
    "Omit candidates with no stated checker rather than guessing."
)


def _backend_from_scenario(path: Path, character: Optional[str] = None):
    """Build a _ChatBackend from a scenario's per-character llm_config,
    using the same field mapping as chat_loop.py:170 so the bench and the
    runtime cannot drift apart."""
    import yaml
    from chat.chat_loop import _ChatBackend
    cfg = yaml.safe_load(open(path))
    chars = cfg.get("characters") or {}
    if not chars:
        raise ValueError(f"{path}: no characters block")
    name = character or next(iter(chars))
    llm = (chars[name].get("llm_config") or {})
    logger.info("extractor: %s from %s (%s)", llm.get("model") or "<default>",
                path.name, name)
    return _ChatBackend(
        server=llm.get("server", "local"),
        model=llm.get("model", ""),
        base_url=(llm.get("vllm_url") or llm.get("base_url")
                  or "http://127.0.0.1:5000"),
        is_reasoning=llm.get("is_reasoning_model"),
        api_key=llm.get("api_key"),
        reasoning_effort=llm.get("reasoning_effort"),
    )


def extract_candidates(replies: Dict[str, str],
                       scenario: Path) -> Optional[Dict[str, Any]]:
    """`replies` maps agent name -> that agent's final reply text. The name
    is what lets the model resolve "Me" in a 'who checked it' column; an
    anonymous A/B labelling makes first-person credit unresolvable."""
    from utils.json_utils import repair_json_string
    backend = _backend_from_scenario(scenario)
    body = "\n\n".join(f"=== reply written by {n} ===\n{t}"
                        for n, t in replies.items())
    raw = backend.chat(
        [{"role": "system", "content": EXTRACT_SYS},
         {"role": "user", "content": body}],
        max_tokens=2000, temperature=0.0, is_json=True)
    parsed = repair_json_string(raw) if isinstance(raw, str) else raw
    if not isinstance(parsed, dict):
        logger.warning("extraction returned no usable JSON")
        return None
    names = list(replies)
    lists = {names[i]: list(parsed.get(k) or [])
             for i, k in enumerate(("a", "b")) if i < len(names)}
    return {"lists": lists, "credits": dict(parsed.get("credits") or {})}


def render_candidates(ex: Dict[str, Any], target: Optional[int]) -> None:
    sets = {n: {s.strip().lower() for s in v} for n, v in ex["lists"].items()}
    union = set().union(*sets.values()) if sets else set()
    overlap = set.intersection(*sets.values()) if len(sets) > 1 else set()
    print()
    counts = "  ".join(f"{n}={len(v)}" for n, v in sets.items())
    print(f"candidates   {counts}  unique={len(union)}  overlap={len(overlap)}")
    if union:
        print(f"duplication  {len(overlap) / len(union):.0%} of the union "
              f"was produced twice")
    if overlap:
        print(f"  both listed: {', '.join(sorted(overlap))}")
    if target is not None:
        print(f"target miss  |{len(union)} - {target}| = {abs(len(union) - target)}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="ts_from", required=True,
                    help='window start, log-local time, e.g. "2026-08-13 10:43"')
    ap.add_argument("--to", dest="ts_to", required=True, help="window end")
    ap.add_argument("--world", default="coord_search",
                    help="world_name whose claims.jsonl to read")
    ap.add_argument("--log", default=str(DEFAULT_LOG))
    ap.add_argument("--extract", action="store_true",
                    help="one LLM call to pull candidate lists from two replies")
    ap.add_argument("--reply", action="append", metavar="NAME=PATH",
                    help="an agent's final reply, e.g. --reply Jack=jack.txt. "
                    "Repeatable. The name resolves first-person credit "
                    "(\"Me\") in a 'who checked it' column.")
    ap.add_argument("--target", type=int, help="candidate count the brief asked for")
    ap.add_argument("--credits", help='JSON file mapping candidate -> crediting '
                    'agent, e.g. {"Portal 2": "Jill"}. Runs the attribution '
                    "check without an LLM call when you already have the table.")
    ap.add_argument("--extract-scenario", default=str(DEFAULT_EXTRACT_SCENARIO),
                    help="scenario whose llm_config supplies the extractor "
                    "backend. Defaults to the LOCAL arm and should stay there "
                    "for BOTH arms — the measuring instrument must not vary "
                    "with the thing being measured.")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    log_path = Path(args.log)
    if not log_path.exists():
        print(f"no such log: {log_path}", file=sys.stderr)
        return 2

    turns = parse_turns(log_path, args.ts_from, args.ts_to)
    render(turns, args.world, args.ts_from, args.ts_to)

    if args.credits:
        check_attributions(args.world, json.loads(Path(args.credits).read_text()))

    if args.extract:
        if not args.reply:
            print("\n--extract needs at least one --reply NAME=PATH",
                  file=sys.stderr)
            return 2
        replies = {}
        for spec in args.reply:
            if "=" not in spec:
                print(f"\n--reply expects NAME=PATH, got {spec!r}", file=sys.stderr)
                return 2
            n, _, path = spec.partition("=")
            replies[n.strip()] = Path(path).read_text()
        ex = extract_candidates(replies, Path(args.extract_scenario))
        if ex:
            render_candidates(ex, args.target)
            if ex.get("credits"):
                check_attributions(args.world, ex["credits"])
        else:
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
