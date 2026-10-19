#!/usr/bin/env python3
"""Score a temperature-sweep results file and print the summary.

    python3 measure/temp_sweep/score.py measure/temp_sweep/results/direct_<t>.jsonl

Audit-arm scoring is deterministic, on the answer's structured fields:

  audit   `verdict` is one of the gold verdicts (known_answers/chhoto.yaml);
          `reason_right` when the gold has `cites_any`: some evidence cites
          that document at overlapping lines; each quote is placed with
          claims_audit.schemas.quote_at, the audit's own citation rule.
  review  the outcome is derived from the four observations exactly as
          audit_review.schemas.CLEAN does, and compared with the gold.

An answer that is empty or does not parse is `valid: false` and scores as
wrong; it is counted, not dropped, because how often it happens at each
temperature is one of the things measured.

Writes <results>.scores.jsonl next to the input and prints, for each prompt
and temperature, the spread of reasoning and answer tokens over the repeats
and the quality, then the same pooled over prompts.
"""
from __future__ import annotations

import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "workflowsv2"))
from claims_audit.schemas import quote_at                        # noqa: E402
from audit_review.schemas import CLEAN                           # noqa: E402

HERE = Path(__file__).resolve().parent
TARGET = REPO / "workflowsv2/claims_audit/engagements/chhoto-full/target"


def _overlaps(a, b) -> bool:
    return a[0] <= b[1] and b[0] <= a[1]


def score_audit(ans: dict, gold: dict, shown: list) -> dict:
    out = {"verdict": ans.get("verdict"),
           "right": ans.get("verdict") in gold["verdict"]}
    ev = [e for e in ans.get("evidence") or [] if isinstance(e, dict)]
    if gold.get("cites_any"):
        out["reason_right"] = any(
            e.get("document") == c["document"] and isinstance(e.get("lines"), list)
            and len(e["lines"]) == 2 and _overlaps(e["lines"], c["lines"])
            for e in ev for c in gold["cites_any"])
    statuses = []
    for e in ev:
        doc, lines, quote = e.get("document"), e.get("lines"), e.get("quote") or ""
        if doc not in shown:
            statuses.append("unknown_document")
            continue
        body = (TARGET / doc).read_text(encoding="utf-8").splitlines()
        if not (isinstance(lines, list) and len(lines) == 2
                and 1 <= lines[0] <= lines[1] <= len(body)):
            statuses.append("out_of_range")
            continue
        statuses.append(quote_at(body, lines[0], lines[1], quote)[0])
    out["quotes"] = statuses
    out["quotes_ok"] = sum(s in ("exact", "joined", "prefixed") for s in statuses)
    return out


def score_review(ans: dict, gold: dict) -> dict:
    holds = all(ans.get(k) in v for k, v in CLEAN.items())
    return {"holds": holds, "right": holds == gold["holds"],
            "observations": {k: ans.get(k) for k in CLEAN}}


def score_row(row: dict, prompt: dict) -> dict:
    s = {"prompt": row["prompt"], "temperature": row["sent_temperature"],
         "repeat": row["repeat"], "finish": row.get("finish"),
         "reasoning_tok": row.get("reasoning_tok"),
         "answer_tok": ((row.get("completion_tok") or 0) - (row.get("reasoning_tok") or 0)
                        if row.get("completion_tok") is not None else None),
         "secs": row.get("secs"), "status": row.get("status")}
    try:
        ans = json.loads(row.get("content") or "")
        assert isinstance(ans, dict)
    except Exception:                                          # noqa: BLE001
        s.update(valid=False, right=False)
        return s
    s["valid"] = True
    if prompt["kind"] == "audit":
        s.update(score_audit(ans, prompt["gold"], prompt["files"]))
    else:
        s.update(score_review(ans, prompt["gold"]))
    return s


def _spread(xs) -> str:
    xs = [x for x in xs if x is not None]
    if not xs:
        return "-"
    sd = statistics.stdev(xs) if len(xs) > 1 else 0.0
    return f"{statistics.mean(xs):7.0f} ±{sd:6.0f} [{min(xs)}-{max(xs)}]"


def summarise(scores: list) -> None:
    cells = defaultdict(list)
    for s in scores:
        cells[(s["prompt"], s["temperature"])].append(s)
    print(f"{'prompt':28s} {'temp':>4s}  n  valid right  {'reasoning tokens':26s}  {'answer tokens':26s}  extra")
    for (pid, t), ss in sorted(cells.items()):
        extra = []
        if any("reason_right" in s for s in ss):
            extra.append(f"reason {sum(bool(s.get('reason_right')) for s in ss)}")
        if any("quotes" in s for s in ss):
            q = sum(len(s.get("quotes", [])) for s in ss)
            extra.append(f"quotes ok {sum(s.get('quotes_ok', 0) for s in ss)}/{q}")
        verdicts = [s.get("verdict") or ("holds" if s.get("holds") else
                                        "does_not_hold" if "holds" in s else None)
                    for s in ss]
        extra.append("answers " + ",".join(str(v) for v in verdicts))
        runaway = sum(s.get("finish") == "length" for s in ss)
        if runaway:
            extra.append(f"RUNAWAY {runaway}")
        print(f"{pid:28s} {t:4.1f} {len(ss):2d} {sum(s['valid'] for s in ss):5d} "
              f"{sum(s['right'] for s in ss):5d}  {_spread([s['reasoning_tok'] for s in ss]):26s}  "
              f"{_spread([s['answer_tok'] for s in ss]):26s}  {'; '.join(extra)}")
    print("\npooled over prompts")
    by_t = defaultdict(list)
    for s in scores:
        by_t[s["temperature"]].append(s)
    for t, ss in sorted(by_t.items()):
        print(f"  temp {t:3.1f}: n {len(ss)}, valid {sum(s['valid'] for s in ss)}, "
              f"right {sum(s['right'] for s in ss)}, "
              f"runaway {sum(s.get('finish') == 'length' for s in ss)}, "
              f"reasoning {_spread([s['reasoning_tok'] for s in ss])}, "
              f"answer {_spread([s['answer_tok'] for s in ss])}")


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    path = Path(sys.argv[1])
    prompts = {p["id"]: p for p in map(json.loads, (HERE / "prompts.jsonl").read_text().splitlines())}
    rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    scores = [score_row(r, prompts[r["prompt"]]) for r in rows]
    out = path.with_suffix(".scores.jsonl")
    out.write_text("".join(json.dumps(s) + "\n" for s in scores), encoding="utf-8")
    summarise(scores)
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
