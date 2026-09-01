#!/usr/bin/env python3
"""Does report form break with what is written, or with what is carried?

    python3 measure/form_grid/run.py --models local_qwen38flashnext grok_4p6
    python3 measure/form_grid/run.py --models local_qwen38flashnext --k 10 --n 0

THE OBSERVATION. Qwen3.8-Flash-Next wrote 24 findings on the dataroom fixture
and 11 of them were whole. It dropped `Gap` from findings 5 and 13, then from
13 onward never wrote a whole finding again — 14 to 24 are `Claim` alone. All
of it inside one 13,767-char generation. grok-4.6 wrote 33 of 33 whole on a
larger surface in a comparable reply. The collapse is positional and is not
about `[unverifiable]` — findings 14-19 are ordinary ones.

THE ANCHOR IS `collapse_at`, NOT THE FIRST GAP. Findings 5 and 13 lost one
field and the form came back; from 13 it did not. Those are different events
and one index cannot report both, so this measures the second directly: where
the terminal run of broken findings starts.

THE QUESTION, AND WHY A RUN CANNOT ANSWER IT. Two variables move together in an
audit: K, how much the model is asked to write in one generation, and N, how
much context it carries into that generation. A larger claim surface inflates
both, so varying the surface separates nothing. This varies them independently.

WHAT IT DECIDES. Grouping findings into batches puts each batch in its own leg,
and a leg boundary cuts BOTH variables. K falls because each batch writes fewer
findings. N falls because `react.py` resets `log_appendage_str` to empty at leg
start: the within-leg working log, which grows by literal append with every
action and observation and is never trimmed, does not survive into the next leg.
What crosses a leg boundary is only the store's turns — one pair per leg, 20
deep, each capped at 4,000 chars by `prompts._cap_turn`. Iterations are never
stored, so the 20-turn window counts legs.

So either answer favours grouping, and the grid decides the price rather than
the direction. Grouping's real cost is that the discarded working log takes the
evidence with it: a second batch must re-read the materials it no longer holds.
If K binds, that re-read is the whole bill. If N binds, the reset is doing the
work and the bill buys something. If neither cell breaks, the instrument does
not reproduce the field behaviour and nothing here is decided.

The surgery is not small. `delivered[]` is set on a block's opener, so the
moment a first batch emits `=== REPORT ===` the runner stops guarding the report
and may end the engagement with a half-written one; and a batch that closes its
own block is silently dropped by `blocks.content`, which takes
first-opener-to-first-closer while still reporting the block closed. Worth
predicting before paying for.

THE MODEL ADJUDICATES. Each cell gets the seller's claim and the whole data
room, line-numbered, and must find its own support, decide the verdict and cite
the line. An earlier version handed over the source run's `Evidence:` line and
left only the writing; Qwen then held form through every cell up to K=20 and
N=30,000, well past the 8,173 characters at which it broke in the field. That
arm measured formatting, not auditing, and it is kept as `--mode format`: a
control that says what the collapse is NOT.

REAL MATERIAL. The data room is `measure/fixtures/dataroom/corpus`, the same
nine documents the agent reads. The padding is `inspect_traces/*.txt` from an
actual fixture run — what that agent really read. Nothing here is synthetic
filler. It is still not a whole engagement: the claim surface is fixed instead
of extracted, and a cell that does not reproduce the field behaviour says so
rather than proving its absence.

NO MODEL SCORES ANOTHER. The measure is which §5 fields each finding carries,
which is a parse. The thing under test is form.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src"), str(REPO / "measure")):
    if p not in sys.path:
        sys.path.insert(0, p)

from chat.workflow import load_workflow                        # noqa: E402
from prescreen import probe                                    # noqa: E402

# The run whose material is replayed. Its report is fully formed 33 of 33, so
# the input carries no degradation of its own.
SOURCE_RUN = (REPO / "workflows" / "claims_audit" / "engagements"
              / "dataroom-fixture" / "runs"
              / "2026-08-31T21-37-17Z_fixture_grok_0831")

_HEADER = re.compile(r"(?m)^\*\*Finding \d+[^\n]*\*\*")
_CLAIM = re.compile(r"(?m)^Claim[^\n]*:(.+)$")
_EVIDENCE = re.compile(r"(?m)^Evidence:(.+)$")
# The three §5 fields. A finding that has dropped Evidence or Gap is the
# degradation this measures; Claim alone is what the field run produced from
# finding 14 onward.
_FIELD = re.compile(r"(?m)^\s*\**\s*(Claim|Evidence|Gap)\b")

SYSTEM = (
    "You are the auditor on a technical claims-audit engagement. The method you "
    "work to is given below in full.\n\n=== METHOD ===\n{method}\n=== END METHOD ===")


def material(k: int) -> list:
    """K claims, each with the evidence the source run found for it.

    The claim is the seller's assertion and carries no verdict. The evidence is
    grok's adjudicated `Evidence:` line, used only in `--mode format`.
    """
    text = (SOURCE_RUN / "report.md").read_text(errors="replace")
    heads = [(m.start(), m.group(0)) for m in _HEADER.finditer(text)]
    out = []
    for i, (pos, _hd) in enumerate(heads):
        blk = text[pos:heads[i + 1][0] if i + 1 < len(heads) else len(text)]
        c, e = _CLAIM.search(blk), _EVIDENCE.search(blk)
        if c and e:
            out.append((c.group(1).strip(), e.group(1).strip()))
    if len(out) < k:
        raise SystemExit(f"source run yields {len(out)} claims, need {k}")
    return out[:k]


def corpus() -> str:
    """The whole data room, line-numbered the way `inspect` returns it.

    Always present in `--mode adjudicate`: it is what the claims are checked
    against, so removing it would not shrink the task, it would make the task
    impossible and turn every finding `[unverifiable]`.
    """
    docs = sorted((REPO / "measure" / "fixtures" / "dataroom" / "corpus")
                  .glob("*.md"))
    if not docs:
        raise SystemExit("dataroom corpus not found")
    out = []
    for d in docs:
        body = d.read_text(errors="replace").splitlines()
        out.append(f"=== {d.name} ===")
        out += [f"{i}|{ln}" for i, ln in enumerate(body, 1)]
        out.append("")
    return "\n".join(out)


def padding(n: int) -> str:
    """N characters of what the agent actually read, or nothing."""
    if not n:
        return ""
    files = sorted((SOURCE_RUN / "working_record" / "inspect_traces").glob("*.txt"))
    blob = "\n\n".join(f.read_text(errors="replace") for f in files)
    if len(blob) < n:
        raise SystemExit(f"only {len(blob)} chars of trace, asked for {n}")
    return blob[:n]


def task(k: int, n: int, mode: str = "adjudicate") -> str:
    """The user message for one cell.

    `adjudicate` is the real task: the claim and the data room, nothing else.
    The model finds its own support, decides the verdict and cites the line.

    `format` hands over the source run's `Evidence:` line, leaving only the
    writing. It is the control, and it is not the audit — an auditor is never
    told what the materials show.
    """
    lines = []
    if mode == "adjudicate":
        lines += ["Below is the data room for this engagement, every document "
                  "in full with line numbers.", "",
                  "=== DATA ROOM ===", corpus(), "=== END DATA ROOM ===", ""]
    if n:
        lines += ["Below is the working record of what you have already done "
                  "on this engagement.", "", "=== WORKING RECORD ===",
                  padding(n), "=== END WORKING RECORD ===", ""]
    if mode == "adjudicate":
        lines += [f"Adjudicate each of the following {k} claims against the "
                  "data room and write one finding for each, in the method's "
                  "form, one after another. Decide each verdict yourself and "
                  "cite doc:line for every statement of fact. Output the "
                  "findings and nothing else — no conclusion, no coverage, "
                  "no block markers.", ""]
        for i, (claim, _evid) in enumerate(material(k), 1):
            lines += [f"Claim {i}: {claim}", ""]
    else:
        lines += [f"Write the findings for the following {k} claims, in the "
                  "method's form, one after another. Output the findings and "
                  "nothing else — no conclusion, no coverage, no block "
                  "markers.", ""]
        for i, (claim, evid) in enumerate(material(k), 1):
            lines += [f"Claim {i}: {claim}",
                      f"  What the materials show: {evid}", ""]
    return "\n".join(lines)


def form(reply: str, k: int) -> dict:
    """Which §5 fields each emitted finding carries, and where they stop."""
    heads = [m.start() for m in _HEADER.finditer(reply or "")]
    per = []
    for i, pos in enumerate(heads):
        blk = reply[pos:heads[i + 1] if i + 1 < len(heads) else len(reply)]
        per.append(sorted(set(_FIELD.findall(blk))))
    want = {"Claim", "Evidence", "Gap"}
    ok = [want <= set(f) for f in per]

    # TWO PHENOMENA, NOT ONE. On the field run Qwen dropped `Gap` from findings
    # 5 and 13 — both `[real]`, where the method asks for `Gap: None` — and then
    # collapsed to `Claim` alone from 14 to the end. A single "first incomplete"
    # index reports 5 and hides the collapse, which is the thing under test.
    first_incomplete = next((i + 1 for i, good in enumerate(ok) if not good), None)
    # Where the terminal run of incomplete findings starts: the form stopped
    # here and never came back. None when the last finding is whole.
    collapse = None
    if per and not ok[-1]:
        collapse = len(ok)
        while collapse > 1 and not ok[collapse - 2]:
            collapse -= 1
    # HOW FAR IT GOT, IN CHARACTERS, not just in findings. Qwen spends ~930
    # chars on a finding and grok ~350, so an equal K is not equal output. If
    # the collapse tracks a length rather than a count, K measured in findings
    # is the wrong unit and this is the column that shows it.
    chars_before = heads[collapse - 1] if collapse else len(reply or "")
    return {"findings_emitted": len(per), "asked": k,
            "fully_formed": sum(ok), "first_incomplete": first_incomplete,
            "collapse_at": collapse, "reply_chars": len(reply or ""),
            "chars_before_collapse": chars_before, "fields_by_finding": per}


def model_route(name: str) -> dict:
    """A measure/models/*.yaml mapped onto `prescreen.probe` arguments.

    Same shape as measure/method_probe/run.py: the reader probed is the reader
    deployed. The `/v1` handling is `_ChatBackend.__init__`'s — strip a trailing
    `/v1`, then append `/v1/chat/completions` — because appending blind sends a
    local route to a 404 that reads as a model failure.
    """
    doc = yaml.safe_load((REPO / "measure" / "models" / f"{name}.yaml")
                         .read_text(encoding="utf-8")) or {}
    llm = doc.get("llm_config") or {}
    base = (llm.get("vllm_url") or "http://127.0.0.1:5000").rstrip("/")
    if base.endswith("/v1"):
        base = base[:-3]
    order = ((llm.get("extra_body") or {}).get("provider") or {}).get("order") or []
    return {"config": name, "model": llm.get("model"),
            "url": f"{base}/v1/chat/completions",
            "key_env": llm.get("api_key"),
            "effort": llm.get("reasoning_effort"),
            "tag": order[0] if order else name}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models", nargs="+", required=True)
    ap.add_argument("--k", nargs="*", type=int, default=[5, 10, 20, 30],
                    help="findings requested per call")
    ap.add_argument("--n", nargs="*", type=int, default=[0, 10000, 30000],
                    help="characters of carried context")
    # THE FIELD RUN'S CAP, not a smaller one. The Qwen run being replicated
    # ran at `react_max_tokens: 32768` and spent ~3,800 tokens, so its collapse
    # was not truncation. A tighter cap here would truncate the K=30 cells and
    # the cut-off tail would parse as exactly the collapse under test.
    ap.add_argument("--max-tokens", type=int, default=32768)
    ap.add_argument("--timeout", type=int, default=600)
    ap.add_argument("--mode", choices=("adjudicate", "format"),
                    default="adjudicate",
                    help="adjudicate: claim + data room, model decides the "
                         "verdict. format: the verdict is handed over "
                         "(control)")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    out = args.out or HERE / (
        f"grid_{args.mode}_"
        + time.strftime("%Y-%m-%dT%H-%M-%SZ", time.gmtime()) + ".jsonl")
    method = load_workflow(REPO / "workflows" / "claims_audit" / "method" / "METHOD.md")
    routes = [model_route(m) for m in args.models]

    print(f"K={args.k} x N={args.n} x {len(routes)} models "
          f"= {len(args.k) * len(args.n) * len(routes)} calls")
    print(f"  method as delivered {len(method):,} chars; material from "
          f"{SOURCE_RUN.name}")
    for r in routes:
        print(f"  {r['config']:24} {r['model']} @ {r['effort']}")

    with out.open("w", encoding="utf-8") as fh:
        for r in routes:
            for k in args.k:
                for n in args.n:
                    row = probe(r["model"], r["tag"], effort=r["effort"],
                                url=r["url"], key_env=r["key_env"],
                                max_tokens=args.max_tokens, timeout=args.timeout,
                                system=SYSTEM.format(method=method),
                                user=task(k, n, args.mode), schema=None)
                    rec = {"config": r["config"], "effort": r["effort"],
                           "mode": args.mode, "k": k, "n": n, **row}
                    if row.get("status") == "ok":
                        rec.update(form(row.get("content") or "", k))
                    fh.write(json.dumps(rec) + "\n")
                    fh.flush()
                    print(f"  {r['config'][:20]:22} K={k:<3} N={n:<6} "
                          f"{rec.get('status')}/{rec.get('finish')} "
                          f"emitted={rec.get('findings_emitted')} "
                          f"full={rec.get('fully_formed')} "
                          f"collapse@{rec.get('collapse_at')}")
    print(f"\n{out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
