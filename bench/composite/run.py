#!/usr/bin/env python3
"""bench/composite — M0 frozen composite regression suite.

One command runs the four pinned suites (memory_recall, hle,
discourse_reflect, introspective_fidelity) end-to-end against the pinned
scenarios and appends ONE line to bench/composite/ledger.jsonl. The ledger
is the point: every harness change gets a row, and a change ships only if
its composite is not below baseline by more than the variance band
(docs/harness-roadmap.md, M0).

Usage:
    python bench/composite/run.py                       # full run → ledger
    python bench/composite/run.py --smoke               # tiny plumbing test, no ledger
    python bench/composite/run.py --suites hle          # partial, no ledger
    python bench/composite/run.py --note "after react parse fix"
    python bench/composite/run.py --chat-scenario scenarios/jill-benchmark-chat-opus.yaml \
        --hle-scenario ... --note "frontier comparison"   # different backend, own baseline

Pinning:
    - scenario-chat.yaml / scenario-hle.yaml — frozen copies; an edit is a
      new baseline.
    - hle_pinned_questions.json — written on the first full run, verified
      on every later one (guards against HF dataset-order drift; there is
      no upstream frozen HLE manifest).
    - judge model pinned (JUDGE_MODEL), judge code hashed into each row.
    - refuses to run with a dirty tracked tree unless --allow-dirty.

Requires: live backend at the scenario's vllm_url; CLAUDE_API_KEY for the
judges.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import statistics
import subprocess
import sys
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
BENCH = REPO / "bench"

SUITES = ("memory_recall", "hle", "discourse_reflect", "introspective_fidelity")
JUDGE_MODEL = "claude-sonnet-4-6"
JUDGE_FILES = (
    BENCH / "memory_recall" / "judge.py",
    BENCH / "hle" / "judge.py",
    BENCH / "discourse_reflect" / "judge.py",
    BENCH / "introspective_fidelity" / "judge.py",
)

HLE_START = 0
HLE_MAX_QUESTIONS = 12
HLE_SMOKE_QUESTIONS = 2
DISCOURSE_OUTPUT_FIELD = "production_new_discourse_state__current"
SMOKE_MEMORY_SUBSETS = ("01-direct-recall", "03-negative-fact")

DEFAULT_CHAT_SCENARIO = HERE / "scenario-chat.yaml"
DEFAULT_HLE_SCENARIO = HERE / "scenario-hle.yaml"
PIN_FILE = HERE / "hle_pinned_questions.json"
LEDGER = HERE / "ledger.jsonl"
RESULTS = HERE / "results"


class SuiteError(RuntimeError):
    pass


def _sh(cmd: list, log_path: Path, cwd: Path) -> float:
    """Run one suite phase, teeing output to a log file. Returns duration
    in seconds; raises SuiteError with the log tail on nonzero exit."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = datetime.now(timezone.utc)
    print(f"    $ {' '.join(str(c) for c in cmd)}")
    with open(log_path, "a", encoding="utf-8") as log:
        log.write(f"\n$ {' '.join(str(c) for c in cmd)}\n")
        log.flush()
        proc = subprocess.run(
            [str(c) for c in cmd], cwd=str(cwd),
            stdout=log, stderr=subprocess.STDOUT)
    dur = (datetime.now(timezone.utc) - started).total_seconds()
    if proc.returncode != 0:
        tail = "\n".join(
            log_path.read_text(encoding="utf-8").splitlines()[-25:])
        raise SuiteError(
            f"{cmd[1]} exited {proc.returncode} after {dur:.0f}s; "
            f"log tail ({log_path}):\n{tail}")
    return dur


def _read_json(path: Path) -> dict:
    if not path.is_file():
        raise SuiteError(f"expected output missing: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _read_jsonl(path: Path) -> list:
    if not path.is_file():
        raise SuiteError(f"expected output missing: {path}")
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            out.append(json.loads(line))
    return out


# ── preflight ────────────────────────────────────────────────────────────

def _git_dirty() -> bool:
    out = subprocess.run(
        ["git", "status", "--porcelain", "-uno"], cwd=str(REPO),
        capture_output=True, text=True, check=True).stdout.strip()
    return bool(out)


def _git_commit() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=str(REPO),
        capture_output=True, text=True, check=True).stdout.strip()


def _backend_from_scenario(scenario: Path) -> dict:
    """Resolve backend host/model from the scenario and confirm it is
    live. Mirrors _ChatBackend's URL fallback chain."""
    cfg = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    chars = cfg.get("characters") or {}
    llm = next(iter(chars.values()), {}).get("llm_config") or {}
    url = llm.get("vllm_url") or llm.get("base_url") or "http://127.0.0.1:5000"
    try:
        with urllib.request.urlopen(f"{url}/v1/models", timeout=10) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        model = (data.get("data") or [{}])[0].get("id", "")
    except Exception as e:
        if llm.get("server") == "local":
            raise SystemExit(f"backend at {url} not reachable: {e}")
        # cloud backend: no /v1/models endpoint to probe — say so rather
        # than silently skipping the liveness check.
        print(f"  (backend probe skipped for server="
              f"{llm.get('server')!r}: {e})")
        model = llm.get("model", "")
    return {"server": llm.get("server", "local"), "model": model, "host": url}


def _judge_code_sha() -> str:
    h = hashlib.sha256()
    for f in JUDGE_FILES:
        h.update(f.read_bytes())
    return h.hexdigest()[:12]


# ── suites ───────────────────────────────────────────────────────────────
# Each returns (score_0_to_1, detail_dict). Runners launch with cwd=src/
# (they expect launcher/chat imports resolvable); judges from repo root.

def run_memory_recall(run_dir: Path, chat_scenario: Path, smoke: bool):
    out = run_dir / "memory_recall"
    log = run_dir / "logs" / "memory_recall.log"
    cmd = [sys.executable, BENCH / "memory_recall" / "runner.py",
           "--scenario", chat_scenario, "--output-dir", out]
    if smoke:
        cmd += ["--subsets", *SMOKE_MEMORY_SUBSETS]
    d1 = _sh(cmd, log, cwd=REPO / "src")
    d2 = _sh([sys.executable, BENCH / "memory_recall" / "judge.py",
              "--run-dir", out, "--judge-model", JUDGE_MODEL], log, cwd=REPO)
    # The memory_recall judge scores a failed/unparseable judge call as
    # 0.0 on every axis — indistinguishable from a real regression in
    # summary.json. Count judge_error markers in scored.jsonl and refuse
    # the score if any: judge flakiness must not reach the ledger.
    judge_errs = sum(
        1 for row in _read_jsonl(out / "scored.jsonl")
        for readout in (row.get("scores") or {}).values()
        if readout.get("judge_error"))
    if judge_errs:
        raise SuiteError(
            f"{judge_errs} judge error(s) in {out}/scored.jsonl — scores "
            f"are zero-poisoned; re-run the judge against this run dir")
    summary = _read_json(out / "summary.json")
    readouts = summary.get("overall_by_readout") or {}
    vals = [v for v in (readouts.get("slash"), readouts.get("agent"))
            if isinstance(v, (int, float))]
    if not vals:
        raise SuiteError(f"no overall_by_readout scores in {out}/summary.json")
    return statistics.mean(vals), {
        "by_readout": readouts,
        "n_probes": summary.get("n_probes"),
        "durations_s": {"runner": round(d1, 1), "judge": round(d2, 1)},
    }


def run_hle(run_dir: Path, hle_scenario: Path, smoke: bool):
    out = run_dir / "hle"
    log = run_dir / "logs" / "hle.log"
    n = HLE_SMOKE_QUESTIONS if smoke else HLE_MAX_QUESTIONS
    d1 = _sh([sys.executable, BENCH / "hle" / "runner.py",
              "--scenario", hle_scenario, "--start", HLE_START,
              "--max-questions", n, "--output-dir", out],
             log, cwd=REPO / "src")
    ids = [r.get("question_id") for r in _read_jsonl(out / "raw.jsonl")]
    pin_note = None
    if not smoke:
        if PIN_FILE.is_file():
            pinned = json.loads(PIN_FILE.read_text(encoding="utf-8"))
            if pinned.get("question_ids") != ids:
                raise SuiteError(
                    "HLE question set drifted from hle_pinned_questions.json "
                    "(HF dataset order changed?). Baseline comparability is "
                    "broken — resolve before appending to the ledger.")
        else:
            PIN_FILE.write_text(json.dumps({
                "dataset": "cais/hle", "split": "test",
                "start": HLE_START, "max_questions": HLE_MAX_QUESTIONS,
                "question_ids": ids,
                "pinned_at": datetime.now(timezone.utc).isoformat(),
            }, indent=2) + "\n", encoding="utf-8")
            pin_note = f"hle_pinned_questions.json created ({len(ids)} ids)"
            print(f"    {pin_note}")
    d2 = _sh([sys.executable, BENCH / "hle" / "judge.py",
              "--run-dir", out, "--judge-model", JUDGE_MODEL], log, cwd=REPO)
    summary = _read_json(out / "summary.json")
    overall = summary.get("overall") or {}
    score = overall.get("score_mean")
    if not isinstance(score, (int, float)):
        raise SuiteError(f"no overall.score_mean in {out}/summary.json")
    judge_errors = summary.get("judge_errors",
                               overall.get("judge_errors", 0)) or 0
    # HLE judge failures are EXCLUDED from the denominator (judge.py
    # computes score_sum/(n or 1)); with n=0 score_mean is a fake 0.0
    # and with any exclusion the score is over a different question set
    # than the pin. Either way the row is not comparable — refuse it.
    if not overall.get("n"):
        raise SuiteError(
            f"HLE judged 0 questions (all judge calls failed?) — "
            f"score_mean in {out}/summary.json is meaningless")
    if judge_errors:
        raise SuiteError(
            f"{judge_errors} HLE judge error(s); score_mean denominator "
            f"is n={overall.get('n')} not {len(ids)} — re-run the judge "
            f"against this run dir")
    return float(score), {
        "exact_correct_rate": overall.get("exact_correct_rate"),
        "n": overall.get("n"), "judge_errors": judge_errors,
        "pin_note": pin_note,
        "durations_s": {"runner": round(d1, 1), "judge": round(d2, 1)},
    }


def run_discourse_reflect(run_dir: Path, chat_scenario: Path, smoke: bool):
    # rerun_baseline mutates pair JSONs in place — work on a copy so the
    # checked-in pairs stay pristine and each run is self-contained.
    pairs = run_dir / "discourse_reflect" / "pairs"
    scores = run_dir / "discourse_reflect" / "scores"
    log = run_dir / "logs" / "discourse_reflect.log"
    shutil.copytree(BENCH / "discourse_reflect" / "pairs", pairs)
    limit = ["--limit", "2"] if smoke else []
    d1 = _sh([sys.executable, BENCH / "discourse_reflect" / "rerun_baseline.py",
              "--scenario", chat_scenario, "--pairs-dir", pairs,
              "--output-field", DISCOURSE_OUTPUT_FIELD, *limit],
             log, cwd=REPO / "src")
    d2 = _sh([sys.executable, BENCH / "discourse_reflect" / "judge.py",
              "--pairs-dir", pairs, "--scores-dir", scores,
              "--output-field", DISCOURSE_OUTPUT_FIELD,
              "--model", JUDGE_MODEL, *limit], log, cwd=REPO)
    agg = _read_json(scores / "aggregate.json")
    # The discourse judge exits 0 even when pairs fail to judge; a
    # shrunken pool means the axis means cover a different pair set
    # than the pin — the same comparability break as HLE judge_errors.
    expected_pairs = 2 if smoke else len(list(pairs.glob("pair_*.json")))
    pool = agg.get("n_pairs_in_pool")
    if pool != expected_pairs:
        raise SuiteError(
            f"discourse judged pool {pool} != expected {expected_pairs} "
            f"pair(s) — judge dropped pairs; re-run the judge against "
            f"{pairs}")
    axes = agg.get("axes") or {}
    means = {k: (v or {}).get("mean") for k, v in axes.items()}
    vals = [v for v in means.values() if isinstance(v, (int, float))]
    if not vals:
        raise SuiteError(f"no scored axes in {scores}/aggregate.json")
    return statistics.mean(vals), {
        "axes": means,
        "null_axes": sorted(k for k, v in means.items() if v is None),
        "durations_s": {"runner": round(d1, 1), "judge": round(d2, 1)},
    }


def run_introspective_fidelity(run_dir: Path, chat_scenario: Path, smoke: bool):
    out = run_dir / "introspective_fidelity"
    log = run_dir / "logs" / "introspective_fidelity.log"
    # no smaller smoke shape exists (primer is a fixed 12-probe script);
    # smoke runs it whole — it is the lightest suite.
    d1 = _sh([sys.executable, BENCH / "introspective_fidelity" / "runner.py",
              "--scenario", chat_scenario, "--output-dir", out],
             log, cwd=REPO / "src")
    d2 = _sh([sys.executable, BENCH / "introspective_fidelity" / "judge.py",
              "--run-dir", out, "--judge-model", JUDGE_MODEL], log, cwd=REPO)
    scores = _read_json(out / "scores.json")
    total, max_total = scores.get("total"), scores.get("max_total")
    if not isinstance(total, (int, float)) or not max_total:
        raise SuiteError(f"no total/max_total in {out}/scores.json")
    return float(total) / float(max_total), {
        "total": total, "max_total": max_total,
        "axis_profile": scores.get("axis_profile"),
        "durations_s": {"runner": round(d1, 1), "judge": round(d2, 1)},
    }


RUNNERS = {
    "memory_recall": lambda rd, a: run_memory_recall(rd, a.chat_scenario, a.smoke),
    "hle": lambda rd, a: run_hle(rd, a.hle_scenario, a.smoke),
    "discourse_reflect": lambda rd, a: run_discourse_reflect(rd, a.chat_scenario, a.smoke),
    "introspective_fidelity": lambda rd, a: run_introspective_fidelity(rd, a.chat_scenario, a.smoke),
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--suites", nargs="+", choices=SUITES, default=list(SUITES),
                    help="subset of suites (partial runs never reach the ledger)")
    ap.add_argument("--smoke", action="store_true",
                    help="tiny plumbing run; never reaches the ledger")
    ap.add_argument("--allow-dirty", action="store_true",
                    help="run despite tracked-file changes (row marked dirty)")
    ap.add_argument("--note", default="",
                    help="free text for the ledger row — what changed since last run")
    ap.add_argument("--chat-scenario", type=Path, default=DEFAULT_CHAT_SCENARIO,
                    help="override pinned chat scenario (e.g. frontier backend)")
    ap.add_argument("--hle-scenario", type=Path, default=DEFAULT_HLE_SCENARIO,
                    help="override pinned HLE scenario")
    args = ap.parse_args()
    # Runners are spawned with cwd=src/ — relative overrides like
    # scenarios/jill-benchmark-chat-opus.yaml must be absolutized against
    # the invoker's cwd first or every suite fails file-not-found.
    args.chat_scenario = args.chat_scenario.resolve()
    args.hle_scenario = args.hle_scenario.resolve()
    for p in (args.chat_scenario, args.hle_scenario):
        if not p.is_file():
            raise SystemExit(f"scenario not found: {p}")

    import os
    if not os.environ.get("CLAUDE_API_KEY"):
        raise SystemExit("CLAUDE_API_KEY not set (judges need it)")
    dirty = _git_dirty()
    ledger_intent = not args.smoke and sorted(args.suites) == sorted(SUITES)
    if dirty and ledger_intent and not args.allow_dirty:
        raise SystemExit(
            "tracked tree is dirty — the ledger row's harness_commit would "
            "not describe what ran. Commit first, or pass --allow-dirty.")

    backend = _backend_from_scenario(args.chat_scenario)
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    run_dir = RESULTS / (stamp + ("-smoke" if args.smoke else ""))
    run_dir.mkdir(parents=True, exist_ok=False)
    print(f"composite run → {run_dir}")
    print(f"  backend {backend['server']}:{backend['model']} @ {backend['host']}")

    suites: dict = {}
    detail: dict = {}
    failed: dict = {}
    for name in SUITES:
        if name not in args.suites:
            continue
        print(f"  [{name}]")
        try:
            score, d = RUNNERS[name](run_dir, args)
            suites[name] = round(score, 4)
            detail[name] = d
            print(f"    score {score:.4f}")
        except SuiteError as e:
            failed[name] = str(e)
            print(f"    FAILED: {e}")
        except Exception as e:
            # Any other crash (bad JSON, missing dirs) must not abort
            # the run record — hours of completed suite work would be
            # recorded nowhere. Capture and continue.
            import traceback
            failed[name] = f"{type(e).__name__}: {e}\n{traceback.format_exc()}"
            print(f"    CRASHED: {type(e).__name__}: {e}")

    entry = {
        "ts": stamp,
        "harness_commit": _git_commit(),
        "dirty": dirty,
        "backend": backend,
        "judge": {"model": JUDGE_MODEL, "code_sha": _judge_code_sha()},
        "scenarios": {"chat": str(args.chat_scenario.relative_to(REPO))
                      if args.chat_scenario.is_relative_to(REPO) else str(args.chat_scenario),
                      "hle": str(args.hle_scenario.relative_to(REPO))
                      if args.hle_scenario.is_relative_to(REPO) else str(args.hle_scenario)},
        "suites": suites,
        "composite": round(statistics.mean(suites.values()), 4) if suites else None,
        "tokens": {"agent": None, "judge": None},
        "smoke": args.smoke,
        "partial": sorted(args.suites) != sorted(SUITES),
        "failed_suites": failed,
        "detail": detail,
        "run_dir": str(run_dir.relative_to(REPO)),
        "note": args.note,
    }
    (run_dir / "entry.json").write_text(
        json.dumps(entry, indent=2) + "\n", encoding="utf-8")

    ledger_eligible = not args.smoke and not entry["partial"] and not failed
    if ledger_eligible:
        with open(LEDGER, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        print(f"\nledger ← composite {entry['composite']}  ({LEDGER})")
    else:
        why = ("smoke" if args.smoke else
               "partial" if entry["partial"] else "suite failure")
        print(f"\nNOT appended to ledger ({why}); entry at {run_dir}/entry.json")

    print(f"\n{'suite':<26} score")
    for name, s in suites.items():
        print(f"{name:<26} {s:.4f}")
    for name in failed:
        print(f"{name:<26} FAILED")
    if entry["composite"] is not None:
        print(f"{'composite':<26} {entry['composite']:.4f}")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
