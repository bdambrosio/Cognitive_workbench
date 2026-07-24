#!/usr/bin/env python3
"""disposition/g1 — G1 offline toy experiment (docs/learned-disposition-design.md).

Question: from fire-time context alone, can a small trained model predict
the judged outcome of an autonomous fire better than (a) base rate and
(b) the prompted local backend asked the same question?

Data: `fire_outcome` records joined to their `fire` events by fire_id in
a live autonomy.jsonl (read-only). Label is binary: helped+neutral = good
(0) vs hindered+ignored = bad (1). Features are strictly fire-time —
concern text, instruction, clock, and same-concern recency computed from
the fire stream; nothing observed after the fire (no react_iters /
exit_reason / response fields).

Models:
  - embed+lr    frozen MiniLM embedding of concern+instruction, plus
                numeric recency features, logistic head; leave-one-out
  - numeric-lr  numeric features only (does text matter?); leave-one-out
  - prompted    zero-shot: the local backend predicts p(lands badly)
                (--prompted; run only when the backend is not mid-bench)

Eval: LOO scores pooled → AUROC / average precision with bootstrap CIs;
--loco swaps LOO for leave-one-concern-out (per-concern leakage check).

Usage:
    python bench/disposition/g1.py                 # embed+lr, numeric-lr
    python bench/disposition/g1.py --loco
    python bench/disposition/g1.py --prompted      # adds backend baseline
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "src"))

from utils.json_utils import repair_json_string  # noqa: E402

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
logger = logging.getLogger("disposition.g1")

DEFAULT_LOG = REPO / "scenarios" / "jill_chat" / "Jill" / "memory" / "autonomy.jsonl"
RESULTS = HERE / "results"
EMBEDDER = "sentence-transformers/all-MiniLM-L6-v2"
GOOD, BAD = ("helped", "neutral"), ("hindered", "ignored")

PROMPT = """An autonomous companion agent decided to act on a standing concern \
without being asked. Predict how the user will judge this action, from the \
fire-time context alone.

Concern: {concern}
Instruction the agent gave itself: {instruction}
Local time of the action: {when}

"Lands badly" means the user is hindered by it or ignores it; "lands well" \
means the user finds it helpful or at least neutral. Reply with only a JSON \
object: {{"p_lands_badly": <float 0.0-1.0>}}"""


def load_examples(log_path: Path):
    """Join judged fire_outcome records to their fire events by fire_id.
    Returns (examples, all_fires) — all_fires kept for recency features."""
    fires, outcomes, all_fires = {}, [], []
    for lineno, line in enumerate(log_path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError as e:
            logger.warning("skipping unparseable line %d: %s", lineno, e)
            continue
        if r.get("event") == "fire":
            all_fires.append(r)
            if r.get("fire_id"):
                fires[r["fire_id"]] = r
        elif r.get("event") == "fire_outcome" and r.get("outcome") in GOOD + BAD:
            outcomes.append(r)

    examples, unjoined = [], 0
    for o in outcomes:
        f = fires.get(o.get("fire_id"))
        if f is None:
            unjoined += 1
            continue
        examples.append({
            "fire_id": o["fire_id"],
            "concern_id": o.get("concern_id", ""),
            "concern_text": f.get("concern_text", ""),
            "instruction": f.get("instruction", ""),
            "started_at": f.get("started_at", ""),
            "outcome": o["outcome"],
            "y": 1 if o["outcome"] in BAD else 0,
        })
    if unjoined:
        logger.info("dropped %d judged outcome(s) with no joinable fire event", unjoined)
    return examples, all_fires


def numeric_features(examples, all_fires):
    """Fire-time-only numeric features: clock plus same-concern recency."""
    by_concern = {}
    for f in all_fires:
        ts = f.get("started_at")
        if ts:
            by_concern.setdefault(f.get("concern_id"), []).append(
                datetime.fromisoformat(ts))
    for times in by_concern.values():
        times.sort()

    rows = []
    for ex in examples:
        t = datetime.fromisoformat(ex["started_at"])
        hour = t.hour + t.minute / 60.0
        prior = [p for p in by_concern.get(ex["concern_id"], []) if p < t]
        prev_gap_h = (t - prior[-1]).total_seconds() / 3600.0 if prior else 720.0
        last24 = sum(1 for p in prior if (t - p).total_seconds() <= 86400)
        rows.append([
            math.sin(2 * math.pi * hour / 24), math.cos(2 * math.pi * hour / 24),
            math.log1p(min(prev_gap_h, 720.0)), 1.0 if prior else 0.0,
            math.log1p(last24), math.log1p(len(prior)),
        ])
    return np.array(rows)


def loo_scores(X, y, groups=None):
    """Pooled held-out scores from leave-one-out (or leave-one-group-out)."""
    from sklearn.linear_model import LogisticRegression
    from sklearn.model_selection import LeaveOneGroupOut, LeaveOneOut
    from sklearn.preprocessing import StandardScaler

    splitter = LeaveOneOut() if groups is None else LeaveOneGroupOut()
    scores = np.full(len(y), np.nan)
    for train, test in splitter.split(X, y, groups):
        if len(set(y[train])) < 2:
            logger.warning("fold with single-class training set; scoring 0.5")
            scores[test] = 0.5
            continue
        scaler = StandardScaler().fit(X[train])
        clf = LogisticRegression(class_weight="balanced", max_iter=2000)
        clf.fit(scaler.transform(X[train]), y[train])
        scores[test] = clf.predict_proba(scaler.transform(X[test]))[:, 1]
    return scores


def prompted_scores(examples, vllm_url):
    """Zero-shot p(lands badly) from the local backend via _ChatBackend —
    the same client ChatLoop (and live triage) uses, chat template and all."""
    from chat.backend import _ChatBackend

    llm = _ChatBackend(server="local", model="", base_url=vllm_url)
    logger.info("prompted baseline against %s", vllm_url)
    scores, failures = [], 0
    for i, ex in enumerate(examples):
        when = datetime.fromisoformat(ex["started_at"]).strftime("%A %H:%M")
        text = llm.chat(
            [{"role": "user", "content": PROMPT.format(
                concern=ex["concern_text"], instruction=ex["instruction"],
                when=when)}],
            temperature=0.0, max_tokens=120, is_json=True)
        parsed = repair_json_string(text) if isinstance(text, str) else None
        p = parsed.get("p_lands_badly") if isinstance(parsed, dict) else None
        if isinstance(p, (int, float)) and 0.0 <= p <= 1.0:
            scores.append(float(p))
        else:
            failures += 1
            logger.warning("unparseable prediction %d/%d: %r", i + 1,
                           len(examples), text if isinstance(text, str) else type(text))
            scores.append(0.5)
        if (i + 1) % 10 == 0:
            logger.info("  prompted %d/%d", i + 1, len(examples))
    if failures:
        logger.info("prompted baseline: %d/%d unparseable (scored 0.5)",
                    failures, len(examples))
    return np.array(scores)


def metrics(y, scores, n_boot=2000, seed=0):
    from sklearn.metrics import average_precision_score, roc_auc_score

    auroc = roc_auc_score(y, scores)
    ap = average_precision_score(y, scores)
    rng = np.random.default_rng(seed)
    boots = []
    for _ in range(n_boot):
        idx = rng.integers(0, len(y), len(y))
        if len(set(y[idx])) < 2:
            continue
        boots.append(roc_auc_score(y[idx], scores[idx]))
    lo, hi = np.percentile(boots, [2.5, 97.5])
    return {"auroc": round(auroc, 3), "auroc_ci95": [round(lo, 3), round(hi, 3)],
            "avg_precision": round(ap, 3)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--log", type=Path, default=DEFAULT_LOG)
    ap.add_argument("--loco", action="store_true",
                    help="leave-one-concern-out instead of leave-one-out")
    ap.add_argument("--prompted", action="store_true",
                    help="also run the zero-shot backend baseline")
    ap.add_argument("--vllm-url", default="http://127.0.0.1:5000")
    args = ap.parse_args()

    examples, all_fires = load_examples(args.log)
    y = np.array([ex["y"] for ex in examples])
    logger.info("%d judged examples: %d good, %d bad (prevalence %.3f)",
                len(y), (y == 0).sum(), (y == 1).sum(), y.mean())

    num = numeric_features(examples, all_fires)

    from sentence_transformers import SentenceTransformer
    texts = [f"{ex['concern_text']}\n{ex['instruction']}" for ex in examples]
    emb = SentenceTransformer(EMBEDDER, device="cpu").encode(
        texts, show_progress_bar=False, normalize_embeddings=True)

    groups = np.array([ex["concern_id"] for ex in examples]) if args.loco else None
    results = {
        "embed+lr": {},
        "numeric-lr": {},
    }
    per_record_scores = {}
    for name, X in (("embed+lr", np.hstack([emb, num])), ("numeric-lr", num)):
        scores = loo_scores(X, y, groups)
        results[name] = metrics(y, scores)
        per_record_scores[name] = scores.tolist()

    if args.prompted:
        scores = prompted_scores(examples, args.vllm_url)
        results["prompted"] = metrics(y, scores)
        per_record_scores["prompted"] = scores.tolist()

    split = "leave-one-concern-out" if args.loco else "leave-one-out"
    print(f"\nG1 toy — {len(y)} judged fires, {split}, "
          f"base rate p(bad) = {y.mean():.3f} (AUROC 0.5 by construction)")
    for name, m in results.items():
        print(f"  {name:12s} AUROC {m['auroc']:.3f} "
              f"[{m['auroc_ci95'][0]:.3f}, {m['auroc_ci95'][1]:.3f}]   "
              f"AP {m['avg_precision']:.3f}")

    RESULTS.mkdir(exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    out = RESULTS / f"g1-{stamp}{'-loco' if args.loco else ''}.json"
    out.write_text(json.dumps({
        "ts": stamp, "log": str(args.log), "split": split,
        "n": len(y), "n_bad": int(y.sum()), "embedder": EMBEDDER,
        "results": results,
        "records": [{**{k: ex[k] for k in ("fire_id", "concern_id", "outcome", "y")},
                     **{f"score_{m}": per_record_scores[m][i]
                        for m in per_record_scores}}
                    for i, ex in enumerate(examples)],
    }, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
