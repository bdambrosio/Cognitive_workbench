#!/usr/bin/env python3
"""HLE (Humanity's Last Exam) benchmark — runner.

Per question: fresh ChatLoop with a unique world_name, one user turn
carrying the question + answer-format instruction, capture the reply
verbatim. Scoring happens in a separate phase (judge.py).

Open-book — the scenario keeps all tools enabled (web search, fetch,
process_text). HLE expects retrieval where useful.

Usage (run from src/ so launcher imports resolve):
    cd src
    python ../bench/hle/runner.py \\
        --scenario ../scenarios/jill-benchmark-hle.yaml

    # Subsample for smoke test:
    python ../bench/hle/runner.py \\
        --scenario ../scenarios/jill-benchmark-hle.yaml \\
        --max-questions 5

Then score:
    python ../bench/hle/judge.py \\
        --run-dir ../bench/hle/results/<timestamp>_<scenario_stem>
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"

sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import ChatLoop  # noqa: E402
from launcher import parse_characters  # noqa: E402

logger = logging.getLogger("bench.hle")

SOURCE = "User"


# ---------------------------------------------------------------------------
# Scenario loading (mirrors bench/memory_recall/runner.py)
# ---------------------------------------------------------------------------

def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _build_character_config(scenario_path: Path, world_name_override: str
                            ) -> tuple[str, dict]:
    scenario = _load_yaml(scenario_path)
    llm_cfg = scenario.get("llm_config") or {}
    alt_llm = scenario.get("alt_llm_config") or {}
    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world_name_override
    setting = scenario.get("setting", "")
    chars = parse_characters(scenario, llm_cfg, world_cfg, setting, alt_llm)
    chat_chars = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat_chars) != 1:
        raise RuntimeError(
            f"expected exactly 1 chat character in {scenario_path}, found "
            f"{len(chat_chars)}")
    return chat_chars[0]


# ---------------------------------------------------------------------------
# Dataset field tolerance
# ---------------------------------------------------------------------------

_QUESTION_KEYS = ("question", "Question", "prompt")
_ANSWER_KEYS = ("answer", "Answer", "gold_answer")
_ID_KEYS = ("id", "Id", "question_id")
_CATEGORY_KEYS = ("category", "raw_subject", "subject")
_ANSWER_TYPE_KEYS = ("answer_type", "type")


def _pick(item: Dict[str, Any], keys: tuple) -> str:
    for k in keys:
        v = item.get(k)
        if v not in (None, ""):
            return str(v)
    return ""


# ---------------------------------------------------------------------------
# Prompt
# ---------------------------------------------------------------------------

def _build_prompt(question: str, answer_type: str) -> str:
    """Single user turn carrying the question plus an answer-format
    instruction. The judge expects a clear committed answer; the format
    nudge keeps replies extractable even when reasoning is verbose."""
    lines: List[str] = []
    lines.append("Research question:")
    lines.append(question.strip())
    lines.append("")
    if answer_type and "multiple" in answer_type.lower():
        lines.append("Commit to a single option. End with a line:")
        lines.append("ANSWER: <your choice>")
    else:
        lines.append("Reason as needed, then commit to a single concrete answer. End with a line:")
        lines.append("ANSWER: <your answer>")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Reply capture
# ---------------------------------------------------------------------------

def _latest_reply(loop: ChatLoop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if t.get("direction") == "out":
            return str(t.get("text", ""))
    return ""


# ---------------------------------------------------------------------------
# Per-question session
# ---------------------------------------------------------------------------

def _run_question(scenario_path: Path, world_name: str, question_idx: int,
                  item: Dict[str, Any]) -> Dict[str, Any]:
    """Spin up one ChatLoop, ask one question, tear down. Returns the
    record ready for raw.jsonl."""
    captured_at = datetime.datetime.now(datetime.timezone.utc).isoformat()
    qid = _pick(item, _ID_KEYS) or f"idx-{question_idx}"
    question = _pick(item, _QUESTION_KEYS)
    gold = _pick(item, _ANSWER_KEYS)
    category = _pick(item, _CATEGORY_KEYS)
    answer_type = _pick(item, _ANSWER_TYPE_KEYS)

    char_name, char_config = _build_character_config(scenario_path, world_name)
    prompt = _build_prompt(question, answer_type)

    loop = ChatLoop(character_name=char_name, character_config=char_config)
    t0 = datetime.datetime.now()
    reply = ""
    error: Optional[str] = None
    try:
        loop._process_user_turn(source=SOURCE, text=prompt, close=False)
        reply = _latest_reply(loop, SOURCE)
    except Exception as e:
        error = f"{type(e).__name__}: {e}"
        logger.exception(f"[{qid}] turn failed: {e}")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:
            logger.warning(f"[{qid}] executor shutdown failed: {e}")
        try:
            loop._persist_to_disk()
        except Exception as e:
            logger.warning(f"[{qid}] final persist failed: {e}")
    dt = (datetime.datetime.now() - t0).total_seconds()

    return {
        "question_id": qid,
        "question_index": question_idx,
        "question": question,
        "gold": gold,
        "category": category,
        "answer_type": answer_type,
        "reply": reply,
        "error": error,
        "duration_s": round(dt, 3),
        "world_name": world_name,
        "character_name": char_name,
        "captured_at": captured_at,
    }


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def run_benchmark(scenario_path: Path, dataset_name: str, split: str,
                  max_questions: Optional[int], start: int,
                  output_dir: Path) -> Path:
    from datasets import load_dataset

    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "raw.jsonl"

    print(f"Loading {dataset_name} split={split}…", flush=True)
    ds = load_dataset(dataset_name, split=split)
    print(f"Loaded {len(ds)} rows.", flush=True)

    valid: List[tuple[int, Dict[str, Any]]] = []
    skipped_image = 0
    for i, item in enumerate(ds):
        q = _pick(item, _QUESTION_KEYS)
        a = _pick(item, _ANSWER_KEYS)
        if not q or not a:
            continue
        # v1: skip multimodal questions. Images need separate plumbing
        # to ride through ChatLoop's image_url path; tackle later.
        img = item.get("image")
        if isinstance(img, str) and img.strip():
            skipped_image += 1
            continue
        if isinstance(img, dict) and (img.get("url") or img.get("bytes")):
            skipped_image += 1
            continue
        valid.append((i, item))

    if skipped_image:
        print(f"(skipped {skipped_image} multimodal question(s))", flush=True)

    if start > 0:
        valid = valid[start:]
    if max_questions is not None:
        valid = valid[:max_questions]
    print(f"Running {len(valid)} questions.", flush=True)

    run_stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H-%M-%SZ")

    n_ok = 0
    n_err = 0
    with open(raw_path, "w") as raw_f:
        for k, (orig_idx, item) in enumerate(valid):
            world_name = f"bench-hle-{run_stamp}-{orig_idx:05d}"
            try:
                rec = _run_question(scenario_path, world_name, orig_idx, item)
            except Exception as e:
                tb = traceback.format_exc()
                logger.error(
                    f"[hle#{orig_idx}] question crashed outside session: "
                    f"{e}\n{tb}")
                rec = {
                    "question_id": _pick(item, _ID_KEYS) or f"idx-{orig_idx}",
                    "question_index": orig_idx,
                    "question": _pick(item, _QUESTION_KEYS),
                    "gold": _pick(item, _ANSWER_KEYS),
                    "category": _pick(item, _CATEGORY_KEYS),
                    "reply": "",
                    "error": f"{type(e).__name__}: {e}",
                    "world_name": world_name,
                    "captured_at": datetime.datetime.now(
                        datetime.timezone.utc).isoformat(),
                }
            raw_f.write(json.dumps(rec, default=str) + "\n")
            raw_f.flush()
            if rec.get("error"):
                n_err += 1
            else:
                n_ok += 1
            print(
                f"[hle] {k+1}/{len(valid)} idx={orig_idx} "
                f"({rec.get('duration_s', 0):.1f}s)"
                f"{' ERR' if rec.get('error') else ''}",
                flush=True)

    run_meta = {
        "run_stamp": run_stamp,
        "scenario": str(scenario_path),
        "dataset": dataset_name,
        "split": split,
        "max_questions": max_questions,
        "start": start,
        "n_attempted": len(valid),
        "n_ok": n_ok,
        "n_error": n_err,
        "n_skipped_image": skipped_image,
    }
    (output_dir / "run_meta.json").write_text(
        json.dumps(run_meta, indent=2, default=str))

    print(f"\nWrote raw records: {raw_path}")
    print(f"Run metadata:      {output_dir / 'run_meta.json'}")
    print(f"OK/Errors:         {n_ok}/{n_err}")
    print(f"\nNext: score with bench/hle/judge.py --run-dir {output_dir}")
    return raw_path


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="HLE benchmark runner. Per question: fresh ChatLoop "
                    "with timestamped world_name, single user turn, capture "
                    "reply verbatim. Scoring runs in a separate phase "
                    "(bench/hle/judge.py).")
    parser.add_argument(
        "--scenario", type=Path, required=True,
        help="Path to a chat-mode scenario YAML "
             "(e.g. scenarios/jill-benchmark-hle.yaml).")
    parser.add_argument(
        "--dataset", type=str, default="cais/hle",
        help="HuggingFace dataset name (default cais/hle).")
    parser.add_argument(
        "--split", type=str, default="test",
        help="Dataset split (default test).")
    parser.add_argument(
        "--max-questions", type=int, default=10,
        help="Cap on number of questions to run (default 10 for smoke runs).")
    parser.add_argument(
        "--start", type=int, default=0,
        help="Skip the first N valid questions (0-based offset).")
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Where to write raw.jsonl + run_meta.json. Defaults to "
             "bench/hle/results/<timestamp>_<scenario_stem>.")
    args = parser.parse_args()

    scenario = args.scenario.resolve()
    if args.output_dir:
        out = args.output_dir.resolve()
    else:
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H-%M-%SZ")
        out = HERE / "results" / f"{stamp}_{scenario.stem}"

    run_benchmark(
        scenario_path=scenario,
        dataset_name=args.dataset,
        split=args.split,
        max_questions=args.max_questions,
        start=args.start,
        output_dir=out,
    )


if __name__ == "__main__":
    main()
