#!/usr/bin/env python3
"""MMLU benchmark — runner.

Per question: fresh ChatLoop with a unique world_name, one user turn
carrying the question + k-shot examples + "ANSWER: X" instruction,
extract A/B/C/D from the reply, score against the gold letter.

Closed-book by default — the scenario sets chat.omitted_tools to drop
every external-info tool. Score is exact-letter match; no LLM judge.

Usage (run from src/ so launcher imports resolve):
    cd src
    python ../bench/mmlu/runner.py \\
        --scenario ../scenarios/jill-benchmark-mmlu.yaml

    # Subject filter (lexicographic order preserved):
    python ../bench/mmlu/runner.py \\
        --scenario ../scenarios/jill-benchmark-mmlu.yaml \\
        --subjects high_school_physics formal_logic

    # Full MMLU (all 57 subjects):
    python ../bench/mmlu/runner.py \\
        --scenario ../scenarios/jill-benchmark-mmlu.yaml --subjects all
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import re
import sys
import traceback
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"

sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import ChatLoop  # noqa: E402
from launcher import parse_characters  # noqa: E402

logger = logging.getLogger("bench.mmlu")

SOURCE = "User"
ANSWER_LETTERS = ("A", "B", "C", "D")

# Canonical 57-subject MMLU set. Used when --subjects all is passed.
ALL_SUBJECTS = [
    "abstract_algebra", "anatomy", "astronomy", "business_ethics",
    "clinical_knowledge", "college_biology", "college_chemistry",
    "college_computer_science", "college_mathematics", "college_medicine",
    "college_physics", "computer_security", "conceptual_physics",
    "econometrics", "electrical_engineering", "elementary_mathematics",
    "formal_logic", "global_facts", "high_school_biology",
    "high_school_chemistry", "high_school_computer_science",
    "high_school_european_history", "high_school_geography",
    "high_school_government_and_politics", "high_school_macroeconomics",
    "high_school_mathematics", "high_school_microeconomics",
    "high_school_physics", "high_school_psychology", "high_school_statistics",
    "high_school_us_history", "high_school_world_history", "human_aging",
    "human_sexuality", "international_law", "jurisprudence",
    "logical_fallacies", "machine_learning", "management", "marketing",
    "medical_genetics", "miscellaneous", "moral_disputes", "moral_scenarios",
    "nutrition", "philosophy", "prehistory", "professional_accounting",
    "professional_law", "professional_medicine", "professional_psychology",
    "public_relations", "security_studies", "sociology", "us_foreign_policy",
    "virology", "world_religions",
]

# Small default subset for quick smoke runs.
DEFAULT_SUBJECTS = [
    "high_school_physics", "high_school_world_history", "college_mathematics",
    "formal_logic", "college_medicine", "us_foreign_policy",
]

# Patterns for extracting the model's final letter choice. Output-parsing,
# not classification — the prompt asks for "ANSWER: X" explicitly.
_ANSWER_RE = re.compile(r"ANSWER\s*:\s*([ABCD])", re.IGNORECASE)
_CHOICE_RE = re.compile(
    r"(?:choice|option|matches|answer\s+is)\s+([ABCD])\b", re.IGNORECASE)


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
# Prompt + extraction
# ---------------------------------------------------------------------------

def _build_prompt(question: str, choices: List[str],
                  few_shots: List[Dict[str, Any]]) -> str:
    """Single user turn carrying few-shots + question + answer instruction.

    few_shots: list of {"question", "choices", "answer"} dicts (answer is a
    letter). Empty list → zero-shot.
    """
    lines: List[str] = []
    if few_shots:
        lines.append("Worked examples:")
        lines.append("")
        for i, ex in enumerate(few_shots, start=1):
            lines.append(f"Example {i}:")
            lines.append(f"Q: {ex['question'].strip()}")
            lines.append("Choices:")
            c = ex["choices"]
            lines.append(f"A: {c[0].strip()}")
            lines.append(f"B: {c[1].strip()}")
            lines.append(f"C: {c[2].strip()}")
            lines.append(f"D: {c[3].strip()}")
            lines.append(f"ANSWER: {ex['answer'].strip().upper()}")
            lines.append("")
        lines.append("")

    lines.append("Question:")
    lines.append(question.strip())
    lines.append("")
    lines.append("Choices:")
    lines.append(f"A: {choices[0].strip()}")
    lines.append(f"B: {choices[1].strip()}")
    lines.append(f"C: {choices[2].strip()}")
    lines.append(f"D: {choices[3].strip()}")
    lines.append("")
    lines.append("Reason briefly, then end with a single line:")
    lines.append("ANSWER: X")
    lines.append("where X is A, B, C, or D.")
    return "\n".join(lines)


def _extract_choice(text: str) -> Optional[str]:
    """Pull the final A/B/C/D from the model output. Last match wins so
    a stray mention of an option mid-reasoning doesn't override the
    committed answer."""
    if not text:
        return None
    matches = _ANSWER_RE.findall(text)
    if matches:
        return matches[-1].upper()
    choice_matches = _CHOICE_RE.findall(text)
    if choice_matches:
        return choice_matches[-1].upper()
    stripped = text.strip()
    if stripped and stripped[0] in "ABCD":
        return stripped[0]
    letters = re.findall(r"\b([ABCD])\b", text)
    if letters:
        return letters[-1].upper()
    return None


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

def _run_question(scenario_path: Path, world_name: str, subject: str,
                  question_idx: int, question: str, choices: List[str],
                  gold: str, few_shots: List[Dict[str, Any]]
                  ) -> Dict[str, Any]:
    """Spin up one ChatLoop, ask one question, tear down. Returns the
    record ready for raw.jsonl."""
    captured_at = datetime.datetime.now(datetime.timezone.utc).isoformat()
    char_name, char_config = _build_character_config(scenario_path, world_name)

    prompt = _build_prompt(question, choices, few_shots)
    loop = ChatLoop(character_name=char_name, character_config=char_config)
    t0 = datetime.datetime.now()
    reply = ""
    error: Optional[str] = None
    try:
        loop._process_user_turn(source=SOURCE, text=prompt, close=False)
        reply = _latest_reply(loop, SOURCE)
    except Exception as e:
        error = f"{type(e).__name__}: {e}"
        logger.exception(f"[{subject}#{question_idx}] turn failed: {e}")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:
            logger.warning(
                f"[{subject}#{question_idx}] executor shutdown failed: {e}")
        try:
            loop._persist_to_disk()
        except Exception as e:
            logger.warning(
                f"[{subject}#{question_idx}] final persist failed: {e}")
    dt = (datetime.datetime.now() - t0).total_seconds()

    pred = _extract_choice(reply) if not error else None
    correct = (pred == gold) if pred is not None else False

    return {
        "subject": subject,
        "question_index": question_idx,
        "question": question,
        "choices": list(choices),
        "gold": gold,
        "prediction": pred,
        "correct": correct,
        "reply": reply,
        "error": error,
        "duration_s": round(dt, 3),
        "world_name": world_name,
        "character_name": char_name,
        "captured_at": captured_at,
    }


# ---------------------------------------------------------------------------
# Subject driver
# ---------------------------------------------------------------------------

def _run_subject(scenario_path: Path, subject: str, run_stamp: str,
                 k_shot: int, max_test: Optional[int]
                 ) -> List[Dict[str, Any]]:
    """Load one subject's dev (few-shots) + test splits, run all questions."""
    from datasets import load_dataset

    print(f"\n========== subject {subject} (k={k_shot}) ==========", flush=True)
    ds_dev = load_dataset("cais/mmlu", subject, split="dev")
    ds_test = load_dataset("cais/mmlu", subject, split="test")

    few_shots: List[Dict[str, Any]] = []
    if k_shot > 0:
        for ex in ds_dev.select(range(min(k_shot, len(ds_dev)))):
            few_shots.append({
                "question": ex["question"],
                "choices": list(ex["choices"]),
                "answer": ANSWER_LETTERS[int(ex["answer"])],
            })

    n_test = len(ds_test) if max_test is None else min(max_test, len(ds_test))
    records: List[Dict[str, Any]] = []
    for i in range(n_test):
        item = ds_test[i]
        question = item["question"]
        choices = list(item["choices"])
        gold = ANSWER_LETTERS[int(item["answer"])]
        world_name = f"bench-mmlu-{run_stamp}-{subject}-{i:04d}"
        try:
            rec = _run_question(scenario_path, world_name, subject, i,
                                question, choices, gold, few_shots)
        except Exception as e:
            tb = traceback.format_exc()
            logger.error(
                f"[{subject}#{i}] question crashed outside session: {e}\n{tb}")
            rec = {
                "subject": subject,
                "question_index": i,
                "question": question,
                "choices": choices,
                "gold": gold,
                "prediction": None,
                "correct": False,
                "reply": "",
                "error": f"{type(e).__name__}: {e}",
                "world_name": world_name,
                "captured_at": datetime.datetime.now(
                    datetime.timezone.utc).isoformat(),
            }
        records.append(rec)
        mark = "✓" if rec["correct"] else ("✗" if rec["prediction"] else "?")
        print(
            f"[{subject}] {i:4d} gold={gold} pred={rec.get('prediction') or '?'} "
            f"{mark}  ({rec.get('duration_s', 0):.2f}s)",
            flush=True)

    return records


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------

def _summarize(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    by_subject: Dict[str, Dict[str, int]] = defaultdict(
        lambda: {"correct": 0, "total": 0, "errors": 0})
    overall_correct = 0
    overall_total = 0
    overall_errors = 0
    for r in records:
        s = r["subject"]
        by_subject[s]["total"] += 1
        overall_total += 1
        if r.get("error"):
            by_subject[s]["errors"] += 1
            overall_errors += 1
        if r.get("correct"):
            by_subject[s]["correct"] += 1
            overall_correct += 1
    per_subject = {}
    for s, st in sorted(by_subject.items()):
        acc = st["correct"] / st["total"] if st["total"] > 0 else 0.0
        per_subject[s] = {
            "accuracy": round(acc, 4),
            "correct": st["correct"],
            "total": st["total"],
            "errors": st["errors"],
        }
    overall_acc = overall_correct / overall_total if overall_total > 0 else 0.0
    return {
        "overall_accuracy": round(overall_acc, 4),
        "overall_correct": overall_correct,
        "overall_total": overall_total,
        "overall_errors": overall_errors,
        "per_subject": per_subject,
    }


def _write_summary_md(out_dir: Path, scenario_path: Path, run_stamp: str,
                      k_shot: int, max_test: Optional[int],
                      summary: Dict[str, Any]) -> Path:
    lines: List[str] = []
    lines.append("# MMLU benchmark — summary")
    lines.append("")
    lines.append(f"- run_stamp: `{run_stamp}`")
    lines.append(f"- scenario: `{scenario_path.name}`")
    lines.append(f"- k_shot: {k_shot}")
    lines.append(f"- max_test_per_subject: {max_test if max_test is not None else 'all'}")
    lines.append("")
    lines.append(
        f"**Overall:** {summary['overall_accuracy']:.4f} "
        f"({summary['overall_correct']}/{summary['overall_total']}, "
        f"errors={summary['overall_errors']})")
    lines.append("")
    lines.append("## Per-subject")
    lines.append("")
    lines.append("| subject | accuracy | correct/total | errors |")
    lines.append("|---|---:|---:|---:|")
    for s, st in summary["per_subject"].items():
        lines.append(
            f"| {s} | {st['accuracy']:.4f} | {st['correct']}/{st['total']} | "
            f"{st['errors']} |")
    out = out_dir / "summary.md"
    out.write_text("\n".join(lines))
    return out


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def run_benchmark(scenario_path: Path, subjects: List[str], output_dir: Path,
                  k_shot: int, max_test: Optional[int]) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "raw.jsonl"

    run_stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H-%M-%SZ")

    all_records: List[Dict[str, Any]] = []
    with open(raw_path, "w") as raw_f:
        for subject in subjects:
            try:
                records = _run_subject(scenario_path, subject, run_stamp,
                                       k_shot, max_test)
            except Exception as e:
                tb = traceback.format_exc()
                logger.error(f"[{subject}] subject crashed: {e}\n{tb}")
                err_rec = {
                    "subject": subject,
                    "subject_error": f"{type(e).__name__}: {e}",
                    "subject_traceback": tb,
                    "captured_at": datetime.datetime.now(
                        datetime.timezone.utc).isoformat(),
                }
                raw_f.write(json.dumps(err_rec) + "\n")
                raw_f.flush()
                continue
            for rec in records:
                raw_f.write(json.dumps(rec, default=str) + "\n")
                raw_f.flush()
                all_records.append(rec)

    summary = _summarize(all_records)
    summary_path = _write_summary_md(output_dir, scenario_path, run_stamp,
                                     k_shot, max_test, summary)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, default=str))

    print(f"\nWrote raw records: {raw_path}")
    print(f"Summary:           {summary_path}")
    print(f"Overall accuracy:  {summary['overall_accuracy']:.4f} "
          f"({summary['overall_correct']}/{summary['overall_total']})")
    return raw_path


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="MMLU benchmark runner. Per question: fresh ChatLoop "
                    "with timestamped world_name, single user turn, "
                    "extract A/B/C/D, score against gold letter.")
    parser.add_argument(
        "--scenario", type=Path, required=True,
        help="Path to a chat-mode scenario YAML "
             "(e.g. scenarios/jill-benchmark-mmlu.yaml).")
    parser.add_argument(
        "--subjects", nargs="*", default=None,
        help="Subjects to evaluate. Pass 'all' for the full 57-subject set, "
             "a list of names, or omit for a small default subset.")
    parser.add_argument(
        "--k-shot", type=int, default=5,
        help="Number of dev examples per subject for few-shot "
             "(canonical = 5; 0 for zero-shot).")
    parser.add_argument(
        "--max-test-per-subject", type=int, default=50,
        help="Max test questions per subject (default 50). Omit-equivalent: "
             "pass a large number to run the full split.")
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Where to write raw.jsonl + summary.{md,json}. Defaults to "
             "bench/mmlu/results/<timestamp>_<scenario_stem>.")
    args = parser.parse_args()

    scenario = args.scenario.resolve()
    if args.subjects is None or len(args.subjects) == 0:
        subjects = list(DEFAULT_SUBJECTS)
    elif len(args.subjects) == 1 and args.subjects[0].strip().lower() == "all":
        subjects = list(ALL_SUBJECTS)
    else:
        subjects = list(args.subjects)

    if args.output_dir:
        out = args.output_dir.resolve()
    else:
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H-%M-%SZ")
        out = HERE / "results" / f"{stamp}_{scenario.stem}"

    run_benchmark(scenario_path=scenario, subjects=subjects, output_dir=out,
                  k_shot=args.k_shot, max_test=args.max_test_per_subject)


if __name__ == "__main__":
    main()
