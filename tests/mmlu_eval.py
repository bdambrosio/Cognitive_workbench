#!/usr/bin/env python3
"""
MMLU evaluation harness using Zenoh LLM API.

- Loads MMLU from HuggingFace: cais/mmlu
- Canonical 5-shot (or 0-shot) per subject
- Uses Zenoh LLM API (requires running Jill session)
- Expects model to end with a line: "ANSWER: X" where X in {A,B,C,D}
- Reports per-subject and overall accuracy

Modes:
- Direct mode (default): Uses Zenoh LLM API for raw generation
- Executive mode (--use-executive): Sends full goals through planner/executor
"""

import argparse
import json
import re
import threading
import time
from collections import defaultdict
from datetime import datetime

from datasets import load_dataset
import zenoh

ANSWER_LETTERS = ["A", "B", "C", "D"]


# -----------------------------
# Zenoh LLM API wrapper
# -----------------------------

def llm_generate(session: zenoh.Session, character: str, messages: list, max_tokens: int = 1500, temperature: float = 0.0, timeout: float = 60.0) -> tuple:
    """Call LLM via Zenoh API. Returns (text, error) tuple."""
    query_key = f"cognitive/{character}/llm/generate"
    payload = json.dumps({'messages': messages, 'max_tokens': max_tokens, 'temperature': temperature})
    replies = session.get(query_key, payload=payload.encode('utf-8'), timeout=timeout)
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            result = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if result.get('success'):
                return result.get('text', ''), None
            else:
                return '', result.get('error', 'Unknown error')
    return '', 'No response from LLM API'


def mmlu_query_direct(session: zenoh.Session, character: str, prompt: str, temperature: float = 0.0) -> dict:
    """Query LLM directly via Zenoh API for MMLU question."""
    full_prompt = f"You are a helpful student taking an exam and trying for your best score.\n\n{prompt}\n\nThink step by step before answering. End with ANSWER: X where X is A, B, C, or D."
    
    text, error = llm_generate(session, character, [full_prompt], max_tokens=1500, temperature=temperature)
    
    if error:
        return {'error': error, 'reasoning': '', 'answer': ''}
    
    # Split into reasoning and answer
    if 'ANSWER:' in text.upper():
        parts = text.upper().split('ANSWER:')
        reasoning = text[:text.upper().rfind('ANSWER:')]
        answer_part = parts[-1].strip()
    else:
        reasoning = text
        answer_part = text
    
    return {'reasoning': reasoning, 'answer': answer_part}


def clear_transient_notes(session: zenoh.Session, character: str, timeout: float = 10.0, global_clear: bool = True) -> int:
    """Clear all non-persistent Notes and Collections via Zenoh API.
    
    Args:
        global_clear: If True, also resets world_model and tool_model to empty (default: True for benchmarks)
    """
    # 1. Clear Notes/Collections
    query_key = f"cognitive/{character}/resource/clear_transient"
    payload = json.dumps({"global": False}).encode('utf-8') # Just clear notes first
    replies = session.get(query_key, payload=payload, timeout=timeout)
    deleted_count = 0
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            payload = reply.ok.payload.to_bytes().decode('utf-8')
            result = json.loads(payload)
            if result.get('success'):
                deleted_count += result.get('deleted_notes', 0) + result.get('deleted_collections', 0)
    
    # 2. Explicitly reset models if requested (more robust than payload on clear_transient)
    if global_clear:
        reset_key = f"cognitive/{character}/resource/reset_models"
        replies = session.get(reset_key, timeout=timeout)
        # We don't strictly need to check success here, best effort for benchmarks
        
    return deleted_count


def send_planner_feedback(session: zenoh.Session, character: str, outcome: bool, timeout: float = 5.0) -> bool:
    """Send feedback to planner about plan execution outcome."""
    query_key = f"cognitive/{character}/planner/feedback"
    payload = json.dumps({'outcome': outcome})
    replies = session.get(query_key, payload=payload.encode('utf-8'), timeout=timeout)
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            result = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            return result.get('success', False)
    return False


def mmlu_query_via_executive(
    session: zenoh.Session,
    character: str,
    prompt: str,
    timeout: float = 64.0
) -> dict:
    """
    Send MMLU question to executive_node via Zenoh and wait for response.
    
    Args:
        session: Open Zenoh session
        character: Character name (e.g., 'Jill')
        prompt: The MMLU prompt text
        timeout: Max seconds to wait for response
        
    Returns:
        {'reasoning': str, 'answer': str} or {'error': str}
    """
    response = {'received': False, 'answer': ''}
    response_lock = threading.Lock()
    
    def goal_result_callback(sample):
        payload = sample.payload.to_bytes().decode('utf-8')
        result = json.loads(payload)
        with response_lock:
            if not response['received']:
                response['received'] = True
                # goal_result contains 'response' field (final answer from planner)
                response['answer'] = result.get('response', '')
    
    # Subscribe to goal_result before sending
    subscriber = session.declare_subscriber(
        f"cognitive/{character}/goal_result",
        goal_result_callback
    )
    
    # Publish to sense_data with goal: prefix (same as FastAPI submit)
    sense_data = {
        'timestamp': datetime.now().isoformat(),
        'sequence_id': 0,
        'mode': 'text',
        'content': json.dumps({
            'text': prompt,  # prompt already starts with "goal:"
            'source': 'User'
        })
    }
    session.put(f"cognitive/{character}/sense_data", json.dumps(sense_data))
    
    # Wait for response
    start = time.time()
    while time.time() - start < timeout:
        with response_lock:
            if response['received']:
                break
        time.sleep(0.1)
    
    subscriber.undeclare()
    
    if not response['received']:
        return {'error': 'timeout', 'reasoning': '', 'answer': ''}
    
    # Try to use last_action_result if available and action is 'say' (most reliable)
    answer = response.get('answer')
    if answer and isinstance(answer, str):
        return {'answer': answer}
    
    # Fallback: Try to extract answer from both fields - final_thoughts often has "choice B" format
    # Prioritize final_thoughts as it usually contains the answer determination
    return {'answer': answer}

def build_prompt(
    subject: str,
    question: str,
    choices: list[str],
    few_shot_examples: list[dict],
    use_executive: bool = False,
) -> str:
    """
    Build the planner-style prompt:

    - goal: block with subject, question, choices
    - optional few-shot "domain examples"
    - response instructions with final 'ANSWER: X'
    """

    # Goal block
    lines = []
    lines.append(f"goal:")
    lines.append(f"  question: {question.strip()}") 
    #lines.append(f"  do not use display, search-web, or semantic-scholar in your reasoning. Only use generate to find information.")
    lines.append(f"  do not use search-web or semantic-scholar in your reasoning. use generate or think to surface knowledge, or extract to extract information from text.")
    lines.append(f"  choices:")
    lines.append(f"    A: {choices[0].strip()}")
    lines.append(f"    B: {choices[1].strip()}")
    lines.append(f"    C: {choices[2].strip()}")
    lines.append(f"    D: {choices[3].strip()}")
    lines.append("")

    # Few-shot examples (if any)
    if few_shot_examples:
        lines.append("in reasoning, consider these domain examples:")
        lines.append("")
        for i, ex in enumerate(few_shot_examples, start=1):
            q = ex["question"].strip()
            c = ex["choices"]
            a = ex["answer"].strip().upper()  # already converted to letter
            lines.append(f"Example {i}:")
            lines.append(f"Q: {q}")
            lines.append("Choices:")
            lines.append(f"A: {c[0].strip()}")
            lines.append(f"B: {c[1].strip()}")
            lines.append(f"C: {c[2].strip()}")
            lines.append(f"D: {c[3].strip()}")
            lines.append(f"ANSWER: {a}")
            lines.append("")
        lines.append("")

    # Final instructions
    #lines.append("the correct answer to the original question is:")
    lines.append("")
    lines.append("RESPONSE INSTRUCTIONS:")
    lines.append("output your answer in this format:")
    lines.append("ANSWER: X")
    lines.append("where X is A, B, C, or D.")

    return "\n".join(lines)


ANSWER_RE = re.compile(r"ANSWER\s*:\s*([ABCD])", re.IGNORECASE)
# Match "choice B", "option B", "matches B", "answer is B", etc.
CHOICE_RE = re.compile(r"(?:choice|option|matches|answer\s+is)\s+([ABCD])\b", re.IGNORECASE)


def extract_choice(text: str) -> str | None:
    """
    Extract the final "ANSWER: X" from model output.
    Returns 'A'/'B'/'C'/'D' or None if not found.
    """
    # Try "ANSWER: X" format first
    matches = ANSWER_RE.findall(text)
    if matches:
        return matches[-1].upper()
    
    # Try "choice B", "matches B", "answer is B" patterns
    choice_matches = CHOICE_RE.findall(text)
    if choice_matches:
        return choice_matches[-1].upper()
    
    # Fallback: first char if it's a letter
    text = text.strip()
    if text and text[0] in "ABCD":
        return text[0]
    
    # Last resort: last standalone A/B/C/D in the text
    #print(f'Bad format: {text[:100]}')
    letters = re.findall(r"\b([ABCD])\b", text)
    if not letters:
        return None
    return letters[-1].upper()


def evaluate_subject(
    subject: str,
    k_shot: int,
    max_test: int | None = None,
    temperature: float = 0.0,
    use_executive: bool = False,
    zenoh_session: zenoh.Session = None,
    character: str = "Jill",
) -> tuple[float, int, int]:
    """
    Evaluate a single MMLU subject.

    Args:
        use_executive: If True, send queries via executive_node (planner/executor)
        zenoh_session: Zenoh session for LLM API
        character: Character name (default: Jill)

    Returns:
        (accuracy, num_correct, num_total)
    """
    print(f"\n=== Subject: {subject} (k={k_shot}) ===")

    # Load dev/test splits for this subject from cais/mmlu
    ds_dev = load_dataset("cais/mmlu", subject, split="dev")
    ds_test = load_dataset("cais/mmlu", subject, split="test")

    # Build few-shot examples: convert int answer index -> letter
    if k_shot > 0:
        few_shots = []
        for ex in ds_dev.select(range(min(k_shot, len(ds_dev)))):
            ans_idx = int(ex["answer"])
            ans_letter = ANSWER_LETTERS[ans_idx]
            few_shots.append(
                {
                    "question": ex["question"],
                    "choices": ex["choices"],
                    "answer": ans_letter,
                }
            )
    else:
        few_shots = []

    num_correct = 0
    num_total = 0

    indices = range(len(ds_test))
    if max_test is not None:
        indices = list(indices)[:max_test]

    for idx in indices:
        item = ds_test[idx]
        question = item["question"]
        choices = item["choices"]

        ans_idx = int(item["answer"])
        gold = ANSWER_LETTERS[ans_idx]  # "A"/"B"/"C"/"D"

        prompt = build_prompt(subject, question, choices, few_shots, use_executive)

        t0 = time.time()
        
        if use_executive:
            clear_transient_notes(zenoh_session, character)
            out = mmlu_query_via_executive(zenoh_session, character, prompt)
        else:
            out = mmlu_query_direct(zenoh_session, character, prompt, temperature)
        
        if 'error' in out and out['error']:
            print(f"[{subject}] idx={idx:4d} ERROR: {out['error']}")
            continue

        # Access outputs to force completion before measuring time
        answer_text = out["answer"]
        dt = time.time() - t0
        pred = extract_choice(answer_text)

        correct = (pred == gold)
        num_total += 1
        if correct:
            num_correct += 1

        print(
            f"[{subject}] idx={idx:4d} gold={gold} pred={pred or '?'} "
            f"{'✓' if correct else '✗'}  ({dt:.2f}s)"
        )

    acc = num_correct / num_total if num_total > 0 else 0.0
    print(
        f"Subject {subject}: accuracy = {acc:.3f} "
        f"({num_correct}/{num_total})"
    )
    return acc, num_correct, num_total

# python3 ../tests/mmlu_eval.py --subjects "high_school_physics" --k-shot 1 --max-test-per-subject 10

def main():
    parser = argparse.ArgumentParser(description="MMLU eval via Zenoh LLM API")
    parser.add_argument(
        "--subjects",
        type=str,
        default="",
        help=(
            "Comma-separated MMLU subjects "
            "(e.g. 'high_school_physics,world_history'). "
            "If empty, uses a small default subset."
        ),
    )
    parser.add_argument(
        "--k-shot",
        type=int,
        default=5,
        help="Number of dev examples per subject for few-shot (canonical = 5).",
    )
    parser.add_argument(
        "--max-test-per-subject",
        type=int,
        default=50,
        help="Max number of test questions per subject (for quick runs).",
    )
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.0,
        help="Temperature for generation (0.0 for deterministic).",
    )
    parser.add_argument(
        "--use-executive",
        action="store_true",
        help="Send queries via executive_node (requires running Jill session).",
    )
    parser.add_argument(
        "--character",
        type=str,
        default="Jill",
        help="Character name for executive mode (default: Jill).",
    )

    args = parser.parse_args()

    # Both modes require running Jill session for Zenoh LLM API
    print(f"Connecting to executive_node for character: {args.character}")
    config = zenoh.Config()
    zenoh_session = zenoh.open(config)
    print("Zenoh session opened")
    
    if args.use_executive:
        print("Mode: Executive (full planner/executor pipeline)")
    else:
        print("Mode: Direct LLM (Zenoh API, bypasses planner)")

    # Decide subjects
    
    if args.subjects.strip() == "all":
        subjects = ['abstract_algebra', 'anatomy', 'astronomy', 'business_ethics', 'clinical_knowledge', 'college_biology', 
        'college_chemistry', 'college_computer_science', 'college_mathematics', 'college_medicine', 'college_physics', 'computer_security', 
        'conceptual_physics', 'econometrics', 'electrical_engineering', 'elementary_mathematics', 'formal_logic', 'global_facts', 'high_school_biology', 
        'high_school_chemistry', 'high_school_computer_science', 'high_school_european_history', 'high_school_geography', 'high_school_government_and_politics', 
        'high_school_macroeconomics', 'high_school_mathematics', 'high_school_microeconomics', 'high_school_physics', 'high_school_psychology', 'high_school_statistics', 
        'high_school_us_history', 'high_school_world_history', 'human_aging', 'human_sexuality', 'international_law', 'jurisprudence', 'logical_fallacies', 
        'machine_learning', 'management', 'marketing', 'medical_genetics', 'miscellaneous', 'moral_disputes', 'moral_scenarios', 'nutrition', 'philosophy', 
        'prehistory', 'professional_accounting', 'professional_law', 'professional_medicine', 'professional_psychology', 'public_relations', 'security_studies', 
        'sociology', 'us_foreign_policy', 'virology', 'world_religions']
    elif args.subjects.strip():
        subjects = [s.strip() for s in args.subjects.split(",") if s.strip()]
    else:
        subjects = [
            "high_school_physics",
            "high_school_world_history",
            "college_mathematics",
            "formal_logic",
            "college_medicine",
            "us_foreign_policy"
        ]

    overall_correct = 0
    overall_total = 0
    per_subject = {}

    for subject in subjects:
        acc, n_corr, n_tot = evaluate_subject(
            subject=subject,
            k_shot=args.k_shot,
            max_test=args.max_test_per_subject,
            temperature=args.temperature,
            use_executive=args.use_executive,
            zenoh_session=zenoh_session,
            character=args.character,
        )
        per_subject[subject] = acc
        overall_correct += n_corr
        overall_total += n_tot

    print("\n=== Summary ===")
    for subject, acc in per_subject.items():
        print(f"{subject:25s}: {acc:.3f}")
    overall_acc = overall_correct / overall_total if overall_total > 0 else 0.0
    print(f"\nOverall accuracy: {overall_acc:.3f} ({overall_correct}/{overall_total})")
    
    if zenoh_session:
        zenoh_session.close()


if __name__ == "__main__":
    main()
