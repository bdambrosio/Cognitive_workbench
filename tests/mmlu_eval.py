#!/usr/bin/env python3
"""
Minimal MMLU harness using SGLang directly (no LightEval).

- Loads MMLU from HuggingFace: cais/mmlu
- Canonical 5-shot (or 0-shot) per subject
- Sends a single prompt per question to an SGLang Runtime
- Expects model to end with a line: "ANSWER: X" where X in {A,B,C,D}
- Reports per-subject and overall accuracy
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
import sglang as sgl
from sglang import function, system, user, gen

ANSWER_LETTERS = ["A", "B", "C", "D"]


# -----------------------------
# SGLang function wrapper
# -----------------------------

@function
def mmlu_query(s, prompt: str):
    s += system("You are a helpful student taking an exam and trying for your best score.")
    s += user(prompt + "\nThink step by step before answering.")
    
    # 1. Allow the model to "think" (unconstrained generation)
    # We use a distinct stop token or length limit for the thought process.
    s += gen("reasoning", stop=["ANSWER:"], max_tokens=1024)
    
    # 2. Force the standard format for the final token
    s += "ANSWER:" 
    s += gen("answer", regex=r" [ABCD]")


def mmlu_query_via_executive(
    session: zenoh.Session,
    character: str,
    prompt: str,
    timeout: float = 240.0
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
    response = {'received': False, 'final_thoughts': '', 'final_content': ''}
    response_lock = threading.Lock()
    
    def plan_result_callback(sample):
        payload = sample.payload.to_bytes().decode('utf-8')
        result = json.loads(payload)
        with response_lock:
            if not response['received']:
                response['received'] = True
                response['final_thoughts'] = result.get('final_thoughts', '')
                response['final_content'] = result.get('final_content', '')
                #print(f"Final thoughts: {response['final_thoughts']}")
                #print(f"Final content: {response['final_content']}")
    
    # Subscribe to plan_result before sending
    subscriber = session.declare_subscriber(
        f"cognitive/{character}/plan_result",
        plan_result_callback
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
    
    # Try to extract answer from both fields - final_thoughts often has "choice B" format
    # Prioritize final_thoughts as it usually contains the answer determination
    combined = response['final_thoughts'] + '\n' + response['final_content']
    return {'reasoning': response['final_thoughts'], 'answer': combined}

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
    lines.append(f"  subject: {subject}")
    lines.append(f"  question: {question.strip()}") 
    lines.append(f"  do not use display, query-web, or semantic-scholar in your reasoning. Only use generate to find information.")
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
    if use_executive:
        lines.append("Finally,say the answer to User")


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
    model_path: str,
    subject: str,
    k_shot: int,
    max_test: int | None = None,
    temperature: float = 0.0,
    top_p: float = 1.0,
    use_executive: bool = False,
    zenoh_session: zenoh.Session = None,
    character: str = "Jill",
) -> tuple[float, int, int]:
    """
    Evaluate a single MMLU subject.

    Args:
        use_executive: If True, send queries via executive_node instead of direct SGLang
        zenoh_session: Required if use_executive=True
        character: Character name for executive mode (default: Jill)

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
            out = mmlu_query_via_executive(zenoh_session, character, prompt)
            if 'error' in out and out['error']:
                print(f"[{subject}] idx={idx:4d} ERROR: {out['error']}")
                continue
        else:
            out = mmlu_query.run(
                prompt=prompt,
                temperature=temperature,
                top_p=top_p,
            )

        # Access outputs to force completion before measuring time
        answer_text = out["answer"]
        _ = out["reasoning"]  # Force reasoning generation to complete
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

# python3 ../tests/mmlu_eval.py --model-path RedHatAI/Qwen2.5-14B-Instruct-FP8-dynamic --subjects "high_school_physics" --k-shot 1 --max-test-per-subject 10

def main():
    parser = argparse.ArgumentParser(description="MMLU eval via SGLang")
    parser.add_argument(
        "--model-path",
        type=str,
        default="",
        help="Path to SGLang model (required unless --use-executive).",
    )
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
        "--device", type=str, default="cuda", help="Device for SGLang Runtime."
    )
    parser.add_argument(
        "--dtype",
        type=str,
        default="auto",
        help='Dtype for SGLang Runtime ("auto", "float16", "bf16", ...).',
    )
    parser.add_argument(
        "--context-length",
        type=int,
        default=32768,
        help="Context length for SGLang Runtime.",
    )
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.0,
        help="Temperature for generation (0.0 for deterministic).",
    )
    parser.add_argument(
        "--top-p",
        type=float,
        default=1.0,
        help="Top-p for sampling.",
    )
    parser.add_argument(
        "--attention-backend",
        type=str,
        default=None,
        help="Attention backend for SGLang Runtime (e.g., 'flashinfer'). Required for some GPU architectures.",
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

    zenoh_session = None
    
    if args.use_executive:
        # Executive mode: connect to running session via Zenoh
        print(f"Connecting to executive_node for character: {args.character}")
        config = zenoh.Config()
        zenoh_session = zenoh.open(config)
        print("Zenoh session opened")
    else:
        # Direct SGLang mode
        if not args.model_path:
            parser.error("--model-path required unless --use-executive")
        print("Initializing SGLang Runtime...")
        runtime_kwargs = {
            "model_path": args.model_path,
            "tokenizer_path": args.model_path,
            "device": args.device,
            "context_length": args.context_length,
            "dtype": args.dtype,
            "mem_fraction_static": 0.82
        }
        if args.attention_backend:
            runtime_kwargs["attention_backend"] = args.attention_backend
        sgl.set_default_backend(sgl.Runtime(**runtime_kwargs))

    # Decide subjects
    if args.subjects.strip():
        subjects = [s.strip() for s in args.subjects.split(",") if s.strip()]
    else:
        subjects = [
            "high_school_physics",
            "high_school_world_history",
            "college_mathematics",
            "formal_logic",
            "college_medicine",
            "moral_disputes",
            "us_foreign_policy"
        ]

    overall_correct = 0
    overall_total = 0
    per_subject = {}

    for subject in subjects:
        acc, n_corr, n_tot = evaluate_subject(
            model_path=args.model_path,
            subject=subject,
            k_shot=args.k_shot,
            max_test=args.max_test_per_subject,
            temperature=args.temperature,
            top_p=args.top_p,
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
