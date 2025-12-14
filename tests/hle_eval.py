#!/usr/bin/env python3
"""
Humanity's Last Exam (HLE) evaluation harness for Cognitive Workbench.

- Loads HLE from HuggingFace (dataset name TBD)
- Sends goals via Zenoh to executive_node
- Evaluates with exact match, contains match, F1, and LLM judge
- Clears transient Notes between questions
"""

import argparse
import json
import re
import string
import threading
import time
from collections import Counter
from datetime import datetime

from datasets import load_dataset
import zenoh


def normalize_answer(s: str) -> str:
    """Normalize answer for comparison: lowercase, remove punctuation/whitespace."""
    def remove_punc(text):
        exclude = set(string.punctuation)
        return ''.join(ch for ch in text if ch not in exclude)
    
    return remove_punc(s.lower().strip())


def f1_score(prediction: str, ground_truth: str) -> float:
    """Compute token-level F1 score."""
    pred_tokens = normalize_answer(prediction).split()
    gold_tokens = normalize_answer(ground_truth).split()
    
    if not pred_tokens or not gold_tokens:
        return float(pred_tokens == gold_tokens)
    
    common = Counter(pred_tokens) & Counter(gold_tokens)
    num_same = sum(common.values())
    
    if num_same == 0:
        return 0.0
    
    precision = num_same / len(pred_tokens)
    recall = num_same / len(gold_tokens)
    return 2 * precision * recall / (precision + recall)


def exact_match(prediction: str, ground_truth: str) -> bool:
    """Check if normalized prediction matches ground truth."""
    return normalize_answer(prediction) == normalize_answer(ground_truth)


def contains_match(prediction: str, ground_truth: str) -> bool:
    """Check if normalized ground truth appears in normalized prediction."""
    return normalize_answer(ground_truth) in normalize_answer(prediction)


def llm_generate(session: zenoh.Session, character: str, messages: list, max_tokens: int = 500, temperature: float = 0.0, timeout: float = 30.0) -> tuple:
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


def llm_judge(session: zenoh.Session, character: str, question: str, gold_answer: str, prediction: str, verbose: bool = False) -> tuple:
    """Use LLM to judge if prediction correctly answers the question. Returns (is_correct, raw_response, error)."""
    prompt = f"""You are evaluating an answer to a question.

Question: {question}
Correct answer: {gold_answer}
Predicted answer: {prediction}

Does the predicted answer correctly answer the question? The prediction may be worded differently or include extra context, but should contain the correct information.

Reply with only YES or NO."""
    
    result, error = llm_generate(session, character, [prompt], max_tokens=10, temperature=0.0)
    
    if error:
        print(f"       LLM Judge ERROR: {error}")
        return False, '', error
    
    is_correct = result.strip().upper().startswith('YES')
    
    if verbose or not is_correct:
        print(f"       LLM Judge: '{result.strip()}' -> {is_correct}")
    
    return is_correct, result.strip(), None


def clear_transient_notes(session: zenoh.Session, character: str, timeout: float = 10.0) -> int:
    """Clear all Notes and Collections except Note_null via Zenoh API."""
    query_key = f"cognitive/{character}/resource/clear_transient"
    replies = session.get(query_key, timeout=timeout)
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            payload = reply.ok.payload.to_bytes().decode('utf-8')
            result = json.loads(payload)
            if result.get('success'):
                return result.get('deleted_notes', 0) + result.get('deleted_collections', 0)
    return 0


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


def send_goal_and_wait(
    session: zenoh.Session,
    character: str,
    goal_text: str,
    timeout: float = 600.0
) -> dict:
    """Send goal to executive_node and wait for plan_result."""
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
    
    subscriber = session.declare_subscriber(
        f"cognitive/{character}/plan_result",
        plan_result_callback
    )
    
    sense_data = {
        'timestamp': datetime.now().isoformat(),
        'sequence_id': 0,
        'mode': 'text',
        'content': json.dumps({
            'text': goal_text,
            'source': 'User'
        })
    }
    session.put(f"cognitive/{character}/sense_data", json.dumps(sense_data))
    
    start = time.time()
    while time.time() - start < timeout:
        with response_lock:
            if response['received']:
                break
        time.sleep(0.1)
    
    subscriber.undeclare()
    
    if not response['received']:
        return {'error': 'timeout', 'answer': ''}
    
    combined = response['final_thoughts'] + '\n' + response['final_content']
    return {'reasoning': response['final_thoughts'], 'answer': combined}


def extract_answer(text: str) -> str:
    """Extract the final answer from model output."""
    # Look for explicit "ANSWER:" format
    match = re.search(r'ANSWER:\s*(.+?)(?:\n|$)', text, re.IGNORECASE)
    if match:
        return match.group(1).strip()
    
    # Look for "The answer is..." format
    match = re.search(r'(?:the answer is|answer:)\s*(.+?)(?:\.|$)', text, re.IGNORECASE)
    if match:
        return match.group(1).strip()
    
    # Fallback: last non-empty line
    lines = [l.strip() for l in text.strip().split('\n') if l.strip()]
    if lines:
        return lines[-1]
    return text.strip()


def build_hle_prompt(question: str) -> str:
    """Build goal prompt for HLE question."""
    lines = ["goal:"]
    lines.append(f"  question: {question}")
    lines.append("")
    lines.append("  instructions:")
    lines.append("    - Use appropriate tools (query-web, fetch-text, semantic-scholar, etc.) to find information")
    lines.append("    - Answer the question accurately")
    lines.append("    - Provide a concise answer")
    lines.append("    - Format: ANSWER: <your answer>")
    lines.append("    - Finally, say the answer to User")
    
    return '\n'.join(lines)


def evaluate_hle(
    session: zenoh.Session,
    character: str,
    dataset_name: str = "hle-benchmark/HLE",  # TBD: actual dataset name
    split: str = "test",
    max_questions: int = 50,
    timeout: float = 600.0,
    start_question: int = 0,
    single_question: int = None,
) -> dict:
    """Run HLE evaluation."""
    print(f"\n=== HLE Evaluation (max={max_questions}) ===")
    
    # Load dataset
    try:
        ds = load_dataset(dataset_name, split=split)
        print(f"Loaded HLE dataset: {split} ({len(ds)} questions)")
    except Exception as e:
        print(f"ERROR: Failed to load HLE dataset: {e}")
        print("Note: HLE dataset may require HuggingFace authentication/agreement")
        return {'error': str(e)}
    
    results = {
        'exact_match': 0,
        'contains_match': 0,
        'llm_judge': 0,
        'f1_sum': 0.0,
        'total': 0,
        'errors': 0,
    }
    
    # Filter out questions with missing data
    valid_items = []
    for idx, item in enumerate(ds):
        # TBD: Adjust field names based on actual dataset structure
        question = item.get('question', item.get('Question', item.get('prompt', '')))
        gold_answer = item.get('answer', item.get('Answer', item.get('gold_answer', '')))
        if question and gold_answer:
            valid_items.append((idx, item))
    
    print(f"Found {len(valid_items)} valid questions (skipped {len(ds) - len(valid_items)} with missing data)")
    
    # Apply start_question filter
    if start_question > 0:
        if start_question >= len(valid_items):
            print(f"ERROR: start_question ({start_question}) >= number of valid questions ({len(valid_items)})")
            return {'error': f'start_question out of range'}
        valid_items = valid_items[start_question:]
        print(f"Starting from question {start_question} ({len(valid_items)} questions remaining)")
    
    # Apply single_question filter (overrides max_questions)
    if single_question is not None:
        if single_question >= len(valid_items):
            print(f"ERROR: single_question ({single_question}) >= number of valid questions ({len(valid_items)})")
            return {'error': f'single_question out of range'}
        valid_items = [valid_items[single_question]]
        print(f"Evaluating only question {single_question}")
    else:
        # Limit to max_questions
        valid_items = valid_items[:max_questions]
    
    for orig_idx, item in valid_items:
        # Extract fields (TBD: adjust based on actual dataset structure)
        question = item.get('question', item.get('Question', item.get('prompt', '')))
        gold_answer = item.get('answer', item.get('Answer', item.get('gold_answer', '')))
        
        # Clear transient notes FIRST
        cleared = clear_transient_notes(session, character)
        if cleared > 0:
            print(f"  [Cleared {cleared} notes]")
        
        # Build prompt
        goal_text = build_hle_prompt(question)
        
        # Send goal and wait
        t0 = time.time()
        response = send_goal_and_wait(session, character, goal_text, timeout)
        dt = time.time() - t0
        
        if 'error' in response and response['error']:
            print(f"[{orig_idx:4d}] ERROR: {response['error']}")
            results['errors'] += 1
            continue
        
        pred_answer = extract_answer(response['answer'])
        em = exact_match(pred_answer, gold_answer)
        cm = contains_match(pred_answer, gold_answer)
        f1 = f1_score(pred_answer, gold_answer)
        lj, lj_response, lj_error = llm_judge(session, character, question, gold_answer, pred_answer)
        
        # Outcome is True if ANY scoring method indicates correctness
        outcome = em or cm or lj
        
        results['total'] += 1
        if em:
            results['exact_match'] += 1
        if cm:
            results['contains_match'] += 1
        if lj:
            results['llm_judge'] += 1
        results['f1_sum'] += f1
        
        # Send feedback to planner
        feedback_sent = send_planner_feedback(session, character, outcome)
        if not feedback_sent:
            print(f"       WARNING: Failed to send planner feedback")
        
        status = '✓' if em else ('+' if cm or lj else ('~' if f1 > 0.5 else '✗'))
        print(f"[{orig_idx:4d}] {status} EM={int(em)} CM={int(cm)} LJ={int(lj)} F1={f1:.2f} ({dt:.1f}s)")
        print(f"       Q: {question[:80]}...")
        print(f"       Gold: {gold_answer}")
        print(f"       Pred: {pred_answer}")
    
    # Summary
    if results['total'] > 0:
        em_acc = results['exact_match'] / results['total']
        cm_acc = results['contains_match'] / results['total']
        lj_acc = results['llm_judge'] / results['total']
        avg_f1 = results['f1_sum'] / results['total']
    else:
        em_acc = cm_acc = lj_acc = avg_f1 = 0.0
    
    print(f"\n=== Summary ===")
    print(f"Exact Match:    {em_acc:.3f} ({results['exact_match']}/{results['total']})")
    print(f"Contains Match: {cm_acc:.3f} ({results['contains_match']}/{results['total']})")
    print(f"LLM Judge:      {lj_acc:.3f} ({results['llm_judge']}/{results['total']})")
    print(f"Average F1:     {avg_f1:.3f}")
    print(f"Errors:         {results['errors']}")
    
    return {
        'exact_match': em_acc,
        'contains_match': cm_acc,
        'llm_judge': lj_acc,
        'f1': avg_f1,
        'total': results['total'],
        'errors': results['errors'],
    }


def main():
    parser = argparse.ArgumentParser(description="HLE evaluation for Cognitive Workbench")
    parser.add_argument(
        "--character",
        type=str,
        default="Jill",
        help="Character name for executive_node (default: Jill)",
    )
    parser.add_argument(
        "--dataset",
        type=str,
        default="cais/hle",
        help="HuggingFace dataset name (default: cais/hle)",
    )
    parser.add_argument(
        "--split",
        type=str,
        default="test",
        help="Dataset split: test or validation (default: test)",
    )
    parser.add_argument(
        "--max-questions",
        type=int,
        default=10,
        help="Maximum questions to evaluate (default: 10)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=600.0,
        help="Timeout per question in seconds (default: 600)",
    )
    parser.add_argument(
        "--start",
        type=int,
        default=0,
        help="Start from question index (0-based, default: 0)",
    )
    parser.add_argument(
        "--question",
        type=int,
        default=None,
        help="Evaluate only a single question by index (0-based, overrides --max-questions)",
    )
    args = parser.parse_args()
    
    print(f"Connecting to executive_node for character: {args.character}")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("Zenoh session opened")
    
    results = evaluate_hle(
        session=session,
        character=args.character,
        dataset_name=args.dataset,
        split=args.split,
        max_questions=args.max_questions,
        timeout=args.timeout,
        start_question=args.start,
        single_question=args.question,
    )
    
    session.close()
    print("\nEvaluation complete.")


if __name__ == "__main__":
    main()

