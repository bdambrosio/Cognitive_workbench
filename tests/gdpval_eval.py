#!/usr/bin/env python3
"""
GDPval evaluation harness for Cognitive Workbench (Phase 1: Text-only).

- Loads GDPval from HuggingFace: openai/gdpval
- Handles reference files via fetch-text plan (loads into named notes)
- Sends goals via Zenoh to executive_node
- Evaluates with exact match, contains match, F1, and LLM judge
- Clears transient Notes between questions

Phase 1: Asks for text descriptions of deliverables (not actual file generation).
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
    def remove_articles(text):
        return re.sub(r'\b(a|an|the)\b', ' ', text)
    
    def white_space_fix(text):
        return ' '.join(text.split())
    
    def remove_punc(text):
        exclude = set(string.punctuation)
        return ''.join(ch for ch in text if ch not in exclude)
    
    def lower(text):
        return text.lower()
    
    return white_space_fix(remove_articles(remove_punc(lower(s))))


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


def llm_judge(session: zenoh.Session, character: str, prompt: str, gold_answer: str, prediction: str, verbose: bool = False) -> tuple:
    """Use LLM to judge if prediction correctly answers the question. Returns (is_correct, raw_response, error)."""
    judge_prompt = f"""You are evaluating a task completion. The task asked for specific deliverables.

Task prompt: {prompt[:500]}
Expected deliverables: {gold_answer}
Actual response: {prediction}

Does the actual response demonstrate completion of the task? The response may describe deliverables in text form rather than generating actual files, but should show understanding of what was requested.

Reply with only YES or NO."""
    
    result, error = llm_generate(session, character, [judge_prompt], max_tokens=10, temperature=0.0)
    
    if error:
        print(f"       LLM Judge ERROR: {error}")
        return False, '', error
    
    is_correct = result.strip().upper().startswith('YES')
    
    if verbose or not is_correct:
        print(f"       LLM Judge: '{result.strip()}' -> {is_correct}")
    
    return is_correct, result.strip(), None


def clear_transient_notes(session: zenoh.Session, character: str, timeout: float = 10.0) -> int:
    """Clear all non-persistent Notes via Zenoh API."""
    query_key = f"cognitive/{character}/resource/clear_transient"
    replies = session.get(query_key, timeout=timeout)
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            payload = reply.ok.payload.to_bytes().decode('utf-8')
            result = json.loads(payload)
            if result.get('success'):
                return result.get('deleted_notes', 0) + result.get('deleted_collections', 0)
    return 0


def execute_plan(session: zenoh.Session, character: str, plan: list, timeout: float = 60.0) -> dict:
    """Execute a plan via Zenoh API. Returns result dict with success, bindings, last_action_result."""
    query_key = f"cognitive/{character}/execute_plan_sync"
    payload = json.dumps({'plan': plan})
    replies = session.get(query_key, payload=payload.encode('utf-8'), timeout=timeout)
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
    return {'success': False, 'error': 'No response from execute_plan_sync'}


def fetch_file_to_named_note(session: zenoh.Session, character: str, file_url: str, note_name: str, timeout: float = 60.0) -> bool:
    """Fetch file URL using fetch-text and create named Note. Returns True if successful."""
    plan = [
        {
            "type": "fetch-text",
            "value": file_url,
            "out": "$fetched_data"
        },
        {
            "type": "create-note",
            "value": "$fetched_data",
            "name": note_name,
            "out": "$context_note"
        }
    ]
    result = execute_plan(session, character, plan, timeout)
    return result.get('success', False)


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


def build_gdpval_prompt(prompt: str, reference_files: list, has_files: bool = False) -> str:
    """Build goal prompt for GDPval task."""
    lines = ["goal:"]
    lines.append(f"  task: {prompt}")
    
    if has_files:
        lines.append("")
        lines.append("  reference files:")
        for i, ref_file in enumerate(reference_files, 1):
            lines.append(f"    - {ref_file}")
        lines.append("")
        lines.append("  instructions:")
        lines.append("    - Load reference files using load with target: '<filename>' (without extension)")
        lines.append("    - Review the task requirements and reference materials")
        lines.append("    - Describe what deliverables you would create to complete this task")
        lines.append("    - Include details about the format, content, and structure of each deliverable")
    else:
        lines.append("")
        lines.append("  instructions:")
        lines.append("    - Review the task requirements")
        lines.append("    - Describe what deliverables you would create to complete this task")
        lines.append("    - Include details about the format, content, and structure of each deliverable")
    
    lines.append("    - Format: ANSWER: <description of deliverables>")
    lines.append("    - Finally, say the answer to User")
    
    return '\n'.join(lines)


def evaluate_gdpval(
    session: zenoh.Session,
    character: str,
    max_tasks: int = 50,
    timeout: float = 600.0,
) -> dict:
    """Run GDPval evaluation."""
    print(f"\n=== GDPval Evaluation (max={max_tasks}) ===")
    
    # Load dataset
    try:
        ds = load_dataset("openai/gdpval", split="train")
        print(f"Loaded GDPval dataset: {len(ds)} tasks")
    except Exception as e:
        print(f"ERROR: Failed to load GDPval dataset: {e}")
        print("Note: GDPval dataset may require HuggingFace authentication/agreement")
        return {'error': str(e)}
    
    results = {
        'exact_match': 0,
        'contains_match': 0,
        'llm_judge': 0,
        'f1_sum': 0.0,
        'total': 0,
        'errors': 0,
        'by_sector': {},
    }
    
    indices = range(min(max_tasks, len(ds)))
    
    for idx in indices:
        item = ds[idx]
        task_id = item.get('task_id', '')
        sector = item.get('sector', 'unknown')
        occupation = item.get('occupation', 'unknown')
        prompt = item.get('prompt', '')
        reference_files = item.get('reference_files', [])
        reference_file_urls = item.get('reference_file_urls', [])
        
        # For Phase 1, we don't have ground truth answers in the dataset
        # So we'll use LLM judge primarily, with text-based metrics as secondary
        # The "gold answer" will be a description of what the task requires
        gold_answer = f"Complete the task: {prompt[:200]}"
        
        # Clear transient notes from previous task
        cleared = clear_transient_notes(session, character)
        if cleared > 0:
            print(f"  [Cleared {cleared} transient notes]")
        
        # Handle reference files - fetch and create named notes
        has_files = False
        if reference_files and reference_file_urls:
            print(f"  [Task {idx}: {sector}/{occupation}]")
            print(f"  [Fetching {len(reference_files)} reference file(s)]")
            for ref_file, ref_url in zip(reference_files, reference_file_urls):
                if ref_url:
                    # Use filename (without extension) as note name
                    note_name = ref_file.rsplit('.', 1)[0] if '.' in ref_file else ref_file
                    note_name = note_name.replace(' ', '_').replace('-', '_')
                    print(f"    Fetching {ref_file} -> note '{note_name}'")
                    if fetch_file_to_named_note(session, character, ref_url, note_name, timeout=60.0):
                        has_files = True
                        print(f"    ✓ Loaded into note '{note_name}'")
                    else:
                        print(f"    ✗ Failed to fetch {ref_file}")
        
        # Build prompt
        goal_text = build_gdpval_prompt(prompt, reference_files, has_files)
        
        # Send goal and wait
        t0 = time.time()
        response = send_goal_and_wait(session, character, goal_text, timeout)
        dt = time.time() - t0
        
        if 'error' in response and response['error']:
            print(f"[{idx:4d}] ERROR: {response['error']}")
            results['errors'] += 1
            continue
        
        pred_answer = extract_answer(response['answer'])
        
        # For Phase 1, we primarily use LLM judge since we don't have exact ground truth
        # Text metrics are less meaningful without reference answers
        em = exact_match(pred_answer, gold_answer)  # Will likely be low
        cm = contains_match(pred_answer, gold_answer)  # May catch some matches
        f1 = f1_score(pred_answer, gold_answer)  # Token overlap
        lj, lj_response, lj_error = llm_judge(session, character, prompt, gold_answer, pred_answer)
        
        results['total'] += 1
        if em:
            results['exact_match'] += 1
        if cm:
            results['contains_match'] += 1
        if lj:
            results['llm_judge'] += 1
        results['f1_sum'] += f1
        
        # Track by sector
        if sector not in results['by_sector']:
            results['by_sector'][sector] = {'correct': 0, 'total': 0}
        results['by_sector'][sector]['total'] += 1
        if lj:
            results['by_sector'][sector]['correct'] += 1
        
        status = '✓' if lj else ('+' if cm or f1 > 0.3 else '✗')
        print(f"[{idx:4d}] {status} EM={int(em)} CM={int(cm)} LJ={int(lj)} F1={f1:.2f} ({dt:.1f}s)")
        print(f"       Sector: {sector}, Occupation: {occupation}")
        print(f"       Task: {prompt[:100]}...")
        print(f"       Response: {pred_answer[:150]}...")
    
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
    
    if results['by_sector']:
        print(f"\n=== By Sector ===")
        for sector, stats in sorted(results['by_sector'].items()):
            acc = stats['correct'] / stats['total'] if stats['total'] > 0 else 0.0
            print(f"{sector:30s}: {acc:.3f} ({stats['correct']}/{stats['total']})")
    
    return {
        'exact_match': em_acc,
        'contains_match': cm_acc,
        'llm_judge': lj_acc,
        'f1': avg_f1,
        'total': results['total'],
        'errors': results['errors'],
        'by_sector': results['by_sector'],
    }


def main():
    parser = argparse.ArgumentParser(description="GDPval evaluation for Cognitive Workbench (Phase 1: Text-only)")
    parser.add_argument(
        "--character",
        type=str,
        default="Jill",
        help="Character name for executive_node (default: Jill)",
    )
    parser.add_argument(
        "--max-tasks",
        type=int,
        default=50,
        help="Maximum tasks to evaluate (default: 50)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=600.0,
        help="Timeout per task in seconds (default: 600)",
    )
    
    args = parser.parse_args()
    
    print(f"Connecting to executive_node for character: {args.character}")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("Zenoh session opened")
    
    results = evaluate_gdpval(
        session=session,
        character=args.character,
        max_tasks=args.max_tasks,
        timeout=args.timeout,
    )
    
    session.close()
    print("\nEvaluation complete.")


if __name__ == "__main__":
    main()
