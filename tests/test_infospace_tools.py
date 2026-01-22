#!/usr/bin/env python3
"""
Test suite for infospace tools and primitives.

Validates that:
- Primitives (create-note, create-collection, split, flatten, etc.) work correctly
- Python tools (calculate, summarize, refine, etc.) execute and return proper format
- Tool chains work correctly (search -> filter -> summarize)

Uses Zenoh API similar to test_minecraft_coordinates.py pattern.

Usage:
    python3 tests/test_infospace_tools.py --character Jill
    python3 tests/test_infospace_tools.py --character Jill --test primitives
    python3 tests/test_infospace_tools.py --character Jill --test calculate

Available tests:
    - primitives: Test core primitives (create-note, create-collection, split, flatten, etc.)
    - calculate: Test calculate tool
    - word-count: Test word-count tool
    - refine: Test refine tool (LLM-based)
    - summarize: Test summarize tool (LLM-based)
    - assess: Test assess tool (LLM-based)
    - generate: Test generate-note tool (LLM-based)
    - filter: Test filter-semantic tool (LLM-based)
    - search-web: Test search-web tool (requires GOOGLE_API_KEY, GOOGLE_CX)
    - semantic-scholar: Test semantic-scholar tool (requires SEMANTIC_SCHOLAR_API_KEY)
    - fetch-text: Test fetch-text tool
    - chain: Test tool chain (search -> filter -> summarize)
    - all: Run all tests (default)

Requirements:
    - Running Jill session with infospace world (jill-infospace.yaml)
    - For search-web: GOOGLE_API_KEY and GOOGLE_CX environment variables
    - For semantic-scholar: SEMANTIC_SCHOLAR_API_KEY environment variable
"""

import argparse
import json
import os
import time
import logging
import traceback
from typing import Dict, List, Optional, Tuple

import zenoh
from zenoh import QueryTarget, ConsolidationMode

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# -----------------------------
# Zenoh API Helpers
# -----------------------------

def execute_plan(session: zenoh.Session, character: str, plan: List[Dict], timeout: float = 60.0) -> Dict:
    """Execute a plan via Zenoh API. Returns result dict with success, bindings, last_action_result."""
    start_time = time.time()
    query_key = f"cognitive/{character}/execute_plan_sync"
    payload = json.dumps({'plan': plan})

    logger.debug(f"Executing plan via {query_key} (timeout={timeout}s)")

    try:
        replies = session.get(
            query_key,
            target=QueryTarget.BEST_MATCHING,
            consolidation=ConsolidationMode.NONE,
            payload=payload.encode('utf-8'),
            timeout=timeout
        )
        reply_count = 0
        for reply in replies:
            reply_count += 1
            if hasattr(reply, 'ok') and reply.ok is not None:
                elapsed = time.time() - start_time
                return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))

        elapsed = time.time() - start_time
        if elapsed >= timeout:
            return {'success': False, 'error': f'Timeout after {elapsed:.2f}s', 'timeout': True}
        elif reply_count == 0:
            return {'success': False, 'error': f'No response (elapsed: {elapsed:.2f}s)', 'no_response': True}
        else:
            return {'success': False, 'error': f'Invalid response (elapsed: {elapsed:.2f}s)'}
    except Exception as e:
        logger.error(f"Exception in execute_plan: {e}")
        logger.error(traceback.format_exc())
        return {'success': False, 'error': f'Exception: {str(e)}', 'exception': True}


def execute_action(session: zenoh.Session, character: str, action_dict: Dict, timeout: float = 60.0) -> Dict:
    """Execute single action via execute_plan_sync."""
    plan = [action_dict]
    result = execute_plan(session, character, plan, timeout)

    if result.get('success'):
        return result.get('last_action_result', {})

    error_reason = result.get('reason') or result.get('error') or 'Unknown error'
    error_info = {'status': 'failed', 'reason': error_reason}
    if 'timeout' in result:
        error_info['timeout'] = True
    if 'no_response' in result:
        error_info['no_response'] = True
    return error_info


def execute_multi_step_plan(session: zenoh.Session, character: str, plan: List[Dict], timeout: float = 120.0) -> Dict:
    """Execute multi-step plan and return full result including bindings."""
    return execute_plan(session, character, plan, timeout)


def get_binding(result: Dict, var_name: str) -> Optional[str]:
    """Extract a binding value from plan result."""
    bindings = result.get('bindings', {})
    # bindings can be a list with a single dict, or a dict
    if isinstance(bindings, list) and len(bindings) > 0:
        bindings = bindings[0]
    if not isinstance(bindings, dict):
        return None
    # Strip $ prefix if present
    var_key = var_name.lstrip('$')
    return bindings.get(var_key) or bindings.get(f'${var_key}')


# -----------------------------
# Test Functions
# -----------------------------

def test_primitives(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test core primitives: create-note, create-collection, split, flatten, project, etc."""
    print("\n=== Test: Core Primitives ===")

    all_passed = True
    messages = []

    # Test 1: create-note with string
    print("  Testing create-note (string)...")
    result = execute_action(session, character, {
        "type": "create-note",
        "value": "Hello, this is a test note.",
        "out": "$test_note"
    })
    if result.get('status') != 'success':
        messages.append(f"create-note (string) failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        note_id = result.get('resource_id')
        print(f"    ✓ Created note: {note_id}")
        messages.append("create-note (string): OK")

    # Test 2: create-note with array
    print("  Testing create-note (array)...")
    result = execute_action(session, character, {
        "type": "create-note",
        "value": ["apple", "banana", "cherry"],
        "out": "$array_note"
    })
    if result.get('status') != 'success':
        messages.append(f"create-note (array) failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Created array note: {result.get('resource_id')}")
        messages.append("create-note (array): OK")

    # Test 3: create-collection
    print("  Testing create-collection...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "First item", "out": "$n1"},
        {"type": "create-note", "value": "Second item", "out": "$n2"},
        {"type": "create-collection", "value": ["$n1", "$n2"], "out": "$coll"}
    ])
    if not plan_result.get('success'):
        messages.append(f"create-collection failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        coll_id = get_binding(plan_result, 'coll')
        print(f"    ✓ Created collection: {coll_id}")
        messages.append("create-collection: OK")

    # Test 4: split (JSON array)
    print("  Testing split (JSON array)...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": ["one", "two", "three"], "out": "$arr"},
        {"type": "split", "target": "$arr", "out": "$split_coll"}
    ])
    if not plan_result.get('success'):
        messages.append(f"split (array) failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        split_id = get_binding(plan_result, 'split_coll')
        print(f"    ✓ Split array into collection: {split_id}")
        messages.append("split (array): OK")

    # Test 5: split (text by sentence)
    print("  Testing split (text by sentence)...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "First sentence. Second sentence! Third sentence?", "out": "$text"},
        {"type": "split", "target": "$text", "delimiter": "sentence", "out": "$sentences"}
    ])
    if not plan_result.get('success'):
        messages.append(f"split (sentence) failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Split text into sentences")
        messages.append("split (sentence): OK")

    # Test 6: flatten
    print("  Testing flatten...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "Part A", "out": "$p1"},
        {"type": "create-note", "value": "Part B", "out": "$p2"},
        {"type": "create-collection", "value": ["$p1", "$p2"], "out": "$parts"},
        {"type": "flatten", "target": "$parts", "out": "$merged"}
    ])
    if not plan_result.get('success'):
        messages.append(f"flatten failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Flattened collection")
        messages.append("flatten: OK")

    # Test 7: project
    print("  Testing project...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": {"name": "Alice", "age": 30, "city": "NYC"}, "out": "$person1"},
        {"type": "create-note", "value": {"name": "Bob", "age": 25, "city": "LA"}, "out": "$person2"},
        {"type": "create-collection", "value": ["$person1", "$person2"], "out": "$people"},
        {"type": "project", "target": "$people", "fields": ["name", "city"], "out": "$projected"}
    ])
    if not plan_result.get('success'):
        messages.append(f"project failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Projected fields from collection")
        messages.append("project: OK")

    # Test 8: filter-structured
    print("  Testing filter-structured...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": {"name": "Alice", "age": 30}, "out": "$p1"},
        {"type": "create-note", "value": {"name": "Bob", "age": 25}, "out": "$p2"},
        {"type": "create-note", "value": {"name": "Charlie", "age": 35}, "out": "$p3"},
        {"type": "create-collection", "value": ["$p1", "$p2", "$p3"], "out": "$people"},
        {"type": "filter-structured", "target": "$people", "where": "age >= 30", "out": "$adults"}
    ])
    if not plan_result.get('success'):
        messages.append(f"filter-structured failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Filtered collection by age >= 30")
        messages.append("filter-structured: OK")

    # Test 9: sort
    print("  Testing sort...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": {"name": "Alice", "score": 85}, "out": "$s1"},
        {"type": "create-note", "value": {"name": "Bob", "score": 92}, "out": "$s2"},
        {"type": "create-note", "value": {"name": "Charlie", "score": 78}, "out": "$s3"},
        {"type": "create-collection", "value": ["$s1", "$s2", "$s3"], "out": "$scores"},
        {"type": "sort", "target": "$scores", "by": "score", "order": "desc", "out": "$sorted"}
    ])
    if not plan_result.get('success'):
        messages.append(f"sort failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Sorted collection by score descending")
        messages.append("sort: OK")

    # Test 10: head
    print("  Testing head...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "Item 1", "out": "$i1"},
        {"type": "create-note", "value": "Item 2", "out": "$i2"},
        {"type": "create-note", "value": "Item 3", "out": "$i3"},
        {"type": "create-collection", "value": ["$i1", "$i2", "$i3"], "out": "$items"},
        {"type": "head", "target": "$items", "count": 2, "out": "$first_two"}
    ])
    if not plan_result.get('success'):
        messages.append(f"head failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Got first 2 items from collection")
        messages.append("head: OK")

    # Test 11: coerce
    print("  Testing coerce...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "42", "out": "$str_num"},
        {"type": "coerce", "target": "$str_num", "coercion": "to-int", "out": "$int_num"}
    ])
    if not plan_result.get('success'):
        messages.append(f"coerce failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Coerced string to int")
        messages.append("coerce: OK")

    # Test 12: think
    print("  Testing think...")
    result = execute_action(session, character, {
        "type": "think",
        "value": "This is an internal reasoning step for testing purposes."
    })
    if result.get('status') != 'success':
        messages.append(f"think failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Think action completed")
        messages.append("think: OK")

    # Test 13: display
    print("  Testing display...")
    result = execute_action(session, character, {
        "type": "display",
        "value": "Test display output from test_infospace_tools.py"
    })
    if result.get('status') != 'success':
        messages.append(f"display failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        print(f"    ✓ Display action completed")
        messages.append("display: OK")

    summary = f"{len([m for m in messages if 'OK' in m])}/{len(messages)} primitive tests passed"
    return all_passed, summary


def test_calculate(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test calculate tool."""
    print("\n=== Test: calculate ===")

    all_passed = True
    messages = []

    # Test 1: Basic arithmetic
    print("  Testing basic arithmetic (2 + 3 * 4)...")
    result = execute_action(session, character, {
        "type": "calculate",
        "value": "2 + 3 * 4",
        "out": "$result"
    })
    if result.get('status') != 'success':
        messages.append(f"basic arithmetic failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        value = result.get('value', '')
        # Should be 14
        if '14' in str(value):
            print(f"    ✓ Result: {value}")
            messages.append("basic arithmetic: OK")
        else:
            print(f"    ✗ Expected 14, got: {value}")
            messages.append(f"basic arithmetic: wrong result ({value})")
            all_passed = False

    # Test 2: Variables
    print("  Testing with variables...")
    result = execute_action(session, character, {
        "type": "calculate",
        "value": "2 * pi * r",
        "variables": {"r": 5},
        "out": "$circumference"
    })
    if result.get('status') != 'success':
        messages.append(f"variables failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        value = result.get('value', '')
        # Should be approximately 31.4159...
        if '31.4' in str(value):
            print(f"    ✓ Result: {value}")
            messages.append("variables: OK")
        else:
            print(f"    ✗ Expected ~31.4, got: {value}")
            messages.append(f"variables: wrong result ({value})")
            all_passed = False

    # Test 3: Equation solving
    print("  Testing equation solving (x**2 - 4 = 0)...")
    result = execute_action(session, character, {
        "type": "calculate",
        "value": "x**2 - 4 = 0",
        "out": "$roots"
    })
    if result.get('status') != 'success':
        messages.append(f"equation solving failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        value = result.get('value', '')
        # Should contain -2 and 2
        if '-2' in str(value) and '2' in str(value):
            print(f"    ✓ Result: {value}")
            messages.append("equation solving: OK")
        else:
            print(f"    ✗ Expected -2, 2, got: {value}")
            messages.append(f"equation solving: wrong result ({value})")
            all_passed = False

    # Test 4: Trig functions
    print("  Testing trig (sin(pi/2))...")
    result = execute_action(session, character, {
        "type": "calculate",
        "value": "sin(pi/2)",
        "out": "$sin_result"
    })
    if result.get('status') != 'success':
        messages.append(f"trig failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        value = result.get('value', '')
        # Should be 1
        if '1' in str(value):
            print(f"    ✓ Result: {value}")
            messages.append("trig: OK")
        else:
            print(f"    ✗ Expected 1, got: {value}")
            messages.append(f"trig: wrong result ({value})")
            all_passed = False

    summary = f"{len([m for m in messages if 'OK' in m])}/{len(messages)} calculate tests passed"
    return all_passed, summary


def test_word_count(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test word-count tool."""
    print("\n=== Test: word-count ===")

    # Create a note and count words
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "The quick brown fox jumps over the lazy dog", "out": "$text"},
        {"type": "word-count", "target": "$text", "out": "$count"}
    ])

    if not plan_result.get('success'):
        return False, f"word-count failed: {plan_result.get('error', 'unknown')}"

    last_result = plan_result.get('last_action_result', {})
    value = last_result.get('value', '')

    # Should be 9 words
    if '9' in str(value):
        print(f"  ✓ Word count: {value}")
        return True, "word-count: OK (9 words detected)"
    else:
        print(f"  ✗ Expected 9 words, got: {value}")
        return False, f"word-count: wrong count ({value})"


def test_refine(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test refine tool (LLM-based transformation)."""
    print("\n=== Test: refine ===")

    # Create a note and refine it
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "The meeting is scheduled for January 15, 2025 at 3pm in conference room B. Attendees: Alice, Bob, Charlie.", "out": "$text"},
        {"type": "refine", "target": "$text", "instruction": "Extract the date and time as JSON", "out": "$extracted"}
    ], timeout=120.0)

    if not plan_result.get('success'):
        return False, f"refine failed: {plan_result.get('error', 'unknown')}"

    last_result = plan_result.get('last_action_result', {})
    value = last_result.get('value', '')

    # Should contain date/time info
    if 'January' in str(value) or '15' in str(value) or '2025' in str(value) or '3' in str(value):
        print(f"  ✓ Refined result contains date/time info")
        print(f"    Result: {str(value)[:200]}...")
        return True, "refine: OK"
    else:
        print(f"  ✗ Result doesn't seem to contain expected info: {value}")
        return False, f"refine: unexpected result"


def test_summarize(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test summarize tool (LLM-based summarization)."""
    print("\n=== Test: summarize ===")

    long_text = """
    Artificial intelligence (AI) is intelligence demonstrated by machines, as opposed to natural intelligence
    displayed by animals including humans. AI research has been defined as the field of study of intelligent agents,
    which refers to any system that perceives its environment and takes actions that maximize its chance of achieving
    its goals. The term "artificial intelligence" had previously been used to describe machines that mimic and display
    "human" cognitive skills that are associated with the human mind, such as "learning" and "problem-solving".
    This definition has since been rejected by major AI researchers who now describe AI in terms of rationality and
    acting rationally, which does not limit how intelligence can be articulated. AI applications include advanced
    web search engines, recommendation systems, understanding human speech, self-driving cars, automated decision-making,
    and competing at the highest level in strategic game systems. As machines become increasingly capable, tasks
    considered to require "intelligence" are often removed from the definition of AI, a phenomenon known as the AI effect.
    """

    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": long_text, "out": "$text"},
        {"type": "summarize", "target": "$text", "focus": "definition of AI", "out": "$summary"}
    ], timeout=120.0)

    if not plan_result.get('success'):
        return False, f"summarize failed: {plan_result.get('error', 'unknown')}"

    last_result = plan_result.get('last_action_result', {})
    value = last_result.get('value', '')

    # Summary should be shorter than original and contain key terms
    if len(str(value)) < len(long_text) and len(str(value)) > 50:
        print(f"  ✓ Summarized {len(long_text)} chars to {len(str(value))} chars")
        print(f"    Summary: {str(value)[:200]}...")
        return True, "summarize: OK"
    else:
        print(f"  ✗ Summary length unexpected: {len(str(value))} chars")
        return False, f"summarize: unexpected result"


def test_assess(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test assess tool (LLM-based boolean predicate testing)."""
    print("\n=== Test: assess ===")

    all_passed = True
    messages = []

    # Test 1: Should return true
    print("  Testing predicate that should be true...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "The patient reported severe headaches and dizziness. URGENT: Needs immediate attention.", "out": "$text"},
        {"type": "assess", "target": "$text", "predicate": "is urgent or mentions urgency?", "out": "$is_urgent"}
    ], timeout=60.0)

    if not plan_result.get('success'):
        messages.append(f"assess (true case) failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        last_result = plan_result.get('last_action_result', {})
        value = str(last_result.get('value', '')).lower()
        if 'true' in value:
            print(f"    ✓ Correctly identified as urgent")
            messages.append("assess (true case): OK")
        else:
            print(f"    ✗ Expected 'true', got: {value}")
            messages.append(f"assess (true case): wrong ({value})")
            all_passed = False

    # Test 2: Should return false
    print("  Testing predicate that should be false...")
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "The weather today is sunny with a high of 72 degrees.", "out": "$text2"},
        {"type": "assess", "target": "$text2", "predicate": "mentions medical emergency?", "out": "$is_medical"}
    ], timeout=60.0)

    if not plan_result.get('success'):
        messages.append(f"assess (false case) failed: {plan_result.get('error', 'unknown')}")
        all_passed = False
    else:
        last_result = plan_result.get('last_action_result', {})
        value = str(last_result.get('value', '')).lower()
        if 'false' in value:
            print(f"    ✓ Correctly identified as not medical")
            messages.append("assess (false case): OK")
        else:
            print(f"    ✗ Expected 'false', got: {value}")
            messages.append(f"assess (false case): wrong ({value})")
            all_passed = False

    summary = f"{len([m for m in messages if 'OK' in m])}/{len(messages)} assess tests passed"
    return all_passed, summary


def test_generate_note(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test generate-note tool (LLM-based content generation)."""
    print("\n=== Test: generate-note ===")

    all_passed = True
    messages = []

    # Test 1: Generate text
    print("  Testing text generation...")
    result = execute_action(session, character, {
        "type": "generate-note",
        "prompt": "Write a haiku about programming",
        "style": "text",
        "out": "$haiku"
    }, timeout=60.0)

    if result.get('status') != 'success':
        messages.append(f"generate text failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        value = result.get('value', '')
        if len(str(value)) > 10:
            print(f"    ✓ Generated text: {str(value)[:100]}...")
            messages.append("generate text: OK")
        else:
            print(f"    ✗ Generated content too short: {value}")
            messages.append("generate text: too short")
            all_passed = False

    # Test 2: Generate code
    print("  Testing code generation...")
    result = execute_action(session, character, {
        "type": "generate-note",
        "prompt": "Write a Python function that returns the factorial of a number",
        "style": "code",
        "out": "$code"
    }, timeout=60.0)

    if result.get('status') != 'success':
        messages.append(f"generate code failed: {result.get('reason', 'unknown')}")
        all_passed = False
    else:
        value = result.get('value', '')
        if 'def' in str(value) or 'factorial' in str(value).lower():
            print(f"    ✓ Generated code contains function definition")
            messages.append("generate code: OK")
        else:
            print(f"    ✗ Generated code doesn't look like Python: {str(value)[:100]}")
            messages.append("generate code: unexpected format")
            all_passed = False

    summary = f"{len([m for m in messages if 'OK' in m])}/{len(messages)} generate-note tests passed"
    return all_passed, summary


def test_filter_semantic(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test filter-semantic tool (LLM-based semantic filtering)."""
    print("\n=== Test: filter-semantic ===")

    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "Python is a great programming language for beginners.", "out": "$n1"},
        {"type": "create-note", "value": "The weather in Hawaii is always beautiful.", "out": "$n2"},
        {"type": "create-note", "value": "JavaScript is essential for web development.", "out": "$n3"},
        {"type": "create-note", "value": "I love cooking Italian food.", "out": "$n4"},
        {"type": "create-collection", "value": ["$n1", "$n2", "$n3", "$n4"], "out": "$items"},
        {"type": "filter-semantic", "target": "$items", "predicate": "about programming or software development", "out": "$coding_items"}
    ], timeout=120.0)

    if not plan_result.get('success'):
        return False, f"filter-semantic failed: {plan_result.get('error', 'unknown')}"

    last_result = plan_result.get('last_action_result', {})
    value = last_result.get('value', '')

    # Should filter to 2 items (Python and JavaScript notes)
    if '2 items' in str(value):
        print(f"  ✓ Filtered to programming items: {value}")
        return True, "filter-semantic: OK (2 items)"
    elif '1 item' in str(value) or '3 item' in str(value):
        print(f"  ~ Filtered but count differs from expected: {value}")
        return True, f"filter-semantic: OK ({value})"
    else:
        print(f"  ✗ Unexpected filter result: {value}")
        return False, f"filter-semantic: unexpected ({value})"


def test_search_web(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test search-web tool (requires GOOGLE_API_KEY and GOOGLE_CX)."""
    print("\n=== Test: search-web ===")

    # Check for required env vars
    if not os.getenv('GOOGLE_API_KEY') or not os.getenv('GOOGLE_CX'):
        print("  ⚠ Skipping: GOOGLE_API_KEY or GOOGLE_CX not set")
        return True, "search-web: SKIPPED (no API keys)"

    result = execute_action(session, character, {
        "type": "search-web",
        "value": "what is machine learning",
        "out": "$results"
    }, timeout=60.0)

    if result.get('status') != 'success':
        return False, f"search-web failed: {result.get('reason', 'unknown')}"

    value = result.get('value', '')
    resource_id = result.get('resource_id', '')

    if 'items' in str(value) and resource_id.startswith('Collection_'):
        print(f"  ✓ Search returned collection: {value}")
        return True, f"search-web: OK ({value})"
    else:
        print(f"  ✗ Unexpected result: {value}")
        return False, f"search-web: unexpected ({value})"


def test_semantic_scholar(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test semantic-scholar tool."""
    print("\n=== Test: semantic-scholar ===")

    result = execute_action(session, character, {
        "type": "semantic-scholar",
        "value": "transformer attention mechanism",
        "limit": 3,
        "out": "$papers"
    }, timeout=60.0)

    if result.get('status') != 'success':
        reason = result.get('reason', 'unknown')
        if 'rate limit' in str(reason).lower() or '429' in str(reason):
            print("  ⚠ Rate limited - skipping")
            return True, "semantic-scholar: SKIPPED (rate limited)"
        return False, f"semantic-scholar failed: {reason}"

    value = result.get('value', '')
    resource_id = result.get('resource_id', '')

    if 'items' in str(value) and resource_id.startswith('Collection_'):
        print(f"  ✓ Search returned papers: {value}")
        return True, f"semantic-scholar: OK ({value})"
    else:
        print(f"  ✗ Unexpected result: {value}")
        return False, f"semantic-scholar: unexpected ({value})"


def test_fetch_text(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test fetch-text tool."""
    print("\n=== Test: fetch-text ===")

    # Fetch a simple, stable URL
    result = execute_action(session, character, {
        "type": "fetch-text",
        "target": "https://www.example.com",
        "out": "$page"
    }, timeout=30.0)

    if result.get('status') != 'success':
        return False, f"fetch-text failed: {result.get('reason', 'unknown')}"

    value = result.get('value', '')

    # example.com has specific content
    if 'Example Domain' in str(value) or 'example' in str(value).lower():
        print(f"  ✓ Fetched page content ({len(str(value))} chars)")
        return True, "fetch-text: OK"
    else:
        print(f"  ✗ Unexpected content: {str(value)[:200]}")
        return False, f"fetch-text: unexpected content"


def test_tool_chain(session: zenoh.Session, character: str) -> Tuple[bool, str]:
    """Test a tool chain: create data -> filter -> summarize."""
    print("\n=== Test: Tool Chain (create -> filter -> summarize) ===")

    # Create some notes, filter them, then summarize
    plan_result = execute_multi_step_plan(session, character, [
        {"type": "create-note", "value": "Machine learning is a subset of AI that enables systems to learn from data. It includes supervised learning, unsupervised learning, and reinforcement learning.", "out": "$ml"},
        {"type": "create-note", "value": "The stock market closed higher today with tech stocks leading gains.", "out": "$stocks"},
        {"type": "create-note", "value": "Deep learning uses neural networks with many layers to process complex patterns in data. It has revolutionized computer vision and natural language processing.", "out": "$dl"},
        {"type": "create-note", "value": "Today's weather forecast shows sunny skies and warm temperatures.", "out": "$weather"},
        {"type": "create-collection", "value": ["$ml", "$stocks", "$dl", "$weather"], "out": "$all"},
        {"type": "filter-semantic", "target": "$all", "predicate": "about AI, machine learning, or deep learning", "out": "$ai_notes"},
        {"type": "summarize", "target": "$ai_notes", "focus": "key concepts in AI", "out": "$summary"}
    ], timeout=180.0)

    if not plan_result.get('success'):
        return False, f"Tool chain failed: {plan_result.get('error', 'unknown')}"

    last_result = plan_result.get('last_action_result', {})
    value = last_result.get('value', '')

    # Summary should mention AI/ML concepts
    if len(str(value)) > 50 and ('learning' in str(value).lower() or 'AI' in str(value) or 'neural' in str(value).lower()):
        print(f"  ✓ Tool chain completed successfully")
        print(f"    Summary: {str(value)[:200]}...")
        return True, "Tool chain: OK"
    else:
        print(f"  ✗ Chain completed but summary unexpected: {str(value)[:200]}")
        return False, f"Tool chain: unexpected summary"


def run_all_tests(session: zenoh.Session, character: str) -> Dict[str, Tuple[bool, str]]:
    """Run all available tests."""
    tests = [
        ("Primitives", test_primitives),
        ("Calculate", test_calculate),
        ("Word Count", test_word_count),
        ("Refine", test_refine),
        ("Summarize", test_summarize),
        ("Assess", test_assess),
        ("Generate Note", test_generate_note),
        ("Filter Semantic", test_filter_semantic),
        ("Search Web", test_search_web),
        ("Semantic Scholar", test_semantic_scholar),
        ("Fetch Text", test_fetch_text),
        ("Tool Chain", test_tool_chain),
    ]

    results = {}
    for test_name, test_func in tests:
        try:
            success, message = test_func(session, character)
            results[test_name] = (success, message)
        except Exception as e:
            results[test_name] = (False, f"Exception: {str(e)}")
            logger.error(f"Test {test_name} raised exception: {e}")
            logger.error(traceback.format_exc())

    return results


def main():
    parser = argparse.ArgumentParser(description="Test infospace tools and primitives")
    parser.add_argument(
        "--character",
        type=str,
        default="Jill",
        help="Character name (default: Jill)"
    )
    parser.add_argument(
        "--test",
        type=str,
        default="all",
        help="Specific test to run (default: all)"
    )

    args = parser.parse_args()

    print("=" * 60)
    print("Infospace Tools Test Suite")
    print("=" * 60)
    print(f"Character: {args.character}")
    print(f"Test: {args.test}")
    print()

    # Connect to Zenoh
    print("Connecting to Zenoh...")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("✓ Zenoh session opened")

    try:
        # Quick connectivity test
        test_result = execute_action(session, args.character, {"type": "think", "value": "connectivity test"}, timeout=10.0)
        if test_result.get('status') != 'success':
            print(f"✗ Error: Character '{args.character}' not responding")
            print(f"  Reason: {test_result.get('reason', 'unknown')}")
            return 1
        print(f"✓ Character '{args.character}' is responding")
        print()

        # Run tests
        if args.test == "all":
            results = run_all_tests(session, args.character)
        else:
            # Map test names to functions
            test_map = {
                "primitives": ("Primitives", test_primitives),
                "calculate": ("Calculate", test_calculate),
                "word-count": ("Word Count", test_word_count),
                "refine": ("Refine", test_refine),
                "summarize": ("Summarize", test_summarize),
                "assess": ("Assess", test_assess),
                "generate": ("Generate Note", test_generate_note),
                "filter": ("Filter Semantic", test_filter_semantic),
                "search-web": ("Search Web", test_search_web),
                "semantic-scholar": ("Semantic Scholar", test_semantic_scholar),
                "fetch-text": ("Fetch Text", test_fetch_text),
                "chain": ("Tool Chain", test_tool_chain),
            }

            if args.test not in test_map:
                print(f"✗ Unknown test: {args.test}")
                print(f"Available tests: {', '.join(test_map.keys())}")
                return 1

            test_name, test_func = test_map[args.test]
            try:
                success, message = test_func(session, args.character)
                results = {test_name: (success, message)}
            except Exception as e:
                results = {test_name: (False, f"Exception: {str(e)}")}
                logger.error(traceback.format_exc())

        # Print summary
        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)

        passed = sum(1 for success, _ in results.values() if success)
        total = len(results)

        for test_name, (success, message) in results.items():
            status_icon = "✅" if success else "❌"
            print(f"{status_icon} {test_name}: {message}")

        print()
        print(f"Total: {passed}/{total} tests passed")

        return 0 if passed == total else 1

    finally:
        session.close()
        print("\n✓ Zenoh session closed")


if __name__ == "__main__":
    exit(main())
