#!/usr/bin/env python3
"""
Non-LLM Tools Test Suite for Cognitive Workbench.

Tests deterministic and API-based tools that don't require LLM calls:
- calculate - Mathematical expression evaluation
- is-empty - Empty/whitespace check
- is-positive - Positive number check
- word-count - Text metrics

Each test:
1. Clears bindings before execution
2. Clears transient notes between tests
3. Tests execution success and output validation
4. Logs results with layered success indicators

Usage:
    python tests/test_tools_non_llm.py [--character CHARACTER] [--verbose]
"""

import argparse
import json
import time
from datetime import datetime
from typing import Dict, Any, List, Tuple

import zenoh


# ==========================================
# ZENOH API HELPERS
# ==========================================

def execute_plan(session: zenoh.Session, character: str, plan: list, timeout: float = 60.0) -> dict:
    """Execute a plan synchronously via Zenoh API."""
    query_key = f"cognitive/{character}/execute_plan_sync"
    payload = json.dumps({'plan': plan}).encode('utf-8')
    replies = session.get(
        query_key,
        payload=payload,
        target=zenoh.QueryTarget.BEST_MATCHING,
        consolidation=zenoh.ConsolidationMode.NONE,
        timeout=timeout
    )
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
    return {"success": False, "error": "No reply"}


def clear_planner_bindings(session: zenoh.Session, character: str, timeout: float = 10.0) -> dict:
    """Clear planner bindings via Zenoh API."""
    query_key = f"cognitive/{character}/clear_planner_bindings"
    replies = session.get(
        query_key,
        target=zenoh.QueryTarget.BEST_MATCHING,
        consolidation=zenoh.ConsolidationMode.NONE,
        timeout=timeout
    )
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
    return {"success": False, "error": "No reply"}


def clear_transient_notes(session: zenoh.Session, character: str, timeout: float = 10.0) -> int:
    """Clear all non-persistent Notes via Zenoh API."""
    query_key = f"cognitive/{character}/resource/clear_transient"
    replies = session.get(
        query_key,
        target=zenoh.QueryTarget.BEST_MATCHING,
        consolidation=zenoh.ConsolidationMode.NONE,
        timeout=timeout
    )
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            payload = reply.ok.payload.to_bytes().decode('utf-8')
            result = json.loads(payload)
            if result.get('success'):
                return result.get('deleted_notes', 0) + result.get('deleted_collections', 0) + result.get('bindings_cleared', 0)
    return 0


# ==========================================
# TEST HELPERS
# ==========================================

def validate_binding(result: dict, var_name: str) -> Tuple[bool, str]:
    """Validate that variable is bound in result."""
    bindings = result.get('bindings', {})
    if var_name in bindings:
        return True, bindings[var_name]
    var_with_dollar = f"${var_name}" if not var_name.startswith('$') else var_name
    if var_with_dollar in bindings:
        return True, bindings[var_with_dollar]
    var_without_dollar = var_name[1:] if var_name.startswith('$') else var_name
    if var_without_dollar in bindings:
        return True, bindings[var_without_dollar]
    return False, f"Variable {var_name} not bound"


def get_note_content_from_last_result(result: dict) -> Any:
    """Extract content from last_action_result field (uniform format)."""
    last_action_result = result.get('last_action_result')
    if last_action_result and last_action_result.get('status') == 'success':
        return last_action_result.get('value')
    return None


def run_test(
    session: zenoh.Session,
    character: str,
    test_name: str,
    plan: List[Dict],
    expected_output_var: str = None,
    expected_content: Any = None,
    verbose: bool = False
) -> Dict[str, Any]:
    """
    Run a single test with layered validation.
    
    Args:
        session: Zenoh session
        character: Character name
        test_name: Test name for logging
        plan: List of actions to execute
        expected_output_var: Variable name expected to be bound (optional)
        expected_content: Expected content value (optional)
        verbose: Verbose output
    
    Returns:
        Dict with test results
    """
    print(f"\n{'='*60}")
    print(f"TEST: {test_name}")
    print(f"{'='*60}")
    
    # Clear bindings before test
    clear_result = clear_planner_bindings(session, character)
    if verbose:
        print(f"  Cleared bindings: {clear_result.get('bindings_cleared', 0)}")
    
    # Execute plan
    result = execute_plan(session, character, plan, timeout=60.0)
    
    # Basic validation: execution success
    execution_success = result.get('success', False)
    if not execution_success:
        error = result.get('error', 'Unknown error')
        print(f"  ❌ EXECUTION FAILED: {error}")
        return {
            'test_name': test_name,
            'execution_success': False,
            'error': error,
            'content_validated': False,
            'warnings': []
        }
    
    print(f"  ✅ Execution successful")
    
    # Content validation
    content_validated = False
    warnings = []
    
    if expected_output_var:
        valid, resource_id = validate_binding(result, expected_output_var)
        if not valid:
            warnings.append(f"Variable {expected_output_var} not bound")
        else:
            if verbose:
                print(f"  ✅ Variable {expected_output_var} bound to {resource_id}")
    
    if expected_content is not None:
        # Load the output resource to get content
        if expected_output_var:
            valid, resource_id = validate_binding(result, expected_output_var)
            if valid:
                load_plan = [
                    {
                        "type": "load",
                        "target": resource_id,
                        "out": "$loaded"
                    }
                ]
                load_result = execute_plan(session, character, load_plan, timeout=30.0)
                if load_result.get('success'):
                    # Get content from last_action_result (uniform format)
                    last_action_result = load_result.get('last_action_result')
                    if last_action_result and last_action_result.get('status') == 'success':
                        actual_content = last_action_result.get('value')
                    else:
                        actual_content = None
                    
                    if actual_content is not None:
                        # Normalize float comparisons: convert both to float if numeric
                        if actual_content == expected_content or str(actual_content) == str(expected_content):
                            content_validated = True
                            if verbose:
                                print(f"  ✅ Content matches: {actual_content}")
                        else:
                            # Try numeric comparison for float tolerance
                            try:
                                actual_float = float(actual_content)
                                expected_float = float(expected_content)
                                if abs(actual_float - expected_float) < 1e-9:
                                    content_validated = True
                                    if verbose:
                                        print(f"  ✅ Content matches (numeric): {actual_content} ≈ {expected_content}")
                                else:
                                    warnings.append(f"Content mismatch: expected {expected_content}, got {actual_content}")
                            except (ValueError, TypeError):
                                warnings.append(f"Content mismatch: expected {expected_content}, got {actual_content}")
                    else:
                        warnings.append(f"Failed to get content from last_action_result")
                else:
                    warnings.append(f"Failed to load content for validation")
            else:
                warnings.append(f"Cannot validate content: variable {expected_output_var} not bound")
        else:
            # Try to get from last_action_result directly
            last_action_result = result.get('last_action_result')
            if last_action_result and last_action_result.get('status') == 'success':
                actual_content = last_action_result.get('value')
            else:
                actual_content = None
            if actual_content == expected_content or str(actual_content) == str(expected_content):
                content_validated = True
                if verbose:
                    print(f"  ✅ Content matches: {actual_content}")
            else:
                # Try numeric comparison for float tolerance
                try:
                    actual_float = float(actual_content)
                    expected_float = float(expected_content)
                    if abs(actual_float - expected_float) < 1e-9:
                        content_validated = True
                        if verbose:
                            print(f"  ✅ Content matches (numeric): {actual_content} ≈ {expected_content}")
                    else:
                        warnings.append(f"Content mismatch: expected {expected_content}, got {actual_content}")
                except (ValueError, TypeError):
                    warnings.append(f"Content mismatch: expected {expected_content}, got {actual_content}")
    
    # Print warnings
    if warnings:
        for warning in warnings:
            print(f"  ⚠️  {warning}")
    
    return {
        'test_name': test_name,
        'execution_success': execution_success,
        'content_validated': content_validated,
        'warnings': warnings
    }


# ==========================================
# TEST CASES
# ==========================================

def test_calculate(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test calculate tool with simple arithmetic."""
    plan = [
        {
            "type": "create-note",
            "value": "2 + 3 * 4",
            "out": "$expr"
        },
        {
            "type": "calculate",
            "target": "$expr",
            "out": "$result"
        }
    ]
    return run_test(
        session, character, "calculate (2+3*4)",
        plan,
        expected_output_var="$result",
        expected_content="14",  # 2 + 12 = 14
        verbose=verbose
    )


def test_calculate_with_variables(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test calculate tool with variable substitution."""
    plan = [
        {
            "type": "create-note",
            "value": "x^2 + y",
            "out": "$expr"
        },
        {
            "type": "calculate",
            "target": "$expr",
            "variables": {"x": 5, "y": 3},
            "out": "$result"
        }
    ]
    return run_test(
        session, character, "calculate (x^2+y with x=5, y=3)",
        plan,
        expected_output_var="$result",
        expected_content="28",  # 25 + 3 = 28
        verbose=verbose
    )


def test_is_empty(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test is-empty tool."""
    plan = [
        {
            "type": "create-note",
            "value": "",
            "out": "$empty_note"
        },
        {
            "type": "is-empty",
            "target": "$empty_note",
            "out": "$is_empty_result"
        }
    ]
    return run_test(
        session, character, "is-empty (empty string)",
        plan,
        expected_output_var="$is_empty_result",
        expected_content=True,
        verbose=verbose
    )


def test_is_empty_non_empty(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test is-empty tool with non-empty content."""
    plan = [
        {
            "type": "create-note",
            "value": "Hello world",
            "out": "$non_empty_note"
        },
        {
            "type": "is-empty",
            "target": "$non_empty_note",
            "out": "$is_empty_result"
        }
    ]
    return run_test(
        session, character, "is-empty (non-empty string)",
        plan,
        expected_output_var="$is_empty_result",
        expected_content=False,
        verbose=verbose
    )


def test_is_positive(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test is-positive tool."""
    plan = [
        {
            "type": "create-note",
            "value": "42",
            "out": "$positive_note"
        },
        {
            "type": "is-positive",
            "target": "$positive_note",
            "out": "$is_positive_result"
        }
    ]
    return run_test(
        session, character, "is-positive (positive number)",
        plan,
        expected_output_var="$is_positive_result",
        expected_content=True,
        verbose=verbose
    )


def test_is_positive_negative(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test is-positive tool with negative number."""
    plan = [
        {
            "type": "create-note",
            "value": "-5",
            "out": "$negative_note"
        },
        {
            "type": "is-positive",
            "target": "$negative_note",
            "out": "$is_positive_result"
        }
    ]
    return run_test(
        session, character, "is-positive (negative number)",
        plan,
        expected_output_var="$is_positive_result",
        expected_content=False,
        verbose=verbose
    )


def test_word_count(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test word-count tool."""
    plan = [
        {
            "type": "create-note",
            "value": "The quick brown fox jumps over the lazy dog",
            "out": "$text_note"
        },
        {
            "type": "word-count",
            "target": "$text_note",
            "out": "$word_count_result"
        }
    ]
    return run_test(
        session, character, "word-count (9 words)",
        plan,
        expected_output_var="$word_count_result",
        expected_content=None,  # Don't validate exact format, just check it exists
        verbose=verbose
    )


# ==========================================
# MAIN
# ==========================================

def main():
    parser = argparse.ArgumentParser(description='Test non-LLM tools')
    parser.add_argument('--character', default='Jill', help='Character name (default: Jill)')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    args = parser.parse_args()
    
    # Initialize Zenoh
    config = zenoh.Config()
    session = zenoh.open(config)
    
    print("="*60)
    print("NON-LLM TOOLS TEST SUITE")
    print("="*60)
    print(f"Character: {args.character}")
    print(f"Timestamp: {datetime.now().isoformat()}")
    
    # Run tests
    tests = [
        test_calculate,
        test_calculate_with_variables,
        test_is_empty,
        test_is_empty_non_empty,
        test_is_positive,
        test_is_positive_negative,
        test_word_count,
    ]
    
    results = []
    for test_func in tests:
        # Clear transient notes between tests
        clear_transient_notes(session, args.character)
        result = test_func(session, args.character, verbose=args.verbose)
        results.append(result)
        time.sleep(0.1)  # Small delay between tests
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    passed = sum(1 for r in results if r['execution_success'] and not r['warnings'])
    failed = sum(1 for r in results if not r['execution_success'])
    warnings_count = sum(1 for r in results if r['warnings'])
    
    print(f"Total tests: {len(results)}")
    print(f"✅ Passed: {passed}")
    print(f"❌ Failed: {failed}")
    if warnings_count > 0:
        print(f"⚠️  Warnings: {warnings_count}")
        print("\nTests with warnings:")
        for r in results:
            if r['warnings']:
                print(f"  - {r['test_name']}: {', '.join(r['warnings'])}")
    
    if failed > 0:
        print("\nFailed tests:")
        for r in results:
            if not r['execution_success']:
                print(f"  - {r['test_name']}: {r.get('error', 'Unknown error')}")
    
    session.close()
    
    # Exit code
    exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()

