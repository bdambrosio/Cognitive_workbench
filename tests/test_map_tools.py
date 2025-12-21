#!/usr/bin/env python3
"""
Map Tools Test Suite for Cognitive Workbench.

Tests the `map` primitive with tools that take arguments beyond target/out:
- refine - with instruction parameter
- calculate - with variables parameter
- summarize - with focus/max_length parameters
- relate - with other parameter (Note reference)
- search-web - with query parameter (API-based, long timeout)
- semantic-scholar - with query parameter (API-based, long timeout)

Focus: Dict form of tool action (as planner generates): {"tool": "name", "arg": "value"}

Note: generate-note is excluded - it ignores the mapped value parameter and is
      inappropriate for map operations (designed for scratch generation, not transformation).

Each test:
1. Clears bindings before execution
2. Clears transient notes between tests
3. Tests execution success and output Collection size
4. Validates content where feasible
5. Logs results with layered success indicators

Usage:
    python tests/test_map_tools.py [--character CHARACTER] [--verbose]
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

def execute_plan(session: zenoh.Session, character: str, plan: list, timeout: float = 180.0) -> dict:
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


def run_test(
    session: zenoh.Session,
    character: str,
    test_name: str,
    plan: List[Dict],
    expected_size: int = None,
    validate_content: bool = False,
    verbose: bool = False
) -> Dict[str, Any]:
    """
    Run a single test with layered validation.
    
    Args:
        session: Zenoh session
        character: Character name
        test_name: Test name for logging
        plan: List of actions to execute
        expected_size: Expected Collection size (optional)
        validate_content: Whether to validate content is non-empty (default: False)
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
    start_time = time.time()
    result = execute_plan(session, character, plan, timeout=180.0)
    elapsed = time.time() - start_time
    
    # Basic validation: execution success
    execution_success = result.get('success', False)
    if not execution_success:
        error = result.get('error', 'Unknown error')
        print(f"  ❌ EXECUTION FAILED: {error}")
        return {
            'test_name': test_name,
            'execution_success': False,
            'error': error,
            'size_validated': False,
            'content_validated': False,
            'warnings': [],
            'elapsed_time': elapsed
        }
    
    print(f"  ✅ Execution successful ({elapsed:.2f}s)")
    
    # Size validation
    size_validated = False
    content_validated = False
    warnings = []
    
    # Find the mapped Collection variable (usually "$mapped" or last bound Collection)
    mapped_var = None
    bindings = result.get('bindings', {})
    for var_name in ['$mapped', '$result', '$output']:
        if var_name in bindings:
            mapped_var = var_name
            break
    
    # If not found, try to find any Collection binding
    if not mapped_var:
        for var_name, resource_id in bindings.items():
            if isinstance(resource_id, str) and resource_id.startswith('Collection_'):
                mapped_var = var_name
                break
    
    if mapped_var and expected_size is not None:
        # Get size of mapped Collection
        size_plan = [
            {
                "type": "load",
                "target": bindings[mapped_var],
                "out": "$coll"
            },
            {
                "type": "size",
                "target": "$coll",
                "out": "$size"
            },
            {
                "type": "load",
                "target": "$size",
                "out": "$size_content"
            }
        ]
        size_result = execute_plan(session, character, size_plan, timeout=30.0)
        if size_result.get('success'):
            # Get size from last_action_result (uniform format)
            last_action_result = size_result.get('last_action_result')
            if last_action_result and last_action_result.get('status') == 'success':
                actual_size_str = last_action_result.get('value', '')
            else:
                actual_size_str = ''
            
            try:
                actual_size = int(actual_size_str) if isinstance(actual_size_str, str) else actual_size_str
                if actual_size == expected_size:
                    size_validated = True
                    if verbose:
                        print(f"  ✅ Size matches: {expected_size}")
                else:
                    warnings.append(f"Size mismatch: expected {expected_size}, got {actual_size}")
            except (ValueError, TypeError):
                warnings.append(f"Size validation failed: could not parse '{actual_size_str}'")
        else:
            warnings.append(f"Size check failed: {size_result.get('error', 'Unknown')}")
    
    # Content validation (check first item is non-empty)
    if validate_content and mapped_var:
        valid, resource_id = validate_binding(result, mapped_var)
        if valid:
            # Load Collection and get first item
            content_plan = [
                {
                    "type": "load",
                    "target": resource_id,
                    "out": "$coll"
                },
                {
                    "type": "head",
                    "target": "$coll",
                    "n": 1,
                    "out": "$first_item"
                },
                {
                    "type": "load",
                    "target": "$first_item",
                    "out": "$item_content"
                }
            ]
            content_result = execute_plan(session, character, content_plan, timeout=30.0)
            if content_result.get('success'):
                # Get content from last_action_result (uniform format)
                last_action_result = content_result.get('last_action_result')
                if last_action_result and last_action_result.get('status') == 'success':
                    content = last_action_result.get('value', '')
                else:
                    content = ''
                
                if content and (not isinstance(content, str) or len(content.strip()) > 0):
                    content_validated = True
                    if verbose:
                        print(f"  ✅ Content validated: non-empty result")
                else:
                    warnings.append("Content validation failed: result is empty")
            else:
                warnings.append(f"Content check failed: {content_result.get('error', 'Unknown')}")
    
    # Print warnings
    if warnings:
        for warning in warnings:
            print(f"  ⚠️  {warning}")
    
    return {
        'test_name': test_name,
        'execution_success': execution_success,
        'size_validated': size_validated,
        'content_validated': content_validated,
        'warnings': warnings,
        'elapsed_time': elapsed
    }


# ==========================================
# TEST CASES
# ==========================================

def test_map_refine(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test map with refine tool using instruction parameter."""
    plan = [
        {
            "type": "create-note",
            "value": "hello world",
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": "test message",
            "out": "$note2"
        },
        {
            "type": "create-collection",
            "value": ["$note1", "$note2"],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "refine",
                "instruction": "Convert to uppercase"
            },
            "out": "$mapped"
        }
    ]
    return run_test(
        session, character, "map refine (instruction: uppercase)",
        plan,
        expected_size=2,
        validate_content=True,
        verbose=verbose
    )


def test_map_calculate(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test map with calculate tool using variables parameter."""
    plan = [
        {
            "type": "create-note",
            "value": "x + y",
            "out": "$expr1"
        },
        {
            "type": "create-note",
            "value": "x * y",
            "out": "$expr2"
        },
        {
            "type": "create-collection",
            "value": ["$expr1", "$expr2"],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "calculate",
                "variables": {"x": 5, "y": 3}
            },
            "out": "$mapped"
        }
    ]
    return run_test(
        session, character, "map calculate (variables: x=5, y=3)",
        plan,
        expected_size=2,
        validate_content=True,
        verbose=verbose
    )


def test_map_summarize(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test map with summarize tool using focus parameter."""
    long_text1 = "Cognitive Workbench is a research framework. " * 10
    long_text2 = "Python is a programming language. " * 10
    
    plan = [
        {
            "type": "create-note",
            "value": long_text1.strip(),
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": long_text2.strip(),
            "out": "$note2"
        },
        {
            "type": "create-collection",
            "value": ["$note1", "$note2"],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "summarize",
                "focus": "main topic"
            },
            "out": "$mapped"
        }
    ]
    return run_test(
        session, character, "map summarize (focus: main topic)",
        plan,
        expected_size=2,
        validate_content=True,
        verbose=verbose
    )


def test_map_relate(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test map with relate tool using other parameter."""
    plan = [
        {
            "type": "create-note",
            "value": "Python is a programming language.",
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": "JavaScript is a programming language.",
            "out": "$note2"
        },
        {
            "type": "create-note",
            "value": "Java is a programming language.",
            "out": "$fixed_note"
        },
        {
            "type": "create-collection",
            "value": ["$note1", "$note2"],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "relate",
                "other": "$fixed_note"
            },
            "out": "$mapped"
        }
    ]
    return run_test(
        session, character, "map relate (other: fixed note)",
        plan,
        expected_size=2,
        validate_content=True,
        verbose=verbose
    )


def test_map_query_web(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test map with search-web tool - mapped item becomes value (query string)."""
    plan = [
        {
            "type": "create-note",
            "value": "Python",
            "out": "$query1"
        },
        {
            "type": "create-note",
            "value": "machine learning",
            "out": "$query2"
        },
        {
            "type": "create-collection",
            "value": ["$query1", "$query2"],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "search-web"
                # Mapped item becomes 'value' parameter (query string)
            },
            "out": "$mapped"
        }
    ]
    # Note: search-web may take longer, timeout handled in execute_plan
    return run_test(
        session, character, "map search-web (simple queries)",
        plan,
        expected_size=2,
        validate_content=True,
        verbose=verbose
    )


def test_map_semantic_scholar(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test map with semantic-scholar tool - mapped item becomes value (query string)."""
    plan = [
        {
            "type": "create-note",
            "value": "neural networks",
            "out": "$query1"
        },
        {
            "type": "create-note",
            "value": "transformer architecture",
            "out": "$query2"
        },
        {
            "type": "create-collection",
            "value": ["$query1", "$query2"],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "semantic-scholar"
                # Mapped item becomes 'value' parameter (query string)
            },
            "out": "$mapped"
        }
    ]
    # Note: semantic-scholar may take longer, timeout handled in execute_plan
    return run_test(
        session, character, "map semantic-scholar (simple queries)",
        plan,
        expected_size=2,
        validate_content=True,
        verbose=verbose
    )


# ==========================================
# MAIN
# ==========================================

def main():
    parser = argparse.ArgumentParser(description='Test map operations on tools')
    parser.add_argument('--character', default='Jill', help='Character name (default: Jill)')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--skip-api', action='store_true', help='Skip API-based tests (search-web, semantic-scholar)')
    args = parser.parse_args()
    
    # Initialize Zenoh
    config = zenoh.Config()
    session = zenoh.open(config)
    
    print("="*60)
    print("MAP TOOLS TEST SUITE")
    print("="*60)
    print(f"Character: {args.character}")
    print(f"Timestamp: {datetime.now().isoformat()}")
    print("\nNote: Tests use dict form of tool actions (as planner generates)")
    print("      generate-note excluded: ignores mapped value, inappropriate for map")
    
    # Run tests
    tests = [
        test_map_refine,
        test_map_calculate,
        test_map_summarize,
        test_map_relate,
    ]
    
    if not args.skip_api:
        tests.extend([
            test_map_query_web,
            test_map_semantic_scholar,
        ])
        print("\n⚠️  Note: API tests (search-web, semantic-scholar) may take longer.")
    else:
        print("\n⚠️  Skipping API tests (search-web, semantic-scholar).")
    
    results = []
    total_time = 0.0
    for test_func in tests:
        # Clear transient notes between tests
        clear_transient_notes(session, args.character)
        result = test_func(session, args.character, verbose=args.verbose)
        results.append(result)
        total_time += result.get('elapsed_time', 0.0)
        time.sleep(0.5)  # Small delay between tests
    
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
    
    print(f"\nTotal execution time: {total_time:.2f}s")
    print(f"Average time per test: {total_time/len(results):.2f}s")
    
    session.close()
    
    # Exit code
    exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()

