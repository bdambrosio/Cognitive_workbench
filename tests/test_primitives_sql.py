#!/usr/bin/env python3
"""
SQL-like Primitives Test Suite for Cognitive Workbench.

Tests SQL-like Collection operations that require JSON/dict Notes:
- project
- pluck
- filter-structured
- sort
- head
- join
- coerce (to-json)

Each test:
1. Clears bindings before execution
2. Clears transient notes between tests
3. Tests execution success (basic validation)
4. Attempts content validation where feasible (deeper validation)
5. Logs results with layered success indicators

Usage:
    python tests/test_primitives_sql.py [--character CHARACTER] [--verbose]
"""

import argparse
import json
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

def validate_resource_id(resource_id: str, expected_prefix: str) -> Tuple[bool, str]:
    """Validate that resource ID has correct format."""
    if not isinstance(resource_id, str):
        return False, f"Resource ID is not a string: {type(resource_id).__name__}"
    if not resource_id.startswith(expected_prefix):
        return False, f"Resource ID does not start with {expected_prefix}: {resource_id}"
    return True, ""


def validate_binding(result: dict, var_name: str) -> Tuple[bool, str]:
    """Validate that variable is bound in result."""
    bindings = result.get('bindings', {})
    # Try both with and without $ prefix
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
    validations: List[Dict] = None,
    verbose: bool = False
) -> Dict[str, Any]:
    """
    Run a single test with layered validation.
    
    Args:
        session: Zenoh session
        character: Character name
        test_name: Test name for logging
        plan: List of actions to execute
        validations: List of validation steps (optional)
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
    
    # Debug: show bindings if verbose
    if verbose:
        bindings = result.get('bindings', {})
        if bindings:
            print(f"  Available bindings: {list(bindings.keys())[:10]}")
    
    # Deeper validation if provided
    content_validated = False
    warnings = []
    
    if validations:
        for validation in validations:
            val_type = validation.get('type')
            
            if val_type == 'resource_id':
                # Validate resource ID format
                var_name = validation.get('variable')
                expected_prefix = validation.get('expected_prefix')
                valid, resource_id = validate_binding(result, var_name)
                if not valid:
                    if verbose:
                        bindings = result.get('bindings', {})
                        print(f"  Debug: Looking for '{var_name}', available: {list(bindings.keys())}")
                    warnings.append(f"Variable {var_name} not bound")
                    continue
                valid, msg = validate_resource_id(resource_id, expected_prefix)
                if not valid:
                    warnings.append(f"Resource ID validation failed: {msg}")
                else:
                    content_validated = True
                    if verbose:
                        print(f"  ✅ Resource ID valid: {resource_id}")
            
            elif val_type == 'size_match':
                # Validate Collection size by getting size as last action
                var_name = validation.get('variable')
                expected_size = validation.get('expected_size')
                
                # Get resource ID from bindings
                valid, resource_id = validate_binding(result, var_name)
                if not valid:
                    warnings.append(f"Variable {var_name} not bound for size validation")
                    continue
                
                size_plan = [
                    {
                        "type": "load",
                        "target": resource_id,
                        "out": "$coll"
                    },
                    {
                        "type": "size",
                        "target": "$coll",
                        "out": "$size_note"
                    },
                    {
                        "type": "load",
                        "target": "$size_note",
                        "out": "$size_content"
                    },
                    {
                        "type": "coerce",
                        "target": "$size_content",
                        "coercion": "to-int",
                        "out": "$size_int_note"
                    },
                    {
                        "type": "load",
                        "target": "$size_int_note",
                        "out": "$final_size"
                    }
                ]
                
                size_result = execute_plan(session, character, size_plan, timeout=30.0)
                if not size_result.get('success'):
                    warnings.append(f"Size validation failed for {var_name}: {size_result.get('error', 'Unknown')}")
                    continue
                
                # Get size value from last_action_result (uniform format)
                last_action_result = size_result.get('last_action_result')
                if last_action_result and last_action_result.get('status') == 'success':
                    size_value = last_action_result.get('value')
                    # Try to convert to int if it's a string
                    if isinstance(size_value, str):
                        try:
                            actual_size = int(size_value)
                        except ValueError:
                            actual_size = size_value
                    else:
                        actual_size = size_value
                    
                    if actual_size == expected_size:
                        content_validated = True
                        if verbose:
                            print(f"  ✅ Size matches: {actual_size}")
                    else:
                        warnings.append(f"Size mismatch for {var_name}: expected {expected_size}, got {actual_size}")
                else:
                    warnings.append(f"Failed to get size from last_action_result for {var_name}")
    
    # Report warnings
    if warnings:
        for warning in warnings:
            print(f"  ⚠️  {warning}")
    elif content_validated:
        print(f"  ✅ Content validated")
    else:
        print(f"  ℹ️  Semantic validation not performed (insufficient info)")
    
    return {
        'test_name': test_name,
        'execution_success': True,
        'content_validated': content_validated,
        'warnings': warnings
    }


# ==========================================
# TEST CASES
# ==========================================

def test_project(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test project primitive (extract fields)."""
    plan = [
        {
            "type": "create-note",
            "value": {"name": "Alice", "age": 30, "city": "NYC"},
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": {"name": "Bob", "age": 25, "city": "LA"},
            "out": "$note2"
        },
        {
            "type": "create-collection",
            "value": ["$note1", "$note2"],
            "out": "$coll"
        },
        {
            "type": "project",
            "target": "$coll",
            "fields": ["name", "age"],
            "out": "$projected"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "projected",
            "expected_prefix": "Collection_"
        },
        {
            "type": "size_match",
            "variable": "projected",
            "expected_size": 2
        }
    ]
    
    return run_test(session, character, "project", plan, validations, verbose)


def test_pluck(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test pluck primitive (extract single field)."""
    plan = [
        {
            "type": "create-note",
            "value": {"name": "Alice", "age": 30},
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": {"name": "Bob", "age": 25},
            "out": "$note2"
        },
        {
            "type": "create-collection",
            "value": ["$note1", "$note2"],
            "out": "$coll"
        },
        {
            "type": "pluck",
            "target": "$coll",
            "field": "name",
            "out": "$names"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "names",
            "expected_prefix": "Collection_"
        },
        {
            "type": "size_match",
            "variable": "names",
            "expected_size": 2
        }
    ]
    
    return run_test(session, character, "pluck", plan, validations, verbose)


def test_filter_structured(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test filter-structured primitive."""
    plan = [
        {
            "type": "create-note",
            "value": {"name": "Alice", "age": 30},
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": {"name": "Bob", "age": 25},
            "out": "$note2"
        },
        {
            "type": "create-note",
            "value": {"name": "Charlie", "age": 35},
            "out": "$note3"
        },
        {
            "type": "create-collection",
            "value": ["$note1", "$note2", "$note3"],
            "out": "$coll"
        },
        {
            "type": "filter-structured",
            "target": "$coll",
            "where": "age >= 30",
            "out": "$filtered"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "filtered",
            "expected_prefix": "Collection_"
        },
        {
            "type": "size_match",
            "variable": "filtered",
            "expected_size": 2  # Alice (30) and Charlie (35)
        }
    ]
    
    return run_test(session, character, "filter-structured", plan, validations, verbose)


def test_sort(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test sort primitive."""
    plan = [
        {
            "type": "create-note",
            "value": {"name": "Alice", "age": 30},
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": {"name": "Bob", "age": 25},
            "out": "$note2"
        },
        {
            "type": "create-note",
            "value": {"name": "Charlie", "age": 35},
            "out": "$note3"
        },
        {
            "type": "create-collection",
            "value": ["$note1", "$note2", "$note3"],
            "out": "$coll"
        },
        {
            "type": "sort",
            "target": "$coll",
            "by": "age",
            "order": "asc",
            "out": "$sorted"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "sorted",
            "expected_prefix": "Collection_"
        },
        {
            "type": "size_match",
            "variable": "sorted",
            "expected_size": 3
        }
    ]
    
    return run_test(session, character, "sort", plan, validations, verbose)


def test_head(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test head primitive."""
    plan = [
        {
            "type": "create-note",
            "value": "Item 1",
            "out": "$item1"
        },
        {
            "type": "create-note",
            "value": "Item 2",
            "out": "$item2"
        },
        {
            "type": "create-note",
            "value": "Item 3",
            "out": "$item3"
        },
        {
            "type": "create-collection",
            "value": ["$item1", "$item2", "$item3"],
            "out": "$coll"
        },
        {
            "type": "head",
            "target": "$coll",
            "count": 2,
            "out": "$head_result"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "head_result",
            "expected_prefix": "Collection_"
        },
        {
            "type": "size_match",
            "variable": "head_result",
            "expected_size": 2
        }
    ]
    
    return run_test(session, character, "head", plan, validations, verbose)


def test_join(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test join primitive."""
    plan = [
        {
            "type": "create-note",
            "value": {"id": 1, "name": "Alice"},
            "out": "$user1"
        },
        {
            "type": "create-note",
            "value": {"id": 2, "name": "Bob"},
            "out": "$user2"
        },
        {
            "type": "create-collection",
            "value": ["$user1", "$user2"],
            "out": "$users"
        },
        {
            "type": "create-note",
            "value": {"user_id": 1, "email": "alice@example.com"},
            "out": "$profile1"
        },
        {
            "type": "create-note",
            "value": {"user_id": 2, "email": "bob@example.com"},
            "out": "$profile2"
        },
        {
            "type": "create-collection",
            "value": ["$profile1", "$profile2"],
            "out": "$profiles"
        },
        {
            "type": "join",
            "target": "$users",
            "value": "$profiles",
            "left_key": "id",
            "right_key": "user_id",
            "out": "$joined"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "joined",
            "expected_prefix": "Collection_"
        },
        {
            "type": "size_match",
            "variable": "joined",
            "expected_size": 2  # Both users should match
        }
    ]
    
    return run_test(session, character, "join", plan, validations, verbose)


def test_coerce_to_json(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test coerce to-json."""
    plan = [
        {
            "type": "create-note",
            "value": '{"key": "value", "number": 42}',
            "out": "$json_string_note"
        },
        {
            "type": "coerce",
            "target": "$json_string_note",
            "coercion": "to-json",
            "out": "$parsed_note"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "parsed_note",
            "expected_prefix": "Note_"
        }
    ]
    
    return run_test(session, character, "coerce (to-json)", plan, validations, verbose)


# ==========================================
# MAIN EXECUTION
# ==========================================

def main():
    parser = argparse.ArgumentParser(description='SQL-like Primitives Test Suite')
    parser.add_argument('--character', type=str, default='Jill', help='Character name (default: Jill)')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    args = parser.parse_args()
    
    print(f"SQL-like Primitives Test Suite")
    print(f"Character: {args.character}")
    print(f"Timestamp: {datetime.now().isoformat()}")
    
    config = zenoh.Config()
    session = zenoh.open(config)
    
    try:
        # Clear transient notes before starting
        cleared = clear_transient_notes(session, args.character)
        if args.verbose:
            print(f"Cleared {cleared} transient resources before starting")
        
        # Run all tests
        tests = [
            test_project,
            test_pluck,
            test_filter_structured,
            test_sort,
            test_head,
            test_join,
            test_coerce_to_json,
        ]
        
        results = []
        for test_func in tests:
            # Clear transient notes between tests
            cleared = clear_transient_notes(session, args.character)
            if args.verbose and cleared > 0:
                print(f"  Cleared {cleared} transient resources")
            
            result = test_func(session, args.character, args.verbose)
            results.append(result)
        
        # Summary
        print(f"\n{'='*60}")
        print("TEST SUMMARY")
        print(f"{'='*60}")
        
        passed = sum(1 for r in results if r.get('execution_success'))
        failed = len(results) - passed
        validated = sum(1 for r in results if r.get('content_validated'))
        
        # Count non-fatal problems (warnings)
        tests_with_warnings = [r for r in results if r.get('execution_success') and r.get('warnings')]
        total_warnings = sum(len(r.get('warnings', [])) for r in tests_with_warnings)
        
        print(f"Total tests: {len(results)}")
        print(f"✅ Passed (execution): {passed}")
        print(f"❌ Failed (execution): {failed}")
        print(f"✅ Content validated: {validated}")
        print(f"⚠️  Non-fatal problems: {total_warnings} warning(s) in {len(tests_with_warnings)} test(s)")
        
        if failed > 0:
            print(f"\nFailed tests:")
            for r in results:
                if not r.get('execution_success'):
                    print(f"  - {r.get('test_name')}: {r.get('error', 'Unknown error')}")
        
        if tests_with_warnings:
            print(f"\nTests with warnings:")
            for r in tests_with_warnings:
                print(f"  - {r.get('test_name')}:")
                for warning in r.get('warnings', []):
                    print(f"    • {warning}")
        
        return 0 if failed == 0 else 1
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        session.close()


if __name__ == "__main__":
    exit(main())

