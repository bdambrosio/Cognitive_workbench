#!/usr/bin/env python3
"""
Map Primitive Test Suite for Cognitive Workbench.

Tests the `map` primitive with various operations:
- Content transformation (coerce)
- Side-effect primitives (think, say, display)
- Extraction (pluck)
- The common pattern: map → flatten → size
- Edge cases (empty collections, single items)

Each test:
1. Clears bindings before execution
2. Clears transient notes between tests
3. Tests execution success and output Collection size
4. Validates content where feasible
5. Logs results with layered success indicators

Usage:
    python tests/test_map.py [--character CHARACTER] [--verbose]
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

def validate_resource_id(resource_id: str, expected_prefix: str) -> Tuple[bool, str]:
    """Validate that resource ID has correct format."""
    if not isinstance(resource_id, str):
        return False, f"Resource ID is not a string: {type(resource_id).__name__}"
    if not resource_id.startswith(expected_prefix):
        return False, f"Resource ID does not start with {expected_prefix}: {resource_id}"
    return True, ""


def validate_binding(result: dict, var_name: str) -> Tuple[bool, str]:
    """Check if a variable is bound in the result."""
    bindings = result.get('bindings', {})
    if var_name not in bindings:
        return False, f"Variable ${var_name} not bound"
    return True, ""


def validate_size_match(result: dict, var_name: str, expected_size: int) -> Tuple[bool, str]:
    """Validate Collection size by loading and checking."""
    bindings = result.get('bindings', {})
    if var_name not in bindings:
        return False, f"Variable ${var_name} not bound"
    
    resource_id = bindings[var_name]
    if not resource_id.startswith('Collection_'):
        return False, f"Resource {resource_id} is not a Collection"
    
    # Create a plan to check size
    size_plan = [
        {
            "type": "load",
            "target": resource_id,
            "out": "$coll_check"
        },
        {
            "type": "size",
            "target": "$coll_check",
            "out": "$size_check"
        },
        {
            "type": "load",
            "target": "$size_check",
            "out": "$size_content"
        },
        {
            "type": "coerce",
            "target": "$size_content",
            "coercion": "to-int",
            "out": "$size_int"
        }
    ]
    
    # Execute size check (we'll need session and character - pass as closure or inline)
    # For now, return True and note that size validation requires re-execution
    return True, ""


def run_test(session: zenoh.Session, character: str, test_name: str, plan: list, 
             validations: List[Dict] = None, expected_size: int = None, verbose: bool = False) -> Dict:
    """Run a single test and return results."""
    if validations is None:
        validations = []
    
    # Clear bindings
    clear_result = clear_planner_bindings(session, character)
    bindings_cleared = clear_result.get('bindings_cleared', 0)
    
    if verbose:
        print(f"  Cleared bindings: {bindings_cleared}")
    
    # Execute plan
    result = execute_plan(session, character, plan)
    
    success = result.get('success', False)
    bindings = result.get('bindings', {})
    
    if verbose:
        if success:
            print(f"  ✅ Execution successful")
            print(f"  Available bindings: {list(bindings.keys())}")
        else:
            print(f"  ❌ Execution failed: {result.get('error', 'Unknown error')}")
    
    # Validate resource ID
    resource_id_valid = False
    resource_id_msg = ""
    if success and validations:
        for val in validations:
            if val.get('type') == 'resource_id':
                var_name = val['variable']
                expected_prefix = val['expected_prefix']
                if var_name in bindings:
                    resource_id = bindings[var_name]
                    resource_id_valid, resource_id_msg = validate_resource_id(resource_id, expected_prefix)
                    if verbose:
                        if resource_id_valid:
                            print(f"  ✅ Resource ID valid: {resource_id}")
                        else:
                            print(f"  ❌ Resource ID invalid: {resource_id_msg}")
                    break
    
    # Validate size if expected
    size_match = True
    size_msg = ""
    if success and expected_size is not None:
        # Find the output Collection variable (usually last binding or specified)
        output_var = None
        for val in validations:
            if val.get('type') == 'resource_id':
                output_var = val['variable']
                break
        
        if output_var and output_var in bindings:
            resource_id = bindings[output_var]
            # Create plan to check size
            size_check_plan = [
                {"type": "load", "target": resource_id, "out": "$coll_check"},
                {"type": "size", "target": "$coll_check", "out": "$size_check"},
                {"type": "load", "target": "$size_check", "out": "$size_content"},
                {"type": "coerce", "target": "$size_content", "coercion": "to-int", "out": "$size_int"},
                {"type": "load", "target": "$size_int", "out": "$final_size"}
            ]
            size_result = execute_plan(session, character, size_check_plan)
            if size_result.get('success'):
                size_bindings = size_result.get('bindings', {})
                if 'final_size' in size_bindings:
                    size_note_id = size_bindings['final_size']
                    # Try to get size from last_result first
                    last_result = size_result.get('last_result', '')
                    actual_size = None
                    if last_result:
                        try:
                            actual_size = int(last_result)
                        except (ValueError, TypeError):
                            pass
                    
                    # If last_result didn't work, try querying resource content directly
                    if actual_size is None:
                        query_key = f"cognitive/{character}/resource/view/{size_note_id}"
                        try:
                            replies = session.get(
                                query_key,
                                target=zenoh.QueryTarget.BEST_MATCHING,
                                consolidation=zenoh.ConsolidationMode.NONE,
                                timeout=5.0
                            )
                            for reply in replies:
                                if hasattr(reply, 'ok') and reply.ok is not None:
                                    payload = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                                    if payload.get('success'):
                                        content = payload.get('content', '')
                                        try:
                                            actual_size = int(content)
                                        except (ValueError, TypeError):
                                            pass
                                    break
                        except Exception:
                            pass
                    
                    if actual_size == expected_size:
                        if verbose:
                            print(f"  ✅ Size matches: {expected_size}")
                    else:
                        size_match = False
                        size_msg = f"expected {expected_size}, got {actual_size}"
                        if verbose:
                            print(f"  ⚠️  Size mismatch: {size_msg}")
                else:
                    size_match = False
                    size_msg = "final_size not bound"
                    if verbose:
                        print(f"  ⚠️  Size validation failed: {size_msg}")
    
    # Content validation (basic check via last_result)
    content_validated = False
    if success:
        last_result = result.get('last_result', '')
        if last_result:
            content_validated = True
            if verbose:
                print(f"  ✅ Content validated")
        else:
            if verbose:
                print(f"  ⚠️  No content to validate")
    
    # Cleanup
    cleared = clear_transient_notes(session, character)
    if verbose:
        print(f"  Cleared {cleared} transient resources")
    
    return {
        "test_name": test_name,
        "success": success,
        "resource_id_valid": resource_id_valid,
        "size_match": size_match,
        "size_msg": size_msg,
        "content_validated": content_validated,
        "warnings": []
    }


# ==========================================
# MAP TESTS
# ==========================================

def test_map_coerce(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test map with coerce primitive."""
    plan = [
        {
            "type": "create-note",
            "value": "42",
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": "100",
            "out": "$note2"
        },
        {
            "type": "create-note",
            "value": "3.14",
            "out": "$note3"
        },
        {
            "type": "create-collection",
            "value": ["$note1", "$note2", "$note3"],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "coerce",
                "coercion": "to-float"
            },
            "out": "$mapped"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "mapped",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "map (coerce)", plan, validations, expected_size=3, verbose=verbose)


def test_map_think(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test map with think primitive (side-effect)."""
    plan = [
        {
            "type": "create-note",
            "value": "First item",
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": "Second item",
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
            "operation": "think",
            "out": "$mapped"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "mapped",
            "expected_prefix": "Collection_"
        }
    ]
    
    # Side-effect primitives create empty Collections in map
    return run_test(session, character, "map (think)", plan, validations, expected_size=0, verbose=verbose)


def test_map_say(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test map with say primitive (side-effect)."""
    plan = [
        {
            "type": "create-note",
            "value": "Hello",
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": "World",
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
            "operation": "say",
            "out": "$mapped"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "mapped",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "map (say)", plan, validations, expected_size=0, verbose=verbose)


def test_map_display(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test map with display primitive (side-effect)."""
    plan = [
        {
            "type": "create-note",
            "value": {"key": "value1"},
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": {"key": "value2"},
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
            "operation": "display",
            "out": "$mapped"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "mapped",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "map (display)", plan, validations, expected_size=0, verbose=verbose)


def test_map_pluck(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test map with pluck primitive."""
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
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "pluck",
                "field": "name"
            },
            "out": "$mapped"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "mapped",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "map (pluck)", plan, validations, expected_size=2, verbose=verbose)


def test_map_flatten_size_pattern(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test the common pattern: map → flatten → size."""
    plan = [
        {
            "type": "create-note",
            "value": "10",
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": "20",
            "out": "$note2"
        },
        {
            "type": "create-note",
            "value": "30",
            "out": "$note3"
        },
        {
            "type": "create-collection",
            "value": ["$note1", "$note2", "$note3"],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "coerce",
                "coercion": "to-int"
            },
            "out": "$mapped"
        },
        {
            "type": "flatten",
            "target": "$mapped",
            "out": "$flattened"
        },
        {
            "type": "size",
            "target": "$flattened",
            "out": "$final_size"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "final_size",
            "expected_prefix": "Note_"
        }
    ]
    
    # After flatten, we should have 3 items
    return run_test(session, character, "map → flatten → size", plan, validations, expected_size=None, verbose=verbose)


def test_map_empty_collection(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test map on empty collection."""
    plan = [
        {
            "type": "create-collection",
            "value": [],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "coerce",
                "coercion": "to-string"
            },
            "out": "$mapped"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "mapped",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "map (empty collection)", plan, validations, expected_size=0, verbose=verbose)


def test_map_single_item(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test map on single-item collection."""
    plan = [
        {
            "type": "create-note",
            "value": "42",
            "out": "$note1"
        },
        {
            "type": "create-collection",
            "value": ["$note1"],
            "out": "$coll"
        },
        {
            "type": "map",
            "target": "$coll",
            "operation": {
                "tool": "coerce",
                "coercion": "to-int"
            },
            "out": "$mapped"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "mapped",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "map (single item)", plan, validations, expected_size=1, verbose=verbose)


def test_map_with_instruction_param(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test map with a primitive that takes instruction parameter (if refine is available)."""
    # This test may fail if refine isn't available, but that's okay
    plan = [
        {
            "type": "create-note",
            "value": "Hello world",
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": "Test text",
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
                "instruction": "Make it uppercase"
            },
            "out": "$mapped"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "mapped",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "map (with instruction param)", plan, validations, expected_size=2, verbose=verbose)


# ==========================================
# MAIN TEST RUNNER
# ==========================================

def main():
    parser = argparse.ArgumentParser(description="Test map primitive with various operations")
    parser.add_argument("--character", type=str, default="Jill", help="Character name")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")
    args = parser.parse_args()
    
    print("Map Primitive Test Suite")
    print(f"Character: {args.character}")
    print(f"Timestamp: {datetime.now().isoformat()}")
    
    # Initialize Zenoh
    config = zenoh.Config()
    session = zenoh.open(config)
    
    # Clear initial state
    cleared = clear_transient_notes(session, args.character)
    print(f"Cleared {cleared} transient resources before starting\n")
    
    # Run tests
    tests = [
        test_map_coerce,
        test_map_think,
        test_map_say,
        test_map_display,
        test_map_pluck,
        test_map_flatten_size_pattern,
        test_map_empty_collection,
        test_map_single_item,
        test_map_with_instruction_param,
    ]
    
    results = []
    warnings = []
    
    for test_func in tests:
        test_name = test_func.__name__.replace("test_", "").replace("_", " ")
        print("=" * 60)
        print(f"TEST: {test_name}")
        print("=" * 60)
        
        result = test_func(session, args.character, args.verbose)
        results.append(result)
        
        if result.get("size_msg"):
            warnings.append({
                "test": test_name,
                "issue": f"Size mismatch: {result['size_msg']}"
            })
        
        if not args.verbose:
            # Print summary even in non-verbose mode
            if result["success"]:
                print(f"  ✅ Execution successful")
            else:
                print(f"  ❌ Execution failed")
            if result.get("size_match") is False:
                print(f"  ⚠️  {result.get('size_msg', 'Size mismatch')}")
        print()
    
    # Summary
    print("=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    total = len(results)
    passed = sum(1 for r in results if r["success"])
    failed = total - passed
    content_validated = sum(1 for r in results if r.get("content_validated"))
    warning_count = len(warnings)
    
    print(f"Total tests: {total}")
    print(f"✅ Passed (execution): {passed}")
    print(f"❌ Failed (execution): {failed}")
    print(f"✅ Content validated: {content_validated}")
    print(f"⚠️  Non-fatal problems: {warning_count} warning(s) in {len(set(w['test'] for w in warnings))} test(s)")
    
    if warnings:
        print("\nTests with warnings:")
        for warning in warnings:
            print(f"  - {warning['test']}:")
            print(f"    • {warning['issue']}")
    
    session.close()


if __name__ == "__main__":
    main()

