#!/usr/bin/env python3
"""
Remaining Primitives Test Suite for Cognitive Workbench.

Tests primitives that don't require JSON/SQL-like operations:
- persist
- coerce (to-bool, to-list) - excluding to-json
- split
- flatten
- display
- think
- say
- index
- search-notes
- search-collections
- search-within-collection
- map

Each test:
1. Clears bindings before execution
2. Clears transient notes between tests
3. Tests execution success (basic validation)
4. Attempts content validation where feasible (deeper validation)
5. Logs results with layered success indicators

Usage:
    python tests/test_primitives_remaining.py [--character CHARACTER] [--verbose]
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
                        "out": "$size_loaded"
                    },
                    {
                        "type": "coerce",
                        "target": "$size_loaded",
                        "coercion": "to-int",
                        "out": "$size_int"
                    },
                    {
                        "type": "load",
                        "target": "$size_int",
                        "out": "$final_size"
                    }
                ]
                
                size_result = execute_plan(session, character, size_plan, timeout=30.0)
                if not size_result.get('success'):
                    warnings.append(f"Size validation failed for {var_name}: {size_result.get('error', 'Unknown')}")
                    continue
                
                # Get size value from last_result
                actual_size = size_result.get('last_result')
                if actual_size == expected_size:
                    content_validated = True
                    if verbose:
                        print(f"  ✅ Size matches: {actual_size}")
                else:
                    warnings.append(f"Size mismatch for {var_name}: expected {expected_size}, got {actual_size}")
            
            elif val_type == 'content_match':
                # Validate content matches expected value by loading as last action
                var_name = validation.get('variable')
                expected_value = validation.get('expected_value')
                
                # Get resource ID from bindings
                valid, resource_id = validate_binding(result, var_name)
                if not valid:
                    warnings.append(f"Variable {var_name} not bound for content validation")
                    continue
                
                # Load the resource as last action to get content in last_result
                extract_plan = [
                    {
                        "type": "load",
                        "target": resource_id,
                        "out": "$loaded"
                    }
                ]
                
                extract_result = execute_plan(session, character, extract_plan, timeout=30.0)
                if not extract_result.get('success'):
                    warnings.append(f"Content extraction failed for {var_name}: {extract_result.get('error', 'Unknown')}")
                    continue
                
                # Get content from last_result
                actual_content = extract_result.get('last_result')
                if actual_content == expected_value:
                    content_validated = True
                    if verbose:
                        print(f"  ✅ Content matches: {actual_content}")
                else:
                    warnings.append(f"Content mismatch for {var_name}: expected {expected_value}, got {actual_content}")
            
            elif val_type == 'value_extract':
                # Extract and compare a specific value
                var_name = validation.get('variable')
                expected_value = validation.get('expected_value')
                
                # Get resource ID from bindings
                valid, resource_id = validate_binding(result, var_name)
                if not valid:
                    warnings.append(f"Variable {var_name} not bound for value extraction")
                    continue
                
                # Load the Note to get its content
                extract_plan = [
                    {
                        "type": "load",
                        "target": resource_id,
                        "out": "$loaded"
                    }
                ]
                
                extract_result = execute_plan(session, character, extract_plan, timeout=30.0)
                if not extract_result.get('success'):
                    warnings.append(f"Value extraction failed for {var_name}: {extract_result.get('error', 'Unknown')}")
                    continue
                
                # Get value from last_result
                actual_value = extract_result.get('last_result')
                if actual_value == expected_value:
                    content_validated = True
                    if verbose:
                        print(f"  ✅ Value matches: {actual_value}")
                else:
                    warnings.append(f"Value mismatch for {var_name}: expected {expected_value}, got {actual_value}")
    
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

def test_persist(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test marking a Note as persistent."""
    plan = [
        {
            "type": "create-note",
            "value": "Persistent content",
            "out": "$note"
        },
        {
            "type": "persist",
            "target": "$note"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "note",
            "expected_prefix": "Note_"
        }
    ]
    
    return run_test(session, character, "persist", plan, validations, verbose)


def test_coerce_to_bool(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test coerce to-bool."""
    plan = [
        {
            "type": "create-note",
            "value": "true",
            "out": "$str_note"
        },
        {
            "type": "coerce",
            "target": "$str_note",
            "coercion": "to-bool",
            "out": "$bool_note"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "bool_note",
            "expected_prefix": "Note_"
        },
        {
            "type": "value_extract",
            "variable": "bool_note",
            "expected_value": True
        }
    ]
    
    return run_test(session, character, "coerce (to-bool)", plan, validations, verbose)


def test_coerce_to_list(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test coerce to-list."""
    plan = [
        {
            "type": "create-note",
            "value": "apple,banana,cherry",
            "out": "$str_note"
        },
        {
            "type": "coerce",
            "target": "$str_note",
            "coercion": "to-list",
            "delimiter": ",",
            "out": "$list_note"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "list_note",
            "expected_prefix": "Note_"
        }
    ]
    
    return run_test(session, character, "coerce (to-list)", plan, validations, verbose)


def test_split_json_array(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test split with JSON array."""
    plan = [
        {
            "type": "create-note",
            "value": ["item1", "item2", "item3"],
            "out": "$array_note"
        },
        {
            "type": "split",
            "target": "$array_note",
            "out": "$split_coll"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "split_coll",
            "expected_prefix": "Collection_"
        },
        {
            "type": "size_match",
            "variable": "split_coll",
            "expected_size": 3
        }
    ]
    
    return run_test(session, character, "split (JSON array)", plan, validations, verbose)


def test_split_text_sentence(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test split with text (sentence delimiter)."""
    plan = [
        {
            "type": "create-note",
            "value": "First sentence. Second sentence! Third sentence?",
            "out": "$text_note"
        },
        {
            "type": "split",
            "target": "$text_note",
            "delimiter": "sentence",
            "out": "$split_coll"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "split_coll",
            "expected_prefix": "Collection_"
        },
        {
            "type": "size_match",
            "variable": "split_coll",
            "expected_size": 3
        }
    ]
    
    return run_test(session, character, "split (text, sentence)", plan, validations, verbose)


def test_flatten(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test flatten Collection to single Note."""
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
            "type": "create-collection",
            "value": ["$item1", "$item2"],
            "out": "$coll"
        },
        {
            "type": "flatten",
            "target": "$coll",
            "out": "$flattened_note"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "flattened_note",
            "expected_prefix": "Note_"
        }
    ]
    
    return run_test(session, character, "flatten", plan, validations, verbose)


def test_display(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test display primitive (side-effect only, validates execution)."""
    plan = [
        {
            "type": "create-note",
            "value": "Display test content",
            "out": "$note"
        },
        {
            "type": "display",
            "target": "$note"
        }
    ]
    
    # Display is side-effect only, just validate execution
    return run_test(session, character, "display", plan, None, verbose)


def test_think(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test think primitive (now returns value)."""
    plan = [
        {
            "type": "think",
            "value": "This is a test thought",
            "out": "$thought"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "thought",
            "expected_prefix": "Note_"
        }
    ]
    
    return run_test(session, character, "think", plan, validations, verbose)


def test_say(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test say primitive (now returns value)."""
    plan = [
        {
            "type": "say",
            "value": "Hello, this is a test"
        }
    ]
    
    # Say returns the text value, but we can't easily validate it without checking last_result
    # Just validate execution success
    return run_test(session, character, "say", plan, None, verbose)


def test_index(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test index primitive (build embedding index)."""
    plan = [
        {
            "type": "create-note",
            "value": "First document about machine learning",
            "out": "$doc1"
        },
        {
            "type": "create-note",
            "value": "Second document about neural networks",
            "out": "$doc2"
        },
        {
            "type": "create-collection",
            "value": ["$doc1", "$doc2"],
            "out": "$coll"
        },
        {
            "type": "index",
            "source": "$coll"
        }
    ]
    
    # Index is side-effect (creates index), validate execution
    return run_test(session, character, "index", plan, None, verbose)


def test_search_notes(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test search-notes primitive."""
    plan = [
        {
            "type": "create-note",
            "value": "This is a test document about artificial intelligence",
            "out": "$test_note"
        },
        {
            "type": "search-notes",
            "value": "artificial intelligence",
            "out": "$results"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "results",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "search-notes", plan, validations, verbose)


def test_search_collections(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test search-collections primitive."""
    plan = [
        {
            "type": "create-note",
            "value": "Document 1",
            "out": "$doc1"
        },
        {
            "type": "create-collection",
            "value": ["$doc1"],
            "name": "test-collection",
            "out": "$coll"
        },
        {
            "type": "search-collections",
            "value": "test",
            "out": "$results"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "results",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "search-collections", plan, validations, verbose)


def test_search_within_collection(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test search-within-collection primitive."""
    plan = [
        {
            "type": "create-note",
            "value": "First document about machine learning",
            "out": "$doc1"
        },
        {
            "type": "create-note",
            "value": "Second document about neural networks",
            "out": "$doc2"
        },
        {
            "type": "create-collection",
            "value": ["$doc1", "$doc2"],
            "out": "$coll"
        },
        {
            "type": "index",
            "source": "$coll"
        },
        {
            "type": "search-within-collection",
            "target": "$coll",
            "value": "machine learning",
            "out": "$results"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "results",
            "expected_prefix": "Collection_"
        }
    ]
    
    return run_test(session, character, "search-within-collection", plan, validations, verbose)


def test_map(session: zenoh.Session, character: str, verbose: bool = False) -> Dict:
    """Test map primitive (apply operation to each Collection item)."""
    # Use think as the operation - it's simpler and definitely works with map
    plan = [
        {
            "type": "create-note",
            "value": "hello",
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": "world",
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
            "out": "$mapped_coll"
        }
    ]
    
    validations = [
        {
            "type": "resource_id",
            "variable": "mapped_coll",
            "expected_prefix": "Collection_"
        },
        {
            "type": "size_match",
            "variable": "mapped_coll",
            "expected_size": 0  # think creates empty Collection (side-effects only)
        }
    ]
    
    return run_test(session, character, "map", plan, validations, verbose)


# ==========================================
# MAIN EXECUTION
# ==========================================

def main():
    parser = argparse.ArgumentParser(description='Remaining Primitives Test Suite')
    parser.add_argument('--character', type=str, default='Jill', help='Character name (default: Jill)')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--skip-interactive', action='store_true', help='Skip tests that require user interaction')
    args = parser.parse_args()
    
    print(f"Remaining Primitives Test Suite")
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
            test_persist,
            test_coerce_to_bool,
            test_coerce_to_list,
            test_split_json_array,
            test_split_text_sentence,
            test_flatten,
            test_display,
            test_think,
            test_say,
            test_index,
            test_search_notes,
            test_search_collections,
            test_search_within_collection,
            test_map,
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

