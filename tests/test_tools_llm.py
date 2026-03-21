#!/usr/bin/env python3
"""
LLM Tools Test Suite for Cognitive Workbench.

Tests LLM-based tools that require model inference:
- extract - Text transformation/extraction
- summarize - Text summarization
- generate-note - Content generation
- extract-entities - Named entity extraction
- assess - Quality/relevance assessment
- synthesize (comparison) - Relationship analysis

Each test:
1. Clears bindings before execution
2. Clears transient notes between tests
3. Tests execution success and basic semantic validation
4. Logs results with layered success indicators

Note: These tests are slower and require LLM inference.

Usage:
    python tests/test_tools_llm.py [--character CHARACTER] [--verbose]
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

def execute_plan(session: zenoh.Session, character: str, plan: list, timeout: float = 120.0) -> dict:
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


def validate_content_non_empty(result: dict, var_name: str, session: zenoh.Session, character: str, verbose: bool = False) -> Tuple[bool, str]:
    """Validate that content exists and is non-empty."""
    valid, resource_id = validate_binding(result, var_name)
    if not valid:
        return False, f"Variable {var_name} not bound"
    
    load_plan = [
        {
            "type": "load",
            "target": resource_id,
            "out": "$loaded"
        }
    ]
    load_result = execute_plan(session, character, load_plan, timeout=30.0)
    if not load_result.get('success'):
        return False, f"Failed to load content for validation"
    
    # Get content from last_action_result (uniform format)
    last_action_result = load_result.get('last_action_result')
    if last_action_result and last_action_result.get('status') == 'success':
        content = last_action_result.get('value')
    else:
        content = None
    
    if content is None or (isinstance(content, str) and len(content.strip()) == 0):
        return False, f"Content is empty"
    
    if verbose:
        print(f"  ✅ Content is non-empty: {len(str(content))} chars")
    
    return True, ""


def run_test(
    session: zenoh.Session,
    character: str,
    test_name: str,
    plan: List[Dict],
    expected_output_var: str = None,
    validate_content: bool = True,
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
        validate_content: Whether to validate content is non-empty (default: True)
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
    result = execute_plan(session, character, plan, timeout=120.0)
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
            'content_validated': False,
            'warnings': [],
            'elapsed_time': elapsed
        }
    
    print(f"  ✅ Execution successful ({elapsed:.2f}s)")
    
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
            
            if validate_content:
                valid, msg = validate_content_non_empty(result, expected_output_var, session, character, verbose)
                if valid:
                    content_validated = True
                else:
                    warnings.append(f"Content validation failed: {msg}")
    
    # Print warnings
    if warnings:
        for warning in warnings:
            print(f"  ⚠️  {warning}")
    
    return {
        'test_name': test_name,
        'execution_success': execution_success,
        'content_validated': content_validated,
        'warnings': warnings,
        'elapsed_time': elapsed
    }


# ==========================================
# TEST CASES
# ==========================================

def test_extract_transform(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test extract tool for text transformation."""
    plan = [
        {
            "type": "create-note",
            "value": "The quick brown fox jumps over the lazy dog.",
            "out": "$input_text"
        },
        {
            "type": "extract",
            "target": "$input_text",
            "instruction": "Convert to uppercase",
            "out": "$extracted"
        }
    ]
    return run_test(
        session, character, "extract (uppercase)",
        plan,
        expected_output_var="$extracted",
        validate_content=True,
        verbose=verbose
    )


def test_summarize(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test summarize tool."""
    long_text = """
    Cognitive Workbench is a research framework for building autonomous agents with persistent memory, 
    goal-directed planning, and tool use. It provides a virtual machine paradigm for agent execution, 
    contrasting with the chat loop approach of other frameworks. The system uses SGLang for efficient 
    LLM control flow and structured output, and Zenoh for inter-node communication. Agents operate in 
    information spaces with semantic operations over notes, collections, and web content.
    """ * 3
    
    plan = [
        {
            "type": "create-note",
            "value": long_text.strip(),
            "out": "$input_text"
        },
        {
            "type": "summarize",
            "target": "$input_text",
            "out": "$summary"
        }
    ]
    return run_test(
        session, character, "summarize (long text)",
        plan,
        expected_output_var="$summary",
        validate_content=True,
        verbose=verbose
    )


def test_generate_note(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test generate-note tool."""
    plan = [
        {
            "type": "generate-note",
            "instruction": "Write a haiku about programming",
            "out": "$generated"
        }
    ]
    return run_test(
        session, character, "generate-note (haiku)",
        plan,
        expected_output_var="$generated",
        validate_content=True,
        verbose=verbose
    )


def test_extract_entities(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test extract-entities tool."""
    plan = [
        {
            "type": "create-note",
            "value": "Apple Inc. was founded by Steve Jobs in Cupertino, California in 1976.",
            "out": "$input_text"
        },
        {
            "type": "extract-entities",
            "target": "$input_text",
            "out": "$entities"
        }
    ]
    return run_test(
        session, character, "extract-entities (person, organization, location)",
        plan,
        expected_output_var="$entities",
        validate_content=True,
        verbose=verbose
    )


def test_assess(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test assess tool."""
    plan = [
        {
            "type": "create-note",
            "value": "This is a well-written article about artificial intelligence.",
            "out": "$input_text"
        },
        {
            "type": "assess",
            "target": "$input_text",
            "out": "$assessment"
        }
    ]
    return run_test(
        session, character, "assess (quality assessment)",
        plan,
        expected_output_var="$assessment",
        validate_content=True,
        verbose=verbose
    )


def test_synthesize_comparison(session: zenoh.Session, character: str, verbose: bool = False) -> Dict[str, Any]:
    """Test synthesize tool in comparison mode."""
    plan = [
        {
            "type": "create-note",
            "value": "Python is a programming language.",
            "out": "$note1"
        },
        {
            "type": "create-note",
            "value": "JavaScript is also a programming language.",
            "out": "$note2"
        },
        {
            "type": "synthesize",
            "target": "$note1",
            "other": "$note2",
            "format": "comparison",
            "out": "$relation"
        }
    ]
    return run_test(
        session, character, "synthesize comparison (two notes)",
        plan,
        expected_output_var="$relation",
        validate_content=True,
        verbose=verbose
    )


# ==========================================
# MAIN
# ==========================================

def main():
    parser = argparse.ArgumentParser(description='Test LLM tools')
    parser.add_argument('--character', default='Jill', help='Character name (default: Jill)')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    args = parser.parse_args()
    
    # Initialize Zenoh
    config = zenoh.Config()
    session = zenoh.open(config)
    
    print("="*60)
    print("LLM TOOLS TEST SUITE")
    print("="*60)
    print(f"Character: {args.character}")
    print(f"Timestamp: {datetime.now().isoformat()}")
    print("\n⚠️  Note: These tests require LLM inference and may take longer to run.")
    
    # Run tests
    tests = [
        test_extract_transform,
        test_summarize,
        test_generate_note,
        test_extract_entities,
        test_assess,
        test_synthesize_comparison,
    ]
    
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

