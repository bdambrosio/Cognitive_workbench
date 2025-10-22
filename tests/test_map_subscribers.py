#!/usr/bin/env python3
"""
Test Map Node Subscribers (Request/Response Pattern)

Run this while the Jill scenario is active to test map_node subscribers.
Displays request and response for manual verification.

Coverage: 5 request/response subscribers

Tested:
- tool_request/tool_response - Execute tools
- scan_request/scan_response - Find resources
- move_request/move_response - Move agent
- index_request/index_response - Create search index
- search_request/search_response - Query search index

NOT Tested (fire-and-forget or risky):
- save_all - No response
- shutdown - Destructive
- character actions - State interference
- turn management - Complex coordination
- note/create - Queryable (not subscriber)
- collection/create - Queryable (not subscriber)

Usage:
    python tests/test_map_subscribers.py
"""

import sys
import json
import time
from pathlib import Path
import threading

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

import zenoh


def print_separator():
    print("━" * 80)


def print_test_header(test_name):
    print_separator()
    print(f"TEST: {test_name}")
    print_separator()


def print_request(topic, payload_data):
    print(f"Request:")
    print(f"  Topic: {topic}")
    print(f"  Method: PUT")
    print(f"  Payload:")
    print(json.dumps(payload_data, indent=4))


def print_response(response_data, elapsed_ms):
    print(f"\nResponse ({elapsed_ms:.1f}ms):")
    if response_data is None:
        print("  ❌ No response (timeout)")
        return False
    
    try:
        if isinstance(response_data, str):
            data = json.loads(response_data)
        else:
            data = response_data
        
        # Pretty print JSON
        print(json.dumps(data, indent=2))
        
        # Check status field if present
        if isinstance(data, dict) and 'status' in data:
            if data['status'] == 'success':
                print("\n✓ SUCCESS")
                return True
            else:
                print(f"\n✗ FAILED: {data.get('reason', 'Unknown error')}")
                return False
        else:
            print("\n✓ Response received")
            return True
            
    except json.JSONDecodeError as e:
        print(f"  Raw: {response_data}")
        print(f"\n⚠ JSON parse error: {e}")
        return False


def test_request_response(session, request_topic, response_topic, payload_data, timeout=3.0):
    """
    Test request/response pattern for subscribers.
    
    1. Subscribe to response topic
    2. Publish request
    3. Wait for response with timeout
    4. Return response data and elapsed time
    """
    response_data = [None]  # Use list for closure
    response_lock = threading.Event()
    
    def response_handler(sample):
        response_data[0] = sample.payload.to_bytes().decode('utf-8')
        response_lock.set()
    
    # Subscribe to response topic
    subscriber = session.declare_subscriber(response_topic, response_handler)
    
    # Publish request
    start = time.time()
    payload = json.dumps(payload_data).encode('utf-8')
    session.put(request_topic, payload)
    
    # Wait for response with timeout
    response_lock.wait(timeout)
    elapsed = (time.time() - start) * 1000
    
    # Cleanup
    subscriber.undeclare()
    
    return response_data[0], elapsed


def test_tool_request(session, world_name, agent_name="Jill", tool_name="question-decomposer"):
    """Test: Tool execution request"""
    print_test_header(f"Tool Request: {tool_name}")
    
    request_topic = f"map/{world_name}/tool_request/{agent_name}"
    response_topic = f"map/{world_name}/tool_response/{agent_name}"
    
    payload_data = {
        'agent_name': agent_name,
        'tool_name': tool_name,
        'input_value': 'test input for tool execution',
        'reason': 'testing tool request'
    }
    
    print_request(request_topic, payload_data)
    
    response, elapsed = test_request_response(session, request_topic, response_topic, payload_data)
    result = print_response(response, elapsed)
    print()
    return result


def test_scan_request(session, world_name, agent_name="Jill", target="task-analyzer"):
    """Test: Scan for resource"""
    print_test_header(f"Scan Request: {target}")
    
    request_topic = f"map/{world_name}/scan_request/{agent_name}"
    response_topic = f"map/{world_name}/scan_response/{agent_name}"
    
    payload_data = {
        'agent_name': agent_name,
        'target': target,
        'scan_type': 'tool'
    }
    
    print_request(request_topic, payload_data)
    
    response, elapsed = test_request_response(session, request_topic, response_topic, payload_data)
    result = print_response(response, elapsed)
    print()
    return result


def test_move_request(session, world_name, agent_name="Jill", target="plan-builder"):
    """Test: Move agent to resource"""
    print_test_header(f"Move Request: {agent_name} -> {target}")
    
    request_topic = f"map/{world_name}/move_request/{agent_name}"
    response_topic = f"map/{world_name}/move_response/{agent_name}"
    
    payload_data = {
        'agent_name': agent_name,
        'target': target
    }
    
    print_request(request_topic, payload_data)
    
    response, elapsed = test_request_response(session, request_topic, response_topic, payload_data)
    result = print_response(response, elapsed)
    print()
    return result


def test_index_request(session, world_name, agent_name="Jill"):
    """Test: Create search index"""
    print_test_header(f"Index Request: Create test_store")
    
    request_topic = f"map/{world_name}/index_request/{agent_name}"
    response_topic = f"map/{world_name}/index_response/{agent_name}"
    
    # Create test data to index
    test_data = [
        {'title': 'Test Document 1', 'content': 'This is a test document about AI'},
        {'title': 'Test Document 2', 'content': 'Another document about machine learning'}
    ]
    
    payload_data = {
        'agent_name': agent_name,
        'store_name': 'test_store',
        'source': test_data,
        'index_type': 'semantic',
        'fields': {'title': 'embed', 'content': 'embed'}
    }
    
    print_request(request_topic, payload_data)
    
    response, elapsed = test_request_response(session, request_topic, response_topic, payload_data, timeout=5.0)
    result = print_response(response, elapsed)
    print()
    return result


def test_search_request(session, world_name, agent_name="Jill"):
    """Test: Search indexed store"""
    print_test_header(f"Search Request: Query test_store")
    
    request_topic = f"map/{world_name}/search_request/{agent_name}"
    response_topic = f"map/{world_name}/search_response/{agent_name}"
    
    payload_data = {
        'agent_name': agent_name,
        'store_name': 'test_store',
        'query': 'artificial intelligence',
        'mode': 'semantic',
        'limit': 5,
        'threshold': 0.0
    }
    
    print_request(request_topic, payload_data)
    
    response, elapsed = test_request_response(session, request_topic, response_topic, payload_data, timeout=5.0)
    result = print_response(response, elapsed)
    print()
    return result


def get_world_name(session):
    """Get world name from map summary"""
    for reply in session.get("cognitive/map/summary", timeout=2.0):
        if reply.ok:
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data.get('success'):
                # Extract from map_file (e.g., "infolab.py" -> "infolab")
                map_file = data.get('map_file', '')
                if map_file:
                    return map_file.replace('.py', '')
        break
    return None


def main():
    print("\n" + "=" * 80)
    print("MAP NODE SUBSCRIBERS TEST")
    print("Assumes Jill scenario is running")
    print("=" * 80 + "\n")
    
    # Initialize Zenoh
    print("Connecting to Zenoh...")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("✓ Connected\n")
    
    # Get world name
    print("Discovering world name...")
    world_name = get_world_name(session)
    if not world_name:
        print("❌ Failed to get world name from map")
        session.close()
        return 1
    print(f"✓ World name: {world_name}\n")
    
    results = []
    
    # Run tests
    print("=" * 80)
    print("TOOL EXECUTION")
    print("=" * 80 + "\n")
    
    results.append(("Tool Request", test_tool_request(session, world_name, "Jill", "question-decomposer")))
    
    print("\n" + "=" * 80)
    print("RESOURCE DISCOVERY")
    print("=" * 80 + "\n")
    
    results.append(("Scan Request", test_scan_request(session, world_name, "Jill", "task-analyzer")))
    
    print("\n" + "=" * 80)
    print("AGENT MOVEMENT")
    print("=" * 80 + "\n")
    
    results.append(("Move Request", test_move_request(session, world_name, "Jill", "plan-builder")))
    
    print("\n" + "=" * 80)
    print("SEARCH INDEX OPERATIONS")
    print("=" * 80 + "\n")
    
    results.append(("Index Request", test_index_request(session, world_name, "Jill")))
    results.append(("Search Request", test_search_request(session, world_name, "Jill")))
    
    # Summary
    print("\n" + "=" * 80)
    print_separator()
    print("TEST SUMMARY")
    print_separator()
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status:8} {test_name}")
    
    print_separator()
    print(f"Results: {passed}/{total} passed")
    print(f"Coverage: 5 of 5 request/response subscribers (100%)")
    print("\nNOT TESTED (fire-and-forget or risky):")
    print("  - cognitive/save_all")
    print("  - cognitive/shutdown/shared")
    print("  - cognitive/*/action")
    print("  - cognitive/map/world_state/update")
    print("  - cognitive/map/note/create (queryable, not subscriber)")
    print("  - cognitive/map/collection/create (queryable, not subscriber)")
    print("  - Turn management subscribers")
    print_separator()
    print()
    
    session.close()
    
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

