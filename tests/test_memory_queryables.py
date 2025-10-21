#!/usr/bin/env python3
"""
Test Memory Node Queryables

Run this while the Jill scenario is active to test memory_node queryables.
Displays query and response for manual verification.

Coverage: 5 of 5 queryables (100%)

Tested:
- short_term/* - Recent perception/sense data
- chat/* - Recent chat messages
- entity/* - Entity relationship and dialog data
- inventory - Character inventory items
- rag/search - RAG semantic search

Usage:
    python tests/test_memory_queryables.py
"""

import sys
import json
import time
from pathlib import Path
import urllib.parse

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

import zenoh
from zenoh import QueryTarget, ConsolidationMode


def print_separator():
    print("━" * 80)


def print_test_header(test_name):
    print_separator()
    print(f"TEST: {test_name}")
    print_separator()


def print_query(topic, params=None):
    print(f"Query:")
    print(f"  Topic: {topic}")
    print(f"  Method: GET")
    if params:
        print(f"  Params: {params}")


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
        
        # Check success field if present
        if isinstance(data, dict) and 'success' in data:
            if data['success']:
                print("\n✓ SUCCESS")
                return True
            else:
                print(f"\n✗ FAILED: {data.get('error', 'Unknown error')}")
                return False
        else:
            print("\n✓ Response received")
            return True
            
    except json.JSONDecodeError as e:
        print(f"  Raw: {response_data}")
        print(f"\n⚠ JSON parse error: {e}")
        return False


def query_get(session, topic, timeout=2.0):
    """Execute GET query and return response"""
    start = time.time()
    
    for reply in session.get(
        topic,
        target=QueryTarget.BEST_MATCHING,
        consolidation=ConsolidationMode.NONE,
        timeout=timeout
    ):
        elapsed = (time.time() - start) * 1000
        if reply.ok:
            payload = reply.ok.payload.to_bytes().decode('utf-8')
            return payload, elapsed
        break
    
    elapsed = (time.time() - start) * 1000
    return None, elapsed


def test_short_term_memory(session, character_name="Jill", limit=5):
    """Test: Query short-term memory"""
    print_test_header(f"Short-Term Memory ({character_name})")
    
    topic = f"cognitive/{character_name}/memory/short_term/*?limit={limit}"
    print_query(topic, params={'limit': limit})
    
    response, elapsed = query_get(session, topic)
    result = print_response(response, elapsed)
    print()
    return result


def test_chat_memory(session, character_name="Jill", limit=10):
    """Test: Query chat memory"""
    print_test_header(f"Chat Memory ({character_name})")
    
    topic = f"cognitive/{character_name}/memory/chat/*?limit={limit}"
    print_query(topic, params={'limit': limit})
    
    response, elapsed = query_get(session, topic)
    result = print_response(response, elapsed)
    print()
    return result


def test_entity_query(session, character_name="Jill", entity_name="User", limit=10):
    """Test: Query entity data"""
    print_test_header(f"Entity Query ({character_name} -> {entity_name})")
    
    topic = f"cognitive/{character_name}/memory/entity/{entity_name}?query=dialog&limit={limit}&scope=all"
    print_query(topic, params={'query': 'dialog', 'entity': entity_name, 'limit': limit, 'scope': 'all'})
    
    response, elapsed = query_get(session, topic, timeout=5.0)
    result = print_response(response, elapsed)
    print()
    return result


def test_inventory_list(session, character_name="Jill"):
    """Test: Query inventory (list all)"""
    print_test_header(f"Inventory List ({character_name})")
    
    topic = f"cognitive/{character_name}/memory/inventory"
    print_query(topic)
    
    response, elapsed = query_get(session, topic)
    result = print_response(response, elapsed)
    print()
    return result


def test_inventory_check(session, character_name="Jill", item_name="Berries25"):
    """Test: Query inventory (check specific item)"""
    print_test_header(f"Inventory Check ({character_name} has {item_name}?)")
    
    encoded_item = urllib.parse.quote(item_name)
    topic = f"cognitive/{character_name}/memory/inventory?item={encoded_item}"
    print_query(topic, params={'item': item_name})
    
    response, elapsed = query_get(session, topic)
    result = print_response(response, elapsed)
    print()
    return result


def test_rag_search(session, character_name="Jill", query_text="research", k=5):
    """Test: RAG semantic search"""
    print_test_header(f"RAG Search ({character_name})")
    
    encoded_query = urllib.parse.quote(query_text)
    topic = f"cognitive/{character_name}/memory/rag/search?query={encoded_query}&k={k}"
    print_query(topic, params={'query': query_text, 'k': k})
    
    response, elapsed = query_get(session, topic, timeout=5.0)
    result = print_response(response, elapsed)
    print()
    return result


def main():
    print("\n" + "=" * 80)
    print("MEMORY NODE QUERYABLES TEST")
    print("Assumes Jill scenario is running")
    print("=" * 80 + "\n")
    
    # Initialize Zenoh
    print("Connecting to Zenoh...")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("✓ Connected\n")
    
    character_name = "Jill"
    
    results = []
    
    # Memory Queries
    print("=" * 80)
    print("MEMORY QUERIES")
    print("=" * 80 + "\n")
    
    results.append(("Short-Term Memory", test_short_term_memory(session, character_name, limit=5)))
    results.append(("Chat Memory", test_chat_memory(session, character_name, limit=10)))
    
    # Entity Queries
    print("\n" + "=" * 80)
    print("ENTITY QUERIES")
    print("=" * 80 + "\n")
    
    results.append(("Entity: User", test_entity_query(session, character_name, "User", limit=10)))
    
    # Inventory Queries
    print("\n" + "=" * 80)
    print("INVENTORY QUERIES")
    print("=" * 80 + "\n")
    
    results.append(("Inventory List", test_inventory_list(session, character_name)))
    results.append(("Inventory Check Item", test_inventory_check(session, character_name, "Berries25")))
    
    # RAG Search
    print("\n" + "=" * 80)
    print("RAG SEMANTIC SEARCH")
    print("=" * 80 + "\n")
    
    results.append(("RAG Search", test_rag_search(session, character_name, "research", k=5)))
    
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
    print(f"Coverage: 5 of 5 queryables (100%)")
    print("\nNOTE: Empty results are normal if memory has no data yet.")
    print("      Tests verify the queryables respond, not that data exists.")
    print_separator()
    print()
    
    session.close()
    
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

