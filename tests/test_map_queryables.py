#!/usr/bin/env python3
"""
Test Map Node Queryables

Run this while the Jill scenario is active to test map_node queryables.
Displays query and response for manual verification.

Coverage: 19 of 22 queryables (86%)

Tested:
- Basic queries: summary, types, resources, terrain, time, world_state
- Agent queries: look, register, move, move_to, use_path
- Location queries: random_location by resource/terrain
- Conversation locks: status, acquire, release
- Turn management: status
- Resource queries: by name, rules by type

NOT Tested (destructive):
- resource/remove/* - Deletes resources
- resource/place/* - Creates resources
- information/create - Creates dynamic resources

Usage:
    python tests/test_map_queryables.py
"""

import sys
import json
import time
from pathlib import Path

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


def print_query(topic, method="GET", params=None):
    print(f"Query:")
    print(f"  Topic: {topic}")
    print(f"  Method: {method}")
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


def query_get(session, topic, timeout=2.0, payload=None):
    """Execute GET query and return response
    
    Args:
        session: Zenoh session
        topic: Query topic
        timeout: Timeout in seconds
        payload: Optional payload bytes to send with query
    """
    start = time.time()
    
    for reply in session.get(
        topic,
        target=QueryTarget.BEST_MATCHING,
        consolidation=ConsolidationMode.NONE,
        timeout=timeout,
        payload=payload
    ):
        elapsed = (time.time() - start) * 1000
        if reply.ok:
            response_payload = reply.ok.payload.to_bytes().decode('utf-8')
            return response_payload, elapsed
        break
    
    elapsed = (time.time() - start) * 1000
    return None, elapsed


def test_map_summary(session):
    """Test: Get map summary"""
    print_test_header("Map Summary")
    print_query("cognitive/map/summary")
    
    response, elapsed = query_get(session, "cognitive/map/summary")
    result = print_response(response, elapsed)
    print()
    return result


def test_map_types(session):
    """Test: Get map types"""
    print_test_header("Map Types")
    print_query("cognitive/map/types")
    
    response, elapsed = query_get(session, "cognitive/map/types")
    result = print_response(response, elapsed)
    print()
    return result


def test_resources_list(session):
    """Test: Get all resources"""
    print_test_header("Resources List")
    print_query("cognitive/map/resources")
    
    response, elapsed = query_get(session, "cognitive/map/resources")
    result = print_response(response, elapsed)
    print()
    return result


def test_resource_rules_skill(session):
    """Test: Get Skill resource rules"""
    print_test_header("Resource Rules: Skill")
    print_query("cognitive/map/resource_rules/Skill")
    
    response, elapsed = query_get(session, "cognitive/map/resource_rules/Skill")
    result = print_response(response, elapsed)
    print()
    return result


def test_resource_by_name(session, resource_name):
    """Test: Get specific resource by name"""
    print_test_header(f"Resource By Name: {resource_name}")
    print_query(f"cognitive/map/resource/{resource_name}")
    
    response, elapsed = query_get(session, f"cognitive/map/resource/{resource_name}")
    result = print_response(response, elapsed)
    print()
    return result


def test_terrain(session):
    """Test: Get terrain info"""
    print_test_header("Terrain Info")
    print_query("cognitive/map/terrain")
    
    response, elapsed = query_get(session, "cognitive/map/terrain")
    result = print_response(response, elapsed)
    print()
    return result


def test_simulation_time(session):
    """Test: Get simulation time"""
    print_test_header("Simulation Time")
    print_query("cognitive/map/simulation_time")
    
    response, elapsed = query_get(session, "cognitive/map/simulation_time")
    result = print_response(response, elapsed)
    print()
    return result


def test_world_state(session):
    """Test: Get world state"""
    print_test_header("World State")
    print_query("cognitive/map/world_state")
    
    response, elapsed = query_get(session, "cognitive/map/world_state")
    result = print_response(response, elapsed)
    print()
    return result


def test_agent_look(session, agent_name="Jill"):
    """Test: Agent look (perception)"""
    print_test_header(f"Agent Look: {agent_name}")
    print_query(f"cognitive/map/agent/{agent_name}/look")
    
    response, elapsed = query_get(session, f"cognitive/map/agent/{agent_name}/look")
    result = print_response(response, elapsed)
    print()
    return result


def test_conversation_lock_status(session, agent_name="Jill"):
    """Test: Conversation lock status"""
    print_test_header(f"Conversation Lock Status: {agent_name}")
    print_query(f"cognitive/map/conversation/lock/status/{agent_name}")
    
    response, elapsed = query_get(session, f"cognitive/map/conversation/lock/status/{agent_name}")
    result = print_response(response, elapsed)
    print()
    return result


def test_random_location_resource(session, resource_type="Skill"):
    """Test: Get random location for resource type"""
    print_test_header(f"Random Location by Resource: {resource_type}")
    print_query(f"cognitive/map/random_location/resource/{resource_type}")
    
    response, elapsed = query_get(session, f"cognitive/map/random_location/resource/{resource_type}")
    result = print_response(response, elapsed)
    print()
    return result


def test_random_location_terrain(session, terrain_type="flat"):
    """Test: Get random location by terrain type"""
    print_test_header(f"Random Location by Terrain: {terrain_type}")
    print_query(f"cognitive/map/random_location/terrain/{terrain_type}")
    
    response, elapsed = query_get(session, f"cognitive/map/random_location/terrain/{terrain_type}")
    result = print_response(response, elapsed)
    print()
    return result


def test_turn_status(session):
    """Test: Get turn management status"""
    print_test_header("Turn Status")
    print_query("cognitive/map/turn/status")
    
    response, elapsed = query_get(session, "cognitive/map/turn/status")
    result = print_response(response, elapsed)
    print()
    return result


def test_agent_register(session, agent_name="TestAgent"):
    """Test: Agent registration (idempotent)"""
    print_test_header(f"Agent Register: {agent_name}")
    print_query(f"cognitive/map/agent/register/{agent_name}")
    
    response, elapsed = query_get(session, f"cognitive/map/agent/register/{agent_name}")
    result = print_response(response, elapsed)
    print()
    return result


def test_conversation_lock_acquire(session, requester="Jill", target="User"):
    """Test: Conversation lock acquire"""
    print_test_header(f"Conversation Lock Acquire: {requester} -> {target}")
    
    # Need to send payload with lock request
    payload_data = {
        'requester': requester,
        'target': target
    }
    payload = json.dumps(payload_data).encode('utf-8')
    
    print_query("cognitive/map/conversation/lock/acquire", params=payload_data)
    
    response, elapsed = query_get(session, "cognitive/map/conversation/lock/acquire", payload=payload)
    result = print_response(response, elapsed)
    print()
    return result


def test_conversation_lock_release(session, character1="Jill", character2="User"):
    """Test: Conversation lock release"""
    print_test_header(f"Conversation Lock Release: {character1} <-> {character2}")
    
    # Need to send payload with lock release (uses character1/character2, not requester/target)
    payload_data = {
        'character1': character1,
        'character2': character2
    }
    payload = json.dumps(payload_data).encode('utf-8')
    
    print_query("cognitive/map/conversation/lock/release", params=payload_data)
    
    response, elapsed = query_get(session, "cognitive/map/conversation/lock/release", payload=payload)
    result = print_response(response, elapsed)
    print()
    return result


def test_agent_move(session, agent_name="Jill", direction="north"):
    """Test: Agent move query"""
    print_test_header(f"Agent Move: {agent_name} {direction}")
    print_query(f"cognitive/map/agent/{agent_name}/move", params={'direction': direction})
    
    response, elapsed = query_get(session, f"cognitive/map/agent/{agent_name}/move")
    result = print_response(response, elapsed)
    print()
    return result


def test_agent_move_to(session, agent_name="Jill", resource_name=None):
    """Test: Agent move to resource"""
    # If no resource provided, try to get one from resources list
    if resource_name is None:
        try:
            resources_response, _ = query_get(session, "cognitive/map/resources")
            if resources_response:
                data = json.loads(resources_response)
                if data.get('success') and data.get('resources'):
                    resources = data['resources']
                    if resources:
                        # Pick first resource
                        first_resource = resources[0]
                        resource_name = first_resource.get('name', first_resource.get('id'))
        except:
            pass
    
    if not resource_name:
        resource_name = "unknown_resource"
    
    print_test_header(f"Agent Move To: {agent_name} -> {resource_name}")
    
    # Send resource_name in payload
    payload_data = {'resource_name': resource_name}
    payload = json.dumps(payload_data).encode('utf-8')
    
    print_query(f"cognitive/map/agent/{agent_name}/move_to", params=payload_data)
    
    response, elapsed = query_get(session, f"cognitive/map/agent/{agent_name}/move_to", payload=payload)
    result = print_response(response, elapsed)
    print()
    return result


def test_agent_use_path(session, agent_name="Jill"):
    """Test: Agent use path"""
    print_test_header(f"Agent Use Path: {agent_name}")
    print_query(f"cognitive/map/agent/{agent_name}/use_path")
    
    response, elapsed = query_get(session, f"cognitive/map/agent/{agent_name}/use_path")
    result = print_response(response, elapsed)
    print()
    return result


def main():
    print("\n" + "=" * 80)
    print("MAP NODE QUERYABLES TEST")
    print("Assumes Jill scenario is running")
    print("=" * 80 + "\n")
    
    # Initialize Zenoh
    print("Connecting to Zenoh...")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("✓ Connected\n")
    
    results = []
    
    # Run tests - Basic Read-Only Queries
    print("=" * 80)
    print("BASIC READ-ONLY QUERIES")
    print("=" * 80 + "\n")
    
    results.append(("Map Summary", test_map_summary(session)))
    results.append(("Map Types", test_map_types(session)))
    results.append(("Resources List", test_resources_list(session)))
    results.append(("Resource Rules: Skill", test_resource_rules_skill(session)))
    results.append(("Terrain Info", test_terrain(session)))
    results.append(("Simulation Time", test_simulation_time(session)))
    results.append(("World State", test_world_state(session)))
    
    # Agent-Specific Queries
    print("\n" + "=" * 80)
    print("AGENT-SPECIFIC QUERIES")
    print("=" * 80 + "\n")
    
    results.append(("Agent Look: Jill", test_agent_look(session, "Jill")))
    results.append(("Conversation Lock Status", test_conversation_lock_status(session, "Jill")))
    
    # Location Queries
    print("\n" + "=" * 80)
    print("LOCATION QUERIES")
    print("=" * 80 + "\n")
    
    results.append(("Random Location: Resource", test_random_location_resource(session, "Skill")))
    results.append(("Random Location: Terrain", test_random_location_terrain(session, "flat")))
    
    # Turn Management
    print("\n" + "=" * 80)
    print("TURN MANAGEMENT")
    print("=" * 80 + "\n")
    
    results.append(("Turn Status", test_turn_status(session)))
    
    # Stateful Operations (safe)
    print("\n" + "=" * 80)
    print("STATEFUL OPERATIONS (SAFE)")
    print("=" * 80 + "\n")
    
    results.append(("Agent Register", test_agent_register(session, "TestAgent")))
    results.append(("Agent Move Query", test_agent_move(session, "Jill", "north")))
    results.append(("Agent Move To Resource", test_agent_move_to(session, "Jill")))  # Auto-picks resource
    results.append(("Agent Use Path", test_agent_use_path(session, "Jill")))
    
    # Conversation Locks
    print("\n" + "=" * 80)
    print("CONVERSATION LOCKS")
    print("=" * 80 + "\n")
    
    results.append(("Lock Acquire", test_conversation_lock_acquire(session, "Jill", "User")))
    results.append(("Lock Release", test_conversation_lock_release(session, "Jill", "User")))
    
    # Individual Resource Lookup
    print("\n" + "=" * 80)
    print("INDIVIDUAL RESOURCE LOOKUP")
    print("=" * 80 + "\n")
    
    print_test_header("Finding a Resource to Test")
    resource_response, _ = query_get(session, "cognitive/map/resources")
    if resource_response:
        try:
            data = json.loads(resource_response)
            if data.get('success') and data.get('resources'):
                first_resource = data['resources'][0]
                resource_name = first_resource.get('name', first_resource.get('id', 'unknown'))
                print(f"Found resource: {resource_name}")
                print()
                results.append((f"Resource: {resource_name}", 
                              test_resource_by_name(session, resource_name)))
        except Exception as e:
            print(f"Could not extract resource name: {e}")
            print()
    
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
    print(f"Coverage: {total} of 22 total queryables ({total/22*100:.0f}%)")
    print("\nNOT TESTED (destructive operations):")
    print("  - cognitive/map/resource/remove/*")
    print("  - cognitive/map/resource/place/*")
    print("  - cognitive/map/information/create")
    print_separator()
    print()
    
    session.close()
    
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

