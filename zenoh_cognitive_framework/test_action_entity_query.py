#!/usr/bin/env python3

import sys
import os
import json
import time

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from action_node import ZenohActionNode

def test_action_entity_query():
    """Test the action_node's entity query functionality"""
    print("🧪 Testing Action Node Entity Query")
    print("=" * 40)
    
    # Create action node instance
    action_node = ZenohActionNode("samantha", {})
    
    try:
        # Wait for memory node to be ready
        time.sleep(2)
        
        # Test querying a non-existent entity
        print("🔍 Testing query for non-existent entity...")
        result = action_node.get_entity_context("nonexistent")
        if result is None:
            print("✅ Correctly returned None for non-existent entity")
        else:
            print(f"❌ Unexpected result: {result}")
        
        # Test querying an existing entity (if any exist)
        print("\n🔍 Testing query for existing entity...")
        result = action_node.get_entity_context("joe")
        if result:
            print(f"✅ Retrieved entity data for joe:")
            print(f"   Entity: {result['entity_name']}")
            print(f"   First seen: {result['first_seen']}")
            print(f"   Last seen: {result['last_seen']}")
            print(f"   Full history count: {result['full_history_count']}")
            print(f"   Recent history: {len(result['conversation_history'])} entries")
        else:
            print("ℹ️ No entity data found for joe (this is normal if no interactions occurred)")
        
        # Test querying with custom limit
        print("\n🔍 Testing query with custom limit...")
        result = action_node.get_entity_context("joe", limit=5)
        if result:
            print(f"✅ Retrieved entity data with custom limit:")
            print(f"   Recent history: {len(result['conversation_history'])} entries")
        else:
            print("ℹ️ No entity data found with custom limit")
        
        print("\n🎉 Action node entity query test completed!")
        
    finally:
        action_node.shutdown()

if __name__ == "__main__":
    test_action_entity_query() 