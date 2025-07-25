#!/usr/bin/env python3

import sys
import os
import json
import time

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zenoh

def test_entity_integration():
    """Test the EntityModel integration with memory node"""
    print("🧪 Testing EntityModel Integration")
    print("=" * 45)
    
    # Create Zenoh configuration
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
    
    # Create Zenoh session
    session = zenoh.open(conf)
    
    try:
        # Wait for memory node to be ready
        time.sleep(2)
        
        # Test entity query for non-existent entity
        print("🔍 Testing query for non-existent entity...")
        for reply in session.get("cognitive/samantha/memory/entity/nonexistent"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if not data['success']:
                print(f"✅ Correctly returned error: {data['error']}")
            else:
                print(f"❌ Unexpected success: {data}")
        
        # Test entity query with limit parameter
        print("\n🔍 Testing entity query with limit parameter...")
        for reply in session.get("cognitive/samantha/memory/entity/joe?limit=5"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                entity_data = data['entity_data']
                print(f"✅ Entity data retrieved:")
                print(f"   Entity: {entity_data['entity_name']}")
                print(f"   First seen: {entity_data['first_seen']}")
                print(f"   Last seen: {entity_data['last_seen']}")
                print(f"   Full history count: {entity_data['full_history_count']}")
                print(f"   Recent history: {len(entity_data['conversation_history'])} entries")
            else:
                print(f"❌ Query failed: {data['error']}")
        
        # Test entity query without limit (should use default 20)
        print("\n🔍 Testing entity query without limit...")
        for reply in session.get("cognitive/samantha/memory/entity/joe"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                entity_data = data['entity_data']
                print(f"✅ Default limit query successful")
                print(f"   Recent history: {len(entity_data['conversation_history'])} entries")
            else:
                print(f"❌ Query failed: {data['error']}")
        
        print("\n🎉 Entity integration test completed!")
        
    finally:
        session.close()

if __name__ == "__main__":
    test_entity_integration() 