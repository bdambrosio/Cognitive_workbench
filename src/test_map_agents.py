#!/usr/bin/env python3

import sys
import os
import json
import time

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zenoh

def test_map_agents():
    """Test the map agent functionality"""
    
    # Create Zenoh configuration
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
    
    # Create Zenoh session
    session = zenoh.open(conf)
    
    try:
        print("🧪 Testing Map Agent Functionality")
        print("=" * 50)
        
        # Test 1: Register an agent
        print("\n1. Registering agent for 'test_character'...")
        try:
            import time
            start_time = time.time()
            for reply in session.get("cognitive/map/agent/register/test_character"):
                if time.time() - start_time > 5:  # 5 second timeout
                    print("❌ Registration query timed out")
                    break
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data['success']:
                    print(f"✅ Agent registered: {data['message']}")
                    print(f"   Location: {data['location']}")
                else:
                    print(f"❌ Registration failed: {data['error']}")
                break
        except Exception as e:
            print(f"❌ Registration query failed: {e}")
        
        # Test 2: Look around
        print("\n2. Testing agent look...")
        for reply in session.get("cognitive/map/agent/test_character/look"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"✅ Look successful")
                print(f"   Result: {data['look_result'][:100]}...")  # Truncate for display
            else:
                print(f"❌ Look failed: {data['error']}")
            break
        
        # Test 3: Move agent
        print("\n3. Testing agent move...")
        # For now, let's test with a simple move without payload
        for reply in session.get("cognitive/map/agent/test_character/move"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"✅ Move successful")
                print(f"   Result: {data['move_result']}")
            else:
                print(f"❌ Move failed: {data['error']}")
            break
        
        # Test 4: Look again after moving
        print("\n4. Looking around after move...")
        for reply in session.get("cognitive/map/agent/test_character/look"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"✅ Look successful after move")
                print(f"   Result: {data['look_result'][:100]}...")  # Truncate for display
            else:
                print(f"❌ Look failed: {data['error']}")
            break
        
        print("\n🎉 All tests completed!")
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
    finally:
        session.close()

if __name__ == "__main__":
    test_map_agents() 