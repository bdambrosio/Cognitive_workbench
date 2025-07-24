#!/usr/bin/env python3

import sys
import os
import json
import time

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zenoh

def test_map_basic():
    """Test basic map node connectivity"""
    
    # Create Zenoh configuration
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
    
    # Create Zenoh session
    session = zenoh.open(conf)
    
    try:
        print("🧪 Testing Basic Map Node Connectivity")
        print("=" * 50)
        
        # Test 1: Map summary (should always work)
        print("\n1. Testing map summary...")
        try:
            for reply in session.get("cognitive/map/summary"):
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data['success']:
                    print(f"✅ Map summary successful")
                    print(f"   Map file: {data['map_file']}")
                    print(f"   Summary length: {len(data['summary'])} characters")
                else:
                    print(f"❌ Map summary failed: {data['error']}")
                break
        except Exception as e:
            print(f"❌ Map summary query failed: {e}")
        
        # Test 2: Resources list
        print("\n2. Testing resources list...")
        try:
            for reply in session.get("cognitive/map/resources"):
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data['success']:
                    print(f"✅ Resources query successful")
                    print(f"   Found {len(data['resources'])} resources")
                else:
                    print(f"❌ Resources query failed: {data['error']}")
                break
        except Exception as e:
            print(f"❌ Resources query failed: {e}")
        
        print("\n🎉 Basic connectivity test completed!")
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
    finally:
        session.close()

if __name__ == "__main__":
    test_map_basic() 