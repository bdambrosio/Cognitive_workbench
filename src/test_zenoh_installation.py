#!/usr/bin/env python3
"""
Test Zenoh Installation

This script verifies that Zenoh is properly installed and working.
"""

import sys
import time

def test_zenoh_import():
    """Test if Zenoh can be imported."""
    try:
        import zenoh
        print("✅ Zenoh import successful")
        return True
    except ImportError as e:
        print(f"❌ Zenoh import failed: {e}")
        print("   Please install Zenoh: pip install zenoh-python")
        return False

def test_zenoh_session():
    """Test if Zenoh session can be created."""
    try:
        import zenoh
        config = zenoh.Config()
        session = zenoh.open(config)
        print("✅ Zenoh session creation successful")
        session.close()
        return True
    except Exception as e:
        print(f"❌ Zenoh session creation failed: {e}")
        return False

def test_zenoh_pub_sub():
    """Test basic pub/sub functionality."""
    try:
        import zenoh
        import json
        import threading
        
        # Create session
        config = zenoh.Config()
        session = zenoh.open(config)
        
        # Test data
        test_data = {"message": "Hello Zenoh!", "timestamp": time.time()}
        
        # Create publisher
        publisher = session.declare_publisher("test/topic")
        
        # Create subscriber
        received_data = None
        received_event = threading.Event()
        
        def callback(sample):
            nonlocal received_data
            received_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            received_event.set()
        
        subscriber = session.declare_subscriber("test/topic", callback)
        
        # Publish data
        publisher.put(json.dumps(test_data))
        
        # Wait for reception
        if received_event.wait(timeout=5.0):
            if received_data == test_data:
                print("✅ Zenoh pub/sub test successful")
                result = True
            else:
                print(f"❌ Data mismatch: {received_data} != {test_data}")
                result = False
        else:
            print("❌ Timeout waiting for message")
            result = False
        
        # Cleanup
        session.close()
        return result
        
    except Exception as e:
        print(f"❌ Zenoh pub/sub test failed: {e}")
        return False

def test_zenoh_storage():
    """Test basic storage functionality."""
    try:
        import zenoh
        import json
        
        # Create session
        config = zenoh.Config()
        session = zenoh.open(config)
        
        # Test data
        test_key = "test/storage/key"
        test_data = {"value": 42, "timestamp": time.time()}
        
        # Store data
        session.put(test_key, json.dumps(test_data))
        
        # Retrieve data
        found_data = False
        for reply in session.get(test_key):
            retrieved_data = json.loads(reply.payload.to_bytes().decode('utf-8'))
            if retrieved_data == test_data:
                print("✅ Zenoh storage test successful")
                found_data = True
                result = True
            else:
                print(f"❌ Storage data mismatch: {retrieved_data} != {test_data}")
                result = False
            break
        
        if not found_data:
            print("⚠️  Zenoh storage not available (this is normal for basic setup)")
            print("   Storage requires additional configuration or plugins")
            result = True  # Not a failure, just not configured
        
        # Cleanup
        session.close()
        return result
        
    except Exception as e:
        print(f"❌ Zenoh storage test failed: {e}")
        return False

def main():
    """Run all tests."""
    print("🧪 Testing Zenoh Installation")
    print("=" * 40)
    
    tests = [
        ("Import", test_zenoh_import),
        ("Session", test_zenoh_session),
        ("Pub/Sub", test_zenoh_pub_sub),
        ("Storage", test_zenoh_storage)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n🔍 Testing {test_name}...")
        if test_func():
            passed += 1
        time.sleep(0.5)  # Brief pause between tests
    
    print("\n" + "=" * 40)
    print(f"📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Zenoh is ready to use.")
        return 0
    else:
        print("⚠️  Some tests failed. Please check the errors above.")
        return 1

if __name__ == '__main__':
    sys.exit(main()) 