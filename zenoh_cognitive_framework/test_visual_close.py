#!/usr/bin/env python3

import sys
import os
import json
import time

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zenoh

def test_visual_close():
    """Test visual events by moving agents close together"""
    print("🧪 Visual Event Test - Close Proximity")
    print("=" * 45)
    
    # Create Zenoh configuration
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
    
    # Create Zenoh session
    session = zenoh.open(conf)
    
    try:
        # Wait for map node to be ready
        time.sleep(2)
        
        # Register two agents
        print("🔧 Registering agents...")
        
        # Register samantha
        for reply in session.get("cognitive/map/agent/register/samantha"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"✅ Samantha registered at {data['location']}")
            else:
                print(f"❌ Samantha registration failed: {data['error']}")
                return
        
        # Register joe
        for reply in session.get("cognitive/map/agent/register/joe"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"✅ Joe registered at {data['location']}")
            else:
                print(f"❌ Joe registration failed: {data['error']}")
                return
        
        # Set up visual event listener for samantha
        print("👁️ Setting up visual event listener...")
        
        def handle_visual_event(sample):
            try:
                data = json.loads(sample.payload.to_bytes().decode('utf-8'))
                print(f"👁️ Visual event received: {data}")
            except Exception as e:
                print(f"❌ Error handling visual event: {e}")
        
        visual_subscriber = session.declare_subscriber(
            "cognitive/samantha/sense/visual/*",
            handle_visual_event
        )
        
        # Wait for listener to be ready
        time.sleep(1)
        
        # Get initial positions
        print("📍 Getting initial positions...")
        for reply in session.get("cognitive/map/agent/samantha/look"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"   Samantha: {data['look_result']}")
        
        for reply in session.get("cognitive/map/agent/joe/look"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"   Joe: {data['look_result']}")
        
        # Try to move agents closer by using specific directions
        print("🚶 Moving agents to get closer...")
        
        # Move joe towards samantha (assuming samantha is at a higher position)
        print("   Moving Joe north...")
        for reply in session.get("cognitive/map/agent/joe/move"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"   ✅ Joe moved: {data['move_result']}")
            else:
                print(f"   ❌ Joe move failed: {data['error']}")
        
        time.sleep(1)
        
        # Move joe east to get closer
        print("   Moving Joe east...")
        for reply in session.get("cognitive/map/agent/joe/move"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"   ✅ Joe moved: {data['move_result']}")
            else:
                print(f"   ❌ Joe move failed: {data['error']}")
        
        time.sleep(1)
        
        # Move joe north again
        print("   Moving Joe north again...")
        for reply in session.get("cognitive/map/agent/joe/move"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"   ✅ Joe moved: {data['move_result']}")
            else:
                print(f"   ❌ Joe move failed: {data['error']}")
        
        # Wait for events
        print("⏳ Waiting for visual events...")
        time.sleep(3)
        
        # Get final positions
        print("📍 Getting final positions...")
        for reply in session.get("cognitive/map/agent/samantha/look"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"   Samantha: {data['look_result']}")
        
        for reply in session.get("cognitive/map/agent/joe/look"):
            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if data['success']:
                print(f"   Joe: {data['look_result']}")
        
        print("🎉 Test completed!")
        
    finally:
        session.close()

if __name__ == "__main__":
    test_visual_close() 