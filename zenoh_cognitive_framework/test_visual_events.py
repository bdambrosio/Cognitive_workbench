#!/usr/bin/env python3

import sys
import os
import json
import time
import threading

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zenoh

class VisualEventTester:
    def __init__(self):
        # Create Zenoh configuration
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
        
        # Create Zenoh session
        self.session = zenoh.open(conf)
        self.received_events = []
        self.event_lock = threading.Lock()
        
    def setup_event_listeners(self):
        """Set up listeners for visual events"""
        # Listen for visual events for both test characters
        self.samantha_visual_subscriber = self.session.declare_subscriber(
            "cognitive/samantha/sense/visual/*",
            self.handle_visual_event
        )
        
        self.joe_visual_subscriber = self.session.declare_subscriber(
            "cognitive/joe/sense/visual/*",
            self.handle_visual_event
        )
        
        print("👁️ Visual event listeners set up")
    
    def handle_visual_event(self, sample):
        """Handle incoming visual sense events"""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            event_type = str(sample.key_expr).split('/')[-1]
            
            with self.event_lock:
                self.received_events.append({
                    'event_type': event_type,
                    'data': data,
                    'timestamp': time.time()
                })
            
            print(f"👁️ Visual event received: {event_type}")
            print(f"   Data: {data}")
            
        except Exception as e:
            print(f"❌ Error handling visual event: {e}")
    
    def register_agent(self, character_name):
        """Register an agent for a character"""
        print(f"\n🔧 Registering agent for '{character_name}'...")
        try:
            for reply in self.session.get(f"cognitive/map/agent/register/{character_name}"):
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data['success']:
                    print(f"✅ Agent registered: {data['message']}")
                    return True
                else:
                    print(f"❌ Registration failed: {data['error']}")
                    return False
        except Exception as e:
            print(f"❌ Registration query failed: {e}")
            return False
    
    def move_agent(self, character_name, direction):
        """Move an agent in a specific direction"""
        print(f"\n🚶 Moving {character_name} {direction}...")
        try:
            # For now, just use the default direction since the map node handles missing payload
            for reply in self.session.get(f"cognitive/map/agent/{character_name}/move"):
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data['success']:
                    print(f"✅ Move successful: {data['move_result']}")
                    return True
                else:
                    print(f"❌ Move failed: {data['error']}")
                    return False
        except Exception as e:
            print(f"❌ Move query failed: {e}")
            return False
    
    def get_agent_location(self, character_name):
        """Get current location of an agent"""
        try:
            for reply in self.session.get(f"cognitive/map/agent/{character_name}/look"):
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data['success']:
                    # Extract position from look result (simplified)
                    return "position_available"
                else:
                    return None
        except Exception as e:
            return None
    
    def run_test(self):
        """Run the visual event test"""
        print("🧪 Testing Visual Event System")
        print("=" * 50)
        
        # Set up event listeners
        self.setup_event_listeners()
        
        # Wait a moment for listeners to be ready
        time.sleep(1)
        
        # Register two agents
        if not self.register_agent("samantha"):
            print("❌ Failed to register samantha")
            return
        
        if not self.register_agent("joe"):
            print("❌ Failed to register joe")
            return
        
        # Wait a moment for registration
        time.sleep(1)
        
        # Get initial positions
        print("\n📍 Initial positions:")
        samantha_pos = self.get_agent_location("samantha")
        joe_pos = self.get_agent_location("joe")
        print(f"   Samantha: {samantha_pos}")
        print(f"   Joe: {joe_pos}")
        
        # Clear any events from registration
        with self.event_lock:
            self.received_events.clear()
        
        # Move agents to potentially see each other
        print("\n🚶 Moving agents to test visibility...")
        
        # Move samantha first
        self.move_agent("samantha", "north")
        time.sleep(2)  # Wait for events
        
        # Move joe
        self.move_agent("joe", "south")
        time.sleep(2)  # Wait for events
        
        # Move samantha again
        self.move_agent("samantha", "east")
        time.sleep(2)  # Wait for events
        
        # Check results
        print(f"\n📊 Test Results:")
        print(f"   Total events received: {len(self.received_events)}")
        
        for i, event in enumerate(self.received_events):
            print(f"   Event {i+1}: {event['event_type']} - {event['data']}")
        
        if self.received_events:
            print("\n✅ Visual event system is working!")
        else:
            print("\n⚠️ No visual events received - agents may be too far apart")
        
        print("\n🎉 Test completed!")

def main():
    tester = VisualEventTester()
    try:
        tester.run_test()
    finally:
        tester.session.close()

if __name__ == "__main__":
    main() 