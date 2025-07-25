#!/usr/bin/env python3

import sys
import os
import json
import time

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from entity_model import EntityModel

def test_entity_model():
    """Test the EntityModel functionality"""
    print("🧪 Testing EntityModel")
    print("=" * 40)
    
    # Create an entity
    entity = EntityModel("samantha")
    print(f"✅ Created entity: {entity.entity_name}")
    
    # Add some conversation entries
    entity.add_conversation_entry("received", "Hello, how are you?", "ui")
    entity.add_conversation_entry("sent", "I'm doing well, thank you!", "samantha")
    entity.add_conversation_entry("received", "That's great to hear!", "ui")
    
    print(f"✅ Added {len(entity.conversation_history)} conversation entries")
    
    # Update visual detection
    entity.update_visual_detection()
    print(f"✅ Updated visual detection: {entity.last_seen}")
    
    # Test getting entity data
    entity_data = entity.get_entity_data(limit=5)
    print(f"✅ Entity data: {json.dumps(entity_data, indent=2)}")
    
    # Test recent conversation
    recent = entity.get_recent_conversation(limit=2)
    print(f"✅ Recent conversation (2 entries): {len(recent)} entries")
    
    print("🎉 EntityModel test completed!")

if __name__ == "__main__":
    test_entity_model() 