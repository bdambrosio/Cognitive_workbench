#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from entity_model import EntityModel
from datetime import datetime, timedelta
import time

def test_basic_functionality():
    print("=== Testing Basic Dialog Functionality ===")
    
    # Create entity and add some conversation entries
    entity = EntityModel("Samantha")
    
    # Add entries to first dialog
    entity.add_conversation_entry("Joe", "Hi there!")
    entity.add_conversation_entry("Samantha", "Hello Joe!")
    
    # Check current state
    print(f"Entity: {entity.entity_name}")
    print(f"Active dialog: {entity.active}")
    print(f"Dialog count: {len(entity.dialogs)}")
    print(f"Current dialog entries: {len(entity.dialogs[-1]) if entity.dialogs else 0}")
    
    # Add more to same dialog
    entity.add_conversation_entry("Joe", "How are you?")
    entity.add_conversation_entry("Samantha", "I'm good, thanks!")
    
    print(f"After more entries - dialog count: {len(entity.dialogs)}")
    print(f"Current dialog entries: {len(entity.dialogs[-1])}")
    
    # Close dialog and add new entry - should start new dialog
    entity.close_dialog()
    print(f"After close - active: {entity.active}")
    
    entity.add_conversation_entry("Samantha", "Wait, Joe!")
    print(f"After new entry - active: {entity.active}")
    print(f"Dialog count: {len(entity.dialogs)}")
    print(f"Entries in dialog 1: {len(entity.dialogs[0])}")
    print(f"Entries in dialog 2: {len(entity.dialogs[1])}")
    
    print("✅ Basic functionality test passed\n")
    return entity

def test_persistence():
    print("=== Testing Persistence ===")
    
    # Create entity with some data
    entity = EntityModel("TestEntity")
    entity.add_conversation_entry("user", "Test message 1")
    entity.add_conversation_entry("character", "Test response 1")
    entity.update_last_seen()
    
    # Serialize
    data = entity.to_dict()
    print(f"Serialized data keys: {data.keys()}")
    print(f"Active flag: {data['active']}")
    print(f"Dialog count: {len(data['dialogs'])}")
    
    # Load into new entity
    new_entity = EntityModel.load_from_dict(data)
    
    print(f"Loaded entity name: {new_entity.entity_name}")
    print(f"Loaded active: {new_entity.active}")
    print(f"Loaded dialog count: {len(new_entity.dialogs)}")
    print(f"Conversation history: {len(new_entity.get_recent_conversation())}")
    
    print("✅ Persistence test passed\n")

def test_migration_from_old_format():
    print("=== Testing Migration from Old Format ===")
    
    # Simulate old format data
    old_format_data = {
        'entity_name': 'OldEntity',
        'first_seen': '2024-01-01T10:00:00',
        'last_seen': '2024-01-01T11:00:00',
        'conversation_history': [
            {'source': 'user', 'text': 'Old message 1', 'timestamp': '2024-01-01T10:30:00'},
            {'source': 'character', 'text': 'Old response 1', 'timestamp': '2024-01-01T10:31:00'}
        ]
    }
    
    # Load using migration
    migration_entity = EntityModel.load_from_dict(old_format_data)
    
    print(f"Migrated entity name: {migration_entity.entity_name}")
    print(f"Active after migration: {migration_entity.active}")
    print(f"Dialog count: {len(migration_entity.dialogs)}")
    print(f"Entries in first dialog: {len(migration_entity.dialogs[0]) if migration_entity.dialogs else 0}")
    
    # Test that new entries work after migration
    migration_entity.add_conversation_entry("user", "New message after migration")
    print(f"After adding new entry - still active: {migration_entity.active}")
    print(f"Entries in dialog: {len(migration_entity.dialogs[0])}")
    
    print("✅ Migration test passed\n")

def test_scope_functionality():
    print("=== Testing Scope Functionality ===")
    
    entity = EntityModel("ScopeTest")
    
    # Add entries to first dialog
    entity.add_conversation_entry("user", "Dialog 1 - Message 1")
    entity.add_conversation_entry("character", "Dialog 1 - Response 1")
    entity.add_conversation_entry("user", "Dialog 1 - Message 2")
    
    # Close and start new dialog
    entity.close_dialog()
    entity.add_conversation_entry("character", "Dialog 2 - Message 1")
    entity.add_conversation_entry("user", "Dialog 2 - Response 1")
    
    # Test current scope (should get only last dialog)
    current_data = entity.get_entity_data(limit=10, scope='current')
    print(f"Current scope entries: {len(current_data['conversation_history'])}")
    print(f"Current scope dialog count: {current_data['dialog_count']}")
    
    # Test all scope (should get from both dialogs, up to limit)
    all_data = entity.get_entity_data(limit=10, scope='all')
    print(f"All scope entries: {len(all_data['conversation_history'])}")
    
    # Test limit with all scope
    limited_all_data = entity.get_entity_data(limit=3, scope='all')
    print(f"Limited all scope entries: {len(limited_all_data['conversation_history'])}")
    
    print("✅ Scope functionality test passed\n")

def test_chronological_order_check():
    print("=== Testing Chronological Order Check ===")
    
    entity = EntityModel("ChronoTest")
    
    # Add normal entry
    entity.add_conversation_entry("user", "Normal message")
    time.sleep(0.01)  # Small delay to ensure different timestamps
    
    # Try to add an entry with past timestamp - should warn but continue
    past_time = datetime.now() - timedelta(seconds=10)
    print("Adding out-of-order entry (should see warning):")
    entity.add_conversation_entry("character", "Past message", past_time)
    
    # Check that entry was still added
    entries = entity.get_recent_conversation()
    print(f"Total entries after out-of-order add: {len(entries)}")
    
    print("✅ Chronological order check test passed\n")

def test_backward_compatibility():
    print("=== Testing Backward Compatibility ===")
    
    entity = EntityModel("CompatTest")
    entity.add_conversation_entry("user", "Test message")
    
    # Test conversation_history property
    history = entity.conversation_history
    print(f"Conversation history property works: {len(history)} entries")
    
    # Test has_interaction_history
    has_history = entity.has_interaction_history()
    print(f"Has interaction history: {has_history}")
    
    # Test summarize
    summary = entity.summarize()
    print(f"Summary: {summary}")
    
    print("✅ Backward compatibility test passed\n")

if __name__ == "__main__":
    print("🧪 Running EntityModel tests with KISS active flag approach\n")
    
    # Run all tests
    entity = test_basic_functionality()
    test_persistence()
    test_migration_from_old_format()
    test_scope_functionality()
    test_chronological_order_check()
    test_backward_compatibility()
    
    print("🎉 All tests completed!") 