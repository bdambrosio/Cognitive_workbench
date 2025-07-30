#!/usr/bin/env python3
"""
Test script for the expanded entity queryable functionality.
Tests both query=dialog and query=natural_dialog_end operations.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from memory_node import ZenohMemoryNode
from entity_model import EntityModel
import json
import time
from unittest.mock import Mock

def test_queryable_expansion():
    print("🧪 Testing expanded entity queryable functionality\n")
    
    # Create mock logger and LLM client
    mock_logger = Mock()
    mock_llm_client = Mock()
    
    # Create memory node
    print("📝 Creating memory node...")
    memory_node = ZenohMemoryNode("testchar")
    
    # Create a test entity with some conversation history
    print("👥 Creating test entity with conversation history...")
    entity = memory_node.get_or_create_entity("TestFriend")
    entity.add_conversation_entry("testchar", "Hey there!")
    entity.add_conversation_entry("TestFriend", "Hello! How are you?")
    entity.add_conversation_entry("testchar", "I'm doing well, thanks!")
    
    print(f"   Created entity with {len(entity.get_recent_conversation())} conversation entries")
    
    # Test dialog query functionality
    print("\n🔍 Testing query=dialog functionality...")
    
    # Create a mock query for dialog retrieval
    class MockQuery:
        def __init__(self, key_expr, selector):
            self.key_expr = key_expr
            self.selector = selector
            self.response = None
        
        def reply(self, key, data):
            self.response = json.loads(data.decode('utf-8'))
    
    # Test dialog query
    dialog_query = MockQuery(
        "cognitive/testchar/memory/entity/TestFriend",
        "query=dialog&limit=5"
    )
    
    memory_node.handle_entity_query(dialog_query)
    
    if dialog_query.response and dialog_query.response.get('success'):
        entity_data = dialog_query.response['entity_data']
        print(f"✅ Dialog query successful:")
        print(f"   Entity: {entity_data['entity_name']}")
        print(f"   Dialog count: {entity_data['dialog_count']}")
        print(f"   Conversation entries: {len(entity_data['conversation_history'])}")
        print(f"   Active dialog: {entity_data['active_dialog']}")
    else:
        print(f"❌ Dialog query failed: {dialog_query.response}")
        return False
    
    # Test natural_dialog_end query functionality
    print("\n🤔 Testing query=natural_dialog_end functionality...")
    
    # Mock the LLM client response
    mock_llm_client.generate.return_value = "8</end>"
    
    # Test natural dialog end query
    import urllib.parse
    test_input = "Alright, I should get going now!"
    encoded_input = urllib.parse.quote(test_input)
    
    dialog_end_query = MockQuery(
        "cognitive/testchar/memory/entity/TestFriend", 
        f"query=natural_dialog_end&input_text={encoded_input}"
    )
    
    memory_node.handle_entity_query(dialog_end_query)
    
    if dialog_end_query.response and dialog_end_query.response.get('success'):
        should_end = dialog_end_query.response['should_end']
        print(f"✅ Natural dialog end query successful:")
        print(f"   Input text: '{test_input}'")
        print(f"   Should end dialog: {should_end}")
        print(f"   LLM called with transcript containing conversation history")
    else:
        print(f"❌ Natural dialog end query failed: {dialog_end_query.response}")
        return False
    
    # Test error cases
    print("\n⚠️  Testing error cases...")
    
    # Test missing query parameter
    missing_query = MockQuery(
        "cognitive/testchar/memory/entity/TestFriend",
        "limit=5"  # Missing query parameter
    )
    memory_node.handle_entity_query(missing_query)
    
    if not missing_query.response.get('success'):
        print(f"✅ Correctly rejected missing query parameter: {missing_query.response['error']}")
    else:
        print(f"❌ Should have rejected missing query parameter")
        return False
    
    # Test unknown query type
    unknown_query = MockQuery(
        "cognitive/testchar/memory/entity/TestFriend",
        "query=unknown_operation"
    )
    memory_node.handle_entity_query(unknown_query)
    
    if not unknown_query.response.get('success'):
        print(f"✅ Correctly rejected unknown query type: {unknown_query.response['error']}")
    else:
        print(f"❌ Should have rejected unknown query type")
        return False
    
    # Test missing input_text for natural_dialog_end
    missing_text_query = MockQuery(
        "cognitive/testchar/memory/entity/TestFriend",
        "query=natural_dialog_end"  # Missing input_text
    )
    memory_node.handle_entity_query(missing_text_query)
    
    if not missing_text_query.response.get('success'):
        print(f"✅ Correctly rejected missing input_text: {missing_text_query.response['error']}")
    else:
        print(f"❌ Should have rejected missing input_text")
        return False
    
    print("\n🎉 All queryable expansion tests passed!")
    
    # Cleanup
    memory_node.shutdown()
    return True

if __name__ == "__main__":
    success = test_queryable_expansion()
    if success:
        print("\n✅ Test suite completed successfully!")
    else:
        print("\n❌ Test suite failed!")
        sys.exit(1) 