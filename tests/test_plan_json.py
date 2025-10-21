#!/usr/bin/env python3
"""
Test file for plan.py parse_plan_json and verify_plan methods (JSON format)
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from plan import parse_plan_json, verify_plan

def test_parse_plan_json():
    """Test the parse_plan_json function with various JSON inputs."""
    print("Testing parse_plan_json with JSON format...")
    
    # Test 1: Simple single action
    print("\n1. Testing single action:")
    plan_text = '{"plan": [{"type": "move", "target": "north"}]}'
    try:
        result = parse_plan_json(plan_text)
        assert 'plan' in result
        assert len(result['plan']) == 1
        assert result['plan'][0]['type'] == 'move'
        assert verify_plan(result)
        print("✓ Single action plan works")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Test 2: With 'plan:' prefix
    print("\n2. Testing with 'plan:' prefix:")
    plan_text = 'plan: {"plan": [{"type": "move", "target": "north"}]}'
    try:
        result = parse_plan_json(plan_text)
        assert 'plan' in result
        assert len(result['plan']) == 1
        assert verify_plan(result)
        print("✓ Plan with prefix works")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Test 3: Array format (auto-wrapped)
    print("\n3. Testing array format:")
    plan_text = '[{"type": "move", "target": "north"}, {"type": "think", "value": "test"}]'
    try:
        result = parse_plan_json(plan_text)
        assert 'plan' in result
        assert len(result['plan']) == 2
        assert verify_plan(result)
        print("✓ Array format works (auto-wrapped)")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Test 4: While loop
    print("\n4. Testing while loop:")
    plan_text = '{"plan": [{"type": "while", "condition": {"type": "notnear", "target": "Well"}, "body": [{"type": "move", "target": "north"}]}]}'
    try:
        result = parse_plan_json(plan_text)
        assert result['plan'][0]['type'] == 'while'
        assert 'condition' in result['plan'][0]
        assert 'body' in result['plan'][0]
        assert verify_plan(result)
        print("✓ While loop works")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Test 5: If-then-else
    print("\n5. Testing if-then-else:")
    plan_text = '{"plan": [{"type": "if", "condition": {"type": "near", "target": "Well"}, "then": [{"type": "think", "value": "at well"}], "else": [{"type": "move", "target": "north"}]}]}'
    try:
        result = parse_plan_json(plan_text)
        assert result['plan'][0]['type'] == 'if'
        assert 'then' in result['plan'][0]
        assert 'else' in result['plan'][0]
        assert verify_plan(result)
        print("✓ If-then-else works")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Test 6: Scan action with variable binding
    print("\n6. Testing scan action:")
    plan_text = '{"plan": [{"type": "scan", "target": "Berries", "out": "found_berries", "prediction": "find berries"}]}'
    try:
        result = parse_plan_json(plan_text)
        assert result['plan'][0]['type'] == 'scan'
        assert result['plan'][0]['out'] == 'found_berries'
        assert verify_plan(result)
        print("✓ Scan action works")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Test 7: Take action
    print("\n7. Testing take action:")
    plan_text = '{"plan": [{"type": "take", "target": "Berries", "prediction": "add to inventory"}]}'
    try:
        result = parse_plan_json(plan_text)
        assert result['plan'][0]['type'] == 'take'
        assert verify_plan(result)
        print("✓ Take action works")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Test 8: Use action
    print("\n8. Testing use action:")
    plan_text = '{"plan": [{"type": "use", "target": "Berries", "reason": "eat", "out": "result", "prediction": "reduce hunger"}]}'
    try:
        result = parse_plan_json(plan_text)
        assert result['plan'][0]['type'] == 'use'
        assert verify_plan(result)
        print("✓ Use action works")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Test 9: Say action
    print("\n9. Testing say action:")
    plan_text = '{"plan": [{"type": "say", "target": "Joe", "value": "Hello, how are you?"}]}'
    try:
        result = parse_plan_json(plan_text)
        assert result['plan'][0]['type'] == 'say'
        assert verify_plan(result)
        print("✓ Say action works")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    # Test 10: Empty plan
    print("\n10. Testing empty plan:")
    plan_text = '{"plan": []}'
    try:
        result = parse_plan_json(plan_text)
        assert result == {"plan": []}
        assert verify_plan(result)
        print("✓ Empty plan works")
    except Exception as e:
        print(f"✗ Failed: {e}")
    
    print("\n✅ All JSON plan tests completed")

if __name__ == '__main__':
    test_parse_plan_json()

