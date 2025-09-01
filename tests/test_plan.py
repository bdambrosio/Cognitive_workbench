#!/usr/bin/env python3
"""
Test file for plan.py parse_plan_text and verify_plan methods
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from plan import parse_plan_text, verify_plan

def test_parse_plan_text():
    """Test the parse_plan_text function with various inputs."""
    print("Testing parse_plan_text...")
    
    # Test 1: Single line plan
    print("\n1. Testing single line plan:")
    plan_text = "plan: move(north)"
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {plan_text}")
        print(f"Result: {result}")
        assert 'plan' in result
        assert len(result['plan']) == 1
        assert result['plan'][0]['type'] == 'move'
        assert result['plan'][0]['target'] == 'north'
        print("✓ Single line plan parsed correctly")
        result = verify_plan(result)
        print(f"Verified: {result}")
        assert result == True
        print("✓ Single line plan verified correctly")
    except Exception as e:
        print(f"✗ Single line plan failed: {e}")
    
    # Test 2: Multi-line simple plan
    print("\n2. Testing multi-line simple plan:")
    plan_text = """plan:
  move(north)
  look()"""
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {repr(plan_text)}")
        print(f"Result: {result}")
        assert 'plan' in result
        assert len(result['plan']) == 2
        assert result['plan'][0]['type'] == 'move'
        assert result['plan'][1]['type'] == 'look'
        print("✓ Multi-line simple plan parsed correctly")
        result = verify_plan(result)
        print(f"Verified: {result}")
        assert result == True
        print("✓ Multi-line simple plan verified correctly")
    except Exception as e:
        print(f"✗ Multi-line simple plan failed: {e}")
    
    # Test 3: Do-while loop
    print("\n3. Testing do-while loop:")
    plan_text = """plan:
  do:
    move(north, 1)
    look()
  while(near(Well23))"""
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {repr(plan_text)}")
        print(f"Result: {result}")
        assert 'plan' in result
        assert len(result['plan']) == 1
        assert result['plan'][0]['type'] == 'do_while'
        assert result['plan'][0]['condition']['type'] == 'near'
        assert result['plan'][0]['condition']['target'] == 'Well23'
        assert len(result['plan'][0]['body']) == 2
        assert result['plan'][0]['body'][0]['type'] == 'move'
        assert result['plan'][0]['body'][1]['type'] == 'look'
        print("✓ Do-while loop parsed correctly")
        result = verify_plan(result)
        print(f"Verified: {result}")
        assert result == True
        print("✓ Do-while loop verified correctly")
    except Exception as e:
        print(f"✗ Do-while loop failed: {e}")
    
    # Test 4: If-then-else
    print("\n4. Testing if-then-else:")
    plan_text = """plan:
  if:
    near(Well23)
  else:
    move(north, 1)
  endif:"""
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {repr(plan_text)}")
        print(f"Result: {result}")
        assert 'plan' in result
        assert len(result['plan']) == 1
        assert result['plan'][0]['type'] == 'if'
        assert result['plan'][0]['condition']['type'] == 'near'
        assert result['plan'][0]['condition']['target'] == 'Well23'
        assert 'then' in result['plan'][0]
        assert 'else' in result['plan'][0]
        assert len(result['plan'][0]['then']) == 0  # No then actions
        assert len(result['plan'][0]['else']) == 1  # One else action
        print("✓ If-then-else parsed correctly")
        result = verify_plan(result)
        print(f"Verified: {result}")
        assert result == True
        print("✓ If-then-else verified correctly")
    except Exception as e:
        print(f"✗ If-then-else failed: {e}")
    
    # Test 4b: If-then-else with belief condition
    print("\n4b. Testing if-then-else with belief condition:")
    plan_text = """plan:
  if:
    near(Well23)
    say(joe, "I'm near the well")
  else:
    move(north, 1)
  endif:"""
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {repr(plan_text)}")
        print(f"Result: {result}")
        assert 'plan' in result
        assert len(result['plan']) == 1
        assert result['plan'][0]['type'] == 'if'
        assert result['plan'][0]['condition']['type'] == 'near'
        assert result['plan'][0]['condition']['target'] == 'Well23'
        assert 'then' in result['plan'][0]
        assert 'else' in result['plan'][0]
        assert len(result['plan'][0]['then']) == 1  # One then action (say)
        assert len(result['plan'][0]['else']) == 1  # One else action
        print("✓ If-then-else parsed correctly")
        result = verify_plan(result)
        print(f"Verified: {result}")
        assert result == True
        print("✓ If-then-else verified correctly")
    except Exception as e:
        print(f"✗ If-then-else failed: {e}")
    
    # Test 5: Empty plan
    print("\n5. Testing empty plan:")
    plan_text = "plan:"
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {plan_text}")
        print(f"Result: {result}")
        assert result == {"plan": []}
        print("✓ Empty plan parsed correctly")
        result = verify_plan(result)
        print(f"Verified: {result}")
        assert result == True
        print("✓ Empty plan verified correctly")
    except Exception as e:
        print(f"✗ Empty plan failed: {e}")
    
    # Test 6: Invalid plan (no 'plan:' prefix)
    print("\n6. Testing invalid plan (no prefix):")
    plan_text = "move(north, 1)"
    try:
        result = parse_plan_text(plan_text)
        print(f"✗ Should have failed but got: {result}")
    except ValueError as e:
        print(f"✓ Correctly rejected invalid plan: {e}")
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
    
    # Test 7: Invalid action format
    print("\n7. Testing invalid action format:")
    plan_text = "plan:\n  invalid_action"
    try:
        result = parse_plan_text(plan_text)
        print(f"✗ Should have failed but got: {result}")
    except ValueError as e:
        print(f"✓ Correctly rejected invalid action: {e}")
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
    
    # Test 8: If-then without else
    print("\n8. Testing if-then without else:")
    plan_text = """plan:
  if:
    near(Well23)
    say(joe, found the well!)
  endif:"""
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {repr(plan_text)}")
        print(f"Result: {result}")
        assert 'plan' in result
        assert len(result['plan']) == 1
        assert result['plan'][0]['type'] == 'if'
        assert result['plan'][0]['condition']['type'] == 'near'
        assert result['plan'][0]['condition']['target'] == 'Well23'
        assert 'then' in result['plan'][0]
        assert 'else' not in result['plan'][0]  # No else clause
        assert len(result['plan'][0]['then']) == 1  # One then action
        print("✓ If-then without else parsed correctly")
        result = verify_plan(result)
        print(f"Verified: {result}")
        assert result == True
        print("✓ If-then without else verified correctly")
    except Exception as e:
        print(f"✗ If-then without else failed: {e}")
    
    # Test 9: Parentheses in values (edge case)
    print("\n9. Testing parentheses in values:")
    plan_text = "plan: say(Joe, are you there? (just asking))"
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {plan_text}")
        print(f"Result: {result}")
        # This test is expected to reveal parsing issues
        print("✓ Parentheses in values parsed (may be incorrect)")
    except Exception as e:
        print(f"✗ Parentheses in values failed: {e}")
    
    # Test 10: Commas in values (edge case)
    print("\n10. Testing commas in values:")
    plan_text = "plan: say(Joe, Hello, how are you, my friend?)"
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {plan_text}")
        print(f"Result: {result}")
        # This test is expected to reveal parsing issues
        print("✓ Commas in values parsed (may be incorrect)")
    except Exception as e:
        print(f"✗ Commas in values failed: {e}")
    
    # Test 10b: Say with colon syntax (new feature)
    print("\n10b. Testing say with colon syntax:")
    plan_text = "plan: say(Joe: Hello, how are you, my friend?)"
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {plan_text}")
        print(f"Result: {result}")
        assert 'plan' in result
        assert len(result['plan']) == 1
        assert result['plan'][0]['type'] == 'say'
        assert result['plan'][0]['target'] == 'Joe'
        assert result['plan'][0]['value'] == 'Hello, how are you, my friend?'
        print("✓ Say with colon syntax parsed correctly")
    except Exception as e:
        print(f"✗ Say with colon syntax failed: {e}")
    
    # Test 11: Empty arguments (edge case)
    print("\n11. Testing empty arguments:")
    plan_text = """plan:
  move()
  look(,)
  say(Joe,)
  use(,water)"""
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {repr(plan_text)}")
        print(f"Result: {result}")
        print("✓ Empty arguments parsed (check for correctness)")
    except Exception as e:
        print(f"✗ Empty arguments failed: {e}")
    
    # Test 12: Special characters in names (edge case)
    print("\n12. Testing special characters in names:")
    plan_text = """plan:
  move(north-east)
  say(Joe-Bob, hello)
  use(Well#23)
  near(Player.Name)"""
    try:
        result = parse_plan_text(plan_text)
        print(f"Input: {repr(plan_text)}")
        print(f"Result: {result}")
        print("✓ Special characters in names parsed (check for correctness)")
    except Exception as e:
        print(f"✗ Special characters in names failed: {e}")

def test_verify_plan():
    """Test the verify_plan function with various inputs."""
    print("\n\nTesting verify_plan...")
    
    # Test 1: Valid action plan
    print("\n1. Testing valid action plan:")
    plan = {
        "plan": [
            {
                "type": "move",
                "target": "north",
                "value": "1"
            }
        ]
    }
    try:
        result = verify_plan(plan)
        print(f"Input: {plan}")
        print(f"Result: {result}")
        assert result == True
        print("✓ Valid action plan verified correctly")
    except Exception as e:
        print(f"✗ Valid action plan failed: {e}")
    
    # Test 2: Valid do_while plan
    print("\n2. Testing valid do_while plan:")
    plan = {
        "plan": [
            {
                "type": "do_while",
                "condition": {
                    "type": "near",
                    "target": "Well23",
                    "value": ""
                },
                "body": [
                    {
                        "type": "move",
                        "target": "north",
                        "value": "1"
                    }
                ]
            }
        ]
    }
    try:
        result = verify_plan(plan)
        print(f"Input: {plan}")
        print(f"Result: {result}")
        assert result == True
        print("✓ Valid do_while plan verified correctly")
    except Exception as e:
        print(f"✗ Valid do_while plan failed: {e}")
    
    # Test 3: Valid if plan
    print("\n3. Testing valid if plan:")
    plan = {
        "plan": [
            {
                "type": "if",
                "condition": {
                    "type": "near",
                    "target": "Well23",
                    "value": ""
                },
                "then": [
                    {
                        "type": "use",
                        "target": "water",
                        "value": ""
                    }
                ],
                "else": [
                    {
                        "type": "move",
                        "target": "north",
                        "value": "1"
                    }
                ]
            }
        ]
    }
    try:
        result = verify_plan(plan)
        print(f"Input: {plan}")
        print(f"Result: {result}")
        assert result == True
        print("✓ Valid if plan verified correctly")
    except Exception as e:
        print(f"✗ Valid if plan failed: {e}")
    
    # Test 4: Invalid plan (missing required keys)
    print("\n4. Testing invalid plan (missing keys):")
    plan = {
        "plan": [
            {
                "type": "move"
                # Missing target and value
            }
        ]
    }
    try:
        result = verify_plan(plan)
        print(f"Input: {plan}")
        print(f"Result: {result}")
        assert result == False
        print("✓ Invalid plan correctly rejected")
    except Exception as e:
        print(f"✗ Invalid plan test failed: {e}")
    
    # Test 5: Invalid plan (wrong type)
    print("\n5. Testing invalid plan (wrong type):")
    plan = {
        "plan": [
            {
                "type": "invalid_type",
                "target": "north",
                "value": "1"
            }
        ]
    }
    try:
        result = verify_plan(plan)
        print(f"Input: {plan}")
        print(f"Result: {result}")
        assert result == False
        print("✓ Invalid type correctly rejected")
    except Exception as e:
        print(f"✗ Invalid type test failed: {e}")
    
    # Test 6: JSON string input
    print("\n6. Testing JSON string input:")
    import json
    plan_json = json.dumps({
        "plan": [
            {
                "type": "move",
                "target": "north",
                "value": "1"
            }
        ]
    })
    try:
        result = verify_plan(plan_json)
        print(f"Input: {plan_json}")
        print(f"Result: {result}")
        assert result == True
        print("✓ JSON string input verified correctly")
    except Exception as e:
        print(f"✗ JSON string input failed: {e}")
    
    # Test 7: Invalid JSON string
    print("\n7. Testing invalid JSON string:")
    plan_json = '{"plan": [{"type": "move"'  # Missing closing brace
    try:
        result = verify_plan(plan_json)
        print(f"Input: {plan_json}")
        print(f"Result: {result}")
        assert result == False
        print("✓ Invalid JSON correctly rejected")
    except Exception as e:
        print(f"✗ Invalid JSON test failed: {e}")

def test_integration():
    """Test integration between parse_plan_text and verify_plan."""
    print("\n\nTesting integration...")
    
    # Test 1: Parse then verify a simple plan
    print("\n1. Testing parse then verify simple plan:")
    plan_text = "plan: move(north, 1)"
    try:
        parsed = parse_plan_text(plan_text)
        print(f"Parsed: {parsed}")
        verified = verify_plan(parsed)
        print(f"Verified: {verified}")
        assert verified == True
        print("✓ Parse then verify worked correctly")
    except Exception as e:
        print(f"✗ Parse then verify failed: {e}")
    
    # Test 2: Parse then verify do-while plan
    print("\n2. Testing parse then verify do-while plan:")
    plan_text = """plan:
  do:
    move(north, 1)
  while(near(Well23))"""
    try:
        parsed = parse_plan_text(plan_text)
        print(f"Parsed: {parsed}")
        verified = verify_plan(parsed)
        print(f"Verified: {verified}")
        assert verified == True
        print("✓ Parse then verify do-while worked correctly")
    except Exception as e:
        print(f"✗ Parse then verify do-while failed: {e}")
    
    # Test 3: Parse then verify if-then-else plan
    print("\n3. Testing parse then verify if-then-else plan:")
    plan_text = """plan:
  if:
    near(Well23)
  else:
    move(north, 1)
  endif:"""
    try:
        parsed = parse_plan_text(plan_text)
        print(f"Parsed: {parsed}")
        verified = verify_plan(parsed)
        print(f"Verified: {verified}")
        assert verified == True
        print("✓ Parse then verify if-then-else worked correctly")
    except Exception as e:
        print(f"✗ Parse then verify if-then-else failed: {e}")

def main():
    """Run all tests."""
    print("=" * 60)
    print("PLAN.PY TEST SUITE")
    print("=" * 60)
    
    test_parse_plan_text()
    test_verify_plan()
    test_integration()
    
    print("\n" + "=" * 60)
    print("TEST SUITE COMPLETE")
    print("=" * 60)

if __name__ == "__main__":
    main() 