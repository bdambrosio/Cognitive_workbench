#!/usr/bin/env python3
"""
Simple test to verify executive node plan execution
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from plan import parse_plan_text, verify_plan
from executive_node import ZenohExecutiveNode

def test_executive_plan_execution():
    """Test that executive node can execute plans correctly."""
    print("Testing executive node plan execution...")
    
    # Create a simple executive node
    executive = ZenohExecutiveNode("test_character")
    
    # Test 1: Simple action plan
    print("\n1. Testing simple action plan execution:")
    plan_text = "plan: move(north, 1)"
    try:
        parsed_plan = parse_plan_text(plan_text)
        print(f"Parsed plan: {parsed_plan}")
        
        # Set the plan in executive
        executive.current_plan = parsed_plan
        executive.current_goal = None  # No goal for this test
        executive.plan_state = {'step_stack': plan.Stack()}
        
        # Execute the plan step
        action = executive._plan_step(parsed_plan)
        print(f"Executed action: {action}")
        
        assert action is not None
        assert action['action'] == 'move'
        assert action['target'] == 'north'
        assert action['value'] == '1'
        print("✓ Simple action plan executed correctly")
        
        # Execute again - should return None (plan complete)
        action2 = executive._plan_step(parsed_plan)
        print(f"Second execution: {action2}")
        assert action2 is None
        print("✓ Plan completion detected correctly")
        
    except Exception as e:
        print(f"✗ Simple action plan failed: {e}")
    
    # Test 2: Multi-step plan
    print("\n2. Testing multi-step plan execution:")
    plan_text = """plan:
  move(north, 1)
  look()"""
    try:
        parsed_plan = parse_plan_text(plan_text)
        print(f"Parsed plan: {parsed_plan}")
        
        # Set the plan in executive
        executive.current_plan = parsed_plan
        executive.current_goal = None
        executive.plan_state = {'step_stack': plan.Stack()}
        
        # Execute first step
        action1 = executive._plan_step(parsed_plan)
        print(f"First action: {action1}")
        assert action1 is not None
        assert action1['action'] == 'move'
        
        # Execute second step
        action2 = executive._plan_step(parsed_plan)
        print(f"Second action: {action2}")
        assert action2 is not None
        assert action2['action'] == 'look'
        
        # Execute third step - should be None (plan complete)
        action3 = executive._plan_step(parsed_plan)
        print(f"Third action: {action3}")
        assert action3 is None
        print("✓ Multi-step plan executed correctly")
        
    except Exception as e:
        print(f"✗ Multi-step plan failed: {e}")
    
    # Test 3: Do-while plan (simplified test)
    print("\n3. Testing do-while plan structure:")
    plan_text = """plan:
  do:
    move(north, 1)
  while(near(Well23))"""
    try:
        parsed_plan = parse_plan_text(plan_text)
        print(f"Parsed plan: {parsed_plan}")
        
        # Verify the plan structure
        assert len(parsed_plan['plan']) == 1
        assert parsed_plan['plan'][0]['type'] == 'do_while'
        assert parsed_plan['plan'][0]['condition'] == 'near(Well23)'
        assert len(parsed_plan['plan'][0]['body']) == 1
        assert parsed_plan['plan'][0]['body'][0]['action'] == 'move'
        print("✓ Do-while plan structure correct")
        
    except Exception as e:
        print(f"✗ Do-while plan failed: {e}")

def main():
    """Run the test."""
    print("=" * 60)
    print("EXECUTIVE NODE PLAN EXECUTION TEST")
    print("=" * 60)
    
    test_executive_plan_execution()
    
    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)

if __name__ == "__main__":
    main() 