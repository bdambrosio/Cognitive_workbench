#!/usr/bin/env python3
"""
Simple CLI chat demo using Cognitive Workbench Zenoh APIs.

Assumes Jill is running. Uses:
- execute_plan_sync API to create Notes
- goal API (via sense_data) to trigger Jill's planning
- plan_result subscription to receive responses

Usage:
    python tests/chat_cli.py [--scenario SCENARIO_FILE]
"""

import argparse
import json
import os
import sys
import threading
import yaml
import zenoh

CHARACTER = "Jill"
PROFILE_CONTENT = """"""


def execute_plan(session, plan_steps: list) -> dict:
    """Execute a plan synchronously via Zenoh API."""
    selector = f"cognitive/{CHARACTER}/execute_plan_sync"
    payload = json.dumps({"plan": plan_steps}).encode('utf-8')
    replies = session.get(selector, payload=payload, target=zenoh.QueryTarget.BEST_MATCHING, consolidation=zenoh.ConsolidationMode.NONE,timeout=60.0)
    for reply in replies:
        if hasattr(reply, 'ok') and reply.ok is not None:
            return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
    return {"success": False, "error": "No reply"}


def create_note(session, content: str, name: str = "", add_to_conversation: bool = False) -> dict:
    """Create a Note via execute_plan_sync API."""
    actions = []
    action = {"type": "create-note", "value": content, "out": "$note"}
    actions.append(action)
    if add_to_conversation:
        action = {"type": "add", "target": "conversation", "value": "$note", "out": "conversation"}
        actions.append(action)
    if name:
        action["name"] = name
    return execute_plan(session, actions)


def send_goal(session, goal_text: str):
    """Send a goal to Jill via sense_data topic."""
    sense_data = {
        "timestamp": "",
        "sequence_id": 0,
        "mode": "text",
        "content": json.dumps({"source": "User", "text": f"goal: {goal_text}"})
    }
    session.put(f"cognitive/{CHARACTER}/sense_data", json.dumps(sense_data).encode('utf-8'))


def shutdown_conversation(session):
    """Summarize conversation and archive to conversation_history."""
    print("\n📝 Summarizing conversation...")
    plan_steps = [
        {"type": "load", "target": "conversation", "out": "$conv"},
        {"type": "summarize", "target": "$conv", "out": "$summary"},
        {"type": "load", "target": "conversation_history", "out": "$past"},
        {"type": "add", "target": "$past", "value": "$summary", "out": "$past"}
    ]
    result = execute_plan(session, plan_steps)
    if result.get("success"):
        print("✓ Conversation archived")
    else:
        print(f"✗ Archive failed: {result.get('error', result.get('reason', 'unknown'))}")


def main():
    parser = argparse.ArgumentParser(description='CLI chat demo using Cognitive Workbench Zenoh APIs')
    parser.add_argument('--scenario', type=str, default=None, help='Path to scenario YAML file (e.g., scenarios/jill.yaml)')
    args = parser.parse_args()
    
    print("🐱 Silly Kitten Chat Demo")
    print("=" * 40)
    print(f"Connecting to {CHARACTER}...")
    
    config = zenoh.Config()
    session = zenoh.open(config)
    
    # Load scenario if provided and create note
    scenario_note_name = None
    if args.scenario:
        try:
            scenario_path = args.scenario
            if not os.path.isabs(scenario_path):
                # Try relative to project root
                project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                scenario_path = os.path.join(project_root, scenario_path)
            
            with open(scenario_path, 'r') as f:
                scenario_data = yaml.safe_load(f)
            
            characters = scenario_data.get('characters', {})
            if characters:
                # Get first character
                first_char_name = list(characters.keys())[0]
                char_data = characters[first_char_name]
                
                character_desc = char_data.get('character', '').strip()
                backstory = char_data.get('backstory', '').strip()
                drives = char_data.get('drives', [])
                
                # Format drives
                if isinstance(drives, list):
                    drives_text = '\n'.join(f"- {d}" for d in drives)
                else:
                    drives_text = str(drives)
                
                # Build scenario content
                scenario_content = f"You are {first_char_name}\n\n"
                if character_desc:
                    scenario_content += f"{character_desc}\n\n"
                if backstory:
                    scenario_content += f"{backstory}\n\n"
                if drives_text:
                    scenario_content += f"{drives_text}\n\n"
                scenario_content += "respond to User, who just said:\n\n"
                
                # Create note with scenario content
                scenario_note_name = "scenario_context"
                result = create_note(session, scenario_content, name=scenario_note_name)
                if result.get("success"):
                    print(f"✓ Created scenario note: {scenario_note_name}")
                else:
                    print(f"✗ Failed to create scenario note: {result.get('error', result.get('reason', 'unknown'))}")
                    scenario_note_name = None
        except Exception as e:
            print(f"[WARNING] Failed to load scenario: {e}")
            scenario_note_name = None
    
    # Response state
    response_received = threading.Event()
    response_data = {}
    
    def plan_result_callback(sample):
        """Handle plan_result messages from Jill."""
        nonlocal response_data
        data = json.loads(sample.payload.to_bytes().decode('utf-8'))
        print(f"\n[DEBUG] plan_result: status={data.get('status')}")
        if data.get('final_content'):
            print(f"[DEBUG] final_content: {data.get('final_content')[:200]}...")
        response_data = data
        response_received.set()
    
    # Subscribe to plan_result
    plan_result_sub = session.declare_subscriber(
        f"cognitive/{CHARACTER}/plan_result",
        plan_result_callback
    )
    
    # Create conversation collection on startup
    result = execute_plan(session, [
        {"type": "load", "target": "conversation_history", "out": "$conversation_history"},
        {"type": "if", "condition": {"type": "notbound", "target": "$conversation_history"}, 
                        "then": [{"type": "create-collection","name": "conversation_history","out": "$conversation_history"},
                                {"type": "index","target": "$conversation_history","store_name": "conversation_history","index_type": "semantic"},
                                {"type": "persist", "target": "$conversation_history"}
                                ] }])
    if result.get("success"):
        print(f"✓ Conversation_history collection created")
    else:
        print(f"✗ Failed to create conversation_history collection: {result.get('error', result.get('reason', 'unknown'))}")
  
    print(f"Creating conversation collection...")
    result = execute_plan(session, [{"type": "create-collection", "name": "conversation", "out": "$conversation"},
                                    {"type": "index", "target": "$conversation", "index_type": "semantic"}])
    if result.get("success"):
        print(f"✓ Conversation collection created")
    else:
        print(f"✗ Failed to create conversation collection: {result.get('error', result.get('reason', 'unknown'))}")
    
    print("\nType your messages (Ctrl+C to exit):")
    print("-" * 40)
    
    turn = 0
    while True:
        try:
            user_input = input("\nYou: ").strip()
            if not user_input:
                continue
            
            if user_input.lower() == 'shutdown':
                shutdown_conversation(session)
                break
            
            turn += 1
            
            # Step 1: Create Note with user's message
            user_note_content = f"User says: {user_input}"
            result = create_note(session, user_note_content, add_to_conversation=True)
            if not result.get("success"):
                print(f"[ERROR] Failed to create user note: {result}")
                continue
            
            # Step 2: Format goal text with scenario note load instruction if applicable
            if scenario_note_name:
                goal_text = (
                    f"First load the note '{scenario_note_name}' to get context about your character and role.\n\n"
                    f"{user_input}\n\n"
                )
            else:
                goal_text = (
                    f"{user_input}\n\n"
                )
            
            goal_text += (
                f"Context: the 'conversation' Collection contains the history of this conversation.\n"
                f"  the 'conversation_history' Collection contains summaries of past conversations.\n\n"
                f"When crafting your response:\n"
                f"- If the user needs information you don't have, or you would benefit from thinking more deeply about user's input, use tools to find it\n"
                f"- Use semantic-scholar for academic papers, query-web for web info, calculator for math\n"
                f"- After gathering information (if needed), use 'say' to deliver your response\n"
                f"- Match your personality in tone and style"
            )
            response_received.clear()
            response_data = {}
            
            print("Jill is thinking...")
            send_goal(session, goal_text)
            
            # Step 3: Wait for response
            if response_received.wait(timeout=120.0):
                final_content = response_data.get("final_content", "")
                
                if final_content:
                    print(f"\nJill: {final_content}")
                    
                    # Create Note with Jill's response
                    jill_note_content = f"Jill responds: {final_content}"
                    create_note(session, jill_note_content, add_to_conversation=True)

                else:
                    print("\n[No response content received]")
                    print(f"[DEBUG] Full response: {response_data}")
            else:
                print("\n[Timeout waiting for response]")
                
        except KeyboardInterrupt:
            shutdown_conversation(session)
            print("\n\nGoodbye! 🐱")
            break
        except EOFError:
            print("\n\nGoodbye! 🐱")
            break
    
    plan_result_sub.undeclare()
    session.close()


if __name__ == "__main__":
    main()

