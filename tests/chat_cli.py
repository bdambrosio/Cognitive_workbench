#!/usr/bin/env python3
"""
Simple CLI chat demo using Cognitive Workbench Zenoh APIs.

Assumes Jill is running. Uses:
- execute_plan_sync API to create Notes
- goal API (via sense_data) to trigger Jill's planning
- plan_result subscription to receive responses

Usage:
    python tests/chat_cli.py
"""

import json
import sys
import threading
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
    """Summarize conversation and archive to pastConversations."""
    print("\n📝 Summarizing conversation...")
    plan_steps = [
        {"type": "load", "resource_id": "conversation", "out": "$conv"},
        {"type": "summarize", "target": "$conv", "out": "$summary"},
        {"type": "load", "resource_id": "pastConversations", "out": "$past"},
        {"type": "add", "target": "$past", "value": "$summary", "out": "$past"}
    ]
    result = execute_plan(session, plan_steps)
    if result.get("success"):
        print("✓ Conversation archived")
    else:
        print(f"✗ Archive failed: {result.get('error', result.get('reason', 'unknown'))}")


def main():
    print("🐱 Silly Kitten Chat Demo")
    print("=" * 40)
    print(f"Connecting to {CHARACTER}...")
    
    config = zenoh.Config()
    session = zenoh.open(config)
    
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
    
    # Create profile Note on startup
    print(f"Creating profile Note...")
    result = create_note(session, PROFILE_CONTENT, name="profile")
    if result.get("success"):
        print(f"✓ Profile created: {PROFILE_CONTENT}")
    else:
        print(f"✗ Failed to create profile: {result.get('error', result.get('reason', 'unknown'))}")
    result = execute_plan(session, [{"type": "create-collection", "name": "conversation", "out": "$conversation"}])
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
            
            # Step 2: Send goal to Jill
            
            goal_text = (
                f"goal:\n"
                f"  subject: agent_response_generation\n"
                f"  input_context: The user just said: '{user_input}'\n"
                f"  conversation_history: see Collection named 'conversation'" 
                f"\n"
                f"  task: Generate the optimal response by strictly following this reasoning chain:\n"
                f"    1. CONTEXT ANALYSIS: Analyze the input against the conversation history. Is this a subject change or a follow-up?\n"
                f"    2. INTENT CLASSIFICATION: Identify what the user wants. Is this social chitchat, or is there an implicit/explicit request for data, calculation, or action?\n"
                f"    3. TOOL FEASIBILITY CHECK: Review your available tools. Does satisfying the user's intent require or benefit from a tool? (e.g., semantic-scholar, calculator, etc.). *Do not assume you are tool-less.*\n"
                f"    4. EXECUTION STRATEGY: \n"
                f"       - If a tool is needed: Define the tool input and use it.\n"
                f"       - If no tool is needed: Draft a response based on your internal knowledge.\n"
                f"    5. PERSONA APPLICATION: Refine the final output to match your personality.\n"
                f"    6. FINAL ACTION: Use 'say' to deliver the response."
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

