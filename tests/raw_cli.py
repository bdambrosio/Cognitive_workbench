#!/usr/bin/env python3
"""
Raw CLI for SGLang OpenAI-Compatible API

Simple CLI that sends text to the API server and displays responses.
Uses the OpenAI-compatible API on port 5000.

Usage:
    python tests/raw_cli.py
    python tests/raw_cli.py --model qwen3-next-80b
    python tests/raw_cli.py --url http://localhost:5000
"""

import json
import sys
import argparse
import requests
from typing import Optional


def get_model_id(url: str) -> Optional[str]:
    """Get model ID from /v1/models endpoint."""
    try:
        response = requests.get(f"{url}/v1/models", timeout=5.0)
        response.raise_for_status()
        data = response.json()
        if data.get("data") and len(data["data"]) > 0:
            return data["data"][0]["id"]
    except Exception as e:
        print(f"[WARNING] Failed to get model ID: {e}")
    return None


def chat_completion(url: str, model: str, messages: list, temperature: float = 0.7, max_tokens: int = 2000):
    """Send chat completion request."""
    payload = {
        "model": model,
        "messages": messages,
        "temperature": temperature,
        "max_tokens": max_tokens
    }
    
    try:
        response = requests.post(
            f"{url}/v1/chat/completions",
            json=payload,
            headers={"Content-Type": "application/json"},
            timeout=300.0
        )
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"[ERROR] Request failed: {e}")
        if hasattr(e, 'response') and e.response is not None:
            try:
                error_data = e.response.json()
                print(f"[ERROR] Response: {json.dumps(error_data, indent=2)}")
            except:
                print(f"[ERROR] Response text: {e.response.text}")
        return None


def main():
    parser = argparse.ArgumentParser(description="Raw CLI for SGLang OpenAI API")
    parser.add_argument("--url", default="http://localhost:5000", help="API server URL")
    parser.add_argument("--model", default=None, help="Model ID (auto-detected if not provided)")
    parser.add_argument("--temperature", type=float, default=0.7, help="Temperature (default: 0.7)")
    parser.add_argument("--max-tokens", type=int, default=2000, help="Max tokens (default: 2000)")
    args = parser.parse_args()
    
    # Get model ID if not provided
    model_id = args.model
    if not model_id:
        print(f"Connecting to {args.url}...")
        model_id = get_model_id(args.url)
        if not model_id:
            print("[ERROR] Could not determine model ID. Please specify with --model")
            sys.exit(1)
        print(f"✓ Detected model: {model_id}")
    else:
        print(f"Using model: {model_id}")
    
    print("\nType your messages (Ctrl+C or 'quit' to exit):")
    print("-" * 60)
    
    conversation = []
    
    while True:
        try:
            user_input = input("\nYou: ").strip()
            if not user_input:
                continue
            
            if user_input.lower() in ('quit', 'exit', 'q'):
                print("\nGoodbye!")
                break
            
            # Add user message to conversation
            conversation.append({"role": "user", "content": user_input})
            
            # Send request
            print("Thinking...")
            response = chat_completion(
                url=args.url,
                model=model_id,
                messages=conversation,
                temperature=args.temperature,
                max_tokens=args.max_tokens
            )
            
            if response:
                # Extract assistant response
                if "choices" in response and len(response["choices"]) > 0:
                    assistant_message = response["choices"][0]["message"]["content"]
                    print(f"\nAssistant: {assistant_message}")
                    
                    # Add assistant response to conversation
                    conversation.append({"role": "assistant", "content": assistant_message})
                else:
                    print(f"\n[ERROR] Unexpected response format:")
                    print(json.dumps(response, indent=2))
            else:
                print("\n[ERROR] No response received")
                
        except KeyboardInterrupt:
            print("\n\nGoodbye!")
            break
        except EOFError:
            print("\n\nGoodbye!")
            break


if __name__ == "__main__":
    main()

