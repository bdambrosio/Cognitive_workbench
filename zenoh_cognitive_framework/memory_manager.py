#!/usr/bin/env python3
"""
Simple Data Manager for Zenoh Cognitive Framework

Usage:
    python memory_manager.py reset --character Joe          # Clear both memory and situation
    python memory_manager.py reset --character Joe --memory-only
    python memory_manager.py reset --character Joe --situation-only
    python memory_manager.py reset --character *            # Clear all characters
    python memory_manager.py list                          # List all characters with data
"""

import argparse
import json
import sys
from pathlib import Path


def list_characters():
    """List all characters with data files."""
    memory_dir = Path("data/memory")
    situation_dir = Path("data/situation")
    world_dir = Path("data/world")
    
    memory_files = list(memory_dir.glob("*_memory.json")) if memory_dir.exists() else []
    situation_files = list(situation_dir.glob("*_situation.json")) if situation_dir.exists() else []
    world_files = list(world_dir.glob("*_world.json")) if world_dir.exists() else []
    
    if not memory_files and not situation_files and not world_files:
        print("No data files found.")
        return
    
    # Get unique character names
    characters = set()
    for file in memory_files:
        characters.add(file.stem.replace("_memory", ""))
    for file in situation_files:
        characters.add(file.stem.replace("_situation", ""))
    for file in world_files:
        characters.add(file.stem.replace("_world", ""))
    
    print("Characters with data:")
    for character in sorted(characters):
        has_memory = any(f.stem.replace("_memory", "") == character for f in memory_files)
        has_situation = any(f.stem.replace("_situation", "") == character for f in situation_files)
        has_world = any(f.stem.replace("_world", "") == character for f in world_files)
        
        status = []
        if has_memory:
            status.append("memory")
        if has_situation:
            status.append("situation")
        if has_world:
            status.append("world")
        
        print(f"  - {character} ({', '.join(status)})")


def reset_data(character_name, memory_only=False, situation_only=False):
    """Reset data for a specific character or all characters."""
    memory_dir = Path("data/memory")
    situation_dir = Path("data/situation")
    world_dir = Path("data/world")
    
    if character_name == "*":
        # Reset all characters
        memory_files = list(memory_dir.glob("*_memory.json")) if memory_dir.exists() else []
        situation_files = list(situation_dir.glob("*_situation.json")) if situation_dir.exists() else []
        world_files = list(world_dir.glob("*_world.json")) if world_dir.exists() else []
        
        if not memory_files and not situation_files and not world_files:
            print("No data files found to reset.")
            return
        
        files_to_delete = []
        if not situation_only:
            files_to_delete.extend(memory_files)
        if not memory_only:
            files_to_delete.extend(situation_files)
            files_to_delete.extend(world_files)  # Always clear world files when clearing situation
        
        if not files_to_delete:
            print("No files to reset based on specified options.")
            return
        
        print(f"Found {len(files_to_delete)} files to reset:")
        for file in files_to_delete:
            print(f"  - {file.name}")
        
        data_type = "data"
        if memory_only:
            data_type = "memory"
        elif situation_only:
            data_type = "situation"
        
        confirm = input(f"Are you sure you want to reset ALL character {data_type}? (yes/no): ")
        if confirm.lower() != "yes":
            print("Reset cancelled.")
            return
        
        for file in files_to_delete:
            file.unlink()
            print(f"Deleted {file.name}")
        
        print(f"All character {data_type} reset.")
    
    else:
        # Reset specific character
        files_to_delete = []
        
        if not situation_only:
            memory_file = memory_dir / f"{character_name}_memory.json"
            if memory_file.exists():
                files_to_delete.append(memory_file)
            elif not memory_only:
                print(f"No memory file found for character '{character_name}'.")
        
        if not memory_only:
            situation_file = situation_dir / f"{character_name}_situation.json"
            if situation_file.exists():
                files_to_delete.append(situation_file)
            elif not situation_only:
                print(f"No situation file found for character '{character_name}'.")
        
        if not files_to_delete:
            print(f"No data files found for character '{character_name}'.")
            return
        
        data_type = "data"
        if memory_only:
            data_type = "memory"
        elif situation_only:
            data_type = "situation"
        
        confirm = input(f"Are you sure you want to reset {data_type} for '{character_name}'? (yes/no): ")
        if confirm.lower() != "yes":
            print("Reset cancelled.")
            return
        
        for file in files_to_delete:
            file.unlink()
            print(f"Deleted {file.name}")
        
        print(f"{data_type.capitalize()} reset for character '{character_name}'.")


def main():
    parser = argparse.ArgumentParser(description="Data Manager for Zenoh Cognitive Framework")
    parser.add_argument("command", choices=["reset", "list"], help="Command to execute", nargs="?")
    parser.add_argument("--character", help="Character name (use '*' for all characters)")
    parser.add_argument("--memory-only", action="store_true", help="Reset only memory data")
    parser.add_argument("--situation-only", action="store_true", help="Reset only situation data")
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.memory_only and args.situation_only:
        print("Error: Cannot specify both --memory-only and --situation-only")
        sys.exit(1)
    
    # If no command provided, run interactive mode
    if not args.command:
        print("Data Manager - Interactive Mode")
        print("=" * 40)
        
        # List available characters
        list_characters()
        print()
        
        # Ask for character name
        character_name = input("Enter character name to reset (or '*' for all): ").strip()
        
        if not character_name:
            print("No character name provided. Exiting.")
            return
        
        # Ask for reset type
        print("\nReset options:")
        print("1. Both memory and situation (default)")
        print("2. Memory only")
        print("3. Situation only")
        
        choice = input("Enter choice (1-3, default=1): ").strip()
        
        memory_only = False
        situation_only = False
        
        if choice == "2":
            memory_only = True
        elif choice == "3":
            situation_only = True
        
        # Verify character exists (unless it's '*')
        if character_name != "*":
            memory_file = Path("data/memory") / f"{character_name}_memory.json"
            situation_file = Path("data/situation") / f"{character_name}_situation.json"
            
            if not memory_file.exists() and not situation_file.exists():
                print(f"Error: No data files found for character '{character_name}'")
                print("Available characters:")
                list_characters()
                return
        
        reset_data(character_name, memory_only, situation_only)
        return
    
    # Command-line mode
    if args.command == "list":
        list_characters()
    elif args.command == "reset":
        if not args.character:
            print("Error: --character is required for reset command")
            sys.exit(1)
        reset_data(args.character, args.memory_only, args.situation_only)


if __name__ == "__main__":
    main() 