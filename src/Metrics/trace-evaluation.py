from __future__ import annotations

from enum import Enum
import random
import subprocess
import time
import argparse
import traceback
import yaml
from weakref import WeakValueDictionary
import os, sys
from datetime import timedelta, datetime
import logging
import json
import numpy as np
import zenoh

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from llm_client import ZenohLLMClient

from Messages import SystemMessage
from activity import load_scenario
# Type checking imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from executive_node import ZenohExecutiveNode

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)

# Ensure logs directory exists
os.makedirs('logs', exist_ok=True)
# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/trace-evaluation.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('activity')

# Add dedicated handler for llm_api logger
llm_api_logger = logging.getLogger('llm_api')
llm_api_file_handler = logging.FileHandler('logs/llm_api.log', mode='a')
llm_api_file_handler.setLevel(logging.INFO)
llm_api_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', '%Y-%m-%d %H:%M:%S'))
llm_api_logger.addHandler(llm_api_file_handler)
llm_api_logger.setLevel(logging.INFO)


COHERENCE_TEMPLATE = """You are evaluating a sequence of goals and actions taken by a character in a story-like environment.

#SETTING
{{$setting}}

#DRIVES
{{$drives}}

#TRACE
{{$trace}}

Task:
Rate how coherent and causally consistent the sequence is.

Guidelines:
- Later goals/actions should follow plausibly from earlier ones and the evolving world state.
- Time and space should be respected (no teleporting, skipping of prerequisites).
- Termination conditions should be met before the agent moves on.

Scoring (1.0–7.0, decimals allowed):
1.0–2.0 = incoherent, actions often random, many contradictions or missing preconditions
3.0–4.0 = weak coherence, some links, but several implausible jumps or unmet goals
5.0–6.0 = good coherence, most actions make sense, only minor plausibility gaps
7.0 = exceptional coherence, unusually natural and consistent for this domain

Use decimals (e.g., 4.3, 5.7) to capture nuance.

Output format:
coherence_score: [1.0–7.0]
rationale: [2–4 sentences, citing action indices if possible]
flags: [contradictions | missing_preconditions | temporal_jumps | unmet_terminations | none]
"""

ALIGNMENT_TEMPLATE = """You are evaluating whether the sequence of actions aligns with the character’s drives.

#SETTING
{{$setting}}

#DRIVES
{{$drives}}

#TRACE
{{$trace}}

Task:
For each drive, classify actions as advancing, conflicting, or irrelevant. Summarize overall alignment.

Scoring (1.0–7.0):
1.0 = sequence mostly conflicts with drives
3.0 = weak alignment, many irrelevant or drifting actions
5.0 = moderate alignment, some consistent pursuit with small conflicts
7.0 = strong alignment, actions consistently advance stated drives

Output format:
alignment_score: [1.0–7.0]
per_drive_counts: {drive1: {advance:X, conflict:Y, irrelevant:Z}, ...}
rationale: [2–4 sentences]
flags: [drive_conflict_unresolved | aimless_drift | none]
"""
REALISM_AND_RISK_TEMPLATE = """You are evaluating whether the sequence reflects realistic survival and common-sense behavior in the setting.

[SETTING]
[DRIVES]
[TRACE]

Task:
Judge how plausible the sequence is with respect to environment and risk:
- Are food, water, and rest addressed at believable intervals?
- Are scans, movements, and tool uses appropriate to terrain and resources?
- Are risky behaviors avoided (e.g., ignoring thirst, wandering aimlessly without supplies)?

Scoring (1.0–7.0):
1.0 = unrealistic or reckless
3.0 = somewhat implausible, major gaps in basic survival logic
5.0 = generally realistic, minor implausibilities
7.0 = strongly realistic and survival-aware

Output format:
realism_score: [1.0–7.0]
rationale: [2–4 sentences]
flags: [resource_neglect | over_caution | reckless | affordance_misuse | none]
"""
MOMENTUM_TEMPLATE = """You are evaluating whether the sequence shows progress or gets stuck in loops.

#SETTING
{{$setting}}

#DRIVES
{{$drives}}

#TRACE
{{$trace}}
Task:
Identify whether the character repeatedly cycles through actions without advancing, or whether momentum toward larger goals is maintained.

Scoring (1.0–7.0):
1 = little or no forward progress
3 = some progress but substantial repetition
5 = mostly forward momentum with occasional loops
7 = steady progress with minimal or no looping

Output format:
momentum_score: [1.0–7.0]
loop_segments: [list indices of any repeated/looping spans, else none]
rationale: [2–4 sentences]
"""
COMPARATIVE_TEMPLATE = """You are comparing three anonymized sequences of actions by the same character under the same setting.

[SETTING]
[DRIVES]
Sequence A: [TRACE_A]
Sequence B: [TRACE_B]
Sequence C: [TRACE_C]

Task:
Rank the three sequences for overall believability and narrative coherence, 1 = best, 3 = worst. Give 2–3 sentences of justification for each.

Output format:
rankings: {A: , B: , C: }
justifications: {A: "...", B: "...", C: "..."}
"""



def main():
    parser = argparse.ArgumentParser(description="Scenario analysis driver for ontology and activities generation")
    parser.add_argument('--trace', required=True, help='Path to trace JSON file')
    parser.add_argument('--scenario', required=True, help='Path to scenario YAML file')
    parser.add_argument('--character', default='all', help="Character name or 'all'")
    parser.add_argument('--server', default='openai', help='LLM server name (for middle ontology/planner)')
    parser.add_argument('--model', default='gpt-4.1', help='LLM model name (for middle ontology/planner)')
    args = parser.parse_args()

    # Load scenario
    scenario = load_scenario(args.scenario)
    if not scenario:
        logger.error("Failed to load scenario")
        sys.exit(1)

    setting = scenario.get('setting', '') or "modern era, western, moderate climate"
    characters = scenario.get('characters', {}) or {}
    if not characters:
        logger.error("No characters in scenario")
        sys.exit(1)

    scenario_dir = os.path.dirname(args.scenario) or '.'
    logger.info(f"Scenario: {args.scenario}")
    logger.info(f"Setting: {setting}")
    logger.info(f"Characters: {', '.join(characters.keys())}")
    character_names = list(characters.keys())

    # Initialize Zenoh session (reused for map/types)
    session = zenoh.open(zenoh.Config())

    # Initialize LL
    llm_client = ZenohLLMClient(server_name=args.server, model_name=args.model, service_timeout=240.0)
    # Helper: run per character
    def process_character(name: str, data: dict):
        logger.info(f"Processing character: {name}")
        if data.get('manual', False):
            logger.info(f"Skipping manual character: {name}")
            return
        character_desc = (data.get('character', '') + '\n' + data.get('status', '')).strip()
        character_drives = data.get('drives', [])
        if not character_desc:
            logger.error(f"No character description for {name}")
            raise RuntimeError("missing character description")

        try:
            # Load trace from JSON or JSONL file path
            trace_str = ''
            trace_items = []
            trace_path = args.trace
            if isinstance(trace_path, str) and trace_path.lower().endswith('.jsonl'):
                with open(trace_path, 'r', encoding='utf-8') as tf:
                    for line in tf:
                        s = line.strip()
                        if not s:
                            continue
                        obj = json.loads(s)
                        trace_items.append(obj)
            else:
                with open(trace_path, 'r', encoding='utf-8') as tf:
                    data = json.load(tf)
                    if isinstance(data, list):
                        trace_items = data
                    else:
                        trace_items = [data]

            for i, item in enumerate(trace_items):
                goal = (item.get('goal') if isinstance(item, dict) else None) or (item.get('current_goal') if isinstance(item, dict) else '') or ''
                if goal:
                    trace_str += f"\n{i} #GOAL: {goal}\n"
                if item.get('summary') and isinstance(item.get('summary'), dict):
                    trace_str += f"\n{i} SUMMARY:\n {item.get('summary', {}).get('summary')}\n"
                    trace_str += f"\n{i} OUTCOME:\n {item.get('summary', {}).get('outcome')}\n"

                actions = (item.get('actions') if isinstance(item, dict) else None) or []
                for act in actions:
                    a = act.get('action') if isinstance(act, dict) else act
                    trace_str += f"\n\t act: {str(a)}\n"
                trace_str += f"##\n\n"

            response = llm_client.llm.ask(
                {"setting": setting, "drives": character_drives, "trace": trace_str},
                [SystemMessage(content=COHERENCE_TEMPLATE)],
                max_tokens=2000,
                #trace=True
                
            )
            print(response)
            response = llm_client.llm.ask(
                {"setting": setting, "drives": character_drives, "trace": trace_str},
                [SystemMessage(content=ALIGNMENT_TEMPLATE)],
                max_tokens=2000,
                
            )
            print(response)
            response = llm_client.llm.ask(
                {"setting": setting, "drives": character_drives, "trace": trace_str},
                [SystemMessage(content=REALISM_AND_RISK_TEMPLATE)],
                max_tokens=2000,
                
            )
            print(response)
            response = llm_client.llm.ask(
                {"setting": setting, "drives": character_drives, "trace": trace_str},
                [SystemMessage(content=MOMENTUM_TEMPLATE)],
                max_tokens=2000,
                
            )
            print(response)
        except Exception as e:
            logger.error(f"Failed to load trace: {e}")
            raise RuntimeError("failed to load trace")
        logger.info(f"Completed: {name}")
    # Character selection
    names = list(characters.keys()) if args.character == 'all' else [args.character]
    try:
        for cname in names:
            if cname not in characters:
                raise KeyError(f"Character not in scenario: {cname}")
            process_character(cname, characters[cname])
    except Exception as e:
        logger.error(f"Scenario run failed: {e}")
        sys.exit(1)

    logger.info("Scenario analysis complete")
    # Cleanup launched map_node if we started it

if __name__ == '__main__':
    main()


