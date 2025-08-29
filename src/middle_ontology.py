from __future__ import annotations

from enum import Enum
import random
import subprocess
import time
import argparse
import yaml
from weakref import WeakValueDictionary
import os, sys
from datetime import timedelta, datetime
import logging
import json
import numpy as np
import zenoh
#from sentence_transformers import SentenceTransformer
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Messages import SystemMessage
from utils import hash_utils
from utils.state_utils import calculate_state_activity_alignment, get_known_states
from templates import PLAN_TEMPLATE, MIDDLE_ONTOLOGY_TEMPLATE
from utils.llm_api import LLM
from activity import load_scenario

# Type checking imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from executive_node import ZenohExecutiveNode

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/middle_ontology.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('middle_ontology')

# Add dedicated handler for llm_api logger
llm_api_logger = logging.getLogger('llm_api')
llm_api_file_handler = logging.FileHandler('logs/llm_api.log', mode='a')
llm_api_file_handler.setLevel(logging.INFO)
llm_api_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', '%Y-%m-%d %H:%M:%S'))
llm_api_logger.addHandler(llm_api_file_handler)
llm_api_logger.setLevel(logging.INFO)

# Add dedicated handler for llm_client logger
llm_client_logger = logging.getLogger('llm_client')
llm_client_file_handler = logging.FileHandler('logs/llm_client.log', mode='a')
llm_client_file_handler.setLevel(logging.INFO)
llm_client_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', '%Y-%m-%d %H:%M:%S'))
llm_client_logger.addHandler(llm_client_file_handler)
llm_client_logger.setLevel(logging.INFO)
from llm_client import ZenohLLMClient

map_types = None


def create_middle_ontology(ontology, llm, map_types, drives):
    map_types_str = ''    
    if map_types:
        map_types_str = f'\n#Available map types:'
        if map_types.get('terrain_types'):
            map_types_str += f"\n\tTerrains: {', '.join(map_types['terrain_types'])}"
        if map_types.get('pathway_types'):
            map_types_str += f"\n\tPathways {', '.join(map_types['infrastructure_types'])}"
        if map_types.get('property_types'):
            map_types_str += f"\n\tProperties: {', '.join(map_types['property_types'])}"
        if map_types.get('resource_types'):
            map_types_str += f"\n\tResources: {', '.join(map_types['resource_types'])}"
        map_types_str += '\n'
    response = llm.ask(
        {"ontology": json.dumps(ontology, indent=2), 
         #"activities": json.dumps(activities, indent=2), 
         "plan_template": PLAN_TEMPLATE, 
         "map_types": map_types_str, 
         "character_drives": drives},
        [SystemMessage(content=MIDDLE_ONTOLOGY_TEMPLATE)],
        max_tokens=10000,
        stops=['</end>'],
        is_json=True,
    )
    print(response)
    return response

def validate_ontology(ont):
    problems = []

    # --- manifest consistency ---
    verb_ids = {v["id"] for v in ont.get("manifest", {}).get("verbs", [])}
    noun_ids = {n["id"] for n in ont.get("manifest", {}).get("nouns", [])}

    # all verb nodes must be in manifest
    for v in ont.get("verbs", {}).get("nodes", []):
        if v["id"] not in verb_ids:
            problems.append(f"Verb {v['id']} not in manifest")

    # all noun nodes must be in manifest
    for n in ont.get("nouns", {}).get("nodes", []):
        if n["id"] not in noun_ids:
            problems.append(f"Noun {n['id']} not in manifest")

    # --- decomposition patterns depth + membership ---
    def check_decomp(node, graph_ids, kind):
        for pat in node.get("decomposition_patterns", []):
            for ref in pat.get("sequence", []):
                if ref not in graph_ids:
                    problems.append(f"{kind} {node['id']} decomposition references missing {ref}")

    for v in ont.get("verbs", {}).get("nodes", []):
        check_decomp(v, verb_ids, "Verb")

    for n in ont.get("nouns", {}).get("nodes", []):
        check_decomp(n, noun_ids, "Noun")

    # --- edges reference existing nodes ---
    def check_edges(edges, graph_ids, kind):
        for e in edges:
            if e["from"] not in graph_ids:
                problems.append(f"{kind} edge from {e['from']} missing")
            if e["to"] not in graph_ids:
                problems.append(f"{kind} edge to {e['to']} missing")

    check_edges(ont.get("verbs", {}).get("edges", []), verb_ids, "Verb")
    check_edges(ont.get("nouns", {}).get("edges", []), noun_ids, "Noun")

    # --- acyclicity (basic DFS) ---
    def is_cyclic(edges, ids):
        graph = {i: [] for i in ids}
        for e in edges:
            graph[e["from"]].append(e["to"])

        visited, stack = set(), set()

        def dfs(node):
            visited.add(node)
            stack.add(node)
            for nbr in graph.get(node, []):
                if nbr not in visited:
                    if dfs(nbr): return True
                elif nbr in stack:
                    return True
            stack.remove(node)
            return False

        return any(dfs(n) for n in ids if n not in visited)

    if is_cyclic(ont.get("verbs", {}).get("edges", []), verb_ids):
        problems.append("Verb graph has a cycle")
    if is_cyclic(ont.get("nouns", {}).get("edges", []), noun_ids):
        problems.append("Noun graph has a cycle")

    # --- node count sanity ---
    if len(verb_ids) > ont["constraints"]["verb_nodes_max"]:
        problems.append("Too many verb nodes")
    if len(noun_ids) > ont["constraints"]["noun_nodes_max"]:
        problems.append("Too many noun nodes")

    return problems

def save_middle_ontology(character_name, ontology, scenario_dir):
    """Save activities to a JSON file in the scenarios directory."""
    filename = f"{character_name}-middle-ontology.json"
    filepath = os.path.join(scenario_dir, filename)
    try:
        with open(filepath, 'w') as file:
            json.dump(ontology, file, indent=2)
        print(f"✅ Saved ontology for {character_name} to {filepath}")
    except Exception as e:
        print(f"❌ Error saving ontology for {character_name}: {e}")

def rewrite_goal(llm, character_name, ontology, middle_ontology, map_types, goal):
    from templates import REWRITE_TEMPLATE
    resp = llm.ask(
        {"activity_name": "EvadeUnknownActor",
         "activity_steps": ["listen for unfamiliar sounds",
                            "stay low and move quietly",
                            "hide behind dense foliage",
                            "observe for movement",
                            "retreat if threatened"
                            ],
         "step_to_rewrite": "listen for unfamiliar sounds",
         "middle_ontology": json.dumps(middle_ontology, indent=2)},
        [SystemMessage(content=REWRITE_TEMPLATE)],
        max_tokens=10000,
        stops=['</end>'],
        is_json=True,
    )
    print(resp)
    return resp

def main(llm):
    global map_types
    
    # Load scenario file
    scenario = load_scenario("../scenarios/jim.yaml")
    if not scenario:
        return
    
    # Get setting from scenario (default to empty string if not present)
    setting = scenario.get('setting', '')
    if not setting:
        print("⚠️  No 'setting' parameter found in scenario file")
        setting = "modern era, western, moderate climate"
    
    # Get characters from scenario
    characters = scenario.get('characters', {})
    if not characters:
        print("❌ No characters found in scenario file")
        return
    
    print(f"📖 Processing scenario: {scenario}")
    print(f"🌍 Setting: {setting}")
    print(f"👥 Characters: {', '.join(characters.keys())}")
    
    for character_name, character_data in characters.items():
        if character_name.lower() != "jim":
            continue
        print(f"\n🎭 Processing character: {character_name}")
        
        # Skip manual characters
        if character_data.get('manual', False):
            print(f"⏭️  Skipping manual character: {character_name}")
            continue
        
        # Get character description
        character_desc = character_data.get('character', '') + '\n' + character_data.get('status', '')
        character_drives = character_data.get('drives', [])
        if not character_desc:
            print(f"⚠️  No character description for {character_name}, skipping")
            continue
        
        # Get ontology
        print(f"🔍 Getting ontology for {character_name}...")
    # Initialize LLM client
    llm = LLM(server_name="openai", model_name="gpt-4.1")
    config = zenoh.Config()
    session = zenoh.open(config)
    
        # Launch map node (required for situation awareness)
    try:
        map_args = [sys.executable, 'map_node.py']
        map_file = scenario.get('map', None)
        if map_file:
            # Decision on reuse/new world is handled early in main()
            world_name = map_file.replace('.py', '')
            map_args.extend(['-m', map_file])
        # Propagate optional debug flag to disable map turn timeouts
        env = os.environ.copy()
        if env.get('CWB_DEBUG', ''):
            logger.info('Debug mode enabled via CWB_DEBUG - map turn timeout will be disabled')
        map_process = subprocess.Popen(map_args, env=env)
        logger.info(f'✅ Map Node launched' + (f' with map: {map_file}' if map_file else ''))
            
        # Wait for map node to be ready (check for initialization message)
        logger.info('⏳ Waiting for Map Node to initialize...')
        time.sleep(8)  # Give map node time to start up
    except Exception as e:
        logger.error(f'❌ Failed to launch Map Node: {e}')

    while not map_types:
        for reply in session.get("cognitive/map/types", timeout=25.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data.get('success'):
                    map_types = data
                    break
    if not map_types:
        logger.error(f"❌ Failed to get map types")
        return
    
    # Get scenario directory for output files
    scenario_dir = os.path.dirname("../scenarios/")
    # Process each character
     
    ontology = json.load(open("../scenarios/Jim-activity-ontology.json"))
    activities = json.load(open("../scenarios/Jim-activities.json"))
    middle_ontology = json.load(open("../scenarios/Jim-middle-ontology.json"))
    goal = "listen for unfamiliar sounds"
    rewritten_goal = rewrite_goal(llm, character_name, ontology, middle_ontology, map_types, goal)

if __name__ == "__main__":
    llm = LLM(server_name="openai", model_name="gpt-4.1")
    main(llm)