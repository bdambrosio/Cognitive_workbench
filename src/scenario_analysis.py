import argparse
import json
import logging
import os
import subprocess
import sys
import time

import zenoh

from activity import (
    load_scenario,
    create_ontology,
    create_activities,
    save_ontology,
    save_activities,
)
from middle_ontology import (
    create_middle_ontology,
    save_middle_ontology,
)
from llm_client import ZenohLLMClient
from utils.llm_api import LLM


logger = logging.getLogger("scenario_analysis")
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')


def _ensure_map_types(session, scenario: dict):
    """Ensure map/types are available.

    1) Try to query existing map/types.
    2) If unavailable, launch map_node (using scenario['map'] if provided), wait ~8s, then retry.

    Returns (map_types: dict, map_process: subprocess.Popen | None)
    """
    # First attempt
    try:
        for reply in session.get("cognitive/map/types", timeout=25.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data.get('success'):
                    return data, None
    except Exception:
        pass

    # Launch map_node if missing
    map_process = None
    try:
        map_args = [sys.executable, 'map_node.py']
        map_file = scenario.get('map', None) if isinstance(scenario, dict) else None
        if map_file:
            map_args.extend(['-m', map_file])
        env = os.environ.copy()
        # Respect CWB_DEBUG passthrough
        if env.get('CWB_DEBUG', ''):
            logger.info('Debug mode enabled via CWB_DEBUG - map turn timeout will be disabled')
        map_process = subprocess.Popen(map_args, env=env)
        logger.info('⏳ Waiting for Map Node to initialize...')
        time.sleep(8)
    except Exception as e:
        logger.error(f"Failed to launch Map Node: {e}")
        map_process = None

    # Retry
    try:
        for reply in session.get("cognitive/map/types", timeout=25.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data.get('success'):
                    return data, map_process
    except Exception:
        pass
    return {}, map_process


def main():
    parser = argparse.ArgumentParser(description="Scenario analysis driver for ontology and activities generation")
    parser.add_argument('--scenario', required=True, help='Path to scenario YAML file')
    parser.add_argument('--character', default='all', help="Character name or 'all'")
    parser.add_argument('--ontology', choices=['load', 'create'], default='load', help='Load or create ontology')
    parser.add_argument('--middle', choices=['load', 'create'], default='load', help='Load or create middle ontology')
    parser.add_argument('--activities', choices=['load', 'create'], default='create', help='Load or create activities')
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

    # Initialize Zenoh session (reused for map/types)
    session = zenoh.open(zenoh.Config())

    # Initialize LLMs
    # - activity.create_ontology/create_activities use global llm_client in activity.py
    # - middle_ontology.create_middle_ontology uses LLM (passed explicitly)
    from activity import llm_client as _activity_llm_global
    if not _activity_llm_global:
        try:
            from activity import llm_client as _mut_llm
            _mut_llm = ZenohLLMClient(service_timeout=240.0)
            # assign into activity module global
            import activity as _act_mod
            _act_mod.llm_client = _mut_llm
        except Exception as e:
            logger.error(f"Failed to init ZenohLLMClient: {e}")
            sys.exit(1)
    middle_llm = LLM(server_name=args.server, model_name=args.model)

    # Try to get map_types (without launching map node). Fail if not present and any stage needs it.
    map_types, map_process = _ensure_map_types(session, scenario)

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

        # Paths
        ontology_path = os.path.join(scenario_dir, f"{name}-activity-ontology.json")
        middle_path = os.path.join(scenario_dir, f"{name}-middle-ontology.json")
        activities_path = os.path.join(scenario_dir, f"{name}-activities.json")

        # Ontology
        if args.ontology == 'load':
            if not os.path.exists(ontology_path):
                raise FileNotFoundError(f"Missing ontology: {ontology_path}")
            with open(ontology_path, 'r') as f:
                ontology = json.load(f)
        else:
            if not map_types:
                logger.error("map/types unavailable; cannot create ontology")
                raise RuntimeError("missing map types")
            ontology = create_ontology(setting, name, character_desc, character_drives, characters)
            if not ontology:
                raise RuntimeError("create_ontology failed")
            save_ontology(name, ontology, scenario_dir)

        # Middle ontology
        if args.middle == 'load':
            if not os.path.exists(middle_path):
                raise FileNotFoundError(f"Missing middle ontology: {middle_path}")
            with open(middle_path, 'r') as f:
                middle = json.load(f)
        else:
            if not map_types:
                logger.error("map/types unavailable; cannot create middle ontology")
                raise RuntimeError("missing map types")
            middle = create_middle_ontology(ontology, middle_llm, map_types, character_drives)
            if not isinstance(middle, dict):
                raise RuntimeError("create_middle_ontology failed")
            save_middle_ontology(name, middle, scenario_dir)

        # Activities
        if args.activities == 'load':
            if not os.path.exists(activities_path):
                raise FileNotFoundError(f"Missing activities: {activities_path}")
            with open(activities_path, 'r') as f:
                activities = json.load(f)
        else:
            if not map_types:
                logger.error("map/types unavailable; cannot create activities")
                raise RuntimeError("missing map types")
            activities = create_activities(setting, name, character_desc, character_drives, characters, ontology, middle)
            if not activities:
                raise RuntimeError("create_activities failed")
            save_activities(name, activities, scenario_dir)

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
    try:
        if map_process is not None:
            map_process.terminate()
    except Exception:
        pass


if __name__ == '__main__':
    main()


