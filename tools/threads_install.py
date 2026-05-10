#!/usr/bin/env python3
"""Threads install tool — install bootstrap-discovered threads into a
character's agent_threads Collection.

Reads the JSON output from tools/threads_bootstrap.py and creates one
thread note per cluster in the live agent_threads Collection of the
target world. Idempotent: skips threads whose name already exists.

Default is DRY-RUN (reports what would be installed without writing).
Pass --execute to actually write.

Usage (dry-run):
    python tools/threads_install.py \\
        --bootstrap-json /tmp/threads_named.json \\
        --scenario scenarios/jill_chat.yaml \\
        --world jill_chat

Execute:
    python tools/threads_install.py \\
        --bootstrap-json /tmp/threads_named.json \\
        --scenario scenarios/jill_chat.yaml \\
        --world jill_chat \\
        --execute

Optional:
    --skip-name N1 N2 ...   names to skip (e.g., --skip-name session_greeting_checkins)
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
SRC_DIR = REPO_ROOT / "src"

sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import ChatLoop  # noqa: E402
from launcher import parse_characters  # noqa: E402

logger = logging.getLogger("tools.threads_install")


def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _build_character_config(scenario_path: Path, world_name_override: str
                            ) -> Tuple[str, dict]:
    """Load scenario YAML; return (character_name, merged_config) for
    the single chat character. Mirrors the bench runners' pattern."""
    scenario = _load_yaml(scenario_path)
    llm_cfg = scenario.get("llm_config") or {}
    alt_llm = scenario.get("alt_llm_config") or {}
    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world_name_override
    setting = scenario.get("setting", "")
    chars = parse_characters(scenario, llm_cfg, world_cfg, setting, alt_llm)
    chat_chars = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat_chars) != 1:
        raise RuntimeError(
            f"expected exactly 1 chat character in {scenario_path}, found "
            f"{len(chat_chars)}"
        )
    return chat_chars[0]


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Install bootstrap-discovered threads into a "
                    "character's agent_threads Collection.")
    parser.add_argument("--bootstrap-json", type=Path, required=True,
                        help="Output JSON from threads_bootstrap.py")
    parser.add_argument("--scenario", type=Path, required=True,
                        help="Scenario YAML (e.g., scenarios/jill_chat.yaml)")
    parser.add_argument("--world", type=str, required=True,
                        help="World name to write into (e.g., 'jill_chat')")
    parser.add_argument("--execute", action="store_true",
                        help="Actually write threads. Without this flag, "
                             "runs in dry-run mode (reports what would happen).")
    parser.add_argument("--skip-name", nargs="+", default=[],
                        help="Thread names to skip (filter from install).")
    args = parser.parse_args()

    bootstrap_path = args.bootstrap_json.resolve()
    if not bootstrap_path.is_file():
        parser.error(f"bootstrap JSON not found: {bootstrap_path}")
    scenario_path = args.scenario.resolve()
    if not scenario_path.is_file():
        parser.error(f"scenario YAML not found: {scenario_path}")

    skip_names: Set[str] = set(args.skip_name)
    payload = json.loads(bootstrap_path.read_text())
    threads = payload.get("threads") or []
    if not threads:
        parser.error(f"no threads in {bootstrap_path}")

    mode = "EXECUTE" if args.execute else "DRY-RUN"
    logger.info(f"mode: {mode}")
    logger.info(f"bootstrap: {bootstrap_path}")
    logger.info(f"scenario: {scenario_path}, world: {args.world}")
    logger.info(f"threads in JSON: {len(threads)}")
    if skip_names:
        logger.info(f"skip-name filter: {sorted(skip_names)}")

    # Filter to threads we'll consider installing.
    candidates: List[Dict[str, Any]] = []
    for t in threads:
        name = (t.get("name") or "").strip()
        if not name:
            logger.warning(f"  skipping cluster {t.get('cluster_id')}: empty name")
            continue
        if name in skip_names:
            logger.info(f"  skipping {name!r}: in skip-name filter")
            continue
        if not t.get("centroid"):
            logger.warning(f"  skipping {name!r}: empty centroid")
            continue
        candidates.append(t)
    logger.info(f"candidates after filter: {len(candidates)}")

    # Instantiate ChatLoop against the target world. This creates the
    # agent_threads Collection if needed (via _init_agent_threads in
    # __init__) and gives us the resource_manager handle. ChatLoop is
    # instantiated WITHOUT calling .run() — Zenoh stays closed.
    char_name, char_config = _build_character_config(scenario_path, args.world)
    logger.info(f"instantiating ChatLoop(character={char_name}, world={args.world})")
    loop = ChatLoop(character_name=char_name, character_config=char_config)
    try:
        # Inspect existing threads.
        existing = loop._get_threads(statuses=None)
        existing_names = {t.get("name") for t in existing}
        logger.info(f"existing threads in collection: {len(existing)} "
                    f"({sorted(existing_names) if existing_names else '(none)'})")

        # Plan + apply.
        installed: List[str] = []
        skipped_existing: List[str] = []
        for t in candidates:
            name = t["name"]
            summary = (t.get("summary") or "").strip()
            centroid = t["centroid"]
            n_turns = int(t.get("turn_count", 0))
            if name in existing_names:
                skipped_existing.append(name)
                logger.info(f"  [{name}] SKIP — already in collection")
                continue
            logger.info(f"  [{name}] {'INSTALL' if args.execute else 'WOULD INSTALL'} "
                        f"(centroid_dim={len(centroid)}, n_turns={n_turns})")
            if args.execute:
                note_id = loop._add_thread(
                    name=name,
                    summary=summary,
                    centroid_embedding=centroid,
                    constituent_turn_count=n_turns,
                    creation_provenance='bootstrap',
                    status='active',
                )
                if note_id:
                    installed.append(name)
                    logger.info(f"    note_id={note_id}")
                else:
                    logger.warning(f"    install failed for {name!r}")

        # Report.
        print()
        print(f"=== Threads install summary ({mode}) ===")
        print(f"candidates considered:   {len(candidates)}")
        print(f"already-existing skip:   {len(skipped_existing)} {sorted(skipped_existing) if skipped_existing else ''}")
        if args.execute:
            print(f"installed:               {len(installed)} {sorted(installed)}")
        else:
            new_names = [t['name'] for t in candidates if t['name'] not in existing_names]
            print(f"would install:           {len(new_names)} {sorted(new_names)}")
            print(f"(dry-run — pass --execute to actually write)")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:
            logger.warning(f"executor shutdown failed: {e}")
        try:
            loop._persist_to_disk()
        except Exception as e:
            logger.warning(f"final persist failed: {e}")


if __name__ == "__main__":
    main()
