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

from utils.format_utils import format_map_places, format_map_tools, format_map_types
#from sentence_transformers import SentenceTransformer
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Messages import SystemMessage, UserMessage
from utils import hash_utils
from utils.state_utils import calculate_state_activity_alignment, get_known_states
from templates import PLAN_TEMPLATE, PLAN_VERBS, MIDDLE_ONTOLOGY_TEMPLATE
from utils.llm_api import LLM
from activity import load_scenario
from activity import derive_drive_lexicon
from templates import PHYSIOLOGICAL_STATES

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

_debug = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')

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


def create_middle_ontology(ontology, llm, map_types, resource_type_str, drives, characters):
    map_types_str = format_map_types(map_types)    
    map_places_str = format_map_places(map_types)
    map_tools_str = format_map_tools(map_types)
    character_drives_str = '\n'.join(drives)   
    drive_lex = derive_drive_lexicon(character_drives_str)

    response = llm.ask(
        {"ontology": json.dumps(ontology, indent=2), 
         #"activities": json.dumps(activities, indent=2), 
         "plan_template": PLAN_TEMPLATE, 
         "character_names": '\n'.join(characters.keys()),
         "primitive_verbs": PLAN_VERBS.strip(),
         "primitive_nouns":  map_types_str.strip()+'\n'+'\n'.join(characters.keys()), 
         "primitive_places": map_places_str.strip(),
         "primitive_tools": resource_type_str.strip(),
         "character_drives": drives,
         "physiological_states": '\n'.join([need['name'] for need in PHYSIOLOGICAL_STATES]),
         "drive_lexicon": json.dumps(drive_lex, indent=2).strip()},
        [SystemMessage(content=MIDDLE_ONTOLOGY_TEMPLATE)],
        max_tokens=20000,
        stops=['</end>'],
        is_json=True,
    )
    #print(response)
    return response

def validate_ontology(ont, map_types):
    problems = []
    map_types_str = format_map_types(map_types)
    report = {
        "missing_decomposition_refs": {
            "verbs": [],
            "nouns": []
        }
    }

    # --- manifest consistency ---
    verb_ids = {v["id"] for v in ont.get("manifest", {}).get("verbs", []) if isinstance(v, dict) and "id" in v}
    noun_ids = {n["id"] for n in ont.get("manifest", {}).get("nouns", []) if isinstance(n, dict) and "id" in n}

    # all verb nodes must be in manifest
    for v in ont.get("verbs", {}).get("nodes", []):
        if not isinstance(v, dict) or "id" not in v:
            problems.append("Verb node missing id")
            continue
        if v["id"] not in verb_ids:
            problems.append(f"Verb {v['id']} not in manifest")

    # all noun nodes must be in manifest
    for n in ont.get("nouns", {}).get("nodes", []):
        if not isinstance(n, dict) or "id" not in n:
            problems.append("Noun node missing id")
            continue
        if n["id"] not in noun_ids:
            problems.append(f"Noun {n['id']} not in manifest")

    # --- decomposition patterns depth + membership ---
    def check_decomp(node, graph_ids, other_ids, kind):
        node_id = node.get("id") if isinstance(node, dict) else None
        for pat in node.get("decomposition_patterns", []) if isinstance(node, dict) else []:
            seq = pat.get("sequence", []) if isinstance(pat, dict) else []
            for ref in seq:
                if ref not in graph_ids:
                    if (kind == "Verb" and ref not in PLAN_VERBS) or (kind == "Noun" and ref not in map_types):
                        # Cross-kind references are invalid; do not treat as missing
                        if ref in other_ids:
                            problems.append(f"{kind} {node_id} decomposition references wrong kind {ref}")
                        else:
                            problems.append(f"{kind} {node_id} decomposition references missing {ref}")
                            bucket = "verbs" if kind == "Verb" else "nouns"
                            report["missing_decomposition_refs"][bucket].append({"owner": node_id, "ref": ref})

    for v in ont.get("verbs", {}).get("nodes", []):
        check_decomp(v, verb_ids, noun_ids, "Verb")

    for n in ont.get("nouns", {}).get("nodes", []):
        check_decomp(n, noun_ids, verb_ids, "Noun")

    # --- edges reference existing nodes ---
    def check_edges(edges, graph_ids, kind):
        for e in edges:
            if not isinstance(e, dict):
                problems.append(f"{kind} edge not a dict")
                continue
            f = e.get("from", e.get("source"))
            t = e.get("to", e.get("target"))
            if f is None or t is None:
                problems.append(f"{kind} edge missing 'from' or 'to': {e}")
                continue
            if f not in graph_ids:
                problems.append(f"{kind} edge from {f} missing")
            if t not in graph_ids:
                problems.append(f"{kind} edge to {t} missing")

    check_edges(ont.get("verbs", {}).get("edges", []), verb_ids, "Verb")
    check_edges(ont.get("nouns", {}).get("edges", []), noun_ids, "Noun")

    # --- acyclicity (basic DFS) ---
    def is_cyclic(edges, ids):
        graph = {i: [] for i in ids}
        for e in edges:
            if not isinstance(e, dict):
                continue
            f = e.get("from", e.get("source"))
            t = e.get("to", e.get("target"))
            if f in graph and t is not None:
                graph[f].append(t)

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

    # --- decomposition-induced acyclicity ---
    def induced_edges_from_decomp(nodes, kind):
        induced = []
        for node in nodes:
            if not isinstance(node, dict):
                continue
            src = node.get('id')
            for pat in node.get('decomposition_patterns', []) if isinstance(node, dict) else []:
                if not isinstance(pat, dict):
                    continue
                # Verbs typically use 'sequence'; nouns often use 'subset_of' in patterns
                seq = pat.get('sequence', [])
                for ref in seq if isinstance(seq, list) else []:
                    induced.append({"from": src, "to": ref})
                subs = pat.get('subset_of', [])
                for ref in subs if isinstance(subs, list) else []:
                    induced.append({"from": src, "to": ref})
        return induced

    verb_nodes = ont.get("verbs", {}).get("nodes", [])
    noun_nodes = ont.get("nouns", {}).get("nodes", [])
    induced_verb_edges = induced_edges_from_decomp(verb_nodes, "Verb")
    induced_noun_edges = induced_edges_from_decomp(noun_nodes, "Noun")

    if is_cyclic(induced_verb_edges, verb_ids):
        problems.append("Verb decomposition graph has a cycle")
    if is_cyclic(induced_noun_edges, noun_ids):
        problems.append("Noun decomposition graph has a cycle")

    # --- leaf requirements ---
    def _leaves(ids, explicit_edges, induced_edges):
        outdeg = {i: 0 for i in ids}
        for e in explicit_edges:
            if isinstance(e, dict):
                f = e.get('from', e.get('source'))
                t = e.get('to', e.get('target'))
                if f in outdeg and t is not None:
                    outdeg[f] += 1
        for f, t in induced_edges:
            if f in outdeg:
                outdeg[f] += 1
        return {i for i, deg in outdeg.items() if deg == 0}

    verb_leaves = _leaves(verb_ids, ont.get("verbs", {}).get("edges", []), [(e.get('from', e.get('source')), e.get('to', e.get('target'))) for e in induced_verb_edges])
    noun_leaves = _leaves(noun_ids, ont.get("nouns", {}).get("edges", []), [(e.get('from', e.get('source')), e.get('to', e.get('target'))) for e in induced_noun_edges])

    # Map ids to nodes for quick lookup
    verb_node_map = {v.get('id'): v for v in ont.get('verbs', {}).get('nodes', []) if isinstance(v, dict)}
    noun_node_map = {n.get('id'): n for n in ont.get('nouns', {}).get('nodes', []) if isinstance(n, dict)}

    for vid in verb_leaves:
        node = verb_node_map.get(vid, {})
        if not node.get('hint_anchor'):
            problems.append(f"Verb leaf missing hint_anchor: {vid}")

    for nid in noun_leaves:
        node = noun_node_map.get(nid, {})
        anchors = node.get('anchor_types')
        if (not isinstance(anchors, list) or len(anchors) == 0) and nid not in map_types_str:
            problems.append(f"Noun leaf missing anchor_types: {nid}")

    # --- node count sanity ---
    constraints = ont.get("constraints", {}) if isinstance(ont, dict) else {}
    verb_max = constraints.get("verb_nodes_max")
    noun_max = constraints.get("noun_nodes_max")
    if isinstance(verb_max, int) and len(verb_ids) > verb_max:
        problems.append("Too many verb nodes")
    if isinstance(noun_max, int) and len(noun_ids) > noun_max:
        problems.append("Too many noun nodes")

    # --- basic stats reporting ---
    verbs_in_ontology = [v["id"] for v in ont.get("verbs", {}).get("nodes", []) if isinstance(v, dict) and "id" in v]
    nouns_in_ontology = [n["id"] for n in ont.get("nouns", {}).get("nodes", []) if isinstance(n, dict) and "id" in n]
    logger.info(f"Ontology stats: verbs={len(verbs_in_ontology)}, nouns={len(nouns_in_ontology)}")
    logger.info(f"Verbs in ontology: {verbs_in_ontology}")
    logger.info(f"Nouns in ontology: {nouns_in_ontology}")

    return problems, report

def repair_ontology(ont, report, mode='nodes'):
    """
    Repair ontology by adding minimal stubs for missing decomposition references.
    - mode='nodes': add to manifest and create minimal nodes; no edges; no recursion
    Returns (repaired_ont, repairs_log)
    """
    repairs = {
        "added_manifest": {"verbs": [], "nouns": []},
        "added_nodes": {"verbs": [], "nouns": []},
        "removed_decomposition_refs": {"verbs": [], "nouns": []},
        "warnings": [],
        "skipped": []
    }

    if not isinstance(ont, dict):
        return ont, repairs

    manifest = ont.setdefault("manifest", {})
    man_verbs = manifest.setdefault("verbs", [])
    man_nouns = manifest.setdefault("nouns", [])

    verb_ids = {v["id"] for v in man_verbs if isinstance(v, dict) and "id" in v}
    noun_ids = {n["id"] for n in man_nouns if isinstance(n, dict) and "id" in n}

    verbs_section = ont.setdefault("verbs", {})
    nouns_section = ont.setdefault("nouns", {})
    verb_nodes = verbs_section.setdefault("nodes", [])
    noun_nodes = nouns_section.setdefault("nodes", [])

    existing_verb_nodes = {v["id"] for v in verb_nodes if isinstance(v, dict) and "id" in v}
    existing_noun_nodes = {n["id"] for n in noun_nodes if isinstance(n, dict) and "id" in n}

    constraints = ont.get("constraints", {}) if isinstance(ont, dict) else {}
    verb_max = constraints.get("verb_nodes_max")
    noun_max = constraints.get("noun_nodes_max")

    # Build current induced graphs (explicit refines + decomposition-induced)
    def _normalized_refines_edges(section):
        edges = []
        for e in section.get("edges", []) if isinstance(section, dict) else []:
            if not isinstance(e, dict):
                continue
            f = e.get('from', e.get('source'))
            t = e.get('to', e.get('target'))
            if f is not None and t is not None:
                edges.append((f, t))
        return edges

    def _induced_edges_from_decomp(nodes):
        edges = []
        for node in nodes:
            if not isinstance(node, dict):
                continue
            src = node.get('id')
            if not src:
                continue
            for pat in node.get('decomposition_patterns', []) if isinstance(node, dict) else []:
                if not isinstance(pat, dict):
                    continue
                seq = pat.get('sequence', [])
                if isinstance(seq, list):
                    for ref in seq:
                        edges.append((src, ref))
                subs = pat.get('subset_of', [])
                if isinstance(subs, list):
                    for ref in subs:
                        edges.append((src, ref))
        return edges

    def _build_graph(ids, section):
        nodes = section.get('nodes', []) if isinstance(section, dict) else []
        edges = set(_normalized_refines_edges(section)) | set(_induced_edges_from_decomp(nodes))
        graph = {i: [] for i in ids}
        for f, t in edges:
            if f in graph:
                graph[f].append(t)
        return graph

    def _has_path(graph, start, goal):
        if start == goal:
            return True
        seen = set()
        stack = [start]
        while stack:
            cur = stack.pop()
            if cur in seen:
                continue
            seen.add(cur)
            for nxt in graph.get(cur, []):
                if nxt == goal:
                    return True
                stack.append(nxt)
        return False

    def _remove_ref_from_owner(section, owner_id, ref_id, is_noun=False):
        nodes = section.get('nodes', []) if isinstance(section, dict) else []
        for node in nodes:
            if not isinstance(node, dict):
                continue
            if node.get('id') != owner_id:
                continue
            patterns = node.get('decomposition_patterns', [])
            changed = False
            for pat in patterns:
                if not isinstance(pat, dict):
                    continue
                if 'sequence' in pat and isinstance(pat['sequence'], list):
                    before = len(pat['sequence'])
                    pat['sequence'] = [x for x in pat['sequence'] if x != ref_id]
                    changed = changed or (len(pat['sequence']) != before)
                if 'subset_of' in pat and isinstance(pat['subset_of'], list):
                    before = len(pat['subset_of'])
                    pat['subset_of'] = [x for x in pat['subset_of'] if x != ref_id]
                    changed = changed or (len(pat['subset_of']) != before)
            return changed
        return False

    # Process verbs
    # Prepare graphs for cycle checks (used only to decide safe removals)
    verb_graph = _build_graph(verb_ids, verbs_section)
    noun_graph = _build_graph(noun_ids, nouns_section)

    # Minimal implementation: for missing verb refs in decompositions, ADD minimal verb nodes + manifest entries.
    # Do NOT add edges or decompositions, and do NOT remove the reference from the owner.
    # Idempotent: skip if node/manifest already present.
    seen_added_verbs = set()
    for item in report.get("missing_decomposition_refs", {}).get("verbs", []):
        ref = item.get("ref")
        owner = item.get("owner")
        if not ref or not owner or not isinstance(ref, str):
            continue
        # Skip if already present as node
        if ref in existing_verb_nodes:
            continue
        # Optional cap check (treat as warning only)
        if isinstance(verb_max, int) and (len(existing_verb_nodes) + len(seen_added_verbs) + 1) > verb_max:
            repairs["warnings"].append(f"Verb cap would be exceeded by adding {ref}; adding anyway")
        # Add minimal node
        verb_nodes.append({
            "id": ref,
            "decomposition_patterns": []
        })
        seen_added_verbs.add(ref)
        repairs["added_nodes"]["verbs"].append(ref)
        # Ensure manifest entry exists
        if ref not in verb_ids:
            man_verbs.append({"id": ref, "gloss": ""})
            repairs["added_manifest"]["verbs"].append(ref)
        # Update in-memory sets for subsequent iterations
        existing_verb_nodes.add(ref)
        verb_ids.add(ref)

    # Process nouns
    for item in report.get("missing_decomposition_refs", {}).get("nouns", []):
        ref = item.get("ref")
        owner = item.get("owner")
        if not ref or not owner:
            continue
        # Minimal repair: remove invalid/missing reference from owner's decomposition
        if _remove_ref_from_owner(nouns_section, owner, ref, is_noun=True):
            repairs["removed_decomposition_refs"]["nouns"].append({"owner": owner, "ref": ref})

    # --- Break cycles in VERB decomposition sequences (minimal, safe) ---
    def _count_verb_seq_refs(section):
        total = 0
        nodes = section.get('nodes', []) if isinstance(section, dict) else []
        for node in nodes:
            if not isinstance(node, dict):
                continue
            for pat in node.get('decomposition_patterns', []) if isinstance(node, dict) else []:
                seq = pat.get('sequence', []) if isinstance(pat, dict) else []
                if isinstance(seq, list):
                    total += len(seq)
        return total

    def _build_adj(section):
        adj = {}
        ids = {v.get('id') for v in section.get('nodes', []) if isinstance(v, dict) and isinstance(v.get('id'), str)}
        for vid in ids:
            adj[vid] = []
        for node in section.get('nodes', []):
            if not isinstance(node, dict):
                continue
            src = node.get('id')
            if src not in adj:
                continue
            for pat in node.get('decomposition_patterns', []) if isinstance(node, dict) else []:
                if not isinstance(pat, dict):
                    continue
                seq = pat.get('sequence', [])
                if isinstance(seq, list):
                    for dst in seq:
                        if isinstance(dst, str) and dst in adj:
                            adj[src].append(dst)
        return adj

    def _find_cycle_edge(section):
        adj = _build_adj(section)
        state = {k: 0 for k in adj.keys()}  # 0=unseen,1=visiting,2=done
        stack = []

        def dfs(u):
            state[u] = 1
            stack.append(u)
            for v in adj[u]:
                if state.get(v, 0) == 0:
                    edge = dfs(v)
                    if edge:
                        return edge
                elif state.get(v) == 1:
                    # back edge u -> v
                    return (u, v)
            stack.pop()
            state[u] = 2
            return None

        for n in adj.keys():
            if state[n] == 0:
                edge = dfs(n)
                if edge:
                    return edge
        return None

    def _prune_empty_patterns(section, owner_id):
        nodes = section.get('nodes', []) if isinstance(section, dict) else []
        for node in nodes:
            if not isinstance(node, dict):
                continue
            if node.get('id') != owner_id:
                continue
            pats = node.get('decomposition_patterns')
            if isinstance(pats, list):
                new_pats = []
                for pat in pats:
                    if not isinstance(pat, dict):
                        continue
                    seq = pat.get('sequence', []) if isinstance(pat.get('sequence'), list) else []
                    subs = pat.get('subset_of', []) if isinstance(pat.get('subset_of'), list) else []
                    # Keep only if at least one of the lists remains non-empty
                    if seq or subs:
                        new_pats.append(pat)
                node['decomposition_patterns'] = new_pats
            return

    # Cap iterations to initial number of verb sequence refs
    max_removals = _count_verb_seq_refs(verbs_section)
    removals = 0
    while removals < max_removals:
        cyc = _find_cycle_edge(verbs_section)
        if not cyc:
            break
        u, v = cyc
        if _remove_ref_from_owner(verbs_section, u, v, is_noun=False):
            _prune_empty_patterns(verbs_section, u)
            repairs["removed_decomposition_refs"]["verbs"].append({"owner": u, "ref": v, "reason": "cycle_break"})
            removals += 1
        else:
            # Nothing removed; avoid infinite loop
            repairs["warnings"].append(f"Cycle detected on {u}->{v} but removal failed; aborting cycle repair loop")
            break

    # If still cyclic after bounded removals, log a warning
    if _find_cycle_edge(verbs_section):
        repairs["warnings"].append("Verb decomposition graph still cyclic after bounded removals")

    return ont, repairs

def build_noun_indices(middle_ontology, map_types, prune_ungrounded=True, logger=None):
    """
    Build noun indices from the noun DAG.

    - Edges: all nouns.edges are 'refines' (parent->child).
    - Also follow noun decomposition_patterns.sequence (node->child noun).
    - Anchors are derived downward: local anchor_types ∪ union(children).
    - Grounded := derived_anchors ∩ resource_types ≠ ∅
    - If prune_ungrounded:
        * Do NOT raise on leafs missing anchors.
        * Indices only include nodes that are grounded; ungrounded nodes are ignored.
      Else:
        * Enforce: every leaf must have explicit, non-empty anchor_types (raise if not).

    Returns:
      {"noun_to_map_types": {noun_id: [map_type,...]},
       "map_type_to_nouns": {map_type: [noun_id,...]}}
    """
    indices = {"noun_to_map_types": {}, "map_type_to_nouns": {}}

    nouns = (middle_ontology or {}).get("nouns", {})
    nodes = nouns.get("nodes", []) or []
    edges = nouns.get("edges", []) or []
    resource_types = set((map_types or {}).get("resource_types", []) or [])
    if not nodes or not resource_types:
        return indices

    # Collect noun ids & local anchors
    noun_ids = set()
    local_anchors = {}
    for n in nodes:
        if not isinstance(n, dict):
            continue
        nid = n.get("id")
        if not isinstance(nid, str):
            continue
        noun_ids.add(nid)
        a = n.get("anchor_types")
        if isinstance(a, list):
            local_anchors[nid] = [x for x in a if isinstance(x, str)]

    # Children adjacency from refines edges (parent->child)
    children = {nid: set() for nid in noun_ids}
    for e in edges:
        if not isinstance(e, dict):
            continue
        parent = e.get("from") or e.get("source")
        child  = e.get("to")   or e.get("target")
        if parent in noun_ids and child in noun_ids:
            children[parent].add(child)

    # Add noun decomposition links: node -> each noun in sequence
    for n in nodes:
        if not isinstance(n, dict):
            continue
        src = n.get("id")
        if src not in noun_ids:
            continue
        for pat in (n.get("decomposition_patterns") or []):
            if not isinstance(pat, dict):
                continue
            seq = pat.get("sequence", [])
            if isinstance(seq, list):
                for dst in seq:
                    if isinstance(dst, str) and dst in noun_ids:
                        children[src].add(dst)

    # Cycle detection
    VISITING, VISITED = 1, 2
    state = {nid: 0 for nid in noun_ids}
    def _dfs_cycle(u):
        state[u] = VISITING
        for v in children[u]:
            if state[v] == VISITING:
                raise ValueError(f"Cycle detected in noun graph: {u} -> {v}")
            if state[v] == 0:
                _dfs_cycle(v)
        state[u] = VISITED
    for nid in noun_ids:
        if state[nid] == 0:
            _dfs_cycle(nid)

    # Leaves (no outgoing links in combined graph)
    leaves = {nid for nid in noun_ids if not children[nid]}

    if not prune_ungrounded:
        # Strict enforcement: leaf must have explicit anchors
        bad = [nid for nid in leaves if not local_anchors.get(nid)]
        if bad:
            raise ValueError("Leaf nouns missing explicit anchor_types: " + ", ".join(sorted(bad)))

    # Derived anchors (downward union)
    from functools import lru_cache
    @lru_cache(maxsize=None)
    def derived_anchors(nid: str):
        acc = set(local_anchors.get(nid, []))
        for ch in children[nid]:
            acc |= derived_anchors(ch)
        return frozenset(acc)

    # Grounded set = nodes that derive at least one resource_type
    grounded = {nid for nid in noun_ids if any(a in resource_types for a in derived_anchors(nid))}

    if prune_ungrounded:
        # Optionally log what we're excluding
        if logger:
            missing = sorted(leaves & (noun_ids - set(local_anchors.keys())))
            if missing:
                logger.warning("Pruning ungrounded leaf nouns (no explicit anchors): " + ", ".join(missing))
        # Build indices only over grounded nodes
        working = grounded
    else:
        working = noun_ids  # strict mode will have already raised if leaves ungrounded

    # Build indices
    map_type_to_nouns = {}
    noun_to_map_types = {}
    for nid in working:
        mts = sorted(a for a in derived_anchors(nid) if a in resource_types)
        if mts:
            noun_to_map_types[nid] = mts
            for mt in mts:
                map_type_to_nouns.setdefault(mt, set()).add(nid)

    # finalize
    indices["noun_to_map_types"] = noun_to_map_types
    indices["map_type_to_nouns"] = {mt: sorted(ns) for mt, ns in map_type_to_nouns.items()}
    return indices



def build_allowed_scan_types(middle_ontology, map_types):
    """
    Build a flat, sorted list of allowed scan target type names.
    Includes:
      - Noun ids that ground to at least one resource type (via indices or anchors)
      - Map resource types
      - Terrain and infrastructure types (if available)
      - Physiological states
      - Generic 'Person'
    """
    try:
        # Prefer precomputed indices
        indices = (
            middle_ontology.get('indices', {}).get('noun', {})
            if isinstance(middle_ontology, dict) else {}
        )
        noun_to_map = indices.get('noun_to_map_types') if isinstance(indices, dict) else None
        allowed_nouns = []
        if isinstance(noun_to_map, dict):
            allowed_nouns = [nid for nid, mts in noun_to_map.items() if isinstance(mts, list) and len(mts) > 0]
        else:
            # Fallback: derive via anchors when indices are missing
            nouns_section = middle_ontology.get('nouns', {}) if isinstance(middle_ontology, dict) else {}
            nodes = nouns_section.get('nodes', []) if isinstance(nouns_section, dict) else []
            for n in nodes:
                if not isinstance(n, dict):
                    continue
                nid = n.get('id')
                anchors = n.get('anchor_types')
                if nid and isinstance(anchors, list) and len(anchors) > 0:
                    allowed_nouns.append(nid)

        res_types = map_types.get('resource_types', []) if isinstance(map_types, dict) else []
        terr_types = map_types.get('terrain_types', []) if isinstance(map_types, dict) else []
        infra_types = map_types.get('infrastructure_types', []) if isinstance(map_types, dict) else []
        phys_states_types = [need['name'] for need in PHYSIOLOGICAL_STATES]

        names = set()
        for s in (allowed_nouns or []):
            if isinstance(s, str) and s:
                names.add(s)
        for s in (res_types or []):
            if isinstance(s, str) and s:
                names.add(s)
        for s in (terr_types or []):
            if isinstance(s, str) and s:
                names.add(s)
        for s in (infra_types or []):
            if isinstance(s, str) and s:
                names.add(s)

        names.add('Person')

        return sorted(names)
    except Exception as e:
        logger.warning(f"Failed to build allowed scan types: {e}")
        return []


def build_verb_indices(middle_ontology, logger: logging.Logger = None):
    """
    Build verb indices using refines edges and decomposition sequences.

    - Local anchors: verb.hint_anchor (must be a string). We do not invent anchors.
    - Derived anchors: local ∪ union(children) along combined graph (refines + decomposition-induced).

    Returns:
      {"verb_to_anchors": {verb_id: [anchor,...]},
       "anchor_to_verbs": {anchor: [verb_id,...]}}
    """
    indices = {"verb_to_anchors": {}, "anchor_to_verbs": {}}

    try:
        verbs = (middle_ontology or {}).get("verbs", {})
        nodes = verbs.get("nodes", []) or []
        edges = verbs.get("edges", []) or []
        if not nodes:
            return indices

        # Collect verb ids and local anchors
        # Remove any verbs that differ only by case from primitive verbs (keep primitives)
        try:
            primitives_lc = {s.strip().lower() for s in (PLAN_VERBS or '').splitlines() if s.strip()}
        except Exception:
            primitives_lc = set()
        verb_ids = set()
        local_anchor = {}
        for v in nodes:
            if not isinstance(v, dict):
                continue
            vid = v.get("id")
            if not isinstance(vid, str):
                continue
            # Exclude case-variants of primitives (e.g., 'Say' vs primitive 'say')
            if vid.lower() in primitives_lc and vid != vid.lower():
                continue
            verb_ids.add(vid)
            ha = v.get("hint_anchor")
            if isinstance(ha, str) and ha:
                local_anchor[vid] = ha

        # Children adjacency from refines edges (parent->child)
        children = {vid: set() for vid in verb_ids}
        for e in edges:
            if not isinstance(e, dict):
                continue
            parent = e.get("from") or e.get("source")
            child = e.get("to") or e.get("target")
            if parent in verb_ids and child in verb_ids:
                children[parent].add(child)

        # Add decomposition links: node -> each verb in sequence
        for v in nodes:
            if not isinstance(v, dict):
                continue
            src = v.get("id")
            if src not in verb_ids:
                continue
            for pat in (v.get("decomposition_patterns") or []):
                if not isinstance(pat, dict):
                    continue
                seq = pat.get("sequence", [])
                if isinstance(seq, list):
                    for dst in seq:
                        if isinstance(dst, str) and dst in verb_ids:
                            children[src].add(dst)

        # Cycle handling: break back-edges instead of aborting
        VISITING, VISITED = 1, 2
        state = {vid: 0 for vid in verb_ids}
        def _dfs_break_cycles(u):
            state[u] = VISITING
            # Iterate over a copy so we can mutate 'children[u]' safely
            for v in list(children[u]):
                if state[v] == VISITING:
                    # Break the back-edge to avoid a cycle and continue
                    try:
                        children[u].remove(v)
                    except KeyError:
                        pass
                    if logger:
                        logger.warning(f"Cycle detected in verb graph: {u} -> {v} (edge removed)")
                    continue
                if state[v] == 0:
                    _dfs_break_cycles(v)
            state[u] = VISITED
        for vid in verb_ids:
            if state[vid] == 0:
                _dfs_break_cycles(vid)

        # Derived anchors via downward union
        from functools import lru_cache
        @lru_cache(maxsize=None)
        def derived(vid: str):
            acc = set()
            if vid in local_anchor:
                acc.add(local_anchor[vid])
            for ch in children[vid]:
                acc |= derived(ch)
            return frozenset(acc)

        # Build indices
        anchor_to_verbs = {}
        verb_to_anchors = {}
        for vid in verb_ids:
            anchors = sorted(a for a in derived(vid) if isinstance(a, str) and a)
            if anchors:
                verb_to_anchors[vid] = anchors
                for a in anchors:
                    anchor_to_verbs.setdefault(a, set()).add(vid)

        indices["verb_to_anchors"] = verb_to_anchors
        indices["anchor_to_verbs"] = {a: sorted(vs) for a, vs in anchor_to_verbs.items()}
        return indices
    except Exception as e:
        if logger:
            logger.warning(f"Failed to build verb indices: {e}")
        return indices


def build_allowed_verbs(middle_ontology, logger: logging.Logger = None):
    """
    Build a flat, sorted list of allowed verbs consisting of:
      - Primitive operators from templates.PLAN_VERBS, and
      - All middle-ontology verb ids whose derived anchors include any primitive operator

    Includes control-flow and condition operators as provided in PLAN_VERBS.
    """
    try:
        # Parse primitives from PLAN_VERBS (newline-separated)
        from templates import PLAN_VERBS as _PLAN_VERBS
        primitives = {s.strip().lower() for s in (_PLAN_VERBS or '').splitlines() if s.strip()}

        # Build or fetch verb indices
        if middle_ontology.get('indices', {}).get('verb'):
            v_idx = middle_ontology.get('indices', {}).get('verb', {})
        else:
            v_idx = build_verb_indices(middle_ontology, logger)
            
        verb_to_anchors = v_idx.get('verb_to_anchors', {}) if isinstance(v_idx, dict) else {}

        allowed = set(primitives)

        # Collect verbs anchored (directly or via descendants) to any primitive
        for vid, anchors in verb_to_anchors.items():
            try:
                if any((str(a).lower() in primitives) for a in (anchors or [])):
                    allowed.add(vid)
            except Exception:
                continue

        # Log invalid/missing hint_anchors for visibility
        try:
            verbs = (middle_ontology or {}).get('verbs', {})
            nodes = verbs.get('nodes', []) or []
            for v in nodes:
                if not isinstance(v, dict):
                    continue
                vid = v.get('id')
                ha = v.get('hint_anchor')
                if not ha:
                    if logger:
                        logger.warning(f"Verb '{vid}' missing hint_anchor")
                    continue
                if str(ha).lower() not in primitives:
                    if logger:
                        logger.warning(f"Verb '{vid}' has non-primitive hint_anchor '{ha}' (not in PLAN_VERBS)")
        except Exception:
            pass

        # Remove any verbs that differ only by case from primitive verbs (keep primitives)
        try:
            allowed = {vid for vid in allowed if not (vid.lower() in primitives and vid != vid.lower())}
        except Exception:
            pass

        # Sort deterministically
        return sorted(allowed)
    except Exception as e:
        if logger:
            logger.warning(f"Failed to build allowed verbs: {e}")
        return []

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

def rewrite_goal(llm:LLM, character_name, current_activity, ontology, middle_ontology, allowed_types, allowed_verbs, goal):
    """Rewrite a goal using the middle ontology and map types."""
    from templates import REWRITE_TEMPLATE
    resp = llm.ask(
        {"activity_name": current_activity['name'],
         "activity_steps": current_activity['steps'],
         "step_to_rewrite": goal,
         "middle_ontology": json.dumps(middle_ontology, indent=2),
         "allowed_nouns": allowed_types,
         "allowed_verbs": allowed_verbs},
        [SystemMessage(content=REWRITE_TEMPLATE)],
        max_tokens=100,
        stops=['</end>'],
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
        for reply in session.get("cognitive/map/types", timeout=25.0 if not _debug else 300.0):
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