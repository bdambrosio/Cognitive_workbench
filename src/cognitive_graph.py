"""
Cognitive Memory Graph — persistent semantic graph for OODA cognitive objects.

Replaces ad-hoc living state with a typed node/edge graph backed by FAISS
for similarity retrieval. See docs/cognitive_graph_spec.md for full spec.
"""

import json
import logging
import threading
import time
import uuid
from pathlib import Path

import faiss
import numpy as np

logger = logging.getLogger(__name__)


class CognitiveGraph:
    """
    Graph store for cognitive objects (events, assessments, decisions, etc.)
    with FAISS-backed semantic search over node content.
    """

    # Default configuration (overridable via constructor)
    DEFAULTS = {
        "embedding_dim": 384,
        "consolidation_threshold_hours": 4,
        "consolidation_window_hours": 1,
        "consolidation_min_nodes": 5,
        "concern_change_threshold": 0.05,
        "max_content_length": 500,
        "persist_interval_seconds": 300,
    }

    def __init__(self, embedder=None, config: dict = None):
        """
        Args:
            embedder: callable(text) -> list[float] — embedding function.
                      If None, semantic_search will raise until one is set.
            config:   dict overriding keys in DEFAULTS.
        """
        cfg = {**self.DEFAULTS, **(config or {})}
        self._config = cfg
        self._dim = cfg["embedding_dim"]
        self._max_content = cfg["max_content_length"]

        # Graph storage
        self._nodes: dict[str, dict] = {}            # node_id -> node dict
        self._edges: dict[str, dict] = {}            # edge_id -> edge dict
        self._edges_from: dict[str, list[str]] = {}  # node_id -> [edge_id, ...]
        self._edges_to: dict[str, list[str]] = {}    # node_id -> [edge_id, ...]

        # Monotonic counters for readable IDs
        self._node_counter = 0
        self._edge_counter = 0

        # FAISS semantic index: IndexIDMap wrapping IndexFlatIP
        self._faiss_index = faiss.IndexIDMap(faiss.IndexFlatIP(self._dim))
        self._faiss_id_counter = 0
        self._faiss_id_to_node: dict[int, str] = {}
        self._node_to_faiss_id: dict[str, int] = {}

        # Embedding function
        self._embedder = embedder

        # Session tracking
        self._session_id = str(uuid.uuid4())

        # Thread safety
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API (§5)
    # ------------------------------------------------------------------

    def add_node(self, type: str, content: str, attrs: dict = None,
                 ts: float = None) -> str:
        """Add a node. Returns node_id. Automatically indexes content in FAISS."""
        content = content[:self._max_content] if content else ""
        ts = ts or time.time()
        with self._lock:
            node_id = self._next_node_id()
            node = {
                "node_id": node_id,
                "type": type,
                "content": content,
                "attrs": dict(attrs) if attrs else {},
                "ts": ts,
                "session_id": self._session_id,
            }
            self._nodes[node_id] = node
            self._edges_from.setdefault(node_id, [])
            self._edges_to.setdefault(node_id, [])
            # Index in FAISS
            self._index_node(node_id, content)
        return node_id

    def add_edge(self, source: str, target: str, type: str,
                 ts: float = None) -> str:
        """Add a directed edge. Returns edge_id."""
        ts = ts or time.time()
        with self._lock:
            if source not in self._nodes:
                raise KeyError(f"Source node {source} not found")
            if target not in self._nodes:
                raise KeyError(f"Target node {target} not found")
            edge_id = self._next_edge_id()
            edge = {
                "edge_id": edge_id,
                "source": source,
                "target": target,
                "type": type,
                "ts": ts,
            }
            self._edges[edge_id] = edge
            self._edges_from[source].append(edge_id)
            self._edges_to[target].append(edge_id)
        return edge_id

    def get_node(self, node_id: str) -> dict | None:
        """Return node dict, or None if not found."""
        with self._lock:
            node = self._nodes.get(node_id)
            return dict(node) if node else None

    def update_attrs(self, node_id: str, attrs: dict) -> None:
        """Merge attrs into existing node attrs. Does not re-embed content."""
        with self._lock:
            node = self._nodes.get(node_id)
            if node is None:
                raise KeyError(f"Node {node_id} not found")
            node["attrs"].update(attrs)

    def get_edges_from(self, node_id: str, edge_type: str = None) -> list[dict]:
        """Return outgoing edges from node, optionally filtered by type."""
        with self._lock:
            edge_ids = self._edges_from.get(node_id, [])
            edges = [self._edges[eid] for eid in edge_ids if eid in self._edges]
            if edge_type:
                edges = [e for e in edges if e["type"] == edge_type]
            return [dict(e) for e in edges]

    def get_edges_to(self, node_id: str, edge_type: str = None) -> list[dict]:
        """Return incoming edges to node, optionally filtered by type."""
        with self._lock:
            edge_ids = self._edges_to.get(node_id, [])
            edges = [self._edges[eid] for eid in edge_ids if eid in self._edges]
            if edge_type:
                edges = [e for e in edges if e["type"] == edge_type]
            return [dict(e) for e in edges]

    def query_nodes(self, type: str = None, attrs_filter: dict = None,
                    since: float = None, until: float = None,
                    limit: int = 100) -> list[dict]:
        """Return nodes matching criteria, most recent first.
        attrs_filter does exact match on attrs keys."""
        with self._lock:
            candidates = list(self._nodes.values())
        # Filter outside lock on snapshot
        results = []
        for node in candidates:
            if type and node["type"] != type:
                continue
            if since and node["ts"] < since:
                continue
            if until and node["ts"] > until:
                continue
            if attrs_filter:
                if not all(node["attrs"].get(k) == v
                           for k, v in attrs_filter.items()):
                    continue
            results.append(dict(node))
        results.sort(key=lambda n: n["ts"], reverse=True)
        return results[:limit]

    def semantic_search(self, query_text: str, k: int = 10,
                        type_filter: str | list[str] = None
                        ) -> list[tuple[str, float]]:
        """Return (node_id, score) pairs for k nearest nodes by FAISS similarity.
        type_filter restricts results to given node type(s)."""
        if self._embedder is None:
            raise RuntimeError("No embedding function configured")
        embedding = self._embed(query_text)
        vec = np.array([embedding], dtype="float32")
        faiss.normalize_L2(vec)

        with self._lock:
            ntotal = self._faiss_index.ntotal
            if ntotal == 0:
                return []
            search_k = min(k * 3, ntotal)  # over-fetch for type filtering
            scores, ids = self._faiss_index.search(vec, search_k)

        # Normalize type_filter to a set
        if isinstance(type_filter, str):
            type_filter = {type_filter}
        elif isinstance(type_filter, list):
            type_filter = set(type_filter)

        results = []
        for score, fid in zip(scores[0], ids[0]):
            if fid < 0:
                continue
            with self._lock:
                node_id = self._faiss_id_to_node.get(int(fid))
                if node_id is None:
                    continue
                node = self._nodes.get(node_id)
                if node is None:
                    continue
                if type_filter and node["type"] not in type_filter:
                    continue
            results.append((node_id, float(score)))
            if len(results) >= k:
                break
        return results

    def expand_subgraph(self, seed_node_ids: list[str], max_hops: int = 2,
                        edge_types: list[str] = None
                        ) -> tuple[list[dict], list[dict]]:
        """BFS expansion from seed nodes along edges (optionally filtered by type).
        Returns (nodes, edges) comprising the subgraph."""
        visited_nodes: set[str] = set()
        visited_edges: set[str] = set()
        frontier = set(seed_node_ids)

        for _ in range(max_hops + 1):
            if not frontier:
                break
            next_frontier: set[str] = set()
            for nid in frontier:
                if nid in visited_nodes:
                    continue
                visited_nodes.add(nid)
                with self._lock:
                    # Outgoing
                    for eid in self._edges_from.get(nid, []):
                        edge = self._edges.get(eid)
                        if edge is None:
                            continue
                        if edge_types and edge["type"] not in edge_types:
                            continue
                        visited_edges.add(eid)
                        next_frontier.add(edge["target"])
                    # Incoming
                    for eid in self._edges_to.get(nid, []):
                        edge = self._edges.get(eid)
                        if edge is None:
                            continue
                        if edge_types and edge["type"] not in edge_types:
                            continue
                        visited_edges.add(eid)
                        next_frontier.add(edge["source"])
            frontier = next_frontier - visited_nodes

        with self._lock:
            nodes = [dict(self._nodes[nid]) for nid in visited_nodes
                     if nid in self._nodes]
            edges = [dict(self._edges[eid]) for eid in visited_edges
                     if eid in self._edges]
        return nodes, edges

    def latest_per_key(self, type: str, key_attr: str) -> list[dict]:
        """Return the most recent node of given type for each distinct value of key_attr."""
        latest: dict[str, dict] = {}
        with self._lock:
            for node in self._nodes.values():
                if node["type"] != type:
                    continue
                key_val = node["attrs"].get(key_attr)
                if key_val is None:
                    continue
                existing = latest.get(key_val)
                if existing is None or node["ts"] > existing["ts"]:
                    latest[key_val] = node
        return [dict(n) for n in latest.values()]

    def prune_before(self, ts: float, exclude_types: list[str] = None) -> int:
        """Delete nodes (and their edges) older than ts. Nodes of exclude_types
        are kept. Returns count of deleted nodes."""
        exclude = set(exclude_types) if exclude_types else set()
        with self._lock:
            to_remove = [
                nid for nid, node in self._nodes.items()
                if node["ts"] < ts and node["type"] not in exclude
            ]
            for nid in to_remove:
                self._remove_node_locked(nid)
            return len(to_remove)

    def node_count(self) -> int:
        """Total number of nodes in the graph."""
        with self._lock:
            return len(self._nodes)

    # ------------------------------------------------------------------
    # Persistence (§11)
    # ------------------------------------------------------------------

    def save(self, path: str) -> None:
        """Persist graph + FAISS index to disk."""
        path = Path(path)
        with self._lock:
            # Graph structure as JSON
            data = {
                "nodes": self._nodes,
                "edges": self._edges,
                "edges_from": self._edges_from,
                "edges_to": self._edges_to,
                "node_counter": self._node_counter,
                "edge_counter": self._edge_counter,
                "faiss_id_counter": self._faiss_id_counter,
                "faiss_id_to_node": {str(k): v for k, v in self._faiss_id_to_node.items()},
                "node_to_faiss_id": {k: v for k, v in self._node_to_faiss_id.items()},
                "session_id": self._session_id,
            }
            json_path = path.with_suffix(".json")
            faiss_path = str(path.with_suffix(".faiss"))

            # Write JSON
            with open(json_path, "w") as f:
                json.dump(data, f)

            # Write FAISS index
            faiss.write_index(self._faiss_index, faiss_path)

        logger.info("CognitiveGraph saved: %d nodes, %d edges → %s",
                     len(self._nodes), len(self._edges), path)

    def load(self, path: str) -> None:
        """Load graph + FAISS index from disk. Starts a new session."""
        path = Path(path)
        json_path = path.with_suffix(".json")
        faiss_path = str(path.with_suffix(".faiss"))

        if not json_path.exists():
            logger.info("No graph file at %s, starting empty", json_path)
            return

        with open(json_path, "r") as f:
            data = json.load(f)

        with self._lock:
            self._nodes = data["nodes"]
            self._edges = data["edges"]
            self._edges_from = data["edges_from"]
            self._edges_to = data["edges_to"]
            self._node_counter = data["node_counter"]
            self._edge_counter = data["edge_counter"]
            self._faiss_id_counter = data["faiss_id_counter"]
            self._faiss_id_to_node = {int(k): v for k, v in data["faiss_id_to_node"].items()}
            self._node_to_faiss_id = data["node_to_faiss_id"]

            if Path(faiss_path).exists():
                self._faiss_index = faiss.read_index(faiss_path)
            else:
                logger.warning("FAISS index not found at %s, rebuilding", faiss_path)
                self._rebuild_faiss_locked()

            # New session
            self._session_id = str(uuid.uuid4())

        logger.info("CognitiveGraph loaded: %d nodes, %d edges from %s",
                     len(self._nodes), len(self._edges), path)

    # ------------------------------------------------------------------
    # Embedding
    # ------------------------------------------------------------------

    def set_embedder(self, embedder) -> None:
        """Set or replace the embedding function: callable(str) -> list[float]."""
        self._embedder = embedder

    def _embed(self, text: str) -> list[float]:
        """Generate embedding for text via the configured embedder."""
        return self._embedder(text)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _next_node_id(self) -> str:
        self._node_counter += 1
        return f"n_{self._node_counter:04d}"

    def _next_edge_id(self) -> str:
        self._edge_counter += 1
        return f"e_{self._edge_counter:04d}"

    def _next_faiss_id(self) -> int:
        self._faiss_id_counter += 1
        return self._faiss_id_counter

    def _index_node(self, node_id: str, content: str) -> None:
        """Add a node's content to the FAISS index. Caller must hold _lock."""
        if not self._embedder or not content.strip():
            return
        try:
            embedding = self._embed(content)
            vec = np.array([embedding], dtype="float32")
            faiss.normalize_L2(vec)
            fid = self._next_faiss_id()
            ids = np.array([fid], dtype="int64")
            self._faiss_index.add_with_ids(vec, ids)
            self._faiss_id_to_node[fid] = node_id
            self._node_to_faiss_id[node_id] = fid
        except Exception:
            logger.exception("Failed to index node %s in FAISS", node_id)

    def _remove_node_locked(self, node_id: str) -> None:
        """Remove a node and all its edges. Caller must hold _lock."""
        # Remove from FAISS
        fid = self._node_to_faiss_id.pop(node_id, None)
        if fid is not None:
            try:
                self._faiss_index.remove_ids(np.array([fid], dtype="int64"))
            except Exception:
                logger.exception("Failed to remove FAISS id %d for node %s", fid, node_id)
            self._faiss_id_to_node.pop(fid, None)

        # Remove outgoing edges
        for eid in list(self._edges_from.get(node_id, [])):
            edge = self._edges.pop(eid, None)
            if edge:
                target = edge["target"]
                if target in self._edges_to:
                    try:
                        self._edges_to[target].remove(eid)
                    except ValueError:
                        pass

        # Remove incoming edges
        for eid in list(self._edges_to.get(node_id, [])):
            edge = self._edges.pop(eid, None)
            if edge:
                source = edge["source"]
                if source in self._edges_from:
                    try:
                        self._edges_from[source].remove(eid)
                    except ValueError:
                        pass

        # Remove node itself
        self._edges_from.pop(node_id, None)
        self._edges_to.pop(node_id, None)
        self._nodes.pop(node_id, None)

    def _rebuild_faiss_locked(self) -> None:
        """Rebuild FAISS index from scratch. Caller must hold _lock."""
        if not self._embedder:
            logger.warning("Cannot rebuild FAISS: no embedder configured")
            return
        new_index = faiss.IndexIDMap(faiss.IndexFlatIP(self._dim))
        new_fid_to_node = {}
        new_node_to_fid = {}
        fid_counter = 0
        for node_id, node in self._nodes.items():
            content = node.get("content", "").strip()
            if not content:
                continue
            try:
                embedding = self._embed(content)
                vec = np.array([embedding], dtype="float32")
                faiss.normalize_L2(vec)
                fid_counter += 1
                ids = np.array([fid_counter], dtype="int64")
                new_index.add_with_ids(vec, ids)
                new_fid_to_node[fid_counter] = node_id
                new_node_to_fid[node_id] = fid_counter
            except Exception:
                logger.exception("Failed to re-index node %s", node_id)
        # Atomic swap
        self._faiss_index = new_index
        self._faiss_id_to_node = new_fid_to_node
        self._node_to_faiss_id = new_node_to_fid
        self._faiss_id_counter = fid_counter
        logger.info("FAISS index rebuilt: %d vectors", fid_counter)

    # ------------------------------------------------------------------
    # Consolidation (§8)
    # ------------------------------------------------------------------

    CONSOLIDATION_EXEMPT_TYPES = {
        "consolidation", "concern_created", "task_created", "situation_update",
    }

    def consolidate(self) -> int:
        """Compress old graph regions into summary nodes.

        Groups nodes older than consolidation_threshold_hours into 1-hour
        windows and replaces each window (with > min_nodes) with a single
        consolidation summary node. Returns the number of nodes consolidated.
        """
        threshold_s = self._config["consolidation_threshold_hours"] * 3600
        window_s = self._config["consolidation_window_hours"] * 3600
        min_nodes = self._config["consolidation_min_nodes"]
        cutoff = time.time() - threshold_s

        # Gather eligible nodes
        with self._lock:
            eligible = [
                (nid, node) for nid, node in self._nodes.items()
                if node["ts"] < cutoff
                and node["type"] not in self.CONSOLIDATION_EXEMPT_TYPES
            ]
        if not eligible:
            return 0

        # Group by window
        windows: dict[int, list[tuple[str, dict]]] = {}
        for nid, node in eligible:
            bucket = int(node["ts"] // window_s)
            windows.setdefault(bucket, []).append((nid, node))

        total_consolidated = 0
        for bucket, entries in sorted(windows.items()):
            if len(entries) < min_nodes:
                continue
            entries.sort(key=lambda x: x[1]["ts"])
            t_start = entries[0][1]["ts"]
            t_end = entries[-1][1]["ts"]

            # Count by type
            type_counts: dict[str, int] = {}
            concern_ids: set[str] = set()
            goal_count = 0
            goal_succeeded = 0
            salience_nodes: list[tuple[float, str]] = []

            for nid, node in entries:
                ntype = node["type"]
                type_counts[ntype] = type_counts.get(ntype, 0) + 1
                attrs = node.get("attrs", {})
                if ntype == "concern_change":
                    concern_ids.add(attrs.get("concern_id", ""))
                elif ntype == "goal_launch":
                    goal_count += 1
                elif ntype == "goal_outcome":
                    if attrs.get("success"):
                        goal_succeeded += 1
                # Estimate salience from assessment nodes
                if ntype == "assessment":
                    sal = attrs.get("salience", {})
                    novelty_score = {"high": 3, "moderate": 2, "low": 1, "none": 0}
                    score = novelty_score.get(
                        sal.get("novelty", "none") if isinstance(sal, dict) else "none", 0)
                    salience_nodes.append((score, node.get("content", "")))

            # Build template summary
            t_start_str = time.strftime("%H:%M", time.localtime(t_start))
            t_end_str = time.strftime("%H:%M", time.localtime(t_end))
            time_range = f"{t_start_str}–{t_end_str}"
            n_events = type_counts.get("event", 0)
            concern_list = ", ".join(sorted(concern_ids - {""})) or "none"

            summary = (
                f"[{time_range}] {n_events} events, {goal_count} goals "
                f"({goal_succeeded} succeeded), concerns active: {concern_list}."
            )

            # Append top-3 highest-salience content for better embedding
            salience_nodes.sort(key=lambda x: x[0], reverse=True)
            top_content = [c for _, c in salience_nodes[:3] if c]
            if top_content:
                summary += " Key: " + " | ".join(top_content)
            summary = summary[:self._max_content]

            # Create consolidation node
            con_nid = self.add_node("consolidation", summary,
                attrs={
                    "time_range_start": t_start,
                    "time_range_end": t_end,
                    "node_count_consolidated": len(entries),
                    "node_types_summary": type_counts,
                })

            # Prune original nodes
            node_ids_to_remove = [nid for nid, _ in entries]
            with self._lock:
                for nid in node_ids_to_remove:
                    self._remove_node_locked(nid)
            total_consolidated += len(entries)

        if total_consolidated:
            logger.info("Consolidated %d nodes into summary nodes", total_consolidated)
        return total_consolidated

    # ------------------------------------------------------------------
    # Context Assembly (§9.2)
    # ------------------------------------------------------------------

    def assemble_context(self, query_text: str, concern_activations: dict[str, float] = None,
                         k: int = 20, max_hops: int = 2) -> str:
        """Assemble a rendered context block for LLM prompts.

        1. Semantic search for relevant nodes
        2. Boost nodes connected to active concerns
        3. Expand subgraph for provenance
        4. Render as linearized narrative

        Args:
            query_text: the goal/question text to search for
            concern_activations: {concern_id: activation} for boosting
            k: number of seed nodes from semantic search
            max_hops: BFS expansion depth

        Returns:
            Rendered text block suitable for LLM prompt injection.
        """
        if not self._embedder or self.node_count() == 0:
            return ""

        # Step 1: seed selection
        try:
            raw_results = self.semantic_search(query_text, k=k)
        except Exception:
            return ""
        if not raw_results:
            return ""

        # Step 2: concern weighting — boost nodes connected to active concerns
        scored: list[tuple[str, float]] = []
        activations = concern_activations or {}
        for node_id, sim_score in raw_results:
            boost = 1.0
            if activations:
                # Check if node is connected to an active concern
                edges_out = self.get_edges_from(node_id, edge_type="bumped")
                edges_in = self.get_edges_to(node_id, edge_type="bumped")
                for edge in edges_out + edges_in:
                    # Look at the concern_change node's concern_id
                    other_id = edge["target"] if edge["source"] == node_id else edge["source"]
                    other = self.get_node(other_id)
                    if other and other["type"] == "concern_change":
                        cid = other["attrs"].get("concern_id", "")
                        if cid in activations:
                            boost = max(boost, 1.0 + activations[cid])
            scored.append((node_id, sim_score * boost))

        scored.sort(key=lambda x: x[1], reverse=True)
        seed_ids = [nid for nid, _ in scored[:k]]

        # Step 3: subgraph expansion
        nodes, edges = self.expand_subgraph(seed_ids, max_hops=max_hops)

        # Step 4: render
        return GraphRenderer.render(nodes, edges)


class GraphRenderer:
    """Serializes cognitive graph subgraphs into text for LLM context windows."""

    @staticmethod
    def render(nodes: list[dict], edges: list[dict]) -> str:
        """Render a subgraph as a linearized narrative with provenance markers.

        Organizes nodes into OODA chains (event → assessment → decision → result)
        and renders them chronologically with indented provenance.
        """
        if not nodes:
            return ""

        # Build lookup structures
        node_map = {n["node_id"]: n for n in nodes}
        edge_map: dict[str, list[dict]] = {}  # source -> [edges]
        for e in edges:
            edge_map.setdefault(e["source"], []).append(e)

        # Find chain roots: events and conversation_turns that start chains
        root_types = {"event", "conversation_turn", "consolidation",
                      "goal_launch", "concern_created"}
        roots = [n for n in nodes if n["type"] in root_types]
        roots.sort(key=lambda n: n["ts"])

        # Track which nodes have been rendered to avoid duplicates
        rendered: set[str] = set()
        lines: list[str] = []

        for root in roots:
            chain = GraphRenderer._render_chain(root, node_map, edge_map, rendered, depth=0)
            if chain:
                lines.append(chain)

        # Render any orphan nodes not part of a chain
        for n in sorted(nodes, key=lambda n: n["ts"]):
            if n["node_id"] not in rendered:
                rendered.add(n["node_id"])
                lines.append(GraphRenderer._render_node(n, depth=0))

        return "\n".join(lines)

    @staticmethod
    def render_snapshot(concern_states: list[dict], recent_events: list[dict],
                        active_goals: list[dict]) -> str:
        """Render a living-state-style snapshot from graph queries.

        Provides a quick summary of current concern activations,
        recent events, and active goals.
        """
        parts = []

        # Concerns
        if concern_states:
            parts.append("Concern activations:")
            for n in sorted(concern_states, key=lambda x: x["attrs"].get("new_activation", 0), reverse=True):
                attrs = n["attrs"]
                parts.append(
                    f"  {attrs.get('concern_label', attrs.get('concern_id', '?'))}: "
                    f"{attrs.get('new_activation', 0):.2f}")

        # Active goals
        if active_goals:
            parts.append("Active goals:")
            for g in active_goals:
                parts.append(f"  - {g['content'][:120]} [{g['attrs'].get('goal_id', '?')}]")

        # Recent events
        if recent_events:
            parts.append("Recent events:")
            for ev in recent_events[:10]:
                ts_str = time.strftime("%H:%M", time.localtime(ev["ts"]))
                source = ev["attrs"].get("source", "")
                parts.append(f"  [{ts_str}] ({source}) {ev['content'][:150]}")

        return "\n".join(parts)

    # ------------------------------------------------------------------
    # Internal rendering helpers
    # ------------------------------------------------------------------

    # Edge types that form provenance chains (source_type → target_type)
    _CHAIN_EDGES = {
        "observed", "decided_from", "executed", "spawned_goal",
        "produced", "bumped", "triggered_by", "nominated", "triaged",
        "spawned_task", "updated_concern",
    }

    @staticmethod
    def _render_chain(node: dict, node_map: dict, edge_map: dict,
                      rendered: set, depth: int) -> str:
        """Recursively render a node and its downstream chain."""
        nid = node["node_id"]
        if nid in rendered:
            return ""
        rendered.add(nid)

        lines = [GraphRenderer._render_node(node, depth)]

        # Follow outgoing chain edges
        for edge in edge_map.get(nid, []):
            if edge["type"] not in GraphRenderer._CHAIN_EDGES:
                continue
            target = node_map.get(edge["target"])
            if target and target["node_id"] not in rendered:
                child = GraphRenderer._render_chain(
                    target, node_map, edge_map, rendered, depth + 1)
                if child:
                    lines.append(child)

        return "\n".join(lines)

    @staticmethod
    def _render_node(node: dict, depth: int) -> str:
        """Render a single node as a text line."""
        ts_str = time.strftime("%H:%M", time.localtime(node["ts"]))
        indent = "  " * depth
        arrow = "→ " if depth > 0 else ""
        ntype = node["type"]
        content = node["content"][:200]
        attrs = node.get("attrs", {})

        if ntype == "event":
            source = attrs.get("source", "")
            return f"{indent}[{ts_str}] {arrow}Event ({source}): {content}"
        elif ntype == "conversation_turn":
            entity = attrs.get("entity", "")
            direction = attrs.get("direction", "")
            arrow_char = "←" if direction == "in" else "→"
            return f"{indent}[{ts_str}] {arrow}{arrow_char} {entity}: {content}"
        elif ntype == "assessment":
            action = attrs.get("action_choice", "")
            return f"{indent}{arrow}Assessment: {content}" + (f" → {action}" if action else "")
        elif ntype == "decision":
            return f"{indent}{arrow}Decided: {attrs.get('action_type', '')} — {content}"
        elif ntype == "action_result":
            ok = "success" if attrs.get("success") else "failed"
            return f"{indent}{arrow}Result: {ok}"
        elif ntype == "goal_launch":
            gid = attrs.get("goal_id", "")
            status = attrs.get("status", "active")
            return f"{indent}{arrow}Goal [{gid}] ({status}): {content}"
        elif ntype == "goal_outcome":
            ok = "success" if attrs.get("success") else "failed"
            pp = attrs.get("primary_product", "")
            extra = f", produced {pp}" if pp else ""
            return f"{indent}{arrow}Goal outcome: {ok}{extra} — {content}"
        elif ntype == "concern_change":
            return f"{indent}{arrow}Concern: {content}"
        elif ntype == "concern_created":
            return f"{indent}{arrow}New concern: {content}"
        elif ntype == "triage_nomination":
            return f"{indent}{arrow}Triage nominated: {content}"
        elif ntype == "triage_decision":
            action = attrs.get("action", "")
            return f"{indent}{arrow}Triage decided: {action} — {content}"
        elif ntype == "task_created":
            wip = attrs.get("task_wip_id", "")
            return f"{indent}{arrow}Task created [{wip}]: {content}"
        elif ntype == "consolidation":
            return f"{indent}[{ts_str}] {arrow}Summary: {content}"
        else:
            return f"{indent}[{ts_str}] {arrow}{ntype}: {content}"
