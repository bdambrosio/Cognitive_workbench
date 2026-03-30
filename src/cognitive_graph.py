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
