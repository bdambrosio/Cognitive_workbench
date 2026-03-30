"""Unit tests for CognitiveGraph (Phase 1)."""

import json
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from src.cognitive_graph import CognitiveGraph, GraphRenderer


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _fake_embedder(text: str) -> list[float]:
    """Deterministic 384-dim embedding based on hash of text."""
    rng = np.random.RandomState(hash(text) % (2**31))
    vec = rng.randn(384).astype("float32")
    return vec.tolist()


def _make_graph(**kwargs) -> CognitiveGraph:
    return CognitiveGraph(embedder=_fake_embedder, **kwargs)


# ---------------------------------------------------------------------------
# Node CRUD
# ---------------------------------------------------------------------------

class TestNodes:
    def test_add_and_get(self):
        g = _make_graph()
        nid = g.add_node("event", "User said hello", attrs={"source": "chat"})
        assert nid == "n_0001"
        node = g.get_node(nid)
        assert node["type"] == "event"
        assert node["content"] == "User said hello"
        assert node["attrs"]["source"] == "chat"
        assert node["session_id"] == g._session_id

    def test_get_missing_returns_none(self):
        g = _make_graph()
        assert g.get_node("n_9999") is None

    def test_content_truncation(self):
        g = _make_graph(config={"max_content_length": 20})
        nid = g.add_node("event", "A" * 100)
        assert len(g.get_node(nid)["content"]) == 20

    def test_node_count(self):
        g = _make_graph()
        assert g.node_count() == 0
        g.add_node("event", "a")
        g.add_node("event", "b")
        assert g.node_count() == 2

    def test_update_attrs(self):
        g = _make_graph()
        nid = g.add_node("goal_launch", "Do thing", attrs={"status": "active"})
        g.update_attrs(nid, {"status": "completed", "extra": 42})
        node = g.get_node(nid)
        assert node["attrs"]["status"] == "completed"
        assert node["attrs"]["extra"] == 42

    def test_update_attrs_missing_node(self):
        g = _make_graph()
        with pytest.raises(KeyError):
            g.update_attrs("n_9999", {"x": 1})


# ---------------------------------------------------------------------------
# Edges
# ---------------------------------------------------------------------------

class TestEdges:
    def test_add_and_query(self):
        g = _make_graph()
        n1 = g.add_node("event", "ev")
        n2 = g.add_node("assessment", "assess")
        eid = g.add_edge(n1, n2, "observed")
        assert eid == "e_0001"

        out = g.get_edges_from(n1)
        assert len(out) == 1
        assert out[0]["target"] == n2
        assert out[0]["type"] == "observed"

        inc = g.get_edges_to(n2)
        assert len(inc) == 1
        assert inc[0]["source"] == n1

    def test_filter_by_type(self):
        g = _make_graph()
        n1 = g.add_node("event", "ev")
        n2 = g.add_node("assessment", "a1")
        n3 = g.add_node("decision", "d1")
        g.add_edge(n1, n2, "observed")
        g.add_edge(n1, n3, "triggered_by")

        assert len(g.get_edges_from(n1, edge_type="observed")) == 1
        assert len(g.get_edges_from(n1, edge_type="triggered_by")) == 1
        assert len(g.get_edges_from(n1)) == 2

    def test_missing_source_raises(self):
        g = _make_graph()
        n1 = g.add_node("event", "ev")
        with pytest.raises(KeyError):
            g.add_edge("n_9999", n1, "observed")

    def test_missing_target_raises(self):
        g = _make_graph()
        n1 = g.add_node("event", "ev")
        with pytest.raises(KeyError):
            g.add_edge(n1, "n_9999", "observed")


# ---------------------------------------------------------------------------
# query_nodes
# ---------------------------------------------------------------------------

class TestQueryNodes:
    def test_filter_by_type(self):
        g = _make_graph()
        g.add_node("event", "e1")
        g.add_node("assessment", "a1")
        g.add_node("event", "e2")
        results = g.query_nodes(type="event")
        assert len(results) == 2
        assert all(n["type"] == "event" for n in results)

    def test_filter_by_attrs(self):
        g = _make_graph()
        g.add_node("concern_change", "c1", attrs={"concern_id": "attend"})
        g.add_node("concern_change", "c2", attrs={"concern_id": "homeostasis"})
        results = g.query_nodes(attrs_filter={"concern_id": "attend"})
        assert len(results) == 1
        assert results[0]["attrs"]["concern_id"] == "attend"

    def test_time_range(self):
        g = _make_graph()
        t0 = 1000.0
        g.add_node("event", "old", ts=t0)
        g.add_node("event", "mid", ts=t0 + 100)
        g.add_node("event", "new", ts=t0 + 200)
        results = g.query_nodes(since=t0 + 50, until=t0 + 150)
        assert len(results) == 1
        assert results[0]["content"] == "mid"

    def test_limit(self):
        g = _make_graph()
        for i in range(20):
            g.add_node("event", f"e{i}")
        results = g.query_nodes(limit=5)
        assert len(results) == 5

    def test_most_recent_first(self):
        g = _make_graph()
        g.add_node("event", "old", ts=100.0)
        g.add_node("event", "new", ts=200.0)
        results = g.query_nodes()
        assert results[0]["content"] == "new"


# ---------------------------------------------------------------------------
# Semantic search
# ---------------------------------------------------------------------------

class TestSemanticSearch:
    def test_basic_search(self):
        g = _make_graph()
        g.add_node("event", "The cat sat on the mat")
        g.add_node("event", "Quantum computing advances")
        g.add_node("assessment", "User asked about cats")

        results = g.semantic_search("cat", k=2)
        assert len(results) <= 2
        # Returns (node_id, score) tuples
        assert all(isinstance(r, tuple) and len(r) == 2 for r in results)

    def test_type_filter_string(self):
        g = _make_graph()
        g.add_node("event", "alpha bravo")
        g.add_node("assessment", "alpha charlie")
        results = g.semantic_search("alpha", k=10, type_filter="event")
        assert all(g.get_node(nid)["type"] == "event" for nid, _ in results)

    def test_type_filter_list(self):
        g = _make_graph()
        g.add_node("event", "alpha")
        g.add_node("assessment", "alpha")
        g.add_node("decision", "alpha")
        results = g.semantic_search("alpha", k=10,
                                     type_filter=["event", "assessment"])
        types = {g.get_node(nid)["type"] for nid, _ in results}
        assert "decision" not in types

    def test_empty_graph(self):
        g = _make_graph()
        assert g.semantic_search("anything") == []

    def test_no_embedder_raises(self):
        g = CognitiveGraph()  # no embedder
        g.add_node("event", "test")
        with pytest.raises(RuntimeError):
            g.semantic_search("test")


# ---------------------------------------------------------------------------
# Subgraph expansion
# ---------------------------------------------------------------------------

class TestExpandSubgraph:
    def _build_chain(self):
        """event -> assessment -> decision -> action_result"""
        g = _make_graph()
        n1 = g.add_node("event", "ev")
        n2 = g.add_node("assessment", "assess")
        n3 = g.add_node("decision", "decide")
        n4 = g.add_node("action_result", "result")
        g.add_edge(n1, n2, "observed")
        g.add_edge(n2, n3, "decided_from")
        g.add_edge(n3, n4, "executed")
        return g, n1, n2, n3, n4

    def test_zero_hops(self):
        g, n1, n2, n3, n4 = self._build_chain()
        nodes, edges = g.expand_subgraph([n1], max_hops=0)
        assert len(nodes) == 1
        assert nodes[0]["node_id"] == n1

    def test_one_hop(self):
        g, n1, n2, n3, n4 = self._build_chain()
        nodes, edges = g.expand_subgraph([n2], max_hops=1)
        node_ids = {n["node_id"] for n in nodes}
        # n2 + neighbors n1 and n3
        assert node_ids == {n1, n2, n3}

    def test_full_chain(self):
        g, n1, n2, n3, n4 = self._build_chain()
        nodes, edges = g.expand_subgraph([n1], max_hops=3)
        assert len(nodes) == 4
        assert len(edges) == 3

    def test_edge_type_filter(self):
        g, n1, n2, n3, n4 = self._build_chain()
        nodes, edges = g.expand_subgraph([n2], max_hops=2,
                                          edge_types=["decided_from"])
        node_ids = {n["node_id"] for n in nodes}
        # Only follows decided_from: n2 -> n3
        assert n1 not in node_ids
        assert n3 in node_ids


# ---------------------------------------------------------------------------
# latest_per_key
# ---------------------------------------------------------------------------

class TestLatestPerKey:
    def test_basic(self):
        g = _make_graph()
        g.add_node("concern_change", "old attend",
                    attrs={"concern_id": "attend", "new_activation": 0.3}, ts=100.0)
        g.add_node("concern_change", "new attend",
                    attrs={"concern_id": "attend", "new_activation": 0.7}, ts=200.0)
        g.add_node("concern_change", "homeo",
                    attrs={"concern_id": "homeostasis", "new_activation": 0.5}, ts=150.0)

        results = g.latest_per_key("concern_change", "concern_id")
        by_id = {n["attrs"]["concern_id"]: n for n in results}
        assert len(by_id) == 2
        assert by_id["attend"]["attrs"]["new_activation"] == 0.7
        assert by_id["homeostasis"]["attrs"]["new_activation"] == 0.5


# ---------------------------------------------------------------------------
# Pruning
# ---------------------------------------------------------------------------

class TestPrune:
    def test_basic_prune(self):
        g = _make_graph()
        n1 = g.add_node("event", "old", ts=100.0)
        n2 = g.add_node("event", "new", ts=300.0)
        g.add_edge(n1, n2, "observed")

        deleted = g.prune_before(200.0)
        assert deleted == 1
        assert g.get_node(n1) is None
        assert g.get_node(n2) is not None
        assert g.node_count() == 1
        # Edge referencing pruned node should be gone
        assert g.get_edges_to(n2) == []

    def test_exclude_types(self):
        g = _make_graph()
        g.add_node("event", "old event", ts=100.0)
        g.add_node("concern_created", "old concern", ts=100.0)
        deleted = g.prune_before(200.0, exclude_types=["concern_created"])
        assert deleted == 1
        assert g.node_count() == 1

    def test_prune_removes_from_faiss(self):
        g = _make_graph()
        n1 = g.add_node("event", "test content for faiss", ts=100.0)
        assert g._faiss_index.ntotal == 1
        g.prune_before(200.0)
        assert g._faiss_index.ntotal == 0
        assert n1 not in g._node_to_faiss_id


# ---------------------------------------------------------------------------
# Persistence
# ---------------------------------------------------------------------------

class TestPersistence:
    def test_save_and_load(self):
        g = _make_graph()
        n1 = g.add_node("event", "hello world", attrs={"source": "test"})
        n2 = g.add_node("assessment", "assessed it")
        g.add_edge(n1, n2, "observed")

        with tempfile.TemporaryDirectory() as tmpdir:
            path = str(Path(tmpdir) / "cognitive_graph")
            g.save(path)

            # Verify files exist
            assert Path(path).with_suffix(".json").exists()
            assert Path(path).with_suffix(".faiss").exists()

            # Load into new graph
            g2 = _make_graph()
            g2.load(path)
            assert g2.node_count() == 2
            node = g2.get_node(n1)
            assert node["content"] == "hello world"
            assert node["attrs"]["source"] == "test"
            edges = g2.get_edges_from(n1)
            assert len(edges) == 1
            assert edges[0]["type"] == "observed"

    def test_load_new_session(self):
        g = _make_graph()
        old_session = g._session_id
        g.add_node("event", "test")

        with tempfile.TemporaryDirectory() as tmpdir:
            path = str(Path(tmpdir) / "cg")
            g.save(path)
            g.load(path)
            assert g._session_id != old_session

    def test_load_missing_file(self):
        g = _make_graph()
        g.load("/nonexistent/path/cg")  # should not raise
        assert g.node_count() == 0

    def test_roundtrip_faiss(self):
        g = _make_graph()
        g.add_node("event", "searchable content about cats")
        g.add_node("event", "quantum computing paper")

        with tempfile.TemporaryDirectory() as tmpdir:
            path = str(Path(tmpdir) / "cg")
            g.save(path)

            g2 = _make_graph()
            g2.load(path)
            # FAISS index should be functional after load
            results = g2.semantic_search("cats", k=2)
            assert len(results) > 0


# ---------------------------------------------------------------------------
# Thread safety
# ---------------------------------------------------------------------------

class TestThreadSafety:
    def test_concurrent_adds(self):
        g = _make_graph()
        errors = []

        def add_nodes(prefix, count):
            try:
                for i in range(count):
                    g.add_node("event", f"{prefix}_{i}")
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=add_nodes, args=(f"t{t}", 50))
                   for t in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert not errors
        assert g.node_count() == 200

    def test_concurrent_read_write(self):
        g = _make_graph()
        stop = threading.Event()
        errors = []

        def writer():
            try:
                for i in range(100):
                    n = g.add_node("event", f"write_{i}")
                    if i > 0:
                        prev = f"n_{i:04d}"
                        try:
                            g.add_edge(prev, n, "observed")
                        except KeyError:
                            pass  # race with pruning is ok
            except Exception as e:
                errors.append(e)
            finally:
                stop.set()

        def reader():
            try:
                while not stop.is_set():
                    g.query_nodes(type="event", limit=10)
                    g.node_count()
            except Exception as e:
                errors.append(e)

        t_write = threading.Thread(target=writer)
        t_read = threading.Thread(target=reader)
        t_read.start()
        t_write.start()
        t_write.join()
        t_read.join()
        assert not errors


# ---------------------------------------------------------------------------
# GraphRenderer
# ---------------------------------------------------------------------------

class TestGraphRenderer:
    def _build_ooda_chain(self):
        """Build a small OODA chain: event → assessment → decision → action_result."""
        g = _make_graph()
        t = 1000.0
        n1 = g.add_node("event", "User asked about cats", ts=t,
                          attrs={"event_type": "user_text", "source": "User",
                                 "classification": "chat"})
        n2 = g.add_node("assessment", "Strong attend_to_user, novel request", ts=t + 1,
                          attrs={"action_choice": "chat_response",
                                 "salience": {"novelty": "high"}})
        n3 = g.add_node("decision", "Decided: chat_response", ts=t + 2,
                          attrs={"action_type": "chat_response",
                                 "payload_summary": "text about cats"})
        n4 = g.add_node("action_result", "Executed chat_response: success", ts=t + 3,
                          attrs={"action_type": "chat_response", "success": True})
        g.add_edge(n1, n2, "observed")
        g.add_edge(n2, n3, "decided_from")
        g.add_edge(n3, n4, "executed")
        return g, [n1, n2, n3, n4]

    def test_render_chain(self):
        g, nids = self._build_ooda_chain()
        nodes = [g.get_node(nid) for nid in nids]
        edges = []
        for nid in nids:
            edges.extend(g.get_edges_from(nid))
        text = GraphRenderer.render(nodes, edges)
        assert "Event (User)" in text
        assert "Assessment:" in text
        assert "Decided:" in text
        assert "Result: success" in text

    def test_render_preserves_order(self):
        g = _make_graph()
        n1 = g.add_node("event", "First event", ts=100.0,
                          attrs={"source": "sensor"})
        n2 = g.add_node("event", "Second event", ts=200.0,
                          attrs={"source": "User"})
        nodes = [g.get_node(n2), g.get_node(n1)]  # reversed
        text = GraphRenderer.render(nodes, [])
        # First event should appear before second
        pos1 = text.find("First event")
        pos2 = text.find("Second event")
        assert pos1 < pos2

    def test_render_snapshot(self):
        g = _make_graph()
        g.add_node("concern_change", "attend bumped",
                    attrs={"concern_id": "attend", "concern_label": "attend_to_user",
                           "new_activation": 0.72}, ts=100.0)
        g.add_node("event", "User said hello",
                    attrs={"source": "User"}, ts=200.0)
        g.add_node("goal_launch", "Respond to User",
                    attrs={"goal_id": "g1", "status": "active"}, ts=300.0)

        concerns = g.latest_per_key("concern_change", "concern_id")
        events = g.query_nodes(type="event", limit=10)
        goals = g.query_nodes(type="goal_launch",
                               attrs_filter={"status": "active"})
        text = GraphRenderer.render_snapshot(concerns, events, goals)
        assert "attend_to_user" in text
        assert "0.72" in text
        assert "Respond to User" in text
        assert "User said hello" in text

    def test_render_empty(self):
        assert GraphRenderer.render([], []) == ""


# ---------------------------------------------------------------------------
# Consolidation
# ---------------------------------------------------------------------------

class TestConsolidation:
    def test_basic_consolidation(self):
        g = _make_graph(config={
            "consolidation_threshold_hours": 0,  # consolidate everything
            "consolidation_window_hours": 1,
            "consolidation_min_nodes": 3,
        })
        # Create 5 nodes in the same 1-hour window
        base_ts = 1000.0
        for i in range(5):
            g.add_node("event", f"Event {i}", ts=base_ts + i * 60,
                        attrs={"source": "test"})

        initial_count = g.node_count()
        consolidated = g.consolidate()
        assert consolidated == 5
        # Should have 1 consolidation node
        cons_nodes = g.query_nodes(type="consolidation")
        assert len(cons_nodes) == 1
        assert "5 events" in cons_nodes[0]["content"]

    def test_exempt_types_preserved(self):
        g = _make_graph(config={
            "consolidation_threshold_hours": 0,
            "consolidation_window_hours": 1,
            "consolidation_min_nodes": 3,
        })
        base_ts = 1000.0
        for i in range(5):
            g.add_node("event", f"Event {i}", ts=base_ts + i * 60)
        g.add_node("concern_created", "Important concern", ts=base_ts + 10)
        g.add_node("task_created", "Important task", ts=base_ts + 20)

        g.consolidate()
        # Exempt nodes should survive
        assert len(g.query_nodes(type="concern_created")) == 1
        assert len(g.query_nodes(type="task_created")) == 1

    def test_below_min_nodes_not_consolidated(self):
        g = _make_graph(config={
            "consolidation_threshold_hours": 0,
            "consolidation_window_hours": 1,
            "consolidation_min_nodes": 10,
        })
        for i in range(3):
            g.add_node("event", f"Event {i}", ts=1000.0 + i * 60)

        consolidated = g.consolidate()
        assert consolidated == 0
        assert g.node_count() == 3

    def test_recent_nodes_not_consolidated(self):
        g = _make_graph(config={
            "consolidation_threshold_hours": 4,
            "consolidation_window_hours": 1,
            "consolidation_min_nodes": 3,
        })
        # Create recent nodes (within threshold)
        now = time.time()
        for i in range(5):
            g.add_node("event", f"Event {i}", ts=now - 60 + i)

        consolidated = g.consolidate()
        assert consolidated == 0

    def test_consolidation_preserves_salience_content(self):
        g = _make_graph(config={
            "consolidation_threshold_hours": 0,
            "consolidation_window_hours": 1,
            "consolidation_min_nodes": 3,
        })
        base_ts = 1000.0
        g.add_node("event", "Boring event", ts=base_ts)
        g.add_node("assessment", "Important assessment about quantum computing",
                    ts=base_ts + 60,
                    attrs={"salience": {"novelty": "high"}})
        g.add_node("event", "Another event", ts=base_ts + 120)
        g.add_node("event", "Third event", ts=base_ts + 180)
        g.add_node("event", "Fourth event", ts=base_ts + 240)

        g.consolidate()
        cons = g.query_nodes(type="consolidation")
        assert len(cons) == 1
        # High-salience content should be preserved in summary
        assert "quantum computing" in cons[0]["content"]


# ---------------------------------------------------------------------------
# Context Assembly
# ---------------------------------------------------------------------------

class TestContextAssembly:
    def test_assemble_context(self):
        g = _make_graph()
        t = 1000.0
        n1 = g.add_node("event", "User asked about machine learning", ts=t,
                          attrs={"source": "User"})
        n2 = g.add_node("assessment", "Novel ML question, strong attend", ts=t + 1,
                          attrs={"action_choice": "chat_response"})
        g.add_edge(n1, n2, "observed")

        ctx = g.assemble_context("machine learning")
        assert ctx  # non-empty
        assert "machine learning" in ctx.lower() or "ML" in ctx

    def test_assemble_empty_graph(self):
        g = _make_graph()
        ctx = g.assemble_context("anything")
        assert ctx == ""

    def test_assemble_no_embedder(self):
        g = CognitiveGraph()  # no embedder
        g.add_node("event", "test")
        ctx = g.assemble_context("test")
        assert ctx == ""
