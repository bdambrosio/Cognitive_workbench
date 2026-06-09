"""Thread anchors (activity-level state surfaces with centroid
embeddings) — ThreadsMixin for ChatLoop, moved verbatim from chat_loop.py
in the 2026-06 mixin refactor."""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger('chat_loop')

_AGENT_THREADS_COLLECTION_NAME  = "agent_threads"


class ThreadsMixin:
    """Mixin for ChatLoop — moved verbatim from chat_loop.py."""

    def _init_agent_threads(self) -> None:
        """Get-or-create the agent_threads Collection, mark it persistent,
        ensure semantic index exists. Threads are activity-level anchors;
        each note carries a centroid_embedding (activation-weighted mean
        of constituent turn embeddings) in its properties for direct
        cosine-similarity activation against new turns. The note's
        text field holds the LLM-generated summary, which the semantic
        index will pick up — useful for thread-similarity queries during
        consolidation. Idempotent across restarts."""
        try:
            cid = self.resource_manager.named_collections.get(_AGENT_THREADS_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _AGENT_THREADS_COLLECTION_NAME,
                    {"kind": "agent_threads"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create agent_threads collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self.resource_manager.index_collection(self.character_name, cid, index_type='semantic')
            self._agent_threads_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_agent_threads failed: {e}")

    # ------------------------------------------------------------------
    # Thread anchors. Threads are activity-level state surfaces with a
    # centroid_embedding stored as a property (NOT in the FAISS index;
    # the index covers the summary text for thread-similarity queries
    # during consolidation, but activation matching uses the centroid
    # directly via cosine similarity over the active-thread set).
    # No firing, no decay, no autonomous behavior. Read-only at this
    # stage — install populated by tools/threads_install.py; reflection-
    # side updates land in Stage 5.
    # ------------------------------------------------------------------

    def _add_thread(self, name: str, summary: str,
                    centroid_embedding: List[float],
                    constituent_turn_count: int = 0,
                    creation_provenance: str = 'discovered',
                    status: str = 'active',
                    exemplar_pairs: Optional[List[Dict[str, str]]] = None,
                    extra_properties: Optional[Dict[str, Any]] = None
                    ) -> Optional[str]:
        """Create a thread note in the agent_threads Collection. The
        note's text field gets the summary; centroid_embedding is
        stored as a list[float] in properties. No similarity-based
        dedup at this layer (callers are expected to enforce
        idempotency by checking name).

        exemplar_pairs is a render-only payload — list of
        {"user": str, "agent": str} dicts captured at bootstrap as
        representative exchanges. Surfaced inside the active-threads
        block to give Jill concrete touchstones for what the thread is.
        Never read by activation, classification, or centroid drift."""
        if not self._agent_threads_collection_id:
            return None
        name = (name or "").strip()
        summary = (summary or "").strip()
        if not name or not summary:
            return None
        if not centroid_embedding:
            logger.warning(f"[{self.character_name}] _add_thread: centroid_embedding is empty for {name!r}")
            return None
        if creation_provenance not in ('bootstrap', 'discovered'):
            creation_provenance = 'discovered'
        if status not in ('active', 'dormant', 'archived'):
            status = 'active'
        now_iso = datetime.now(timezone.utc).isoformat()
        # Note: `name` is stored as `note_name` via create_note's positional
        # arg (which lands in properties.note_name); `created_at` is set
        # automatically by create_note. Don't duplicate them here.
        clean_exemplars: List[Dict[str, str]] = []
        for ex in (exemplar_pairs or []):
            u = str((ex or {}).get("user") or "").strip()
            a = str((ex or {}).get("agent") or "").strip()
            if u and a:
                clean_exemplars.append({"user": u, "agent": a})
        properties: Dict[str, Any] = {
            "kind": "thread",
            "status": status,
            "summary": summary,
            "centroid_embedding": list(map(float, centroid_embedding)),
            "constituent_turn_count": int(constituent_turn_count),
            "creation_provenance": creation_provenance,
            "last_activated_at": now_iso,
            "last_centroid_update_at": now_iso,
            "attached_concern_ids": [],
            "attached_rule_ids": [],
            "exemplar_pairs": clean_exemplars,
        }
        if extra_properties:
            properties.update(extra_properties)
        try:
            with self._faiss_lock:
                success, note_id, err, _ = self.resource_manager.create_note(
                    self.character_name, summary, "text", "chat-loop",
                    "", name, properties)
                if not success or not note_id:
                    logger.warning(f"[{self.character_name}] thread create failed: {err}")
                    return None
                self.resource_manager.mark_persistent(note_id, self.character_name)
                ok, _, add_err = self.resource_manager.add_to_collection(
                    self._agent_threads_collection_id, note_id, self.character_name)
                if not ok:
                    logger.warning(f"[{self.character_name}] thread add_to_collection failed: {add_err}")
                return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _add_thread failed: {e}")
            return None

    def _get_threads(self, statuses: Optional[Tuple[str, ...]] = ('active',)
                     ) -> List[Dict[str, Any]]:
        """Return thread records matching any of the given statuses
        (default: active only). Each record is the resource dict with
        properties merged in for convenience."""
        if not self._agent_threads_collection_id:
            return []
        try:
            coll = self.resource_manager.get_resource(self._agent_threads_collection_id)
            if not coll:
                return []
            note_ids = (coll.get("properties") or {}).get("content", []) or []
            out: List[Dict[str, Any]] = []
            status_set = set(statuses) if statuses else None
            for nid in note_ids:
                note = self.resource_manager.get_resource(nid)
                if not note:
                    continue
                props = note.get("properties") or {}
                if props.get("kind") != "thread":
                    continue
                if status_set is not None and props.get("status") not in status_set:
                    continue
                # Human-readable name lives in properties.note_name (set
                # via create_note's positional arg); created_at is set by
                # create_note automatically.
                out.append({
                    "id": nid,
                    "name": props.get("note_name", ""),
                    "summary": props.get("summary", ""),
                    "status": props.get("status", "active"),
                    "centroid_embedding": props.get("centroid_embedding") or [],
                    "constituent_turn_count": int(props.get("constituent_turn_count", 0)),
                    "creation_provenance": props.get("creation_provenance"),
                    "created_at": props.get("created_at"),
                    "last_activated_at": props.get("last_activated_at"),
                    "last_centroid_update_at": props.get("last_centroid_update_at"),
                    "attached_concern_ids": props.get("attached_concern_ids") or [],
                    "attached_rule_ids": props.get("attached_rule_ids") or [],
                    "exemplar_pairs": props.get("exemplar_pairs") or [],
                })
            return out
        except Exception as e:
            logger.warning(f"[{self.character_name}] _get_threads failed: {e}")
            return []

    def _find_thread_by_name(self, name: str) -> Optional[Dict[str, Any]]:
        """Return the active thread record with the given name, or None."""
        for t in self._get_threads(statuses=None):
            if t.get("name") == name:
                return t
        return None

    def _compute_thread_activation(self, text: str, temperature: float = 4.0
                                   ) -> List[Tuple[Dict[str, Any], float]]:
        """Compute the activation distribution over active threads for
        the given text. Embeds the text with the same model used
        elsewhere (bge-small-en-v1.5, L2-normalized), computes cosine
        similarity to each active thread's centroid (also L2-normalized),
        and applies softmax with the given temperature.

        Higher temperature → flatter distribution (more threads share
        weight). temperature=1.0 is "honest softmax" but tends to
        produce winner-take-all assignments because cosine similarities
        in this embedding space cluster within a narrow range.
        temperature=4.0 (default) gives a more usable distribution where
        secondary threads carry meaningful weight.

        Returns a list of (thread_record, weight) tuples sorted by
        weight descending. Empty list if no active threads or text is
        empty. The list represents a probability distribution; weights
        sum to 1.0 (within float precision).

        Cheap — single embedding call + N cosine sims for N active
        threads. With <100 active threads this is sub-millisecond on
        the GPU."""
        text = (text or "").strip()
        if not text:
            return []
        threads = self._get_threads(statuses=('active',))
        if not threads:
            return []

        try:
            self.resource_manager._init_embedder()
            embedder = self.resource_manager.embedder
            if embedder is None:
                logger.warning(
                    f"[{self.character_name}] _compute_thread_activation: "
                    f"embedder unavailable")
                return []
            import numpy as np
            turn_emb = embedder.encode(
                text, normalize_embeddings=True, convert_to_numpy=True,
                show_progress_bar=False)
            # Cache for reuse by _update_thread_centroids in
            # _post_turn_work — same embedding feeds both activation
            # readout and post-turn centroid drift.
            self._current_turn_embedding = turn_emb
            # Cosine similarity = dot product on L2-normalized vectors.
            sims: List[float] = []
            valid_threads: List[Dict[str, Any]] = []
            for t in threads:
                c = t.get("centroid_embedding") or []
                if not c:
                    continue
                cv = np.asarray(c, dtype=np.float32)
                if cv.shape != turn_emb.shape:
                    logger.warning(
                        f"[{self.character_name}] thread {t.get('name')!r} "
                        f"centroid dim {cv.shape} != turn emb dim "
                        f"{turn_emb.shape} — skipping")
                    continue
                sims.append(float(np.dot(turn_emb, cv)))
                valid_threads.append(t)
            if not sims:
                return []
            # Softmax with temperature. We expect cos sim in roughly
            # [-0.2, 0.9] range for bge-small; temperature scales these
            # before exp.
            arr = np.asarray(sims, dtype=np.float64) * float(temperature)
            arr -= arr.max()  # numerical stability
            exp = np.exp(arr)
            weights = exp / exp.sum()
            paired = list(zip(valid_threads, [float(w) for w in weights]))
            paired.sort(key=lambda x: -x[1])
            return paired
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] _compute_thread_activation failed: {e}")
            return []

    def _render_active_threads_block(
            self,
            activation: List[Tuple[Dict[str, Any], float]],
            primary_threshold: float = 0.30,
            secondary_threshold: float = 0.20,
            secondary_cap: int = 2,
            ) -> str:
        """Render the activation-weighted thread block for the system
        prompt. Returns empty string when no thread has meaningful
        activation — caller should skip the block in that case.

        Two thresholds, both load-bearing:

        - primary_threshold: minimum weight for the top thread to
          count as a primary. Below this, the distribution is too
          uniform to claim any thread is "what we're working on" —
          render nothing rather than a misleading attribution. With
          5 active threads, uniform random would give 0.20 each;
          threshold 0.30 means the primary is at least 1.5× uniform.

        - secondary_threshold: minimum weight for a thread to appear
          in "also touching on". Below this, the secondary is too
          weak to be informative.

        secondary_cap bounds the secondaries list (top-N after
        filtering). Activation numbers themselves are not surfaced
        as floats — prominence in the prompt encodes the
        distribution."""
        if not activation:
            return ""
        primary, primary_w = activation[0]
        if primary_w < primary_threshold:
            # No thread is clearly active; suppress the block rather
            # than mislead with a low-confidence "primary" assignment.
            return ""

        primary_name = primary.get("name", "")
        primary_summary = (primary.get("summary") or "").strip()

        secondaries: List[Tuple[Dict[str, Any], float]] = []
        for t, w in activation[1:]:
            if w < secondary_threshold:
                break
            secondaries.append((t, w))
            if len(secondaries) >= secondary_cap:
                break

        def _render_exemplars(thread: Dict[str, Any], k: int,
                              indent: str = "") -> List[str]:
            """Format up to k exemplar pairs as 'User: … / Jill: …'
            lines. Returns empty list when the thread has no exemplars
            (e.g. installed before exemplars existed). Render-only —
            never feeds back into activation or drift."""
            pairs = thread.get("exemplar_pairs") or []
            if not pairs:
                return []
            out: List[str] = []
            for ex in pairs[:k]:
                u = str((ex or {}).get("user") or "").strip()
                a = str((ex or {}).get("agent") or "").strip()
                if not u or not a:
                    continue
                out.append(f"{indent}- User: {u}")
                out.append(f"{indent}  Jill: {a}")
            return out

        lines: List[str] = []
        lines.append("## Current activity context (from session threads)")
        lines.append(
            "Threads are activity-level anchors inferred from the "
            "shape of recent conversation. The primary thread reflects "
            "what we're most engaged with right now; secondary threads "
            "are activities the current turn also touches. Exemplar "
            "exchanges (when present) are representative past pairs "
            "from each thread, captured at bootstrap.")
        lines.append("")
        lines.append(f"**Primary:** `{primary_name}` — {primary_summary}")
        primary_ex = _render_exemplars(primary, k=2)
        if primary_ex:
            lines.append("Exemplar exchanges:")
            lines.extend(primary_ex)
        if secondaries:
            lines.append("")
            lines.append("**Also touching on:**")
            for t, _w in secondaries:
                name = t.get("name", "")
                summary = (t.get("summary") or "").strip()
                lines.append(f"- `{name}` — {summary}")
                lines.extend(_render_exemplars(t, k=1, indent="  "))
        return "\n".join(lines)

    def _update_thread_centroids(
            self,
            min_activation: float = 0.20,
            learning_rate: float = 0.05,
            ) -> int:
        """Stage 5 — incremental centroid evolution. Drift each active
        thread's centroid toward the current turn's embedding,
        weighted by how much that thread participated in the turn.

        Algorithm: for each active thread t with activation_t above
        min_activation:

            c_new  = c_old + lr * activation_t * (turn_emb - c_old)
            c_norm = c_new / ||c_new||
            n_new  = n_old + activation_t

        With lr=0.05 and activation=0.5 a single turn shifts the
        centroid by 2.5%; the constituent count grows by 0.5 (real-
        valued count, since membership is graded).

        min_activation defaults to 0.20, matching the secondary-render
        threshold in _render_active_threads_block: a thread that's
        below the 'meaningful enough to surface' threshold shouldn't
        drift toward the current turn either. The bge-small embedding
        space has a ~13-17% noise floor on cosine activation across
        unrelated threads; updating below the secondary threshold
        would smear all centroids on every off-topic query.

        Mutates the underlying note properties in-place; persistence
        is handled by the post-turn _persist_to_disk call. Returns
        the number of thread centroids actually updated."""
        activation = self._current_thread_activation
        turn_emb = self._current_turn_embedding
        if not activation or turn_emb is None:
            return 0
        try:
            import numpy as np
        except ImportError:
            logger.warning(
                f"[{self.character_name}] _update_thread_centroids: numpy unavailable")
            return 0

        now_iso = datetime.now(timezone.utc).isoformat()
        updated = 0
        for thread, weight in activation:
            if weight < min_activation:
                continue
            note_id = thread.get("id")
            if not note_id:
                continue
            note = self.resource_manager.get_resource(note_id)
            if not note:
                continue
            props = note.get("properties") or {}
            old_c = props.get("centroid_embedding") or []
            if not old_c:
                continue
            try:
                old_arr = np.asarray(old_c, dtype=np.float32)
                if old_arr.shape != turn_emb.shape:
                    logger.warning(
                        f"[{self.character_name}] centroid update: dim "
                        f"mismatch for thread {props.get('note_name')!r}, skipping")
                    continue
                step = float(learning_rate) * float(weight)
                new_arr = old_arr + step * (turn_emb - old_arr)
                norm = float(np.linalg.norm(new_arr))
                if norm > 0:
                    new_arr = new_arr / norm
                old_count = float(props.get("constituent_turn_count") or 0)
                new_count = old_count + float(weight)
                props["centroid_embedding"] = [float(x) for x in new_arr.tolist()]
                props["constituent_turn_count"] = new_count
                props["last_centroid_update_at"] = now_iso
                # last_activated_at marks the most recent turn this
                # thread had non-trivial weight on — useful for
                # consolidation / archival logic later.
                props["last_activated_at"] = now_iso
                updated += 1
            except Exception as e:
                logger.warning(
                    f"[{self.character_name}] centroid update for "
                    f"{props.get('note_name')!r} failed: {e}")
                continue
        if updated:
            logger.info(
                f"[{self.character_name}] thread centroids updated: "
                f"{updated}/{len(activation)} threads")
        return updated

    def _render_threads_for_subagent(
            self, threads: List[Dict[str, Any]]) -> str:
        """Render the full active-threads list as a system-prompt
        section for subagents (currently: remember). Distinct from
        _render_active_threads_block, which surfaces only the
        currently-activated threads to Jill's main turn — subagents
        operating over the whole memory benefit from seeing the full
        thread inventory regardless of current activation, since
        retrospective queries may target dormant-but-active threads."""
        if not threads:
            return ""
        lines: List[str] = []
        lines.append("## Session threads (activity-level structure of the conversation)")
        lines.append(
            "Threads are activity-level anchors inferred from the "
            "shape of the user's conversation. Each thread groups "
            "turns that participated in a coherent activity. Use these "
            "to (a) understand the topical structure of the user's "
            "interactions, (b) scope retrospective queries to the "
            "relevant thread when the question is about a specific "
            "activity, and (c) answer direct enumeration questions "
            "(\"what threads do you have?\") without needing to read "
            "memory files. Numbers are constituent turn counts.")
        lines.append("")
        for t in threads:
            name = t.get("name", "")
            n = int(t.get("constituent_turn_count", 0))
            summary = (t.get("summary") or "").strip()
            lines.append(f"- `{name}` ({n} turns): {summary}")
        return "\n".join(lines)
