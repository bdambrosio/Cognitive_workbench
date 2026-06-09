"""Long-term memories (FAISS-indexed Collection) — MemoriesMixin for
ChatLoop, moved verbatim from chat_loop.py in the 2026-06 mixin refactor."""

from __future__ import annotations

import json
import logging
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger('chat_loop')

# Per-character collection holding episodic specifics across conversations.
# Auto-RAG searches this on every turn; post-turn reflection writes to it.
_MEMORIES_COLLECTION_NAME = "memories"

# Memory subtype, stored on the note's `properties.category`. Distinct from
# the existing `properties.kind="memory"` discriminator (which says what
# class of note this is — memory vs companion_state vs web_search etc).
# Reflection picks one per memory; default is `fact`. Unknown values from
# the model are coerced to `fact` rather than dropped.
_MEMORY_CATEGORIES = ('fact', 'preference', 'commitment')


class MemoriesMixin:
    """Mixin for ChatLoop — moved verbatim from chat_loop.py."""

    # ------------------------------------------------------------------
    # Long-term memory — per-character "memories" Collection.
    # Notes carry kind=memory + entity; the Collection is FAISS-indexed so
    # `search_collection` returns ranked chunks. add_to_collection
    # auto-updates the index for indexed Collections.
    # ------------------------------------------------------------------

    def _init_memories(self) -> None:
        """Get-or-create the memories Collection, mark it persistent, ensure
        it has a semantic index. Idempotent: safe across restarts because
        load_from_file restores named_collections, and index_collection is
        a no-op for already-indexed collections."""
        try:
            cid = self.resource_manager.named_collections.get(_MEMORIES_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _MEMORIES_COLLECTION_NAME,
                    {"kind": "memories"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create memories collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self.resource_manager.index_collection(self.character_name, cid, index_type='semantic')
            self._memories_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_memories failed: {e}")

    def _remember(self, text: str, entity: str = "",
                  category: str = 'fact',
                  polarity: str = 'positive') -> Optional[str]:
        """Persist one memory string into the memories Collection. Returns
        the new note_id, or None on failure. `category` is one of
        _MEMORY_CATEGORIES; unknown values coerce to 'fact'. `polarity`
        is 'positive' (default) or 'negative' — negatives are explicit
        rejections / dispreferences.

        Held under _faiss_lock for the FAISS-touching span (create_note
        indexes the new note, add_to_collection updates the memories
        collection's vector store). Without the lock, a concurrent
        _recall in the main thread could race a background _remember
        and corrupt the FAISS index.
        """
        if not self._memories_collection_id:
            return None
        text = (text or "").strip()
        if not text:
            return None
        if category not in _MEMORY_CATEGORIES:
            category = 'fact'
        if polarity not in ('positive', 'negative'):
            polarity = 'positive'
        try:
            with self._faiss_lock:
                success, note_id, err, _ = self.resource_manager.create_note(
                    self.character_name, text, "text", "chat-loop", entity or "",
                    "",
                    {
                        "kind": "memory",
                        "category": category,
                        "polarity": polarity,
                        "entity": entity,
                        "created_at": datetime.now(timezone.utc).isoformat(),
                    },
                )
                if not success or not note_id:
                    logger.warning(f"[{self.character_name}] memory create failed: {err}")
                    return None
                self.resource_manager.mark_persistent(note_id, self.character_name)
                ok, _, add_err = self.resource_manager.add_to_collection(
                    self._memories_collection_id, note_id, self.character_name)
                if not ok:
                    logger.warning(f"[{self.character_name}] memory add_to_collection failed: {add_err}")
                # Trace: append to memories.jsonl for provenance audit and
                # remember-subagent visibility. Trigger is hardcoded
                # 'reflection' since that's currently the sole writer; if
                # other writers appear (manual /recall, autonomous), pass
                # trigger as a parameter instead.
                self._write_memory_event({
                    'event': 'write',
                    'note_id': note_id,
                    'text': text,
                    'category': category,
                    'polarity': polarity,
                    'entity': entity,
                    'source_turn_seq': getattr(self, '_turn_seq', None),
                    'trigger': 'reflection',
                })
                return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _remember failed: {e}")
            return None

    def _recency_adjust(self, score: float, created_at_iso: Optional[str]) -> float:
        """Apply a saturating age penalty so recall ties break toward
        newer memories. Stable facts older than ~6 weeks lose at most
        ~20% of score; iteration-accumulation stops promoting outdated
        playbook revisions over fresh ones. Failure-tolerant: any
        parse/arithmetic error returns the raw score unchanged."""
        if not created_at_iso:
            return score
        try:
            import math
            t = datetime.fromisoformat(str(created_at_iso))
            if t.tzinfo is None:
                t = t.replace(tzinfo=timezone.utc)
            age_days = max(
                0.0,
                (datetime.now(timezone.utc) - t).total_seconds() / 86400.0)
            factor = 1.0 - self._RECALL_RECENCY_MAX_PENALTY * (
                1.0 - math.exp(-age_days / self._RECALL_RECENCY_TAU_DAYS))
            return score * factor
        except Exception:
            return score

    def _recall(self, query: str, k: int = 3, threshold: float = 0.5
                ) -> List[Tuple[str, str, str]]:
        """Semantic search over the memories Collection. Returns ranked
        (text, category, polarity) tuples, highest score first. Category
        is read from the source note's properties; pre-taxonomy memories
        without a category default to 'fact'. Polarity is 'positive' or
        'negative'; pre-J memories without a polarity default to
        'positive'. Empty list on miss / not yet indexed / any error.

        Re-ranks by recency-adjusted score: fetch up to 2*k candidates
        from FAISS, apply a saturating age penalty (see _recency_adjust),
        then return the top-k. Raw similarity still dominates — recency
        only flips ties within the same topical band."""
        if not self._memories_collection_id or not query:
            return []
        try:
            fetch = max(k * 2, 6)
            with self._faiss_lock:
                ok, results, err = self.resource_manager.search_collection(
                    self.character_name, self._memories_collection_id, query,
                    mode='semantic', limit=fetch, threshold=threshold)
            if not ok or not results:
                return []
            scored: List[Tuple[float, str, str, str]] = []
            for r in results:
                if not isinstance(r, dict):
                    continue
                doc = r.get('document')
                if not isinstance(doc, str) or not doc.strip():
                    continue
                # Map chunk back to source note for category + polarity +
                # created_at. Chunks store source_note_id in metadata (per
                # _index_note_chunks). Missing note → fall back to 'fact'
                # category, 'positive' polarity, no recency adjustment.
                # No lock needed for this lookup — get_resource is a dict
                # read, not a FAISS operation.
                cat = 'fact'
                pol = 'positive'
                created_at: Optional[str] = None
                meta = r.get('metadata') or {}
                note_id = meta.get('source_note_id')
                if note_id:
                    note = self.resource_manager.get_resource(note_id)
                    if note:
                        props = note.get('properties') or {}
                        raw = props.get('category')
                        if isinstance(raw, str) and raw in _MEMORY_CATEGORIES:
                            cat = raw
                        raw_pol = props.get('polarity')
                        if isinstance(raw_pol, str) and raw_pol in ('positive', 'negative'):
                            pol = raw_pol
                        ca = props.get('created_at')
                        if isinstance(ca, str):
                            created_at = ca
                base_score = float(r.get('score') or 0.0)
                adj = self._recency_adjust(base_score, created_at)
                scored.append((adj, doc.strip(), cat, pol))
            scored.sort(key=lambda x: x[0], reverse=True)
            return [(text, cat, pol) for _adj, text, cat, pol in scored[:k]]
        except Exception as e:
            logger.warning(f"[{self.character_name}] _recall failed: {e}")
            return []

    def _memories_log_path(self) -> 'Path':
        """Path to <memory>/memories.jsonl — one JSON record per memory
        write. Append-only, separate from the live memories collection so
        provenance audit + temporal queries don't require iterating the
        Notes store. The `remember` subagent can grep this file for
        questions like 'when did I learn X?' or 'what memories were
        written from yesterday's turns?'."""
        return self._memory_dir() / 'memories.jsonl'

    def _write_memory_event(self, event: Dict[str, Any]) -> None:
        """Append one event record to memories.jsonl. Stamps `ts` and
        `character` if not already set. Best-effort — failures log a
        warning but don't disrupt the write that triggered the event."""
        path = self._memories_log_path()
        record = dict(event)
        record.setdefault('ts', datetime.now(timezone.utc).isoformat())
        record.setdefault('character', self.character_name)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        except Exception as e:
            logger.warning(f"[{self.character_name}] memories.jsonl write failed: {e}")

    def _memory_dir(self) -> 'Path':
        """Per-world per-agent memory directory — substrate the active-recall
        subagent will read. Layout: scenarios/<world>/<agent>/memory/.
        Derived from resource_manager.base_dir (which is
        scenarios/<world>/resources/<agent>/ once the resource manager
        applies its per-agent scoping) by going up two levels and over.
        Created on first write by callers."""
        from pathlib import Path
        return (
            self.resource_manager.base_dir.parent.parent
            / self.character_name / 'memory'
        )
