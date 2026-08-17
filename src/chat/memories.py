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

    # ------------------------------------------------------------------
    # Supersession at write time.
    #
    # Reflection is told not to re-emit a memory we already hold, but the
    # "## Existing memories" list it checks against is that turn's recall
    # top-k — so it is structurally blind to a near-duplicate that did not
    # happen to be recalled. Measured 2026-08-17 over 223 memories: 13% of
    # the store is redundant text, and 39% of recalling turns spend one of
    # their three slots on a near-duplicate ("User's name is Bruce" x4).
    #
    # The gate is at write because there is no removal path afterwards.
    # delete_resource only marks the note-level indexer; search_collection
    # reads the per-collection chunk store, which nothing removes from and
    # which add_to_collection only appends to. A deleted memory is still
    # recallable. Hence the soft-delete below: _recall already dereferences
    # the source note for category/created_at, so a `superseded_by`
    # property there is honoured immediately with no index surgery.
    # Reclaiming the chunks needs an offline rebuild.
    _MEMORY_SIMILAR_THRESHOLD = 0.80    # matches _CONCERN_RECURRENCE_THRESHOLD
    # ------------------------------------------------------------------

    def _find_similar_memory(self, text: str,
                             entity: str = "") -> Optional[Tuple[str, str]]:
        """Top existing memory similar enough to `text` to be the same
        fact, as (note_id, its current text). None on miss. Skips notes
        already superseded — a chain should point at the live one.

        Same-entity only, and that is load-bearing rather than tidiness.
        "The user's name is Bruce" (learned with Sentinel) and "The user's
        name is Jack" (learned with Jack) are one edit apart in text and
        score as the same fact, but they are two facts about two people —
        and the judge, asked which supersedes which, will pick the newer.
        A cleanup pass did exactly that on 2026-08-17 and left the agent
        believing the user had been renamed. Sameness of subject is
        structural here; it does not need a probe and must not depend on
        one."""
        if not self._memories_collection_id or not text:
            return None
        try:
            with self._faiss_lock:
                ok, results, _ = self.resource_manager.search_collection(
                    self.character_name, self._memories_collection_id, text,
                    mode='semantic', limit=1,
                    threshold=self._MEMORY_SIMILAR_THRESHOLD)
            if not ok or not results or not isinstance(results[0], dict):
                return None
            note_id = (results[0].get('metadata') or {}).get('source_note_id')
            if not note_id:
                return None
            note = self.resource_manager.get_resource(note_id)
            if not note:
                return None
            props = note.get('properties') or {}
            if props.get('superseded_by') or props.get('retired'):
                return None
            if (props.get('entity') or '') != (entity or ''):
                return None
            existing = str(props.get('content', '') or '').strip()
            return (note_id, existing) if existing else None
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] similar-memory lookup failed: {e}")
            return None

    def _memory_relation(self, new_text: str, existing_text: str) -> str:
        """How a new memory relates to the near-identical one already held:
        'restatement', 'revision', or 'distinct'.

        Similarity cannot decide this — "User is 79 years old" and "User is
        80 years old" score nearly identically to a pair of verbatim
        duplicates, and the right handling is opposite in each case. So it
        is a meaning question, asked as one. Fails open to 'distinct',
        which is today's behaviour: write it and keep both.
        """
        sys_msg = ("You compare two statements a system has recorded about "
                   "the same subject. Answer with exactly one word.")
        user_msg = (
            "HELD is a fact already stored. NEW is about to be stored.\n\n"
            f"HELD:\n{existing_text}\n\nNEW:\n{new_text}\n\n"
            "Answer with one word:\n"
            "restatement — NEW says the same thing as HELD and adds "
            "nothing; storing it would only duplicate.\n"
            "revision — NEW is the same fact with a changed or corrected "
            "value, so HELD is now out of date (an age that went up, a "
            "setting that changed, a plan replaced).\n"
            "distinct — NEW carries something HELD does not; both are "
            "worth keeping.\n\n"
            "Two statements about DIFFERENT subjects are always distinct, "
            "however alike they read. A fact about one person is never a "
            "revision of the same fact about another person.")
        try:
            raw = self.backend.chat(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': user_msg}],
                max_tokens=8, temperature=0.0, cot_profile='none')
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] memory-relation probe failed: {e}; "
                f"keeping both")
            return 'distinct'
        verdict = (raw or '').strip().strip('.`"\'').lower()
        for known in ('restatement', 'revision', 'distinct'):
            if verdict.startswith(known):
                return known
        logger.warning(
            f"[{self.character_name}] memory-relation probe returned "
            f"{verdict[:40]!r}, no verdict; keeping both")
        return 'distinct'

    def _mark_superseded(self, note_id: str, by_note_id: str) -> None:
        """Soft-delete: point a stale memory at the one that replaced it.
        _recall skips these, so the effect is immediate; the chunks stay in
        the collection store until an offline rebuild reclaims them."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return
        props = note.setdefault('properties', {})
        props['superseded_by'] = by_note_id
        props['superseded_at'] = datetime.now(timezone.utc).isoformat()

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
        # Supersession gate. Only consulted when something close enough to
        # be the same fact already exists, so the common path (a genuinely
        # new memory) costs one vector search and no LLM call.
        prior = self._find_similar_memory(text, entity)
        prior_id: Optional[str] = None
        if prior:
            prior_id, prior_text = prior
            relation = self._memory_relation(text, prior_text)
            if relation == 'restatement':
                logger.info(
                    f"[{self.character_name}] memory restates {prior_id}, "
                    f"not stored: {text[:80]!r}")
                self._write_memory_event({
                    'event': 'restatement_skipped',
                    'note_id': prior_id,
                    'text': text,
                    'category': category,
                    'polarity': polarity,
                    'entity': entity,
                    'source_turn_seq': getattr(self, '_turn_seq', None),
                    'trigger': 'reflection',
                })
                return prior_id
            if relation != 'revision':
                prior_id = None
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
                        # Provenance: same turn pointer memories.jsonl
                        # records, kept on the note itself so audit tools
                        # can walk memory → reasoning_trace turn without
                        # the sidecar. (created_at is auto-stamped by
                        # create_note; passing it here was a silent drop.)
                        "source_turn_seq": getattr(self, '_turn_seq', None),
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
                # Retire the fact this one revises, now that its
                # replacement exists and has an id to point at.
                if prior_id:
                    self._mark_superseded(prior_id, note_id)
                    logger.info(
                        f"[{self.character_name}] memory {note_id} "
                        f"supersedes {prior_id}")
                event = {
                    'event': 'write',
                    'note_id': note_id,
                    'text': text,
                    'category': category,
                    'polarity': polarity,
                    'entity': entity,
                    'source_turn_seq': getattr(self, '_turn_seq', None),
                    'trigger': 'reflection',
                }
                if prior_id:
                    event['supersedes'] = prior_id
                self._write_memory_event(event)
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
                ) -> List[Tuple[str, str, str, Optional[str], Optional[str]]]:
        """Semantic search over the memories Collection. Returns ranked
        (text, category, polarity, note_id, created_at) tuples, highest
        score first. Category is read from the source note's properties;
        pre-taxonomy memories without a category default to 'fact'.
        Polarity is 'positive' or 'negative'; pre-J memories without a
        polarity default to 'positive'. note_id/created_at are None when
        the chunk's source note is missing — they carry provenance
        (claim-level citation, recency display) and must survive to the
        prompt render and the reasoning trace, not die in this tuple.
        Empty list on miss / not yet indexed / any error.

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
            scored: List[Tuple[float, str, str, str, Optional[str], Optional[str]]] = []
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
                        # Withdrawn: either replaced by a later revision of
                        # the same fact (superseded_by), or retired outright
                        # as false with nothing to replace it (retired). The
                        # chunk is still in the collection store — nothing
                        # removes from it, see the supersession note above —
                        # so this dereference is the only place a retirement
                        # can take effect.
                        if props.get('superseded_by') or props.get('retired'):
                            continue
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
                scored.append((adj, doc.strip(), cat, pol, note_id or None,
                               created_at))
            scored.sort(key=lambda x: x[0], reverse=True)
            return [(text, cat, pol, nid, ca)
                    for _adj, text, cat, pol, nid, ca in scored[:k]]
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
        """Append one event record to memories.jsonl (stamps ts/character;
        best-effort — see utils.file_utils.append_jsonl)."""
        from utils.file_utils import append_jsonl
        append_jsonl(self._memories_log_path(), event,
                     character=self.character_name)

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
