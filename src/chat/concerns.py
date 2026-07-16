"""Concerns — agent_concerns (pressure-driven, fire autonomously) and
user_concerns (recall-driven, shape responses): creation, dynamics,
fire-time triage, WIP continuity, serialization. ConcernsMixin for
ChatLoop, moved verbatim from chat_loop.py in the 2026-06 mixin
refactor."""

from __future__ import annotations

import json
import logging
import os
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from utils.json_utils import repair_json_string  # noqa: E402
from utils.file_utils import atomic_write_json  # noqa: E402

logger = logging.getLogger('chat_loop')

# ===== Concerns: two collections, asymmetric dynamics =====
#
# Concerns split into two collections that share infrastructure but
# carry different semantics and different update rhythms.
#
#   user_concerns: "what the user cares about." Models the user's current
#       preoccupations. Strength decays each user turn (recall-driven —
#       if it doesn't come up, we stop tracking it). Bumped when user
#       input semantic-matches the concern. Pruned below threshold.
#       Surfaces in the prompt adjacent to companion. Does NOT fire
#       autonomously — its job is to shape Jill's responses to requests
#       by being recalled when relevant.
#
#   agent_concerns: "what I (Jill) want to act on." Pressure-driven.
#       Activation grows each tick at a rate derived from rhythm_hours;
#       decremented on service (autonomous fire). Fires when activation
#       >= threshold AND the concern carries an instruction. Seeds live
#       here as architectural baseline (seed=True) and source derived
#       agent_concerns via reflection.
#
# The asymmetry — strength decays, activation grows — is the real point:
# user models are recall-driven (silence implies disinterest); agent action
# is pressure-driven (silence implies untended work).
_AGENT_CONCERNS_COLLECTION_NAME = "agent_concerns"

_USER_CONCERNS_COLLECTION_NAME  = "user_concerns"

_CONCERN_STATUSES = ('active', 'satisfied', 'abandoned')

# ----- user_concerns dynamics -----
# strength ∈ [0, 1]. Decay applied at user-turn entry. Bump applied
# when input similarity >= bump threshold, capped to the top few matches
# per turn so broad topical conversation can't sustain an unbounded
# population (observed 2026-06: 34 concerns, bump uncapped at +0.30,
# decay never won). Pruned below prune threshold. Concerns whose last
# bump is older than the stale window are swept active → satisfied
# (wall-clock obsoleting; reflection recurrence can reopen them).
_USER_CONCERN_DECAY_PER_TURN  = 0.05   # strength lost per user turn

_USER_CONCERN_BUMP_THRESHOLD  = 0.5    # similarity ≥ this counts as a hit

_USER_CONCERN_BUMP_AMOUNT     = 0.15   # gained per hit (capped at 1.0)

_USER_CONCERN_BUMP_MAX_PER_TURN = 3    # at most this many bumped per turn

_USER_CONCERN_PRUNE_THRESHOLD = 0.10   # delete below this

_USER_CONCERN_PROMPT_BUDGET   = 5      # top-K by strength surfaced

_USER_CONCERN_STALE_DAYS      = 14     # unbumped this long → satisfied

# Heat coupling user model → agenda: a user_concern bumped ACROSS this
# strength (crossing, not dwelling — one bump per turn at most) applies
# an evidence bump to any active agent_concern carrying the
# user_model_reviewer property (the "review what the user has been
# tracking" seed). Hot-but-transient concerns can spike to 1.0 and
# decay to prune well inside the reviewer's rhythm; the crossing pulls
# the reviewer's fire forward so it looks while the heat is real.
# Whether anything warrants action stays with the reviewer's
# instruction at fire time.
_USER_CONCERN_HIGH_STRENGTH   = 0.70   # crossing this bumps the reviewer

# ----- agent_concerns dynamics -----
# activation ∈ [0, 1]. Grows each tick proportional to elapsed wall-
# clock time. Decremented on service per exit_reason. Fires when
# activation ≥ fire threshold AND instruction is non-null AND status
# is active. Floored at 0; capped at 1.
_AGENT_CONCERN_FIRE_THRESHOLD   = 0.70 # activation ≥ this allows fire

_AGENT_CONCERN_SERVICE_FULL     = 0.60 # decrement on respond exit

_AGENT_CONCERN_SERVICE_PARTIAL  = 0.25 # decrement on max_iters exit

_AGENT_CONCERN_PROMPT_BUDGET    = 5    # top-M by activation surfaced

# Evidence bumps: incoming (non-autonomous) input that semantically
# matches an active agent_concern raises its activation — evidence
# accumulating around a domain pulls it toward firing ahead of its
# wall-clock rhythm. A bump also clears any cached triage 'defer'
# verdict (see _triage_agent_concern): new evidence reopens the
# question of whether action is warranted.
_AGENT_CONCERN_BUMP_THRESHOLD = 0.50   # similarity ≥ this counts as a hit

_AGENT_CONCERN_BUMP_AMOUNT    = 0.15   # gained per hit (capped at 1.0)

# rhythm_hours: declared target fire interval. Used at concern creation
# to derive activation growth-per-elapsed-hour:
#   growth_per_hour = (FIRE_THRESHOLD - POST_SERVICE_FLOOR) / rhythm_hours
# where POST_SERVICE_FLOOR ≈ FIRE_THRESHOLD - SERVICE_FULL.
# A concern with rhythm_hours=24 fires roughly daily after full service.
#
# Allowlist matches the legacy cadence values. Default is weekly (168h)
# — chosen to err toward not firing too much when reflection lacks
# rhythm signal (typical for seed-derived self-orientation concerns).
# rhythm_source provenance ('external'|'urgency'|'default') is recorded
# on the note so we can audit how often the default fires versus
# reflection-extracted rhythms.
_AGENT_CONCERN_RHYTHM_HOURS_ALLOWED = (1, 2, 4, 8, 12, 24, 168)

_AGENT_CONCERN_DEFAULT_RHYTHM_HOURS = 168    # weekly default

# Successor-concern depth cap. When a fired concern's ReAct loop hits
# REACT_MAX_ITERS, the fallback path can spawn a successor concern with
# the narrowed remainder of the work. To prevent slow-motion infinite
# loops, depth is capped — a chain of original → successor → successor
# is the most we'll ever produce automatically. Beyond that, the fallback
# response just stands and the user can re-engage manually.
_CONCERN_SUCCESSOR_MAX_DEPTH = 2

# Per-concern work-in-progress: a running summary of autonomous-fire
# findings kept in the `wip` property of the root concern of a chain.
# Injected into the fire frame, the triage prompt, and the successor
# synthesizer so consecutive fires don't start cold. Rewritten (not
# appended) after each completed fire; hard length cap as a backstop
# against summarizer drift.
_CONCERN_WIP_MAX_CHARS = 1500

# Similarity threshold for recurrence detection on creation. A candidate
# concern whose top match in its collection exceeds this value is treated
# as the same concern: we refresh / promote the existing note rather
# than create a near-duplicate. Tuned high vs the 0.5 used for surfacing
# recall because a false merge silently loses specificity while a missed
# merge just creates a sibling that can be merged manually.
_CONCERN_RECURRENCE_THRESHOLD = 0.8

# ----- fire-outcome capture (phase 1: capture only) -----
# Each autonomous fire registers a pending record awaiting outcome
# judgment (docs/fire-outcome-capture.md). Judgment rides the existing
# post-turn reflection call as stage 6 — zero added LLM calls. Pending
# records age per user turn; the reaction window closes at the turn cap
# or the wall-clock cap, whichever hits first, resolving to 'unobserved'
# (distinct from 'unobservable' — a silent fire that never had anything
# user-visible to react to).
_FIRE_OUTCOME_EXPIRY_TURNS = 3         # user turns before pending → unobserved

_FIRE_OUTCOME_EXPIRY_DAYS = 7          # wall-clock cap on the reaction window

_FIRE_OUTCOME_MAX_PER_REFLECTION = 3   # outcomes judged per reflection call

_FIRE_OUTCOME_DIGEST_CHARS = 280       # reply digest cap in the pending record

_FIRE_OUTCOME_EVIDENCE_CHARS = 200     # evidence cap in the outcome record

# Fire digest: pending fires surfaced (once each) in the next user turn's
# prompt so the user gets a reaction opportunity inside the judgment
# window. Cap mirrors _FIRE_OUTCOME_MAX_PER_REFLECTION's conservatism.
_FIRE_DIGEST_MAX_ITEMS = 3

# Outcomes the reflection LLM may assign. 'unobserved'/'unobservable'
# are runtime-resolved, never LLM-judged.
_FIRE_OUTCOME_JUDGED = ('helped', 'neutral', 'hindered', 'ignored')

# --- Legacy aliases (in-flight rename; will be removed once dynamics rewrite lands)
_CONCERN_CATEGORIES = ('one_shot', 'durable', 'derived')

_CONCERN_CADENCE_HOURS_ALLOWED = _AGENT_CONCERN_RHYTHM_HOURS_ALLOWED

_CONCERN_DEFAULT_CADENCE_HOURS = {'one_shot': 1, 'derived': 24, 'durable': 24}

_CONCERN_DEFAULT_LIFETIME_DAYS = {'one_shot': 0.5, 'derived': 2.0, 'durable': 120.0}

_CONCERN_SATISFIED_THRESHOLD = 0.1

_CONCERN_LIFETIME_MIN_DAYS, _CONCERN_LIFETIME_MAX_DAYS = 0.1, 3650.0

_CONCERN_ALWAYS_ON_BUDGET = _AGENT_CONCERN_PROMPT_BUDGET

def _agent_concern_growth_for_elapsed(rhythm_hours: float, elapsed_hours: float) -> float:
    """Activation growth proportional to elapsed wall-clock hours.

    A concern with rhythm_hours=24 (daily target) grows from POST_SERVICE_FLOOR
    to FIRE_THRESHOLD over 24 elapsed hours. Independent of tick frequency —
    uses real elapsed time, so changing the tick schedule doesn't shift the
    firing rhythm.
    """
    if rhythm_hours <= 0 or elapsed_hours <= 0:
        return 0.0
    floor = max(0.0, _AGENT_CONCERN_FIRE_THRESHOLD - _AGENT_CONCERN_SERVICE_FULL)
    span = _AGENT_CONCERN_FIRE_THRESHOLD - floor
    return span * (elapsed_hours / float(rhythm_hours))

def _triage_defer_cooldown_hours(rhythm_hours: float) -> float:
    """How long a cached triage 'defer' verdict suppresses re-triage.
    Scaled to the concern's rhythm so a weekly concern deferred once
    isn't re-asked hourly, clamped to [1h, 24h]. A semantic evidence
    bump clears the cached verdict early (see
    _bump_agent_concerns_on_input)."""
    return min(24.0, max(1.0, float(rhythm_hours) / 8.0))

def _snap_rhythm_hours(value) -> int:
    """Snap a rhythm_hours value to the nearest allowed bucket. Returns
    the default when value is None / unparseable / out of range."""
    try:
        v = float(value) if value is not None else None
    except (TypeError, ValueError):
        v = None
    if v is None or v <= 0:
        return _AGENT_CONCERN_DEFAULT_RHYTHM_HOURS
    return min(_AGENT_CONCERN_RHYTHM_HOURS_ALLOWED, key=lambda b: abs(b - v))


class ConcernsMixin:
    """Mixin for ChatLoop — moved verbatim from chat_loop.py."""

    # ------------------------------------------------------------------
    # Concerns — actionable directives Jill should keep ready to advance.
    # Distinct from memories (stable specifics): a concern is something
    # she can reasonably expect to act on as an instruction; a preference
    # ("be brief") is a modifier that stays in memories.
    #
    # Lifecycle is lazy: weight = exp(-(now - last_engaged_at) / tau)
    # is computed at read time. Recall hits refresh last_engaged_at.
    # When weight drops below _CONCERN_SATISFIED_THRESHOLD the concern
    # transitions active → satisfied at the next read, except for seed
    # concerns (architectural baseline from YAML, immune to decay).
    # ------------------------------------------------------------------

    def _init_agent_concerns(self) -> None:
        """Get-or-create the agent_concerns Collection, mark it persistent,
        ensure semantic index exists. Idempotent across restarts."""
        try:
            cid = self.resource_manager.named_collections.get(_AGENT_CONCERNS_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _AGENT_CONCERNS_COLLECTION_NAME,
                    {"kind": "agent_concerns"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create agent_concerns collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self.resource_manager.index_collection(self.character_name, cid, index_type='semantic')
            self._agent_concerns_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_agent_concerns failed: {e}")

    def _init_user_concerns(self) -> None:
        """Get-or-create the user_concerns Collection, mark it persistent,
        ensure semantic index exists. Distinct from agent_concerns:
        decays per turn, bumped by user-input similarity, never fires
        autonomously. Idempotent across restarts."""
        try:
            cid = self.resource_manager.named_collections.get(_USER_CONCERNS_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _USER_CONCERNS_COLLECTION_NAME,
                    {"kind": "user_concerns"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create user_concerns collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self.resource_manager.index_collection(self.character_name, cid, index_type='semantic')
            self._user_concerns_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_user_concerns failed: {e}")

    def _autonomy_log_path(self) -> 'Path':
        """Path to <memory>/autonomy.jsonl — one JSON record per
        autonomous-fire event (fire/deferred). Append-only, separate
        from reasoning_trace.jsonl so 'what has Jill been doing on her
        own?' is grep-able as a clean stream."""
        return self._memory_dir() / 'autonomy.jsonl'

    def _write_autonomy_event(self, event: Dict[str, Any]) -> None:
        """Append one event record to autonomy.jsonl. Stamps `ts` and
        `character` if not already set. Best-effort — failures log a
        warning but don't disrupt the autonomous turn."""
        path = self._autonomy_log_path()
        record = dict(event)
        record.setdefault('ts', datetime.now(timezone.utc).isoformat())
        record.setdefault('character', self.character_name)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        except Exception as e:
            logger.warning(f"[{self.character_name}] autonomy.jsonl write failed: {e}")

    @staticmethod
    def _seed_concern_name(idx: int) -> str:
        return f"chat:agent_concern:seed:{idx}"

    def _seed_concerns_from_config(self, character_config: dict) -> None:
        """Instantiate seed concerns listed under YAML key `concerns:`.
        Seeds become agent_concerns with seed=True. They source derived
        agent_concerns via reflection and also carry their own instruction
        (if any) for direct firing. Idempotent — each seed gets a stable
        named-note slot.

        YAML may specify `rhythm_hours` (preferred) or legacy
        `cadence_hours` per seed; default is weekly (168h). `instruction`
        is optional — without it the concern accumulates activation but
        never fires (it's a source for derived concerns)."""
        seeds = character_config.get('concerns') or []
        if not isinstance(seeds, list) or not self._agent_concerns_collection_id:
            return
        for idx, seed in enumerate(seeds):
            if not isinstance(seed, dict):
                continue
            text = str(seed.get('text', '') or '').strip()
            if not text:
                continue
            entity = str(seed.get('entity', 'User') or 'User')
            name = self._seed_concern_name(idx)
            reviewer = bool(seed.get('user_model_reviewer'))
            self_ext = bool(seed.get('self_extension'))
            wip_rev = bool(seed.get('wip_reviewer'))
            domain = str(seed.get('domain') or '').strip()
            if name in self.resource_manager.named_notes:
                # Already seeded; preserve any edits — but sync the
                # designation flags so config can flag an existing seed
                # note (heat coupling / reflection target these properties).
                note = self.resource_manager.get_resource(
                    self.resource_manager.named_notes[name])
                if note:
                    props = note.setdefault('properties', {})
                    if reviewer and not props.get('user_model_reviewer'):
                        props['user_model_reviewer'] = True
                    if self_ext and not props.get('self_extension'):
                        props['self_extension'] = True
                    if wip_rev and not props.get('wip_reviewer'):
                        props['wip_reviewer'] = True
                    if domain and props.get('domain') != domain:
                        props['domain'] = domain
                    # Sync instruction from YAML: config is the source of
                    # truth for a seed's PROCEDURE; runtime state
                    # (activation, WIP, status) stays untouched. Without
                    # this, YAML instruction edits never reach an
                    # already-seeded concern (observed live 2026-07-12:
                    # the self-extension concern fired its original
                    # instruction, missing the inspect-first clause added
                    # to the YAML 2026-06-19). YAML-less instructions are
                    # left alone — sync never deletes.
                    yaml_instr = str(seed.get('instruction') or '').strip()
                    cur_instr = str(props.get('instruction') or '').strip()
                    if yaml_instr and yaml_instr != cur_instr:
                        props['instruction'] = yaml_instr
                        logger.info(
                            f"[{self.character_name}] seed {name}: "
                            f"instruction synced from scenario YAML")
                continue
            rhythm_h = seed.get('rhythm_hours')
            if rhythm_h is None:
                rhythm_h = seed.get('cadence_hours')   # legacy field
            if rhythm_h is None and seed.get('cadence_days') is not None:
                try:
                    rhythm_h = float(seed.get('cadence_days')) * 24.0
                except (TypeError, ValueError):
                    rhythm_h = None
            extra: Dict[str, Any] = {}
            if reviewer:
                extra['user_model_reviewer'] = True
            if self_ext:
                extra['self_extension'] = True
            if wip_rev:
                extra['wip_reviewer'] = True
            if domain:
                extra['domain'] = domain
            self._add_agent_concern(
                text, entity=entity, provenance='asserted', seed=True,
                name=name, rhythm_hours=rhythm_h, rhythm_source='external',
                instruction=seed.get('instruction'),
                extra_properties=extra or None)

    # ------------------------------------------------------------------
    # Concern creation: shared note-create path + per-collection helpers.
    # ------------------------------------------------------------------

    def _find_similar_concern(self, text: str, collection_id: Optional[str],
                              threshold: float = _CONCERN_RECURRENCE_THRESHOLD
                              ) -> Optional[str]:
        """Find the top semantically-similar existing concern in
        `collection_id` whose similarity meets `threshold`. Returns the
        source note_id, or None. Excludes abandoned concerns. Caller
        decides what to do with active vs satisfied matches (per-class
        promote/bump logic differs)."""
        if not collection_id or not text:
            return None
        try:
            with self._faiss_lock:
                ok, results, _ = self.resource_manager.search_collection(
                    self.character_name, collection_id, text,
                    mode='semantic', limit=1, threshold=threshold)
            if not ok or not results:
                return None
            r = results[0] if isinstance(results[0], dict) else None
            if not r:
                return None
            meta = r.get('metadata') or {}
            note_id = meta.get('source_note_id')
            if not note_id:
                return None
            note = self.resource_manager.get_resource(note_id)
            if not note:
                return None
            status = (note.get('properties') or {}).get('status', 'active')
            if status == 'abandoned':
                return None
            return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _find_similar_concern failed: {e}")
            return None

    def _promote_existing_agent_concern(self, note_id: str) -> str:
        """Recurrence: revive a near-twin agent_concern rather than
        creating a duplicate. status: satisfied → active. Activation
        left as-is so existing pressure is preserved; the user's
        re-engagement is for the topic, not against the queue."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return note_id
        props = note.setdefault('properties', {})
        if props.get('status') == 'satisfied':
            props['status'] = 'active'
        return note_id

    def _resolve_concern_by_text(
            self, text: str,
            shown: List[Tuple[str, str, float, Dict[str, Any]]],
            collection_id: Optional[str]
    ) -> Optional[str]:
        """Resolve a reflection-emitted concern text to a note id in
        `collection_id`. Exact (case-insensitive) match against the list
        the LLM was shown first; falls back to the recurrence-threshold
        semantic search for paraphrases. Returns None when nothing
        matches — caller skips rather than guessing."""
        needle = (text or '').strip().lower()
        if not needle:
            return None
        for nid, shown_text, _rank, _props in shown:
            if shown_text.strip().lower() == needle:
                return nid
        return self._find_similar_concern(text, collection_id)

    def _resolve_user_concern_by_text(
            self, text: str,
            shown: List[Tuple[str, str, float, Dict[str, Any]]]
    ) -> Optional[str]:
        """Resolve a reflection-emitted closure text to a user_concern
        note id (see _resolve_concern_by_text)."""
        return self._resolve_concern_by_text(
            text, shown, self._user_concerns_collection_id)

    def _apply_agent_concern_closures(
            self, raw: Any,
            shown: List[Tuple[str, str, float, Dict[str, Any]]]
    ) -> List[str]:
        """Retire agent_concerns reflection says the user agreed to drop
        (the WIP reviewer's escalate-or-retire loop closes here). Each
        emitted text resolves against the list the LLM was shown; seeds
        are architectural baseline and never close; misses are skipped.
        Status → 'abandoned', not 'satisfied': a drop decision blocks
        recurrence revival (_find_similar_concern excludes abandoned).
        Returns the closed texts."""
        if not isinstance(raw, list):
            return []
        closed: List[str] = []
        for item in raw:
            if isinstance(item, str):
                text = item.strip()
            elif isinstance(item, dict):
                text = str(item.get('text', '') or '').strip()
            else:
                continue
            if not text:
                continue
            nid = self._resolve_concern_by_text(
                text, shown, self._agent_concerns_collection_id)
            if not nid:
                logger.info(
                    f"[{self.character_name}] agent-concern close skipped — "
                    f"no match for {text[:80]!r}")
                continue
            note = self.resource_manager.get_resource(nid) or {}
            props = note.get('properties') or {}
            if props.get('seed'):
                logger.info(
                    f"[{self.character_name}] agent-concern close refused — "
                    f"{nid} is a seed (architectural baseline)")
                continue
            ok, err = self._set_concern_status(nid, 'abandoned')
            if ok:
                closed.append(text)
                self._write_autonomy_event({
                    'event': 'concern_abandoned',
                    'concern_id': nid,
                    'concern_text': text,
                    'via': 'reflection',
                })
            else:
                logger.warning(
                    f"[{self.character_name}] agent-concern close failed "
                    f"for {nid}: {err}")
        return closed

    def _bump_existing_user_concern(self, note_id: str,
                                    context: Optional[str] = None) -> str:
        """Recurrence: bump strength on a near-twin user_concern rather
        than creating a duplicate. When the duplicate candidate carried a
        context, refresh the stored one (newest-wins) — recurrences are
        evidence, and the fresher reading is usually the better one."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return note_id
        props = note.setdefault('properties', {})
        s = float(props.get('strength', 0.0) or 0.0)
        props['strength'] = min(1.0, s + _USER_CONCERN_BUMP_AMOUNT)
        props['last_bumped_at'] = datetime.now(timezone.utc).isoformat()
        if props.get('status') != 'active':
            props['status'] = 'active'
        if context and context.strip():
            self._set_user_concern_context(note, context)
        return note_id

    def _set_user_concern_context(self, note: Dict[str, Any],
                                  context: str) -> None:
        """Write a user_concern's context + timestamp, and keep the note's
        browser-visible description in sync (replaces the create_note
        boilerplate with something meaningful)."""
        context = context.strip()
        props = note.setdefault('properties', {})
        props['context'] = context
        props['context_updated_at'] = datetime.now(timezone.utc).isoformat()
        note['description'] = context[:140]

    def _update_user_concern_context(self, note_id: str,
                                     context: str) -> bool:
        """Reflection judged that an exchange materially developed an
        existing user_concern: rewrite its context (current-state, not
        changelog). Returns True on success."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            logger.info(
                f"[{self.character_name}] context update skipped: "
                f"{note_id} no longer exists")
            return False
        if not (context or '').strip():
            return False
        self._set_user_concern_context(note, context)
        return True

    def _user_concerns_for_evaluator(self) -> List[Dict[str, Any]]:
        """Top active user_concerns shaped for character_evaluator's
        _format_user_concerns_block: concern_label / concern_description /
        status (browser vocabulary via _USER_CONCERN_STATUS_MAP)."""
        out: List[Dict[str, Any]] = []
        for _nid, text, _strength, props in self._top_active_user_concerns():
            status = str((props or {}).get('status', 'active') or 'active')
            out.append({
                'concern_label': text,
                'concern_description': str((props or {}).get('context', '') or ''),
                'status': self._USER_CONCERN_STATUS_MAP.get(status, status),
            })
        return out

    @staticmethod
    def _clamp_optional(value: Any, lo: float, hi: float) -> Optional[float]:
        """Clamp a numeric value into [lo, hi]; pass None through."""
        if value is None:
            return None
        try:
            v = float(value)
        except (TypeError, ValueError):
            return None
        return max(lo, min(hi, v))

    @classmethod
    def _resolve_rhythm_hours(cls, properties: Dict[str, Any]) -> int:
        """Read rhythm_hours from an agent_concern note. Falls back to
        legacy `cadence_hours` (current data) and `cadence_days` (older
        migration). Always returns a value from the allowed bucket;
        defaults to weekly when nothing usable is on the note."""
        raw = properties.get('rhythm_hours')
        if raw is None:
            raw = properties.get('cadence_hours')
        if raw is None and properties.get('cadence_days') is not None:
            try:
                raw = float(properties['cadence_days']) * 24.0
            except (TypeError, ValueError):
                raw = None
        return _snap_rhythm_hours(raw)

    def _create_concern_note(self, text: str, name: str, entity: str,
                             properties: Dict[str, Any],
                             collection_id: str) -> Optional[str]:
        """Shared note creation path. create_note + mark_persistent +
        add_to_collection, all under _faiss_lock."""
        try:
            with self._faiss_lock:
                success, note_id, err, _ = self.resource_manager.create_note(
                    self.character_name, text, "text", "chat-loop",
                    entity or "", name or "", properties)
                if not success or not note_id:
                    logger.warning(
                        f"[{self.character_name}] concern create failed: {err}")
                    return None
                self.resource_manager.mark_persistent(note_id, self.character_name)
                ok, _, add_err = self.resource_manager.add_to_collection(
                    collection_id, note_id, self.character_name)
                if not ok:
                    logger.warning(
                        f"[{self.character_name}] concern add_to_collection failed: {add_err}")
                return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _create_concern_note failed: {e}")
            return None

    def _add_agent_concern(self, text: str, entity: str = 'User',
                           provenance: str = 'asserted', seed: bool = False,
                           name: str = '',
                           rhythm_hours: Optional[int] = None,
                           rhythm_source: str = 'default',
                           instruction: Optional[str] = None,
                           skip_recurrence: bool = False,
                           extra_properties: Optional[Dict[str, Any]] = None
                           ) -> Optional[str]:
        """Create an agent_concern. Activation starts at 0; per-tick
        growth is proportional to elapsed wall-clock / rhythm_hours.
        Fires when activation ≥ _AGENT_CONCERN_FIRE_THRESHOLD AND
        instruction is non-null. Seeds carry seed=True; their activation
        still grows but they won't fire without an instruction.

        skip_recurrence bypasses similarity merge — used by successor
        concerns. extra_properties merges into the note's properties
        (e.g. successor_of, successor_depth)."""
        if not self._agent_concerns_collection_id:
            return None
        text = (text or "").strip()
        if not text:
            return None
        rhythm_hours = _snap_rhythm_hours(rhythm_hours)
        instruction = (str(instruction).strip() if instruction else '') or None
        if rhythm_source not in ('external', 'urgency', 'default'):
            rhythm_source = 'default'
        if provenance not in ('asserted', 'inferred'):
            provenance = 'asserted'
        if not seed and not skip_recurrence:
            existing = self._find_similar_concern(
                text, self._agent_concerns_collection_id)
            if existing:
                return self._promote_existing_agent_concern(existing)
        now_iso = datetime.now(timezone.utc).isoformat()
        properties: Dict[str, Any] = {
            "kind": "agent_concern",
            "status": "active",
            "entity": entity,
            "provenance": provenance,
            "seed": bool(seed),
            "instruction": instruction,
            "rhythm_hours": rhythm_hours,
            "rhythm_source": rhythm_source,
            "activation": 0.0,
            "last_activation_update_at": now_iso,
            "last_fired_at": None,
        }
        if extra_properties:
            properties.update(extra_properties)
        return self._create_concern_note(
            text, name, entity, properties,
            self._agent_concerns_collection_id)

    def _add_user_concern(self, text: str, entity: str = 'User',
                          name: str = '',
                          initial_strength: float = 1.0,
                          skip_recurrence: bool = False,
                          context: Optional[str] = None,
                          extra_properties: Optional[Dict[str, Any]] = None
                          ) -> Optional[str]:
        """Create a user_concern. Strength starts at initial_strength
        (1.0 default). Decays each user turn; bumped on similarity hit;
        pruned below threshold. Never fires. `text` is the stable short
        handle (dedup / ranking / closure matching key); `context` is the
        evolving meaning — evidence + what the user appears to want."""
        if not self._user_concerns_collection_id:
            return None
        text = (text or "").strip()
        if not text:
            return None
        if not skip_recurrence:
            existing = self._find_similar_concern(
                text, self._user_concerns_collection_id)
            if existing:
                return self._bump_existing_user_concern(existing, context=context)
        now_iso = datetime.now(timezone.utc).isoformat()
        s = max(0.0, min(1.0, float(initial_strength)))
        properties: Dict[str, Any] = {
            "kind": "user_concern",
            "status": "active",
            "entity": entity,
            "strength": s,
            "last_bumped_at": now_iso,
        }
        if context and context.strip():
            properties["context"] = context.strip()
            properties["context_updated_at"] = now_iso
        if extra_properties:
            properties.update(extra_properties)
        note_id = self._create_concern_note(
            text, name, entity, properties,
            self._user_concerns_collection_id)
        # Browser-visible description: the context beats create_note's
        # "Note artifact created by chat-loop" boilerplate.
        if note_id and properties.get("context"):
            note = self.resource_manager.get_resource(note_id)
            if note:
                note['description'] = properties["context"][:140]
        return note_id

    # ------------------------------------------------------------------
    # Per-tick / per-turn dynamics. Pure arithmetic — no LLM in these
    # paths. Called from _handle_tick (growth, fire-check) and
    # _process_user_turn (decay, bump). Cheap enough to run every
    # tick / every turn without throttling.
    # ------------------------------------------------------------------

    def _grow_agent_concerns_per_tick(self) -> None:
        """Apply elapsed-time growth to every active agent_concern.
        activation += growth_for_elapsed(rhythm_hours, Δhours). Caps
        at 1.0; updates last_activation_update_at."""
        if not self._agent_concerns_collection_id:
            return
        coll = self.resource_manager.get_resource(self._agent_concerns_collection_id)
        if not coll:
            return
        note_ids = (coll.get('properties') or {}).get('content', []) or []
        now = datetime.now(timezone.utc)
        for nid in note_ids:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.setdefault('properties', {})
            if props.get('status') != 'active':
                continue
            rhythm_h = self._resolve_rhythm_hours(props)
            last_str = (props.get('last_activation_update_at')
                        or props.get('created_at'))
            try:
                last = datetime.fromisoformat(str(last_str)) if last_str else now
                if last.tzinfo is None:
                    last = last.replace(tzinfo=timezone.utc)
            except (TypeError, ValueError):
                last = now
            elapsed_h = max(0.0, (now - last).total_seconds() / 3600.0)
            growth = _agent_concern_growth_for_elapsed(rhythm_h, elapsed_h)
            a = float(props.get('activation', 0.0) or 0.0)
            props['activation'] = min(1.0, a + growth)
            props['last_activation_update_at'] = now.isoformat()

    def _decay_user_concerns_per_turn(self) -> None:
        """Apply per-turn strength decay to active user_concerns. Hard-
        deletes any concern that falls below the prune threshold (a
        topic the user hasn't engaged with for ~18 turns at default
        rates). Also wall-clock obsoleting: concerns whose last bump
        (fallback: creation) is older than _USER_CONCERN_STALE_DAYS are
        swept active → satisfied instead of decayed — turn-based decay
        can't age concerns between sessions, this can. Swept concerns
        stay recallable and reopen via reflection recurrence if the
        theme genuinely returns."""
        if not self._user_concerns_collection_id:
            return
        coll = self.resource_manager.get_resource(self._user_concerns_collection_id)
        if not coll:
            return
        note_ids = list((coll.get('properties') or {}).get('content', []) or [])
        now = datetime.now(timezone.utc)
        pruned: List[str] = []
        swept: List[str] = []
        for nid in note_ids:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.setdefault('properties', {})
            if props.get('status') != 'active':
                continue
            anchor_str = props.get('last_bumped_at') or props.get('created_at')
            if anchor_str:
                try:
                    anchor = datetime.fromisoformat(str(anchor_str))
                    if anchor.tzinfo is None:
                        anchor = anchor.replace(tzinfo=timezone.utc)
                    if (now - anchor).days >= _USER_CONCERN_STALE_DAYS:
                        props['status'] = 'satisfied'
                        swept.append(nid)
                        continue
                except (TypeError, ValueError) as e:
                    logger.warning(
                        f"[{self.character_name}] unparseable bump/create "
                        f"timestamp on {nid} ({anchor_str!r}): {e}")
            s = float(props.get('strength', 1.0) or 0.0)
            s = max(0.0, s - _USER_CONCERN_DECAY_PER_TURN)
            props['strength'] = s
            if s < _USER_CONCERN_PRUNE_THRESHOLD:
                ok, err = self.resource_manager.delete_resource(nid)
                if ok:
                    pruned.append(nid)
                else:
                    logger.warning(
                        f"[{self.character_name}] user_concern prune failed for "
                        f"{nid}: {err}")
        if swept:
            logger.info(
                f"[{self.character_name}] swept {len(swept)} stale "
                f"user_concern(s) to satisfied (unbumped "
                f">{_USER_CONCERN_STALE_DAYS}d)")
        if pruned:
            logger.info(
                f"[{self.character_name}] pruned {len(pruned)} user_concern(s) "
                f"below strength threshold")

    def _bump_user_concerns_on_input(self, text: str) -> None:
        """Semantic-search user_concerns for input similarity; bump
        strength on the top few hits above threshold. Cheap (one FAISS
        query) and called once per user-turn entry. Bumps are capped at
        _USER_CONCERN_BUMP_MAX_PER_TURN (results arrive ranked, so the
        best matches win) — uncapped, one topical turn sustained every
        concern in its neighborhood and decay never pruned anything."""
        if not self._user_concerns_collection_id or not text:
            return
        try:
            with self._faiss_lock:
                ok, results, _ = self.resource_manager.search_collection(
                    self.character_name, self._user_concerns_collection_id,
                    text, mode='semantic',
                    limit=_USER_CONCERN_PROMPT_BUDGET * 2,
                    threshold=_USER_CONCERN_BUMP_THRESHOLD)
            if not ok or not results:
                return
            now_iso = datetime.now(timezone.utc).isoformat()
            bumped = 0
            crossed_high = False
            for r in results:
                if bumped >= _USER_CONCERN_BUMP_MAX_PER_TURN:
                    break
                if not isinstance(r, dict):
                    continue
                meta = r.get('metadata') or {}
                nid = meta.get('source_note_id')
                if not nid:
                    continue
                note = self.resource_manager.get_resource(nid)
                if not note:
                    continue
                props = note.setdefault('properties', {})
                if props.get('status') != 'active':
                    continue
                s = float(props.get('strength', 0.0) or 0.0)
                new_s = min(1.0, s + _USER_CONCERN_BUMP_AMOUNT)
                props['strength'] = new_s
                props['last_bumped_at'] = now_iso
                if s < _USER_CONCERN_HIGH_STRENGTH <= new_s:
                    crossed_high = True
                bumped += 1
            if crossed_high:
                self._bump_user_model_reviewer(now_iso)
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] _bump_user_concerns_on_input failed: {e}")

    def _bump_user_model_reviewer(self, now_iso: str) -> None:
        """A user_concern just crossed _USER_CONCERN_HIGH_STRENGTH:
        evidence-bump the agent_concern(s) carrying the
        user_model_reviewer property (designated on the YAML seed) so
        the periodic review of the user model fires ahead of its rhythm
        while the heat is current. The reviewer's instruction still
        judges at fire time whether anything warrants action."""
        if not self._agent_concerns_collection_id:
            return
        coll = self.resource_manager.get_resource(self._agent_concerns_collection_id)
        if not coll:
            return
        for nid in (coll.get('properties') or {}).get('content', []) or []:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.setdefault('properties', {})
            if props.get('status') != 'active' or not props.get('user_model_reviewer'):
                continue
            self._apply_agent_concern_evidence_bump(props, now_iso)
            logger.info(
                f"[{self.character_name}] user_concern crossed "
                f"{_USER_CONCERN_HIGH_STRENGTH} strength; bumped reviewer "
                f"{nid} to activation={props['activation']:.2f}")

    def _self_extension_concern_id(self) -> Optional[str]:
        """Note id of the active agent_concern flagged self_extension
        (the propose-new-tools seed), or None. Reflection records gaps
        onto it; _handle_tick tags its fires as capability_proposal."""
        for nid, note, _a in self._iter_active_agent_concerns():
            if (note.get('properties') or {}).get('self_extension'):
                return nid
        return None

    def _record_capability_gap(self, gap: str) -> None:
        """Reflection noticed {character} lacked a tool this turn. Append
        the gap to the self-extension concern's WIP (so it rides into the
        fire frame) and evidence-bump its activation (so recurring gaps
        fire ahead of the weekly rhythm). No-op if the seed is absent.
        Repetition in WIP is informative — it's the recurrence signal the
        fire instruction looks for — so gaps are not deduped at capture."""
        gap = (gap or '').strip()
        if not gap:
            return
        nid = self._self_extension_concern_id()
        if not nid:
            logger.debug(
                f"[{self.character_name}] capability gap noted but no "
                f"self_extension concern present; dropping: {gap[:80]!r}")
            return
        note = self.resource_manager.get_resource(nid)
        if not note:
            return
        props = note.setdefault('properties', {})
        now_iso = datetime.now(timezone.utc).isoformat()
        day = now_iso[:10]
        entry = f"GAP NOTED ({day}): {gap}"
        prev_wip = str(props.get('wip', '') or '').strip()
        new_wip = (prev_wip + "\n" + entry).strip() if prev_wip else entry
        # Cap to the same budget as post-fire WIP; keep the most-recent
        # tail when over budget (recent gaps matter most).
        if len(new_wip) > _CONCERN_WIP_MAX_CHARS:
            new_wip = new_wip[-_CONCERN_WIP_MAX_CHARS:]
        props['wip'] = new_wip
        props['wip_updated_at'] = now_iso
        self._apply_agent_concern_evidence_bump(props, now_iso)
        self._write_autonomy_event({
            'event': 'capability_gap',
            'concern_id': nid,
            'gap': gap,
            'activation': round(float(props.get('activation', 0.0) or 0.0), 3),
        })
        logger.info(
            f"[{self.character_name}] capability gap recorded → "
            f"self_extension concern {nid} activation="
            f"{props['activation']:.2f}: {gap[:80]!r}")

    @staticmethod
    def _apply_agent_concern_evidence_bump(props: Dict[str, Any],
                                           now_iso: str) -> None:
        """Apply one evidence bump to an active agent_concern's props:
        raise activation, stamp last_bumped_at, and clear any cached
        triage 'defer' verdict — new evidence reopens the fire/defer
        question."""
        a = float(props.get('activation', 0.0) or 0.0)
        props['activation'] = min(1.0, a + _AGENT_CONCERN_BUMP_AMOUNT)
        props['last_bumped_at'] = now_iso
        for stale in ('triage_verdict', 'triage_at', 'triage_reason'):
            props.pop(stale, None)

    def _bump_agent_concerns_on_input(self, text: str) -> None:
        """Semantic-search agent_concerns for input similarity; bump
        activation on each hit above threshold. Evidence-driven analog
        of the wall-clock growth in _grow_agent_concerns_per_tick: input
        touching a concern's domain pulls it toward firing ahead of its
        rhythm. Clears any cached triage 'defer' verdict on bumped
        concerns — new evidence reopens the fire/defer question. Called
        once per non-autonomous turn, next to the user_concern bump."""
        if not self._agent_concerns_collection_id or not text:
            return
        try:
            with self._faiss_lock:
                ok, results, _ = self.resource_manager.search_collection(
                    self.character_name, self._agent_concerns_collection_id,
                    text, mode='semantic',
                    limit=_AGENT_CONCERN_PROMPT_BUDGET * 2,
                    threshold=_AGENT_CONCERN_BUMP_THRESHOLD)
            if not ok or not results:
                return
            now_iso = datetime.now(timezone.utc).isoformat()
            for r in results:
                if not isinstance(r, dict):
                    continue
                meta = r.get('metadata') or {}
                nid = meta.get('source_note_id')
                if not nid:
                    continue
                note = self.resource_manager.get_resource(nid)
                if not note:
                    continue
                props = note.setdefault('properties', {})
                if props.get('status') != 'active':
                    continue
                self._apply_agent_concern_evidence_bump(props, now_iso)
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] _bump_agent_concerns_on_input failed: {e}")

    def _service_agent_concern(self, note_id: str, exit_reason: str) -> None:
        """Decrement activation on service. Called after autonomous fire
        completes. exit_reason determines decrement size:
          'respond'   → full service (ReAct ran to completion)
          'yield'     → full service (intentional boundary; the successor
                        concern carries the remainder — the parent should
                        not re-fire the same work on top of it)
          'max_iters' → partial (work continued via successor concern)
          others      → no decrement (fire didn't really happen)
        Activation floors at 0; last_fired_at recorded so we have a
        fire history independent of the activation curve."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return
        props = note.setdefault('properties', {})
        if exit_reason in ('respond', 'yield'):
            decrement = _AGENT_CONCERN_SERVICE_FULL
        elif exit_reason == 'max_iters':
            decrement = _AGENT_CONCERN_SERVICE_PARTIAL
        else:
            decrement = 0.0
        if decrement > 0:
            a = float(props.get('activation', 0.0) or 0.0)
            props['activation'] = max(0.0, a - decrement)
        props['last_fired_at'] = datetime.now(timezone.utc).isoformat()

    # ------------------------------------------------------------------
    # Fire-outcome capture (phase 1: capture only) — pending registry +
    # outcome records (docs/fire-outcome-capture.md). Signed per-ledger
    # outcomes for autonomous fires, judged from the cheapest reliable
    # evidence: the user's subsequent turns, via reflection stage 6.
    # Everything here is instrumentation — failure-tolerant, never
    # allowed to disrupt the turn loop.
    # ------------------------------------------------------------------

    def _pending_fire_outcomes_path(self) -> 'Path':
        """Path to <memory>/pending_fire_outcomes.json — the registry of
        autonomous fires awaiting outcome judgment. Full-rewrite file, so
        writes go through atomic_write_json (crash mid-write can't leave
        a truncated registry)."""
        return self._memory_dir() / 'pending_fire_outcomes.json'

    def _load_pending_fire_outcomes(self) -> List[Dict[str, Any]]:
        """Read the pending registry. Missing or corrupt file → [] —
        losing pending records costs one outcome datum, not a turn."""
        path = self._pending_fire_outcomes_path()
        try:
            if not path.exists():
                return []
            data = json.loads(path.read_text(encoding='utf-8'))
            if isinstance(data, list):
                return [r for r in data if isinstance(r, dict)]
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] pending_fire_outcomes read failed: {e}")
        return []

    def _save_pending_fire_outcomes(self, records: List[Dict[str, Any]]) -> None:
        """Atomically rewrite the pending registry."""
        try:
            path = self._pending_fire_outcomes_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            atomic_write_json(path, records)
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] pending_fire_outcomes write failed: {e}")

    def _register_fire_outcome(self, fire_id: str, concern_id: str,
                               exit_reason: str, reply: str,
                               intentionally_silent: bool) -> None:
        """Register a completed autonomous fire for outcome judgment.
        Called after reply publication. A silent fire never enters the
        registry: its downstream outcome is structurally unobservable
        (nothing user-visible to react to — a different fact than
        'observed and neutral'), so it resolves immediately with the
        process fields only."""
        try:
            note = self.resource_manager.get_resource(concern_id) or {}
            props = note.get('properties') or {}
            concern_text = str(
                props.get('content', '') or note.get('text', '') or '').strip()
            now_iso = datetime.now(timezone.utc).isoformat()
            if intentionally_silent:
                self._write_autonomy_event({
                    'event': 'fire_outcome',
                    'fire_id': fire_id,
                    'concern_id': concern_id,
                    'concern_text': concern_text,
                    'exit_reason': exit_reason,
                    'outcome': 'unobservable',
                    'valence': None,
                    'user_impact': None,
                    'evidence': None,
                    'latency_turns': 0,
                    'observed_at': now_iso,
                })
                return
            records = self._load_pending_fire_outcomes()
            records.append({
                'fire_id': fire_id,
                'concern_id': concern_id,
                'concern_text': concern_text,
                'fired_at': now_iso,
                'exit_reason': exit_reason,
                'reply_digest': (reply or '')[:_FIRE_OUTCOME_DIGEST_CHARS],
                'intentionally_silent': False,
                'user_turns_since': 0,
            })
            self._save_pending_fire_outcomes(records)
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] fire-outcome registration failed "
                f"for {fire_id}: {e}")

    def _age_pending_fire_outcomes(self) -> None:
        """Per-user-turn aging of the pending registry: each user turn
        widens the reaction window. Records past the turn cap or the
        wall-clock cap resolve to 'unobserved' (valence null — consumers
        weight these down, not zero-code them as neutral) and leave the
        registry. Called once per non-autonomous turn, next to the
        user_concern decay."""
        try:
            records = self._load_pending_fire_outcomes()
            if not records:
                return
            now = datetime.now(timezone.utc)
            keep: List[Dict[str, Any]] = []
            for rec in records:
                rec['user_turns_since'] = int(rec.get('user_turns_since', 0) or 0) + 1
                expired = rec['user_turns_since'] >= _FIRE_OUTCOME_EXPIRY_TURNS
                if not expired:
                    try:
                        fired_at = datetime.fromisoformat(str(rec.get('fired_at')))
                        if fired_at.tzinfo is None:
                            fired_at = fired_at.replace(tzinfo=timezone.utc)
                        expired = (now - fired_at) > timedelta(
                            days=_FIRE_OUTCOME_EXPIRY_DAYS)
                    except (TypeError, ValueError):
                        # Unparseable fired_at can never wall-clock expire;
                        # expire now rather than let it pend forever.
                        expired = True
                if expired:
                    self._write_autonomy_event({
                        'event': 'fire_outcome',
                        'fire_id': rec.get('fire_id'),
                        'concern_id': rec.get('concern_id'),
                        'concern_text': rec.get('concern_text'),
                        'exit_reason': rec.get('exit_reason'),
                        'outcome': 'unobserved',
                        'valence': None,
                        'user_impact': None,
                        'evidence': None,
                        'latency_turns': rec['user_turns_since'],
                        'observed_at': now.isoformat(),
                    })
                else:
                    keep.append(rec)
            self._save_pending_fire_outcomes(keep)
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] fire-outcome aging failed: {e}")

    def _take_unsurfaced_pending_fires(self) -> List[Dict[str, Any]]:
        """Select pending fires not yet surfaced in a user-turn prompt,
        mark them surfaced, and return them (registry order = oldest
        first, capped at _FIRE_DIGEST_MAX_ITEMS). One digest appearance
        per fire — enough to invite a reaction without nagging every
        turn. Called once per non-autonomous turn, after aging, so
        user_turns_since is current."""
        try:
            records = self._load_pending_fire_outcomes()
            if not records:
                return []
            picked: List[Dict[str, Any]] = []
            for rec in records:
                if len(picked) >= _FIRE_DIGEST_MAX_ITEMS:
                    break
                if not rec.get('surfaced'):
                    rec['surfaced'] = True
                    picked.append(rec)
            if picked:
                self._save_pending_fire_outcomes(records)
            return picked
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] pending-fire surfacing failed: {e}")
            return []

    def _apply_fire_outcome_judgments(self, raw: Any,
                                      pending: List[Dict[str, Any]]
                                      ) -> List[str]:
        """Validate reflection stage-6 judgments against the pending
        records the LLM was shown; write one fire_outcome event per
        accepted judgment and drop those records from the registry.
        Conservative, mirroring the one-patch ethos: unknown fire_ids,
        bad outcome values, and entries beyond the per-reflection cap
        are skipped — they stay pending and age out naturally. Returns
        the resolved fire_ids."""
        if not isinstance(raw, list) or not pending:
            return []

        def _clamp(v: Any) -> Optional[float]:
            try:
                return max(-1.0, min(1.0, float(v)))
            except (TypeError, ValueError):
                return None

        by_id = {str(r.get('fire_id')): r for r in pending if r.get('fire_id')}
        resolved: List[str] = []
        now_iso = datetime.now(timezone.utc).isoformat()
        for item in raw:
            if len(resolved) >= _FIRE_OUTCOME_MAX_PER_REFLECTION:
                break
            if not isinstance(item, dict):
                continue
            fid = str(item.get('fire_id', '') or '').strip()
            rec = by_id.get(fid)
            if rec is None or fid in resolved:
                continue
            outcome = str(item.get('outcome', '') or '').strip().lower()
            if outcome not in _FIRE_OUTCOME_JUDGED:
                continue
            evidence = str(item.get('evidence', '') or '').strip()
            self._write_autonomy_event({
                'event': 'fire_outcome',
                'fire_id': fid,
                'concern_id': rec.get('concern_id'),
                'concern_text': rec.get('concern_text'),
                'exit_reason': rec.get('exit_reason'),
                'outcome': outcome,
                'valence': _clamp(item.get('valence')),
                'user_impact': _clamp(item.get('user_impact')),
                'evidence': evidence[:_FIRE_OUTCOME_EVIDENCE_CHARS] or None,
                'latency_turns': int(rec.get('user_turns_since', 0) or 0),
                'observed_at': now_iso,
            })
            resolved.append(fid)
        if resolved:
            # Reload before rewriting: aging on the inbox thread may have
            # expired records while reflection ran on the post-turn
            # executor — filtering the fresh registry can't resurrect them.
            current = self._load_pending_fire_outcomes()
            gone = set(resolved)
            self._save_pending_fire_outcomes(
                [r for r in current if str(r.get('fire_id')) not in gone])
            logger.info(
                f"[{self.character_name}] fire outcomes judged: "
                f"{len(resolved)} record(s) resolved")
        return resolved

    # ------------------------------------------------------------------
    # Surfacing helpers — top-K iteration over each collection for
    # prompt rendering and resource_browser queries.
    # ------------------------------------------------------------------

    def _iter_active_agent_concerns(self) -> List[Tuple[str, Dict[str, Any], float]]:
        """Iterate active agent_concerns: (note_id, note, activation)."""
        if not self._agent_concerns_collection_id:
            return []
        coll = self.resource_manager.get_resource(self._agent_concerns_collection_id)
        if not coll:
            return []
        out: List[Tuple[str, Dict[str, Any], float]] = []
        for nid in (coll.get('properties') or {}).get('content', []) or []:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.get('properties') or {}
            if props.get('status') != 'active':
                continue
            a = float(props.get('activation', 0.0) or 0.0)
            out.append((nid, note, a))
        return out

    def _iter_active_user_concerns(self) -> List[Tuple[str, Dict[str, Any], float]]:
        """Iterate active user_concerns: (note_id, note, strength)."""
        if not self._user_concerns_collection_id:
            return []
        coll = self.resource_manager.get_resource(self._user_concerns_collection_id)
        if not coll:
            return []
        out: List[Tuple[str, Dict[str, Any], float]] = []
        for nid in (coll.get('properties') or {}).get('content', []) or []:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.get('properties') or {}
            if props.get('status') != 'active':
                continue
            s = float(props.get('strength', 0.0) or 0.0)
            out.append((nid, note, s))
        return out

    def _top_active_agent_concerns(self, n: int = _AGENT_CONCERN_PROMPT_BUDGET
                                   ) -> List[Tuple[str, str, float, Dict[str, Any]]]:
        """Top-n agent_concerns by activation, descending. Tuple:
        (note_id, text, activation, props)."""
        active = self._iter_active_agent_concerns()
        active.sort(key=lambda t: t[2], reverse=True)
        out: List[Tuple[str, str, float, Dict[str, Any]]] = []
        for nid, note, a in active[:max(0, n)]:
            props = note.get('properties') or {}
            text = str(props.get('content', '') or note.get('text', '') or '').strip()
            if not text:
                continue
            out.append((nid, text, a, props))
        return out

    def _collect_concern_wip(self, exclude_id: Optional[str] = None
                             ) -> List[Tuple[str, str, float, str]]:
        """WIP inventory for a wip_reviewer fire: every active
        agent_concern carrying non-empty WIP, as (note_id, text,
        activation, wip), activation-descending. `exclude_id` drops the
        reviewer concern itself — its own WIP is review bookkeeping,
        not reviewable work."""
        out: List[Tuple[str, str, float, str]] = []
        for nid, note, a in self._iter_active_agent_concerns():
            if exclude_id and nid == exclude_id:
                continue
            props = note.get('properties') or {}
            wip = str(props.get('wip', '') or '').strip()
            if not wip:
                continue
            text = str(props.get('content', '') or note.get('text', '') or '').strip()
            out.append((nid, text, a, wip))
        out.sort(key=lambda t: t[2], reverse=True)
        return out

    def _top_active_user_concerns(self, n: int = _USER_CONCERN_PROMPT_BUDGET
                                  ) -> List[Tuple[str, str, float, Dict[str, Any]]]:
        """Top-n user_concerns by strength, descending. Tuple:
        (note_id, text, strength, props)."""
        active = self._iter_active_user_concerns()
        active.sort(key=lambda t: t[2], reverse=True)
        out: List[Tuple[str, str, float, Dict[str, Any]]] = []
        for nid, note, s in active[:max(0, n)]:
            props = note.get('properties') or {}
            text = str(props.get('content', '') or note.get('text', '') or '').strip()
            if not text:
                continue
            out.append((nid, text, s, props))
        return out

    def _set_concern_status(self, concern_id: str, new_status: str
                            ) -> Tuple[bool, Optional[str]]:
        """Manual status transition (browser-driven abandon/close).
        Accepts notes from either collection — kind check covers
        agent_concern, user_concern, and legacy 'concern'."""
        if new_status not in _CONCERN_STATUSES:
            return False, f"invalid status {new_status!r}"
        note = self.resource_manager.get_resource(concern_id)
        if not note:
            return False, f"concern {concern_id} not found"
        props = note.get('properties') or {}
        if props.get('kind') not in ('concern', 'agent_concern', 'user_concern'):
            return False, f"{concern_id} is not a concern"
        props['status'] = new_status
        return True, None

    # ------------------------------------------------------------------
    # Fire gate — deterministic, no LLM. Walks active agent_concerns
    # and returns those whose activation has crossed _AGENT_CONCERN_
    # FIRE_THRESHOLD AND have a non-null instruction. Service decrement
    # is applied separately via _service_agent_concern after the
    # autonomous run completes.
    # ------------------------------------------------------------------

    def _check_and_fire_agent_concerns(self) -> List[Tuple[str, str, str]]:
        """Identify agent_concerns ready to fire. Pure arithmetic over
        activation + instruction presence + status. Returns (note_id,
        text, instruction) tuples sorted by activation desc so the
        most-pressured concerns fire first when the per-tick cap
        bites."""
        active = self._iter_active_agent_concerns()
        active.sort(key=lambda t: t[2], reverse=True)
        fired: List[Tuple[str, str, str]] = []
        for nid, note, a in active:
            if a < _AGENT_CONCERN_FIRE_THRESHOLD:
                continue
            props = note.get('properties') or {}
            instruction = props.get('instruction')
            if not instruction:
                continue
            text = str(props.get('content', '') or note.get('text', '') or '').strip()
            fired.append((nid, text, str(instruction)))
        return fired

    # ------------------------------------------------------------------
    # Fire-time triage — the judgment step between "activation crossed
    # threshold" and "spin up a ReAct loop". One LLM call per candidate,
    # with the verdict cached on the note and aged so an over-threshold
    # concern that triage deferred doesn't re-cost an LLM call every
    # tick. Cache is invalidated by elapsed cooldown (rhythm-scaled) or
    # by a semantic evidence bump.
    # ------------------------------------------------------------------

    def _root_concern_id(self, nid: str) -> str:
        """Walk successor_of links to the root concern of a chain.
        Successor depth is capped at _CONCERN_SUCCESSOR_MAX_DEPTH so the
        walk is bounded; cycles (shouldn't happen) are guarded anyway."""
        seen = set()
        current = nid
        while current and current not in seen:
            seen.add(current)
            note = self.resource_manager.get_resource(current)
            if not note:
                return current
            parent = (note.get('properties') or {}).get('successor_of')
            if not parent:
                return current
            current = parent
        return current or nid

    def _triage_cache_live(self, props: Dict[str, Any]) -> bool:
        """True if the note carries a 'defer' verdict still inside its
        cooldown window. Evidence bumps clear the cached fields outright,
        so a live cache means: deferred recently, nothing new since."""
        if props.get('triage_verdict') != 'defer':
            return False
        triage_at_str = props.get('triage_at')
        if not triage_at_str:
            return False
        try:
            triage_at = datetime.fromisoformat(str(triage_at_str))
            if triage_at.tzinfo is None:
                triage_at = triage_at.replace(tzinfo=timezone.utc)
        except (TypeError, ValueError) as e:
            logger.warning(
                f"[{self.character_name}] unparseable triage_at "
                f"{triage_at_str!r}: {e}")
            return False
        elapsed_h = (datetime.now(timezone.utc) - triage_at).total_seconds() / 3600.0
        rhythm_h = self._resolve_rhythm_hours(props)
        return elapsed_h < _triage_defer_cooldown_hours(rhythm_h)

    def _triage_fire_candidate(self, nid: str, text: str, instruction: str) -> str:
        """Triage one fire-eligible concern. Returns 'fire', 'defer', or
        'reset'.

        defer — not worth acting on right now; verdict cached on the note
                and re-asked only after the cooldown or new evidence. No
                activation change, so the concern stays due.
        reset — nothing needed this cycle; activation decremented as if
                serviced (resets the pressure clock) without running
                ReAct. Triage cannot close or abandon a concern.
        fire  — dispatch as usual. Also the failure default: an LLM error
                or unparseable verdict preserves pre-triage behavior.
        """
        note = self.resource_manager.get_resource(nid)
        if not note:
            return 'defer'
        props = note.setdefault('properties', {})
        if self._triage_cache_live(props):
            logger.debug(
                f"[{self.character_name}] triage cache hit (defer) for {nid}")
            return 'defer'

        root = self.resource_manager.get_resource(self._root_concern_id(nid))
        wip = str(((root or {}).get('properties') or {}).get('wip', '') or '').strip()
        sys_msg = (
            f"You are the autonomy triage step for the agent {self.character_name}. "
            "A standing concern's activation has crossed its fire threshold; "
            "before an autonomous work loop is spun up, judge whether acting "
            "NOW is actually warranted. Consider: is there anything new since "
            "the last fire, is the work already done (see work-in-progress), "
            "and would acting help?\n\n"
            "Respond ONLY with JSON, no prose, no markdown. Shapes:\n"
            "  {\"verdict\": \"fire\"}                          ← act now\n"
            "  {\"verdict\": \"defer\", \"reason\": \"<short>\"}  ← not now; re-ask later or on new evidence\n"
            "  {\"verdict\": \"reset\", \"reason\": \"<short>\"}  ← nothing needed this cycle; reset the pressure clock"
        )
        lines = [
            f"Concern: {text}",
            f"Instruction that would run: {instruction}",
            f"Activation: {float(props.get('activation', 0.0) or 0.0):.2f}",
            f"Last fired: {props.get('last_fired_at') or 'never'}",
        ]
        if props.get('last_bumped_at'):
            lines.append(f"Last evidence bump: {props['last_bumped_at']}")
        if props.get('triage_reason'):
            lines.append(f"Previous defer reason: {props['triage_reason']}")
        if wip:
            lines.append(f"Work-in-progress from earlier fires:\n{wip}")
        user_msg = "\n".join(lines)

        verdict, reason = 'fire', ''
        try:
            raw = self.backend.chat(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': user_msg}],
                max_tokens=512, temperature=0.2, cot_profile='none')
            data = repair_json_string(raw or '')
            if isinstance(data, dict) and data.get('verdict') in ('fire', 'defer', 'reset'):
                verdict = data['verdict']
                reason = str(data.get('reason') or '').strip()
            else:
                logger.warning(
                    f"[{self.character_name}] triage verdict unparseable for "
                    f"{nid} (raw={(raw or '')[:200]!r}); failing open to fire")
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] triage LLM failed for {nid}: {e}; "
                f"failing open to fire")

        now_iso = datetime.now(timezone.utc).isoformat()
        if verdict == 'defer':
            props['triage_verdict'] = 'defer'
            props['triage_at'] = now_iso
            props['triage_reason'] = reason
        else:
            for stale in ('triage_verdict', 'triage_at', 'triage_reason'):
                props.pop(stale, None)
            if verdict == 'reset':
                a = float(props.get('activation', 0.0) or 0.0)
                props['activation'] = max(0.0, a - _AGENT_CONCERN_SERVICE_FULL)
                props['last_fired_at'] = now_iso
        self._write_autonomy_event({
            'event': 'triage',
            'concern_id': nid,
            'concern_text': text,
            'verdict': verdict,
            'reason': reason,
        })
        return verdict

    # ------------------------------------------------------------------
    # Concern queryables (browser-facing).
    # The Resource Browser frontend expects the executive's shape
    # ({user_concerns: [...], derived_concerns: [...], activations: {}});
    # we adapt chat's single concerns collection to that split:
    #   - user_concerns  = category in (one_shot, durable). Status remap:
    #                      active→open, satisfied→closed, abandoned→abandoned
    #                      so the UI's user-concern color/label code (which
    #                      checks "open"/"closed") works unchanged.
    #   - derived_concerns = category == derived. Status passthrough.
    # ------------------------------------------------------------------

    _USER_CONCERN_STATUS_MAP = {'active': 'open', 'satisfied': 'closed',
                                'abandoned': 'abandoned'}

    def _serialize_concern(self, note_id: str, note: Dict[str, Any],
                           is_user_kind: bool) -> Dict[str, Any]:
        """Build a dict matching the field names the resource_browser
        frontend looks for. The browser falls back gracefully through
        concern_description || description, concern_label || name, etc.,
        so we populate both forms to keep the UI working."""
        props = note.get('properties') or {}
        text = str(props.get('content', '') or '')
        status = props.get('status', 'active')
        if is_user_kind:
            status = self._USER_CONCERN_STATUS_MAP.get(status, status)
        # Compact label for the list-row heading: first line, capped.
        first_line = text.split('\n', 1)[0].strip()
        if len(first_line) > 60:
            label = first_line[:60].rstrip() + '…'
        else:
            label = first_line or note_id
        return {
            # IDs (browser checks concern_id first, falls back to id)
            'concern_id': note_id,
            'id': note_id,
            # Headings (browser checks concern_label || name)
            'concern_label': label,
            'name': label,
            # Body text (browser checks concern_description || description)
            'concern_description': text,
            'description': text,
            # State + lifecycle
            'category': props.get('category', 'durable'),  # legacy field; may be empty for new notes
            'kind': props.get('kind', 'concern'),
            'status': status,
            # Browser displays a single 'weight' bar — pick activation
            # for agent_concerns, strength for user_concerns. Both are
            # in [0,1] so the visual scale is consistent.
            'weight': float(
                (props.get('activation') if props.get('kind') == 'agent_concern'
                 else props.get('strength') if props.get('kind') == 'user_concern'
                 else 0.0) or 0.0),
            'activation': props.get('activation'),
            'strength': props.get('strength'),
            'provenance': props.get('provenance', 'asserted'),
            'origin': 'seed' if props.get('seed') else 'reflection',
            'seed': bool(props.get('seed', False)),
            'entity': props.get('entity', ''),
            'created_at': props.get('created_at', ''),
            'created': props.get('created_at', ''),
            'last_engaged_at': props.get('last_engaged_at', ''),
            'last_bumped_at': props.get('last_bumped_at', ''),
            'recency': (props.get('last_bumped_at')
                        or props.get('last_engaged_at', '')),
            # Firing parameters (agent_concerns only; user_concerns ignore).
            # rhythm_hours editable via concern_manage set_rhythm_hours.
            'rhythm_hours': self._resolve_rhythm_hours(props),
            'rhythm_source': props.get('rhythm_source', 'default'),
            'rhythm_hours_allowed': list(_AGENT_CONCERN_RHYTHM_HOURS_ALLOWED),
            # Legacy aliases retained so the browser UI keeps rendering
            # while we refactor it in item 7.
            'cadence_hours': self._resolve_rhythm_hours(props),
            'cadence_hours_allowed': list(_AGENT_CONCERN_RHYTHM_HOURS_ALLOWED),
            'lifetime_days': None,
            'instruction': props.get('instruction'),
            'last_acted_at': (props.get('last_fired_at')
                              or props.get('last_acted_at')   # legacy
                              or None),
            'last_fired_at': props.get('last_fired_at'),
        }

    def _all_concerns_split(self) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        """Iterate both concern collections for the browser API. Returns
        (user_concerns, agent_concerns) — keyed at the API layer as
        user_concerns / derived_concerns for back-compat with the
        existing UI tabs (rename to come in item 7). No status filter:
        browser shows all states."""
        user_out: List[Dict[str, Any]] = []
        agent_out: List[Dict[str, Any]] = []
        if self._user_concerns_collection_id:
            coll = self.resource_manager.get_resource(self._user_concerns_collection_id)
            if coll:
                for nid in (coll.get('properties') or {}).get('content', []) or []:
                    note = self.resource_manager.get_resource(nid)
                    if not note:
                        continue
                    user_out.append(self._serialize_concern(nid, note, is_user_kind=True))
        if self._agent_concerns_collection_id:
            coll = self.resource_manager.get_resource(self._agent_concerns_collection_id)
            if coll:
                for nid in (coll.get('properties') or {}).get('content', []) or []:
                    note = self.resource_manager.get_resource(nid)
                    if not note:
                        continue
                    agent_out.append(self._serialize_concern(nid, note, is_user_kind=False))
        return (user_out, agent_out)

    def _maybe_spawn_successor_concern(self, parent_id: str, parent_instruction: str,
                                       log: List[Tuple[str, str]]) -> Optional[str]:
        """When an autonomous ReAct loop hits max_iters, decide whether
        to spawn a successor concern carrying the narrowed remainder of
        the work. Returns the new concern's id, or None if no successor
        was spawned (work complete, depth cap reached, parse/LLM failure,
        or LLM judged the remainder not worth pursuing).

        Successor depth is capped at _CONCERN_SUCCESSOR_MAX_DEPTH so
        chains can't run away. The successor is created with skip_recurrence
        so it doesn't fold back into the parent via similarity merge.
        """
        parent = self.resource_manager.get_resource(parent_id)
        if not parent:
            return None
        parent_props = parent.get('properties') or {}
        parent_depth = int(parent_props.get('successor_depth', 0) or 0)
        if parent_depth >= _CONCERN_SUCCESSOR_MAX_DEPTH:
            logger.info(
                f"[{self.character_name}] successor cap reached for {parent_id} "
                f"(depth={parent_depth}); standing on fallback response.")
            return None
        # WIP from earlier fires: lets the synthesizer judge what's left
        # against accumulated findings, not just this loop's log tail.
        root = self.resource_manager.get_resource(self._root_concern_id(parent_id))
        wip = str(((root or {}).get('properties') or {}).get('wip', '') or '').strip()
        next_slice = self._synthesize_remainder(parent_instruction, log, wip=wip)
        if not next_slice:
            return None
        return self._create_successor_concern(parent_id, next_slice)

    def _synthesize_remainder(self, instruction: str,
                              log: List[Tuple[str, str]],
                              wip: str = '') -> Optional[str]:
        """LLM judgment shared by the max_iters continuation routes
        (autonomous successor + user-turn spawn): did the cut-off loop
        substantively complete its instruction, and if not, what narrow
        next slice should run next? Returns the imperative slice, or
        None (complete, LLM failure, or parse failure)."""
        # Show the synthesizer enough log to judge what's left, but not
        # so much it drowns in detail. Last 10 entries, content trimmed.
        tail = log[-10:] if len(log) > 10 else log
        summary = "\n".join(f"{label}: {content[:300]}" for label, content in tail)
        sys_msg = (
            f"You evaluate whether a {self.character_name}-side ReAct loop "
            "completed its instruction. The loop ran to its iteration cap "
            "without emitting a final `respond` action. Decide: did the "
            "work substantively complete (just missing the wrap-up), or "
            "is real work left? If left, what is the narrow next slice "
            "(one ReAct loop's worth, ~12 tool calls) that should run "
            "next?\n\nRespond ONLY with JSON, no prose, no markdown."
        )
        wip_section = (
            f"Accumulated work-in-progress from earlier fires:\n{wip}\n\n"
            if wip else ""
        )
        user_msg = (
            f"Original instruction: {instruction}\n\n"
            f"{wip_section}"
            f"ReAct working log (last entries):\n{summary}\n\n"
            "JSON shapes:\n"
            "  {\"complete\": true}                                  ← work substantively done\n"
            "  {\"complete\": false, \"next_slice\": \"<imperative>\"}  ← real work remains"
        )
        try:
            raw = self.backend.chat(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': user_msg}],
                max_tokens=4096, temperature=0.2, cot_profile='none')
        except Exception as e:
            logger.warning(f"[{self.character_name}] successor synth LLM failed: {e}")
            return None
        data = repair_json_string(raw or '')
        if data is None:
            logger.warning(
                f"[{self.character_name}] successor JSON parse failed; "
                f"raw={(raw or '')[:200]!r}")
            return None
        if not isinstance(data, dict) or data.get('complete'):
            return None
        return str(data.get('next_slice') or '').strip() or None

    def _spawn_successor_from_yield(self, parent_id: str,
                                    next_slice: str) -> Optional[str]:
        """Intentional-yield continuation: the autonomous ReAct loop ended
        with a `yield` action carrying the follow-up instruction verbatim,
        so no synthesizer LLM pass is needed — the agent stated the
        remainder herself at a boundary she chose. Same creation path and
        depth cap as the reactive max_iters route."""
        return self._create_successor_concern(parent_id, next_slice)

    def _spawn_concern_from_user_yield(self, turn_text: str,
                                       next_slice: str) -> Optional[str]:
        """User-turn intentional yield: a ReAct loop serving a USER (or
        sensor) turn ended with `yield`, carrying the remainder verbatim.
        There is no parent concern, so create a fresh agent_concern —
        the concern machinery then continues the user's task without
        another prod. Starts at successor_depth 0: if the spawned
        concern's own fire yields again, the ordinary successor path and
        its depth cap take over."""
        next_slice = (next_slice or '').strip()
        if not next_slice:
            return None
        turn_text = ' '.join((turn_text or '').split())
        succ_text = (f"continue work I yielded mid-turn: {turn_text[:140]}"
                     if turn_text else "continue work I yielded mid-turn")
        new_id = self._add_agent_concern(
            text=succ_text, entity='User', provenance='inferred',
            seed=False, name='',
            rhythm_hours=1, rhythm_source='urgency',
            instruction=next_slice,
            skip_recurrence=True,
            extra_properties={
                # Prime activation just below threshold so the next tick's
                # growth pushes it over — same priming as successors. The
                # provenance trail lives in autonomy.jsonl (via: user_yield);
                # no dedicated note property (create_note whitelists fields).
                'activation': max(0.0,
                                  _AGENT_CONCERN_FIRE_THRESHOLD - 0.1),
            },
        )
        if not new_id:
            return None
        logger.info(
            f"[{self.character_name}] spawned agent concern {new_id} from "
            f"user-turn yield: {next_slice[:120]!r}")
        return new_id

    def _create_successor_concern(self, parent_id: str,
                                  next_slice: str) -> Optional[str]:
        """Create the successor agent_concern carrying next_slice as its
        instruction. Shared tail of the reactive (max_iters + synthesizer)
        and intentional (yield) continuation paths. Enforces the depth cap
        — successor chains can't run away regardless of route. Returns the
        new concern id, or None (missing parent, cap, empty slice)."""
        next_slice = (next_slice or '').strip()
        if not next_slice:
            return None
        parent = self.resource_manager.get_resource(parent_id)
        if not parent:
            return None
        parent_props = parent.get('properties') or {}
        parent_depth = int(parent_props.get('successor_depth', 0) or 0)
        if parent_depth >= _CONCERN_SUCCESSOR_MAX_DEPTH:
            logger.info(
                f"[{self.character_name}] successor cap reached for {parent_id} "
                f"(depth={parent_depth}); no successor created.")
            return None
        # Successor inherits the parent's surface text + a depth tag so it
        # reads coherently in the active-concerns surface.
        parent_text = str(parent_props.get('content', '') or parent.get('text', '') or '').strip()
        new_depth = parent_depth + 1
        succ_text = (f"{parent_text} — continuation (depth {new_depth})"
                     if parent_text else f"continuation of {parent_id} (depth {new_depth})")
        # Successor: same kind as parent (agent_concern). Fast rhythm
        # so the remainder runs promptly. Pre-loaded activation so it
        # fires on the next tick without waiting for full growth from
        # 0 — successors carry the parent's not-quite-finished work.
        new_id = self._add_agent_concern(
            text=succ_text, entity='User', provenance='inferred',
            seed=False, name='',
            rhythm_hours=1, rhythm_source='urgency',
            instruction=next_slice,
            skip_recurrence=True,
            extra_properties={
                'successor_of': parent_id,
                'successor_depth': new_depth,
                # Prime activation just below threshold so the next tick's
                # growth pushes it over without making the spawn itself
                # eligible (avoids re-firing in the same tick window).
                'activation': max(0.0,
                                  _AGENT_CONCERN_FIRE_THRESHOLD - 0.1),
            },
        )
        if not new_id:
            return None
        logger.info(
            f"[{self.character_name}] spawned successor concern {new_id} for "
            f"{parent_id} (depth {new_depth}): {next_slice[:120]!r}")
        return new_id

    def _update_concern_wip(self, concern_id: str, fire_text: str,
                            log: List[Tuple[str, str]], reply: str,
                            exit_reason: str) -> None:
        """Rewrite the root concern's work-in-progress summary after an
        autonomous fire. One LLM call: previous WIP + this fire's log
        tail + reply → updated running summary. Runs on the post-turn
        executor so it never blocks the inbox loop; persists when done
        so WIP survives a restart. Failures log and stop — a missed WIP
        update just means the next fire starts from the previous one."""
        try:
            root_id = self._root_concern_id(concern_id)
            root = self.resource_manager.get_resource(root_id)
            if not root:
                logger.info(
                    f"[{self.character_name}] WIP update skipped: concern "
                    f"{root_id} no longer exists")
                return
            props = root.setdefault('properties', {})
            prev_wip = str(props.get('wip', '') or '').strip()
            tail = log[-10:] if len(log) > 10 else log
            summary = "\n".join(f"{label}: {content[:300]}" for label, content in tail)
            sys_msg = (
                "You maintain the work-in-progress (WIP) note for a standing "
                f"concern of the agent {self.character_name}. The concern just "
                "fired an autonomous work loop. Rewrite the WIP as a running "
                "summary of accumulated findings and open threads that the "
                "NEXT fire should know: what's been established, what's "
                "pending, what to not redo. Drop items that are stale or "
                "resolved. If this fire left a genuine unfinished arc — a "
                "lead worth pursuing, a question raised but not answered, "
                "work blocked on something — end the WIP with one line "
                "starting exactly 'NEXT: ' stating the single most promising "
                "next step. If nothing is genuinely pending, omit the NEXT "
                "line entirely — do not invent one. Max 200 words. Output "
                "the WIP text only — no preamble, no headers."
            )
            user_msg = (
                f"Concern: {str(props.get('content', '') or '').strip()}\n\n"
                f"Previous WIP:\n{prev_wip or '(none)'}\n\n"
                f"This fire's instruction: {fire_text}\n"
                f"Exit: {exit_reason}\n"
                f"Working log (last entries):\n{summary}\n\n"
                f"Final output of this fire:\n{(reply or '').strip()[:1000]}"
            )
            new_wip = self.backend.chat(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': user_msg}],
                max_tokens=1024, temperature=0.3, cot_profile='none')
            new_wip = (new_wip or '').strip()
            if not new_wip:
                logger.warning(
                    f"[{self.character_name}] WIP update produced empty "
                    f"summary for {root_id}; keeping previous WIP")
                return
            props['wip'] = new_wip[:_CONCERN_WIP_MAX_CHARS]
            props['wip_updated_at'] = datetime.now(timezone.utc).isoformat()
            self._persist_to_disk()
            logger.info(
                f"[{self.character_name}] WIP updated for {root_id} "
                f"({len(props['wip'])} chars)")
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] _update_concern_wip failed for "
                f"{concern_id}: {e}")
