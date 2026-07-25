"""Disposition state — Tier-1 capture and Tier-2 render.

Step 1 of docs/learned-disposition-design.md §Build order: shadow-log the
state a learned disposition scorer will eventually condition on. No
behavior change; nothing here feeds the triage verdict yet.

Two tiers (settled 2026-07-25):

  Tier 1 — capture. One JSON record per real triage evaluation, written
    to <memory>/disposition_state.jsonl. Deliberately wider than what
    v1 will condition on: fields not logged from day one are gone
    forever, and the render schema stays revisable afterwards.

  Tier 2 — condition on. `render_disposition_state()` turns a record
    (plus derived history) into the fixed-schema text ledger the scorer
    reads. Revisable; bump _SCHEMA_VERSION when the record shape changes.

Capture principle: **snapshot what is ephemeral, derive what is
reconstructible.** Counts over autonomy.jsonl (fire rates, outcome
histories, hit rate) and over this file (precedent density) are exactly
reconstructible after the fact by filtering on ts, so they are NOT
captured — they are computed by the derive_* functions below, which
every caller must feed only records preceding the row's own timestamp.
Computing them over the whole corpus at training time would leak each
row's own future.

Joining a state row to its label: rows carry concern_id + ts; the fire
event that follows a 'fire' verdict is minted seconds later in the same
tick (fire_id does not exist yet at triage time). Offline, join by
nearest-preceding state row for that concern_id.
"""

from __future__ import annotations

import json
import logging
import statistics
from datetime import datetime, timedelta, timezone
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger('chat_loop')

_SCHEMA_VERSION = 1
_STATE_FILE = 'disposition_state.jsonl'

# Clip budgets — the render is read by a ≤1B model; text length is the
# scarce resource, not field count.
_CLIP_WIP = 400
_CLIP_TURN = 300
_CLIP_LAST_USER = 200
_CLIP_CONCERN_TEXT = 300

_RECENT_TURNS = 4          # captured; only the last inbound one renders
_HOT_CONCERNS = 3
_OUTCOME_HISTORY = 5       # captured depth; render shows 3
_HIT_RATE_WINDOW = 8       # judged acts behind "recent read on this user"

# Precedent density (epistemic novelty). Thresholds are UNTUNED — there
# is no corpus yet to tune them against; revisit once one exists.
_PRECEDENT_SIM = 0.70      # bge-small cosine
_PRECEDENT_WINDOW_DAYS = 90
_PRECEDENT_MIN = 5         # below this the line renders as insufficient

_GOOD_OUTCOMES = ('helped', 'neutral')
_BAD_OUTCOMES = ('hindered', 'ignored')


# ----------------------------------------------------------------------
# Capture (live, on the triage path)
# ----------------------------------------------------------------------

class DispositionMixin:
    """Mixin for ChatLoop. Capture only — no scoring, no gating."""

    def _disposition_state_path(self) -> 'Any':
        """Path to <memory>/disposition_state.jsonl. Separate from
        autonomy.jsonl on purpose: that file is the grep-able 'what has
        she been doing' stream and a ~2KB state block per triage would
        wreck it."""
        return self._memory_dir() / _STATE_FILE

    def _capture_disposition_state(
            self, nid: str, text: str, instruction: str,
            props: Dict[str, Any], wip: str) -> Optional[Dict[str, Any]]:
        """Snapshot the ephemeral half of triage state. Called at the TOP
        of _triage_fire_candidate, before the verdict mutates props
        (defer writes triage_reason; reset decrements activation).

        Returns the record for the caller to stamp a verdict onto and
        write, or None if capture failed. Never raises: a lost state row
        costs one training datum, not a turn."""
        try:
            now = datetime.now(timezone.utc)
            local = datetime.now()
            record: Dict[str, Any] = {
                'schema_version': _SCHEMA_VERSION,
                'ts': now.isoformat(),
                'character': self.character_name,
                'concern': {
                    'id': nid,
                    'root_id': self._root_concern_id(nid),
                    'text': text,
                    'instruction': instruction,
                    'activation': float(props.get('activation', 0.0) or 0.0),
                    'threshold': _fire_threshold(),
                    'rhythm_hours': self._resolve_rhythm_hours(props),
                    'last_fired_at': props.get('last_fired_at'),
                    'last_bumped_at': props.get('last_bumped_at'),
                    'prior_defer_reason': props.get('triage_reason'),
                    'wip': wip,
                    'status': props.get('status'),
                    'status_changed_at': props.get('status_changed_at'),
                    'self_extension': bool(props.get('self_extension')),
                    'domain': props.get('domain'),
                },
                'situation': {
                    'local_time': local.isoformat(),
                    'weekday': local.strftime('%a'),
                    'hour': local.hour,
                    'pending_unjudged_fires': len(self._load_pending_fire_outcomes()),
                },
                'substrate': {
                    'backend_server': getattr(self.backend, 'server', None),
                    'backend_model': getattr(self.backend, 'model', None),
                },
            }
            record['situation'].update(self._disposition_dialog_slice())
            record['situation']['hot_user_concerns'] = [
                {'text': t, 'strength': round(float(s), 4)}
                for _, t, s, _ in self._top_active_user_concerns(_HOT_CONCERNS)
            ]
            record['situation']['topic_fit'] = self._disposition_topic_fit(text)
            return record
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] disposition capture failed for {nid}: {e}")
            return None

    def _disposition_dialog_slice(self) -> Dict[str, Any]:
        """Recent exchange, last-inbound-turn age/text, and the companion
        and discourse state blocks. Ages are computed here rather than
        stored as two timestamps: turn records stamp naive local time
        (conversation_store) while autonomy stamps aware UTC, and
        differencing them offline is a silent-offset bug waiting to
        happen.

        Entity is 'User' — the same default prompts.py assumes for
        history. Off-turn there is no source to resolve."""
        out: Dict[str, Any] = {
            'recent_exchange': [], 'user_last_turn_age_s': None,
            'user_last_turn_text': None, 'companion_state': None,
            'discourse_state': None,
        }
        entity = 'User'
        try:
            out['companion_state'] = (self._companion_state.get(entity) or '').strip() or None
            out['discourse_state'] = (self._discourse_state.get(entity) or '').strip() or None
            turns = self.store.get_recent_turns(entity, limit=_RECENT_TURNS, scope='all')
            now_local = datetime.now()
            for t in turns:
                out['recent_exchange'].append({
                    'source': t.get('source'),
                    'direction': t.get('direction'),
                    'ts': t.get('timestamp'),
                    'text': _clip(str(t.get('text', '')), _CLIP_TURN),
                })
            for t in reversed(turns):
                if t.get('direction') != 'in':
                    continue
                out['user_last_turn_text'] = _clip(str(t.get('text', '')), _CLIP_LAST_USER)
                stamped = _parse_ts(t.get('timestamp'))
                if stamped is not None:
                    out['user_last_turn_age_s'] = max(
                        0.0, (now_local - stamped.replace(tzinfo=None)).total_seconds())
                break
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] disposition dialog slice failed: {e}")
        return out

    def _disposition_topic_fit(self, text: str) -> Optional[Dict[str, Any]]:
        """Cosine of the concern text against active thread centroids —
        how close this concern sits to what has actually been under
        discussion.

        Deliberately does NOT call _compute_thread_activation: that
        helper caches its embedding in _current_turn_embedding, which
        _update_thread_centroids consumes on the next post-turn pass.
        Calling it off-turn would drift centroids toward concern text."""
        try:
            threads = self._get_threads(statuses=('active',))
            if not threads:
                return None
            self.resource_manager._init_embedder()
            embedder = self.resource_manager.embedder
            if embedder is None:
                logger.warning(
                    f"[{self.character_name}] disposition topic_fit: embedder unavailable")
                return None
            import numpy as np
            emb = embedder.encode(text, normalize_embeddings=True,
                                  convert_to_numpy=True, show_progress_bar=False)
            best_name, best_cos = None, None
            for t in threads:
                c = t.get('centroid_embedding') or []
                if not c:
                    continue
                cv = np.asarray(c, dtype=np.float32)
                if cv.shape != emb.shape:
                    continue
                cos = float(np.dot(emb, cv))
                if best_cos is None or cos > best_cos:
                    best_cos, best_name = cos, t.get('name')
            if best_cos is None:
                return None
            return {'top_thread': best_name, 'cosine': round(best_cos, 4),
                    'n_active_threads': len(threads)}
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] disposition topic_fit failed: {e}")
            return None

    def _log_disposition_state(self, record: Optional[Dict[str, Any]],
                               verdict: str, reason: str) -> None:
        """Stamp the verdict onto a captured record and append it.
        Best-effort; failures warn and are dropped."""
        if not record:
            return
        try:
            record['verdict'] = verdict
            record['verdict_reason'] = reason
            path = self._disposition_state_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] disposition_state write failed: {e}")


# ----------------------------------------------------------------------
# Derivation (offline / pure). Every function takes only history that
# precedes `ts` — callers must not hand it the row's own future.
# ----------------------------------------------------------------------

def load_states(path) -> List[Dict[str, Any]]:
    """Read disposition_state.jsonl. Unparseable lines warn and skip."""
    out: List[Dict[str, Any]] = []
    try:
        text = path.read_text(encoding='utf-8')
    except FileNotFoundError:
        return out
    for lineno, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError as e:
            logger.warning("disposition_state line %d unparseable: %s", lineno, e)
    return out


def derive_act_history(events: List[Dict[str, Any]], concern_id: str,
                       ts: datetime) -> Dict[str, Any]:
    """Act history from autonomy.jsonl events strictly before `ts`.

    'Real progress' for the fires-since counter means an outcome judged
    helpful, or the concern reaching satisfied — a fire that produced
    neither advanced nothing, which is the rumination signal.
    """
    past = [e for e in events if _before(e.get('ts'), ts)]
    fires = [e for e in past if e.get('event') == 'fire']
    day, week = ts - timedelta(days=1), ts - timedelta(days=7)

    def _since(seq, cut):
        return [e for e in seq if _before(cut, _parse_ts(e.get('ts')) or ts)]

    mine = [e for e in fires if e.get('concern_id') == concern_id]
    outcomes = [e for e in past
                if e.get('event') == 'fire_outcome'
                and e.get('outcome') in _GOOD_OUTCOMES + _BAD_OUTCOMES]
    mine_outcomes = [e for e in outcomes if e.get('concern_id') == concern_id]

    # Consecutive ignored, walking back from the most recent judged act.
    streak = 0
    for e in reversed(outcomes):
        if e.get('outcome') == 'ignored':
            streak += 1
        else:
            break

    # Fires since the last helped-outcome or satisfaction for this concern.
    progress_at = None
    for e in past:
        if e.get('concern_id') != concern_id:
            continue
        if (e.get('event') == 'concern_satisfied'
                or (e.get('event') == 'fire_outcome' and e.get('outcome') == 'helped')):
            progress_at = _parse_ts(e.get('ts')) or progress_at
    fires_since_progress = (len(mine) if progress_at is None
                            else len([e for e in mine
                                      if _before(progress_at, _parse_ts(e.get('ts')) or ts)]))

    window = outcomes[-_HIT_RATE_WINDOW:]
    return {
        'fires_24h': len(_since(fires, day)),
        'fires_7d': len(_since(fires, week)),
        'fires_24h_concern': len(_since(mine, day)),
        'fires_7d_concern': len(_since(mine, week)),
        # Trailing baseline: mean fires/day over the week, so the 24h
        # count reads as a deviation rather than a bare integer.
        'fires_24h_baseline': round(len(_since(fires, week)) / 7.0, 1),
        'fires_since_progress': fires_since_progress,
        'deferred_24h': len(_since([e for e in past if e.get('event') == 'deferred'], day)),
        'capability_gaps_7d': len(_since(
            [e for e in past if e.get('event') == 'capability_gap'], week)),
        'outcomes_concern': [e.get('outcome') for e in mine_outcomes[-_OUTCOME_HISTORY:]],
        'outcomes_recent': [e.get('outcome') for e in outcomes[-_OUTCOME_HISTORY:]],
        'consecutive_ignored': streak,
        'triage_verdicts_concern': [
            e.get('verdict') for e in past
            if e.get('event') == 'triage' and e.get('concern_id') == concern_id
        ][-_OUTCOME_HISTORY:],
        'hit_rate_n': len(window),
        'hit_rate_good': len([e for e in window if e.get('outcome') in _GOOD_OUTCOMES]),
    }


def derive_user_rhythm(turns: List[Dict[str, Any]], ts: datetime) -> Dict[str, Any]:
    """Where `ts` sits in this user's own activity rhythm — activity level
    for this hour-of-week and the typical inbound gap around it. Absolute
    clock time is a weak feature; position in the user's cycle is the one
    the literature says carries the signal."""
    stamps = sorted(s for s in (_parse_ts(t.get('timestamp')) for t in turns
                                if t.get('direction') == 'in') if s is not None)
    if len(stamps) < 20:
        return {'activity_level': None, 'typical_gap_s': None, 'n_turns': len(stamps)}

    hour_counts: Dict[int, int] = {}
    for s in stamps:
        hour_counts[s.hour] = hour_counts.get(s.hour, 0) + 1
    here = hour_counts.get(ts.hour, 0)
    ranked = sorted(hour_counts.values())
    pct = len([c for c in ranked if c < here]) / float(len(ranked))
    level = 'high' if pct >= 0.66 else ('medium' if pct >= 0.33 else 'low')

    # Typical gap: median inbound inter-arrival among turns in this hour ±1.
    gaps = [(b - a).total_seconds()
            for a, b in zip(stamps, stamps[1:])
            if abs(b.hour - ts.hour) <= 1 and (b - a).total_seconds() < 86400]
    return {'activity_level': level,
            'typical_gap_s': round(statistics.median(gaps), 1) if gaps else None,
            'n_turns': len(stamps)}


def derive_precedent_density(prior_states: List[Dict[str, Any]],
                             record: Dict[str, Any], embedder,
                             judged_concern_ids: Optional[set] = None
                             ) -> Dict[str, Any]:
    """Epistemic novelty: how many states like this one the corpus
    already holds, and how many of those were ever judged. The
    calibration guard — a scorer should read differently on a state with
    no precedent than on a well-trodden one.

    `prior_states` MUST already be filtered to rows preceding `record`;
    computing this over the full corpus lets every training row see its
    own future, which looks like a strong feature and generalizes to
    nothing."""
    text = ((record.get('concern') or {}).get('text') or '').strip()
    if not text or not prior_states or embedder is None:
        return {'n_precedents': 0, 'n_judged': 0}
    try:
        import numpy as np
        cutoff = (_parse_ts(record.get('ts')) or datetime.now(timezone.utc)
                  ) - timedelta(days=_PRECEDENT_WINDOW_DAYS)
        window = [r for r in prior_states
                  if _before(cutoff, _parse_ts(r.get('ts')) or cutoff)]
        if not window:
            return {'n_precedents': 0, 'n_judged': 0}
        texts = [((r.get('concern') or {}).get('text') or '') for r in window]
        embs = embedder.encode(texts + [text], normalize_embeddings=True,
                               convert_to_numpy=True, show_progress_bar=False)
        sims = np.dot(embs[:-1], embs[-1])
        near = [r for r, s in zip(window, sims) if float(s) >= _PRECEDENT_SIM]
        judged = judged_concern_ids or set()
        return {
            'n_precedents': len(near),
            'n_judged': len([r for r in near
                             if (r.get('concern') or {}).get('id') in judged]),
        }
    except Exception as e:
        logger.warning("precedent density failed: %s", e)
        return {'n_precedents': 0, 'n_judged': 0}


# ----------------------------------------------------------------------
# Render (Tier 2) — the fixed-schema text the scorer conditions on.
# ----------------------------------------------------------------------

def render_disposition_state(record: Dict[str, Any],
                             derived: Optional[Dict[str, Any]] = None) -> str:
    """Render one state record as the Tier-2 text ledger.

    `derived` merges the derive_* outputs; every key is optional and
    renders as 'unknown' / 'insufficient history' when absent. That is
    not a placeholder for missing code — for the first months of
    collection it is the honest reading, and the scorer should see it."""
    d = derived or {}
    c = record.get('concern') or {}
    s = record.get('situation') or {}
    ts = _parse_ts(record.get('ts')) or datetime.now(timezone.utc)
    L: List[str] = ['== concern ==', _clip(str(c.get('text') or ''), _CLIP_CONCERN_TEXT)]

    L.append(f"instruction: {_clip(str(c.get('instruction') or ''), _CLIP_CONCERN_TEXT)}")
    rhythm = c.get('rhythm_hours')
    L.append(
        f"activation {float(c.get('activation') or 0.0):.2f} "
        f"(fires at {float(c.get('threshold') or 0.0):.2f})"
        + (f" · rhythm {rhythm}h" if rhythm else ""))
    L.append(f"last fired {_ago(c.get('last_fired_at'), ts)} · "
             f"last evidence bump {_ago(c.get('last_bumped_at'), ts)}")
    if c.get('prior_defer_reason'):
        L.append(f'prior defer: "{c["prior_defer_reason"]}"')
    if c.get('wip'):
        L.append(f"wip: {_clip(str(c['wip']), _CLIP_WIP)}")
    if 'fires_since_progress' in d:
        L.append(f"fires since last real progress: {d['fires_since_progress']}")
    L.append("outcomes for this concern: "
             + (", ".join(d.get('outcomes_concern') or []) or "none yet"))

    L.append('')
    L.append('== situation ==')
    local = _parse_ts(s.get('local_time'))
    when = local.strftime('%a %H:%M') if local else 'unknown time'
    L.append(f"{when} local — user usually active at this hour: "
             f"{d.get('activity_level') or 'unknown'}")
    gap = d.get('typical_gap_s')
    L.append(f"user last spoke {_ago_s(s.get('user_last_turn_age_s'))}"
             + (f" (typical gap at this hour: {_dur(gap)})" if gap else ""))
    if s.get('user_last_turn_text'):
        L.append(f'user last said: "{s["user_last_turn_text"]}"')
    fires, base = d.get('fires_24h'), d.get('fires_24h_baseline')
    if fires is not None:
        L.append(f"my last 24h: {fires} fires"
                 + (f" (typical {base})" if base is not None else "")
                 + " · last 3 landed: "
                 + (", ".join((d.get('outcomes_recent') or [])[-3:]) or "nothing judged"))
    if s.get('pending_unjudged_fires'):
        L.append(f"{s['pending_unjudged_fires']} fires still unacknowledged")
    hot = s.get('hot_user_concerns') or []
    if hot:
        L.append("hot user concerns: "
                 + "; ".join(f"{h['text']} ({h['strength']:.2f})" for h in hot))
    L.append(f"this concern vs. current conversation: {_fit((s.get('topic_fit') or {}).get('cosine'))}")
    n, good = d.get('hit_rate_n'), d.get('hit_rate_good')
    L.append(f"recent read on this user: last {n} judged acts — {good} landed well"
             if n else "recent read on this user: nothing judged yet")
    prec, judged = d.get('n_precedents'), d.get('n_judged')
    L.append(f"states like this: {prec} precedents in 90d, {judged} judged"
             if prec is not None and prec >= _PRECEDENT_MIN
             else "states like this: insufficient history")
    return "\n".join(L)


# ----------------------------------------------------------------------
# Small shared helpers
# ----------------------------------------------------------------------

def _fire_threshold() -> float:
    from chat.concerns import _AGENT_CONCERN_FIRE_THRESHOLD
    return _AGENT_CONCERN_FIRE_THRESHOLD


def _clip(text: str, n: int) -> str:
    text = (text or '').strip()
    return text if len(text) <= n else text[:n].rstrip() + '…'


def _parse_ts(value: Any) -> Optional[datetime]:
    if not value:
        return None
    try:
        return datetime.fromisoformat(str(value))
    except (TypeError, ValueError):
        return None


def _before(earlier: Any, later: Any) -> bool:
    """True when `earlier` strictly precedes `later`. Accepts ISO strings
    or datetimes; naive values are read as UTC so mixed-source history
    (naive-local turns, aware-UTC autonomy events) still orders."""
    a = earlier if isinstance(earlier, datetime) else _parse_ts(earlier)
    b = later if isinstance(later, datetime) else _parse_ts(later)
    if a is None or b is None:
        return False
    if a.tzinfo is None:
        a = a.replace(tzinfo=timezone.utc)
    if b.tzinfo is None:
        b = b.replace(tzinfo=timezone.utc)
    return a < b


def _dur(seconds: float) -> str:
    """Bare span — '45s', '15m', '4h', '3d'. Callers add 'ago' when the
    span is an age rather than a duration."""
    s = max(0.0, float(seconds))
    if s < 90:
        return f"{int(s)}s"
    if s < 5400:
        return f"{int(s // 60)}m"
    if s < 172800:
        return f"{int(s // 3600)}h"
    return f"{int(s // 86400)}d"


def _ago_s(seconds: Optional[float]) -> str:
    return 'unknown' if seconds is None else f"{_dur(seconds)} ago"


def _ago(stamp: Any, ref: datetime) -> str:
    when = _parse_ts(stamp)
    if when is None:
        return 'never'
    if when.tzinfo is None:
        when = when.replace(tzinfo=timezone.utc)
    return f"{_dur(max(0.0, (ref - when).total_seconds()))} ago"


def _fit(cosine: Optional[float]) -> str:
    if cosine is None:
        return 'unknown'
    return 'near' if cosine >= 0.55 else ('mid' if cosine >= 0.35 else 'far')
