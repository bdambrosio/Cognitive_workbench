#!/usr/bin/env python3
"""
Derived concern model — agent-originated concerns derived from user concerns
and orientation state.

Mirrors UserConcernModel architecture: LLM patch system, named Note persistence,
controlled vocabularies. Triggered during idle OODA ticks and after goal completion.
"""

import json
import logging
import time
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)

# Controlled vocabularies
ORIGIN_VALUES = ('user_concern_derived', 'orientation_derived', 'goal_reflection', 'seed')
STATUS_VALUES = ('surfaced', 'active', 'satisfied', 'abandoned')

NOTE_NAME = '_derived_concerns'
IDLE_COOLDOWN_SECS = 60.0

PATCH_SYSTEM_PROMPT = """\
You maintain a compact model of an autonomous agent's self-derived concerns.

These are concerns the agent chooses to pursue based on its orientation: user concerns \
it wants to proactively address, patterns it notices across goals, or operational issues \
it identifies. They are NOT user concerns — they are the agent's own working priorities \
derived from observing user concerns, goal outcomes, and orientation state.

Given the current derived concern list, user concerns, and a trigger context, \
emit exactly ONE JSON patch operation.

Patch types:
- surface_concern: a new concern the agent has identified (status: surfaced)
- activate_concern: a surfaced concern the agent commits to acting on (status: active)
- satisfy_concern: a concern currently addressed — will revisit after its revisit period (status: satisfied)
- abandon_concern: a concern no longer worth pursuing (status: abandoned)
- no_change: no material change warranted

Rules:
- Emit at most one patch per invocation.
- Prefer activating or satisfying existing concerns over surfacing new ones.
- Only surface when clearly warranted by user concerns or orientation state.
- Concerns should be actionable — something the agent can actually do.
- Link to parent user concern when the derivation is clear.
- weight is a float 0.0-1.0 reflecting the agent's assessment of priority.
- Keep rationale short and concrete.
- Maximum 8 active+surfaced concerns at any time. If at limit, satisfy or abandon before surfacing.
- "satisfied" is NOT terminal — satisfied concerns automatically return to "active" after their \
revisit period expires. Use satisfy_concern when a task has adequately addressed a concern for now.
- Include revisit_hours in field_updates when satisfying (default: 24 for most concerns, \
4 for homeostatic/operational concerns). The user can override this.

Controlled vocabularies:
  origin: user_concern_derived, orientation_derived, goal_reflection, seed
  status: surfaced, active, satisfied, abandoned

Respond with ONLY a JSON object. No markdown fences, no commentary."""

SURFACE_CONCERN_SCHEMA = """\
For surface_concern:
{
  "op": "surface_concern",
  "new_concern": {
    "concern_label": "...",
    "concern_description": "1-3 sentences describing what the agent should attend to",
    "weight": 0.0-1.0,
    "origin": "user_concern_derived|orientation_derived|goal_reflection",
    "parent_user_concern_id": "concern_NNN or null",
    "status_rationale": "why this concern was surfaced"
  },
  "why_this_surface": "..."
}"""

ACTIVATE_CONCERN_SCHEMA = """\
For activate_concern:
{
  "op": "activate_concern",
  "concern_id": "dconcern_NNN",
  "field_updates": {
    "weight": ...,
    "status_rationale": "..."
  },
  "why_this_activate": "..."
}"""

SATISFY_CONCERN_SCHEMA = """\
For satisfy_concern:
{
  "op": "satisfy_concern",
  "concern_id": "dconcern_NNN",
  "field_updates": {
    "weight": ...,
    "revisit_hours": 24,
    "status_rationale": "..."
  },
  "why_this_satisfy": "..."
}"""

ABANDON_CONCERN_SCHEMA = """\
For abandon_concern:
{
  "op": "abandon_concern",
  "concern_id": "dconcern_NNN",
  "field_updates": {
    "status_rationale": "..."
  },
  "why_this_abandon": "..."
}"""

NO_CHANGE_SCHEMA = """\
For no_change:
{
  "op": "no_change",
  "why_no_change": "..."
}"""

_MAX_ACTIVE_CONCERNS = 8


class DerivedConcernModel:
    """Incrementally maintained model of agent-derived concerns."""

    def __init__(self, resource_manager: Any, character_name: str,
                 llm_generate: Callable, infospace_executor: Any):
        self.resource_manager = resource_manager
        self.character_name = character_name
        self.llm_generate = llm_generate
        self.infospace_executor = infospace_executor
        self.concerns: List[Dict[str, Any]] = []
        self._concern_counter = 0
        self._last_updated: Optional[str] = None
        self._last_idle_update: float = 0.0
        self._seed_concerns: List[Dict[str, Any]] = []

    def set_seed_concerns(self, seeds: List[Dict[str, Any]]):
        """Set seed concern definitions from character config. Call before load()."""
        self._seed_concerns = seeds or []

    # ── Persistence ───────────────────────────────────────────────────

    def load(self) -> bool:
        """Load concern list from the persisted _derived_concerns Note.

        On first run (no note or empty), injects seed concerns if configured.
        On subsequent runs, re-surfaces any seeded concerns that were abandoned.
        """
        if not self.infospace_executor:
            return False
        try:
            result = self.infospace_executor.execute_action(
                {"type": "load", "target": NOTE_NAME, "out": "$_dcm_tmp"}
            )
            if result.get('status') != 'success' or not result.get('resource_id'):
                logger.info('No derived concerns note found (first run)')
                if self._seed_concerns:
                    self._inject_seeds()
                    self._save()
                return True

            content = self.infospace_executor._get_content(result['resource_id'])
            if not content or not isinstance(content, str) or not content.strip():
                logger.info('Derived concerns note is empty')
                if self._seed_concerns:
                    self._inject_seeds()
                    self._save()
                return True

            data = json.loads(content)
            self.concerns = data.get('concerns', [])
            self._last_updated = data.get('last_updated')
            # Restore counter from highest existing ID
            for c in self.concerns:
                cid = c.get('concern_id', '')
                if cid.startswith('dconcern_'):
                    try:
                        num = int(cid.split('_', 1)[1])
                        if num > self._concern_counter:
                            self._concern_counter = num
                    except (ValueError, IndexError):
                        pass

            # Migrate legacy 'resolved' status to 'satisfied'
            migrated = 0
            for c in self.concerns:
                if c.get('status') == 'resolved':
                    c['status'] = 'satisfied'
                    if not c.get('satisfied_at'):
                        c['satisfied_at'] = c.get('recency', datetime.now().isoformat())
                    if not c.get('revisit_hours'):
                        c['revisit_hours'] = 4.0 if c.get('seeded') else 24.0
                    migrated += 1
                # Ensure all concerns have the new fields
                if 'revisit_hours' not in c:
                    c['revisit_hours'] = 4.0 if c.get('seeded') else 24.0
                if 'satisfied_at' not in c:
                    c['satisfied_at'] = None
            if migrated:
                logger.info(f'Migrated {migrated} resolved concern(s) to satisfied')

            # Inject any configured seeds not already present (upgrade path)
            injected_missing = 0
            if self._seed_concerns:
                existing_labels = {c.get('concern_label') for c in self.concerns
                                   if c.get('seeded')}
                missing = [s for s in self._seed_concerns
                           if s.get('label') not in existing_labels]
                if missing:
                    old_seeds = self._seed_concerns
                    self._seed_concerns = missing
                    self._inject_seeds()
                    self._seed_concerns = old_seeds
                    injected_missing = len(missing)

            if migrated or injected_missing:
                self._save()

            logger.info(f'\u2713 Loaded {len(self.concerns)} derived concerns')
            return True
        except json.JSONDecodeError as e:
            logger.warning(f'Failed to parse derived concerns note: {e}')
            return False
        except Exception as e:
            logger.warning(f'Error loading derived concerns: {e}')
            return False

    def _inject_seeds(self):
        """Create initial derived concerns from seed definitions."""
        for seed in self._seed_concerns:
            self._concern_counter += 1
            concern = {
                'concern_id': f'dconcern_{self._concern_counter}',
                'concern_label': seed.get('label', ''),
                'concern_description': seed.get('description', ''),
                'weight': float(seed.get('weight', 0.3)),
                'origin': 'seed',
                'status': 'active',
                'status_rationale': 'Standing concern from character configuration',
                'parent_user_concern_id': None,
                'category': seed.get('category', ''),
                'seeded': True,
                'revisit_hours': float(seed.get('revisit_hours', 4.0)),
                'satisfied_at': None,
                'recency': datetime.now().isoformat(),
                'touch_count': 0,
                'evidence_refs': ['character_config_seed'],
                'history_summary': '',
            }
            self.concerns.append(concern)
        logger.info(f'Injected {len(self._seed_concerns)} seed concerns')

    def _save(self):
        """Persist the concern list to the _derived_concerns Note."""
        if not self.infospace_executor:
            return
        data = {
            "concerns": self.concerns,
            "last_updated": datetime.now().isoformat(),
            "version": 1,
        }
        content = json.dumps(data, indent=2, ensure_ascii=False)

        result = self.infospace_executor.execute_action(
            {"type": "load", "target": NOTE_NAME, "out": "$_dcm_save"}
        )
        if result.get('status') == 'success' and result.get('resource_id'):
            note_id = result['resource_id']
            success, err = self.resource_manager.update_note_content(note_id, content)
            if not success:
                logger.warning(f'Failed to update derived concerns note: {err}')
        else:
            self.infospace_executor.execute_action(
                {"type": "create-note", "value": content, "name": NOTE_NAME, "out": "$_dcm_save"}
            )
            self.infospace_executor.execute_action(
                {"type": "persist", "target": "$_dcm_save"}
            )
        logger.info(f'\u2713 Saved {len(self.concerns)} derived concerns')

    # ── Revisit expiration ──────────────────────────────────────────

    def check_revisit_expirations(self) -> List[str]:
        """Check satisfied concerns for expired revisit timers. Returns reactivated IDs."""
        reactivated = []
        now = datetime.now()
        for c in self.concerns:
            if c.get('status') != 'satisfied':
                continue
            satisfied_at = c.get('satisfied_at')
            revisit_hours = c.get('revisit_hours')
            if not satisfied_at or not revisit_hours:
                continue
            try:
                sat_dt = datetime.fromisoformat(satisfied_at)
                elapsed_hours = (now - sat_dt).total_seconds() / 3600.0
                if elapsed_hours >= float(revisit_hours):
                    c['status'] = 'active'
                    c['satisfied_at'] = None
                    c['status_rationale'] = f'Revisit period ({revisit_hours}h) expired — reactivated'
                    c['recency'] = now.isoformat()
                    reactivated.append(c.get('concern_id', ''))
                    logger.info(
                        f'Concern {c.get("concern_id")} reactivated after '
                        f'{elapsed_hours:.1f}h (revisit_hours={revisit_hours})')
            except (ValueError, TypeError):
                continue
        if reactivated:
            self._save()
        return reactivated

    # ── Trigger: idle tick ────────────────────────────────────────────

    def update_from_idle_tick(self, living_state, user_concerns: List[Dict[str, Any]]) -> None:
        """Update derived concerns during idle OODA ticks. Rate-limited."""
        # Rate limit
        now = time.monotonic()
        if now - self._last_idle_update < IDLE_COOLDOWN_SECS:
            return
        # Skip if no user concerns to derive from
        if not user_concerns:
            return

        self._last_idle_update = now

        # Build context from living state
        orientation_summary = self._build_orientation_summary(living_state)
        timestamp = datetime.now().isoformat()

        interaction_text = (
            f"Idle orientation tick.\n"
            f"Agent is not currently executing a goal.\n"
            f"{orientation_summary}\n"
            f"Timestamp: {timestamp}"
        )
        evidence_ref = f"idle_tick:{timestamp}"
        self._run_patch(interaction_text, user_concerns, evidence_ref)

    # ── Trigger: goal completion ──────────────────────────────────────

    def update_from_goal_completion(
        self,
        goal_statement: str,
        outcome_summary: str,
        goal_id: str,
        success: bool,
        user_concerns: List[Dict[str, Any]],
        living_state=None,
    ) -> None:
        """Update derived concerns after a goal completes."""
        if not goal_statement:
            return

        timestamp = datetime.now().isoformat()
        status_word = "succeeded" if success else "failed"
        orientation_summary = self._build_orientation_summary(living_state)

        parts = [
            f"Goal completion ({status_word}):",
            f"Goal: {goal_statement}",
            f"Outcome: {outcome_summary or '(no summary)'}",
            orientation_summary,
            f"Timestamp: {timestamp}",
        ]
        interaction_text = "\n".join(parts)
        evidence_ref = f"goal:{goal_id}:{timestamp}"
        self._run_patch(interaction_text, user_concerns, evidence_ref)

    # ── Core patch logic ──────────────────────────────────────────────

    def _run_patch(self, interaction_text: str, user_concerns: List[Dict[str, Any]],
                   evidence_ref: str):
        """Run one LLM call to produce a patch, then apply it."""
        concerns_json = json.dumps(self.concerns, indent=2, ensure_ascii=False) if self.concerns else "[]"
        uc_json = json.dumps(
            [{"concern_id": c.get("concern_id"), "concern_label": c.get("concern_label"),
              "status": c.get("status"), "weight": c.get("weight")}
             for c in user_concerns[:10]],
            indent=2, ensure_ascii=False,
        ) if user_concerns else "[]"

        # Count active+surfaced to inform the LLM about capacity
        active_count = sum(1 for c in self.concerns
                           if c.get('status') in ('surfaced', 'active'))

        user_prompt = (
            f"## Current derived concerns ({active_count}/{_MAX_ACTIVE_CONCERNS} active+surfaced)\n"
            f"{concerns_json}\n\n"
            f"## Active user concerns (for reference — do not duplicate these)\n"
            f"{uc_json}\n\n"
            f"## Trigger context\n{interaction_text}\n\n"
            f"## Schemas\n{SURFACE_CONCERN_SCHEMA}\n{ACTIVATE_CONCERN_SCHEMA}\n"
            f"{SATISFY_CONCERN_SCHEMA}\n{ABANDON_CONCERN_SCHEMA}\n{NO_CHANGE_SCHEMA}\n\n"
            f"Analyze the context and emit one JSON patch.\n</end>"
        )

        try:
            result = self.llm_generate(
                messages=[PATCH_SYSTEM_PROMPT, user_prompt],
                max_tokens=1512,
                temperature=0.3,
                is_json=True,
                stops=['</end>'],
            )
            if not result.success or not result.text:
                logger.warning(f'Derived concern LLM call failed: {getattr(result, "error", "unknown")}')
                return

            patch = self._parse_patch(result.text)
            if not patch:
                return

            # Skip save for no-op patches
            if patch['op'] == 'no_change':
                self._apply_patch(patch, evidence_ref)
                return

            self._apply_patch(patch, evidence_ref)
            self._save()

        except Exception as e:
            logger.error(f'Error in derived concern patch: {e}')

    def _parse_patch(self, text) -> Optional[Dict[str, Any]]:
        """Parse LLM response into a patch dict."""
        if isinstance(text, dict):
            patch = text
        else:
            text = str(text).strip()
            if text.startswith('```'):
                lines = text.split('\n')
                lines = [l for l in lines if not l.strip().startswith('```')]
                text = '\n'.join(lines).strip()
            try:
                patch = json.loads(text)
            except json.JSONDecodeError as e:
                logger.warning(f'Failed to parse derived concern patch JSON: {e} — {text[:200]}')
                return None

        if not isinstance(patch, dict) or 'op' not in patch:
            logger.warning(f'Derived concern patch missing "op" field: {str(patch)[:200]}')
            return None
        valid_ops = ('surface_concern', 'activate_concern', 'satisfy_concern', 'abandon_concern', 'no_change',
                     'resolve_concern')  # resolve_concern accepted for backward compat, mapped to satisfy
        if patch['op'] not in valid_ops:
            logger.warning(f'Unknown derived concern patch op: {patch["op"]}')
            return None
        return patch

    def _apply_patch(self, patch: Dict[str, Any], evidence_ref: str):
        """Apply a single patch to the concern list."""
        op = patch['op']

        if op == 'no_change':
            logger.info(f'Derived concern model: no_change \u2014 {patch.get("why_no_change", "")}')
            return

        if op == 'surface_concern':
            new_concern = patch.get('new_concern', {})
            if not new_concern or not new_concern.get('concern_label'):
                logger.warning('surface_concern patch missing new_concern or concern_label')
                return
            # Check capacity
            active_count = sum(1 for c in self.concerns
                               if c.get('status') in ('surfaced', 'active'))
            if active_count >= _MAX_ACTIVE_CONCERNS:
                logger.warning(f'Derived concern capacity reached ({active_count}/{_MAX_ACTIVE_CONCERNS}), skipping surface')
                return
            self._concern_counter += 1
            concern_id = f'dconcern_{self._concern_counter}'
            origin = new_concern.get('origin', 'orientation_derived')
            if origin not in ORIGIN_VALUES:
                origin = 'orientation_derived'
            entry = {
                'concern_id': concern_id,
                'concern_label': new_concern.get('concern_label', ''),
                'concern_description': new_concern.get('concern_description', ''),
                'weight': float(new_concern.get('weight', 0.5)),
                'origin': origin,
                'parent_user_concern_id': new_concern.get('parent_user_concern_id'),
                'status': 'surfaced',
                'status_rationale': new_concern.get('status_rationale', ''),
                'recency': datetime.now().isoformat(),
                'touch_count': 1,
                'evidence_refs': [evidence_ref],
                'history_summary': '',
            }
            self.concerns.append(entry)
            logger.info(f'Derived concern model: +{concern_id} "{entry["concern_label"]}" \u2014 {patch.get("why_this_surface", "")}')
            return

        # activate, resolve, abandon — find the target
        concern_id = patch.get('concern_id', '')
        target = None
        for c in self.concerns:
            if c.get('concern_id') == concern_id:
                target = c
                break
        if not target:
            logger.warning(f'Derived concern {concern_id} not found for {op}')
            return

        updates = patch.get('field_updates', {})

        if op == 'activate_concern':
            updates['status'] = 'active'
            target['satisfied_at'] = None  # Clear any previous satisfaction
            self._merge_updates(target, updates, evidence_ref)
            logger.info(f'Derived concern model: \u25b6{concern_id} activated \u2014 {patch.get("why_this_activate", "")}')

        elif op in ('satisfy_concern', 'resolve_concern'):
            # resolve_concern is accepted for backward compat, treated as satisfy
            updates['status'] = 'satisfied'
            revisit = float(updates.pop('revisit_hours', 0) or target.get('revisit_hours', 24))
            updates['revisit_hours'] = revisit
            self._merge_updates(target, updates, evidence_ref)
            target['satisfied_at'] = datetime.now().isoformat()
            why_key = 'why_this_satisfy' if op == 'satisfy_concern' else 'why_this_resolve'
            logger.info(f'Derived concern model: \u2713{concern_id} satisfied (revisit in {revisit}h) \u2014 {patch.get(why_key, "")}')

        elif op == 'abandon_concern':
            updates['status'] = 'abandoned'
            self._merge_updates(target, updates, evidence_ref)
            logger.info(f'Derived concern model: \u2717{concern_id} abandoned \u2014 {patch.get("why_this_abandon", "")}')

    def _merge_updates(self, target: Dict[str, Any], updates: Dict[str, Any],
                       evidence_ref: str):
        """Merge field updates into an existing concern entry."""
        for field in ('weight', 'status', 'status_rationale', 'concern_description',
                      'concern_label', 'origin', 'parent_user_concern_id', 'history_summary',
                      'category', 'revisit_hours'):
            if field in updates:
                if field == 'weight':
                    target[field] = float(updates[field])
                elif field == 'status' and updates[field] not in STATUS_VALUES:
                    continue  # reject invalid status
                else:
                    target[field] = updates[field]

        target['recency'] = datetime.now().isoformat()
        target['touch_count'] = target.get('touch_count', 0) + 1

        refs = target.get('evidence_refs', [])
        refs.append(evidence_ref)
        target['evidence_refs'] = refs[-10:]

    # ── Read access ───────────────────────────────────────────────────

    def get_concerns(self, active_only: bool = False) -> List[Dict[str, Any]]:
        """Return current concern list, optionally filtered to active+surfaced."""
        if not active_only:
            return list(self.concerns)
        return [c for c in self.concerns if c.get('status') in ('surfaced', 'active')]

    def get_concerns_for_evaluator(self, max_count: int = 5) -> List[Dict[str, str]]:
        """Return derived concerns in character evaluator format.

        Returns top concerns by weight, formatted as:
        [{"id": "dconcern_1", "label": "...", "description": "..."}]

        This format matches DEFAULT_CHARACTER_CONCERNS and can be concatenated with it.
        """
        active = [c for c in self.concerns if c.get('status') in ('surfaced', 'active')]
        active.sort(key=lambda c: c.get('weight', 0), reverse=True)
        return [
            {
                "id": c.get("concern_id", ""),
                "label": c.get("concern_label", ""),
                "description": (
                    f"Agent-derived concern: {c.get('concern_description', '')} "
                    f"[{c.get('status', '?')}, origin: {c.get('origin', '?')}]"
                ),
            }
            for c in active[:max_count]
        ]

    # ── Helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _build_orientation_summary(living_state) -> str:
        """Build compact orientation summary from living state."""
        if living_state is None:
            return "Orientation: (no living state available)"
        parts = ["Orientation:"]

        activations = getattr(living_state, 'concern_activations', [])
        if activations:
            top = activations[:3]
            items = [f"{a['id']}={a.get('activation', 0):.2f}({a.get('trend', '?')})" for a in top]
            parts.append(f"  Top concerns: {', '.join(items)}")

        uc_snapshot = getattr(living_state, 'user_concern_snapshot', [])
        if uc_snapshot:
            if isinstance(uc_snapshot, list):
                # New format: list of concern dicts
                status_counts = {}
                for c in uc_snapshot:
                    s = c.get('status', '?')
                    status_counts[s] = status_counts.get(s, 0) + 1
                items = [f"{count} {status}" for status, count in sorted(status_counts.items())]
            elif isinstance(uc_snapshot, dict):
                # Legacy format: {status: count}
                items = [f"{count} {status}" for status, count in uc_snapshot.items()]
            else:
                items = []
            if items:
                parts.append(f"  User concerns: {', '.join(items)}")

        fg = getattr(living_state, 'foregrounded_goal_id', None)
        if fg:
            parts.append(f"  Foregrounded goal: {fg}")

        transitions = list(getattr(living_state, 'transitions', []))
        if transitions:
            parts.append(f"  Recent: {transitions[-1]}")

        return "\n".join(parts)
