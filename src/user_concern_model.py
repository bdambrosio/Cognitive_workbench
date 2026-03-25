#!/usr/bin/env python3
"""
User concern model — tracks active and recent user concerns.

Maintained as a persistent Note (_user_concerns) containing a JSON concern list.
Updated incrementally on conversation end and goal completion via single-patch
LLM calls.
"""

import json
import logging
import time
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)

# Controlled vocabularies
STANCE_VALUES = (
    'exploratory', 'skeptical', 'committed', 'concerned',
    'frustrated', 'enthusiastic', 'ambivalent',
)
END_DISPOSITION_VALUES = (
    'resolved', 'clearer', 'still_uncertain', 'still_concerned',
    'more_interested', 'blocked', 'deferred', 'satisfied', 'dissatisfied',
)
STATUS_VALUES = ('open', 'closed')

PATCH_SYSTEM_PROMPT = """\
You maintain a compact model of a user's active concerns.

Given the current concern list and a completed interaction, emit exactly ONE JSON patch operation.

Patch types:
- add_concern: new concern not matched by existing entries
- update_concern: materially advances, clarifies, intensifies, or shifts one known concern
- close_concern: concern appears resolved
- no_change: no material concern-level change

Rules:
- Emit at most one patch per invocation.
- Prefer updating an existing concern over adding a new one.
- Only add when clearly warranted (not incidental mentions).
- Only close when closure is reasonably explicit.
- Do not infer personality traits or deep motives.
- Keep rationale short and concrete.
- Concerns are specific (not bare topics). Good: "concern about inverter startup sag". Bad: "solar".
- weight is a float 0.0-1.0 reflecting importance and activation (not mere frequency).

Controlled vocabularies:
  stance: exploratory, skeptical, committed, concerned, frustrated, enthusiastic, ambivalent
  end_disposition: resolved, clearer, still_uncertain, still_concerned, more_interested, blocked, deferred, satisfied, dissatisfied
  status: open, closed

Respond with ONLY a JSON object. No markdown fences, no commentary."""

ADD_CONCERN_SCHEMA = """\
For add_concern:
{
  "op": "add_concern",
  "new_concern": {
    "concern_label": "...",
    "concern_description": "1-3 sentences",
    "weight": 0.0-1.0,
    "stance": "...",
    "stance_rationale": "...",
    "end_disposition": "...",
    "end_disposition_rationale": "...",
    "status": "open",
    "status_rationale": "...",
    "history_summary": "..."
  },
  "why_this_add": "..."
}"""

UPDATE_CONCERN_SCHEMA = """\
For update_concern:
{
  "op": "update_concern",
  "concern_id": "concern_NNN",
  "field_updates": {
    "weight": ...,
    "stance": "...",
    "end_disposition": "...",
    "status": "...",
    ...any fields to update...
  },
  "why_this_update": "..."
}"""

CLOSE_CONCERN_SCHEMA = """\
For close_concern:
{
  "op": "close_concern",
  "concern_id": "concern_NNN",
  "closing_updates": {
    "status": "closed",
    "end_disposition": "...",
    "weight": ...,
    ...
  },
  "why_this_close": "..."
}"""

NO_CHANGE_SCHEMA = """\
For no_change:
{
  "op": "no_change",
  "why_no_change": "..."
}"""

NOTE_NAME = '_user_concerns'


class UserConcernModel:
    """Incrementally maintained model of user concerns."""

    def __init__(self, resource_manager: Any, character_name: str,
                 llm_generate: Callable, infospace_executor: Any):
        self.resource_manager = resource_manager
        self.character_name = character_name
        self.llm_generate = llm_generate
        self.infospace_executor = infospace_executor
        self.concerns: List[Dict[str, Any]] = []
        self._concern_counter = 0
        self._last_updated: Optional[str] = None

    # ── Persistence ───────────────────────────────────────────────────

    def load(self) -> bool:
        """Load concern list from the persisted _user_concerns Note."""
        if not self.infospace_executor:
            return False
        try:
            result = self.infospace_executor.execute_action(
                {"type": "load", "target": NOTE_NAME, "out": "$_ucm_tmp"}
            )
            if result.get('status') != 'success' or not result.get('resource_id'):
                logger.info('No user concerns note found (first run)')
                return True  # Not an error

            content = self.infospace_executor._get_content(result['resource_id'])
            if not content or not isinstance(content, str) or not content.strip():
                logger.info('User concerns note is empty')
                return True

            data = json.loads(content)
            self.concerns = data.get('concerns', [])
            self._last_updated = data.get('last_updated')
            # Restore counter from highest existing ID
            for c in self.concerns:
                cid = c.get('concern_id', '')
                if cid.startswith('concern_'):
                    try:
                        num = int(cid.split('_', 1)[1])
                        if num > self._concern_counter:
                            self._concern_counter = num
                    except (ValueError, IndexError):
                        pass
            # Migrate old statuses to simplified 2-value model
            migrated = 0
            for c in self.concerns:
                old_status = c.get('status', '')
                if old_status in ('ongoing', 'reopened'):
                    c['status'] = 'open'
                    migrated += 1
                elif old_status == 'dormant':
                    c['status'] = 'closed'
                    migrated += 1
            if migrated:
                logger.info(f'Migrated {migrated} concerns to new status model (open/closed)')
            logger.info(f'✓ Loaded {len(self.concerns)} user concerns')
            return True
        except json.JSONDecodeError as e:
            logger.warning(f'Failed to parse user concerns note: {e}')
            return False
        except Exception as e:
            logger.warning(f'Error loading user concerns: {e}')
            return False

    def _save(self):
        """Persist the concern list to the _user_concerns Note."""
        if not self.infospace_executor:
            return
        data = {
            "concerns": self.concerns,
            "last_updated": datetime.now().isoformat(),
            "version": 1,
        }
        content = json.dumps(data, indent=2, ensure_ascii=False)

        # Use the same create-or-update pattern as _write_named_note
        result = self.infospace_executor.execute_action(
            {"type": "load", "target": NOTE_NAME, "out": "$_ucm_save"}
        )
        if result.get('status') == 'success' and result.get('resource_id'):
            note_id = result['resource_id']
            success, err = self.resource_manager.update_note_content(note_id, content)
            if not success:
                logger.warning(f'Failed to update user concerns note: {err}')
        else:
            self.infospace_executor.execute_action(
                {"type": "create-note", "value": content, "name": NOTE_NAME, "out": "$_ucm_save"}
            )
            self.infospace_executor.execute_action(
                {"type": "persist", "target": "$_ucm_save"}
            )
        logger.info(f'✓ Saved {len(self.concerns)} user concerns')

    # ── Trigger: conversation end ─────────────────────────────────────

    def update_from_conversation(self, conversation_summary: str,
                                  dialog_id: str, entity: str):
        """Update concern model after a conversation ends."""
        if not conversation_summary or not conversation_summary.strip():
            return
        timestamp = datetime.now().isoformat()
        interaction_text = (
            f"Completed conversation with {entity} (dialog {dialog_id}):\n"
            f"{conversation_summary}\n"
            f"Timestamp: {timestamp}"
        )
        evidence_ref = f"conversation:{dialog_id}:{timestamp}"
        self._run_patch(interaction_text, evidence_ref)

    # ── Trigger: goal completion ──────────────────────────────────────

    def update_from_goal_completion(self, goal_statement: str,
                                     outcome_summary: str, goal_id: str,
                                     success: bool,
                                     result_artifact: str = ''):
        """Update concern model after a goal completes."""
        if not goal_statement:
            return
        timestamp = datetime.now().isoformat()
        status_word = "succeeded" if success else "failed"
        parts = [
            f"Completed goal ({status_word}):",
            f"Goal: {goal_statement}",
            f"Outcome: {outcome_summary or '(no summary)'}",
        ]
        if result_artifact:
            parts.append(f"Result artifact: {result_artifact}")
        parts.append(f"Timestamp: {timestamp}")
        interaction_text = "\n".join(parts)
        evidence_ref = f"goal:{goal_id}:{timestamp}"
        self._run_patch(interaction_text, evidence_ref)

    # ── Core patch logic ──────────────────────────────────────────────

    def _run_patch(self, interaction_text: str, evidence_ref: str):
        """Run one LLM call to produce a patch, then apply it."""
        concerns_json = json.dumps(self.concerns, indent=2, ensure_ascii=False) if self.concerns else "[]"

        user_prompt = (
            f"## Current concern list\n{concerns_json}\n\n"
            f"## Completed interaction\n{interaction_text}\n\n"
            f"## Schemas\n{ADD_CONCERN_SCHEMA}\n{UPDATE_CONCERN_SCHEMA}\n"
            f"{CLOSE_CONCERN_SCHEMA}\n{NO_CHANGE_SCHEMA}\n\n"
            f"Analyze the interaction and emit one JSON patch."
        )

        try:
            result = self.llm_generate(
                messages=[PATCH_SYSTEM_PROMPT, user_prompt],
                max_tokens=3000,
                temperature=0.3,
                is_json=True,
            )
            if not result.success or not result.text:
                logger.warning(f'Concern model LLM call failed: {getattr(result, "error", "unknown")}')
                return

            patch = self._parse_patch(result.text)
            if not patch:
                return

            self._apply_patch(patch, evidence_ref)
            self._save()

        except Exception as e:
            logger.error(f'Error in concern model patch: {e}')

    def _parse_patch(self, text) -> Optional[Dict[str, Any]]:
        """Parse LLM response into a patch dict. Accepts str or already-parsed dict."""
        # Some providers return a dict directly when is_json=True
        if isinstance(text, dict):
            patch = text
        else:
            text = str(text).strip()
            # Strip markdown fences if present
            if text.startswith('```'):
                lines = text.split('\n')
                lines = [l for l in lines if not l.strip().startswith('```')]
                text = '\n'.join(lines).strip()
            try:
                patch = json.loads(text)
            except json.JSONDecodeError as e:
                logger.warning(f'Failed to parse concern patch JSON: {e} — {text[:200]}')
                return None

        if not isinstance(patch, dict) or 'op' not in patch:
            logger.warning(f'Concern patch missing "op" field: {str(patch)[:200]}')
            return None
        if patch['op'] not in ('add_concern', 'update_concern', 'close_concern', 'no_change'):
            logger.warning(f'Unknown concern patch op: {patch["op"]}')
            return None
        return patch

    def _apply_patch(self, patch: Dict[str, Any], evidence_ref: str):
        """Apply a single patch to the concern list."""
        op = patch['op']

        if op == 'no_change':
            logger.info(f'Concern model: no_change — {patch.get("why_no_change", "")}')
            return

        if op == 'add_concern':
            new_concern = patch.get('new_concern', {})
            if not new_concern or not new_concern.get('concern_label'):
                logger.warning('add_concern patch missing new_concern or concern_label')
                return
            self._concern_counter += 1
            concern_id = f'concern_{self._concern_counter}'
            entry = {
                'concern_id': concern_id,
                'concern_label': new_concern.get('concern_label', ''),
                'concern_description': new_concern.get('concern_description', ''),
                'weight': float(new_concern.get('weight', 0.5)),
                'recency': datetime.now().isoformat(),
                'touch_count': 1,
                'stance': new_concern.get('stance', 'exploratory'),
                'stance_rationale': new_concern.get('stance_rationale', ''),
                'end_disposition': new_concern.get('end_disposition', 'still_uncertain'),
                'end_disposition_rationale': new_concern.get('end_disposition_rationale', ''),
                'status': new_concern.get('status', 'open'),
                'status_rationale': new_concern.get('status_rationale', ''),
                'evidence_refs': [evidence_ref],
                'history_summary': new_concern.get('history_summary', ''),
            }
            self.concerns.append(entry)
            logger.info(f'Concern model: +{concern_id} "{entry["concern_label"]}" — {patch.get("why_this_add", "")}')
            return

        # update_concern or close_concern — find the target
        concern_id = patch.get('concern_id', '')
        target = None
        for c in self.concerns:
            if c.get('concern_id') == concern_id:
                target = c
                break
        if not target:
            logger.warning(f'Concern {concern_id} not found for {op}')
            return

        if op == 'update_concern':
            updates = patch.get('field_updates', {})
            self._merge_updates(target, updates, evidence_ref)
            logger.info(f'Concern model: ~{concern_id} — {patch.get("why_this_update", "")}')

        elif op == 'close_concern':
            updates = patch.get('closing_updates', {})
            updates.setdefault('status', 'closed')
            self._merge_updates(target, updates, evidence_ref)
            logger.info(f'Concern model: x{concern_id} closed — {patch.get("why_this_close", "")}')

    def _merge_updates(self, target: Dict[str, Any], updates: Dict[str, Any],
                       evidence_ref: str):
        """Merge field updates into an existing concern entry."""
        # Direct field updates
        for field in ('weight', 'stance', 'stance_rationale',
                      'end_disposition', 'end_disposition_rationale',
                      'status', 'status_rationale', 'concern_description',
                      'concern_label', 'history_summary'):
            if field in updates:
                if field == 'weight':
                    target[field] = float(updates[field])
                else:
                    target[field] = updates[field]
                # Normalize old status values the LLM might still emit
                if field == 'status':
                    if target[field] in ('ongoing', 'reopened'):
                        target[field] = 'open'
                    elif target[field] == 'dormant':
                        target[field] = 'closed'

        # Always update recency and touch_count
        target['recency'] = datetime.now().isoformat()
        target['touch_count'] = target.get('touch_count', 0) + 1

        # Append evidence ref
        refs = target.get('evidence_refs', [])
        refs.append(evidence_ref)
        # Keep last 10 refs
        target['evidence_refs'] = refs[-10:]

    # ── Read access ───────────────────────────────────────────────────

    def get_concerns(self, active_only: bool = False) -> List[Dict[str, Any]]:
        """Return current concern list, optionally filtered to active concerns."""
        if not active_only:
            return list(self.concerns)
        return [c for c in self.concerns if c.get('status') == 'open']
