"""Zenoh wiring — session open/close, sense_data ingestion, say
publication, and the queryable handlers behind the resource browser /
CLI (resources, concerns, status, recall, external repo). ZenohMixin for
ChatLoop, moved verbatim from chat_loop.py in the 2026-06 mixin
refactor."""

from __future__ import annotations

import json
import logging
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from chat.concerns import _snap_rhythm_hours  # noqa: E402

logger = logging.getLogger('chat_loop')


class ZenohMixin:
    """Mixin for ChatLoop — moved verbatim from chat_loop.py."""

    # ------------------------------------------------------------------
    # Zenoh wiring
    # ------------------------------------------------------------------

    def _open_zenoh(self) -> None:
        import zenoh
        from utils.zenoh_utils import make_localhost_config

        self._zenoh_session = zenoh.open(make_localhost_config())
        self._affect.attach_session(self._zenoh_session)
        self._canvas.attach_session(self._zenoh_session)
        self._action_pub = self._zenoh_session.declare_publisher(
            f"cognitive/{self.character_name}/action"
        )
        self._sense_sub = self._zenoh_session.declare_subscriber(
            f"cognitive/{self.character_name}/sense_data",
            self._on_sense_data,
        )

        # Resource queryables for resource_browser. Chat mode now exposes
        # Notes, Collections, AND concerns (graph still executive-only).
        self._resources_list_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/resources",
            self._handle_resources_list_query,
        )
        self._resource_remove_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/resource/remove/*",
            self._handle_resource_remove_query,
        )
        self._resource_update_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/resource/update/*",
            self._handle_resource_update_query,
        )
        # Single-chunk wildcard — does not match resource/remove/* or
        # resource/update/* (those have an extra path segment).
        self._resource_by_id_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/resource/*",
            self._handle_resource_by_id_query,
        )
        # Concerns queryables. The browser frontend already expects the
        # executive's shape (user_concerns + derived_concerns + activations);
        # _handle_concerns_query adapts chat's flat concerns collection to
        # that shape so the existing UI works unchanged.
        self._concerns_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/concerns",
            self._handle_concerns_query,
        )
        self._concern_manage_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/control/concern_manage",
            self._handle_concern_manage_query,
        )
        # Direct subagent invocation — bypasses Jill's ReAct loop. Used by
        # `/recall <query>` in the CLI to test/inspect the active-recall
        # subagent without spending a full Jill turn.
        self._remember_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/recall",
            self._handle_remember_query,
        )
        # External-repo binding control. CLI emits set / clear / get;
        # response carries the resolved path or an error.
        self._external_repo_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/control/external_repo",
            self._handle_external_repo_query,
        )
        # Status (/status): is Jill ready for new input, currently
        # processing a turn, or running autonomous work?
        self._status_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/status",
            self._handle_status_query,
        )

        logger.info(
            f"[{self.character_name}] chat zenoh ready "
            f"(in=cognitive/{self.character_name}/sense_data, "
            f"out=cognitive/{self.character_name}/action, "
            f"backend={self.backend.server}@{self.backend.base_url}, "
            f"discourse={self.discourse_enabled}, orientation={self.orientation_enabled})"
        )

    def _on_sense_data(self, sample) -> None:
        try:
            raw = bytes(sample.payload).decode('utf-8', errors='replace')
            envelope = json.loads(raw)
        except Exception as e:
            logger.warning(f"[{self.character_name}] sense_data decode failed: {e}")
            return

        if envelope.get('mode') != 'text':
            return

        content = envelope.get('content')
        source = "User"
        text = ""
        close = False
        image_url: Optional[str] = None
        if isinstance(content, str):
            try:
                inner = json.loads(content)
                source = inner.get('source', source)
                text = inner.get('text', '')
                close = bool(inner.get('close', False))
                img = inner.get('image')
                if isinstance(img, dict):
                    url = img.get('url')
                    if isinstance(url, str) and url:
                        image_url = url
            except Exception:
                text = content
        elif isinstance(content, dict):
            source = content.get('source', source)
            text = content.get('text', '')
            close = bool(content.get('close', False))
            img = content.get('image')
            if isinstance(img, dict):
                url = img.get('url')
                if isinstance(url, str) and url:
                    image_url = url

        # Sensor dispatch: source is e.g. 'sensor:tick'. Recognized sensors
        # become typed events on the inbox; unknown sensors fall through to
        # the empty-text drop below.
        if isinstance(source, str) and source == 'sensor:tick':
            self._inbox.put({'kind': 'tick'})
            return

        if not text and not close and not image_url:
            return

        msg: Dict[str, Any] = {'kind': 'user', 'source': source, 'text': text, 'close': close}
        if image_url:
            msg['image_url'] = image_url
        self._inbox.put(msg)

    def _publish_say(self, text: str) -> None:
        if not self._action_pub:
            return
        payload = {
            'type': 'say',
            'source': 'assistant',
            'text': text,
            'timestamp': datetime.now(timezone.utc).isoformat(),
        }
        try:
            self._action_pub.put(json.dumps(payload).encode('utf-8'))
        except Exception as e:
            logger.warning(f"[{self.character_name}] publish failed: {e}")

    # ------------------------------------------------------------------
    # Resource queryable handlers (chat-mode subset of executive_node)
    # ------------------------------------------------------------------

    @staticmethod
    def _json_safe_resource(resource: Dict[str, Any]) -> Dict[str, Any]:
        rc = resource.copy()
        if 'type' in rc:
            rc['type'] = str(rc['type'])
        if 'location' in rc and isinstance(rc['location'], tuple):
            rc['location'] = list(rc['location'])
        if 'properties' in rc and isinstance(rc['properties'], dict):
            props = rc['properties'].copy()
            for k, v in props.items():
                if isinstance(v, datetime):
                    props[k] = v.isoformat()
            rc['properties'] = props
        return rc

    def _reply(self, query, payload: Dict[str, Any]) -> None:
        try:
            query.reply(query.key_expr, json.dumps(payload).encode('utf-8'))
        except Exception as e:
            logger.warning(f"[{self.character_name}] query reply failed: {e}")

    def _handle_resources_list_query(self, query) -> None:
        try:
            resources = self.resource_manager.get_resource_list()
            self._reply(query, {
                'success': True,
                'resources': [self._json_safe_resource(r) for r in resources],
            })
        except Exception as e:
            logger.error(f"[{self.character_name}] resources list query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_resource_by_id_query(self, query) -> None:
        try:
            resource_id = str(query.key_expr).split('/')[-1]
            r = self.resource_manager.get_resource(resource_id)
            if not r:
                self._reply(query, {'success': False, 'error': f'Resource {resource_id} not found'})
                return
            self._reply(query, {'success': True, 'resource': self._json_safe_resource(r)})
        except Exception as e:
            logger.error(f"[{self.character_name}] resource by id query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_resource_remove_query(self, query) -> None:
        try:
            resource_id = str(query.key_expr).split('/')[-1]
            success, err = self.resource_manager.delete_resource(resource_id)
            if success:
                self._reply(query, {'success': True, 'message': f'Resource {resource_id} deleted'})
                self._persist_to_disk()
            else:
                self._reply(query, {'success': False, 'error': err or f'Failed to delete {resource_id}'})
        except Exception as e:
            logger.error(f"[{self.character_name}] resource remove query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_resource_update_query(self, query) -> None:
        try:
            resource_id = str(query.key_expr).split('/')[-1]
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            new_content = params.get('content', '')
            success, err = self.resource_manager.update_note_content(resource_id, new_content)
            if success:
                self._reply(query, {'success': True, 'message': f'Note {resource_id} updated'})
                self._persist_to_disk()
            else:
                self._reply(query, {'success': False, 'error': err or f'Failed to update {resource_id}'})
        except Exception as e:
            logger.error(f"[{self.character_name}] resource update query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_remember_query(self, query) -> None:
        """Direct invocation of the active-recall subagent. Payload:
        {"query": "<natural-language question>"}. Returns {"success": bool,
        "answer": "<synthesized answer>", "trace_dir": "<path>"}. Bypasses
        Jill's ReAct loop — used by `/recall` in the CLI for test/inspect."""
        try:
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            q_text = str(params.get('query', '') or '').strip()
            if not q_text:
                self._reply(query, {'success': False, 'error': 'empty query'})
                return
            answer = self._run_remember(q_text)
            self._reply(query, {
                'success': True,
                'answer': answer,
                'trace_dir': str(self._subagent_traces_dir()),
            })
        except Exception as e:
            logger.error(f"[{self.character_name}] remember query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_status_query(self, query) -> None:
        """Report whether Jill is ready for new input. Payload is empty
        (or {}). Response shape:
          {
            success: True,
            ready: bool,                # True iff no turn currently in flight
            action: str,                # human-readable description
            current_turn: dict | null,  # kind/source/text_preview/started_at
            post_turn_busy: bool,       # discourse + reflection still running
            inbox_backlog: int,         # queued user/tick items
            last_reply_at: str | null,  # ISO timestamp of last published reply
            character: str,
          }
        ready=True does NOT require post-turn reflection to have finished —
        Jill accepts new input while reflection runs in parallel. The
        /status caller can render post_turn_busy as informational context."""
        try:
            ct = self._current_turn  # snapshot the dict reference
            ready = ct is None
            if ct is not None:
                kind = ct.get('kind')
                src = ct.get('source', '?')
                if kind == 'autonomous':
                    cid = ct.get('autonomous_concern_id') or '(no concern_id)'
                    action = f"running autonomous turn ({cid})"
                else:
                    action = f"processing user input from {src}"
            elif self._post_turn_busy:
                action = "idle (post-turn reflection still running)"
            else:
                action = "idle"
            try:
                backlog = self._inbox.qsize()
            except Exception:
                backlog = 0
            # Canonical log directory. All in-process loggers and
            # subprocess loggers (resource_browser, fastapi_action_display,
            # discourse, llm_api, executive_node via templates) are pinned
            # to <repo>/logs/ — see launcher.py / discourse.py / etc.
            # Single line, one place to look.
            log_dir = str(
                Path(__file__).resolve().parent.parent.parent / 'logs')
            self._reply(query, {
                'success': True,
                'ready': ready,
                'action': action,
                'current_turn': ct,
                'post_turn_busy': self._post_turn_busy,
                'inbox_backlog': backlog,
                'last_reply_at': self._last_reply_at,
                'character': self.character_name,
                'log_dir': log_dir,
                'backend': {
                    'server': self.backend.server,
                    'base_url': self.backend.base_url,
                    'model': self.backend.model,
                },
            })
        except Exception as e:
            logger.error(f"[{self.character_name}] status query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_external_repo_query(self, query) -> None:
        """Bind / unbind / inspect the external-repo binding for this
        chat session. Payload: {"action": "set|clear|get", "path": "..."}.
        Responses:
          set  → {success, path}                  on success
                 {success: false, error: "..."}  on failure (e.g. bad path)
          clear → {success, was_bound: bool, path: <prior or null>}
          get   → {success, path: <str or null>}"""
        try:
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            action = str(params.get('action') or '').strip().lower()
            if action == 'set':
                path_arg = str(params.get('path') or '').strip()
                if not path_arg:
                    self._reply(query, {'success': False, 'error': 'missing path'})
                    return
                ok, msg = self._set_external_repo(path_arg, persist=True)
                if ok:
                    self._reply(query, {'success': True, 'path': msg})
                else:
                    self._reply(query, {'success': False, 'error': msg})
            elif action == 'clear':
                prior = self._external_repo
                was_bound = self._clear_external_repo()
                self._reply(query, {
                    'success': True,
                    'was_bound': was_bound,
                    'path': str(prior) if prior else None,
                })
            elif action == 'get':
                bound = self._get_external_repo()
                self._reply(query, {
                    'success': True,
                    'path': str(bound) if bound else None,
                })
            else:
                self._reply(query, {
                    'success': False,
                    'error': f'unknown action {action!r}; expected set|clear|get',
                })
        except Exception as e:
            logger.error(f"[{self.character_name}] external_repo query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_concerns_query(self, query) -> None:
        try:
            user_concerns, derived_concerns = self._all_concerns_split()
            self._reply(query, {
                'success': True,
                'user_concerns': user_concerns,
                'derived_concerns': derived_concerns,
                'activations': {},  # chat has no activation system
            })
        except Exception as e:
            logger.error(f"[{self.character_name}] concerns query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_concern_manage_query(self, query) -> None:
        """Browser → chat: status transitions, cadence edit, hard delete,
        bulk wipe.
        Payload: {"concern_id": "Note_X", "action": "close|reopen|satisfy|
                  abandon|activate|delete|set_cadence_hours|wipe_non_seed",
                  "type": "user|derived",
                  "value": <int or null>}  # required for set_cadence_hours

        The action vocabulary mirrors what resource_browser.py emits.
        `delete` is a hard delete (the browser confirms with a dialog
        first); status actions go through _set_concern_status;
        set_cadence_hours snaps the value to the allowed bucket and
        writes to the concern note. `wipe_non_seed` hard-deletes every
        concern whose `seed` property is False and does not require a
        concern_id."""
        try:
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            action = str(params.get('action', '') or '').strip().lower()
            if not action:
                self._reply(query, {'success': False,
                                    'error': "missing action"})
                return

            # Bulk wipe — does not require concern_id. Walks the concerns
            # collection and hard-deletes every note whose `seed` flag is
            # False. Seeds stay (they're architectural baseline).
            if action == 'wipe_non_seed':
                deleted: List[str] = []
                kept_seed: List[str] = []
                errors: List[str] = []
                # Walk both collections. Agent_concerns: keep seeds.
                # User_concerns: no seeds exist (none authored from YAML),
                # so all get wiped.
                for coll_id, keep_seeds in (
                        (self._agent_concerns_collection_id, True),
                        (self._user_concerns_collection_id, False)):
                    if not coll_id:
                        continue
                    coll = self.resource_manager.get_resource(coll_id)
                    note_ids = list(((coll or {}).get('properties') or {}).get('content', []) or [])
                    for nid in note_ids:
                        note = self.resource_manager.get_resource(nid)
                        if not note:
                            continue
                        props = note.get('properties') or {}
                        if keep_seeds and bool(props.get('seed')):
                            kept_seed.append(nid)
                            continue
                        ok, err = self.resource_manager.delete_resource(nid)
                        if ok:
                            deleted.append(nid)
                        else:
                            errors.append(f"{nid}: {err}")
                if deleted:
                    self._persist_to_disk()
                logger.info(
                    f"[{self.character_name}] wipe_non_seed: deleted "
                    f"{len(deleted)}, kept {len(kept_seed)} seed, "
                    f"{len(errors)} errors")
                self._reply(query, {
                    'success': True,
                    'deleted_ids': deleted,
                    'deleted_count': len(deleted),
                    'kept_seed_ids': kept_seed,
                    'kept_seed_count': len(kept_seed),
                    'errors': errors,
                })
                return

            concern_id = str(params.get('concern_id', '') or '').strip()
            if not concern_id:
                self._reply(query, {'success': False,
                                    'error': "missing concern_id"})
                return

            # Hard delete — the browser confirms first.
            if action == 'delete':
                ok, err = self.resource_manager.delete_resource(concern_id)
                if ok:
                    self._persist_to_disk()
                    self._reply(query, {'success': True,
                                        'message': f'{concern_id} deleted'})
                else:
                    self._reply(query, {'success': False,
                                        'error': err or f'delete failed for {concern_id}'})
                return

            # Instruction edit from the browser textarea. Empty / whitespace-
            # only input → null so the concern stops firing, matching the
            # reflection-side null convention (a concern with no instruction
            # is logged for context but never fires). Whitespace-trimming is
            # surface-only; intentional internal blank lines are preserved.
            if action == 'set_instruction':
                note = self.resource_manager.get_resource(concern_id)
                kind = (note.get('properties') or {}).get('kind') if note else None
                if not note or kind not in ('concern', 'agent_concern'):
                    self._reply(query, {'success': False,
                                        'error': f"{concern_id} is not an agent_concern"})
                    return
                raw = params.get('value')
                if raw is None:
                    new_instr: Optional[str] = None
                else:
                    s = str(raw)
                    new_instr = s if s.strip() else None
                note['properties']['instruction'] = new_instr
                self._persist_to_disk()
                preview = (new_instr or '')[:80].replace('\n', ' ')
                logger.info(
                    f"[{self.character_name}] {concern_id} instruction set "
                    f"({len(new_instr or '')} chars): {preview!r}")
                self._reply(query, {
                    'success': True,
                    'message': f'{concern_id} instruction updated',
                    'instruction_chars': len(new_instr or ''),
                })
                return

            # Rhythm edit from the browser combo-box. Accepts both the
            # new action name and the legacy 'set_cadence_hours' so the
            # browser UI keeps working through item-7 refactor.
            if action in ('set_rhythm_hours', 'set_cadence_hours'):
                note = self.resource_manager.get_resource(concern_id)
                kind = (note.get('properties') or {}).get('kind') if note else None
                if not note or kind not in ('concern', 'agent_concern'):
                    self._reply(query, {'success': False,
                                        'error': f"{concern_id} is not an agent_concern"})
                    return
                snapped = _snap_rhythm_hours(params.get('value'))
                note['properties']['rhythm_hours'] = snapped
                # Reset the activation update anchor so the new rhythm
                # takes effect from now (otherwise a long-stalled note
                # would jump immediately).
                note['properties']['last_activation_update_at'] = (
                    datetime.now(timezone.utc).isoformat())
                self._persist_to_disk()
                self._reply(query, {'success': True,
                                    'message': f'{concern_id} rhythm_hours → {snapped}',
                                    'rhythm_hours': snapped,
                                    'cadence_hours': snapped})  # legacy alias
                return

            # Status transitions — map browser vocabulary to chat statuses.
            action_to_status = {
                'close': 'satisfied', 'closed': 'satisfied',
                'satisfy': 'satisfied', 'satisfied': 'satisfied',
                'abandon': 'abandoned', 'abandoned': 'abandoned',
                'reopen': 'active', 'reactivate': 'active',
                'activate': 'active', 'active': 'active',
            }
            new_status = action_to_status.get(action)
            if not new_status:
                self._reply(query, {'success': False,
                                    'error': f"unknown action {action!r}"})
                return
            ok, err = self._set_concern_status(concern_id, new_status)
            if ok:
                self._persist_to_disk()
                self._reply(query, {'success': True,
                                    'message': f'{concern_id} → {new_status}'})
            else:
                self._reply(query, {'success': False, 'error': err})
        except Exception as e:
            logger.error(f"[{self.character_name}] concern_manage query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _shutdown_zenoh(self) -> None:
        try:
            self._affect.close()
        except Exception as e:
            logger.warning(f"[{self.character_name}] affect close failed: {e}")
        try:
            self._canvas.close()
        except Exception as e:
            logger.warning(f"[{self.character_name}] canvas close failed: {e}")
        try:
            if self._sense_sub:
                self._sense_sub.undeclare()
        except Exception:
            pass
        try:
            if self._action_pub:
                self._action_pub.undeclare()
        except Exception:
            pass
        try:
            if self._zenoh_session:
                self._zenoh_session.close()
        except Exception:
            pass
        logger.info(f"[{self.character_name}] chat loop exited")
