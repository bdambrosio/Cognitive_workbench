#!/usr/bin/env python3
"""
Conversation store capability backed by infospace Notes/Collections.

Runtime-only dialog state is tracked in memory.
"""

import json
from datetime import datetime
from typing import Any, Dict, List, Optional


# Cap on how many conversation turn Notes the in-memory collection
# retains. Older Notes are trimmed (and the underlying Notes deleted)
# after each append. The on-disk conversation.txt remains the canonical
# full history; the in-memory collection only feeds the rolling
# last-N-turns prompt window and discourse tracking, both of which read
# at most ~20 turns. 50 leaves headroom above the read window without
# unbounded RAM/persist growth.
_CONVERSATION_RETENTION = 50


class ConversationStore:
    """Single conversation source-of-truth over Notes/Collections."""

    def __init__(self, resource_manager: Any, character_name: str, logger: Any):
        self.resource_manager = resource_manager
        self.character_name = character_name
        self.logger = logger

        self.collection_name = "conversation"
        self._dialog_counter = 0
        self._current_dialog_id: Dict[str, str] = {}
        self._active_dialogs: Dict[str, bool] = {}
        self._dialog_turn_counts: Dict[str, int] = {}

    def initialize(self) -> bool:
        """Ensure conversation collection exists."""
        return self._ensure_conversation_collection() is not None

    def _normalize_entity(self, entity_name: Optional[str]) -> str:
        if not entity_name:
            return "User"
        if entity_name.lower() in ("console", "unknown"):
            return "User"
        if entity_name.lower() == "user":
            return "User"
        return entity_name.capitalize()

    def _ensure_conversation_collection(self) -> Optional[str]:
        if not self.resource_manager:
            return None
        collection_id = self.resource_manager.named_collections.get(self.collection_name)
        if collection_id and self.resource_manager.get_resource(collection_id):
            return collection_id

        success, collection_id, error_msg, _ = self.resource_manager.create_collection(
            self.character_name, [], "list", "conversation-store", "", self.collection_name, {}
        )
        if not success:
            self.logger.warning(f'Failed to create conversation collection: {error_msg}')
            return None
        self.logger.info(f'✓ Created conversation collection ({collection_id})')
        return collection_id

    def _next_dialog_id(self, entity: str) -> str:
        self._dialog_counter += 1
        return f"{entity}:{self._dialog_counter}"

    def _get_or_start_dialog(self, entity: str) -> str:
        if self._active_dialogs.get(entity) and self._current_dialog_id.get(entity):
            return self._current_dialog_id[entity]
        dialog_id = self._next_dialog_id(entity)
        self._current_dialog_id[entity] = dialog_id
        self._active_dialogs[entity] = True
        self._dialog_turn_counts[entity] = 0
        return dialog_id

    def _append_turn(self, turn: Dict[str, Any]) -> bool:
        if not self.resource_manager:
            return False
        collection_id = self._ensure_conversation_collection()
        if not collection_id:
            return False

        content = json.dumps(turn, ensure_ascii=True)
        note_props = {"kind": "conversation_turn", "entity": turn.get("entity"), "source": turn.get("source")}
        success, note_id, error_msg, _ = self.resource_manager.create_note(
            self.character_name, content, "text", "conversation-store", turn.get("text", "")[:100], "", note_props
        )
        if not success or not note_id:
            self.logger.warning(f'Failed to create conversation turn note: {error_msg}')
            return False

        added, _, add_error = self.resource_manager.add_to_collection(collection_id, note_id, self.character_name, operation='add')
        if not added:
            self.logger.warning(f'Failed to add conversation turn note to collection: {add_error}')
            return False
        self._trim_collection()
        return True

    def _trim_collection(self, retain: int = _CONVERSATION_RETENTION) -> None:
        """Drop oldest Notes from the conversation collection beyond the
        retention cap, deleting the underlying Note resources too. The
        collection content list is insertion-ordered (oldest first) per
        add_to_collection's append semantics, so [:-retain] is the
        oldest tail to drop. Best-effort — failures log and continue."""
        try:
            collection_id = self._ensure_conversation_collection()
            if not collection_id:
                return
            collection = self.resource_manager.get_resource(collection_id)
            if not collection:
                return
            current = (collection.get("properties") or {}).get("content") or []
            if not isinstance(current, list) or len(current) <= retain:
                return
            to_remove = current[:-retain]
            self._remove_notes_from_collection(to_remove, delete_notes=True)
        except Exception as e:
            self.logger.warning(f'conversation collection trim failed: {e}')

    def record_incoming(self, source: str, text: str, close: bool = False,
                        modality: Optional[str] = None) -> bool:
        entity = self._normalize_entity(source)
        dialog_id = self._get_or_start_dialog(entity)
        turn = {
            "entity": entity,
            "source": source,
            "target": self.character_name,
            "text": str(text),
            "act_type": "inbound",
            "direction": "in",
            "timestamp": datetime.now().isoformat(),
            "dialog_id": dialog_id,
            "close": bool(close)
        }
        # Provenance: voice vs typed turn. Only stamped when the transport
        # supplied it, so older records keep their shape.
        if modality:
            turn["modality"] = modality
        ok = self._append_turn(turn)
        if ok:
            self._dialog_turn_counts[entity] = self._dialog_turn_counts.get(entity, 0) + 1
            if close:
                self._active_dialogs[entity] = False
        return ok

    def record_outgoing(self, target: str, text: str, act_type: str = "say", close: bool = False) -> bool:
        entity = self._normalize_entity(target)
        dialog_id = self._get_or_start_dialog(entity)
        turn = {
            "entity": entity,
            "source": self.character_name,
            "target": target,
            "text": str(text),
            "act_type": act_type,
            "direction": "out",
            "timestamp": datetime.now().isoformat(),
            "dialog_id": dialog_id,
            "close": bool(close)
        }
        ok = self._append_turn(turn)
        if ok:
            self._dialog_turn_counts[entity] = self._dialog_turn_counts.get(entity, 0) + 1
            if close:
                self._active_dialogs[entity] = False
        return ok

    def close_dialog(self, entity_name: str = "User") -> None:
        """Mark the entity's dialog inactive and clear per-entity dialog
        bookkeeping. Turn Notes themselves are not touched here — they
        live in the conversation collection until trimmed by the
        retention cap (see _trim_collection)."""
        entity = self._normalize_entity(entity_name)
        dialog_id = self._current_dialog_id.get(entity)
        if not self._active_dialogs.get(entity) and not dialog_id:
            return  # Nothing to close
        self._active_dialogs[entity] = False
        self._current_dialog_id.pop(entity, None)
        self._dialog_turn_counts[entity] = 0

    def _remove_notes_from_collection(self, note_ids_to_remove: List[str],
                                      collection_name: str = "conversation",
                                      delete_notes: bool = False) -> bool:
        """Remove specific note IDs from a named collection, optionally deleting the notes.

        Args:
            note_ids_to_remove: Note IDs to remove from the collection.
            collection_name: Name of the collection to remove from (default: conversation).
            delete_notes: If True, also delete the underlying Note resources.
        """
        if collection_name == "conversation":
            collection_id = self._ensure_conversation_collection()
        else:
            collection_id = self.resource_manager.named_collections.get(collection_name)
        if not collection_id:
            return False
        collection = self.resource_manager.get_resource(collection_id)
        if not collection:
            return False
        current = collection.get("properties", {}).get("content", [])
        if not isinstance(current, list):
            return False
        remove_set = set(note_ids_to_remove)
        remaining = [nid for nid in current if nid not in remove_set]
        success, _, err = self.resource_manager.add_to_collection(
            collection_id, None, self.character_name, operation='update', content=remaining
        )
        if not success:
            self.logger.warning(f'Failed to remove notes from {collection_name}: {err}')
            return False

        if delete_notes:
            for note_id in note_ids_to_remove:
                ok, del_err = self.resource_manager.delete_resource(note_id)
                if not ok:
                    self.logger.warning(f'Failed to delete note {note_id}: {del_err}')

        return True

    def has_active_dialogs(self) -> bool:
        return any(self._active_dialogs.values())

    def is_dialog_active(self, entity_name: str = "User") -> bool:
        entity = self._normalize_entity(entity_name)
        return bool(self._active_dialogs.get(entity, False))

    def get_turn_count(self, entity_name: str = "User") -> int:
        entity = self._normalize_entity(entity_name)
        return int(self._dialog_turn_counts.get(entity, 0))

    def _parse_turn_note(self, note_id: str) -> Optional[Dict[str, Any]]:
        resource = self.resource_manager.get_resource(note_id) if self.resource_manager else None
        if not resource:
            return None
        content = resource.get("properties", {}).get("content")
        if not isinstance(content, str) or not content.strip():
            return None
        try:
            parsed = json.loads(content)
            if isinstance(parsed, dict):
                return parsed
        except Exception:
            return None
        return None

    def is_ask_only_conversation(self) -> bool:
        """Return True if the conversation has exactly one turn and it is an outgoing ask."""
        collection_id = self._ensure_conversation_collection()
        if not collection_id or not self.resource_manager:
            return False
        collection = self.resource_manager.get_resource(collection_id)
        if not collection:
            return False
        note_ids = collection.get("properties", {}).get("content", [])
        if not isinstance(note_ids, list) or len(note_ids) != 1:
            return False
        note_id = note_ids[0]
        if not isinstance(note_id, str) or not note_id.startswith("Note_"):
            return False
        turn = self._parse_turn_note(note_id)
        if not turn:
            return False
        return turn.get("act_type") == "ask" and turn.get("direction") == "out"

    def _get_all_entity_turns(self, entity: str) -> List[Dict[str, Any]]:
        collection_id = self._ensure_conversation_collection()
        if not collection_id:
            return []
        collection = self.resource_manager.get_resource(collection_id)
        if not collection:
            return []
        note_ids = collection.get("properties", {}).get("content", [])
        if not isinstance(note_ids, list):
            return []

        turns: List[Dict[str, Any]] = []
        for note_id in note_ids:
            if not isinstance(note_id, str) or not note_id.startswith("Note_"):
                continue
            turn = self._parse_turn_note(note_id)
            if not turn:
                continue
            if self._normalize_entity(turn.get("entity")) != entity:
                continue
            turns.append(turn)
        return turns

    def get_recent_turns(self, entity_name: str, limit: int = 20, scope: str = "all") -> List[Dict[str, Any]]:
        entity = self._normalize_entity(entity_name)
        turns = self._get_all_entity_turns(entity)
        if not turns:
            return []

        if scope == "current":
            dialog_id = self._current_dialog_id.get(entity)
            if not dialog_id:
                dialog_id = turns[-1].get("dialog_id")
            if dialog_id:
                turns = [t for t in turns if t.get("dialog_id") == dialog_id]

        if limit <= 0:
            return []
        return turns[-limit:]
