#!/usr/bin/env python3
"""
Conversation store capability backed by infospace Notes/Collections.

Runtime-only dialog state is tracked in memory.
"""

import json
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional


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
        # Called with (entity, dialog_id, note_ids) when a dialog is closed
        self._archive_callback: Optional[Callable[[str, str, List[str]], None]] = None

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
        return True

    def record_incoming(self, source: str, text: str, close: bool = False) -> bool:
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
        """Close dialog with entity, archive its turns, then remove them from conversation collection."""
        entity = self._normalize_entity(entity_name)
        dialog_id = self._current_dialog_id.get(entity)
        if not self._active_dialogs.get(entity) and not dialog_id:
            return  # Nothing to close

        self._active_dialogs[entity] = False

        # Collect note IDs belonging to this dialog and trigger archival
        if dialog_id and self._archive_callback:
            note_ids = self._get_dialog_note_ids(dialog_id)
            if note_ids:
                try:
                    self._archive_callback(entity, dialog_id, note_ids)
                    self._remove_notes_from_collection(note_ids)
                except Exception as e:
                    self.logger.error(f'Error during dialog archive for {entity}/{dialog_id}: {e}')

        # Clear dialog tracking for this entity
        self._current_dialog_id.pop(entity, None)
        self._dialog_turn_counts[entity] = 0

    def _get_dialog_note_ids(self, dialog_id: str) -> List[str]:
        """Return note IDs in the conversation collection that belong to a specific dialog_id."""
        collection_id = self._ensure_conversation_collection()
        if not collection_id:
            return []
        collection = self.resource_manager.get_resource(collection_id)
        if not collection:
            return []
        note_ids = collection.get("properties", {}).get("content", [])
        if not isinstance(note_ids, list):
            return []

        matched = []
        for note_id in note_ids:
            if not isinstance(note_id, str) or not note_id.startswith("Note_"):
                continue
            turn = self._parse_turn_note(note_id)
            if turn and turn.get("dialog_id") == dialog_id:
                matched.append(note_id)
        return matched

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

    # ── Conversation history (archived summaries) ──────────────────

    def get_history_notes(self) -> List[Dict[str, Any]]:
        """Return all notes in conversation_history with their IDs and properties.

        Each returned dict has keys: note_id, content, concern_id, timestamp.
        """
        collection_id = self.resource_manager.named_collections.get("conversation_history")
        if not collection_id:
            return []
        collection = self.resource_manager.get_resource(collection_id)
        if not collection:
            return []
        note_ids = collection.get("properties", {}).get("content", [])
        if not isinstance(note_ids, list):
            return []

        entries = []
        for note_id in note_ids:
            if not isinstance(note_id, str) or not note_id.startswith("Note_"):
                continue
            resource = self.resource_manager.get_resource(note_id)
            if not resource:
                continue
            props = resource.get("properties", {})
            content = props.get("content", "")
            entries.append({
                "note_id": note_id,
                "content": content if isinstance(content, str) else str(content),
                "concern_id": props.get("concern_id"),
                "goal_id": props.get("goal_id"),
                "timestamp": props.get("timestamp", props.get("created_at", "")),
            })
        return entries

    def tag_note_concern(self, note_id: str, concern_id: str) -> bool:
        """Tag a note with a concern_id in its properties."""
        resource = self.resource_manager.get_resource(note_id)
        if not resource:
            return False
        resource.get("properties", {})["concern_id"] = concern_id
        return True

    def tag_note_goal(self, note_id: str, goal_id: str) -> bool:
        """Tag a note with a goal_id in its properties."""
        resource = self.resource_manager.get_resource(note_id)
        if not resource:
            return False
        resource.get("properties", {})["goal_id"] = goal_id
        return True

    def get_themed_context(self, concerns: List[Dict], max_total_chars: int = 3000) -> str:
        """Build conversation context organized by concern themes.

        Args:
            concerns: List of concern dicts from the concern model (must have
                      concern_id, concern_label, weight, status, history_summary,
                      history_note_ids).
            max_total_chars: Budget for the entire themed context block.

        Returns:
            Formatted string ready for inclusion in a prompt.
        """
        history_notes = self.get_history_notes()

        # Index history notes by note_id for quick lookup
        notes_by_id = {n["note_id"]: n for n in history_notes}

        # --- Pool C: last 2 history entries for recency (any theme) ---
        last_two = history_notes[-2:] if len(history_notes) >= 2 else list(history_notes)
        last_two_ids = {n["note_id"] for n in last_two}

        # --- Pool A+B: concern-based entries ---
        # Score and rank open concerns
        active_concerns = [c for c in concerns if c.get("status") == "open"]
        active_concerns.sort(key=lambda c: float(c.get("weight", 0)), reverse=True)
        top_concerns = active_concerns[:5]

        # Budget allocation
        recency_budget = min(800, max_total_chars // 3)
        theme_budget = max_total_chars - recency_budget

        parts = []

        # Recency section
        if last_two:
            parts.append("## RECENT CONVERSATIONS (for continuity)")
            for entry in last_two:
                text = entry["content"][:recency_budget // max(len(last_two), 1)]
                parts.append(f"  {text}")

        # Theme section
        if top_concerns:
            per_concern_budget = theme_budget // max(len(top_concerns), 1)
            parts.append("\n## CONVERSATION THEMES")
            for concern in top_concerns:
                cid = concern.get("concern_id", "")
                label = concern.get("concern_label", "unknown")
                weight = concern.get("weight", 0)
                history_summary = concern.get("history_summary", "")
                history_note_ids = concern.get("history_note_ids", [])

                section_parts = []
                section_parts.append(f"### {label} (weight: {weight:.1f})")

                chars_used = 0

                # Pool A: rolled-up history_summary
                if history_summary:
                    truncated = history_summary[:per_concern_budget // 2]
                    section_parts.append(f"  {truncated}")
                    chars_used += len(truncated)

                # Pool B: unconsolidated entries for this concern
                remaining = per_concern_budget - chars_used
                for nid in history_note_ids:
                    if nid in last_two_ids:
                        continue  # already in recency section
                    note = notes_by_id.get(nid)
                    if note and remaining > 0:
                        text = note["content"][:remaining]
                        section_parts.append(f"  {text}")
                        remaining -= len(text)

                parts.append("\n".join(section_parts))

        return "\n".join(parts) if parts else ""

    def get_entity_context(self, entity_name: str, limit: int = 20, scope: str = "all") -> Dict[str, Any]:
        entity = self._normalize_entity(entity_name)
        all_turns = self._get_all_entity_turns(entity)
        turns = self.get_recent_turns(entity, limit=limit, scope=scope)
        dialog_ids = [str(t.get("dialog_id", "")) for t in all_turns if t.get("dialog_id")]
        unique_dialog_ids = set(dialog_ids)

        # Backfill with prior session summaries when current conversation is short.
        # Proportional: fewer current turns → more backfill, capped at 5.
        prior_summaries = []
        available_slots = limit - len(turns)
        if available_slots > 3:
            backfill_count = min(available_slots // 3, 5)
            try:
                history_notes = self.get_history_notes()
                if history_notes:
                    # Most recent summaries first
                    history_notes.sort(
                        key=lambda h: h.get("timestamp", ""), reverse=True)
                    for h in history_notes[:backfill_count]:
                        content = h.get("content", "")
                        if content and len(content) > 20:
                            entry = {
                                "source": "prior_session",
                                "text": content[:500],
                                "timestamp": h.get("timestamp", ""),
                                "note_id": h.get("note_id", ""),
                            }
                            if h.get("goal_id"):
                                entry["goal_id"] = h["goal_id"]
                            prior_summaries.append(entry)
            except Exception:
                pass

        return {
            "entity_name": entity,
            "conversation_history": turns,
            "prior_session_summaries": prior_summaries,
            "full_history_count": len(all_turns),
            "dialog_count": len(unique_dialog_ids),
            "active_dialog": bool(self._active_dialogs.get(entity, False)),
            "last_interaction_type": "text" if all_turns else "none",
            "scope": scope
        }
