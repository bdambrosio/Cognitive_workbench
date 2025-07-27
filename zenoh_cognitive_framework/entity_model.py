#!/usr/bin/env python3
"""
Entity Model for tracking character interactions and conversations.
"""

from datetime import datetime
from typing import List, Dict, Any, Optional


class EntityModel:
    """
    Tracks conversation history and interaction patterns for a specific entity.
    
    Attributes:
        entity_name: Name of the entity (character)
        conversation_history: Complete list of conversation entries
        last_seen: Timestamp of last visual detection (not conversation)
        first_seen: Timestamp of first interaction
    """
    
    def __init__(self, entity_name: str):
        self.entity_name: str = entity_name
        self.conversation_history: List[Dict[str, Any]] = []
        self.last_seen: Optional[datetime] = None
        self.first_seen: Optional[datetime] = None
    
    def add_conversation_entry(self, direction: str, text: str, source: str) -> None:
        """
        Add a conversation entry to the history.
        
        Args:
            direction: 'sent' or 'received'
            text: The message text
            source: 'ui' or character name
        """
        entry = {
            'timestamp': datetime.now().isoformat(),
            'direction': direction,
            'text': text,
            'source': source,
            'is_summary': False  # Flag for future summary entries
        }
        
        self.conversation_history.append(entry)
        
        # Set first_seen if this is the first interaction
        if self.first_seen is None:
            self.first_seen = datetime.now()
    
    def update_visual_detection(self) -> None:
        """
        Update the last_seen timestamp when entity is visually detected.
        This is separate from conversation interactions.
        """
        self.last_seen = datetime.now()
        
        # Set first_seen if this is the first interaction
        if self.first_seen is None:
            self.first_seen = datetime.now()
    
    def get_recent_conversation(self, limit: int = 20) -> List[Dict[str, Any]]:
        """
        Get the most recent conversation entries.
        
        Args:
            limit: Number of recent entries to return (default 20)
            
        Returns:
            List of recent conversation entries
        """
        return self.conversation_history[-limit:] if self.conversation_history else []
    
    def get_entity_data(self, limit: int = 20) -> Dict[str, Any]:
        """
        Get entity data for query response.
        
        Args:
            limit: Number of recent conversation entries to include
            
        Returns:
            Dictionary with entity information and recent conversation
        """
        recent_history = self.get_recent_conversation(limit)
        
        # Determine last interaction type
        last_interaction_type = 'none'
        if self.conversation_history:
            last_interaction_type = 'text'
        elif self.last_seen:
            last_interaction_type = 'visual'
        
        return {
            'entity_name': self.entity_name,
            'first_seen': self.first_seen.isoformat() if self.first_seen else None,
            'last_seen': self.last_seen.isoformat() if self.last_seen else None,
            'conversation_history': recent_history,
            'full_history_count': len(self.conversation_history),
            'last_interaction_type': last_interaction_type
        }
    
    def has_interaction_history(self) -> bool:
        """
        Check if entity has any interaction history.
        
        Returns:
            True if entity has conversation or visual interaction history
        """
        return len(self.conversation_history) > 0 or self.last_seen is not None

    def to_dict(self) -> Dict[str, Any]:
        """Convert entity to dictionary for serialization."""
        return {
            'entity_name': self.entity_name,
            'conversation_history': self.conversation_history,
            'last_seen': self.last_seen.isoformat() if self.last_seen else None,
            'first_seen': self.first_seen.isoformat() if self.first_seen else None
        }

    def load_from_dict(self, data: Dict[str, Any]) -> None:
        """Load entity from dictionary."""
        self.entity_name = data.get('entity_name', self.entity_name)
        self.conversation_history = data.get('conversation_history', [])
        
        # Parse timestamps
        last_seen_str = data.get('last_seen')
        if last_seen_str:
            self.last_seen = datetime.fromisoformat(last_seen_str)
        
        first_seen_str = data.get('first_seen')
        if first_seen_str:
            self.first_seen = datetime.fromisoformat(first_seen_str)

    def summarize(self) -> str:
        """Generate a summary of conversation history. Stub for now."""
        # TODO: Implement summarization logic
        return "Summary stub - to be implemented" 