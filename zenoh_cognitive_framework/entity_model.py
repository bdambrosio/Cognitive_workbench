#!/usr/bin/env python3
"""
Entity Model for tracking character interactions and conversations.
"""

from datetime import datetime
from typing import List, Dict, Any, Optional
import json
import random
try:
    from llm_client import ZenohLLMClient
    LLM_CLIENT_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  LLM Client not available: {e}")
    LLM_CLIENT_AVAILABLE = False

class EntityModel:
    """
    Model of an entity (character or object) from the perspective of another character.
    Tracks visual sightings and conversation history organized as dialogs.
    """
    
    def __init__(self, entity_name: str, logger, llm_client: ZenohLLMClient):
        self.entity_name = entity_name
        self.first_seen: Optional[datetime] = None
        self.last_seen: Optional[datetime] = None
        self.llm_client = llm_client
        self.logger = logger
        
        # Dialog system - list of dialogs, each dialog is a list of conversation entries
        self.dialogs: List[List[Dict[str, Any]]] = []
        self.active: bool = False  # Whether current dialog is active
    
    def add_conversation_entry(self, source: str, text: str, timestamp: Optional[datetime] = None) -> None:
        """
        Add a conversation entry to the current dialog.
        Creates a new dialog if none exists or if previous dialog was closed.
        
        Args:
            source: Who said/wrote this (character name)
            text: What was said/written
            timestamp: When this occurred (defaults to now)
        """
        if timestamp is None:
            timestamp = datetime.now()
        
        entry = {
            'source': source,
            'text': text,
            'timestamp': timestamp.isoformat()
        }
        
        # Start new dialog if no dialogs exist or previous dialog was closed
        if not self.dialogs or not self.active:
            self.dialogs.append([])
            self.active = True
        
        # Simple chronological order check (only within current dialog)
        current_dialog = self.dialogs[-1]
        if current_dialog:
            last_entry = current_dialog[-1]
            if entry['timestamp'] < last_entry['timestamp']:
                print(f"⚠️ Out-of-chronological order entry for entity {self.entity_name}: "
                      f"new={entry['timestamp']}, last={last_entry['timestamp']}")
        
        # Add to current dialog (last dialog in list)
        current_dialog.append(entry)
        
        # Set first_seen if this is the first interaction
        if self.first_seen is None:
            self.first_seen = timestamp
    
    def close_dialog(self) -> None:
        """
        Close the current dialog. Next conversation entry will start a new dialog.
        """
        self.active = False
    
    def update_last_seen(self) -> None:
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
        Get the most recent conversation entries from the last dialog.
        
        Args:
            limit: Number of recent entries to return (default 20)
            
        Returns:
            List of recent conversation entries from last dialog
        """
        if self.dialogs:
            return self.dialogs[-1][-limit:] if self.dialogs[-1] else []
        return []
    
    def get_recent_conversation_all(self, limit: int = 20) -> List[Dict[str, Any]]:
        """
        Get the most recent conversation entries from all dialogs.
        Works backwards through dialogs to get last N entries chronologically.
        
        Args:
            limit: Number of recent entries to return (default 20)
            
        Returns:
            List of recent conversation entries from all dialogs
        """
        all_entries = []
        for dialog in reversed(self.dialogs):
            remaining = limit - len(all_entries)
            if remaining <= 0:
                break
            dialog_entries = dialog[-remaining:] if len(dialog) > remaining else dialog
            all_entries = dialog_entries + all_entries
        return all_entries
    
    @property
    def conversation_history(self) -> List[Dict[str, Any]]:
        """
        Backward compatibility property that returns current dialog entries.
        """
        return self.get_recent_conversation()
    
    def get_entity_data(self, limit: int = 20, scope: str = 'current') -> Dict[str, Any]:
        """
        Get comprehensive entity data including conversation history.
        
        Args:
            limit: Number of recent conversation entries to include (default 20)
            scope: 'current' for last dialog only, 'all' for entries from all dialogs
            
        Returns:
            Dictionary with entity information and recent conversation
        """
        if scope == 'all':
            recent_history = self.get_recent_conversation_all(limit)
        else:
            recent_history = self.get_recent_conversation(limit)
        
        # Count total conversation entries across all dialogs
        total_entries = sum(len(dialog) for dialog in self.dialogs)
        
        # Determine last interaction type
        last_interaction_type = 'none'
        if total_entries > 0:
            last_interaction_type = 'text'
        elif self.last_seen:
            last_interaction_type = 'visual'
        
        return {
            'entity_name': self.entity_name,
            'first_seen': self.first_seen.isoformat() if self.first_seen else None,
            'last_seen': self.last_seen.isoformat() if self.last_seen else None,
            'conversation_history': recent_history,  # Current or all dialogs based on scope
            'full_history_count': total_entries,
            'dialog_count': len(self.dialogs),
            'active_dialog': self.active,
            'last_interaction_type': last_interaction_type,
            'scope': scope  # Include scope in response for clarity
        }
    
    def has_interaction_history(self) -> bool:
        """
        Check if this entity has any recorded interactions.
        
        Returns:
            True if entity has been seen or conversed with
        """
        return self.last_seen is not None or len(self.dialogs) > 0
    
    def summarize(self) -> str:
        """
        Generate a summary of this entity model.
        
        Returns:
            Human-readable summary string
        """
        summary_parts = [f"Entity: {self.entity_name}"]
        
        if self.first_seen:
            summary_parts.append(f"First seen: {self.first_seen.strftime('%Y-%m-%d %H:%M')}")
        
        if self.last_seen:
            summary_parts.append(f"Last seen: {self.last_seen.strftime('%Y-%m-%d %H:%M')}")
        
        total_entries = sum(len(dialog) for dialog in self.dialogs)
        if total_entries > 0:
            summary_parts.append(f"Total conversations: {total_entries} entries across {len(self.dialogs)} dialogs")
            if self.active:
                summary_parts.append("Current dialog: Active")
            else:
                summary_parts.append("Current dialog: Closed")
        
        return " | ".join(summary_parts)
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Serialize entity model to dictionary for persistence.
        
        Returns:
            Dictionary representation of entity model
        """
        return {
            'entity_name': self.entity_name,
            'first_seen': self.first_seen.isoformat() if self.first_seen else None,
            'last_seen': self.last_seen.isoformat() if self.last_seen else None,
            'dialogs': self.dialogs,
            'active': self.active
        }
    
    @classmethod
    def load_from_dict(cls, data: Dict[str, Any], logger=None, llm_client=None) -> 'EntityModel':
        """
        Load entity model from dictionary (for persistence).
        Handles migration from old conversation_history format.
        
        Args:
            data: Dictionary representation of entity model
            logger: Logger instance (optional)
            llm_client: LLM client instance (optional)
            
        Returns:
            EntityModel instance
        """
        entity = cls(data['entity_name'], logger, llm_client)
        
        # Load timestamps
        if data.get('first_seen'):
            entity.first_seen = datetime.fromisoformat(data['first_seen'])
        if data.get('last_seen'):
            entity.last_seen = datetime.fromisoformat(data['last_seen'])
        
        # Handle migration from old format or load new format
        if 'conversation_history' in data and 'dialogs' not in data:
            # Migration from old format - convert old conversation_history to first dialog
            old_history = data.get('conversation_history', [])
            if old_history:
                entity.dialogs = [old_history]
                entity.active = True  # Assume old conversations were active
            else:
                entity.dialogs = []
                entity.active = False
        else:
            # New format
            entity.dialogs = data.get('dialogs', [])
            entity.active = data.get('active', False)
        
        return entity 
    
    def natural_dialog_end(self, input_text):
        """
        Analyze whether a dialog should naturally end after the given input.
        
        Args:
            input_text: The text input that would end the dialog
            
        Returns:
            bool: True if dialog should end, False if it should continue
        """
        # Build transcript from recent conversation
        transcript_text = ''
        if self.active:
            conversation_entries = self.get_recent_conversation(20)
            if conversation_entries:
                for entry in conversation_entries:
                    if isinstance(entry, dict) and 'source' in entry and 'text' in entry:
                        transcript_text += f"{entry['source']}: {entry['text']}\n"
        
        # Add the proposed input to transcript
        # don't actually need this, already in transcript!
        #transcript_text += f"{self.entity_name}: {input_text}\n"
        
        # If no LLM client available, default to continuing dialog
        if not self.llm_client:
            self.logger.warning(f'No LLM client available for natural_dialog_end, defaulting to continue')
            return False
        
        system_prompt = """Given the following dialog transcript, rate the naturalness of ending at this point.

#Transcript
{{$transcript}}
##
                              
For example, if the last entry in the transcript is a question that expects an answer (as opposed to merely musing), ending at this point is likely not expected.
On the other hand, if the last entry is an agreement to an earlier suggestion, this is a natural end.
Dialogs are short, and should be resolved quickly.
Respond only with a rating between 0 and 10, where
0 expects continuation of the dialog (i.e., termination at this point would be unnatural)
10 expects termination at this point (i.e., continuation is highly unexpected, unnatural, or repetitious).   
                                                  
Do not include any text in your response, ONLY the numeric rating.

My rating is:
"""  
        try:
            response = self.llm_client.generate([system_prompt], bindings={'transcript': transcript_text}, stops=['</end>'], max_tokens=20)
            if response.success:
            # Extract rating from response
                response=response.text
                rating = int(response.lower().replace('</end>','').strip())
                try:
                    rating = int(''.join(filter(str.isdigit, response)))
                    if rating < 0 or rating > 10:
                        rating = 7
                except ValueError:
                    self.logger.warning(f'{self.entity_name} natural_dialog_end: invalid rating: {response}')
                    rating = 7
            
            # Determine if dialog should end based on rating and some randomness
            should_end = rating > 7 or (random.randint(4, 10) < rating) or ((rating + len(transcript_text.split('\n'))) > random.randint(8,10))
            self.logger.info(f'{self.entity_name} natural_dialog_end: rating: {rating}, should_end: {should_end}')
            if should_end:
                self.close_dialog()
            return should_end
            
        except Exception as e:
            self.logger.error(f'Error in natural_dialog_end for {self.entity_name}: {e}')
            return False
    
