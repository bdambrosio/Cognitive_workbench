#!/usr/bin/env python3
"""
Entity Model for tracking character interactions and conversations.
"""

from datetime import datetime
import traceback
from typing import List, Dict, Any, Optional
import json
import random
import discourse
from utils import hash_utils
from typing import Callable

class EntityModel:
    """
    Model of an entity (character or object) from the perspective of another character.
    Tracks visual sightings and conversation history organized as dialogs.
    """
    
    def __init__(self, character_name: str, entity_name: str, logger, llm_generate: Callable):
        self.character_name = character_name
        self.entity_name = entity_name
        self.first_seen: Optional[datetime] = None
        self.last_seen: Optional[datetime] = None
        self.llm_generate = llm_generate
        self.logger = logger
        self.discourse = discourse.DiscourseTracker(llm_generate, character_name, entity_name)
        self.discourse_state = ""
        self.tom_model = ""
        
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
        # Consolidate dialogs if there are more than 6
        dialog = self.dialogs[-1]
        self.discourse_state = self.discourse.analyze_segment(dialog, previous_discourse_state=self.discourse_state, tom=self.tom_model)
        self.tom_model = self.discourse.update_tom_from_discourse_segment(dialog, self.entity_name, discourse_state=self.discourse_state, previous_tom_state=self.tom_model)
        if len(self.dialogs) <= 6:
            return
        dialog_to_consolidate = self.dialogs[-6]

        if dialog_to_consolidate and isinstance(dialog_to_consolidate, list):
            dialog_history = f"Your recent conversation with {self.entity_name} has been:\n"
            for i, memory in enumerate(dialog_to_consolidate):
                if isinstance(memory, dict) and 'source' in memory and 'text' in memory:
                    dialog_history += f"\t{memory['source']}: {memory['text']}\n"

        prompt = """Given the following dialog transcript, create two summaries, one for each of the two characters.
Each summary should be a single sentence accurately conveying the essence of that character's part of the conversation.
Each summary Should accurately reflect both the content and tone of that character's part of the conversation.

#Dialog transcript
{{$transcript}}
##

Respond with the two summaries in hash-formatted text using the following format:

#{{$name}} <Summary for {{$name}}>
#{{$other_name}} <Summary for {{$other_name}}>
##

Do not include any other introductory, explanatory, discursive, or formatting text in your response.
End your response with: 
</end>"""
        
        # Apply bindings
        prompt_with_bindings = prompt
        prompt_with_bindings = prompt_with_bindings.replace("{{$transcript}}", dialog_history)
        prompt_with_bindings = prompt_with_bindings.replace("{{$name}}", self.character_name)
        prompt_with_bindings = prompt_with_bindings.replace("{{$other_name}}", self.entity_name)
        
        response = self.llm_generate(
            messages=[prompt_with_bindings],
            bindings={},
            max_tokens=60,
            temperature=0.7,
            stops=['</end>'],
            is_json=False
        )
        
        # Handle response format
        if isinstance(response, dict):
            response_text = response.get('text', '')
            success = response.get('success', False)
        elif hasattr(response, 'text'):
            response_text = response.text
            success = response.success if hasattr(response, 'success') else True
        else:
            response_text = str(response)
            success = True
        
        if success and response_text:
            me = hash_utils.find(f'{self.character_name}', response_text)
            other = hash_utils.find(f'{self.entity_name}', response_text)
            # Wrap summaries in proper entry dicts to maintain consistent structure
            self.dialogs[-6] = [
                {
                    'source': self.character_name,
                    'text': me,
                    'timestamp': datetime.now().isoformat(),
                    'summary': True
                },
                {
                    'source': self.entity_name,
                    'text': other,
                    'timestamp': datetime.now().isoformat(),                    
                    'summary': True
                }
            ]
        else:
            self.logger.error(f'Error in consolidate_dialog for {self.character_name} and {self.entity_name}: {response_text if success else "LLM call failed"}')


    
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
            'dialogs': self.dialogs,  # Full nested dialog structure for RAG context expansion
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
            'active': self.active,
            'discourse_state': self.discourse_state,
            'tom_model': self.tom_model
        }
    
    @classmethod
    def load_from_dict(cls, character_name: str, data: Dict[str, Any], logger=None, llm_generate=None) -> 'EntityModel':
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
        entity = cls(character_name, data['entity_name'], logger, llm_generate=llm_generate)
        
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
        
        # Migrate malformed dialog entries (bare strings instead of dicts)
        # This can happen from old consolidation code that didn't wrap summaries properly
        for dialog_idx, dialog in enumerate(entity.dialogs):
            if not isinstance(dialog, list):
                if logger:
                    logger.error(f'Dialog {dialog_idx} for {entity.entity_name} is not a list: {type(dialog)} - skipping')
                continue
                
            normalized_dialog = []
            for entry_idx, entry in enumerate(dialog):
                if isinstance(entry, str):
                    # Malformed entry: bare string instead of dict (likely from old summary consolidation)
                    if logger:
                        logger.warning(f'Migrating malformed dialog entry for {entity.entity_name}, dialog {dialog_idx}, entry {entry_idx}')
                    # Infer source based on position (summaries alternate: character, entity)
                    inferred_source = character_name if entry_idx % 2 == 0 else entity.entity_name
                    normalized_entry = {
                        'source': inferred_source,
                        'text': entry,
                        'timestamp': datetime.now().isoformat(),
                        'migrated': True,  # Mark for tracking
                        'summary': True    # Assume it was a summary
                    }
                    normalized_dialog.append(normalized_entry)
                elif isinstance(entry, dict):
                    # Already correct format
                    normalized_dialog.append(entry)
                else:
                    # Completely broken, skip with warning
                    if logger:
                        logger.error(f'Skipping unrecognizable dialog entry for {entity.entity_name}: type={type(entry)}, value={entry}')
                    # Don't append - skip corrupted entries
            
            entity.dialogs[dialog_idx] = normalized_dialog
        
        # Load discourse and ToM state (backward compatible with defaults)
        entity.discourse_state = data.get('discourse_state', '')
        entity.tom_model = data.get('tom_model', '')
        
        return entity 
    
    def natural_dialog_end(self, input_text, context):
        """
        Analyze whether a dialog should naturally end after the given input.
        
        Args:
            input_text: The text input that would end the dialog
            
        Returns:
            bool: True if dialog should end, False if it should continue
        """
        # Build transcript from recent conversation
        if input_text.strip().endswith('?') or self.entity_name.strip() == 'User':
            return False
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
        
        # If no LLM generate callback available, default to continuing dialog
        if not self.llm_generate:
            self.logger.warning(f'No LLM generate callback available for natural_dialog_end, defaulting to continue')
            return False
        
        prompt = """Given the following context and dialog transcript, rate the naturalness of ending at this point.
Include in your consideration of whether you are likely to end the dialog your personality and drives. 

#Context
You are {{$context}}
##

#Transcript
{{$transcript}}
##
                              
For example, if the last entry in the transcript contains a question that expects an answer (as opposed to merely musing), ending at this point is likely not expected.
On the other hand, if the last entry is an agreement to an earlier suggestion, this may be a natural end.
Dialogs may be short or longer depending on the characters involved and their personalities, conversational styles, and relationship.
Respond only with a rating between 0 and 10, where
0 expects continuation of the dialog (i.e., termination at this point would be unnatural)
10 expects termination at this point (i.e., continuation is highly unexpected, unnatural, or repetitious).   
                                                  
Do not include any text in your response, ONLY the numeric rating.

My rating is:
"""
        
        # Apply bindings
        prompt_with_bindings = prompt
        prompt_with_bindings = prompt_with_bindings.replace("{{$context}}", context)
        prompt_with_bindings = prompt_with_bindings.replace("{{$transcript}}", transcript_text)
        
        try:
            response = self.llm_generate(
                messages=[prompt_with_bindings],
                bindings={},
                max_tokens=20,
                temperature=0.7,
                stops=['</end>'],
                is_json=False
            )
            
            # Handle response format
            if isinstance(response, dict):
                response_text = response.get('text', '')
                success = response.get('success', False)
            elif hasattr(response, 'text'):
                response_text = response.text
                success = response.success if hasattr(response, 'success') else True
            else:
                response_text = str(response)
                success = True
            
            if success:
                # Extract rating from response
                try:
                    rating = int(''.join(filter(str.isdigit, response_text)))
                    if rating < 0 or rating > 10:
                        rating = 7
                except ValueError:
                    self.logger.warning(f'{self.entity_name} natural_dialog_end: invalid rating: {response_text}')
                    rating = 7
            
            # Determine if dialog should end based on rating and some randomness
            should_end = rating > 7 or (random.randint(4, 10) < rating) or ((rating + len(transcript_text.split('\n'))) > random.randint(12,14))
            self.logger.info(f'{self.entity_name} natural_dialog_end: rating: {rating}, should_end: {should_end}')
            if should_end:
                self.close_dialog()
            return should_end
            
        except Exception as e:
            self.logger.error(f'Error in natural_dialog_end for {self.entity_name}: {e}')
            traceback.print_exc()
            self.close_dialog()
            return True
    
