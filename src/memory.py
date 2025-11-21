"""
Memory module - wraps EntityModel and discourse for character memory management.

Simple module with no Zenoh dependencies - direct method calls only.
Uses llm_generate callback for SGLang runtime support.
"""

import json
import logging
from pathlib import Path
from typing import Dict, List, Any, Optional, Callable
from datetime import datetime

from entity_model import EntityModel

logger = logging.getLogger(__name__)


class Memory:
    """
    Simple memory module wrapping EntityModel and discourse.
    No Zenoh dependencies - direct method calls only.
    """
    
    def __init__(self, character_name: str, llm_generate: Callable, persistence_path: str):
        """
        Initialize memory module.
        
        Args:
            character_name: Name of the character
            llm_generate: Callback function for LLM generation (from executive_node, uses SGLang)
            persistence_path: Path to JSON file for persistence
        """
        self.character_name = character_name.capitalize()
        self.llm_generate = llm_generate
        self.persistence_path = Path(persistence_path)
        self.persistence_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Entity registry - keyed by entity name, defaults to "User"
        self.entity_models: Dict[str, EntityModel] = {}
        
        # Track last activity per entity
        self.entity_last_activity: Dict[str, datetime] = {}
        
        # Load existing memory
        self.load()
        
        logger.info(f'Memory module initialized for {self.character_name}')
    
    def get_or_create_entity(self, entity_name: str = "User") -> EntityModel:
        """
        Get an existing entity or create a new one (case-insensitive).
        Defaults to "User" if entity_name not provided.
        
        Args:
            entity_name: Name of the entity (defaults to "User")
            
        Returns:
            EntityModel instance
        """
        # Canonicalize entity name using capitalize
        canonical_entity_name = entity_name.capitalize()
        
        if canonical_entity_name not in self.entity_models:
            # Create new entity with canonicalized name
            self.entity_models[canonical_entity_name] = EntityModel(
                self.character_name, 
                canonical_entity_name, 
                logger, 
                self.llm_generate
            )
            logger.info(f'👥 Created new entity: {canonical_entity_name}')
            return self.entity_models[canonical_entity_name]
        else:
            # Return existing entity
            return self.entity_models[canonical_entity_name]
    
    def add_conversation_entry(self, entity_name: str, source: str, text: str) -> None:
        """
        Add a conversation entry for an entity.
        
        Args:
            entity_name: Name of the entity (defaults to "User" if not provided)
            source: Character name who said this
            text: The message text
        """
        # Default to "User" if not provided
        if not entity_name:
            entity_name = "User"
            
        entity = self.get_or_create_entity(entity_name)
        entity.add_conversation_entry(source, text)
        self.entity_last_activity[entity_name] = datetime.now()
        logger.info(f'💬 Added conversation entry for {entity_name}: {source} "{text[:50]}..."')
    
    def close_dialog(self, entity_name: str = "User") -> None:
        """
        Close the active dialog for an entity.
        
        Args:
            entity_name: Name of the entity (defaults to "User")
        """
        canonical_entity_name = entity_name.capitalize()
        
        if canonical_entity_name in self.entity_models:
            self.entity_models[canonical_entity_name].close_dialog()
            logger.info(f'🔚 Closed dialog with {canonical_entity_name}')
        else:
            logger.warning(f'Cannot close dialog - entity {canonical_entity_name} not found')
    
    def update_visual_detection(self, entity_name: str) -> None:
        """
        Update visual detection for an entity.
        
        Args:
            entity_name: Name of the entity
        """
        entity = self.get_or_create_entity(entity_name)
        entity.update_last_seen()
        logger.info(f'👁️ Updated visual detection for {entity_name}')
    
    def get_entity_data(self, entity_name: str, limit: int = 20, scope: str = 'current') -> Optional[Dict[str, Any]]:
        """
        Get comprehensive entity data including conversation history.
        
        Args:
            entity_name: Name of the entity
            limit: Number of recent conversation entries to include
            scope: 'current' for last dialog only, 'all' for entries from all dialogs
            
        Returns:
            Dictionary with entity information or None if not found
        """
        canonical_entity_name = entity_name.capitalize()
        
        if canonical_entity_name not in self.entity_models:
            return None
        
        entity = self.entity_models[canonical_entity_name]
        return entity.get_entity_data(limit=limit, scope=scope)
    
    def get_entity_discourse_tom(self, entity_name: str) -> Dict[str, Any]:
        """
        Get entity discourse state and ToM model.
        
        Args:
            entity_name: Name of the entity
            
        Returns:
            Dictionary with discourse_state and tom_model
        """
        canonical_entity_name = entity_name.capitalize()
        
        if canonical_entity_name not in self.entity_models:
            return {
                'success': False,
                'discourse_state': '',
                'tom_model': ''
            }
        
        entity = self.entity_models[canonical_entity_name]
        return {
            'success': True,
            'discourse_state': entity.discourse_state,
            'tom_model': entity.tom_model,
            'entity_name': canonical_entity_name
        }
    
    def save(self):
        """Save memory to file."""
        try:
            data = {
                'character_name': self.character_name,
                'entities': {}
            }
            for entity_name, entity in self.entity_models.items():
                data['entities'][entity_name] = entity.to_dict()
            
            with open(self.persistence_path, 'w') as f:
                json.dump(data, f, indent=2, default=str)
            logger.info(f'💾 Saved memory to {self.persistence_path}')
        except Exception as e:
            logger.error(f'Error saving memory: {e}')
    
    def load(self):
        """Load memory from file."""
        try:
            if self.persistence_path.exists():
                with open(self.persistence_path, 'r') as f:
                    data = json.load(f)
                    self.entity_models = {}
                    for entity_name, entity_data in data.get('entities', {}).items():
                        entity = EntityModel.load_from_dict(
                            self.character_name, 
                            entity_data, 
                            logger, 
                            self.llm_generate
                        )
                        self.entity_models[entity_name] = entity
                    
                logger.info(f'📂 Loaded memory from {self.persistence_path}')
            else:
                logger.info(f'📂 No existing memory file, starting fresh')
        except Exception as e:
            logger.error(f'Error loading memory: {e}')

