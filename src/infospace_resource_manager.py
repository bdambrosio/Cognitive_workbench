"""
Infospace Resource Manager

Manages Notes, Collections, and indexing infrastructure for the infospace cognitive map.
Extracts resource management logic from map_node to improve modularity and maintainability.
"""

import json
import logging
import hashlib
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, Any, Optional, List, Tuple

from infospace_types import InfospaceResources, ResourceTypeRegistry

logger = logging.getLogger(__name__)


class FAISSStore:
    """
    FAISS-based vector store for semantic search.
    Stores documents with embeddings and metadata.
    """
    
    def __init__(self, dimension=384, logger=None):
        import faiss
        import numpy as np
        
        self.dimension = dimension
        self.index = faiss.IndexFlatIP(dimension)  # Inner product (cosine similarity)
        self.documents = []
        self.metadata = []
        self.original_contents = []
        self.logger = logger
        self.np = np
    
    def add(self, document, embedding, metadata=None, original_content=None):
        """Add document with embedding to store"""
        import faiss
        
        # Normalize for cosine similarity
        embedding_array = self.np.array([embedding], dtype='float32')
        faiss.normalize_L2(embedding_array)
        
        self.index.add(embedding_array)
        self.documents.append(document)
        self.metadata.append(metadata or {})
        self.original_contents.append(original_content if original_content is not None else document)
    
    def search(self, query_embedding, limit=5, threshold=0.0, max_tokens=None, token_estimate_chars_per_token=4):
        """
        Search for similar documents.
        
        Args:
            query_embedding: Query embedding vector
            limit: Maximum number of chunks to return (chunk count limit)
            threshold: Minimum similarity score
            max_tokens: Optional maximum token count (if set, accumulates chunks until token budget reached)
            token_estimate_chars_per_token: Characters per token for estimation (default 4)
            
        Returns:
            List of result dicts with 'document', 'score', 'metadata', 'original_content'
        """
        import faiss
        
        # Check if index has documents
        if self.index.ntotal == 0 or len(self.documents) == 0:
            return []
        
        query_array = self.np.array([query_embedding], dtype='float32')
        faiss.normalize_L2(query_array)
        
        # Ensure k > 0 for FAISS
        # If max_tokens is set, we may need more chunks to accumulate tokens, so increase k
        # Otherwise use limit as-is
        k = min(limit if max_tokens is None else max(limit, limit * 2), len(self.documents))
        if k <= 0:
            return []
        
        # Search
        scores, indices = self.index.search(query_array, k)
        
        # Filter by threshold and accumulate results (with token awareness if max_tokens set)
        results = []
        cumulative_tokens = 0
        
        for score, idx in zip(scores[0], indices[0]):
            if idx < 0:  # FAISS returns -1 for no match
                continue
            if score < threshold:
                continue
            
            # Check token limit if max_tokens is set
            if max_tokens is not None:
                chunk_text = self.documents[idx]
                chunk_tokens = len(chunk_text) // token_estimate_chars_per_token
                
                # Stop if adding this chunk would exceed max_tokens
                if cumulative_tokens + chunk_tokens > max_tokens:
                    break
                
                cumulative_tokens += chunk_tokens
            
            results.append({
                'document': self.documents[idx],
                'score': float(score),
                'metadata': self.metadata[idx],
                'original_content': self.original_contents[idx]
            })
            
            # Stop if we've reached chunk limit (when not using max_tokens)
            if max_tokens is None and len(results) >= limit:
                break
        
        return results
    
    def save(self, path):
        """Persist to disk"""
        import faiss
        import pickle
        
        faiss.write_index(self.index, f"{path}.faiss")
        with open(f"{path}.meta", 'wb') as f:
            pickle.dump({
                'documents': self.documents,
                'metadata': self.metadata,
                'original_contents': self.original_contents
            }, f)
    
    def load(self, path):
        """Load from disk"""
        import faiss
        import pickle
        
        self.index = faiss.read_index(f"{path}.faiss")
        with open(f"{path}.meta", 'rb') as f:
            data = pickle.load(f)
            self.documents = data['documents']
            self.metadata = data['metadata']
            self.original_contents = data.get('original_contents', self.documents)


class ResourceIndexer:
    """
    Indexes Notes and Collections for semantic retrieval.
    
    Maintains separate FAISS indexes for Notes and Collections to enable
    type-filtered search. Stores embedding text in resource metadata for
    restart recovery.
    """
    
    def __init__(self, resource_manager, base_dir: Path):
        """
        Initialize resource indexer.
        
        Args:
            resource_manager: InfospaceResourceManager instance (for embedder and resource access)
            base_dir: Base directory for indexes (from resource_manager, e.g., scenarios/<world_name>/resources/)
        """
        self.resource_manager = resource_manager
        self.notes_index = FAISSStore(dimension=384, logger=logger)
        self.collections_index = FAISSStore(dimension=384, logger=logger)
        
        # Track resource_id -> index position for updates
        self.note_id_to_index = {}  # resource_id -> index position
        self.collection_id_to_index = {}  # resource_id -> index position
        
        # Track deleted resources (for filtering search results)
        self.deleted_note_ids = set()
        self.deleted_collection_ids = set()
        
        # Base directory for persistence (use same directory as resources)
        self.index_dir = base_dir
        self.index_dir.mkdir(parents=True, exist_ok=True)
    
    def _build_embedding_text_note(self, resource_id: str, resource_data: Dict, commentary: str = "") -> str:
        """
        Build embedding text for a Note.
        
        Format: "{name_or_id}\n{source_skill} on {source_value}\n{commentary}\n{content_preview}"
        """
        props = resource_data.get('properties', {})
        name = props.get('note_name') or resource_id
        source_skill = props.get('source_skill', '')
        source_value = props.get('source_value', '')
        content = props.get('content', '')
        
        # Build content preview (first 200 chars)
        if isinstance(content, (dict, list)):
            content_preview = json.dumps(content)[:200]
        else:
            content_preview = str(content)[:200]
        
        parts = [name]
        if source_skill:
            parts.append(f"{source_skill} on {source_value}" if source_value else source_skill)
        if commentary:
            parts.append(commentary)
        if content_preview:
            parts.append(content_preview)
        
        return "\n".join(parts)
    
    def _build_embedding_text_collection(self, resource_id: str, resource_data: Dict, commentary: str = "") -> str:
        """
        Build embedding text for a Collection.
        
        Format: "{name_or_id}\n{source_skill} on {source_value}\n{item_count} items\n{commentary}\n{first_3_summaries}"
        """
        props = resource_data.get('properties', {})
        name = props.get('collection_name') or resource_id
        source_skill = props.get('source_skill', '')
        source_value = props.get('source_value', '')
        item_count = props.get('item_count', 0)
        content = props.get('content', [])
        
        parts = [name]
        if source_skill:
            parts.append(f"{source_skill} on {source_value}" if source_value else source_skill)
        parts.append(f"{item_count} items")
        if commentary:
            parts.append(commentary)
        
        # Add first few item summaries if available
        if isinstance(content, list) and len(content) > 0:
            item_summaries = []
            for i, note_id in enumerate(content[:3]):
                if isinstance(note_id, str) and note_id.startswith('Note_'):
                    note = self.resource_manager.get_resource(note_id)
                    if note:
                        note_content = note.get('properties', {}).get('content', '')
                        if isinstance(note_content, (dict, list)):
                            preview = json.dumps(note_content)[:100]
                        else:
                            preview = str(note_content)[:100]
                        item_summaries.append(preview)
            if item_summaries:
                parts.append("Items: " + " | ".join(item_summaries))
        
        return "\n".join(parts)
    
    def index_note(self, resource_id: str, commentary: str = "") -> bool:
        """
        Index a Note resource.
        
        Args:
            resource_id: Note ID (e.g., "Note_42")
            commentary: Optional Stage 3 commentary to include
            
        Returns:
            True if indexed successfully
        """
        resource = self.resource_manager.get_resource(resource_id)
        if not resource:
            logger.warning(f"ResourceIndexer: Note {resource_id} not found")
            return False
        
        # Skip indexing if exclude_from_index flag is set
        if resource.get('properties', {}).get('exclude_from_index', False):
            logger.debug(f"ResourceIndexer: Skipping index for Note {resource_id} (exclude_from_index=True)")
            return False
        
        # Build embedding text
        embedding_text = self._build_embedding_text_note(resource_id, resource, commentary)
        
        # Store embedding text in resource properties for restart recovery
        if 'properties' not in resource:
            resource['properties'] = {}
        resource['properties']['embedding_text'] = embedding_text
        
        # Generate embedding
        self.resource_manager._init_embedder()
        embedding = self.resource_manager._generate_embedding(embedding_text)
        
        # Check if already indexed (update vs. add)
        if resource_id in self.note_id_to_index:
            # Update existing entry (FAISS doesn't support updates, so we'd need to rebuild)
            # For now, we'll just re-add (FAISS allows duplicates, we'll handle dedup in search)
            logger.debug(f"ResourceIndexer: Updating index for Note {resource_id}")
        
        # Add to index
        metadata = {
            'resource_id': resource_id,
            'name': resource.get('properties', {}).get('note_name', resource_id),
            'type': 'Note'
        }
        self.notes_index.add(embedding_text, embedding, metadata, original_content=resource)
        
        # Track index position
        self.note_id_to_index[resource_id] = len(self.notes_index.documents) - 1
        
        logger.debug(f"ResourceIndexer: Indexed Note {resource_id}")
        return True
    
    def index_collection(self, resource_id: str, commentary: str = "") -> bool:
        """
        Index a Collection resource.
        
        Args:
            resource_id: Collection ID (e.g., "Collection_15")
            commentary: Optional Stage 3 commentary to include
            
        Returns:
            True if indexed successfully
        """
        resource = self.resource_manager.get_resource(resource_id)
        if not resource:
            logger.warning(f"ResourceIndexer: Collection {resource_id} not found")
            return False
        
        # Build embedding text
        embedding_text = self._build_embedding_text_collection(resource_id, resource, commentary)
        
        # Store embedding text in resource properties for restart recovery
        if 'properties' not in resource:
            resource['properties'] = {}
        resource['properties']['embedding_text'] = embedding_text
        
        # Generate embedding
        self.resource_manager._init_embedder()
        embedding = self.resource_manager._generate_embedding(embedding_text)
        
        # Add to index
        metadata = {
            'resource_id': resource_id,
            'name': resource.get('properties', {}).get('collection_name', resource_id),
            'type': 'Collection',
            'item_count': resource.get('properties', {}).get('item_count', 0)
        }
        self.collections_index.add(embedding_text, embedding, metadata, original_content=resource)
        
        # Track index position
        self.collection_id_to_index[resource_id] = len(self.collections_index.documents) - 1
        
        logger.debug(f"ResourceIndexer: Indexed Collection {resource_id}")
        return True
    
    def search_notes(self, query: str, k: int = 3, threshold: float = 0.3) -> List[Dict]:
        """
        Search Notes index.
        
        Args:
            query: Search query text
            k: Number of results to return
            threshold: Minimum similarity score
            
        Returns:
            List of result dicts with resource_id, name, score, metadata
        """
        if self.notes_index.index.ntotal == 0:
            return []
        
        self.resource_manager._init_embedder()
        query_embedding = self.resource_manager._generate_embedding(query)
        
        results = self.notes_index.search(query_embedding, limit=k * 2, threshold=threshold)  # Get extra to filter deleted
        
        # Format results and filter deleted resources
        formatted = []
        for r in results:
            resource_id = r['metadata']['resource_id']
            # Skip deleted resources
            if resource_id in self.deleted_note_ids:
                continue
            # Verify resource still exists
            resource = self.resource_manager.get_resource(resource_id)
            if not resource:
                # Mark as deleted for future searches
                self.deleted_note_ids.add(resource_id)
                continue
            
            # Extract only serializable fields from resource (avoid ResourceType objects)
            resource_props = {}
            if resource:
                resource_props = resource.get('properties', {}).copy() if resource.get('properties') else {}
            
            formatted.append({
                'resource_id': resource_id,
                'name': r['metadata'].get('name', resource_id),
                'score': r['score'],
                'type': 'Note',
                'metadata': r['metadata'],
                'properties': resource_props  # Only include properties, not full resource with ResourceType
            })
            
            if len(formatted) >= k:
                break
        
        return formatted
    
    def search_collections(self, query: str, k: int = 2, threshold: float = 0.3) -> List[Dict]:
        """
        Search Collections index.
        
        Args:
            query: Search query text
            k: Number of results to return
            threshold: Minimum similarity score
            
        Returns:
            List of result dicts with resource_id, name, score, metadata
        """
        if self.collections_index.index.ntotal == 0:
            return []
        
        self.resource_manager._init_embedder()
        query_embedding = self.resource_manager._generate_embedding(query)
        
        results = self.collections_index.search(query_embedding, limit=k * 2, threshold=threshold)  # Get extra to filter deleted
        
        # Format results and filter deleted resources
        formatted = []
        for r in results:
            resource_id = r['metadata']['resource_id']
            # Skip deleted resources
            if resource_id in self.deleted_collection_ids:
                continue
            # Verify resource still exists
            resource = self.resource_manager.get_resource(resource_id)
            if not resource:
                # Mark as deleted for future searches
                self.deleted_collection_ids.add(resource_id)
                continue
            
            # Extract only serializable fields from resource (avoid ResourceType objects)
            resource_props = {}
            if resource:
                resource_props = resource.get('properties', {}).copy() if resource.get('properties') else {}
            
            formatted.append({
                'resource_id': resource_id,
                'name': r['metadata'].get('name', resource_id),
                'score': r['score'],
                'type': 'Collection',
                'item_count': r['metadata'].get('item_count', 0),
                'metadata': r['metadata'],
                'properties': resource_props  # Only include properties, not full resource with ResourceType
            })
            
            if len(formatted) >= k:
                break
        
        return formatted
    
    def save_indexes(self, world_name: str):
        """Save indexes to disk."""
        notes_path = self.index_dir / f"{world_name}_notes.index"
        collections_path = self.index_dir / f"{world_name}_collections.index"
        
        try:
            self.notes_index.save(str(notes_path))
            self.collections_index.save(str(collections_path))
            logger.info(f"ResourceIndexer: Saved indexes to {self.index_dir}")
        except Exception as e:
            logger.error(f"ResourceIndexer: Failed to save indexes: {e}")
    
    def load_indexes(self, world_name: str):
        """Load indexes from disk."""
        notes_path = self.index_dir / f"{world_name}_notes.index"
        collections_path = self.index_dir / f"{world_name}_collections.index"
        
        try:
            if notes_path.with_suffix('.faiss').exists():
                self.notes_index.load(str(notes_path))
                # Rebuild id_to_index mapping
                for i, doc in enumerate(self.notes_index.documents):
                    if i < len(self.notes_index.metadata):
                        resource_id = self.notes_index.metadata[i].get('resource_id')
                        if resource_id:
                            self.note_id_to_index[resource_id] = i
                logger.info(f"ResourceIndexer: Loaded notes index ({len(self.notes_index.documents)} entries)")
            
            if collections_path.with_suffix('.faiss').exists():
                self.collections_index.load(str(collections_path))
                # Rebuild id_to_index mapping
                for i, doc in enumerate(self.collections_index.documents):
                    if i < len(self.collections_index.metadata):
                        resource_id = self.collections_index.metadata[i].get('resource_id')
                        if resource_id:
                            self.collection_id_to_index[resource_id] = i
                logger.info(f"ResourceIndexer: Loaded collections index ({len(self.collections_index.documents)} entries)")
        except Exception as e:
            logger.warning(f"ResourceIndexer: Failed to load indexes: {e}")
    
    def reindex_persistent_resources(self):
        """
        Re-index persistent resources that are missing from loaded indexes.
        Called on restart to fill gaps (avoids double-indexing).
        """
        reindexed = 0
        for resource_id, resource_data in self.resource_manager.resource_registry.items():
            resource_type = resource_data.get('type')
            type_name = resource_type.name if hasattr(resource_type, 'name') else str(resource_type)
            
            if type_name == 'Note':
                props = resource_data.get('properties', {})
                if props.get('persistent', False):
                    # Only re-index if not already in loaded index
                    if resource_id not in self.note_id_to_index:
                        embedding_text = props.get('embedding_text')
                        if embedding_text:
                            # Re-index using saved text
                            self.resource_manager._init_embedder()
                            embedding = self.resource_manager._generate_embedding(embedding_text)
                            metadata = {
                                'resource_id': resource_id,
                                'name': props.get('note_name', resource_id),
                                'type': 'Note'
                            }
                            self.notes_index.add(embedding_text, embedding, metadata, original_content=resource_data)
                            self.note_id_to_index[resource_id] = len(self.notes_index.documents) - 1
                            reindexed += 1
            
            elif type_name == 'Collection':
                props = resource_data.get('properties', {})
                if props.get('persistent', False):
                    # Only re-index if not already in loaded index
                    if resource_id not in self.collection_id_to_index:
                        embedding_text = props.get('embedding_text')
                        if embedding_text:
                            # Re-index using saved text
                            self.resource_manager._init_embedder()
                            embedding = self.resource_manager._generate_embedding(embedding_text)
                            metadata = {
                                'resource_id': resource_id,
                                'name': props.get('collection_name', resource_id),
                                'type': 'Collection',
                                'item_count': props.get('item_count', 0)
                            }
                            self.collections_index.add(embedding_text, embedding, metadata, original_content=resource_data)
                            self.collection_id_to_index[resource_id] = len(self.collections_index.documents) - 1
                            reindexed += 1
        
        logger.info(f"ResourceIndexer: Re-indexed {reindexed} missing persistent resources")


class InfospaceResourceManager:
    """
    Manages Notes, Collections, Relations, and indexing for infospace maps.
    
    Responsibilities:
    - Note and Collection creation
    - Collection mutation (add/remove items)
    - Vector store indexing
    - Semantic search
    - Persistence helpers
    """
    
    def __init__(self, world_name: str, session=None, world_config: Optional[Dict] = None, agent_name: Optional[str] = None):
        """
        Initialize the resource manager.
        
        Args:
            world_name: Name of the world (for persistence/index naming)
            session: Optional Zenoh session for save_all subscriber
            world_config: Optional world config dict with 'world_name' field for world-specific directory
            agent_name: Optional agent name to scope resources per-agent
        """
        self.world_name = world_name
        self.agent_name = agent_name
        self.resource_types = ResourceTypeRegistry(InfospaceResources)
        self.resource_registry: Dict[str, Dict[str, Any]] = {}
        
        # Resource counters
        self.note_counter = 0
        self.collection_counter = 0
        self.relation_counter = 0
        
        # Named resource registries
        self.named_collections: Dict[str, str] = {}
        self.named_notes: Dict[str, str] = {}
        
        # Indexing infrastructure
        self.collection_indexes: Dict[str, str] = {}  # collection_id -> collection_id (marks indexed)
        self.vector_stores: Dict[str, FAISSStore] = {}  # collection_id -> FAISSStore
        
        # Embedding model (lazy loaded)
        self.embedder = None

        # When True, skip embedding on note/collection creation (batch-index after goal cleanup)
        self.indexing_deferred = False
        self._deferred_note_ids: list = []
        
        # Determine base directory for resources and indexes
        # If world_config.world_name is set, use scenarios/<world_name>/resources/
        # Otherwise, use data/resources/ and data/vector/
        # Paths are relative to project root (where scenarios/ directory exists)
        if world_config and world_config.get('world_name'):
            world_dir_name = world_config['world_name']
            # Find project root (where scenarios/ directory exists)
            # Start from this file's directory and walk up to find scenarios/
            current_dir = Path(__file__).parent
            project_root = current_dir.parent  # src/ -> project root
            self.base_dir = project_root / "scenarios" / world_dir_name / "resources"
        else:
            # Find project root for data/ directory
            current_dir = Path(__file__).parent
            project_root = current_dir.parent  # src/ -> project root
            self.base_dir = project_root / "data" / "resources"
        
        # Scope per agent if agent_name provided
        if self.agent_name:
            self.base_dir = self.base_dir / self.agent_name
        
        self.base_dir.mkdir(parents=True, exist_ok=True)
        
        # File persistence path
        self.resources_file = self.base_dir / "resources.json"
        
        # Resource indexer for Stage 0 retrieval (pass base_dir for indexes)
        self.resource_indexer = ResourceIndexer(self, self.base_dir)
        
        # Zenoh session (save_all handled by executive_node, not here to avoid duplicate saves)
        self.session = session
        
        # Create Note_null system resource (required global unique ID)
        self._create_note_null()
    
    def _create_note_null(self):
        """Create the distinguished Note_null system resource (required global unique ID)."""
        if 'Note_null' in self.resource_registry:
            return  # Already exists
        
        try:
            note_type = self.resource_types.Note
        except AttributeError:
            logger.warning("Note resource type not available, cannot create Note_null")
            return
        
        note_null_data = {
            'name': 'Note_null',
            'type': note_type,
            'location': (0, 0),
            'description': 'System null Note resource (sentinel for empty/null values)',
            'remove_on_take': False,
            'properties': {
                'content': None,
                'format': 'text',
                'created_by': 'System',
                'created_at': datetime.now().isoformat(),
                'source_skill': 'system',
                'source_value': '',
                'note_name': 'null',
                'is_system_resource': True
            }
        }
        
        self.resource_registry['Note_null'] = note_null_data
        logger.info("📝 Created system Note_null resource")

    def _sync_counters_from_registry(self):
        """Ensure counters match highest existing Note_/Collection_/Relation_ IDs."""
        max_note = 0
        max_collection = 0
        max_relation = 0
        for resource_id in self.resource_registry.keys():
            if resource_id.startswith('Note_'):
                suffix = resource_id.split('_', 1)[1]
                if suffix.isdigit():
                    max_note = max(max_note, int(suffix))
            elif resource_id.startswith('Collection_'):
                suffix = resource_id.split('_', 1)[1]
                if suffix.isdigit():
                    max_collection = max(max_collection, int(suffix))
            elif resource_id.startswith('Relation_'):
                suffix = resource_id.split('_', 1)[1]
                if suffix.isdigit():
                    max_relation = max(max_relation, int(suffix))
        if max_note != self.note_counter or max_collection != self.collection_counter or max_relation != self.relation_counter:
            logger.info(f"🔢 Resynced counters (notes={max_note}, collections={max_collection}, relations={max_relation})")
        self.note_counter = max_note
        self.collection_counter = max_collection
        self.relation_counter = max_relation

    # ==================== Core Accessors ====================

    def get_resource(self, name_or_id: Optional[str]) -> Optional[Dict[str, Any]]:
        """Retrieve a resource by ID, stable name, or canonicalized label."""
        if not name_or_id:
            return None

        resource_id = self._resolve_resource_id(name_or_id)
        if resource_id:
            return self.resource_registry.get(resource_id)
        return None

    def get_resource_list(self) -> List[Dict[str, Any]]:
        """Return a list of shallow copies of every registered resource."""
        resource_list = []
        for resource_id, resource_data in self.resource_registry.items():
            item = dict(resource_data)
            item['id'] = resource_id
            resource_list.append(item)
        return resource_list

    def delete_resource(self, resource_id: str) -> Tuple[bool, Optional[str]]:
        """Remove a resource and clean up associated indexes/mappings."""
        if not resource_id:
            return False, "Missing resource_id"

        resource = self.resource_registry.pop(resource_id, None)
        if not resource:
            return False, f"Resource {resource_id} not found"

        resource_type = resource.get('type')
        type_name = getattr(resource_type, 'name', str(resource_type))

        if resource_id.startswith('Collection_'):
            self._cleanup_collection(resource_id)
        elif resource_id.startswith('Note_'):
            self._cleanup_note(resource_id)
        elif resource_id.startswith('Relation_'):
            self._cleanup_relation(resource_id)

        # Cascade-delete any Relation touching this Note/Collection.
        # Rule: relation is removed whenever either endpoint is removed.
        if resource_id.startswith('Note_') or resource_id.startswith('Collection_'):
            self._remove_relations_for_endpoint(resource_id)

        # Remove from name registries
        self._remove_named_reference(resource_id, type_name, resource)

        # Remove from vector/semantic indexes
        self.remove_resource_from_index(resource_id)

        return True, None

    def _resolve_resource_id(self, name_or_id: str) -> Optional[str]:
        """Map IDs or friendly names to canonical resource IDs."""
        if name_or_id in self.resource_registry:
            return name_or_id
        if name_or_id in self.named_notes:
            return self.named_notes[name_or_id]
        if name_or_id in self.named_collections:
            return self.named_collections[name_or_id]

        canonical = name_or_id.capitalize()
        matches = []
        for resource_id, resource_data in self.resource_registry.items():
            matched = False
            if str(resource_data.get('name', '')).capitalize() == canonical:
                matched = True
            if not matched:
                props = resource_data.get('properties', {})
                note_name = props.get('note_name')
                if note_name and str(note_name).capitalize() == canonical:
                    matched = True
                collection_name = props.get('collection_name')
                if not matched and collection_name and str(collection_name).capitalize() == canonical:
                    matched = True
                relation_name = props.get('relation_name')
                if not matched and relation_name and str(relation_name).capitalize() == canonical:
                    matched = True
            if matched:
                matches.append(resource_id)
        if not matches:
            return None
        if len(matches) == 1:
            return matches[0]
        # Multiple matches: return the most recently created (highest numeric suffix)
        def _numeric_suffix(rid):
            parts = rid.rsplit('_', 1)
            try:
                return int(parts[-1])
            except (ValueError, IndexError):
                return -1
        return max(matches, key=_numeric_suffix)

    def _cleanup_collection(self, collection_id: str):
        """Remove tracking state for a collection."""
        name_to_remove = None
        for name, cid in self.named_collections.items():
            if cid == collection_id:
                name_to_remove = name
                break
        if name_to_remove:
            del self.named_collections[name_to_remove]

        self.vector_stores.pop(collection_id, None)
        self.collection_indexes.pop(collection_id, None)

    def _cleanup_note(self, note_id: str):
        """Remove a note from named registries and collections."""
        name_to_remove = None
        for name, nid in self.named_notes.items():
            if nid == note_id:
                name_to_remove = name
                break
        if name_to_remove:
            del self.named_notes[name_to_remove]

        # Remove from all collections
        for resource_data in self.resource_registry.values():
            props = resource_data.get('properties', {})
            content = props.get('content')
            if isinstance(content, list) and note_id in content:
                props['content'] = [nid for nid in content if nid != note_id]
                props['item_count'] = len(props['content'])

    def _cleanup_relation(self, relation_id: str):
        """No-op hook for relation cleanup symmetry."""
        return

    def _remove_relations_for_endpoint(self, endpoint_id: str):
        """Remove relations where endpoint_id is source or target."""
        relation_ids = []
        for rid, rdata in self.resource_registry.items():
            if not isinstance(rid, str) or not rid.startswith('Relation_'):
                continue
            props = rdata.get('properties', {})
            if props.get('source') == endpoint_id or props.get('target') == endpoint_id:
                relation_ids.append(rid)

        for relation_id in relation_ids:
            # Direct pop avoids recursive cascade work for relation objects.
            relation_resource = self.resource_registry.pop(relation_id, None)
            if relation_resource:
                self._remove_named_reference(relation_id, 'Relation', relation_resource)
                logger.info(f"Removed relation {relation_id} due to endpoint deletion: {endpoint_id}")

    def _remove_named_reference(self, resource_id: str, type_name: str, resource: Dict[str, Any]):
        """Best-effort cleanup of named lookups."""
        if type_name == 'Collection':
            props = resource.get('properties', {})
            collection_name = props.get('collection_name')
            if collection_name and self.named_collections.get(collection_name) == resource_id:
                del self.named_collections[collection_name]
        elif type_name == 'Note':
            props = resource.get('properties', {})
            note_name = props.get('note_name')
            if note_name and self.named_notes.get(note_name) == resource_id:
                del self.named_notes[note_name]
    
    def update_resource_commentary(self, resource_id: str, commentary: str):
        """
        Update index with Stage 3 commentary for a resource.
        
        Args:
            resource_id: Note or Collection ID
            commentary: Stage 3 reflection commentary
        """
        if resource_id.startswith('Note_'):
            self.resource_indexer.index_note(resource_id, commentary=commentary)
        elif resource_id.startswith('Collection_'):
            self.resource_indexer.index_collection(resource_id, commentary=commentary)
        else:
            logger.warning(f"update_resource_commentary: Invalid resource_id format: {resource_id}")
    
    def remove_resource_from_index(self, resource_id: str):
        """
        Mark a resource as deleted in the index (for filtering search results).
        
        Args:
            resource_id: Note or Collection ID to mark as deleted
        """
        if resource_id.startswith('Note_'):
            self.resource_indexer.deleted_note_ids.add(resource_id)
            # Remove from tracking
            if resource_id in self.resource_indexer.note_id_to_index:
                del self.resource_indexer.note_id_to_index[resource_id]
        elif resource_id.startswith('Collection_'):
            self.resource_indexer.deleted_collection_ids.add(resource_id)
            # Remove from tracking
            if resource_id in self.resource_indexer.collection_id_to_index:
                del self.resource_indexer.collection_id_to_index[resource_id]
    
    # Shared embedder instance across all ResourceManager instances (thread-safe).
    # SentenceTransformer is not safe to initialize concurrently from multiple
    # threads — use a class-level lock and singleton to prevent races.
    _shared_embedder = None
    _embedder_lock = threading.Lock()

    def _init_embedder(self):
        """Lazy init embedding model (shared singleton across all agents)."""
        if self.embedder is not None:
            return
        with InfospaceResourceManager._embedder_lock:
            # Double-check after acquiring lock
            if InfospaceResourceManager._shared_embedder is not None:
                self.embedder = InfospaceResourceManager._shared_embedder
                return
            # Recommended: BGE-Small (33MB, much smarter than MiniLM, 512 context)
            model_name = 'BAAI/bge-small-en-v1.5'
            from sentence_transformers import SentenceTransformer
            # Use GPU if available for ~10x faster embedding
            import torch
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            try:
                embedder = SentenceTransformer(model_name, local_files_only=True, device=device)
                logger.info(f"Initialized embedding model: {model_name} (from cache, device={device})")
            except Exception:
                logger.info(f"Cache miss, downloading embedding model: {model_name}")
                embedder = SentenceTransformer(model_name, device=device)
                logger.info(f"Initialized embedding model: {model_name} (downloaded, device={device})")
            InfospaceResourceManager._shared_embedder = embedder
            self.embedder = embedder
    
    # ==================== Note Creation ====================
    
    def create_note(self, character_name: str, content: Any, format_type: str,
                   source_skill: str, source_value: str, note_name: str,
                   extra_props: Dict) -> Tuple[bool, Optional[str], Optional[str], Optional[Tuple[int, int]]]:
        """
        Create a Note resource.
        
        Args:
            character_name: Agent creating the note
            content: Note content (any type)
            format_type: Content format ('text' or 'json')
            source_skill: Skill that created this note
            source_value: Input value to the skill
            note_name: Optional stable name for referencing
            extra_props: Extra metadata properties
            
        Returns:
            Tuple of (success, note_id, error_msg, location)
        """
        # Validate inputs
        if not character_name:
            return False, None, "Missing character_name", None
        if content is None:
            return False, None, "Missing content", None
        
        canonical_character_name = character_name.capitalize()
        location = (0, 0)  # Spatial coordinates removed in infospace mode
        
        # Generate unique Note ID
        self.note_counter += 1
        note_id = f"Note_{self.note_counter}"
        
        # Get Note type from resource registry
        try:
            note_type = self.resource_types.Note
        except AttributeError:
            return False, None, "Note resource type not available in this map", None
        
        # Normalize content to string (text-only model: no native dict/list in storage)
        if isinstance(content, (dict, list)):
            stored_content = json.dumps(content, sort_keys=True)
        else:
            stored_content = str(content) if content is not None else ""
        # Build Note resource data structure
        note_data = {
            'name': note_id,
            'type': note_type,
            'location': location,
            'description': f"Note artifact created by {source_skill}",
            'remove_on_take': False,
            'properties': {
                'content': stored_content,
                'format': format_type,
                'created_by': canonical_character_name,
                'created_at': datetime.now().isoformat(),
                'source_skill': source_skill,
                'source_value': source_value,
                'note_name': note_name
            }
        }
        
        # Compute metadata
        try:
            content_str = stored_content
            note_data['properties']['content_type'] = 'json' if isinstance(content, (dict, list)) else 'text'
            note_data['properties']['length'] = len(content_str)
            note_data['properties']['fingerprint'] = hashlib.sha1(content_str.encode('utf-8')).hexdigest()
        except Exception as e:
            logger.warning(f"Failed to compute Note metadata: {e}")
        
        # Merge allowed extra properties (tool_metadata: internal only, planner/tools have no access)
        allowed_fields = {'kind', 'category', 'status', 'provenance', 'last_engaged_at',
                          'last_acted_at', 'last_fired_at',  # last_fired_at retained for legacy reads
                          'cadence_hours', 'cadence_days',
                          'lifetime_days', 'instruction', 'seed',
                          # Phase-A asymmetric concern dynamics — agent_concerns
                          # use activation/rhythm_hours/rhythm_source/last_activation_update_at,
                          # user_concerns use strength/last_bumped_at, and
                          # successor concerns use successor_of/successor_depth.
                          # Without these the fields are silently dropped at
                          # create_note time and notes load with activation=null.
                          'activation', 'last_activation_update_at',
                          'rhythm_hours', 'rhythm_source',
                          'strength', 'last_bumped_at',
                          'successor_of', 'successor_depth',
                          # reasoning_history (awareness feed) fields
                          'user_text', 'autonomous', 'exit_reason', 'n_iters', 'compressed',
                          'parent_id', 'order', 'span', 'section', 'source', 'entity', 'edge',
                          'exclude_from_index', 'tool_metadata', 'placeholder'}
        for key in allowed_fields:
            if key in extra_props:
                note_data['properties'][key] = extra_props[key]
        
        # Register in registry
        self.resource_registry[note_id] = note_data
        
        # Register named note if name provided, deleting any prior note with same name
        if note_name:
            old_id = self.named_notes.get(note_name)
            if old_id and old_id != note_id and old_id in self.resource_registry:
                logger.info(f"📝 Replacing named Note '{note_name}': {old_id} → {note_id}")
                self.delete_resource(old_id)
            self.named_notes[note_name] = note_id
            logger.info(f"📝 Created named Note: '{note_name}' = {note_id} by {canonical_character_name}")
        else:
            logger.info(f"📝 Created Note instance: {note_id} by {canonical_character_name}")
        
        # Index the Note immediately (for Stage 0 retrieval) unless excluded or deferred
        if not note_data['properties'].get('exclude_from_index', False):
            if self.indexing_deferred:
                self._deferred_note_ids.append(note_id)
            else:
                self.resource_indexer.index_note(note_id, commentary="")

        # Metadata reification: tool_metadata is stored as a separate metadata Note
        # linked by a typed Relation (note -> metadata note, relation_type='meta').
        tool_metadata = note_data['properties'].get('tool_metadata')
        if isinstance(tool_metadata, dict) and tool_metadata:
            self._upsert_note_tool_metadata_relation(note_id, tool_metadata, canonical_character_name)
        
        return True, note_id, None, location

    def flush_deferred_indexes(self):
        """Index deferred notes that survived goal cleanup. Call after _cleanup_transient_resources."""
        if not self._deferred_note_ids:
            self.indexing_deferred = False
            return
        import time as _time
        _t0 = _time.monotonic()
        indexed = 0
        for note_id in self._deferred_note_ids:
            if note_id in self.resource_registry:
                self.resource_indexer.index_note(note_id, commentary="")
                indexed += 1
        total = len(self._deferred_note_ids)
        elapsed = _time.monotonic() - _t0
        self._deferred_note_ids.clear()
        self.indexing_deferred = False
        msg = f"INDEX | batch-indexed {indexed}/{total} surviving notes in {elapsed:.1f}s"
        logger.info(msg)
        print(msg, flush=True)

    def update_note_content(self, note_id: str, new_content: Any, reindex: bool = True,
                           tool_metadata: Optional[Dict] = None) -> Tuple[bool, Optional[str]]:
        """
        Update the content of an existing Note.
        
        Args:
            note_id: Note ID to update
            new_content: New content to set
            reindex: Whether to re-index the Note after update
            tool_metadata: Optional tool metadata to merge (internal only)
            
        Returns:
            Tuple of (success, error_msg)
        """
        if not note_id.startswith('Note_'):
            return False, f"Invalid note_id format: {note_id}"
        
        # Get existing Note
        note_data = self.resource_registry.get(note_id)
        if not note_data:
            return False, f"Note {note_id} not found"
        
        # Normalize content to string (text-only model)
        if isinstance(new_content, (dict, list)):
            stored_content = json.dumps(new_content, sort_keys=True)
            note_data['properties']['format'] = 'json'
            note_data['properties']['content_type'] = 'json'
        else:
            stored_content = str(new_content) if new_content is not None else ""
            note_data['properties']['format'] = 'text'
            note_data['properties']['content_type'] = 'text'
        note_data['properties']['content'] = stored_content
        if tool_metadata is not None:
            note_data['properties']['tool_metadata'] = tool_metadata
            if isinstance(tool_metadata, dict) and tool_metadata:
                self._upsert_note_tool_metadata_relation(note_id, tool_metadata, note_data['properties'].get('created_by', 'System'))
        content_str = stored_content
        
        # Update metadata
        note_data['properties']['length'] = len(content_str)
        note_data['properties']['fingerprint'] = hashlib.sha1(content_str.encode('utf-8')).hexdigest()
        
        # Re-index if requested and Note was previously indexed
        if reindex and note_id in self.resource_indexer.note_id_to_index:
            self.resource_indexer.index_note(note_id, commentary="")
        
        logger.info(f"📝 Updated Note {note_id} content")
        return True, None

    def upsert_metadata(self, resource_id: str, meta_text: str, character_name: str) -> Optional[str]:
        """Create or update a 'meta' relation from resource_id to a metadata Note.

        Works for both Notes and Collections. Returns the metadata Note ID, or None on failure.
        """
        if not isinstance(resource_id, str) or not (
            resource_id.startswith('Note_') or resource_id.startswith('Collection_')
        ):
            return None

        metadata_note_id = None
        for rel in self.find_relations(source_id=resource_id, relation_type='meta'):
            target_id = rel.get('properties', {}).get('target')
            if isinstance(target_id, str) and target_id.startswith('Note_') and target_id in self.resource_registry:
                metadata_note_id = target_id
                break

        if metadata_note_id:
            self.update_note_content(metadata_note_id, meta_text, reindex=False)
            return metadata_note_id
        else:
            success, metadata_note_id, error_msg, _ = self.create_note(
                character_name=character_name,
                content=meta_text,
                format_type='text',
                source_skill='meta-relation',
                source_value=resource_id,
                note_name='',
                extra_props={'kind': 'metadata', 'source': resource_id, 'exclude_from_index': True}
            )
            if not success or not metadata_note_id:
                logger.warning(f"Failed creating metadata Note for {resource_id}: {error_msg}")
                return None
            rel_success, relation_id, rel_error, _ = self.create_relation(
                character_name=character_name,
                source_id=resource_id,
                target_id=metadata_note_id,
                relation_type='meta',
                extra_props={'exclude_from_index': True}
            )
            if not rel_success:
                logger.warning(f"Failed creating meta relation for {resource_id}: {rel_error}")
                return None
            logger.info(f"Linked metadata for {resource_id} via {relation_id}")
            return metadata_note_id

    def get_metadata_note_id(self, resource_id: str) -> Optional[str]:
        """Return the metadata Note ID linked via 'meta' relation, or None."""
        for rel in self.find_relations(source_id=resource_id, relation_type='meta'):
            target_id = rel.get('properties', {}).get('target')
            if isinstance(target_id, str) and target_id.startswith('Note_') and target_id in self.resource_registry:
                return target_id
        return None

    def _upsert_note_tool_metadata_relation(self, note_id: str, tool_metadata: Dict[str, Any], character_name: str):
        """Create/update Note->meta Note relation for tool metadata. Delegates to upsert_metadata."""
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            return
        if not isinstance(tool_metadata, dict) or not tool_metadata:
            return

        meta_text = json.dumps(tool_metadata, ensure_ascii=False, sort_keys=True)
        metadata_note_id = self.upsert_metadata(note_id, meta_text, character_name)
        if not metadata_note_id:
            return

        source_note = self.resource_registry.get(note_id, {})
        source_persistent = bool(source_note.get('properties', {}).get('persistent', False))
        if source_persistent:
            self.mark_persistent(metadata_note_id, character_name)
    
    # ==================== Collection Creation ====================
    
    def create_collection(self, character_name: str, content: Any, format_type: str,
                         source_skill: str, source_value: str, collection_name: str,
                         extra_props: Dict) -> Tuple[bool, Optional[str], Optional[str], Optional[Tuple[int, int]]]:
        """
        Create a Collection resource.
        
        Args:
            character_name: Agent creating the collection (None for system collections)
            content: Collection content (list or dict)
            format_type: Content format ('list' or 'dict')
            source_skill: Skill that created this collection
            source_value: Input value to the operation
            collection_name: Optional stable name for referencing
            extra_props: Extra metadata properties
            
        Returns:
            Tuple of (success, collection_id, error_msg, location)
        """
        # Validate inputs
        if content is None:
            content = []
        
        # Handle system collections (character_name=None)
        if character_name is None:
            canonical_character_name = 'System'
        else:
            if not character_name:
                return False, None, "Missing character_name", None
            canonical_character_name = character_name.capitalize()
        location = (0, 0)  # Spatial coordinates removed in infospace mode
        
        # Generate unique Collection ID
        self.collection_counter += 1
        collection_id = f"Collection_{self.collection_counter}"
        
        # Get Collection type from resource registry
        try:
            collection_type = self.resource_types.Collection
        except AttributeError:
            return False, None, "Collection resource type not available in this map", None
        
        # Build Collection resource data structure
        collection_data = {
            'name': collection_id,
            'type': collection_type,
            'location': location,
            'description': f"Collection artifact{' created by ' + source_skill if source_skill else ''}",
            'remove_on_take': False,
            'properties': {
                'content': content,
                'format': format_type,
                'created_by': canonical_character_name,
                'created_at': datetime.now().isoformat(),
                'source_skill': source_skill,
                'source_value': source_value,
                'item_count': len(content) if isinstance(content, (list, dict)) else 0,
                'collection_name': collection_name
            }
        }
        
        # Merge allowed extra properties
        allowed_fields = {'kind', 'doc_meta', 'chunking', 'indexes'}
        for key in allowed_fields:
            if key in extra_props:
                collection_data['properties'][key] = extra_props[key]
        
        # Set chunk_count for document kind
        if collection_data['properties'].get('kind') == 'document':
            collection_data['properties']['chunk_count'] = collection_data['properties'].get('item_count', 0)
        
        # Register in registry
        self.resource_registry[collection_id] = collection_data
        
        # Register named collection if name provided
        if collection_name:
            self.named_collections[collection_name] = collection_id
            logger.info(f"📚 Created named Collection: '{collection_name}' = {collection_id} by {canonical_character_name} ({collection_data['properties']['item_count']} items)")
        else:
            logger.info(f"📚 Created Collection instance: {collection_id} by {canonical_character_name} ({collection_data['properties']['item_count']} items)")
        
        # Index the Collection immediately (for Stage 0 retrieval)
        self.resource_indexer.index_collection(collection_id, commentary="")
        
        return True, collection_id, None, location

    # ==================== Relation Creation ====================

    def create_relation(self, character_name: str, source_id: str, target_id: str, relation_type: str,
                        relation_name: str = '', extra_props: Optional[Dict[str, Any]] = None) -> Tuple[bool, Optional[str], Optional[str], Optional[Tuple[int, int]]]:
        """
        Create a Relation resource (typed directed edge): source -> target.

        Args:
            character_name: Agent creating the relation
            source_id: Source Note/Collection ID
            target_id: Target Note/Collection ID
            relation_type: Relation type label (e.g., 'meta')
            relation_name: Optional stable name
            extra_props: Optional extra properties

        Returns:
            Tuple of (success, relation_id, error_msg, location)
        """
        if not character_name:
            return False, None, "Missing character_name", None
        if not isinstance(source_id, str) or not source_id:
            return False, None, "Missing source_id", None
        if not isinstance(target_id, str) or not target_id:
            return False, None, "Missing target_id", None
        if not isinstance(relation_type, str) or not relation_type.strip():
            return False, None, "Missing relation_type", None
        if source_id not in self.resource_registry:
            return False, None, f"Source resource not found: {source_id}", None
        if target_id not in self.resource_registry:
            return False, None, f"Target resource not found: {target_id}", None

        canonical_character_name = character_name.capitalize()
        location = (0, 0)
        self.relation_counter += 1
        relation_id = f"Relation_{self.relation_counter}"

        try:
            relation_resource_type = self.resource_types.Relation
        except AttributeError:
            return False, None, "Relation resource type not available in this map", None

        relation_props = {
            'source': source_id,
            'target': target_id,
            'relation_type': relation_type.strip(),
            'content': json.dumps({'source': source_id, 'target': target_id, 'type': relation_type.strip()}, sort_keys=True),
            'format': 'text',
            'created_by': canonical_character_name,
            'created_at': datetime.now().isoformat(),
            'relation_name': relation_name
        }
        if isinstance(extra_props, dict):
            relation_props.update(extra_props)

        relation_data = {
            'name': relation_id,
            'type': relation_resource_type,
            'location': location,
            'description': f"Relation {source_id} -[{relation_type.strip()}]-> {target_id}",
            'remove_on_take': False,
            'properties': relation_props
        }

        self.resource_registry[relation_id] = relation_data
        logger.info(f"🔗 Created Relation {relation_id}: {source_id} -[{relation_type.strip()}]-> {target_id}")
        return True, relation_id, None, location

    def find_relations(self, source_id: Optional[str] = None, target_id: Optional[str] = None,
                       relation_type: Optional[str] = None) -> List[Dict[str, Any]]:
        """Find relation resources matching optional source/target/type filters."""
        matches: List[Dict[str, Any]] = []
        for resource_id, resource_data in self.resource_registry.items():
            if not resource_id.startswith('Relation_'):
                continue
            props = resource_data.get('properties', {})
            if source_id and props.get('source') != source_id:
                continue
            if target_id and props.get('target') != target_id:
                continue
            if relation_type and props.get('relation_type') != relation_type:
                continue
            matches.append({'id': resource_id, 'properties': props})
        return matches
    
    # ==================== Collection Mutation ====================
    
    def add_to_collection(self, collection_id: str, note_id: Optional[str],
                         agent_name: str, operation: str = 'add',
                         content: Optional[List] = None) -> Tuple[bool, Optional[int], Optional[str]]:
        """
        Add a note to a collection or update collection content.
        
        Args:
            collection_id: Collection to modify
            note_id: Note to add (for 'add' operation)
            agent_name: Agent performing the operation
            operation: 'add' or 'update'
            content: New content (for 'update' operation)
            
        Returns:
            Tuple of (success, item_count, error_msg)
        """
        # Fetch Collection
        collection = self.resource_registry.get(collection_id)
        if not collection:
            return False, None, f"Collection {collection_id} not found"
        
        # Get content list
        content_list = collection['properties'].get('content', [])
        if not isinstance(content_list, list):
            return False, None, f"Collection {collection_id} content is not a list"
        
        if operation == 'update' and content is not None:
            # Update: replace entire content
            if not isinstance(content, list):
                return False, None, "Update content must be a list"
            collection['properties']['content'] = content
            collection['properties']['item_count'] = len(content)
            logger.info(f"📚 Updated {collection_id} content (now {len(content)} items) by {agent_name}")
        else:
            # Add: append single note
            if not note_id:
                return False, None, "Missing note_id for add operation"
            content_list.append(note_id)
            collection['properties']['item_count'] = len(content_list)
            logger.info(f"📚 Added {note_id} to {collection_id} (now {len(content_list)} items) by {agent_name}")
            
            # Auto-update associated index (if Collection is indexed)
            if collection_id in self.collection_indexes:
                if collection_id in self.vector_stores:
                    store = self.vector_stores[collection_id]
                    chunks_indexed = self._index_note_chunks(note_id, collection_id, store, fields={}, index_type='semantic')
                    if chunks_indexed > 0:
                        logger.info(f"  ↳ Updated index for {collection_id} with {chunks_indexed} chunks from {note_id}")
                    else:
                        logger.warning(f"  ↳ Failed to index {note_id} in {collection_id}")
        
        return True, collection['properties']['item_count'], None
    
    def mark_persistent(self, resource_id: str, character_name: str) -> Tuple[bool, Optional[str]]:
        """
        Mark a Note, Collection, or Relation as persistent.
        If Collection, also persists all Notes in it that aren't already persistent.
        
        Args:
            resource_id: Resource to mark as persistent
            character_name: Agent performing the operation
            
        Returns:
            Tuple of (success, error_msg)
        """
        # Fetch resource
        resource = self.resource_registry.get(resource_id)
        if not resource:
            return False, f"Resource {resource_id} not found"
        
        # If Collection, persist all Notes in it first
        if resource_id.startswith('Collection_'):
            content = resource.get('properties', {}).get('content', [])
            persisted_count = 0
            for note_id in content:
                if isinstance(note_id, str) and note_id.startswith('Note_'):
                    note_resource = self.resource_registry.get(note_id)
                    if note_resource and not note_resource.get('properties', {}).get('persistent', False):
                        note_resource['properties']['persistent'] = True
                        note_resource['properties']['persisted_at'] = datetime.now().isoformat()
                        note_resource['properties']['persisted_by'] = character_name
                        persisted_count += 1
                        logger.info(f"💾 Auto-persisted Note {note_id} (in Collection {resource_id})")
            if persisted_count > 0:
                logger.info(f"💾 Auto-persisted {persisted_count} Notes in Collection {resource_id}")
        
        # Mark resource as persistent
        resource['properties']['persistent'] = True
        resource['properties']['persisted_at'] = datetime.now().isoformat()
        resource['properties']['persisted_by'] = character_name

        # Persist linked relations whenever an endpoint is persistent.
        # Rule: if either endpoint is persistent, relation should be persistent.
        if resource_id.startswith('Note_') or resource_id.startswith('Collection_'):
            rel_count = 0
            for rid, rdata in self.resource_registry.items():
                if not rid.startswith('Relation_'):
                    continue
                props = rdata.get('properties', {})
                if props.get('source') == resource_id or props.get('target') == resource_id:
                    if not rdata.get('properties', {}).get('persistent', False):
                        rdata['properties']['persistent'] = True
                        rdata['properties']['persisted_at'] = datetime.now().isoformat()
                        rdata['properties']['persisted_by'] = character_name
                        rel_count += 1
            if rel_count:
                logger.info(f"💾 Auto-persisted {rel_count} Relations linked to {resource_id}")
        
        logger.info(f"💾 Marked {resource_id} as persistent by {character_name}")
        
        return True, None
    
    def set_resource_name(self, resource_id: str, name: str) -> Tuple[bool, Optional[str]]:
        """
        Assign or update the stable name of an existing Note or Collection.
        Registers the name for lookup (load by name, _resolve_resource_id).
        
        Args:
            resource_id: Note or Collection ID
            name: Stable name to assign (e.g., "berkeley_weather_report")
            
        Returns:
            Tuple of (success, error_msg)
        """
        if not name or not isinstance(name, str) or not name.strip():
            return False, "Name must be a non-empty string"
        name = name.strip()
        resource = self.resource_registry.get(resource_id)
        if not resource:
            return False, f"Resource {resource_id} not found"
        props = resource.get('properties', {})
        if resource_id.startswith('Note_'):
            old_name = props.get('note_name')
            if old_name and self.named_notes.get(old_name) == resource_id:
                del self.named_notes[old_name]
            # Delete any prior note that held this name
            prior_id = self.named_notes.get(name)
            if prior_id and prior_id != resource_id and prior_id in self.resource_registry:
                logger.info(f"📝 Replacing named Note '{name}': {prior_id} → {resource_id}")
                self.delete_resource(prior_id)
            props['note_name'] = name
            self.named_notes[name] = resource_id
            logger.info(f"📝 Named Note {resource_id} as '{name}'")
        elif resource_id.startswith('Collection_'):
            old_name = props.get('collection_name')
            if old_name and self.named_collections.get(old_name) == resource_id:
                del self.named_collections[old_name]
            props['collection_name'] = name
            self.named_collections[name] = resource_id
            logger.info(f"📚 Named Collection {resource_id} as '{name}'")
        else:
            return False, f"Cannot set name on {resource_id} (only Notes and Collections supported)"
        return True, None
    
    # ==================== Indexing ====================
    
    def index_collection(self, agent_name: str, collection_id: str,
                        index_type: str = 'semantic', fields: Dict = None) -> Tuple[bool, Optional[int], Optional[str]]:
        """
        Index a Collection for semantic search.
        
        Args:
            agent_name: Agent requesting the index
            collection_id: Collection to index
            index_type: 'semantic', 'keyword', or 'hybrid'
            fields: Field specifications for extraction
            
        Returns:
            Tuple of (success, indexed_count, error_msg)
        """
        if fields is None:
            fields = {}
        
        if not collection_id:
            return False, None, "Missing collection_id"
        
        # Fetch Collection
        collection = self.get_resource(collection_id)
        if not collection:
            return False, None, f"Collection {collection_id} not found"
        
        # Check if collection is already indexed
        if collection_id in self.collection_indexes:
            # Check if vector store exists and has content
            if collection_id in self.vector_stores:
                store = self.vector_stores[collection_id]
                existing_chunks = store.index.ntotal if hasattr(store, 'index') else 0
                if existing_chunks > 0:
                    logger.info(f"Collection {collection_id} is already indexed ({existing_chunks} chunks), skipping re-indexing")
                    return True, existing_chunks, None
        
        note_ids = collection.get('properties', {}).get('content', [])
        
        logger.info(f"Index request from {agent_name}: indexing {collection_id} ({len(note_ids)} notes)")
        
        # Create or get store - use Collection ID as store name
        if collection_id not in self.vector_stores:
            self.vector_stores[collection_id] = FAISSStore(dimension=384, logger=logger)
            logger.info(f"Created vector store for {collection_id}")
        
        store = self.vector_stores[collection_id]
        
        # Index each note
        total_chunks = 0
        for note_id in note_ids:
            chunks_indexed = self._index_note_chunks(note_id, collection_id, store, fields, index_type)
            total_chunks += chunks_indexed
        
        # Track that this Collection is indexed
        if collection_id not in self.collection_indexes:
            self.collection_indexes[collection_id] = collection_id
            logger.info(f"Tracking: {collection_id} is indexed for auto-updates")
        
        # Record index info on the Collection
        try:
            props = collection.setdefault('properties', {})
            indexes = props.get('indexes', {})
            indexes[index_type] = collection_id
            props['indexes'] = indexes
            props['indexed_at'] = datetime.now().isoformat()
        except Exception as e:
            logger.warning(f"Failed to record index info on Collection: {e}")
        
        logger.info(f"Indexed {total_chunks} chunks for {collection_id}")
        
        return True, total_chunks, None
    
    def _index_note_chunks(self, note_id: str, collection_id: str, store: FAISSStore,
                          fields: Dict, index_type: str) -> int:
        """
        Helper to chunk and index a single note.
        
        Args:
            note_id: Note to index
            collection_id: Collection ID (for logging)
            store: FAISSStore instance
            fields: Field extraction specs
            index_type: Index type
            
        Returns:
            Number of chunks indexed
        """
        # Fetch Note
        note = self.get_resource(note_id)
        if not note:
            logger.warning(f"Note {note_id} not found, skipping indexing")
            return 0
        
        note_content = note.get('properties', {}).get('content')
        if note_content is None:
            logger.warning(f"Note {note_id} has no content, skipping indexing")
            return 0
        
        # Extract content for embedding
        content_str = self._extract_content_for_embedding(note_content, fields)
        
        # Create chunks with overlap
        chunks = self._create_chunks(content_str, chunk_size=512, overlap=128)
        
        # Index each chunk
        indexed_count = 0
        for i, chunk in enumerate(chunks):
            if index_type in ['semantic', 'hybrid']:
                # Generate embedding for chunk
                embedding = self._generate_embedding(chunk)
                
                # Store chunk with metadata
                metadata = {
                    'timestamp': time.time(),
                    'source_note_id': note_id,
                    'chunk_index': i,
                    'chunk_total': len(chunks),
                    'is_complete_note': len(chunks) == 1
                }
                
                # Store chunk text as document, original content for reference
                store.add(chunk, embedding, metadata=metadata, original_content=note_content)
                indexed_count += 1
        
        return indexed_count
    
    # ==================== Search ====================
    
    def search_collection(self, agent_name: str, collection_id: str, query: str,
                         mode: str = 'semantic', limit: int = 5, threshold: float = 0.0,
                         return_mode: str = 'chunks', max_tokens: Optional[int] = None,
                         token_estimate_chars_per_token: int = 4) -> Tuple[bool, Optional[List], Optional[str]]:
        """
        Search an indexed Collection. Always returns chunks (the actual matched text).
        
        Args:
            agent_name: Agent requesting search
            collection_id: Collection to search
            query: Search query text
            mode: 'semantic', 'keyword', or 'hybrid'
            limit: Max chunks to return (chunk count limit)
            threshold: Minimum similarity threshold
            return_mode: Deprecated, always returns chunks
            max_tokens: Optional maximum token count (if set, accumulates chunks until token budget reached)
            token_estimate_chars_per_token: Characters per token for estimation (default 4)
            
        Returns:
            Tuple of (success, results, error_msg)
            results: List of dicts with 'document' (chunk text), 'score', 'metadata' fields
        """
        logger.info(f"Search request from {agent_name}: '{query}' in {collection_id} (limit={limit}, max_tokens={max_tokens})")
        
        # Check if Collection is indexed
        if collection_id not in self.vector_stores:
            return False, None, f"Collection '{collection_id}' is not indexed. Use 'index' primitive first."
        
        store = self.vector_stores[collection_id]
        
        if mode == 'semantic':
            # Generate query embedding
            query_embedding = self._generate_embedding(query)
            
            # Search for chunks
            # If max_tokens is set, we need more chunks to accumulate tokens, so increase search_limit
            if max_tokens is not None:
                # For token accumulation, get more chunks than limit to have enough to accumulate
                search_limit = max(limit, limit * 2)
            else:
                search_limit = limit
            
            chunk_results = store.search(query_embedding, search_limit, threshold, max_tokens, token_estimate_chars_per_token)
            
            # Always return chunks as-is
            results = chunk_results
            logger.info(f"Search found {len(results)} chunk results")
            
            return True, results, None
        else:
            # Keyword/hybrid not implemented
            return False, None, f"Mode '{mode}' not yet implemented"
    
    # ==================== Embedding Helpers ====================
    
    def _generate_embedding(self, text: str):
        """Generate embedding vector for text"""
        self._init_embedder()
        
        # Convert to string if needed
        if not isinstance(text, str):
            text = str(text)
        
        embedding = self.embedder.encode(text, convert_to_tensor=False, show_progress_bar=False)
        return embedding.tolist()
    
    def _extract_content_for_embedding(self, item: Any, fields: Dict) -> str:
        """
        Extract content from item for embedding based on field specifications.
        
        Args:
            item: The data item (dict, string, or other)
            fields: Dict mapping field names to actions (embed, keyword, store)
            
        Returns:
            String content to embed
        """
        if isinstance(item, str):
            return item
        
        if not isinstance(item, dict):
            return str(item)
        
        # Collect fields marked for embedding
        content_parts = []
        for field_name, action in fields.items():
            if action == 'embed' and field_name in item:
                content_parts.append(str(item[field_name]))
        
        # If no specific fields, use whole item
        if not content_parts:
            return json.dumps(item)
        
        return ' '.join(content_parts)
    
    def _create_chunks(self, text: str, chunk_size: int = 512, overlap: int = 128) -> List[str]:
        """
        Split text into overlapping chunks for embedding.
        Uses paragraph-first strategy: chunks on paragraph boundaries first,
        then segments oversized paragraphs on sentence boundaries.
        
        Args:
            text: String content to chunk
            chunk_size: Target chunk size in characters
            overlap: Overlap between chunks in characters
            
        Returns:
            List of chunk strings
        """
        if len(text) <= chunk_size:
            return [text]
        
        import re
        
        # Stage 1: Split into paragraphs (on double+ newlines)
        paragraphs = []
        for para in re.split(r'\n\n+', text):
            para = para.strip()
            if para:  # Only keep non-empty paragraphs
                paragraphs.append(para)
        
        # If no paragraph breaks found, try escaped '\n\n' strings (for JSON content)
        if not paragraphs:
            escaped_paragraphs = []
            for para in re.split(r'\\n\\n+', text):
                para = para.strip()
                if para:
                    escaped_paragraphs.append(para)
            if len(escaped_paragraphs) > 1:
                paragraphs = escaped_paragraphs
        
        # If still no paragraph breaks found, treat entire text as one paragraph
        if not paragraphs:
            paragraphs = [text.strip()] if text.strip() else []
        
        chunks = []
        
        # Stage 2: Process each paragraph
        for para in paragraphs:
            if len(para) <= chunk_size:
                # Paragraph fits - keep as single chunk
                chunks.append(para)
            else:
                # Paragraph exceeds chunk_size - segment on sentence boundaries
                para_chunks = self._segment_paragraph(para, chunk_size, overlap)
                chunks.extend(para_chunks)
        
        # Stage 3: Apply overlap between consecutive chunks from different paragraphs
        if len(chunks) <= 1:
            return chunks
        
        overlapped_chunks = [chunks[0]]  # First chunk unchanged
        
        for i in range(1, len(chunks)):
            prev_chunk = chunks[i-1]
            curr_chunk = chunks[i]
            
            # Check if chunks are from different paragraphs (heuristic: no overlap in content)
            # Apply overlap: include last 'overlap' chars of previous chunk
            if len(prev_chunk) >= overlap:
                overlap_text = prev_chunk[-overlap:]
                # Only add overlap if it doesn't duplicate content at start of current chunk
                if not curr_chunk.startswith(overlap_text):
                    overlapped_chunks.append(overlap_text + curr_chunk)
                else:
                    overlapped_chunks.append(curr_chunk)
            else:
                overlapped_chunks.append(curr_chunk)
        
        return overlapped_chunks
    
    def _segment_paragraph(self, para: str, chunk_size: int, overlap: int) -> List[str]:
        """
        Segment a single paragraph that exceeds chunk_size on sentence boundaries.
        
        Args:
            para: Paragraph text to segment
            chunk_size: Target chunk size
            overlap: Overlap between chunks
            
        Returns:
            List of chunk strings
        """
        chunks = []
        start = 0
        
        while start < len(para):
            end = start + chunk_size
            
            # Try to find sentence boundary near the end
            if end < len(para):
                search_start = max(start, end - 100)
                sentence_end = max(
                    para.rfind('. ', search_start, end),
                    para.rfind('.\n', search_start, end),
                    para.rfind('! ', search_start, end),
                    para.rfind('!\n', search_start, end),
                    para.rfind('? ', search_start, end),
                    para.rfind('?\n', search_start, end)
                )
                if sentence_end > start:
                    # Include the sentence ending punctuation
                    if para[sentence_end:sentence_end+2] in ['. ', '.\n', '! ', '!\n', '? ', '?\n']:
                        end = sentence_end + 2
                    else:
                        end = sentence_end + 1
                else:
                    # Sentence boundary not found - try literal '\n' string (escaped newline)
                    escaped_newline_end = para.rfind('\\n', search_start, end)
                    if escaped_newline_end > start:
                        end = escaped_newline_end + 2  # Include the '\n' string
                    else:
                        # Try word boundary (space)
                        space_end = para.rfind(' ', search_start, end)
                        if space_end > start:
                            end = space_end + 1  # Include the space
                        # Otherwise fall back to hard character count limit (end already set)
            
            chunks.append(para[start:end])
            
            # Move start forward by chunk_size minus overlap
            start = start + chunk_size - overlap
            
            # Break if we've covered the paragraph
            if end >= len(para):
                break
        
        return chunks
    
    # ==================== Persistence Helpers ====================
    
    def save_resources(self) -> Dict[str, Any]:
        """
        Collect Note, Collection, and Relation instances for persistence.
        
        Returns:
            Dict with note_instances, note_counter, collection_instances, collection_counter
        """
        note_instances = {}
        collection_instances = {}
        relation_instances = {}
        
        for resource_id, resource_data in self.resource_registry.items():
            resource_type = resource_data.get('type')
            type_name = resource_type.name if hasattr(resource_type, 'name') else str(resource_type)
            
            if type_name == 'Note':
                # Save persistent Notes only
                if resource_data.get('properties', {}).get('persistent', False):
                    info_serialized = {
                        'name': resource_data.get('name'),
                        'location': list(resource_data.get('location', (0, 0))),
                        'description': resource_data.get('description'),
                        'properties': resource_data.get('properties', {})
                    }
                    note_instances[resource_id] = info_serialized
                    logger.debug(f"Saving persistent Note {resource_id}")
            elif type_name == 'Collection':
                # Save persistent Collections only
                if resource_data.get('properties', {}).get('persistent', False):
                    info_serialized = {
                        'name': resource_data.get('name'),
                        'location': list(resource_data.get('location', (0, 0))),
                        'description': resource_data.get('description'),
                        'properties': resource_data.get('properties', {})
                    }
                    collection_instances[resource_id] = info_serialized
                    logger.debug(f"Saving persistent Collection {resource_id}")
            elif type_name == 'Relation':
                props = resource_data.get('properties', {})
                source_id = props.get('source')
                target_id = props.get('target')
                source_persistent = bool(self.resource_registry.get(source_id, {}).get('properties', {}).get('persistent', False))
                target_persistent = bool(self.resource_registry.get(target_id, {}).get('properties', {}).get('persistent', False))
                if props.get('persistent', False) or source_persistent or target_persistent:
                    info_serialized = {
                        'name': resource_data.get('name'),
                        'location': list(resource_data.get('location', (0, 0))),
                        'description': resource_data.get('description'),
                        'properties': props
                    }
                    relation_instances[resource_id] = info_serialized
                    logger.debug(f"Saving Relation {resource_id}")
        
        # Scan persistent Collections for non-persistent Notes before save (for logging edge cases)
        # Notes should be auto-persisted when Collection is persisted, but check for Notes added after
        for collection_id, coll_serialized in collection_instances.items():
            coll_props = coll_serialized.get('properties', {})
            if coll_props.get('persistent', False):
                content = coll_props.get('content', [])
                full_coll_data = self.resource_registry.get(collection_id)
                if full_coll_data:
                    for note_id in content:
                        if isinstance(note_id, str) and note_id.startswith('Note_'):
                            note_data = self.resource_registry.get(note_id)
                            if note_data and not note_data.get('properties', {}).get('persistent', False):
                                # Auto-persist Note added after Collection was persisted
                                note_data['properties']['persistent'] = True
                                note_data['properties']['persisted_at'] = datetime.now().isoformat()
                                note_data['properties']['persisted_by'] = 'System'
                                logger.info(f"💾 Auto-persisted Note '{note_id}' added to persistent Collection '{collection_id}' after Collection was persisted")
        
        # Save resource indexes
        self.resource_indexer.save_indexes(self.world_name)
        
        return {
            'note_instances': note_instances,
            'note_counter': self.note_counter,
            'collection_instances': collection_instances,
            'collection_counter': self.collection_counter,
            'relation_instances': relation_instances,
            'relation_counter': self.relation_counter
        }
    
    def load_resources(self, world_data: Dict):
        """
        Restore Note, Collection, and Relation instances from persisted data.
        
        Args:
            world_data: Dict containing note_instances and collection_instances
        """
        # Restore Note instances
        if 'note_instances' in world_data:
            try:
                # Restore counter
                self.note_counter = world_data.get('note_counter', 0)
                
                # Get Note type from resource registry
                note_type = getattr(self.resource_types, 'Note', None)
                
                if note_type:
                    instances = world_data['note_instances']
                    for info_id, info_data in instances.items():
                        resource_data = {
                            'name': info_data['name'],
                            'type': note_type,
                            'location': tuple(info_data['location']),
                            'description': info_data['description'],
                            'remove_on_take': False,
                            'properties': info_data.get('properties', {})
                        }
                        
                        self.resource_registry[info_id] = resource_data
                        
                        note_name = info_data.get('properties', {}).get('note_name')
                        if note_name:
                            self.named_notes[note_name] = info_id
                    
                    logger.info(f"📂 Restored {len(instances)} Note instances, counter at {self.note_counter}")
            except Exception as e:
                logger.error(f"Error restoring Note instances: {e}")
        
        # Restore Collection instances
        if 'collection_instances' in world_data:
            try:
                # Restore counter
                self.collection_counter = world_data.get('collection_counter', 0)
                
                # Get Collection type from resource registry
                collection_type = getattr(self.resource_types, 'Collection', None)
                
                if collection_type:
                    instances = world_data['collection_instances']
                    for info_id, info_data in instances.items():
                        resource_data = {
                            'name': info_data['name'],
                            'type': collection_type,
                            'location': tuple(info_data['location']),
                            'description': info_data['description'],
                            'remove_on_take': False,
                            'properties': info_data.get('properties', {})
                        }
                        
                        self.resource_registry[info_id] = resource_data
                        
                        collection_name = info_data.get('properties', {}).get('collection_name')
                        if collection_name:
                            self.named_collections[collection_name] = info_id
                    
                    logger.info(f"📂 Restored {len(instances)} Collection instances, counter at {self.collection_counter}")
            except Exception as e:
                logger.error(f"Error restoring Collection instances: {e}")
            
            # Cleanup dangling note_ids in restored persistent Collections
            for info_id, info_data in instances.items():
                resource_data = self.resource_registry.get(info_id)
                if resource_data and resource_data.get('properties', {}).get('persistent', False):
                    content = resource_data['properties'].setdefault('content', [])
                    original_len = len(content)
                    content = [nid for nid in content if isinstance(nid, str) and nid.startswith('Note_') and self.resource_registry.get(nid)]
                    resource_data['properties']['content'] = content
                    resource_data['properties']['item_count'] = len(content)
                    removed_count = original_len - len(content)
                    if removed_count > 0:
                        logger.warning(f"Cleaned {removed_count} dangling note_ids from restored persistent Collection '{info_id}'")

        # Restore Relation instances
        if 'relation_instances' in world_data:
            try:
                self.relation_counter = world_data.get('relation_counter', 0)
                relation_type = getattr(self.resource_types, 'Relation', None)

                if relation_type:
                    instances = world_data['relation_instances']
                    for info_id, info_data in instances.items():
                        props = info_data.get('properties', {})
                        source_id = props.get('source')
                        target_id = props.get('target')
                        # Drop dangling persisted relations.
                        if source_id not in self.resource_registry or target_id not in self.resource_registry:
                            logger.warning(f"Skipping dangling Relation '{info_id}' (source={source_id}, target={target_id})")
                            continue

                        resource_data = {
                            'name': info_data['name'],
                            'type': relation_type,
                            'location': tuple(info_data.get('location', (0, 0))),
                            'description': info_data.get('description', ''),
                            'remove_on_take': False,
                            'properties': props
                        }
                        self.resource_registry[info_id] = resource_data

                    logger.info(f"📂 Restored {len(instances)} Relation instances, counter at {self.relation_counter}")
            except Exception as e:
                logger.error(f"Error restoring Relation instances: {e}")
        
        # Load resource indexes and re-index persistent resources
        self.resource_indexer.load_indexes(self.world_name)
        self.resource_indexer.reindex_persistent_resources()
    
    def save_to_file(self, file_path: Optional[Path] = None) -> bool:
        """
        Save resources to JSON file.
        
        Args:
            file_path: Optional path to save file (defaults to self.resources_file)
            
        Returns:
            True if saved successfully
        """
        try:
            target_file = file_path or self.resources_file
            target_file.parent.mkdir(parents=True, exist_ok=True)
            
            resource_data = self.save_resources()
            
            save_data = {
                'world_name': self.world_name,
                'timestamp': datetime.now().isoformat(),
                'note_instances': resource_data['note_instances'],
                'note_counter': resource_data['note_counter'],
                'collection_instances': resource_data['collection_instances'],
                'collection_counter': resource_data['collection_counter'],
                'relation_instances': resource_data.get('relation_instances', {}),
                'relation_counter': resource_data.get('relation_counter', 0)
            }
            
            with open(target_file, 'w') as f:
                json.dump(save_data, f, indent=2)
            
            logger.info(f"💾 Saved resources to {target_file}")
            return True
        except Exception as e:
            logger.error(f"Error saving resources to file: {e}")
            return False
    
    def load_from_file(self, file_path: Optional[Path] = None) -> bool:
        """
        Load resources from JSON file.
        
        Args:
            file_path: Optional path to load file (defaults to self.resources_file)
            
        Returns:
            True if loaded successfully
        """
        try:
            target_file = file_path or self.resources_file
            if not target_file.exists():
                logger.info(f"📂 No existing resources file at {target_file}, starting fresh")
                loaded = False
            else:
                with open(target_file, 'r') as f:
                    save_data = json.load(f)
                self.load_resources(save_data)
                loaded = True
            
            self._sync_counters_from_registry()
            if loaded:
                logger.info(f"📂 Loaded resources from {target_file}")
            return loaded
        except Exception as e:
            logger.error(f"Error loading resources from file: {e}")
            return False
    
    # ==================== Public Query Methods ====================
    
    def get_resource_by_id(self, resource_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a resource by ID (public method for external callers).
        
        Args:
            resource_id: Resource ID (e.g., "Note_42", "Collection_15")
            
        Returns:
            Resource dict or None if not found
        """
        return self.get_resource(resource_id)
    
    def get_resource_content(self, resource_id: str) -> Optional[Dict[str, Any]]:
        """
        Get resource content formatted for viewing (public method for UI).
        
        Args:
            resource_id: Resource ID
            
        Returns:
            Dict with type, content, metadata, or None if not found
        """
        resource = self.get_resource(resource_id)
        if not resource:
            return None
        
        resource_type = resource.get('type')
        type_name = resource_type.name if hasattr(resource_type, 'name') else str(resource_type)
        
        content = resource.get('properties', {}).get('content', '')
        if type_name == 'Collection':
            # For Collections, content is a list of Note IDs
            content = content if isinstance(content, list) else []
        
        # Make properties JSON-serializable
        properties = resource.get('properties', {}).copy()
        serializable_properties = {}
        for key, value in properties.items():
            if isinstance(value, datetime):
                serializable_properties[key] = value.isoformat()
            elif isinstance(value, (str, int, float, bool, type(None))):
                serializable_properties[key] = value
            elif isinstance(value, (dict, list)):
                # Recursively clean nested structures
                serializable_properties[key] = self._make_json_serializable(value)
            else:
                # Convert other types to string
                serializable_properties[key] = str(value)
        
        return {
            'success': True,
            'type': type_name,
            'content': content,
            'metadata': {
                'id': resource_id,
                'name': resource.get('name', ''),
                'description': resource.get('description', ''),
                'properties': serializable_properties
            }
        }
    
    def _make_json_serializable(self, obj):
        """Recursively convert objects to JSON-serializable types."""
        if isinstance(obj, datetime):
            return obj.isoformat()
        elif isinstance(obj, (str, int, float, bool, type(None))):
            return obj
        elif isinstance(obj, dict):
            return {k: self._make_json_serializable(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [self._make_json_serializable(item) for item in obj]
        else:
            return str(obj)
    
    def get_resource_types(self) -> Dict[str, Any]:
        """
        Get all resource types (public method for external callers).
        
        Returns:
            Dict with success flag and list of resource type names
        """
        type_names = []
        for attr_name in dir(self.resource_types):
            if not attr_name.startswith('_'):
                attr_value = getattr(self.resource_types, attr_name)
                if hasattr(attr_value, 'name'):
                    type_names.append(attr_value.name)
        
        return {
            'success': True,
            'resource_types': type_names
        }
    
    def get_resource_rules(self, type_name: str) -> Dict[str, Any]:
        """
        Get resource rules for a specific type (public method for external callers).
        
        Args:
            type_name: Resource type name (e.g., "Note", "Collection", "Skill")
            
        Returns:
            Dict with success flag and resource_rules
        """
        resource_type = getattr(self.resource_types, type_name, None)
        if not resource_type:
            return {
                'success': False,
                'error': f'Resource type {type_name} not found'
            }
        
        # Build rules dict from resource type
        rules = {
            'description': getattr(resource_type, 'description', ''),
            'use': getattr(resource_type, 'use', []),
            'take': getattr(resource_type, 'take', []),
            'create': getattr(resource_type, 'create', [])
        }
        
        # Special handling for Skill type - include skill instances
        if type_name.lower() == 'skill':
            skill_instances = []
            # Look for Skill resources in registry
            for resource_id, resource_data in self.resource_registry.items():
                if resource_data.get('type') == resource_type:
                    skill_instances.append({
                        'skill_name': resource_data.get('name', resource_id),
                        'name': resource_data.get('name', resource_id),
                        'description': resource_data.get('description', '')
                    })
            rules['skill_instances'] = skill_instances
        
        return {
            'success': True,
            'resource_rules': rules
        }
