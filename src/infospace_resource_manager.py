"""
Infospace Resource Manager

Manages Notes, Collections, and indexing infrastructure for the infospace cognitive map.
Extracts resource management logic from map_node to improve modularity and maintainability.
"""

import json
import logging
import hashlib
import time
from datetime import datetime
from typing import Dict, Any, Optional, List, Tuple

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
    
    def search(self, query_embedding, limit=5, threshold=0.0):
        """Search for similar documents"""
        import faiss
        
        # Check if index has documents
        if self.index.ntotal == 0 or len(self.documents) == 0:
            return []
        
        query_array = self.np.array([query_embedding], dtype='float32')
        faiss.normalize_L2(query_array)
        
        # Ensure k > 0 for FAISS
        k = min(limit, len(self.documents))
        if k <= 0:
            return []
        
        # Search
        scores, indices = self.index.search(query_array, k)
        
        # Filter by threshold and format results
        results = []
        for score, idx in zip(scores[0], indices[0]):
            if idx < 0:  # FAISS returns -1 for no match
                continue
            if score < threshold:
                continue
            
            results.append({
                'document': self.documents[idx],
                'score': float(score),
                'metadata': self.metadata[idx],
                'original_content': self.original_contents[idx]
            })
        
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


class InfospaceResourceManager:
    """
    Manages Notes, Collections, and indexing for infospace maps.
    
    Responsibilities:
    - Note and Collection creation
    - Collection mutation (add/remove items)
    - Vector store indexing
    - Semantic search
    - Persistence helpers
    """
    
    def __init__(self, world_map, session, world_name, agent_registry):
        """
        Initialize the resource manager.
        
        Args:
            world_map: WorldMap or InfospaceMap instance
            session: Zenoh session for communication
            world_name: Name of the world (for response topics)
            agent_registry: Dict mapping character_name -> Agent instance
        """
        self.world_map = world_map
        self.session = session
        self.world_name = world_name
        self.agent_registry = agent_registry
        
        # Resource counters
        self.note_counter = 0
        self.collection_counter = 0
        
        # Named collections registry (name -> collection_id)
        self.named_collections = {}
        
        # Indexing infrastructure
        self.collection_indexes = {}  # collection_id -> collection_id (marks indexed)
        self.vector_stores = {}  # collection_id -> FAISSStore
        
        # Embedding model (lazy loaded)
        self.embedder = None
        
        # System collections
        self.system_notes_collection_id = None  # ID of the "Notes" system collection
    
    def _init_embedder(self):
        """Lazy init embedding model"""
        if self.embedder is None:
            from sentence_transformers import SentenceTransformer
            self.embedder = SentenceTransformer('all-MiniLM-L6-v2')
            logger.info("Initialized embedding model: all-MiniLM-L6-v2")
    
    # ==================== System Collections ====================
    
    def initialize_system_collections(self):
        """
        Initialize system collections at startup.
        
        Creates the "Notes" collection that holds all note instances.
        This collection is recreated at startup, so non-persistent notes
        are automatically removed.
        """
        # Create the "Notes" system collection
        success, collection_id, error_msg, location = self.create_collection(
            character_name=None,  # System collection
            content=[],  # Empty at creation
            format_type='list',
            source_skill='system',
            source_value='',
            collection_name='Notes',
            extra_props={'kind': 'system'}
        )
        
        if success:
            self.system_notes_collection_id = collection_id
            
            # Mark as indexed for auto-indexing when notes are added
            self.collection_indexes[collection_id] = collection_id
            
            # Create initial empty vector store so auto-indexing works during Note restoration
            self.vector_stores[collection_id] = FAISSStore(dimension=384)
            
            logger.info(f"📚 System 'Notes' collection created and marked for indexing: {collection_id}")
        else:
            logger.error(f"❌ Failed to create system 'Notes' collection: {error_msg}")
    
    def add_note_to_system_collection(self, note_id: str):
        """
        Add a note to the system "Notes" collection.
        
        Args:
            note_id: ID of the note to add
        """
        if not self.system_notes_collection_id:
            logger.warning("System 'Notes' collection not initialized, skipping add")
            return
        
        # Add to the Notes collection (will auto-index due to collection_indexes)
        success, item_count, error_msg = self.add_to_collection(
            collection_id=self.system_notes_collection_id,
            note_id=note_id,
            agent_name='system',
            operation='add',
            content=None
        )
        
        if not success:
            logger.warning(f"Failed to add {note_id} to Notes collection: {error_msg}")
    
    # ==================== Note Creation ====================
    
    def create_note(self, character_name: str, content: Any, format_type: str,
                   source_skill: str, source_value: str, extra_props: Dict) -> Tuple[bool, Optional[str], Optional[str], Optional[Tuple[int, int]]]:
        """
        Create a Note resource.
        
        Args:
            character_name: Agent creating the note
            content: Note content (any type)
            format_type: Content format ('text' or 'json')
            source_skill: Skill that created this note
            source_value: Input value to the skill
            extra_props: Extra metadata properties
            
        Returns:
            Tuple of (success, note_id, error_msg, location)
        """
        # Validate inputs
        if not character_name:
            return False, None, "Missing character_name", None
        if content is None:
            return False, None, "Missing content", None
        
        # Get agent location
        canonical_character_name = character_name.capitalize()
        if canonical_character_name not in self.agent_registry:
            return False, None, f"Agent for character '{character_name}' not found", None
        
        agent = self.agent_registry[canonical_character_name]
        location = (agent.x, agent.y)
        
        # Generate unique Note ID
        self.note_counter += 1
        note_id = f"Note_{self.note_counter}"
        
        # Get Note type from resource registry
        try:
            note_type = self.world_map.resource_types.Note
        except AttributeError:
            return False, None, "Note resource type not available in this map", None
        
        # Build Note resource data structure
        note_data = {
            'name': note_id,
            'type': note_type,
            'location': location,
            'description': f"Note artifact created by {source_skill}",
            'remove_on_take': False,
            'properties': {
                'content': content,
                'format': format_type,
                'created_by': canonical_character_name,
                'created_at': datetime.now().isoformat(),
                'source_skill': source_skill,
                'source_value': source_value
            }
        }
        
        # Compute metadata
        try:
            if isinstance(content, (dict, list)):
                content_str = json.dumps(content, sort_keys=True)
                note_data['properties']['content_type'] = 'json'
            else:
                content_str = str(content)
                note_data['properties']['content_type'] = 'text'
            note_data['properties']['length'] = len(content_str)
            note_data['properties']['fingerprint'] = hashlib.sha1(content_str.encode('utf-8')).hexdigest()
        except Exception as e:
            logger.warning(f"Failed to compute Note metadata: {e}")
        
        # Merge allowed extra properties
        allowed_fields = {'kind', 'parent_id', 'order', 'span', 'section', 'source', 'entity', 'edge'}
        for key in allowed_fields:
            if key in extra_props:
                note_data['properties'][key] = extra_props[key]
        
        # Register in world map
        self.world_map.resource_registry[note_id] = note_data
        x, y = location
        self.world_map.patches[x][y].resources[note_id] = note_data
        
        logger.info(f"📝 Created Note instance: {note_id} at ({x}, {y}) by {canonical_character_name}")
        
        # Auto-add to system "Notes" collection
        self.add_note_to_system_collection(note_id)
        
        return True, note_id, None, location
    
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
            # System collection: use fixed location and system creator
            canonical_character_name = 'System'
            location = (0, 0)
        else:
            # Regular collection: validate character and get agent location
            if not character_name:
                return False, None, "Missing character_name", None
            
            canonical_character_name = character_name.capitalize()
            if canonical_character_name not in self.agent_registry:
                return False, None, f"Agent for character '{character_name}' not found", None
            
            agent = self.agent_registry[canonical_character_name]
            location = (agent.x, agent.y)
        
        # Generate unique Collection ID
        self.collection_counter += 1
        collection_id = f"Collection_{self.collection_counter}"
        
        # Get Collection type from resource registry
        try:
            collection_type = self.world_map.resource_types.Collection
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
        
        # Register in world map
        self.world_map.resource_registry[collection_id] = collection_data
        x, y = location
        self.world_map.patches[x][y].resources[collection_id] = collection_data
        
        # Register named collection if name provided
        if collection_name:
            self.named_collections[collection_name] = collection_id
            logger.info(f"📚 Created named Collection: '{collection_name}' = {collection_id} at ({x}, {y}) by {canonical_character_name} ({collection_data['properties']['item_count']} items)")
        else:
            logger.info(f"📚 Created Collection instance: {collection_id} at ({x}, {y}) by {canonical_character_name} ({collection_data['properties']['item_count']} items)")
        
        return True, collection_id, None, location
    
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
        collection = self.world_map.resource_registry.get(collection_id)
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
        Mark a Note or Collection as persistent.
        If Collection, also persists all Notes in it that aren't already persistent.
        
        Args:
            resource_id: Resource to mark as persistent
            character_name: Agent performing the operation
            
        Returns:
            Tuple of (success, error_msg)
        """
        # Fetch resource
        resource = self.world_map.resource_registry.get(resource_id)
        if not resource:
            return False, f"Resource {resource_id} not found"
        
        # If Collection, persist all Notes in it first
        if resource_id.startswith('Collection_'):
            content = resource.get('properties', {}).get('content', [])
            persisted_count = 0
            for note_id in content:
                if isinstance(note_id, str) and note_id.startswith('Note_'):
                    note_resource = self.world_map.resource_registry.get(note_id)
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
        
        logger.info(f"💾 Marked {resource_id} as persistent by {character_name}")
        
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
        collection = self.world_map.get_resource_by_name(collection_id)
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
        note = self.world_map.get_resource_by_name(note_id)
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
                         return_mode: str = 'chunks') -> Tuple[bool, Optional[List], Optional[str]]:
        """
        Search an indexed Collection.
        
        Args:
            agent_name: Agent requesting search
            collection_id: Collection to search
            query: Search query text
            mode: 'semantic', 'keyword', or 'hybrid'
            limit: Max results to return
            threshold: Minimum similarity threshold
            return_mode: 'chunks' or 'notes'
            
        Returns:
            Tuple of (success, results, error_msg)
        """
        logger.info(f"Search request from {agent_name}: '{query}' in {collection_id} (return_mode={return_mode})")
        
        # Check if Collection is indexed
        if collection_id not in self.vector_stores:
            return False, None, f"Collection '{collection_id}' is not indexed. Use 'index' primitive first."
        
        store = self.vector_stores[collection_id]
        
        if mode == 'semantic':
            # Generate query embedding
            query_embedding = self._generate_embedding(query)
            
            # Search for chunks (get more than limit for notes mode deduplication)
            search_limit = limit * 3 if return_mode == 'notes' else limit
            chunk_results = store.search(query_embedding, search_limit, threshold)
            
            if return_mode == 'notes':
                # Deduplicate by source_note_id, keeping best score per note
                note_scores = {}
                note_chunks = {}
                for result in chunk_results:
                    source_note_id = result['metadata'].get('source_note_id')
                    if source_note_id:
                        score = result['score']
                        if source_note_id not in note_scores or score > note_scores[source_note_id]:
                            note_scores[source_note_id] = score
                            note_chunks[source_note_id] = result
                
                # Sort by score and limit
                sorted_notes = sorted(note_scores.items(), key=lambda x: x[1], reverse=True)[:limit]
                
                # Return original content for each note
                results = []
                for note_id, score in sorted_notes:
                    result = note_chunks[note_id]
                    results.append({
                        'document': result.get('original_content', result['document']),
                        'score': score,
                        'metadata': {
                            'source_note_id': note_id,
                            'return_mode': 'notes'
                        }
                    })
                
                logger.info(f"Search found {len(chunk_results)} chunks → {len(results)} unique notes")
            else:
                # Return chunks as-is
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
        
        embedding = self.embedder.encode(text, convert_to_tensor=False)
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
        Collect Note and Collection instances for persistence.
        
        Returns:
            Dict with note_instances, note_counter, collection_instances, collection_counter
        """
        note_instances = {}
        collection_instances = {}
        
        for resource_id, resource_data in self.world_map.resource_registry.items():
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
        
        # Scan persistent Collections for non-persistent Notes before save (for logging edge cases)
        # Notes should be auto-persisted when Collection is persisted, but check for Notes added after
        for collection_id, coll_serialized in collection_instances.items():
            coll_props = coll_serialized.get('properties', {})
            if coll_props.get('persistent', False):
                content = coll_props.get('content', [])
                full_coll_data = self.world_map.resource_registry.get(collection_id)
                if full_coll_data:
                    for note_id in content:
                        if isinstance(note_id, str) and note_id.startswith('Note_'):
                            note_data = self.world_map.resource_registry.get(note_id)
                            if note_data and not note_data.get('properties', {}).get('persistent', False):
                                # Auto-persist Note added after Collection was persisted
                                note_data['properties']['persistent'] = True
                                note_data['properties']['persisted_at'] = datetime.now().isoformat()
                                note_data['properties']['persisted_by'] = 'System'
                                logger.info(f"💾 Auto-persisted Note '{note_id}' added to persistent Collection '{collection_id}' after Collection was persisted")
        
        return {
            'note_instances': note_instances,
            'note_counter': self.note_counter,
            'collection_instances': collection_instances,
            'collection_counter': self.collection_counter
        }
    
    def load_resources(self, world_data: Dict):
        """
        Restore Note and Collection instances from persisted data.
        
        Args:
            world_data: Dict containing note_instances and collection_instances
        """
        # Restore Note instances
        if 'note_instances' in world_data:
            try:
                # Restore counter
                self.note_counter = world_data.get('note_counter', 0)
                
                # Get Note type from resource registry
                try:
                    note_type = self.world_map.resource_types.Note
                except AttributeError:
                    logger.warning("Note type not available in this map, skipping Note restoration")
                    note_type = None
                
                if note_type:
                    instances = world_data['note_instances']
                    for info_id, info_data in instances.items():
                        # Reconstruct resource_data structure
                        resource_data = {
                            'name': info_data['name'],
                            'type': note_type,
                            'location': tuple(info_data['location']),
                            'description': info_data['description'],
                            'remove_on_take': False,
                            'properties': info_data.get('properties', {})
                        }
                        
                        # Register in resource_registry
                        self.world_map.resource_registry[info_id] = resource_data
                        
                        # Place in spatial grid
                        x, y = info_data['location']
                        self.world_map.patches[x][y].resources[info_id] = resource_data
                        
                        # Add to system "Notes" collection
                        self.add_note_to_system_collection(info_id)
                    
                    logger.info(f"📂 Restored {len(instances)} Note instances, counter at {self.note_counter}")
            except Exception as e:
                logger.error(f"Error restoring Note instances: {e}")
        
        # Restore Collection instances
        if 'collection_instances' in world_data:
            try:
                # Restore counter
                self.collection_counter = world_data.get('collection_counter', 0)
                
                # Get Collection type from resource registry
                try:
                    collection_type = self.world_map.resource_types.Collection
                except AttributeError:
                    logger.warning("Collection type not available in this map, skipping Collection restoration")
                    collection_type = None
                
                if collection_type:
                    instances = world_data['collection_instances']
                    for info_id, info_data in instances.items():
                        # Reconstruct resource_data structure
                        resource_data = {
                            'name': info_data['name'],
                            'type': collection_type,
                            'location': tuple(info_data['location']),
                            'description': info_data['description'],
                            'remove_on_take': False,
                            'properties': info_data.get('properties', {})
                        }
                        
                        # Register in resource_registry
                        self.world_map.resource_registry[info_id] = resource_data
                        
                        # Place in spatial grid
                        x, y = info_data['location']
                        self.world_map.patches[x][y].resources[info_id] = resource_data
                        
                        # Restore named collection mapping if present
                        collection_name = info_data.get('properties', {}).get('collection_name')
                        if collection_name:
                            self.named_collections[collection_name] = info_id
                    
                    logger.info(f"📂 Restored {len(instances)} Collection instances, counter at {self.collection_counter}")
            except Exception as e:
                logger.error(f"Error restoring Collection instances: {e}")
            
            # Cleanup dangling note_ids in restored persistent Collections
            for info_id, info_data in instances.items():
                resource_data = self.world_map.resource_registry.get(info_id)
                if resource_data and resource_data.get('properties', {}).get('persistent', False):
                    content = resource_data['properties'].setdefault('content', [])
                    original_len = len(content)
                    content = [nid for nid in content if isinstance(nid, str) and nid.startswith('Note_') and self.world_map.resource_registry.get(nid)]
                    resource_data['properties']['content'] = content
                    resource_data['properties']['item_count'] = len(content)
                    removed_count = original_len - len(content)
                    if removed_count > 0:
                        logger.warning(f"Cleaned {removed_count} dangling note_ids from restored persistent Collection '{info_id}'")

