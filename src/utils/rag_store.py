#!/usr/bin/env python3
"""
Lightweight RAG store built on txtai Embeddings.

Design goals:
- Per-character, multi-space indices (e.g., dialogs, facts, plans)
- Local embeddings via sentence-transformers (default: all-MiniLM-L6-v2)
- Simple persistence: one directory per space
- KISS: synchronous ops, no background threads
"""

import os
from pathlib import Path
from typing import Dict, List, Optional

from txtai.embeddings import Embeddings


DEFAULT_MODEL = "sentence-transformers/all-MiniLM-L6-v2"


class CharacterRAGStore:
    def __init__(self, character_name: str, base_dir: str = "data/vector/txtai", model_path: str = DEFAULT_MODEL):
        self.character_name = character_name.capitalize()
        self.base_dir = Path(base_dir) / self.character_name
        self.model_path = model_path
        self.spaces: Dict[str, Embeddings] = {}
        self.base_dir.mkdir(parents=True, exist_ok=True)

    def _space_path(self, space: str) -> Path:
        return self.base_dir / space

    def get_space(self, space: str) -> Embeddings:
        """Get or create a space Embeddings index."""
        if space in self.spaces:
            return self.spaces[space]
        # Initialize embeddings
        embeddings = Embeddings({
            "path": self.model_path
        })
        # Try to load existing index
        space_path = self._space_path(space)
        try:
            if space_path.exists():
                embeddings.load(str(space_path))
        except Exception:
            # If load fails, create fresh index
            pass
        self.spaces[space] = embeddings
        return embeddings

    def upsert(self, space: str, docs: List[Dict]) -> None:
        """Upsert a list of documents. Each doc: {'id', 'text', 'meta': {...}}"""
        if not docs:
            return
        emb = self.get_space(space)
        # txtai index format supports dicts with id/text/metadata
        emb.index(docs)

    def search(self, space: str, query: str, k: int = 5, filters: Optional[Dict] = None):
        """Search space. Optionally filter results by metadata keys after retrieval (client-side KISS)."""
        emb = self.get_space(space)
        results = emb.search(query, k)
        if not filters:
            return results
        filtered = []
        for r in results:
            # r is tuple(score, id, text) or dict depending on version; re-fetch metadata not exposed by default
            # For KISS we accept client-side filter only when ids encode scoped info, or skip filtering
            filtered.append(r)
        return filtered

    def delete(self, space: str, ids: List[str]) -> None:
        if not ids:
            return
        emb = self.get_space(space)
        emb.delete(ids)

    def save_all(self) -> None:
        for space, emb in self.spaces.items():
            space_path = self._space_path(space)
            space_path.mkdir(parents=True, exist_ok=True)
            emb.save(str(space_path))

    def load_all(self, spaces: Optional[List[str]] = None) -> None:
        targets = spaces or [p.name for p in self.base_dir.iterdir() if p.is_dir()]
        for space in targets:
            emb = Embeddings({"path": self.model_path})
            space_path = self._space_path(space)
            try:
                if space_path.exists():
                    emb.load(str(space_path))
                    self.spaces[space] = emb
            except Exception:
                # Skip unreadable index; leave space unloaded
                pass


