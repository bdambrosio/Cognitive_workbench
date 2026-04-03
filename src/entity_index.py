"""
Entity Index: lightweight NER extraction and entity tracking.

Extracts named entities from text via LLM and maintains an in-memory
index mapping entity names to resource IDs (Notes, conversation turns,
goals) that mention them.  Integrates with the cognitive graph by
emitting persistent entity nodes and 'mentions' edges.
"""

import json
import logging
from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional, Set

logger = logging.getLogger(__name__)

# Lightweight NER prompt — much smaller than the full extract-entities tool.
# Returns only entities (no relationships, dates, key_concepts) to keep
# the call fast and the output compact.
_NER_PROMPT = """\
Extract named entities from the text below. Return JSON with these fields:
- people: list of person/character names
- organizations: list of companies/organizations
- locations: list of places
- topics: list of main topics or subjects (max 5)

Text:
{text}

Return only valid JSON, no explanation.</end>"""


class EntityIndex:
    """In-memory entity index with cognitive graph integration."""

    def __init__(self, cognitive_graph=None):
        # entity_name (lowercase) → set of resource IDs that mention it
        self._index: Dict[str, Set[str]] = defaultdict(set)
        # alias_map: alias (lowercase) → canonical name (lowercase)
        self._aliases: Dict[str, str] = {}
        # cognitive graph entity node IDs: canonical_name → graph node_id
        self._entity_nodes: Dict[str, str] = {}
        self._cognitive_graph = cognitive_graph

    # ── Alias management ─────────────────────────────────────────────

    def add_alias(self, alias: str, canonical: str):
        """Register an alias for an entity (e.g., 'Bruce' → 'user')."""
        self._aliases[alias.lower()] = canonical.lower()

    def add_aliases(self, mapping: Dict[str, str]):
        """Bulk-add aliases: {alias: canonical}."""
        for alias, canonical in mapping.items():
            self._aliases[alias.lower()] = canonical.lower()

    def canonicalize(self, name: str) -> str:
        """Resolve an entity name through the alias map."""
        return self._aliases.get(name.lower(), name.lower())

    # ── Extraction ───────────────────────────────────────────────────

    def extract_entities(self, text: str, llm_generate: Callable) -> Dict[str, List[str]]:
        """Extract entities from text via a lightweight LLM call.

        Returns dict with keys: people, organizations, locations, topics.
        Each value is a list of entity name strings.
        """
        if not text or len(text.strip()) < 20:
            return {"people": [], "organizations": [], "locations": [], "topics": []}

        # Truncate to keep the LLM call fast
        truncated = text[:4000]
        prompt = _NER_PROMPT.format(text=truncated)
        try:
            result = llm_generate(
                messages=[prompt], max_tokens=256, temperature=0.1,
                is_json=True, stops=["</end>"])
            if result.success and result.text:
                parsed = result.text if isinstance(result.text, dict) else json.loads(result.text)
                # Normalize: ensure all expected keys exist and values are lists of strings
                out = {}
                for key in ("people", "organizations", "locations", "topics"):
                    raw = parsed.get(key, [])
                    if isinstance(raw, list):
                        out[key] = [str(e).strip() for e in raw if e and str(e).strip()]
                    else:
                        out[key] = []
                return out
        except (json.JSONDecodeError, TypeError, ValueError) as e:
            logger.debug(f"Entity extraction parse error: {e}")
        except Exception as e:
            logger.warning(f"Entity extraction failed: {e}")

        return {"people": [], "organizations": [], "locations": [], "topics": []}

    # ── Indexing ─────────────────────────────────────────────────────

    def index_entities(self, entities: Dict[str, List[str]], resource_id: str,
                       graph_source_node: str = ""):
        """Add entity→resource_id mappings and emit graph nodes/edges.

        Args:
            entities: Output of extract_entities().
            resource_id: The Note/resource ID that contains these entities.
            graph_source_node: Optional cognitive graph node_id to link via
                              'mentions' edges (e.g., a conversation_turn node).
        """
        all_names = []
        for category in ("people", "organizations", "locations", "topics"):
            all_names.extend(entities.get(category, []))

        for raw_name in all_names:
            canonical = self.canonicalize(raw_name)
            self._index[canonical].add(resource_id)
            # Emit graph entity node (idempotent)
            entity_nid = self._ensure_entity_node(canonical, raw_name)
            # Link content node → entity node
            if graph_source_node and entity_nid and self._cognitive_graph:
                try:
                    self._cognitive_graph.add_edge(graph_source_node, entity_nid, "mentions")
                except (KeyError, Exception) as e:
                    logger.debug(f"Entity graph edge failed: {e}")

    def _ensure_entity_node(self, canonical: str, display_name: str) -> str:
        """Create or return the cognitive graph node for an entity."""
        if canonical in self._entity_nodes:
            return self._entity_nodes[canonical]
        if not self._cognitive_graph:
            return ""
        try:
            nid = self._cognitive_graph.add_node(
                "entity", display_name,
                attrs={"canonical": canonical, "display": display_name})
            self._entity_nodes[canonical] = nid
            return nid
        except Exception as e:
            logger.debug(f"Entity node creation failed: {e}")
            return ""

    # ── Queries ──────────────────────────────────────────────────────

    def lookup(self, entity_name: str) -> Set[str]:
        """Return resource IDs that mention an entity."""
        canonical = self.canonicalize(entity_name)
        return set(self._index.get(canonical, set()))

    def get_all_entities(self) -> Dict[str, Set[str]]:
        """Return the full entity→resource_ids index."""
        return dict(self._index)

    def get_entity_graph_node(self, entity_name: str) -> str:
        """Return cognitive graph node_id for an entity, or ''."""
        canonical = self.canonicalize(entity_name)
        return self._entity_nodes.get(canonical, "")

    # ── Persistence ──────────────────────────────────────────────────

    def to_dict(self) -> Dict[str, Any]:
        """Serialize for persistence."""
        return {
            "index": {k: sorted(v) for k, v in self._index.items()},
            "aliases": dict(self._aliases),
            "entity_nodes": dict(self._entity_nodes),
        }

    def load_dict(self, data: Dict[str, Any]):
        """Restore from serialized data."""
        for k, v in data.get("index", {}).items():
            self._index[k] = set(v)
        self._aliases.update(data.get("aliases", {}))
        self._entity_nodes.update(data.get("entity_nodes", {}))
