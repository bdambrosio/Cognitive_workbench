"""
World Model Schema

Defines the structure for the persistent world model used by the incremental planner.
"""

WORLD_MODEL_SCHEMA = {
  "world_model": {
    "version": "1.0",
    "domain": "minecraft",

    "facts": [
      {
        "id": "wmf_001",
        "fact": "Gravity causes unsupported entities to fall downward",
        "confidence": "high",
        "stability": "invariant",
        "source": "tool_guarantee",
        "first_observed": "timestamp",
        "last_confirmed": "timestamp",
        "support_count": 12,
        "contradiction_count": 0
      }
    ],

    "tool_contracts": [
      {
        "tool": "mc-staircase",
        "properties": [
          {
            "claim": "Does not guarantee vertical ascent",
            "confidence": "high",
            "status": "constrained",
            "evidence_count": 3
          }
        ]
      }
    ]
  }
}

from typing import Dict, List, Any, Optional
from copy import deepcopy
from datetime import datetime
from pathlib import Path
import json
import logging
import hashlib
import random
import re

logger = logging.getLogger(__name__)


# ============================================================
# World Model Schema (Concrete, Mutable, Persistent)
# ============================================================

WorldModel = Dict[str, Any]

RAW_WORLD_MODEL_VERSION = "2.0"
BELIEFS_WORLD_MODEL_VERSION = "1.0"


def _now_iso() -> str:
    return datetime.utcnow().isoformat()


def _norm_text(s: str) -> str:
    """Deterministic, cheap canonicalization for keying evidence."""
    if not isinstance(s, str):
        s = str(s)
    s = s.strip().lower()
    s = re.sub(r"\s+", " ", s)
    return s


def empty_world_model() -> WorldModel:
    """Planner-facing beliefs view (kept backward compatible)."""
    now = _now_iso()
    return {"version": BELIEFS_WORLD_MODEL_VERSION, "created_at": now, "updated_at": now, "facts": [], "tool_contracts": []}


def empty_world_model_raw() -> WorldModel:
    """Persistent raw evidence store (beliefs are derived, not stored)."""
    now = _now_iso()
    return {
        "version": RAW_WORLD_MODEL_VERSION,
        "created_at": now,
        "updated_at": now,
        "raw_data": {"facts": [], "tool_contracts": []},
    }


class WorldModel:
    def __init__(self, world_name: str, agent_name: str, resource_manager, executor=None, available_tools: Dict[str, Dict] = None):
        self.world_name = world_name
        self.agent_name = agent_name
        self.resource_manager = resource_manager
        self.executor = executor
        self.available_tools = available_tools
        # Store prior creation date to avoid re-reading file at save time
        self.prior_create_date = None
        
        # Determine base directory (same logic as InfospaceResourceManager)
        if resource_manager and hasattr(resource_manager, 'base_dir'):
            self.base_dir = resource_manager.base_dir
        else:
            # Fallback: find project root and use scenarios/<world_name>/resources/
            current_dir = Path(__file__).parent
            project_root = current_dir.parent  # src/ -> project root
            self.base_dir = project_root / "scenarios" / world_name / "resources"
        
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.world_model_file = self.base_dir / "world_model.json"
        
        # Persistent state: raw evidence only
        self.world_model = self.load()
        # Derived state: beliefs (planner-facing)
        self._beliefs_cache = self._derive_beliefs(self.world_model)

    def load(self) -> WorldModel:
        """Load raw world_model from world_model.json file, or return empty raw store if not found."""
        if not self.world_model_file.exists():
            return empty_world_model_raw()
        
        try:
            with open(self.world_model_file, 'r') as f:
                content = json.load(f)
            
            # Handle both wrapped format (with 'world_model' key) and direct format
            if isinstance(content, dict) and 'world_model' in content:
                world_model = content['world_model']
            else:
                world_model = content
            
            # Only raw v2.0 is supported. Any other format/version resets to empty raw.
            if not (isinstance(world_model, dict) and world_model.get("version") == RAW_WORLD_MODEL_VERSION and "raw_data" in world_model):
                logger.warning(
                    f"WorldModel: unsupported or legacy format in {self.world_model_file}; resetting to empty raw v{RAW_WORLD_MODEL_VERSION}"
                )
                return empty_world_model_raw()

            # Extract and store created_at for backup filename
            self.prior_create_date = world_model.get('created_at') or world_model.get('updated_at')
            
            # Validate tool_contracts against available_tools (raw store)
            removed_tools = []
            if self.available_tools is not None:
                available_tool_names = set(self.available_tools.keys())
                raw_data = world_model.get("raw_data", {})
                tool_contracts = raw_data.get('tool_contracts', [])
                original_count = len(tool_contracts or [])
                
                # Filter out contracts for unavailable tools
                raw_data['tool_contracts'] = [contract for contract in (tool_contracts or []) if contract.get('tool') in available_tool_names]
                world_model["raw_data"] = raw_data
                
                # Log warnings for removed contracts and track removed tool names
                removed_count = original_count - len(raw_data.get('tool_contracts', []))
                if removed_count > 0:
                    removed_tools = [
                        contract.get('tool') for contract in tool_contracts
                        if contract.get('tool') not in available_tool_names
                    ]
                    for tool_name in removed_tools:
                        logger.warning(f"Removed tool_contract for unavailable tool '{tool_name}' from world_model")
                
                # Remove raw facts that mention any removed tool names (cheap heuristic)
                if removed_tools:
                    facts = world_model.get("raw_data", {}).get('facts', [])
                    original_facts_count = len(facts or [])
                    
                    world_model["raw_data"]["facts"] = [fact for fact in (facts or []) if not any(tool_name in fact.get('fact', '') for tool_name in removed_tools)]
                    
                    # Log warnings for removed facts
                    removed_facts_count = original_facts_count - len(world_model.get("raw_data", {}).get('facts', []))
                    if removed_facts_count > 0:
                        for fact in facts:
                            fact_text = fact.get('fact', '')
                            if any(tool_name in fact_text for tool_name in removed_tools):
                                logger.warning(f"Removed fact mentioning unavailable tool(s) from world_model: '{fact_text[:100]}...'")
            
            logger.info(f"📂 Loaded world_model from {self.world_model_file}")
            return world_model
        except (json.JSONDecodeError, IOError) as e:
            logger.warning(f"Failed to load world_model from {self.world_model_file}: {e}")
            return empty_world_model_raw()

    def save(self) -> bool:
        """Save raw world_model to world_model.json file, backing up existing file first."""
        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            
            # Backup existing file if it exists
            if self.world_model_file.exists():
                backup_path = Path(str(self.world_model_file) + ".bak")
                try:
                    self.world_model_file.rename(backup_path)
                    logger.info(f"📦 Backed up world_model to {backup_path.name}")
                except Exception as e:
                    logger.warning(f"Failed to backup world_model file: {e}")
            
            # Ensure created_at is set (preserve existing or set new)
            save_data = self.world_model.copy()
            if 'created_at' not in save_data:
                save_data['created_at'] = _now_iso()
            
            save_data['updated_at'] = _now_iso()
            
            # Save new file
            with open(self.world_model_file, 'w') as f:
                json.dump(save_data, f, indent=2)
            
            # Update prior_create_date for next save
            self.prior_create_date = save_data['created_at']
            
            logger.info(f"💾 Saved world_model to {self.world_model_file}")
            return True
        except Exception as e:
            logger.error(f"Failed to save world_model to {self.world_model_file}: {e}")
            return False

    def update(self, reflection_frame: Dict[str, Any]):
        """
        Deterministically update raw evidence store from a ReflectionFrame,
        then rebuild beliefs (planner-facing).

        ReflectionFrame is treated as a *proposal*, not ground truth.
        """

        wm = deepcopy(self.world_model)
        now = _now_iso()
        if not isinstance(wm, dict) or wm.get("version") != RAW_WORLD_MODEL_VERSION or "raw_data" not in wm:
            wm = empty_world_model_raw()
        
        # Preserve created_at if it exists, otherwise set it
        if 'created_at' not in wm:
            wm['created_at'] = self.prior_create_date or now
        # Update prior_create_date if not already set
        if not self.prior_create_date:
            self.prior_create_date = wm.get('created_at')
        
        wm["updated_at"] = now

        wm["updated_at"] = now
        raw_data = wm.get("raw_data", {})
        raw_facts = raw_data.get("facts", [])
        raw_tool_contracts = raw_data.get("tool_contracts", [])

        for update in reflection_frame.get("world_model_updates", []):
            fact_text = update.get("fact")
            if not fact_text:
                continue
            polarity = (update.get("polarity") or "support").strip().lower()
            confidence = (update.get("confidence") or "medium").strip().lower()
            source = update.get("source") or "observation"

            # Guardrail: ignore low-confidence proposals (legacy field)
            if confidence == "low":
                continue

            # Guardrail: require generality (LLM-assisted)
            if not self.llm_fact_generalization_check(fact_text):
                continue

            key = _norm_text(fact_text)
            existing = None
            for rf in raw_facts:
                if rf.get("key") == key:
                    existing = rf
                    break
            if existing is None:
                existing = {
                    "key": key,
                    "fact": fact_text,
                    "support_count": 0,
                    "contradiction_count": 0,
                    "source_counts": {},
                    "first_observed_at": now,
                    "last_observed_at": now,
                }
                raw_facts.append(existing)
            if polarity == "contradict":
                existing["contradiction_count"] = int(existing.get("contradiction_count", 0)) + 1
            else:
                existing["support_count"] = int(existing.get("support_count", 0)) + 1
            existing["last_observed_at"] = now
            sc = existing.get("source_counts") or {}
            sc[source] = int(sc.get(source, 0)) + 1
            existing["source_counts"] = sc

        for ti in reflection_frame.get("tool_insights", []):
            tool = ti["tool"]
            insight = ti["insight"]
            status = ti["status"]
            if not tool or not insight:
                continue
            insight_key = _norm_text(insight)
            existing = None
            for rtc in raw_tool_contracts:
                if rtc.get("tool") == tool and rtc.get("insight_key") == insight_key:
                    existing = rtc
                    break
            if existing is None:
                existing = {
                    "tool": tool,
                    "insight_key": insight_key,
                    "insight": insight,
                    "votes": {"reliable": 0, "unreliable": 0, "constrained": 0, "unknown": 0},
                    "first_observed_at": now,
                    "last_observed_at": now,
                }
                raw_tool_contracts.append(existing)
            votes = existing.get("votes") or {"reliable": 0, "unreliable": 0, "constrained": 0, "unknown": 0}
            votes[status] = int(votes.get(status, 0)) + 1
            existing["votes"] = votes
            existing["last_observed_at"] = now

        raw_data["facts"] = raw_facts
        raw_data["tool_contracts"] = raw_tool_contracts
        wm["raw_data"] = raw_data
        self.world_model = wm
        self._beliefs_cache = self._derive_beliefs(self.world_model)


    def get(self) -> WorldModel:
        return self._beliefs_cache

    def get_raw(self) -> WorldModel:
        return self.world_model

    def _rng_for_key(self, key: str) -> random.Random:
        h = hashlib.sha256(key.encode("utf-8")).hexdigest()
        seed = int(h[:16], 16)
        return random.Random(seed)

    def _beta_prob_gt(self, alpha: float, beta: float, threshold: float, key: str, samples: int = 512) -> float:
        rng = self._rng_for_key(key)
        wins = 0
        for _ in range(samples):
            if rng.betavariate(alpha, beta) > threshold:
                wins += 1
        return wins / float(samples)

    def _dirichlet_winner_prob(self, counts: Dict[str, int], key: str, samples: int = 512) -> tuple[str, float]:
        """Dirichlet posterior over status votes; returns (winner_label, P(winner is argmax))."""
        labels = ["reliable", "unreliable", "constrained"]
        rng = self._rng_for_key(key)
        # Symmetric Dirichlet(1,1,1) prior
        alpha = {lab: 1.0 + float(max(0, int(counts.get(lab, 0)))) for lab in labels}
        win_counts = {lab: 0 for lab in labels}
        for _ in range(samples):
            xs = {lab: rng.gammavariate(alpha[lab], 1.0) for lab in labels}
            winner = max(labels, key=lambda lab: xs[lab])
            win_counts[winner] += 1
        winner = max(labels, key=lambda lab: win_counts[lab])
        return winner, win_counts[winner] / float(samples)

    def _derive_beliefs(self, raw_model: WorldModel) -> WorldModel:
        """Build beliefs (planner-facing) from raw evidence using Bayesian thresholds."""
        beliefs = empty_world_model()
        created_at = raw_model.get("created_at") if isinstance(raw_model, dict) else None
        updated_at = raw_model.get("updated_at") if isinstance(raw_model, dict) else None
        if created_at:
            beliefs["created_at"] = created_at
        if updated_at:
            beliefs["updated_at"] = updated_at

        raw_data = raw_model.get("raw_data", {}) if isinstance(raw_model, dict) else {}
        for rf in raw_data.get("facts", []) or []:
            fact_text = rf.get("fact")
            key = rf.get("key") or _norm_text(fact_text or "")
            support = int(rf.get("support_count") or 0)
            contradiction = int(rf.get("contradiction_count") or 0)
            n = support + contradiction
            if not fact_text or n <= 0:
                continue

            # Beta(1,1) prior; evidence: support vs contradiction
            alpha = 1.0 + float(support)
            beta = 1.0 + float(contradiction)
            p_true = self._beta_prob_gt(alpha, beta, 0.5, key=key)

            # Error probability thresholds: 10% / 5% / 2%  => p_true >= 0.90 / 0.95 / 0.98
            if p_true >= 0.98 and support >= 10 and contradiction == 0:
                stability = "invariant"
                confidence = "high"
            elif p_true >= 0.95 and support >= 3:
                stability = "regularity"
                confidence = "medium" if p_true < 0.98 else "high"
            elif p_true >= 0.90 and support >= 1:
                stability = "anecdote"
                confidence = "low" if p_true < 0.95 else "medium"
            else:
                continue

            source_counts = rf.get("source_counts") or {}
            source = "observation"
            if isinstance(source_counts, dict) and source_counts:
                source = max(source_counts.keys(), key=lambda k: int(source_counts.get(k, 0) or 0))

            beliefs["facts"].append({
                "fact": fact_text,
                "confidence": confidence,
                "source": source,
                "stability": stability,
                "introduced_at": rf.get("first_observed_at") or beliefs["created_at"],
                "last_confirmed_at": rf.get("last_observed_at") or beliefs["updated_at"],
                "support_count": support,
                "contradiction_count": contradiction,
            })

        for rtc in raw_data.get("tool_contracts", []) or []:
            tool = rtc.get("tool")
            insight = rtc.get("insight")
            votes = rtc.get("votes") or {}
            if not tool or not insight:
                continue

            total_votes = sum(int(votes.get(k, 0) or 0) for k in ["reliable", "unreliable", "constrained"])
            if total_votes <= 0:
                continue

            key = f"{tool}:{rtc.get('insight_key') or _norm_text(insight)}"
            winner, p_winner = self._dirichlet_winner_prob(votes, key=key)
            status = winner if p_winner >= 0.90 else "unknown"

            # Only include tool_contracts with reliable or constrained status (exclude "unknown" and "unreliable")
            if status in ("unknown", "unreliable"):
                continue

            beliefs["tool_contracts"].append({
                "tool": tool,
                "insight": insight,
                "status": status,
                "introduced_at": rtc.get("first_observed_at") or beliefs["created_at"],
                "last_confirmed_at": rtc.get("last_observed_at") or beliefs["updated_at"],
                "evidence_count": total_votes,
            })

        return beliefs

    def llm_fact_generalization_check(self, fact_text: str) -> bool:
        """
        Return True iff the fact is sufficiently general to belong
        in the persistent world model.

        General facts:
        - Not tied to a specific episode, time, coordinate, or inventory
        - Express a reusable affordance, invariant, or regularity
        - Would plausibly hold in a future episode of the same world

        Non-general facts (must return False):
        - Mentions exact locations, counts, directions, or timestamps
        - Describes a single observation without generalization
        - Refers to "current", "now", "this time", etc.
        """

        prompt = f"""
You are a STRICT epistemic classifier.

Determine whether the following statement should be promoted to a PERSISTENT WORLD MODEL.

A world-model fact must:
- Be reusable across future episodes
- Describe a general affordance, regularity, or invariant
- NOT depend on a specific time, location, count, or episode

If the statement is episodic, contextual, or snapshot-specific, it MUST be rejected.

Respond with EXACTLY one token:
- YES  (general, promotable)
- NO   (episodic or context-specific)

Statement:
    {fact_text}
"""

        # --- LLM CALL ---
        if not self.executor:
            logger.warning("WorldModel: executor not available for generalization check, defaulting to False")
            return False
        
        response = self.executor.llm_generate(prompt, temperature=0, max_tokens=1)
        
        # Extract text from response (handles both LLMResponse object and dict)
        if hasattr(response, 'text'):
            response_text = response.text
        elif isinstance(response, dict):
            response_text = response.get('text', '')
        else:
            response_text = str(response)
        
        answer = response_text.strip().upper()
        if answer not in {"YES", "NO"}:
            # Defensive default: reject on malformed output
            logger.debug(f"WorldModel: malformed generalization response: {response_text!r}")            
            return False

        return answer == "YES"

    def llm_fact_equivalence_check(self, new_fact: str, existing_facts: list[dict],) -> int | None:
        """
        Return the index of an existing fact that is semantically equivalent
        to `new_fact`, or None if no such fact exists.

        Equivalence means:
        - Same underlying claim about the world
        - Differences in wording only
        - NOT merely related, implied, or more general/specific

        This function is conservative: uncertainty => no equivalence.
        """

        if not self.executor:
            logger.warning(
                "WorldModel: executor not available for equivalence check, defaulting to no match"
            )
            return None

        for idx, existing in enumerate(existing_facts):
            existing_fact = existing.get("fact", "")
            if not existing_fact:
                continue

            prompt = f"""
You are a STRICT semantic equivalence checker.

Determine whether the following TWO statements express the SAME WORLD FACT.

They are equivalent ONLY IF:
- They make the same claim about the world
- Any differences are purely wording or phrasing
- Neither is more general or more specific than the other

They are NOT equivalent if:
- One is a special case of the other
- One adds constraints, locations, quantities, or conditions
- They merely concern the same objects but assert different facts

Respond with EXACTLY one token:
- YES  (statements are equivalent)
- NO   (statements are not equivalent)

Statement A:
    {new_fact}

Statement B:
    {existing_fact}
""".strip()

            response = self.executor.llm_generate(prompt,temperature=0,max_tokens=3)

            # Extract text robustly
            if hasattr(response, "text"):
                response_text = response.text
            elif isinstance(response, dict):
                response_text = response.get("text", "")
            else:
                response_text = str(response)

            answer = response_text.strip().upper().split()[0]
            if answer == "YES":
                return idx
            elif answer == "NO":
                continue
            else:
                # Malformed output ⇒ treat as NO
                logger.debug(
                    f"WorldModel: malformed equivalence response: {response_text!r}"
                )
                continue

        return None

    def llm_tool_contract_conflict_check(self, new_insight: str, existing_insight: str) -> bool:
        """
        Return True iff the new insight conflicts with the existing insight.
        """

        prompt = f"""
        You are a STRICT conflict checker.
        """

        return False