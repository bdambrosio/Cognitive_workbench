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

logger = logging.getLogger(__name__)


# ============================================================
# World Model Schema (Concrete, Mutable, Persistent)
# ============================================================

WorldModel = Dict[str, Any]

def empty_world_model() -> WorldModel:
    return {
        "version": "1.0",
        "created_at": datetime.utcnow().isoformat(),
        "updated_at": datetime.utcnow().isoformat(),
        "facts": [
            # Each fact:
            # {
            #   "fact": str,
            #   "confidence": "low|medium|high",
            #   "source": "observation|tool_guarantee|inference",
            #   "stability": "anecdote|regularity|invariant",
            #   "introduced_at": ISO timestamp,
            #   "last_confirmed_at": ISO timestamp,
            # }
        ],
        "tool_contracts": [
            # Each tool contract:
            # {
            #   "tool": str,
            #   "insight": str,
            #   "status": "reliable|unreliable|unknown",
            #   "introduced_at": ISO timestamp,
            #   "last_confirmed_at": ISO timestamp,
            # }
        ],
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
        
        self.world_model = self.load()

    def load(self) -> WorldModel:
        """Load world_model from world_model.json file, or return empty if not found."""
        if not self.world_model_file.exists():
            return empty_world_model()
        
        try:
            with open(self.world_model_file, 'r') as f:
                content = json.load(f)
            
            # Handle both wrapped format (with 'world_model' key) and direct format
            if isinstance(content, dict) and 'world_model' in content:
                world_model = content['world_model']
            else:
                world_model = content
            
            # Extract and store created_at for backup filename
            self.prior_create_date = world_model.get('created_at')
            if not self.prior_create_date:
                # Fallback to updated_at if created_at missing (legacy files)
                self.prior_create_date = world_model.get('updated_at')
            
            # Validate tool_contracts against available_tools
            removed_tools = []
            if self.available_tools is not None:
                available_tool_names = set(self.available_tools.keys())
                tool_contracts = world_model.get('tool_contracts', [])
                original_count = len(tool_contracts)
                
                # Filter out contracts for unavailable tools
                world_model['tool_contracts'] = [
                    contract for contract in tool_contracts
                    if contract.get('tool') in available_tool_names
                ]
                
                # Log warnings for removed contracts and track removed tool names
                removed_count = original_count - len(world_model['tool_contracts'])
                if removed_count > 0:
                    removed_tools = [
                        contract.get('tool') for contract in tool_contracts
                        if contract.get('tool') not in available_tool_names
                    ]
                    for tool_name in removed_tools:
                        logger.warning(f"Removed tool_contract for unavailable tool '{tool_name}' from world_model")
                
                # Remove facts that mention any removed tool names
                if removed_tools:
                    facts = world_model.get('facts', [])
                    original_facts_count = len(facts)
                    
                    world_model['facts'] = [
                        fact for fact in facts
                        if not any(tool_name in fact.get('fact', '') for tool_name in removed_tools)
                    ]
                    
                    # Log warnings for removed facts
                    removed_facts_count = original_facts_count - len(world_model['facts'])
                    if removed_facts_count > 0:
                        for fact in facts:
                            fact_text = fact.get('fact', '')
                            if any(tool_name in fact_text for tool_name in removed_tools):
                                logger.warning(f"Removed fact mentioning unavailable tool(s) from world_model: '{fact_text[:100]}...'")
            
            logger.info(f"📂 Loaded world_model from {self.world_model_file}")
            return world_model
        except (json.JSONDecodeError, IOError) as e:
            logger.warning(f"Failed to load world_model from {self.world_model_file}: {e}")
            return empty_world_model()

    def save(self) -> bool:
        """Save world_model to world_model.json file, backing up existing file first."""
        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            
            # Backup existing file if it exists
            if self.world_model_file.exists():
                # Use updated_at from the file being backed up (not created_at) so each backup has unique name
                backup_date = None
                try:
                    with open(self.world_model_file, 'r') as f:
                        existing_content = json.load(f)
                    if isinstance(existing_content, dict) and 'world_model' in existing_content:
                        existing_content = existing_content['world_model']
                    # Prefer updated_at (when this version was last saved) for unique backup names
                    backup_date = existing_content.get('updated_at') or existing_content.get('created_at')
                except Exception:
                    # Last resort: use file modification time
                    backup_date = datetime.fromtimestamp(self.world_model_file.stat().st_mtime).isoformat()
                
                if backup_date:
                    # Format date for filename: replace colons and dots with hyphens
                    # e.g., "2025-12-31T21-55-15" from "2025-12-31T21:55:15.178017"
                    formatted_date = backup_date.replace(':', '-').replace('.', '-').split('+')[0].split('Z')[0]
                    backup_filename = f"world_model_{formatted_date}.json"
                    backup_path = self.base_dir / backup_filename
                    
                    try:
                        self.world_model_file.rename(backup_path)
                        logger.info(f"📦 Backed up world_model to {backup_filename}")
                    except Exception as e:
                        logger.warning(f"Failed to backup world_model file: {e}")
            
            # Ensure created_at is set (preserve existing or set new)
            save_data = self.world_model.copy()
            if 'created_at' not in save_data:
                save_data['created_at'] = datetime.utcnow().isoformat()
            
            save_data['updated_at'] = datetime.utcnow().isoformat()
            
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
        Deterministically update world_model from a ReflectionFrame.
        ReflectionFrame is treated as a *proposal*, not ground truth.
        """

        wm = deepcopy(self.world_model)
        now = datetime.utcnow().isoformat()
        
        # Preserve created_at if it exists, otherwise set it
        if 'created_at' not in wm:
            wm['created_at'] = self.prior_create_date or now
        # Update prior_create_date if not already set
        if not self.prior_create_date:
            self.prior_create_date = wm.get('created_at')
        
        wm["updated_at"] = now

        # --------------------------------------------------------
        # 1. Promote World Model Facts
        # --------------------------------------------------------

        for update in reflection_frame.get("world_model_updates", []):
            fact_text = update["fact"]
            confidence = update["confidence"]
            source = update["source"]
            stability = update.get("stability", "anecdote")

            # Guardrail 1: reject low-confidence promotions
            if confidence == "low":
                continue

            # Guardrail 2: require generality (LLM-assisted)
            try:
                is_general = self.llm_fact_generalization_check(fact_text)
            except NotImplementedError:
                is_general = True  # fail-open during early development

            if not is_general:
                continue

            # Check for equivalence with existing facts
            eq_index = None
            for i, f in enumerate(wm["facts"]):
                if f["fact"] == fact_text:
                    eq_index = i
                    break

            if eq_index is None:
                # Try semantic equivalence
                try:
                    eq_index = self.llm_fact_equivalence_check(fact_text, wm["facts"])
                except NotImplementedError:
                    eq_index = None

            if eq_index is None:
                # New fact
                wm["facts"].append({
                    "fact": fact_text,
                    "confidence": confidence,
                    "source": source,
                    "stability": stability,
                    "introduced_at": now,
                    "last_confirmed_at": now,
                })
            else:
                # Existing fact: update confidence conservatively
                existing = wm["facts"][eq_index]
                existing["last_confirmed_at"] = now

                # Confidence can only increase, never decrease automatically
                if confidence == "high" and existing["confidence"] != "high":
                    existing["confidence"] = "high"

                # Stability can only strengthen
                stability_rank = {"anecdote": 0, "regularity": 1, "invariant": 2}
                if stability_rank[stability] > stability_rank.get(existing["stability"], 0):
                    existing["stability"] = stability

        # --------------------------------------------------------
        # 2. Update Tool Contracts
        # --------------------------------------------------------

        for ti in reflection_frame.get("tool_insights", []):
            tool = ti["tool"]
            insight = ti["insight"]
            status = ti["status"]

            # Find existing contract
            existing_idx = None
            for i, tc in enumerate(wm["tool_contracts"]):
                if tc["tool"] == tool:
                    existing_idx = i
                    break

            if existing_idx is None:
                wm["tool_contracts"].append({
                    "tool": tool,
                    "insight": insight,
                    "status": status,
                    "introduced_at": now,
                    "last_confirmed_at": now,
                })
            else:
                existing = wm["tool_contracts"][existing_idx]
                existing["last_confirmed_at"] = now

                # Detect conflicts (LLM-assisted)
                try:
                    conflict = self.llm_tool_contract_conflict_check( insight, existing["insight"])
                except NotImplementedError:
                    conflict = False

                if conflict:
                    # Downgrade reliability on conflict
                    existing["status"] = "unreliable"
                else:
                    # Strengthen reliability if consistent
                    if status == "reliable":
                        existing["status"] = "reliable"

        # --------------------------------------------------------
        # 3. Ignore Task-State and Context-Forget
        # --------------------------------------------------------
        # By design:
        # - task_state is ephemeral
        # - context_forget is handled by the planner/runtime
        # - neither affects the world model directly

        self.world_model = wm


    def get(self) -> WorldModel:
        return self.world_model

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