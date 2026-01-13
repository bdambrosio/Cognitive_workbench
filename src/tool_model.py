"""
Tool Model Schema

Defines the structure for persistent tool model used by the incremental planner.
"""

from typing import Dict, List, Any, Optional, Tuple
from pathlib import Path
from datetime import datetime
import json
import logging
import re
import hashlib
import math

logger = logging.getLogger(__name__)


def empty_tool_model() -> Dict[str, Any]:
    """Return an empty tool model structure."""
    return {
        "version": "1.0",
        "created_at": datetime.utcnow().isoformat(),
        "updated_at": datetime.utcnow().isoformat(),
        "training_records": [
            # Each record from extract_training_records()
        ],
    }

class ToolModel:
    def __init__(self, world_name: str, agent_name: str, resource_manager, executor=None, available_tools: Dict[str, Dict] = None):
        """
        Initialize ToolModel.
        
        Args:
            world_name: World name (required)
            agent_name: Agent name (required)
            resource_manager: Resource manager instance (required)
            executor: InfospaceExecutor instance (optional)
            available_tools: Dictionary of available tools for filtering (optional)
        """
        self.world_name = world_name
        self.agent_name = agent_name
        self.resource_manager = resource_manager
        self.executor = executor
        self.available_tools = available_tools
        
        # Store prior creation date to avoid re-reading file at save time
        self.prior_create_date = None
        
        # Cache of existing record hashes (reconstructed at load time, not stored)
        self.existing_hashes = set()
        
        # Determine base directory (same logic as WorldModel and InfospaceResourceManager)
        if resource_manager and hasattr(resource_manager, 'base_dir'):
            self.base_dir = resource_manager.base_dir
        else:
            # Fallback: find project root and use scenarios/<world_name>/resources/
            current_dir = Path(__file__).parent
            project_root = current_dir.parent  # src/ -> project root
            self.base_dir = project_root / "scenarios" / world_name / "resources"
        
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.tool_model_file = self.base_dir / "tool_model.json"
        
        self.tool_model = self.load()
        
        logger.info(f'🔧 ToolModel initialized for {self.agent_name} in {self.world_name}')

    def load(self) -> Dict[str, Any]:
        """Load tool_model from tool_model.json file, or return empty if not found."""
        if not self.tool_model_file.exists():
            return empty_tool_model()
        
        try:
            with open(self.tool_model_file, 'r') as f:
                content = json.load(f)
            
            # Handle both wrapped format (with 'tool_model' key) and direct format
            if isinstance(content, dict) and 'tool_model' in content:
                tool_model = content['tool_model']
            else:
                tool_model = content
            
            # Extract and store created_at for backup filename
            self.prior_create_date = tool_model.get('created_at')
            if not self.prior_create_date:
                # Fallback to updated_at if created_at missing (legacy files)
                self.prior_create_date = tool_model.get('updated_at')
            
            # Validate training_records against available_tools
            if self.available_tools is not None:
                available_tool_names = set(self.available_tools.keys())
                training_records = tool_model.get('training_records', [])
                original_count = len(training_records)
                
                # Filter out records for unavailable tools and clean legal_tools lists
                filtered_records = []
                for rec in training_records:
                    chosen_tool = rec.get('chosen_tool')
                    # Remove record if chosen_tool is not available
                    if chosen_tool and chosen_tool not in available_tool_names:
                        logger.warning(f"Removed training_record for unavailable tool '{chosen_tool}' from tool_model")
                        continue
                    
                    # Clean legal_tools list to only include available tools
                    legal_tools = rec.get('legal_tools', [])
                    if legal_tools:
                        cleaned_legal_tools = [tool for tool in legal_tools if tool in available_tool_names]
                        removed_legal = set(legal_tools) - set(cleaned_legal_tools)
                        if removed_legal:
                            logger.warning(f"Removed unavailable tools from legal_tools in training_record: {removed_legal}")
                        rec['legal_tools'] = cleaned_legal_tools
                    
                    filtered_records.append(rec)
                
                tool_model['training_records'] = filtered_records
                
                # Reconstruct existing_hashes from filtered records
                self.existing_hashes = set()
                for rec in filtered_records:
                    h = rec.get("_record_hash")
                    if h:
                        self.existing_hashes.add(h)
            else:
                # Reconstruct existing_hashes from loaded records (not stored, computed at load time)
                self.existing_hashes = set()
                for rec in tool_model.get("training_records", []):
                    h = rec.get("_record_hash")
                    if h:
                        self.existing_hashes.add(h)
            
            logger.info(f"📂 Loaded tool_model from {self.tool_model_file}")
            return tool_model
        except (json.JSONDecodeError, IOError) as e:
            logger.warning(f"Failed to load tool_model from {self.tool_model_file}: {e}")
            return empty_tool_model()

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding vector for text using shared embedder from resource_manager.
        Embeddings are normalized (L2) for efficient cosine similarity computation.
        
        Args:
            text: Text to embed
        
        Returns:
            List of floats representing the normalized embedding vector
        
        Contract:
          - deterministic for same input
          - fixed-length vector (384 dimensions for BGE-Small)
          - L2-normalized (||vector|| = 1.0)
        """
        if self.resource_manager:
            # Use shared embedder from resource_manager
            # _generate_embedding already calls _init_embedder() and returns a list
            embedding = self.resource_manager._generate_embedding(text)
            
            # Normalize L2 for efficient cosine similarity (dot product = cosine similarity)
            norm = math.sqrt(sum(x * x for x in embedding))
            if norm == 0:
                logger.warning(f"Zero-norm embedding for text: {text[:50]}...")
                return embedding  # Return as-is if zero norm
            
            return [x / norm for x in embedding]
        else:
            # Fallback: extremely simple deterministic placeholder (DO NOT USE IN PRODUCTION)
            logger.warning("embed_text called without resource_manager, using hash-based placeholder")
            placeholder = [float((hash(text) >> i) & 0xFF) for i in range(0, 256, 32)]
            # Normalize placeholder too
            norm = math.sqrt(sum(x * x for x in placeholder))
            if norm == 0:
                return placeholder
            return [x / norm for x in placeholder]

    def cosine_similarity(self, a: List[float], b: List[float]) -> float:
        """
        Compute cosine similarity between two vectors.
        Assumes both vectors are already L2-normalized (from embed_text).
        For normalized vectors, dot product = cosine similarity
        Args:
            a: First normalized embedding vector
            b: Second normalized embedding vector
        
        Returns:
            Cosine similarity score (range: -1.0 to 1.0, typically 0.0 to 1.0 for embeddings)
        """
        if not a or not b or len(a) != len(b):
            return 0.0
        
        # Since vectors are normalized, dot product = cosine similarity
        # No need to compute norms or divide
        return sum(x * y for x, y in zip(a, b))

    def save(self) -> bool:
        """Save tool_model to tool_model.json file, backing up existing file first."""
        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            
            # Backup existing file if it exists
            if self.tool_model_file.exists():
                backup_path = Path(str(self.tool_model_file) + ".bak")
                try:
                    self.tool_model_file.rename(backup_path)
                    logger.info(f"📦 Backed up tool_model to {backup_path.name}")
                except Exception as e:
                    logger.warning(f"Failed to backup tool_model file: {e}")
            
            # Ensure created_at is set (preserve existing or set new)
            save_data = self.tool_model.copy()
            if 'created_at' not in save_data:
                save_data['created_at'] = datetime.utcnow().isoformat()
            
            save_data['updated_at'] = datetime.utcnow().isoformat()
            
            # Save new file
            with open(self.tool_model_file, 'w') as f:
                json.dump(save_data, f, indent=2)
            
            # Update prior_create_date for next save
            self.prior_create_date = save_data['created_at']
            
            logger.info(f"💾 Saved tool_model to {self.tool_model_file}")
            return True
        except Exception as e:
            logger.error(f"Failed to save tool_model to {self.tool_model_file}: {e}")
            return False

    def update_from_trace(self, trace_file_path: Optional[Path] = None, trace_str: Optional[str] = None) -> int:
        """
        Update the tool model by extracting training records from a planner trace
        and appending any new records that are not already present.

        This method is intentionally non-aggregative:
        - It does NOT compute statistics or reliability yet.
        - It only persists normalized step-level evidence for later analysis.

        Args:
            trace_file_path: Path to trace file (optional, if trace_str not provided)
            trace_str: Trace content as string (optional, if trace_file_path not provided)
            
        Returns:
            int: number of new training records added
        """
        # 1. Extract records from trace
        new_records = self.extract_training_records(trace_file_path=trace_file_path, trace_str=trace_str)
        if not new_records:
            logger.info("🟡 No training records extracted from trace.")
            return 0

        # 2. Use cached existing_hashes (reconstructed at load time, not stored)
        added = 0

        # 3. Normalize + hash each new record
        for rec in new_records:
            # Normalize record for hashing (drop volatile fields)
            hash_basis = {
                "goal_id": rec.get("goal_id"),
                "step_index": rec.get("step_index"),
                "world": rec.get("world"),
                "current_task": rec.get("current_task"),
                "agent_state_hypotheses": rec.get("agent_state_hypotheses"),
                "chosen_tool": rec.get("chosen_tool"),
                "tool_args": rec.get("tool_args"),
                "tool_status": rec.get("tool_status"),
                "audit_verdicts": rec.get("audit_verdicts"),
                "done_flag": rec.get("done_flag"),
            }

            hash_text = json.dumps(hash_basis, sort_keys=True, ensure_ascii=True)
            record_hash = hashlib.sha256(hash_text.encode("utf-8")).hexdigest()

            if record_hash in self.existing_hashes:
                continue  # already ingested

            # Attach bookkeeping metadata
            rec["_record_hash"] = record_hash
            rec["_ingested_at"] = datetime.utcnow().isoformat()
            rec["_source"] = "planner_trace"

            self.tool_model.setdefault("training_records", []).append(rec)
            self.existing_hashes.add(record_hash)  # Update cache
            added += 1

        if added > 0:
            self.tool_model["updated_at"] = datetime.utcnow().isoformat()
            self.save()
            logger.info(f"✅ ToolModel updated: {added} new training records added.")
        else:
            logger.info("🟢 ToolModel already up to date; no new records added.")

        return added

    def extract_training_records(self, trace_file_path: Optional[Path] = None, trace_str: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Process a trace file or trace string and extract training records for each planner step.
        
        Each record contains:
        1. Task Context (goal_id, current_task, step_index, world, legal_tools, forbidden_tools)
        2. Agent-State Hypotheses (from STAGE 2-PRE)
        3. Action Selection (chosen_tool, tool_args)
        4. Outcome Signal (tool_status, result_summary, new_bindings)
        5. Local Evaluation (audit_verdicts, done_flag, next_task)
        6. Counterfactual Signal (optional, placeholder for offline computation)
        
        Args:
            trace_file_path: Path to trace file (optional, if trace_str not provided)
            trace_str: Trace content as string (optional, if trace_file_path not provided)
            
        Returns:
            List of training record dictionaries, one per planner step.
        """
        # Get trace content from string or file
        if trace_str is not None:
            trace_content = trace_str
        elif trace_file_path is not None:
            if not trace_file_path.exists():
                logger.warning(f"Trace file not found: {trace_file_path}")
                return []
            with open(trace_file_path, 'r', encoding='utf-8') as f:
                trace_content = f.read()
        else:
            # Use default trace file path (same pattern as IncrementalPlanner)
            trace_dir = Path(__file__).parent / "logs"
            trace_file_path = trace_dir / f"planner_trace_{self.agent_name}.txt"
            if not trace_file_path.exists():
                logger.warning(f"Trace file not found: {trace_file_path}")
                return []
            with open(trace_file_path, 'r', encoding='utf-8') as f:
                trace_content = f.read()
        
        # Extract goal from trace header
        goal_match = re.search(r'Goal:\s*(.+?)(?:\n|$)', trace_content)
        goal_text = goal_match.group(1).strip() if goal_match else "unknown"
        goal_id = hashlib.sha256(goal_text.encode()).hexdigest()[:16]
        
        # Determine world from executor or world_name
        world = self.world_name.lower() if self.world_name else "unknown"
        
        # Get legal tools from executor
        legal_tools = list(self.executor.available_tools.keys()) if hasattr(self.executor, 'available_tools') else []
        
        # Extract forbidden_tools from goal constraints (placeholder - would need goal parsing)
        forbidden_tools = []  # TODO: Extract from goal constraints if available
        
        # Parse trace sections from conversation format
        records = []
        
        # Find all step indices from STAGE markers
        step_indices = set()
        stage2_pre_pattern = r'STAGE 2-PRE \(step (\d+)/(\d+)\):'
        stage2_pattern = r'STAGE 2 \(step (\d+)/(\d+)\):'
        stage3_pattern = r'STAGE 3 - TOOL EXECUTION COMPLETE \(step (\d+)/(\d+)\)'
        
        for match in re.finditer(stage2_pre_pattern, trace_content):
            step_indices.add(int(match.group(1)) - 1)  # Convert to 0-indexed
        
        for match in re.finditer(stage2_pattern, trace_content):
            step_indices.add(int(match.group(1)) - 1)
        
        for match in re.finditer(stage3_pattern, trace_content):
            step_indices.add(int(match.group(1)) - 1)
        
        # Process each step
        steps = sorted(step_indices)
        
        for step_index in steps:
            step_num = step_index + 1  # 1-indexed for display
            
            # Extract STAGE 2-PRE: Agent-State Hypotheses
            agent_state_hypotheses = []
            agent_state_support = {}
            
            # Find STAGE 2-PRE section
            stage2_pre_match = re.search(
                rf'STAGE 2-PRE \(step {step_num}/\d+\):(.*?)(?=STAGE 2-PRE|STAGE 2|STAGE 3|$)',
                trace_content,
                re.DOTALL
            )
            if stage2_pre_match:
                stage2_pre_section = stage2_pre_match.group(1)
                
                # Extract AGENT_STATE_HYPOTHESES (bullet points or list format)
                hyp_section_match = re.search(
                    r'AGENT_STATE_HYPOTHESES:\s*\n(.*?)(?:\nAGENT_STATE_SUPPORT:|$)',
                    stage2_pre_section,
                    re.DOTALL
                )
                if hyp_section_match:
                    hyp_text = hyp_section_match.group(1).strip()
                    # Parse bullet points: "- hypothesis text"
                    for line in hyp_text.split('\n'):
                        line = line.strip()
                        if line.startswith('-'):
                            # Remove leading "- " and clean up
                            hyp = line[1:].strip()
                            if hyp:
                                agent_state_hypotheses.append(hyp)
                    # If no bullet points found, try JSON list format
                    if not agent_state_hypotheses:
                        list_match = re.search(r'\[(.*?)\]', hyp_text, re.DOTALL)
                        if list_match:
                            list_content = list_match.group(1)
                            agent_state_hypotheses = [h.strip().strip('"\'') for h in list_content.split(',') if h.strip()]
                
                # Extract AGENT_STATE_SUPPORT
                support_section_match = re.search(
                    r'AGENT_STATE_SUPPORT:\s*\n(.*?)(?:\n<\|im_end\>|$)',
                    stage2_pre_section,
                    re.DOTALL
                )
                if support_section_match:
                    support_text = support_section_match.group(1).strip()
                    # Parse support lines: "- <h1>: <evidence>" or "- none"
                    for line in support_text.split('\n'):
                        line = line.strip()
                        if line.startswith('-'):
                            parts = line[1:].split(':', 1)
                            if len(parts) == 2:
                                hyp_key = parts[0].strip()
                                evidence = parts[1].strip()
                                agent_state_support[hyp_key] = evidence
                            elif len(parts) == 1:
                                # Handle "- none" case
                                value = parts[0].strip()
                                if value.lower() == 'none':
                                    # Store as empty or special marker
                                    agent_state_support['_none'] = 'none'
            
            # Extract STAGE 2: Tool selection
            current_task = None
            chosen_tool = None
            tool_args = None
            tool_args_raw = None
            
            # Find STAGE 2 section
            stage2_match = re.search(
                rf'STAGE 2 \(step {step_num}/\d+\):(.*?)(?=STAGE 2-PRE|STAGE 2|STAGE 3|$)',
                trace_content,
                re.DOTALL
            )
            if stage2_match:
                stage2_section = stage2_match.group(1)
                
                # Extract CURRENT_TASK
                task_match = re.search(r'CURRENT_TASK:\s*(.+?)(?:\n|$)', stage2_section, re.MULTILINE)
                if task_match:
                    current_task = task_match.group(1).strip()
                
                # Extract TOOL_NAME
                tool_name_match = re.search(r'TOOL_NAME:\s*([\w-]+)', stage2_section)
                if tool_name_match:
                    chosen_tool = tool_name_match.group(1).strip()
                
                # Extract TOOL_ARGS_JSON
                tool_args_match = re.search(r'TOOL_ARGS_JSON:\s*(\{.*?\})', stage2_section, re.DOTALL)
                if tool_args_match:
                    tool_args_raw = tool_args_match.group(1).strip()
                    try:
                        tool_args = json.loads(tool_args_raw)
                    except:
                        tool_args = {}
            
            # Extract STAGE 3: Outcome and evaluation
            tool_status = None
            result_summary = None
            new_bindings = []
            audit_verdicts = {}
            done_flag = None
            next_task = None
            
            # Find STAGE 3 section
            stage3_match = re.search(
                rf'STAGE 3 - TOOL EXECUTION COMPLETE \(step {step_num}/\d+\)(.*?)(?=STAGE 3|STAGE 2|$)',
                trace_content,
                re.DOTALL
            )
            if stage3_match:
                stage3_section = stage3_match.group(1)
                
                # Extract tool status from result
                status_match = re.search(r'(SUCCESS|FAILED|OK)', stage3_section)
                if status_match:
                    tool_status = 'success' if status_match.group(1) in ['SUCCESS', 'OK'] else 'failure'
                
                # Extract ACTUAL RESULT
                result_match = re.search(r'>> ACTUAL RESULT.*?<<\n(.*?)\n>> END RESULT', stage3_section, re.DOTALL)
                if result_match:
                    result_text = result_match.group(1).strip()
                    result_summary = self._summarize_result(result_text)
                
                # Extract DONE flag
                done_match = re.search(r'DONE:\s*(.+?)(?:\n|$)', stage3_section, re.MULTILINE)
                if done_match:
                    done_text = done_match.group(1).strip().upper()
                    done_flag = done_text.startswith('YES')
                
                # Extract NEXT_TASK
                next_match = re.search(r'NEXT_TASK:\s*(.+?)(?:\n|$)', stage3_section, re.MULTILINE)
                if next_match:
                    next_task = next_match.group(1).strip()
                
                # Extract AUDIT verdicts
                audit_match = re.search(r'AUDIT:\s*(.*?)(?:\nDONE|$)', stage3_section, re.DOTALL)
                if audit_match:
                    audit_text = audit_match.group(1).strip()
                    if audit_text and audit_text != '[]':
                        audit_verdicts = self._parse_audit_verdicts(audit_text)
            
            # Build training record
            record = {
                # 1. Task Context
                'goal_id': goal_id,
                'current_task': current_task,
                'step_index': step_index,
                'world': world,
                'legal_tools': legal_tools,
                'forbidden_tools': forbidden_tools,
                
                # 2. Agent-State Hypotheses
                'agent_state_hypotheses': agent_state_hypotheses,
                'agent_state_support': agent_state_support,
                
                # 3. Action Selection
                'chosen_tool': chosen_tool,
                'tool_args': tool_args,
                'tool_args_raw': tool_args_raw,
                
                # 4. Outcome Signal
                'tool_status': tool_status,
                'result_summary': result_summary,
                'new_bindings': new_bindings,
                
                # 5. Local Evaluation
                'audit_verdicts': audit_verdicts,
                'done_flag': done_flag,
                'next_task': next_task,
                
                # 6. Counterfactual Signal (placeholder)
                'tool_was_necessary': None,
                'tool_was_sufficient': None,
                'redundant_step': None,
            }
            
            records.append(record)
        
        logger.info(f"Extracted {len(records)} training records from {trace_file_path}")
        return records
    
    def _summarize_result(self, result_text: str) -> Dict[str, Any]:
        """Extract structural summary from result text (keys, counts, booleans)."""
        summary = {
            'text_length': len(result_text),
            'has_json': False,
            'keys': [],
            'counts': {},
            'booleans': {},
        }
        
        # Try to parse as JSON
        try:
            result_json = json.loads(result_text)
            summary['has_json'] = True
            if isinstance(result_json, dict):
                summary['keys'] = list(result_json.keys())
                # Count types
                for k, v in result_json.items():
                    if isinstance(v, bool):
                        summary['booleans'][k] = v
                    elif isinstance(v, (int, float)):
                        summary['counts'][k] = v
                    elif isinstance(v, (list, dict)):
                        summary['counts'][k] = len(v) if v else 0
        except:
            # Not JSON, look for patterns
            # Count lines, look for key:value patterns
            lines = result_text.split('\n')
            summary['line_count'] = len(lines)
            # Look for boolean-like patterns
            if 'true' in result_text.lower() or 'false' in result_text.lower():
                summary['has_booleans'] = True
        
        return summary
    
    def _parse_audit_verdicts(self, audit_text: str) -> Dict[str, Dict[str, str]]:
        """Parse audit verdicts from audit text."""
        verdicts = {}
        
        # Split by hypothesis blocks (each hypothesis followed by scope and verdict)
        # Pattern: hypothesis text\nscope: <scope>\nverdict: <verdict>
        parts = re.split(r'\n(?:scope|verdict):', audit_text)
        
        current_hyp = None
        for i, part in enumerate(parts):
            part = part.strip()
            if not part:
                continue
            
            if i == 0 or not part.startswith(('observational', 'observed-set', 'global', 'SUPPORTED', 'UNSUPPORTED', 'CONTRADICTED')):
                # This is a hypothesis
                current_hyp = part
                if current_hyp not in verdicts:
                    verdicts[current_hyp] = {}
            else:
                # This is scope or verdict
                if part.startswith(('observational', 'observed-set', 'global')):
                    if current_hyp:
                        verdicts[current_hyp]['scope'] = part.split()[0] if part.split() else part
                elif part.startswith(('SUPPORTED', 'UNSUPPORTED', 'CONTRADICTED')):
                    if current_hyp:
                        verdicts[current_hyp]['verdict'] = part.split()[0] if part.split() else part
        
        return verdicts


    def build_task_tool_index(self) -> Dict[str, Any]:
        """
        Compile training_records into a task-embedding → tool-success index.

        Aggregates records by (task_text, tool) pairs and computes success rates.

        Returns an in-memory structure:
        {
            "entries": [
                {
                    "embedding": [...],
                    "task_text": "...",
                    "tool_stats": {
                        "tool_name": {
                            "success": int,
                            "failure": int,
                            "total": int,
                            "rate": float  # success_rate = success / total
                        },
                        ...
                    }
                },
                ...
            ]
        }
        """
        records = self.tool_model.get("training_records", [])
        
        # First pass: aggregate by (task_text, tool) pairs
        # Key: (task_text, tool) -> {"success": count, "failure": count}
        task_tool_stats: Dict[Tuple[str, str], Dict[str, int]] = {}
        
        for rec in records:
            task = rec.get("current_task")
            tool = rec.get("chosen_tool")
            tool_status = rec.get("tool_status")
            
            if not task or not tool or not tool_status:
                continue
            
            key = (task, tool)
            if key not in task_tool_stats:
                task_tool_stats[key] = {"success": 0, "failure": 0}
            
            if tool_status == "success":
                task_tool_stats[key]["success"] += 1
            elif tool_status == "failure":
                task_tool_stats[key]["failure"] += 1
        
        # Second pass: group by task_text and build entries
        # Key: task_text -> {"embedding": [...], "tool_stats": {...}}
        task_index: Dict[str, Dict[str, Any]] = {}
        
        for (task_text, tool), counts in task_tool_stats.items():
            success_count = counts["success"]
            failure_count = counts["failure"]
            total_count = success_count + failure_count
            success_rate = success_count / total_count if total_count > 0 else 0.0
            
            if task_text not in task_index:
                # Embed task text (only once per unique task)
                try:
                    emb = self.embed_text(task_text)
                except Exception as e:
                    logger.warning(f"Embedding failed for task '{task_text}': {e}")
                    continue
                
                task_index[task_text] = {
                    "embedding": emb,
                    "task_text": task_text,
                    "tool_stats": {}
                }
            
            # Add tool statistics
            task_index[task_text]["tool_stats"][tool] = {
                "success": success_count,
                "failure": failure_count,
                "total": total_count,
                "rate": success_rate
            }
        
        entries = list(task_index.values())
        
        # Log summary statistics
        total_tool_uses = sum(
            sum(stats["total"] for stats in entry["tool_stats"].values())
            for entry in entries
        )
        logger.info(
            f"🔧 Compiled task-tool index: {len(entries)} unique tasks, "
            f"{total_tool_uses} total tool uses aggregated"
        )
        
        return {
            "entries": entries,
        }

    def suggest_tools_for_task(
        self,
        task_text: str,
        compiled_index: Dict[str, Any],
        top_k: int = 5,
        min_similarity: float = 0.2,
        min_success_rate: float = 0.0,
    ) -> List[Tuple[str, float]]:
        """
        Query the compiled index with a task description and return
        ranked tool suggestions.

        Scoring: similarity × success_rate × total_count
        - Higher similarity = more relevant task match
        - Higher success_rate = more reliable tool choice
        - Higher total_count = more evidence (confidence)

        Args:
            task_text: Task description to find tools for
            compiled_index: Index from build_task_tool_index()
            top_k: Maximum number of suggestions to return
            min_similarity: Minimum cosine similarity threshold
            min_success_rate: Minimum success rate threshold (0.0-1.0)

        Returns:
          List of (tool_name, score), sorted descending.
        """
        if not task_text or not compiled_index:
            return []

        try:
            query_emb = self.embed_text(task_text)
        except Exception as e:
            logger.warning(f"Embedding failed for query task '{task_text}': {e}")
            return []

        # Aggregate scores per tool across all matching entries
        # Key: tool_name -> {"weighted_score": float, "max_similarity": float, "best_rate": float}
        tool_aggregates: Dict[str, Dict[str, float]] = {}

        for entry in compiled_index.get("entries", []):
            sim = self.cosine_similarity(query_emb, entry["embedding"])
            if sim < min_similarity:
                continue

            for tool_name, stats in entry["tool_stats"].items():
                success_rate = stats["rate"]
                total_count = stats["total"]
                
                # Filter by minimum success rate
                if success_rate < min_success_rate:
                    continue
                
                # Score: similarity × success_rate × log(total_count + 1)
                # Using log to prevent high-count entries from dominating
                # +1 to avoid log(0)
                score = sim * success_rate * math.log(total_count + 1)
                
                if tool_name not in tool_aggregates:
                    tool_aggregates[tool_name] = {
                        "weighted_score": 0.0,
                        "max_similarity": 0.0,
                        "best_rate": 0.0,
                    }
                
                # Accumulate weighted score
                tool_aggregates[tool_name]["weighted_score"] += score
                # Track best similarity and rate for this tool
                tool_aggregates[tool_name]["max_similarity"] = max(
                    tool_aggregates[tool_name]["max_similarity"], sim
                )
                tool_aggregates[tool_name]["best_rate"] = max(
                    tool_aggregates[tool_name]["best_rate"], success_rate
                )

        # Rank tools by weighted score
        ranked = sorted(
            tool_aggregates.items(),
            key=lambda x: x[1]["weighted_score"],
            reverse=True
        )
        
        # Return (tool_name, score) pairs
        return [(tool, stats["weighted_score"]) for tool, stats in ranked[:top_k]]

        """
FailurePatternStore

Level-1 scaffolding for persistent storage of failure / success patterns
derived from reflection output.

This module is intentionally semantic-light.
It provides CRUD + lifecycle only.
"""

from typing import Dict, Any, List, Optional
from pathlib import Path
from datetime import datetime
import json
import uuid
import logging

logger = logging.getLogger(__name__)


def empty_pattern_store() -> Dict[str, Any]:
    return {
        "version": "1.0",
        "created_at": datetime.utcnow().isoformat(),
        "updated_at": datetime.utcnow().isoformat(),
        "patterns": []  # list of pattern records
    }


class FailurePatternStore:
    """
    World-scoped persistent store for mechanism / failure patterns.
    Semantics are opaque payloads supplied by reflection code.
    """

    def __init__(self, world_name: str, base_dir: Path):
        self.world_name = world_name
        self.base_dir = base_dir
        self.base_dir.mkdir(parents=True, exist_ok=True)

        self.store_path = self.base_dir / "failure_patterns.json"
        self.store = self.load()
        self._existing_hashes = set()

        logger.info(f"🧠 FailurePatternStore initialized for world='{world_name}'")

    # ---------- Persistence ----------

    def load(self) -> Dict[str, Any]:
        if not self.store_path.exists():
            return empty_pattern_store()

        try:
            with open(self.store_path, "r") as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"Failed to load failure pattern store: {e}")
            return empty_pattern_store()

    def save(self) -> None:
        self.store["updated_at"] = datetime.utcnow().isoformat()
        with open(self.store_path, "w") as f:
            json.dump(self.store, f, indent=2)

    # ---------- CRUD ----------

    def add_pattern(
        self,
        *,
        source_reflection_id: str,
        pattern_type: str,
        semantics: Dict[str, Any],
        status: str = "tentative",
        confidence: Optional[float] = None,
    ) -> str:
        """
        Add a new failure (or success) pattern.

        semantics: opaque dict produced by reflection code
        """

        pattern_id = str(uuid.uuid4())

        record = {
            "pattern_id": pattern_id,
            "world": self.world_name,
            "pattern_type": pattern_type,  # e.g. mechanism_constraint
            "status": status,               # tentative | active | deprecated
            "confidence": confidence,       # optional scalar
            "source_reflection_id": source_reflection_id,
            "created_at": datetime.utcnow().isoformat(),
            "updated_at": datetime.utcnow().isoformat(),
            "semantics": semantics,          # 🔒 opaque payload
        }

        self.store["patterns"].append(record)
        self.save()

        logger.info(f"➕ Added failure pattern {pattern_id}")
        return pattern_id

    def get_pattern(self, pattern_id: str) -> Optional[Dict[str, Any]]:
        for p in self.store.get("patterns", []):
            if p["pattern_id"] == pattern_id:
                return p
        return None

    def list_patterns(
        self,
        *,
        pattern_type: Optional[str] = None,
        status: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        results = self.store.get("patterns", [])

        if pattern_type:
            results = [p for p in results if p["pattern_type"] == pattern_type]
        if status:
            results = [p for p in results if p["status"] == status]

        return results

    def update_pattern(
        self,
        pattern_id: str,
        *,
        status: Optional[str] = None,
        confidence: Optional[float] = None,
        semantics: Optional[Dict[str, Any]] = None,
    ) -> bool:
        pattern = self.get_pattern(pattern_id)
        if not pattern:
            return False

        if status is not None:
            pattern["status"] = status
        if confidence is not None:
            pattern["confidence"] = confidence
        if semantics is not None:
            pattern["semantics"] = semantics

        pattern["updated_at"] = datetime.utcnow().isoformat()
        self.save()
        return True

    def retire_pattern(self, pattern_id: str) -> bool:
        return self.update_pattern(pattern_id, status="deprecated")

    # ---------- Query (minimal, non-semantic) ----------

    def find_by_tool(self, tool_name: str) -> List[Dict[str, Any]]:
        """
        Shallow helper: returns patterns whose semantics mention a tool name.
        No interpretation beyond substring match.
        """
        matches = []
        for p in self.store.get("patterns", []):
            sem = p.get("semantics", {})
            if tool_name in json.dumps(sem):
                matches.append(p)
        return matches

    def ingest_reflection(self, reflection: Dict[str, Any]) -> int:
        """
        Extract and store failure patterns from a ReflectionFrame.

        Returns number of new patterns added.
        """
        if "ReflectionFrame" not in reflection:
            logger.warning("No ReflectionFrame found; skipping ingestion.")
            return 0

        frame = reflection["ReflectionFrame"]
        added = 0

        # 1. Primary extraction: tool_insights
        for insight in frame.get("tool_insights", []):
            pattern = self._pattern_from_tool_insight(
                insight=insight,
                failure_mode=frame.get("failure_mode"),
            )
            if self._add_pattern(pattern):
                added += 1

        # 2. Secondary reinforcement: immediate_blockers
        for blocker in frame.get("task_state", {}).get("immediate_blockers", []):
            pattern = self._pattern_from_blocker(
                blocker=blocker,
                failure_mode=frame.get("failure_mode"),
            )
            if pattern and self._add_pattern(pattern):
                added += 1

        if added > 0:
            self.store["updated_at"] = datetime.utcnow().isoformat()

        logger.info(f"🧠 FailurePatternStore: added {added} new patterns")
        return added

    # -----------------------------
    # Pattern extraction helpers
    # -----------------------------

    def _pattern_from_tool_insight(self, insight: Dict[str, Any], failure_mode: str) -> Dict[str, Any]:
        """
        Convert a tool_insight entry into a normalized failure pattern.
        """
        pattern = {
            "pattern_type": "mechanism_constraint",
            "mechanism": insight.get("tool"),
            "constraint": insight.get("insight"),
            "status": insight.get("status"),
            "failure_mode": failure_mode,
            "evidence": insight.get("evidence"),
            "source": "tool_insight",
            "first_seen": datetime.utcnow().isoformat(),
        }
        return pattern

    def _pattern_from_blocker(self, blocker: str, failure_mode: str) -> Dict[str, Any] | None:
        """
        Heuristically promote immediate blockers into tentative failure patterns.
        Only promote blockers that describe mechanism-level constraints.
        """
        lower = blocker.lower()

        # Extremely conservative promotion rule
        if any(k in lower for k in ["does not", "unable to", "cannot", "prevent"]):
            return {
                "pattern_type": "mechanism_constraint",
                "mechanism": "unknown",
                "constraint": blocker,
                "status": "tentative",
                "failure_mode": failure_mode,
                "evidence": "immediate_blocker",
                "source": "task_state.immediate_blockers",
                "first_seen": datetime.utcnow().isoformat(),
            }

        return None

    # -----------------------------
    # Storage helpers
    # -----------------------------

    def _add_pattern(self, pattern: Dict[str, Any]) -> bool:
        """
        Deduplicate and store a pattern.
        """
        h = self._hash_pattern(pattern)
        if h in self._existing_hashes:
            return False

        pattern["_pattern_hash"] = h
        self.store["patterns"].append(pattern)
        self._existing_hashes.add(h)
        return True

    def _hash_pattern(self, pattern: Dict[str, Any]) -> str:
        """
        Hash only semantic fields (exclude timestamps).
        """
        hash_basis = {
            "pattern_type": pattern.get("pattern_type"),
            "mechanism": pattern.get("mechanism"),
            "constraint": pattern.get("constraint"),
            "status": pattern.get("status"),
            "failure_mode": pattern.get("failure_mode"),
            "source": pattern.get("source"),
        }
        txt = json.dumps(hash_basis, sort_keys=True, ensure_ascii=True)
        return hashlib.sha256(txt.encode("utf-8")).hexdigest()
