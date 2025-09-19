import argparse
import json
import logging
import os
import random
import uuid
import re
import math
import time
from typing import List, Dict, Any, Tuple

from templates import PLAN_TEMPLATE
from utils.llm_api import LLM
from Messages import SystemMessage, UserMessage


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# Add dedicated handler for llm_api logger (CWD-agnostic)
llm_api_logger = logging.getLogger('llm_api')
try:
    _log_dir = os.path.join(os.path.dirname(__file__), 'logs')
    os.makedirs(_log_dir, exist_ok=True)
    _log_path = os.path.join(_log_dir, 'llm_api.log')
    llm_api_file_handler = logging.FileHandler(_log_path, mode='a')
    llm_api_file_handler.setLevel(logging.INFO)
    llm_api_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', '%Y-%m-%d %H:%M:%S'))
    llm_api_logger.addHandler(llm_api_file_handler)
except Exception:
    # Fall back to default root handlers if file setup fails
    pass
llm_api_logger.setLevel(logging.INFO)

# Simple normalization targets (can be tuned per project needs)
STEPS_TARGET = 6
TIME_TARGET_MIN = 15


def _extract_minutes(metrics: Dict[str, Any]) -> int:
    time_m = metrics.get('time', {}) if isinstance(metrics, dict) else {}
    return int(time_m.get('actual_minutes', time_m.get('minutes_advanced', 0)) or 0)


def _need_improvement_score(metrics: Dict[str, Any]) -> float:
    """Map need deltas to [0,1], where improvements score higher.

    Currently uses hunger delta only if present: negative delta = improvement.
    Scaled linearly with cap at 5 minutes-equivalent of improvement.
    """
    if not isinstance(metrics, dict):
        return 0.0
    hunger = metrics.get('hunger', {}) or {}
    delta = hunger.get('delta')
    if delta is None:
        return 0.0
    try:
        delta_f = float(delta)
    except Exception:
        return 0.0
    if delta_f >= 0:
        return 0.0
    # Scale improvements: delta -5 -> 1.0, delta -2.5 -> 0.5, etc., capped in [0,1]
    return max(0.0, min(1.0, (-delta_f) / 5.0))


def compute_plan_score(plan_entry: Dict[str, Any]) -> Dict[str, Any]:
    """Compute composite plan score based on provided metrics and goal satisfaction.

    Uses a simple weighted formula:
    score = GS + 0.1*(1 - failure_rate) + 0.05*(1 - time_norm) + 0.05*(1 - steps_norm) + 0.05*need_improvement
    Where:
      - failure_rate = failures / max(1, steps_total)
      - time_norm = min(minutes / TIME_TARGET_MIN, 1)
      - steps_norm = min(steps_total / STEPS_TARGET, 1)
      - need_improvement in [0,1] from hunger delta (negative = good)
    """
    metrics: Dict[str, Any] = plan_entry.get('metrics', {}) or {}
    steps_info = metrics.get('steps', {}) if isinstance(metrics, dict) else {}
    steps_total = int(steps_info.get('total', 0) or 0)
    failures = int(steps_info.get('failures', 0) or 0)
    minutes = _extract_minutes(metrics)

    # Goal satisfaction may be provided at metrics or top-level
    goal_satisfaction = None
    if isinstance(metrics, dict):
        goal_satisfaction = metrics.get('goal_satisfaction')
    if goal_satisfaction is None:
        goal_satisfaction = plan_entry.get('goal_satisfaction')
    try:
        gs = float(goal_satisfaction)
    except Exception:
        gs = None

    failure_rate = failures / max(1, steps_total)
    time_norm = min(float(minutes) / float(TIME_TARGET_MIN), 1.0) if TIME_TARGET_MIN > 0 else 1.0
    steps_norm = min(float(steps_total) / float(STEPS_TARGET), 1.0) if STEPS_TARGET > 0 else 1.0
    need_improvement = _need_improvement_score(metrics)

    score = None
    if gs is not None:
        score = (
            gs
            + 0.1 * (1.0 - failure_rate)
            + 0.05 * (1.0 - time_norm)
            + 0.05 * (1.0 - steps_norm)
            + 0.05 * need_improvement
        )

    return {
        'goal_satisfaction': gs,
        'steps_total': steps_total,
        'failures': failures,
        'minutes': minutes,
        'failure_rate': failure_rate,
        'time_norm': time_norm,
        'steps_norm': steps_norm,
        'need_improvement': need_improvement,
        'composite_score': score,
    }

def _build_prompts_from_plan_log(plan_log: List[Dict[str, Any]]) -> Tuple[str, str]:
    """Construct system and user prompts for reflection from a list of plan_log entries.

    Expects each entry to contain: goal, prompt, plan, summary, metrics.
    """
    system_prompt = """Role: You are a conservative editor that proposes only CRUD updates to Part C of the given PLAN_TEMPLATE.
Authority: You must not alter Parts A or B, nor change their semantics. Your output is only a JSON object describing edits to Part C.
Contract: All examples/heuristics you add must be consistent with Part A actions/conditions and not exceed limits in Part B.
Output format (strict JSON, no prose):
{
  "updates": [
    {
      "op": "append" | "replace" | "delete",
      "block_id": "kebab-case-unique-id",
      "kind": "example" | "heuristic",
      "title": "Short human title",
      "rationale": "Why this is useful, ≤ 40 words",
      "content": "<exact block text to insert when op in ['append','replace']>"
    }
  ]
}

Wrap each block exactly like this so the applier can target it safely:

# BEGIN_C_BLOCK block_id=<BLOCK_ID> kind=<example|heuristic> title="<TITLE>"
{ ...JSON example or heuristic text... }
# END_C_BLOCK block_id=<BLOCK_ID>

Rules you must enforce before proposing updates:

No contradictions with Part A/B.

Step budget: Any { "plan": [...] } you add must have total steps (including nested bodies and all if-branches) ≤ max_total_steps_including_nested in Part B (currently 8).

Variable discipline: Any $var used must be bound earlier in the same example via a "scan" with "out":"var".

Allowed primitives only: Use only actions/conditions enumerated in Part A.

Examples ≤ 6 visible steps unless demonstrating control flow (then still ≤ 8 overall).

Prefer additions over deletions. Replace only to fix correctness, and keep the original as a commented legacy block inside content if possible.
What to edit: Only Part C: examples, reusable pattern guidance, and heuristics.
What not to edit: Part A, Part B, headings, or the final reminders.
Your JSON must parse. No code fences, no comments outside the content string.

"""

    user_prompt = f"""
#Plan syntax specification:
{PLAN_TEMPLATE}

"""

    for n, item in enumerate(plan_log):
        metrics = item.get('metrics', {})
        steps = metrics.get('steps', {})
        hunger = metrics.get('hunger', {})
        time_m = metrics.get('time', {})
        minutes = time_m.get('actual_minutes', time_m.get('minutes_advanced', 0))
        metrics_summary = (
            f"steps: {steps.get('total', 0)}, "
            f"failures: {steps.get('failures', 0)}, "
            f"minutes: {minutes}, "
            f"hunger Δ: {hunger.get('delta', 0)}"
        )

        user_prompt += f"""

#### Planning effort {n+1}

# Goal the plan was created for: 
{item.get('goal','')}

#Planning prompt:
{item.get('prompt','')}

#Resulting Plan:
{json.dumps(item.get('plan',{}), indent=2)}

#Plan metrics (Phase 1):
{metrics_summary}

#Plan post-mortem summary:
{item.get('summary','')}

"""

    user_prompt += """
Respond with an analysis of how the plan syntax and or the planning prompt could be improved to better achieve the goal. For example:
- If the plan syntax is ambiguous or unclear explain what is unclear and how to improve it.
- If an action alternative is missing or could be modified to be more appropriate, explain what is missing and how to improve it. Provide specific instances where possible.
- If the planning prompt contins inadequate information about the character's current situation for planning for the goal, suggest additional information that would improve the planning process.
- If the planning prompt instructions could be improved or re-arranged describe how to improve it.

Identify any other issues with the planning process and recommend improvements.

Provide your response as a list of items with full text descriptions, no other text.
End your response with </end>
"""

    return system_prompt, user_prompt


def review_planning(plan_log: List[Dict[str, Any]], server_name: str = "openai", model_name: str = "gpt-4.1") -> str:
    """Run the planning review using the provided plan_log entries and return the LLM response text."""
    if not plan_log or len(plan_log) < 3:
        logger.info("Not enough plan entries to review (need at least 3).")
        return ""

    system_prompt, user_prompt = _build_prompts_from_plan_log(plan_log)

    llm = LLM(server_name=server_name, model_name=model_name)
    logger.info('📝 Invoking LLM for planning review')
    response = llm.ask({}, [SystemMessage(content=system_prompt), UserMessage(content=user_prompt)],
        max_tokens=800,
        stops=['</end>'],
        is_json=False
    )
    text = response if isinstance(response, str) else getattr(response, 'text', str(response))
    logger.info('📝 Planning review completed')

    # Compute and log per-plan composite scores (does not change LLM output)
    try:
        for idx, entry in enumerate(plan_log):
            scores = compute_plan_score(entry)
            logger.info(
                f"Plan {idx+1} scoring » GS={scores['goal_satisfaction']} "
                f"fail_rate={scores['failure_rate']:.2f} time_norm={scores['time_norm']:.2f} "
                f"steps_norm={scores['steps_norm']:.2f} need_impr={scores['need_improvement']:.2f} "
                f"composite={scores['composite_score']}"
            )
    except Exception as e:
        logger.warning(f"Failed to compute plan scores: {e}")
    return text


def _load_plan_log_from_jsonl(path: str, max_entries: int = 20) -> List[Dict[str, Any]]:
    """Load most recent plan_log entries from a JSONL file (each line a JSON object)."""
    if not os.path.exists(path):
        logger.error(f"Plan log file not found: {path}")
        return []
    entries: List[Dict[str, Any]] = []
    try:
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                    entries.append(obj)
                except Exception:
                    continue
    except Exception as e:
        logger.error(f"Failed to read plan log: {e}")
        return []
    # Keep only the most recent max_entries
    return entries[-max_entries:]


# -------------------------
# New V2 multi-stage pipeline (Stage 1 implemented, others as stubs)
# -------------------------

_S1_PRIORITIES = ["validity", "goal", "efficiency", "realism", "strategy"]


def _format_plan_text_block(plan_id: str, item: Dict[str, Any], idx: int) -> str:
    """Format a textual block for a single plan entry (not JSON),
    mirroring the existing review prompt structure and adding a Plan ID.
    """
    metrics = item.get('metrics', {}) or {}
    steps = metrics.get('steps', {}) or {}
    hunger = metrics.get('hunger', {}) or {}
    time_m = metrics.get('time', {}) or {}
    minutes = time_m.get('actual_minutes', time_m.get('minutes_advanced', 0))
    metrics_summary = (
        f"steps: {steps.get('total', 0)}, "
        f"failures: {steps.get('failures', 0)}, "
        f"minutes: {minutes}, "
        f"hunger Δ: {hunger.get('delta', 0)}"
    )
    goal = item.get('goal', '') or ''
    prompt = item.get('prompt', '') or ''
    plan_obj = item.get('plan', {}) or {}
    summary = item.get('summary', '') or ''
    # Build a compact structured appendix with only new fields when present
    appendix_actions = []
    try:
        for ar in item.get('actions', []) or []:
            try:
                step_id = ar.get('step_id')
                action_type = (ar.get('action') or {}).get('type')
                bindings_after = ar.get('bindings_after')
                binding_evidence = ar.get('binding_evidence')
                feature_snapshot = ar.get('feature_snapshot')
                if bindings_after or binding_evidence or feature_snapshot:
                    appendix_actions.append({
                        'step_id': step_id,
                        'type': action_type,
                        **({'bindings_after': bindings_after} if bindings_after else {}),
                        **({'binding_evidence': binding_evidence} if binding_evidence else {}),
                        **({'feature_snapshot': feature_snapshot} if feature_snapshot else {}),
                    })
            except Exception:
                continue
    except Exception:
        appendix_actions = []

    structured_appendix = ""
    if appendix_actions:
        structured_appendix = (
            "\n#Structured appendix (selected fields)\n"
            + json.dumps({"actions": appendix_actions}, ensure_ascii=False, indent=2)
            + "\n"
        )

    return (
        f"# Plan ID: {plan_id}\n\n"
        f"# Goal the plan was created for:\n{goal}\n\n"
        f"#Planning prompt:\n{prompt}\n\n"
        f"#Resulting Plan:\n{json.dumps(plan_obj, indent=2)}\n\n"
        f"#Plan metrics (Phase 1):\n{metrics_summary}\n\n"
        f"#Plan post-mortem summary:\n{summary}\n"
        f"{structured_appendix}"
    )


def _sample_microbatch(plan_log: List[Dict[str, Any]], k: int = 5, seed: Any = None, exclude_idxs: set = None) -> Tuple[List[Tuple[str, Dict[str, Any]]], List[int]]:
    """Randomly sample up to k entries without replacement vs exclude_idxs and pair with unique plan_id.

    Returns (microbatch, chosen_indices).
    """
    if not plan_log:
        return [], []
    n = min(len(plan_log), max(1, k))
    rng = random.Random(seed)
    available_idxs = [i for i in range(len(plan_log)) if not exclude_idxs or i not in exclude_idxs]
    if not available_idxs:
        return [], []
    chosen = rng.sample(available_idxs, min(n, len(available_idxs)))
    micro = [(f"plan-{uuid.uuid4().hex[:8]}", plan_log[i]) for i in chosen]
    return micro, chosen


def _build_s1_prompts_textual(microbatch: List[Tuple[str, Dict[str, Any]]], batch_id: str) -> Tuple[str, str]:
    """Construct S1 prompts using PLAN_TEMPLATE and a textual MICRO_BATCH."""
    system_prompt = (
        "You are a precise auditor. Extract concrete problems from the given plans and metrics. "
        "Do not propose fixes. Do not invent data. Keep outputs short and evidence-linked."
    )

    parts: List[str] = []
    parts.append(
        "You will receive:\n"
        "- PART_A, PART_B (immutable constraints)\n"
        "- PART_C (current examples/heuristics; for reference only)\n"
        "- MICRO_BATCH: 3–5 plan logs as textual blocks (with Plan ID, plan JSON, metrics summary, prompt/summary),\n"
        "  each delimited by '----- MICRO_BATCH ENTRY (plan_id=...) -----' and '----- END MICRO_BATCH ENTRY (plan_id=...) -----'\n"
    )
    parts.append(
        f"Batch metadata:\n- BATCH_ID: {batch_id}\n- PID FORMAT: '<BATCH_ID>:<plan_id>:p<k>' (use 1-based k per plan)\n"
    )
    parts.append(
        "Tasks:\n"
        "1) For each plan, enumerate specific problems found (zero or more).\n"
        "2) For every problem, include: textual issue, evidence (a minimal quote or JSON pointer), impacted priority, severity (1–5), and plan_id.\n"
        "3) Do NOT cluster; emit a flat list.\n"
        "4) If a plan has no valid problems under the rules below, emit no entries for that plan (do not invent issues).\n"
    )
    parts.append(
        "Evidence guidance:\n"
        "- Prefer evidence_ptr into plan_json or into the Structured appendix (bindings_after, binding_evidence.distance, feature_snapshot.distance_to_target).\n"
        "- If an evidence_ptr is available, include it (required). Use evidence_quote only if a pointer is not possible.\n"
    )
    parts.append(
        "Classification guidance:\n"
        "- validity: only schema/semantic violations vs Part A/B or Plan Template (incl. exceeding Part B step budget for static plan structure only).\n"
        "- efficiency: longer-than-needed plans, redundant steps, avoidable scans, unnecessary branching/time.\n"
        "- strategy: suboptimal choices to pursue the goal (still aligned with the goal).\n"
        "- goal: direct contradiction with the stated goal or failure to pursue it at all.\n"
        "- realism: implausible/physically impossible assumptions or steps.\n"
        "If uncertain between classes, prefer efficiency over goal; prefer strategy over goal.\n"
    )
    parts.append(
        "Severity rubric (use the full 1–5 range across the batch):\n"
        "1: cosmetic/minor clarity (no measurable cost).\n"
        "2: small inefficiency or low-risk ambiguity (≤10% extra steps/time).\n"
        "3: moderate inefficiency or non-critical misprioritization (10–30% extra; mild failure risk).\n"
        "4: major inefficiency or likely failure without external help (30–60% extra; clear risk).\n"
        "5: critical: invalid plan or near-certain failure (schema violation, forbidden action, exceeds Part B).\n"
    )
    parts.append(
        "Metrics guidance:\n"
        "- Use steps.total, failures/total, minutes, hunger delta, and any composite/goal satisfaction to calibrate impact and severity.\n"
        "- Downweight inefficiency if failures=0 and hunger delta indicates clear improvement.\n"
        "- Prefer efficiency (not goal) when the plan meets the goal but with excess steps/time.\n"
        "- If hunger delta is strongly negative (improvement), do not flag a 'goal' violation unless the plan contradicts the goal text.\n"
        "- Do not use runtime step counts or loop iterations to claim a Part B step-budget validity violation; that budget applies to static plan structure. Treat runtime excess as efficiency.\n"
        "- Use failure_rate, time_norm, and steps_norm to avoid extremes unless warranted.\n"
        "- Use new structured signals when present: bindings_after to confirm variable resolution; binding_evidence.distance for nearest/efficiency judgments; feature_snapshot.distance_to_target to calibrate severity and avoid mislabeling goal vs efficiency.\n"
    )
    parts.append(
        "Batch calibration:\n"
        "- Avoid assigning only 1s and 5s; distribute severities 2–4 where appropriate based on metrics and impact.\n"
        "- Do not duplicate essentially the same issue for the same plan; prefer one precise problem with the best single evidence link.\n"
    )
    parts.append(
        "Context-aware goal checks:\n"
        "- Do not flag 'goal' if a step addresses a closely related survival need unless the goal text explicitly excludes it; prefer 'strategy' or 'efficiency'.\n"
    )
    parts.append(f"Priorities (fixed): {json.dumps(_S1_PRIORITIES)}\n")
    parts.append(
        "Output (strict JSON):\n"
        "{\n"
        "  \"problems\":[\n"
        "    {\n"
        "      \"pid\":\"<BATCH_ID>:<plan_id>:p1\",\n"
        "      \"plan_id\":\"{{plan_id}}\",\n"
        "      \"issue\":\"short text\",\n"
        "      \"evidence_ptr\":\"/plan/0/type\",\n"
        "      \"evidence_quote\":\"optional minimal quote\",\n"
        "      \"impact\":\"validity|goal|efficiency|realism|strategy\",\n"
        "      \"severity\": 1\n"
        "    }\n"
        "  ]\n"
        "}\n"
    )
    parts.append("INPUT:\n")
    parts.append("PLAN_TEMPLATE:\n")
    parts.append(str(PLAN_TEMPLATE))
    parts.append("\nMICRO_BATCH:\n")
    parts.append("(Some plans may include a 'Structured appendix (selected fields)' section for precise evidence_ptrs.)\n")

    # Add each entry with a clear separator and embedded plan_id
    for idx, (plan_id, entry) in enumerate(microbatch):
        parts.append(f"----- MICRO_BATCH ENTRY (plan_id={plan_id}) -----\n")
        parts.append(_format_plan_text_block(plan_id, entry, idx))
        parts.append(f"----- END MICRO_BATCH ENTRY (plan_id={plan_id}) -----\n")

    user_prompt = "\n".join(parts)
    return system_prompt, user_prompt


def _extract_part_a_b_from_template(template_text: str) -> Tuple[str, str]:
    """Best-effort extraction of PART_A and PART_B from PLAN_TEMPLATE.
    PART_A: actions, conditions, required_fields, variables
    PART_B: constraints like max_steps, expand_patterns, no_sequential_say
    """
    # Heuristic: parse JSON-like meta-spec block inside PLAN_TEMPLATE
    # Fallback to entire template if parsing fails
    part_a = ""
    part_b = ""
    try:
        # Find the first JSON block after 'Meta-spec'
        meta_match = re.search(r"Meta-spec.*?\{([\s\S]*?)\}\s*\n", template_text)
        if meta_match:
            meta_json_str = "{" + meta_match.group(1) + "}"
            # Relax trailing commas
            meta_json_str = re.sub(r",\s*\}\s*$", "}\n", meta_json_str, flags=re.M)
            meta = json.loads(meta_json_str)
            # Construct PART_A and PART_B
            part_a_obj = {
                "actions": meta.get("actions"),
                "conditions": meta.get("conditions"),
                "required_fields": meta.get("required_fields"),
                "variables": meta.get("variables"),
            }
            part_b_obj = {
                k: meta.get(k)
                for k in ["max_steps", "expand_patterns", "no_sequential_say"]
                if k in meta
            }
            part_a = json.dumps(part_a_obj, ensure_ascii=False, indent=2)
            part_b = json.dumps(part_b_obj, ensure_ascii=False, indent=2)
        else:
            part_a = template_text
            part_b = "{}"
    except Exception:
        part_a = template_text
        part_b = "{}"
    return part_a, part_b


def _build_s2_prompts(problems_list: List[Dict[str, Any]]) -> Tuple[str, str]:
    """Construct Stage 2 prompts using PART_A, PART_B and aggregated S1 problems."""
    part_a, part_b = _extract_part_a_b_from_template(str(PLAN_TEMPLATE))
    system_prompt = (
        "You are a taxonomy builder. Group related problem items into emergent problem classes. "
        "Create short, stable ids."
    )
    user_parts: List[str] = []
    user_parts.append(
        "You will receive:\n- PART_A, PART_B (immutable)\n- S1_PROBLEMS: array aggregated from multiple S1 runs {problems:[...]}\n"
    )
    user_parts.append(
        "Tasks:\n"
        "1) Cluster problems into classes. Name each class and assign a stable id: class_id must be kebab-case and prefixed \"pc-\".\n"
        "2) For each class, list member problem ids and compute urgency = max(severity among members).\n"
        "3) Map each class to one or more priorities from:\n   [\"validity\",\"goal\",\"efficiency\",\"realism\",\"strategy\"]\n"
        "4) Produce a global priority order (ties broken by number of affected problems, then max severity).\n"
    )
    user_parts.append(
        "Output (strict JSON):\n{\n  \"classes\":[\n    {\n      \"class_id\":\"pc-{{kebab}}\",\n      \"name\":\"short human name\",\n      \"problems\":[\"p1\",\"p7\"],\n      \"priorities\":[\"validity\",\"efficiency\"],\n      \"urgency\":5\n    }\n  ],\n  \"priority_order\":[\"validity\",\"goal\",\"efficiency\",\"realism\",\"strategy\"]\n}\n"
    )
    user_parts.append(
        "Also include: problem_to_classes (map pid -> [class_id]), and priority_rank (map priority -> numeric rank, higher is more important; e.g., validity:5, goal:4, efficiency:3, realism:2, strategy:1).\n"
    )
    user_parts.append("INPUT:\n")
    user_parts.append("PART_A:\n")
    user_parts.append(part_a)
    user_parts.append("\nPART_B:\n")
    user_parts.append(part_b)
    user_parts.append("\nS1_PROBLEMS:\n")
    user_parts.append(json.dumps({"problems": problems_list}, ensure_ascii=False, indent=2))
    user_prompt = "\n".join(user_parts)
    return system_prompt, user_prompt


def _aggregate_s1_problems_from_jsonl(path: str) -> List[Dict[str, Any]]:
    """Read a JSONL file of Stage 1 outputs and return union of all problems arrays."""
    results: List[Dict[str, Any]] = []
    if not path or not os.path.exists(path):
        return results
    try:
        with open(path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                    probs = obj.get('problems') if isinstance(obj, dict) else None
                    if isinstance(probs, list):
                        results.extend([p for p in probs if isinstance(p, dict)])
                except Exception:
                    continue
    except Exception:
        return results
    return results


def _build_s3_prompts(classes_obj: Dict[str, Any], max_updates: int) -> Tuple[str, str]:
    """Construct Stage 3 prompts using full PLAN_TEMPLATE as PART_C context and CLASSES."""
    system_prompt = (
        "You are a planner ensuring coverage. Plan the minimum set of updates to cover all classes and touch every global priority at least once."
    )
    user_parts: List[str] = []
    user_parts.append("You will receive:\n- PART_C (current)\n- CLASSES: from S2 (classes + priority_order)\n")
    user_parts.append(
        "Constraints:\n"
        "- Updates are CRUD on Part C only.\n"
        "- Each class must be addressed by ≥1 update (example or heuristic).\n"
        "- Each global priority must be addressed by ≥1 update overall.\n"
        f"- Target ≤ {{MAX_UPDATES}} total updates (prefer heuristics unless an example is necessary to demonstrate safety/validity).\n"
    )
    user_parts.append(
        "Output (strict JSON):\n{\n  \"coverage\":[\n    {\n      \"class_id\":\"pc-{{id}}\",\n      \"updates\":[\n        {\"op\":\"append\",\"kind\":\"heuristic\",\"title\":\"…\"},\n        {\"op\":\"append\",\"kind\":\"example\",\"title\":\"…\"}\n      ]\n    }\n  ],\n  \"global_priorities_addressed\":[\"validity\",\"goal\",\"efficiency\",\"realism\",\"strategy\"]\n}\n"
    )
    user_parts.append("INPUT:\n")
    user_parts.append("PART_C (embedded within full PLAN_TEMPLATE):\n")
    user_parts.append(str(PLAN_TEMPLATE))
    user_parts.append("\nCLASSES:\n")
    user_parts.append(json.dumps(classes_obj, ensure_ascii=False, indent=2))
    user_parts.append("\nMAX_UPDATES:\n")
    user_parts.append(str(max_updates))
    user_prompt = "\n".join(user_parts)
    return system_prompt, user_prompt


def stage3_coverage(classes: Dict[str, Any], max_updates: int = 8, output_path: str = None) -> Dict[str, Any]:
    """Plan coverage updates: ensure every class and each global priority is addressed at least once.

    Writes result once to output_path (overwrite) if provided.
    """
    system_prompt, user_prompt = _build_s3_prompts(classes, max_updates)
    llm = LLM(server_name="openai", model_name="gpt-4.1")
    response = llm.ask(
        {},
        [SystemMessage(content=system_prompt), UserMessage(content=user_prompt)],
        max_tokens=900,
        is_json=True
    )
    if isinstance(response, str):
        try:
            result = json.loads(response)
        except Exception:
            result = {"coverage": [], "global_priorities_addressed": []}
    elif isinstance(response, dict):
        result = response
    else:
        text = getattr(response, 'text', None)
        try:
            result = json.loads(text) if text else {"coverage": [], "global_priorities_addressed": []}
        except Exception:
            result = {"coverage": [], "global_priorities_addressed": []}

    if output_path:
        try:
            dirpath = os.path.dirname(output_path)
            if dirpath:
                os.makedirs(dirpath, exist_ok=True)
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
        except Exception:
            pass
    return result


def _build_s5_prompts(classes_obj: Dict[str, Any], coverage_obj: Dict[str, Any], updates_s4: Dict[str, Any], max_steps: int) -> Tuple[str, str]:
    """Construct Stage 5 prompts using PART_A, PART_B, CLASSES, COVERAGE, and S4 UPDATES."""
    part_a, part_b = _extract_part_a_b_from_template(str(PLAN_TEMPLATE))
    system_prompt = (
        "You are a strict validator. Check the S4 updates against constraints. "
        "If any check fails, output a corrected updates JSON that passes all checks. Otherwise, echo the original updates JSON unchanged.\n"
        "Checks:\n"
        "1) JSON parses; only CRUD ops; A/B untouched.\n"
        "2) Each update has block_id starting with some class_id from CLASSES.\n"
        "3) Content is wrapped with BEGIN/END_C_BLOCK and consistent block_id/kind/title.\n"
        "4) Examples obey MAX_STEPS and variable discipline; only allowed actions/conditions; no sequential \"say\".\n"
        "5) Coverage: every class_id in CLASSES appears in at least one update; each global priority appears at least once (infer from update titles/rationales).\n"
        "6) No contradictions with PART_A/PART_B.\n\n"
        "Action:\n- If all checks pass: output the updates JSON as-is.\n- Else: minimally repair to pass all checks; prefer turning overly long examples into heuristics, or trimming steps.\n\n"
        "Output: (strict JSON, same schema as S4)"
    )
    user_parts: List[str] = []
    user_parts.append("INPUTS:\n")
    user_parts.append("PART_A:\n")
    user_parts.append(part_a)
    user_parts.append("\nPART_B:\n")
    user_parts.append(part_b)
    user_parts.append("\nCLASSES:\n")
    user_parts.append(json.dumps(classes_obj, ensure_ascii=False, indent=2))
    user_parts.append("\nCOVERAGE:\n")
    user_parts.append(json.dumps(coverage_obj, ensure_ascii=False, indent=2))
    user_parts.append("\nUPDATES_S4:\n")
    user_parts.append(json.dumps(updates_s4, ensure_ascii=False, indent=2))
    user_parts.append("\nMAX_STEPS:\n")
    user_parts.append(str(max_steps))
    user_prompt = "\n".join(user_parts)
    return system_prompt, user_prompt


def stage5_validate_and_repair(
    classes: Dict[str, Any],
    coverage: Dict[str, Any],
    updates_s4: Dict[str, Any],
    output_path: str = None,
) -> Dict[str, Any]:
    """Validate S4 output against constraints and minimally repair if needed. Writes once if output_path provided."""
    # Determine MAX_STEPS from PART_B if available; fallback to 8
    part_a, part_b = _extract_part_a_b_from_template(str(PLAN_TEMPLATE))
    max_steps = 8
    try:
        pb = json.loads(part_b)
        if isinstance(pb, dict) and isinstance(pb.get('max_steps'), int):
            max_steps = int(pb['max_steps'])
    except Exception:
        pass

    system_prompt, user_prompt = _build_s5_prompts(classes, coverage, updates_s4, max_steps)
    llm = LLM(server_name="openai", model_name="gpt-4.1")
    response = llm.ask(
        {},
        [SystemMessage(content=system_prompt), UserMessage(content=user_prompt)],
        max_tokens=1500,
        is_json=True
    )
    if isinstance(response, str):
        try:
            result = json.loads(response)
        except Exception:
            result = updates_s4 or {"updates": []}
    elif isinstance(response, dict):
        result = response
    else:
        text = getattr(response, 'text', None)
        try:
            result = json.loads(text) if text else (updates_s4 or {"updates": []})
        except Exception:
            result = updates_s4 or {"updates": []}

    if output_path:
        try:
            dirpath = os.path.dirname(output_path)
            if dirpath:
                os.makedirs(dirpath, exist_ok=True)
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
        except Exception:
            pass
    return result


def stage1_diagnose(
    plan_log: List[Dict[str, Any]],
    server_name: str = "openai",
    model_name: str = "gpt-4.1",
    microbatch_size: int = 5,
    seed: Any = None,
    output_path: str = "data/reflection_stage1.json",
    iterations: int = 0,
) -> Dict[str, Any]:
    """Run Stage 1 Diagnose → Problem list on a random micro-batch and return JSON.

    Also writes the result to output_path if provided (for separate staged apps).
    """
    # Auto iterations: min(5, ceil(#plan_logs / microbatch_size)) when iterations <= 0
    if isinstance(iterations, int) and iterations > 0:
        total_iterations = min(5, int(iterations))
    else:
        denom = max(1, int(microbatch_size) if microbatch_size else 5)
        auto_iters = int(math.ceil(len(plan_log) / denom))
        total_iterations = min(5, max(1, auto_iters))
    rng_seed_base = seed
    used_indices: set = set()
    aggregated: List[Dict[str, Any]] = []
    last_result: Dict[str, Any] = {"problems": []}
    llm = LLM(server_name=server_name, model_name=model_name)
    # Create a run-level batch identifier to construct globally unique pids
    batch_id = f"s1-{int(time.time())}-{uuid.uuid4().hex[:6]}"

    for it in range(total_iterations):
        microbatch, chosen = _sample_microbatch(
            plan_log, k=microbatch_size, seed=(None if rng_seed_base is None else rng_seed_base + it), exclude_idxs=used_indices
        )
        if not microbatch:
            break
        used_indices.update(chosen)
        system_prompt, user_prompt = _build_s1_prompts_textual(microbatch, batch_id)
        response = llm.ask(
            {},
            [SystemMessage(content=system_prompt), UserMessage(content=user_prompt)],
            max_tokens=800,
            is_json=True
        )
        if isinstance(response, str):
            try:
                last_result = json.loads(response)
            except Exception:
                last_result = {"problems": []}
        elif isinstance(response, dict):
            last_result = response
        else:
            text = getattr(response, 'text', None)
            try:
                last_result = json.loads(text) if text else {"problems": []}
            except Exception:
                last_result = {"problems": []}

        # Persist each iteration result as a JSONL line
        if output_path:
            try:
                dirpath = os.path.dirname(output_path)
                if dirpath:
                    os.makedirs(dirpath, exist_ok=True)
                with open(output_path, 'a', encoding='utf-8') as f:
                    f.write(json.dumps(last_result, ensure_ascii=False))
                    f.write("\n")
            except Exception:
                pass

        # Aggregate problems across iterations
        probs = last_result.get('problems') if isinstance(last_result, dict) else None
        if isinstance(probs, list):
            aggregated.extend([p for p in probs if isinstance(p, dict)])

    return {"problems": aggregated}


def stage2_cluster(problems: Dict[str, Any], output_path: str = None) -> Dict[str, Any]:
    """Cluster S1 problems into emergent classes with stable IDs and priority map using LLM.

    Accepts either a dict containing {"problems": [...]}, a list of problem dicts, or an empty/None value.
    If empty, this function expects the orchestrator to pass aggregated problems via file.
    """
    problems_list: List[Dict[str, Any]] = []
    if isinstance(problems, dict) and 'problems' in problems:
        problems_list = problems.get('problems') or []
    elif isinstance(problems, list):
        problems_list = problems
    else:
        problems_list = []

    # Build S2 prompts
    system_prompt, user_prompt = _build_s2_prompts(problems_list)

    llm = LLM(server_name="openai", model_name="gpt-4.1")
    response = llm.ask(
        {},
        [SystemMessage(content=system_prompt), UserMessage(content=user_prompt)],
        max_tokens=900,
        is_json=True
    )

    if isinstance(response, str):
        try:
            result = json.loads(response)
        except Exception:
            result = {"classes": [], "priority_order": list(_S1_PRIORITIES)}
    elif isinstance(response, dict):
        result = response
    else:
        text = getattr(response, 'text', None)
        try:
            result = json.loads(text) if text else {"classes": [], "priority_order": list(_S1_PRIORITIES)}
        except Exception:
            result = {"classes": [], "priority_order": list(_S1_PRIORITIES)}

    # Persist Stage 2 output once (overwrite) if a path is provided
    if output_path:
        try:
            dirpath = os.path.dirname(output_path)
            if dirpath:
                os.makedirs(dirpath, exist_ok=True)
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    return result


def _build_s4_prompts(classes_obj: Dict[str, Any], coverage_obj: Dict[str, Any], max_steps: int) -> Tuple[str, str]:
    """Construct Stage 4 prompts using PART_A, PART_B, full PLAN_TEMPLATE as PART_C, CLASSES, and COVERAGE."""
    part_a, part_b = _extract_part_a_b_from_template(str(PLAN_TEMPLATE))
    system_prompt = (
        "You are a conservative editor. Produce only CRUD updates for Part C. Do not alter A/B semantics. "
        "Examples must obey budgets and variable discipline."
    )
    user_parts: List[str] = []
    user_parts.append(
        "Format rules:\n"
        "- Output ONLY the JSON object:\n  {\"updates\":[{...}]}\n"
        "- For every update, include \"block_id\" that starts with the class_id (e.g., \"pc-var-safety-guard-1\").\n"
        "- Wrap content EXACTLY as:\n\n"
        "# BEGIN_C_BLOCK block_id=<BLOCK_ID> kind=<example|heuristic> title=\"<TITLE>\"\n"
        "{ ...JSON example or heuristic text... }\n"
        "# END_C_BLOCK block_id=<BLOCK_ID>\n"
    )
    user_parts.append(
        "Safety rules to enforce:\n"
        f"- Steps ≤ PART_B.max_total_steps_including_nested (currently {max_steps})\n"
        "- Only actions/conditions in PART_A\n"
        "- No sequential \"say\"\n"
        "- Every $var is bound earlier via \"scan\" with \"out\"\n"
        "- If demonstrating environment use (e.g., Spring), show required preconditions (e.g., near) consistent with PART_A\n"
    )
    user_parts.append(
        "Output schema (strict):\n{\n  \"updates\":[\n    {\n      \"op\":\"append\"|\"replace\"|\"delete\",\n      \"block_id\":\"kebab-case-unique-id\",\n      \"kind\":\"example\"|\"heuristic\",\n      \"title\":\"Short human title\",\n      \"rationale\":\"≤ 40 words\",\n      \"content\":\"<BEGIN/END_C_BLOCK wrapped content as above>\"\n    }\n  ]\n}\n"
    )
    user_parts.append(
        "Additional requirements:\n"
        "- block_id must start with the target class_id and include a deterministic suffix (e.g., short hash of title) to avoid duplicates on re-runs.\n"
        "- Include source_pids: [\"<BATCH_ID>:<plan_id>:p1\", ...] to trace updates back to problems addressed.\n"
    )
    user_parts.append("INPUTS:\n")
    user_parts.append("PART_A:\n")
    user_parts.append(part_a)
    user_parts.append("\nPART_B:\n")
    user_parts.append(part_b)
    user_parts.append("\nPART_C (embedded within full PLAN_TEMPLATE):\n")
    user_parts.append(str(PLAN_TEMPLATE))
    user_parts.append("\nCLASSES:\n")
    user_parts.append(json.dumps(classes_obj, ensure_ascii=False, indent=2))
    user_parts.append("\nCOVERAGE:\n")
    user_parts.append(json.dumps(coverage_obj, ensure_ascii=False, indent=2))
    user_prompt = "\n".join(user_parts)
    return system_prompt, user_prompt


def stage4_draft_updates(classes: Dict[str, Any], coverage_plan: Dict[str, Any], output_path: str = None) -> Dict[str, Any]:
    """Draft CRUD updates with BEGIN_C_BLOCK/END_C_BLOCK wrappers.

    Writes result once to output_path (overwrite) if provided.
    """
    # Determine MAX_STEPS from PART_B if available; fallback to 8
    part_a, part_b = _extract_part_a_b_from_template(str(PLAN_TEMPLATE))
    max_steps = 8
    try:
        pb = json.loads(part_b)
        if isinstance(pb, dict) and isinstance(pb.get('max_steps'), int):
            max_steps = int(pb['max_steps'])
    except Exception:
        pass

    system_prompt, user_prompt = _build_s4_prompts(classes, coverage_plan, max_steps)
    llm = LLM(server_name="openai", model_name="gpt-4.1")
    response = llm.ask(
        {},
        [SystemMessage(content=system_prompt), UserMessage(content=user_prompt)],
        max_tokens=1500,
        is_json=True
    )
    if isinstance(response, str):
        try:
            result = json.loads(response)
        except Exception:
            result = {"updates": []}
    elif isinstance(response, dict):
        result = response
    else:
        text = getattr(response, 'text', None)
        try:
            result = json.loads(text) if text else {"updates": []}
        except Exception:
            result = {"updates": []}

    if output_path:
        try:
            dirpath = os.path.dirname(output_path)
            if dirpath:
                os.makedirs(dirpath, exist_ok=True)
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
        except Exception:
            pass
    return result


def stage5_validate_and_repair(
    updates: Dict[str, Any],
    classes: Dict[str, Any],
    priorities: List[str]
) -> Dict[str, Any]:
    """Stub: validate S4 output and repair if needed."""
    return {"updates": updates.get("updates", [])}


def run_reflection_pipeline_v2(
    plan_log: List[Dict[str, Any]],
    server_name: str = "openai",
    model_name: str = "gpt-4.1",
    microbatch_size: int = 5,
    seed: Any = None,
    s1_output_path: str = "data/reflection_stage1.json",
    s1_iterations: int = 1,
    s2_output_path: str = "data/reflection_stage2.json",
    s3_output_path: str = "data/reflection_stage3.json",
    s3_max_updates: int = 8,
    s4_output_path: str = "data/reflection_stage4.json",
    s5_output_path: str = "data/reflection_stage5.json",
) -> Dict[str, Any]:
    """Orchestrate the new multi-stage pipeline; S1 implemented, S2–S5 are stubs."""
    s1 = stage1_diagnose(
        plan_log,
        server_name=server_name,
        model_name=model_name,
        microbatch_size=microbatch_size,
        seed=seed,
        output_path=s1_output_path,
        iterations=s1_iterations,
    )
    # Aggregate all S1 problems from the JSONL file to form the union for S2
    aggregated_problems = {"problems": _aggregate_s1_problems_from_jsonl(s1_output_path)}
    s2 = stage2_cluster(aggregated_problems, output_path=s2_output_path)
    s3 = stage3_coverage(s2, max_updates=s3_max_updates, output_path=s3_output_path)
    s4 = stage4_draft_updates(s2, s3, output_path=s4_output_path)
    s5 = stage5_validate_and_repair(s4, s2, s3, output_path=s5_output_path)
    
    return {"s1": s1, "s2": s2, "s3": s3, "s4": s4, "s5": s5}

def main():
    parser = argparse.ArgumentParser(description='Run planning reflection over saved plan logs (v1) or the new pipeline v2')
    parser.add_argument('--plans', default='data/plans.jsonl', help='Path to plans JSONL file')
    parser.add_argument('--analysis', default=None, help='Analysis run directory name or path (e.g., Francoise-YYYYMMDD-HHMMSS). Bare names resolve under analysis/.')
    parser.add_argument('--min', dest='min_entries', type=int, default=3, help='Minimum entries required to run review')
    parser.add_argument('--max', dest='max_entries', type=int, default=20, help='Max entries to load from JSONL')
    parser.add_argument('--server', default='openai', help='LLM server name')
    parser.add_argument('--model', default='gpt-4.1', help='LLM model name')
    parser.add_argument('--v2', action='store_true', default=True, help='Run the new multi-stage pipeline v2 (Stage 1 implemented)')
    parser.add_argument('--microbatch', type=int, default=5, help='Micro-batch size for Stage 1 (v2)')
    parser.add_argument('--seed', type=int, default=None, help='Random seed for micro-batch sampling (v2)')
    parser.add_argument('--s1_out', default='data/reflection_stage1.json', help='Output path for Stage 1 results (v2)')
    parser.add_argument('--s1_iters', type=int, default=0, help='Number of Stage 1 iterations (v2, max 5). 0 = auto (min(5, ceil(#plans/microbatch))). Sampling without replacement within a run.')
    parser.add_argument('--s2_out', default='data/reflection_stage2.json', help='Output path for Stage 2 results (v2)')
    parser.add_argument('--s3_out', default='data/reflection_stage3.json', help='Output path for Stage 3 results (v2)')
    parser.add_argument('--s3_max', type=int, default=8, help='Max total updates to target in Stage 3 coverage planning (v2)')
    parser.add_argument('--s4_out', default='data/reflection_stage4.json', help='Output path for Stage 4 results (v2)')
    parser.add_argument('--s5_out', default='data/reflection_stage5.json', help='Output path for Stage 5 results (v2)')
    # Replan harness (offline) flags
    parser.add_argument('--replan', action='store_true', help='Replan a single entry from plan log using pure planner (testing)')
    parser.add_argument('--replan_mode', choices=['first','random','index'], default='first', help='Selection mode for replan entry')
    parser.add_argument('--replan_index', type=int, default=0, help='Index for replan when mode=index')

    args = parser.parse_args()

    # Resolve analysis directory if provided (assumes working dir is src/Metrics)
    run_dir = None
    if args.analysis:
        # Treat bare names as analysis/<name>, otherwise use as path
        if os.sep in args.analysis or args.analysis.startswith('.') or args.analysis.startswith('/'):
            run_dir = args.analysis
        else:
            run_dir = os.path.join('analysis', args.analysis)
        if not os.path.isdir(run_dir):
            logger.error(f"Analysis directory not found: {run_dir}")
            return
        # Find exactly one *-plans.jsonl in run_dir
        candidates = [f for f in os.listdir(run_dir) if f.endswith('-plans.jsonl')]
        if len(candidates) == 0:
            logger.error(f"No *-plans.jsonl found in {run_dir}")
            return
        if len(candidates) > 1:
            logger.error(f"Multiple plan files found in {run_dir}: {candidates}. Please specify --plans instead.")
            return
        resolved_plans = os.path.join(run_dir, candidates[0])
        plans_path = resolved_plans
    else:
        plans_path = args.plans

    plan_log = _load_plan_log_from_jsonl(plans_path, max_entries=args.max_entries)

    # Run new v2 pipeline if requested
    if args.v2 and not args.replan:
        # Determine output paths: if --analysis provided, write all stage outputs to that directory
        if run_dir:
            s1_out = os.path.join(run_dir, 'reflection_stage1.json')
            s2_out = os.path.join(run_dir, 'reflection_stage2.json')
            s3_out = os.path.join(run_dir, 'reflection_stage3.json')
            s4_out = os.path.join(run_dir, 'reflection_stage4.json')
            s5_out = os.path.join(run_dir, 'reflection_stage5.json')
        else:
            s1_out = args.s1_out
            s2_out = args.s2_out
            s3_out = args.s3_out
            s4_out = args.s4_out
            s5_out = args.s5_out

        # Clear Stage 1 output file at the start to prepare for multiple runs/appends
        if s1_out:
            try:
                dirpath = os.path.dirname(s1_out)
                if dirpath:
                    os.makedirs(dirpath, exist_ok=True)
                with open(s1_out, 'w', encoding='utf-8') as f:
                    f.write("")
            except Exception:
                pass
        result = run_reflection_pipeline_v2(
            plan_log,
            server_name=args.server,
            model_name=args.model,
            microbatch_size=args.microbatch,
            seed=args.seed,
            s1_output_path=s1_out,
            s1_iterations=args.s1_iters,
            s2_output_path=s2_out,
            s3_output_path=s3_out,
            s3_max_updates=args.s3_max,
            s4_output_path=s4_out,
            s5_output_path=s5_out,
        )
        if result:
            # Print Stage 1 output JSON for convenience
            print(json.dumps(result.get('s1', {}), ensure_ascii=False, indent=2))
        return

    # Replan harness (offline)
    if args.replan:
        entries = _load_plan_log_from_jsonl(args.plans, max_entries=args.max_entries)
        if not entries:
            print(json.dumps({"plan": []}))
            return
        import random as _rnd
        if args.replan_mode == 'first':
            entry = entries[0]
        elif args.replan_mode == 'random':
            entry = _rnd.choice(entries)
        else:
            idx = max(0, min(len(entries)-1, int(args.replan_index)))
            entry = entries[idx]
        from plan import generate_plan_with_context
        goal_text = entry.get('goal','')
        prompt_text = entry.get('prompt','')
        percepts = entry.get('percepts_at_plan')
        print(f"Replanning entry {args.replan_index} of {len(entries)}: {entry.get('plan_id','')}\n{json.dumps(entry.get('plan',''), indent=2)}")
        plan_obj = generate_plan_with_context(goal_text, prompt_text, percepts, server_name=args.server, model_name=args.model)
        print(json.dumps(plan_obj, ensure_ascii=False, indent=2))
        return

    # Default: legacy v1 review flow
    if len(plan_log) < args.min_entries:
        logger.info(f"Not enough plan entries to review (have {len(plan_log)}, need {args.min_entries}).")
        return

    text = review_planning(plan_log, server_name=args.server, model_name=args.model)
    if text:
        print(text)


if __name__ == '__main__':
    main()


