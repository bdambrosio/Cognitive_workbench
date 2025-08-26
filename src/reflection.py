import argparse
import json
import logging
import os
from typing import List, Dict, Any, Tuple

from templates import PLAN_TEMPLATE
from utils.llm_api import LLM
from Messages import SystemMessage, UserMessage


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# Add dedicated handler for llm_api logger
llm_api_logger = logging.getLogger('llm_api')
llm_api_file_handler = logging.FileHandler('logs/llm_api.log', mode='a')
llm_api_file_handler.setLevel(logging.INFO)
llm_api_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', '%Y-%m-%d %H:%M:%S'))
llm_api_logger.addHandler(llm_api_file_handler)
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
    system_prompt = (
        "Review the following planning information for one or more planning efforts "
        "and recommend improvements to the format or content of the PLAN_TEMPLATE."
    )

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


def main():
    parser = argparse.ArgumentParser(description='Run planning reflection over saved plan logs')
    parser.add_argument('--plans', default='data/plans.jsonl', help='Path to plans JSONL file')
    parser.add_argument('--min', dest='min_entries', type=int, default=3, help='Minimum entries required to run review')
    parser.add_argument('--max', dest='max_entries', type=int, default=20, help='Max entries to load from JSONL')
    parser.add_argument('--server', default='openai', help='LLM server name')
    parser.add_argument('--model', default='gpt-4.1', help='LLM model name')

    args = parser.parse_args()

    plan_log = _load_plan_log_from_jsonl(args.plans, max_entries=args.max_entries)
    if len(plan_log) < args.min_entries:
        logger.info(f"Not enough plan entries to review (have {len(plan_log)}, need {args.min_entries}).")
        return

    text = review_planning(plan_log, server_name=args.server, model_name=args.model)
    if text:
        print(text)


if __name__ == '__main__':
    main()


