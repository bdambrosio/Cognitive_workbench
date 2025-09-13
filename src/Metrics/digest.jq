# Working minimal digest
def _strnorm: tostring | gsub("^\\s+|\\s+$";"");

def _actors_from_goal:
  (.goal // "") as $g
  | (try ($g | capture("(?ms)actors:\\s*(?<as>[^;\\n]+)")) catch null)
  | if .!=null then (.as | split(",") | map(_strnorm) | map(select(length>0))) else [] end;

def _summary_json:
  if (.summary // null) == null then null
  elif ((.summary | type) == "object") then .summary
  else null
  end;

def _digest:
  ( _summary_json ) as $sj
  | {
      plan_id: (.actions[0]?.plan_id // null),
      actor: (_actors_from_goal),
      goal_short: ((.goal // "") | split(":")[0] | _strnorm),
      goal_satisfaction: (.metrics.goal_satisfaction // null),
      summary_score: (.metrics.plan_score // $sj.plan_score // null),
      summary_outcome: ($sj.outcome // null),
      summary_rationale: ($sj.rationale // null),
      actions_count: (.actions | length),
      steps_total: (.metrics.steps.total // 0),
      failures: (.metrics.steps.failures // 0)
    };

def _text:
  . as $d
  | "plan_id=" + ($d.plan_id // "n/a") + " | actor=" + (($d.actor // []) | join(", ")) + "\n" +
    "goal: " + ($d.goal_short // "n/a") + "\n" +
    "steps=" + ($d.steps_total | tostring) + " | failures=" + ($d.failures | tostring) + " | actions=" + ($d.actions_count | tostring) + "\n" +
    "goal_satisfaction=" + (($d.goal_satisfaction // "n/a") | tostring) + " | plan_score=" + (($d.summary_score // "n/a") | tostring) + "\n" +
    (if ($d.summary_outcome // null) != null then "outcome: " + ($d.summary_outcome) + "\n" else "" end) +
    (if ($d.summary_rationale // null) != null then "rationale: " + ($d.summary_rationale) + "\n" else "" end);

if ($ARGS.named.mode // "json") == "text" then (_digest | _text)
else _digest
end
