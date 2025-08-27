PLANS="/home/bruce/Downloads/Cognitive_workbench/src/data/plans.jsonl"
(echo -e "goal\tscore\tplan\tsteps\tfailures\tminutes\thunger_delta\tfirst_action\tfirst_target";  jq -r '[.goal // "",
         (.metrics.goal_satisfaction // 0),
         (.plan.name // .plan.activity // .plan.title // "n/a"),
         (.metrics.steps.total // 0), 
         (.metrics.steps.failures // 0),
         (.metrics.time.minutes_advanced // 0),
         (.metrics.hunger.delta // 0),
         (.actions[0].action.type // "n/a"),
         (.actions[0].action.target // "n/a")] | @tsv' "$PLANS") | column -t -s $'\t'

