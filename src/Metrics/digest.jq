# --- digest.jq v1.1 (ASCII-safe) ---

def _strnorm: tostring | gsub("^\\s+|\\s+$";"");
def _nullable_num: if . == null then null else (try tonumber catch null) end;
def _first_or_null: if length>0 then .[0] else null end;

# Extract activity from the prompt
def _activity_from_prompt:
  (.prompt // "") as $p
  | (try ($p | capture("(?ms)#Your current hi-level activity is:\\s*(?<act>[^\\n]+)")) catch null)
  | if .!=null then .act | _strnorm else null end;

# Extract actors from goal
def _actors_from_goal:
  (.goal // "") as $g
  | (try ($g | capture("(?ms)actors:\\s*(?<as>[^;\\n]+)")) catch null)
  | if .!=null then (.as | split(",") | map(_strnorm) | map(select(length>0))) else [] end;

# Extract starting location [x, y] from prompt
def _loc_from_prompt:
  (.prompt // "") as $p
  | (try ($p | capture("(?ms)#You are at location:\\s*\\[(?<x>-?\\d+),\\s*(?<y>-?\\d+)\\]")) catch null)
  | if .!=null then [ (.x|tonumber), (.y|tonumber) ] else null end;

# Try to pull structured JSON (rationale/outcome/score) embedded in .summary
def _summary_json:
  (.summary // "") as $s
  | (try ($s | capture("(?s)(?<obj>\\{.*\\})")) catch null)
  | if .!=null then (try (.obj | fromjson) catch null) else null end;

# Timestamps
def _ts_first: ( [.actions[]? | .timestamp] | sort | _first_or_null );
def _ts_last:  ( [.actions[]? | .timestamp] | sort | (if length>0 then .[-1] else null end) );

# Counts (prefer metrics if present)
def _counts_obj:
  {
    steps_total: (.metrics.steps.total // (.actions|length) // 0),
    moves:       (.metrics.steps.moves // (.actions|map(select(.action.type=="move"))|length) // 0),
    takes:       (.metrics.steps.takes // (.actions|map(select(.action.type=="take"))|length) // 0),
    uses:        (.metrics.steps.uses  // (.actions|map(select(.action.type=="use")) |length) // 0),
    inspects:    (.metrics.steps.inspects // (.actions|map(select(.action.type=="inspect"))|length) // 0),
    scans:       (.metrics.steps.scans // (.actions|map(select(.action.type=="scan"))|length) // 0),
    failures:    (.metrics.steps.failures // (.actions|map(select(.failure_code!=null))|length) // 0),
  };

# Hunger block (tolerant)
def _hunger_obj:
  {
    start: (.metrics.hunger.start // null),
    end:   (.metrics.hunger.end   // null),
    min:   (.metrics.hunger.min   // null),
    max:   (.metrics.hunger.max   // null),
    delta: (.metrics.hunger.delta // ( ( .metrics.hunger.end // null ) as $e | ( .metrics.hunger.start // null ) as $s | if $e!=null and $s!=null then ($e-$s) else null end))
  };

# Loop suspicion: >=5 consecutive 'move' to same resolved target (ASCII-safe)
def _loop_suspect:
  ( [ .actions[]? | select(.action.type=="move") | (.resolved_target // "NULL") ] ) as $seq
  | reduce $seq[] as $cur ({last:null, run:0, max:0};
      if $cur == .last then
        {last:$cur, run:(.run+1), max:( if (.run+1) > .max then (.run+1) else .max end )}
      else
        {last:$cur, run:1, max:( if .run > .max then .run else .max end )}
      end
    )
  | .max >= 5;

# Binding yield over scans
def _binding_yield:
  ( [ .actions[]? | select(.action.type=="scan") ] ) as $scans
  | ( [ .actions[]? | select(.action.type=="scan" and (.bindings_after != null)) ] ) as $binds
  | if ($scans|length)==0 then null else (( $binds|length ) / ( $scans|length )) end;

# Action compaction
def _actions_compact:
  [ .actions[]? |
    {
      k: (.step_id // .k // 0),
      type: (.action.type // "unknown"),
      target: (.action.target // null),
      resolved: (.resolved_target // null),
      result: (if (.result // "") == "" then null else .result end),
      ok: (.failure_code == null),
      minutes: (.proposed_minutes // null),
      hunger_after: (.hunger_after // null),
      fatigue_after: (.fatigue_after // null),
      thirst_after: (.thirst_after // null),
      binding: (
        if (.bindings_after // null) != null then
          { vars: .bindings_after, evidence: (.binding_evidence // null) }
        else null end
      )
    }
  ];

# Build the digest object
def _digest:
  ( _summary_json ) as $sj
  | {
      plan_id: (.actions[0]?.plan_id // null),
      actor: (_actors_from_goal),
      activity: (_activity_from_prompt),
      goal_short: ((.goal // "") | split(":")[0] | _strnorm),
      goal_full: (.goal // ""),
      ts_first_action: (_ts_first),
      ts_last_action: (_ts_last),
      location_start: (_loc_from_prompt),
      counts: (_counts_obj),
      time: {
        minutes_advanced: (.metrics.time.minutes_advanced // null),
        actual_minutes:   (.metrics.time.actual_minutes   // null)
      },
      hunger: (_hunger_obj),
      fatigue: {
        start: (.metrics.fatigue.start // null),
        end:   (.metrics.fatigue.end   // null),
        min:   (.metrics.fatigue.min   // null),
        max:   (.metrics.fatigue.max   // null),
        delta: (.metrics.fatigue.delta // ( ( .metrics.fatigue.end // null ) as $e | ( .metrics.fatigue.start // null ) as $s | if $e!=null and $s!=null then ($e-$s) else null end))
      },
      thirst: {
        start: (.metrics.thirst.start // null),
        end:   (.metrics.thirst.end   // null),
        min:   (.metrics.thirst.min   // null),
        max:   (.metrics.thirst.max   // null),
        delta: (.metrics.thirst.delta // ( ( .metrics.thirst.end // null ) as $e | ( .metrics.thirst.start // null ) as $s | if $e!=null and $s!=null then ($e-$s) else null end))
      },
      drive_fulfillment: (
        ( .metrics.drive_fulfillment // .metrics.drive_fullfillment // {} )
        | {
            summary: (.summary // null),
            drives:  (.drives  // [])
          }
      ),
      goal_satisfaction: (.metrics.goal_satisfaction // null),
      summary_score: ($sj.score // null),
      summary_outcome: ($sj.outcome // null),
      summary_rationale: ($sj.rationale // null),
      actions: (_actions_compact),
      flags: {
        loop_suspect: (_loop_suspect),
        binding_yield: (_binding_yield)
      },
      percepts_highlights: {
        nearest_water: null,
        nearest_hazard: null
      }
    };

# Human text renderer (ASCII separators)
def _text:
  . as $d
  | ($d.ts_first_action // "?") as $t1
  | ($d.ts_last_action // "?")  as $t2
  | ($d.plan_id // "n/a") as $pid
  | ( ($d.actor // []) | join(", ") ) as $actors
  | ($d.activity // "n/a") as $act
  | ($d.goal_short // "n/a") as $gshort
  | ($d.counts.steps_total // 0) as $st
  | ($d.counts.moves // 0) as $mv
  | ($d.counts.takes // 0) as $tk
  | ($d.counts.uses // 0) as $us
  | ($d.counts.inspects // 0) as $in
  | ($d.counts.scans // 0) as $sca
  | ($d.counts.failures // 0) as $fl
  | ($d.time.actual_minutes // ($d.time.minutes_advanced // null)) as $mins
  | ($d.hunger.delta // null) as $hdel
  | ($d.fatigue.delta // null) as $fdel
  | ($d.thirst.delta // null) as $tdel
  | ($d.drive_fulfillment.drives // []) as $dfs
  | ($dfs | length) as $dfs_len
  | ($dfs | map((.name // "?") + "=" + ((.score // 0)|tostring))) as $df_lines
  | ($d.drive_fulfillment.summary // null) as $dfsum
  | ($d.goal_satisfaction // null) as $sat
  | ($d.summary_outcome // null) as $out
  | ($d.summary_rationale // null) as $why
  | ($d.summary_score // null) as $sc
  | ($d.flags.loop_suspect // false) as $loop
  | ($d.flags.binding_yield // null) as $by
  | (
      "[" + ($t1) + " -> " + ($t2) + "] plan_id=" + ($pid) + " | actor=" + ($actors) + " | activity=" + ($act) + "\n" +
      "goal: " + ($gshort) + "\n" +
      "steps=" + ($st|tostring) + " (move=" + ($mv|tostring) + ", take=" + ($tk|tostring) + ", use=" + ($us|tostring) + ", inspect=" + ($in|tostring) + ", scan=" + ($sca|tostring) + ", fail=" + ($fl|tostring) + ")" +
      " | actual=" + (($mins // "n/a")|tostring) + "m | d_hunger=" + (if $hdel!=null then (($hdel|round)|tostring) else "n/a" end) + (if $fdel!=null then " | d_fatigue=" + (($fdel|round)|tostring) else "" end) + (if $tdel!=null then " | d_thirst=" + (($tdel|round)|tostring) else "" end) + " | sat=" + (($sat // "n/a")|tostring) + "\n" +
      ( if ($dfs_len>0) then ("drives:\n" + ($df_lines | map("  - " + .) | join("\n")) + "\n") else "" end ) +
      "actions:\n" +
      ( $d.actions
        | map(
            "  #" + (.k|tostring) + " " + (.type) + " " + ((.target // "")|tostring) +
            " -> result=" + ((.result // "")|tostring) +
            " | " + (if .ok then "ok" else "fail" end) +
            (if .minutes != null then " | +" + (.minutes|tostring) + "m" else "" end) +
            (if .hunger_after != null then " | hunger=" + ((.hunger_after|round)|tostring) else "" end) +
            (if .fatigue_after != null then " | fatigue=" + ((.fatigue_after|round)|tostring) else "" end) +
            (if .thirst_after != null then " | thirst=" + ((.thirst_after|round)|tostring) else "" end) +
            (if .binding != null then
               ( .binding as $b
                 | (if ($b.vars|type) == "object" then
                      ( " | bind " +
                        ( $b.vars | to_entries | map("\(.key)=\(.value)") | join(", ") ) +
                        ( if $b.evidence and ($b.evidence.distance // null)!=null then " (dist=" + ($b.evidence.distance|tostring) + ")" else "" end )
                      )
                    else "" end)
               )
             else "" end)
          )
        | join("\n")
      ) + "\n" +
      (if $out!=null then "outcome: " + ($out) + "\n" else "" end) +
      (if $dfsum!=null then "drive summary: " + ($dfsum) + "\n" else "" end) +
      (if $why!=null then "rationale: " + ($why) + "\n" else "" end) +
      (if $sc!=null then "score: " + ($sc|tostring) + "\n" else "" end) +
      "flags: loop_suspect=" + ($loop|tostring) + " | binding_yield=" + (($by // "n/a")|tostring) + "\n"
    );

# Entry point
if ($mode // "json") == "json" then _digest
elif $mode == "text" then (_digest | _text)
else _digest
end
