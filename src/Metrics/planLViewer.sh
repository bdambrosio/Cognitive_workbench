#!/usr/bin/env bash
# --- planview.sh ---
# Minimal pager for plans.jsonl using digest.jq.
# Controls:
#   n = next, p = prev, j = jump to index, t = toggle text/json, q = quit
# Usage: ./planLViewer.sh <CharacterName>|/path/to/plans.jsonl [start_index]

set -euo pipefail

ARG1="${1:-}"
if [[ -z "${ARG1}" ]]; then
  FILE="../data/plans.jsonl"
else
  if [[ -f "${ARG1}" ]]; then
    FILE="${ARG1}"
  else
    FILE="../data/${ARG1}-plans.jsonl"
  fi
fi
IDX="${2:-1}"
MODE="text"        # or "json"
DIGEST="${DIGEST_JQ:-digest.jq}"

if [[ ! -f "$FILE" ]]; then
  echo "No such file: $FILE" >&2; exit 1
fi
if [[ ! -f "$DIGEST" ]]; then
  echo "Missing digest filter: $DIGEST" >&2; exit 1
fi

TOTAL=$(wc -l < "$FILE")
if [[ "$TOTAL" -lt 1 ]]; then
  echo "Empty file." >&2; exit 1
fi
# clamp initial index
if [[ "$IDX" -lt 1 ]]; then IDX=1; fi
if [[ "$IDX" -gt "$TOTAL" ]]; then IDX="$TOTAL"; fi

render() {
  clear
  echo "[${IDX}/${TOTAL}] file: ${FILE} · mode: ${MODE}"
  # Brief metrics header (goal_satisfaction, plan_score, drive_score)
  if [[ "$MODE" == "text" ]]; then
    # Read directly from raw JSON as a robust fallback
    GS=$(sed -n "${IDX}p" "$FILE" | jq -r '.metrics.goal_satisfaction // "n/a"' 2>/dev/null || echo "n/a")
    PS=$(sed -n "${IDX}p" "$FILE" | jq -r '(.metrics.plan_score // .plan_score) // "n/a"' 2>/dev/null || echo "n/a")
    DS=$(sed -n "${IDX}p" "$FILE" | jq -r '
      (.metrics.drive_fulfillment.drives // []) as $dr
      | ($dr|length) as $n
      | ($n == 0) as $empty
      | (.metrics.goal_satisfaction // null) as $gs
      | if $empty or ($gs==null) then null else
          ( (1 - 0.5) / ( ($n - 1) // 1 ) ) as $alpha
          | [ range(0; $n) as $i | 1 - ($alpha * $i) ] as $raw
          | ($raw | add) as $sum
          | (if $sum == 0 then [ range(0; $n) | (1.0 / (($n // 1) // 1)) ] else [ $raw[] / $sum ] end) as $w
          | reduce range(0; $n) as $i (0; . + ( ( ($dr[$i].plan_score // $dr[$i].score // 0) * $gs ) * $w[$i] ) )
        end // "n/a"' 2>/dev/null || echo "n/a")
    echo "goal_satisfaction: ${GS} | plan_score: ${PS} | drive_score: ${DS}"
  fi
  echo "────────────────────────────────────────────────────────────"
  BODY=$(sed -n "${IDX}p" "$FILE" | jq -r -f "$DIGEST" --arg mode "$MODE" 2>/dev/null || true)
  # If text mode, inject activity (if present) just above the line that starts with "goal: "
  if [[ "$MODE" == "text" ]]; then
    ACT=$(sed -n "${IDX}p" "$FILE" | jq -r '.activity // empty' 2>/dev/null || true)
    # Treat None/null/false as no activity
    if [[ -n "$ACT" ]]; then
      low_act="${ACT,,}"
      if [[ "$low_act" != "none" && "$low_act" != "false" ]]; then
        if [[ -n "$BODY" ]]; then
          BODY=$(printf "%s\n" "$BODY" | awk -v act="$ACT" 'BEGIN{ins=0} { if(ins==0 && $0 ~ /^goal: /){ print "activity: " act; ins=1 } print }')
        fi
      fi
    fi
  fi
  if [[ -n "$BODY" ]]; then
    printf "%s\n" "$BODY"
  else
    echo "(no digest data)"
  fi
  echo "────────────────────────────────────────────────────────────"
  echo "(n)ext (p)rev (j)ump (t)oggle json/text (q)uit"
}

render
while IFS= read -rsn1 key; do
  case "$key" in
    n)
      if [[ "$IDX" -lt "$TOTAL" ]]; then IDX=$((IDX+1)); else IDX=1; fi
      render
      ;;
    p)
      if [[ "$IDX" -gt 1 ]]; then IDX=$((IDX-1)); else IDX="$TOTAL"; fi
      render
      ;;
    j)
      echo -ne "\nJump to index (1..$TOTAL): "
      read -r newidx
      if [[ "$newidx" =~ ^[0-9]+$ ]] && [[ "$newidx" -ge 1 ]] && [[ "$newidx" -le "$TOTAL" ]]; then
        IDX="$newidx"
      else
        echo "Invalid index."
        sleep 0.7
      fi
      render
      ;;
    t)
      if [[ "$MODE" == "text" ]]; then MODE="json"; else MODE="text"; fi
      render
      ;;
    q)
      echo; exit 0
      ;;
    *)
      # ignore
      ;;
  esac
done
