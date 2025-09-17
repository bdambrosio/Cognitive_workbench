#!/usr/bin/env bash
# Simple CLI to extract activity summaries from a *-activities.json scenario file
# Columns: Name | when | where | Steps
# Usage: ./activity-digest.sh [path/to/*-activities.json]

set -euo pipefail

FILE="${1:-scenarios/Jean-activities.json}"
if [[ ! -f "$FILE" ]]; then
  echo "No such file: $FILE" >&2
  exit 1
fi

# Header
printf "Name\twhen\twhere\tSteps\n"

# Body (tab-separated, then aligned with column)
jq -r '
  to_entries[]
  | .value as $v
  | [
      ($v.name // .key // ""),
      ($v.when // ""),
      (($v.where // []) | map(tostring) | join("; ")),
      (($v.steps // []) | map(tostring) | join("; "))
    ]
  | @tsv
' "$FILE" | column -t -s $'\t'


