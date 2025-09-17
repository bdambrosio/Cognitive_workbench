#!/usr/bin/env bash
# Simple CLI to extract activity ontology summary
# Usage: ./ontology-digest.sh [path/to/*-activity-ontology.json]

set -euo pipefail

FILE="${1:-scenarios/Joe-activity-ontology.json}"
if [[ ! -f "$FILE" ]]; then
  echo "No such file: $FILE" >&2
  exit 1
fi

{
  # Header (included in alignment)
  printf "drive\taxis\tactivity\titemclass\n"

  # Body (tab-separated)
  jq -r '
    def truncN($n):
      (tostring) as $s
      | (if ($s|length) > $n then ($s[0:($n-3)] + "...") else $s end);

    .activities[]
    | [
        ((.drive // "") | truncN(30)),
        ((.axis | if (type=="array" and (length>0)) then .[0] else "" end) | truncN(30)),
        ((.activity_itemclass.activity // "") | tostring),
        ((.activity_itemclass.itemclass // "") | tostring)
      ]
    | @tsv
  ' "$FILE"
} | column -t -s $'\t'


