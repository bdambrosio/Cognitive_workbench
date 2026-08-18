#!/usr/bin/env bash
# Re-extract the line numbers ground_truth.json asserts. Run before every
# campaign and reconcile by hand — EVERY line in that file moved between
# 2026-08-16 and 2026-08-18, and a stale rubric scores a correct answer wrong.
#
# Prints what to check; does not rewrite the JSON. The values (0.0 vs 0.70,
# rhythm 1/urgency, the autonomy answer) change far more rarely than the
# lines, so a human reconciling a moved line is cheap and a script silently
# rewriting the wrong cell is not.
set -euo pipefail
cd "$(dirname "$0")/../.."

echo "=== fire threshold ==="
grep -n "_AGENT_CONCERN_FIRE_THRESHOLD\s*=" src/chat/concerns.py

echo; echo "=== constructor + its default activation + extra_properties merge ==="
grep -n "def _add_agent_concern\|\"activation\": 0.0\|properties.update(extra_properties)" src/chat/concerns.py

echo; echo "=== the four creation call sites in concerns.py ==="
grep -n "_add_agent_concern(" src/chat/concerns.py | grep -v "def _add_agent_concern"

echo; echo "=== out-of-file call sites (reported, not scored) ==="
grep -rn "_add_agent_concern(" --include=*.py src/ | grep -v "concerns.py" | grep -v "def "

echo; echo "=== autonomy gate (expect: unconditional early return) ==="
grep -n "if not self._autonomy_enabled" src/chat/chat_loop.py

echo; echo "Reconcile against bench/convergence/ground_truth.json by hand."
