---
name: Cognitive Graph Spec Review (2026-03-29)
description: Review findings and open issues for docs/cognitive_graph_spec.md — must resolve before implementation
type: project
---

Reviewed `docs/cognitive_graph_spec.md` on 2026-03-29. Spec is well-grounded in actual codebase (all referenced methods exist). Three **must-resolve-before-coding** issues:

1. **Embedding model mismatch (HIGH)**: Spec says `all-MiniLM-L6-v2` but codebase already uses `BAAI/bge-small-en-v1.5` in `infospace_resource_manager.py`. Two models = double memory, incompatible vectors. Decision needed: reuse existing model.

2. **Node ID propagation gaps (HIGH)**: Option B (`self._last_*` stash) works for tight OODA chain but breaks for deferred relationships — goal_launch→goal_outcome (minutes/hours apart), idle-tick triage nominations (no orient chain), task creation from triage. Need hybrid: Option B for OODA + `node_id_by_key` lookup dict keyed on goal_id/task_wip_id/concern_id.

3. **Thread safety assumptions (HIGH)**: Spec claims no concurrent writes, but `sense_data_callback` runs on Zenoh subscriber threads, and goal completion callbacks need verification. Must audit before committing to Option B.

Additional issues (can fix during implementation):
- No `update_attrs()` API but §9.3 queries `goal_launch` by `status` attr that's never set (MEDIUM)
- FAISS full rebuild on prune — use `IndexIDMap` with `remove_ids()` instead (MEDIUM)
- Template consolidation (§8.3) embeds poorly — include top-N original content strings (MEDIUM)
- `triggered_by` edge direction reversed (LOW)
- Missing triage→concern_created edge (LOW)
- `semantic_search` type_filter should accept list not just str (LOW)
- No scale estimates documented (~100 nodes/min active, ~9MB FAISS/hr) (LOW)

**Why:** This is the next major feature. Getting the spec right avoids rework in the graph core and integration layer.

**How to apply:** When we start implementation, resolve items 1-3 first. Refer back to this review for the full issue list.
