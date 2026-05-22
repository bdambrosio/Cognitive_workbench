---
name: manage-tasks
type: python
description: "Create, list, update, and delete tasks. Tasks are work items that go through establishment phases (specification, capability evaluation, execution) with milestone-based progress."
schema_hint:
  action: "string: create | list | update | delete"
  intention: "string (required for create): what the task should accomplish"
  task_id: "string (required for update/delete): e.g. _task_wip_3 or just 3"
  status: "string (optional for update): proposed | in_progress | interrupted | abandoned | completed"
  concern_id: "string (optional for create): linked derived concern ID"
  reason: "string (optional for update+abandon): reason for abandoning"
---

# manage-tasks

Create, list, update, and delete tasks. Tasks are persistent work items that progress through establishment phases (specification, capability evaluation, execution) with milestone-based tracking.

## Actions

### create

Create a new task and begin establishment.

- `intention` (required): What the task should accomplish. Be specific.
- `concern_id` (optional): Link to a derived concern that motivated this task.

Note: Only one task can be establishing at a time. If another task is active, creation will fail.

### list

List all tasks with their status, phase, and linked concerns. No additional parameters needed.

### update

Update an existing task's properties or status.

- `task_id` (required): The task ID (e.g. `"_task_wip_3"` or just `"3"`).
- `intention` (optional): New task intention text.
- `status` (optional): New status. Setting to `"in_progress"` on a proposed/interrupted task will approve and activate it. Setting to `"abandoned"` will abandon with cleanup.
- `reason` (optional): Reason for abandoning (used when status is set to `"abandoned"`).

### delete

Delete a task and all its associated artifacts (milestone goals, WIP notes).

- `task_id` (required): The task ID to delete.

## Output

- `value`: Human-readable summary of what was done.
- `data`: Structured result (task dict for create/update, list of tasks for list, boolean for delete).

## Examples

```json
{"type": "manage-tasks", "action": "create", "intention": "Research and summarize recent papers on attention mechanisms", "out": "$new_task"}
```

```json
{"type": "manage-tasks", "action": "list", "out": "$tasks"}
```

```json
{"type": "manage-tasks", "action": "update", "task_id": "3", "status": "abandoned", "reason": "No longer relevant", "out": "$updated"}
```

```json
{"type": "manage-tasks", "action": "delete", "task_id": "3", "out": "$deleted"}
```
