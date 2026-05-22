---
name: manage-goals
type: python
description: "Create, list, update, and delete scheduled goals. Goals are persistent tasks that can run manually, automatically, or on a daily schedule."
schema_hint:
  action: "string: create | list | update | delete"
  goal_text: "string (required for create): the goal instruction text"
  goal_id: "string (required for update/delete): e.g. goal_3"
  name: "string (optional): short display name for the goal"
  schedule_mode: "string (optional): manual | auto | daily"
  run_at: "string (optional, for daily): time in HH:MM format, e.g. 09:00"
---

# manage-goals

Create, list, update, and delete scheduled goals. Goals are persistent tasks that can be triggered manually, automatically by the scheduler, or on a daily schedule at a specific time.

## Actions

### create

Create a new scheduled goal.

- `goal_text` (required): The full goal instruction text. Be specific — this is what the planner will execute.
- `name` (optional): Short display name (max 120 chars). Defaults to first 120 chars of goal_text.
- `schedule_mode` (optional): `manual` (default), `auto`, or `daily`.
- `run_at` (optional): For `daily` mode, the time to run in HH:MM format (e.g. `"09:00"`).

### list

List all scheduled goals with their status, schedule mode, and last result. No additional parameters needed.

### update

Update an existing scheduled goal's properties.

- `goal_id` (required): The goal ID (e.g. `"goal_3"`).
- `name` (optional): New display name.
- `goal_text` (optional): New goal instruction text.
- `schedule_mode` (optional): New schedule mode (`manual`, `auto`, `daily`).
- `run_at` (optional): New run time for daily mode.
- `status` (optional): Set status (e.g. `"ready"` to re-enable a completed goal).

### delete

Delete a scheduled goal permanently.

- `goal_id` (required): The goal ID to delete.

## Output

- `value`: Human-readable summary of what was done.
- `data`: Structured result (goal dict for create/update, list of goals for list, boolean for delete).

## Examples

```json
{"type": "manage-goals", "action": "create", "goal_text": "Fetch latest news headlines from BBC and summarize them", "name": "Daily news summary", "schedule_mode": "daily", "run_at": "09:00", "out": "$new_goal"}
```

```json
{"type": "manage-goals", "action": "list", "out": "$goals"}
```

```json
{"type": "manage-goals", "action": "update", "goal_id": "goal_3", "schedule_mode": "daily", "run_at": "08:30", "out": "$updated"}
```

```json
{"type": "manage-goals", "action": "delete", "goal_id": "goal_3", "out": "$deleted"}
```

## Common Workflows

**User asks to set up a recurring task:**
```json
{"type": "manage-goals", "action": "create", "goal_text": "Check email and summarize new messages", "name": "Morning email check", "schedule_mode": "daily", "run_at": "08:00", "out": "$goal"}
{"type": "say", "target": "User", "value": "Done — I've scheduled a daily email check for 8:00 AM."}
```

**User asks what's scheduled:**
```json
{"type": "manage-goals", "action": "list", "out": "$goals"}
```

**User asks to change a goal's schedule:**
```json
{"type": "manage-goals", "action": "update", "goal_id": "goal_2", "run_at": "07:30", "out": "$updated"}
```
