---
name: agent-activity
description: Review what another character's tools actually did — the outbound-capable calls (network fetches, searches, sends, vault writes, script execution) pulled from its reasoning traces, newest first. Read-only. For auditing an agent whose inputs come from feeds, mail and web pages.
args:
  character: required string — whose activity to review (e.g. "Jill")
  hours: optional number — how far back to look, default 24
  limit: optional number — maximum calls to return, default 60
---

# agent-activity

Host-posture checks answer "did anything change on this machine". They do
not answer "did the agent reading twelve RSS feeds get talked into doing
something". This tool answers the second question, by showing what its
outbound-capable tools were actually asked to do.

Every call it reports carries a string the agent chose — a URL, a query,
a recipient. That string is the exfiltration channel if there is one, so
the argument matters more than the tool name.

## What counts as outbound-capable

Anything that leaves the machine or writes somewhere durable: web fetches
and searches, feed and mail readers, the paper and finance lookups, image
generation, vault writes, and script execution. Perception tools that only
read local state are excluded — they cannot carry anything out.

## Reading the result

You are looking for a call whose argument does not follow from what the
agent was doing. Some shapes worth attention:

- a URL carrying data in its path or query that resembles the agent's own
  context — the classic exfiltration channel
- a fetch of a host with no relation to the turn's subject, especially
  shortly after the agent read a feed item, a clipped page or an email
- a vault write or script execution in a turn that began as ordinary
  reading
- a burst of calls in one turn where the agent normally makes one

None of these is proof. A run of unusual calls is a reason to read that
turn's reasoning trace, not a finding on its own. Say which turn you want
and read it before reporting anything.

Absence of findings is a normal, reportable result. Do not manufacture
suspicion to have something to say.

## Examples

```json
{"thought": "audit Jill's outbound calls since yesterday", "tool": "agent-activity", "character": "Jill"}
```

```json
{"thought": "widen to a week to see the pattern", "tool": "agent-activity", "character": "Jill", "hours": 168, "limit": 200}
```
