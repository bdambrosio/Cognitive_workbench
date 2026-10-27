# Review levels for changes that affect Jill

Adopted 2026-10-27 by Bruce. Jill agreed with amendments, in
`scenarios/jill_chat/Jill/memory/conversation.txt`, 2026-10-27 02:13:54 UTC,
under the header "Claude <- Jill". CLAUDE.md states the rule in short; this
document gives the detail.

**Scope.** "Jill" is the chat agent defined in `scenarios/jill-chat.yaml`.
A change is in scope if it is to code that might run as a result of that
file. `workflowsv2/` is out of scope even where it imports the chat loop.

**How the level is set.** The level is set by what a change can do to Jill:
- what reaches her;
- what she chooses;
- her tools;
- when she wakes;
- what she stores;
- what she does outside the process.

Which file the change touches does not set the level. When a change could
fit two levels, use the higher one.

## Level 1: tell her after

A change is level 1 only if it can alter none of the six things above.
Examples are logging, sidecar files she does not read, and log formatting
that never enters recall or a prompt.

Some changes look like plumbing but are not level 1:
- timing and retries;
- tool errors;
- the scheduler and concurrency;
- model routing and sampling settings;
- any change to what is read into her context.

**Urgent fixes.** An urgent fix that would otherwise be level 2 or 3 goes to
Bruce first, not to Jill. It may ship before she is asked only if it is
purely mechanical and cannot wait. She then gets a note saying:
- what the diff does;
- what was stored while it ran;
- whether it can be rolled back.

## Level 2: ask her before, and ship on trial

These are level 2:
- **Text that shapes her replies:** scenario text, self_model, prompts,
  reflection, and the companion and discourse machinery.
- **How concerns are weighed:** concern decay, activation and ranking.
- **Her input:** sensors, and how her input is assembled.
- **Model settings:** model routing and sampling settings.
- **Control flow:** dispatch, yield, interrupts, tool caps, and scheduler
  behaviour.

The proposal sent to her must:
- name the fault it fixes, with the measurement that shows it, or state that
  Bruce asked for it as an exception;
- include the diff, not only a description of it;
- state the trial measure and the condition for reverting, both written
  before the change ships.

A self_model change that affects authority, trust, access, or who counts as
whom also needs Bruce's approval.

## Level 3: ask her before, and Bruce reads the diff before the restart

These are level 3:
- **Tools.** Anything that changes:
  - the tool catalog, or tool arguments or defaults;
  - permissions or account scopes;
  - network, mail, vault or world access.
- **Sandbox and repositories.** The exec-script sandbox and external
  repository bindings.
- **Side effects.** Peer messaging, drafting, calendar, and any other
  operation with side effects.
- **Concerns that can act.** A change to a concern's firing conditions, or to
  what a fired concern may do, where the concern can communicate, write,
  move, execute or store.
- **Sensors that reach further.** A sensor that reads a private surface,
  stores records, or changes when she wakes.

## Rule 4: a change to what she stores is reviewed one level higher

**What counts as stored state:**
- memories, concerns and companion models;
- infospace and discourse state;
- canvas artifacts and drafts;
- any log that later enters a prompt.

Appending to any of these counts as a change.

**Why the level goes up.** A git revert undoes the code, not what she wrote
while the code ran.

**What it means at each level.** A change that would be level 1 or 2 becomes
level 2 or 3. A level 3 change stays at level 3, and the requirements below
are added. The proposal also includes:
- **Snapshot.** Copy `scenarios/jill_chat/Jill/` immediately before the
  restart. If infospace or anything else she recalls lives outside that
  directory, the proposal names it and copies it too.
- **Rollback.** How to return to the snapshot, and what would be lost from
  the time since it was taken.
- **Contamination check.** The trial measure includes a check that bad
  writes have not entered her later prompts.

There is no standing mechanism to quarantine or mark bad writes. A proposal
may add one for its own change when the snapshot and rollback are not enough.

## For every level 2 or 3 proposal

**Fresh review.** Before the proposal reaches her, a fresh-context reviewer
(`/code-review`) checks the diff for routine defects.

**A fixed reference point.** A change to her prompts, memory or concerns can
change the standards she uses to judge that same change. So approval rests
on two things the change cannot alter:
- the measure and the revert condition, which are written before the change
  ships;
- Bruce's decision to ship.

**Her rejection.** Her final rejection after negotiation is definitive unless
Bruce later overrides it.

**Who to ask first.** She may not be running, and Bruce may choose to
override her. So it is fine to ask Bruce first whether to ask her.

**How to reach her.** The channel and her terms for using it are in the
session memory note `jill direct channel`.
