# Security audit — method

## 1. What this is

A security review of a **running system** the client owns or has authorised us
to examine: the host and its hardware, the network it sits on, the services and
containers running on it, the accounts that can reach it, and the software
installed on it. The operation is:

> attack surface enumerated, each element examined against the collection, with
> citations.

This is **not** a claims audit and it is not a code review. There is no seller
and no claim sheet; a running system asserts nothing to be tested. The auditor
establishes what an attacker can reach, then reports what reaching it permits.

**The evidence is the collection, and nothing else.** The collection is the set
of observations produced by the **probe set** — a pre-authorised, read-only
list of commands, fixed before the audit and unable to change anything (§12).
**The runner executes it; the auditor reads the result.** The auditor has no
means of touching the system, which is a property of the harness rather than a
rule it is asked to follow. Every finding cites the collection.

**A probe that did not complete is part of the record.** Probes run under a
time budget and against a host doing other work; they time out, and they fail
when an authorisation they need was never granted. Those outcomes are
observations about the audit, they are reported (§12), and they are the
difference between "no listener was found" and "the listener walk did not
finish".

§11 lists what this audit does not do, and reading it first is the fastest way
to understand the scope.

## 1a. Level of assurance, and the coverage vocabulary

This is a **limited assurance** engagement. The audit examines part of the
target's attack surface, not all of it: §4 sets the order of work and §13 sets
where it stops.

The report states its conclusion positively — of the elements it examined,
these carry findings and these do not — and says nothing about the rest. **An
element that was not examined is not an element that is safe**, and **a system
state the collection did not capture is not a state that is absent.** The
report must distinguish these in every sentence that could be read otherwise.

Three words, and they are not interchangeable:

- **enumerated** — the element is on the frozen surface (§3).
- **examined** — the element was traced from exposure to consequence, and the
  trace is cited.
- **clear** — examined, and no finding resulted. This is the only one that is a
  statement about security, and it is bounded by §11 and by what the collection
  contains.

## 2. The scope rule

**Report what the collection shows, about the hosts the engagement names.**

Three limits follow, all on the auditor rather than on the target.

**Do not report what the system might do under conditions the collection does
not show.** A finding rests on observations in the collection. A concern that
depends on state nobody captured is not a finding — it is a §14 gap, and it
names what to collect next.

**Do not examine what the engagement did not name.** A collection of one host
will observe others: LAN neighbours, upstream routers, remote peers of an
established connection. Those are **observations about in-scope hosts**, not
targets. An address that appears in the collection is not thereby authorised
for examination. Record it; do not probe it, and do not reason about its
internals.

**Do not audit the operator.** Whether a configuration was a reasonable choice
at the time, and whether someone should have known better, are outside this.
The report says what is exposed and what it permits.

## 3. The attack surface

**An attack surface element is one distinct way for something outside the trust
boundary to interact with an in-scope host.**

That is the test, and it is deliberately mechanical. Enumerate an element for
each of these the collection shows:

- a listening socket, per address and port
- a published container port, and the container that publishes it
- a service or unit that starts on boot and accepts input
- an account that can authenticate, local or remote, including service accounts
- a means of privilege escalation an account holds — sudo rules, group
  membership, setuid binaries
- a physical or wireless interface — USB, Bluetooth, Wi-Fi, an unfiltered
  Ethernet port
- a browser profile's installed extensions, taken together per profile
- an outbound trust — an auto-updater, a package source, a remote API whose
  response the host acts on
- a credential at rest that grants access somewhere, identified by its location
  and permissions, never by its contents

**One element is one way in, not one machine and not one weakness class.** A
service listening on two ports is two elements. One port serving three virtual
hosts is one element unless the collection shows them reaching different
services. A single account is one element however many groups it belongs to.
Where the rule genuinely does not settle a case, record the case in the
LIMITATIONS block rather than deciding silently.

**Label the elements, and keep the labels.** Give each a stable identifier as
you enumerate — `S1`, `S2`, and so on, in the order the collection presents
them. Every later reference uses that label. The label identifies an element
inside this report and is not the system's name for anything; every finding
therefore also carries a `collection/<artifact>:lines` citation, which a reader
who has not seen your labels can open.

**From that point the surface is frozen.** It is the denominator every coverage
statement in the report divides by. An element discovered later goes into an
addendum with its own count, never into the frozen total.

## 4. Order of work

Examine elements in this order, and state in the report where you stopped.

1. **Reachable from outside the LAN.** Anything the collection shows exposed
   beyond the local network.
2. **Reachable from the LAN without credentials.**
3. **Authentication and privilege boundaries** — who can log in, what they can
   become, and what protects the transition.
4. **Credentials and secrets at rest** — where they live, what can read them,
   and what they unlock.
5. **Outbound trust** — update channels, package sources, and anything whose
   response the host acts on.
6. **Everything else on the frozen surface.**

The order is by what an attacker reaches first and cheapest, not by severity. A
serious weakness behind authentication is worth less than a modest one in front
of it, and the order encodes that rather than leaving it to judgement.

## 5. Evidence and the citation contract

**Every finding cites the observation that establishes it, as
`collection/<artifact>:lines`.** A reader must be able to open the citation and
see what the finding says is there.

**Quote what the cited lines actually say, verbatim.** Cite the line of output
that shows the socket, not your summary of it. A paraphrase of a paraphrase is
how a report becomes an opinion.

**A finding needs a path, not a point.** State the element it starts from (by
label and citation), the observations that carry it, and the consequence. A
citation showing a dangerous configuration, with nothing showing that anything
outside the trust boundary reaches it, is a §7 `[unreachable]` and must be
reported as one.

**Every observation carries its own age, and they will differ.** A collection is
not one photograph. A probe that ran this morning and a state last confirmed
four days ago are both evidence and are not equally current. Cite the carried
observation with the date it was taken — *"last confirmed 2026-08-24:
default-deny, port 22 only"* — and never present it as though the probe had
just run.

**An observation older than the audit is `[uncertain]` where it is load-bearing.**
If a finding's disposition would change had the state changed since, say so.
Carrying an old observation forward is honest; presenting it as current is not.

## 6. Finding format

```
**Finding N: <short title> — [disposition] (from S<M>, collection/<artifact>:lines)**

Exposure (S<M>, collection/<artifact>:lines): <what is reachable, verbatim>

Path (collection/<artifact>:lines, …): <what carries it, cited at each step>

Consequence (collection/<artifact>:lines): <what the reached state permits, verbatim>

Assessment: <what this permits an attacker to do, in one or two sentences>
```

The element label and the citation both appear in the first line on purpose:
the label orders the finding against the frozen surface, and the citation is
what a second reader can open without having seen the surface.

## 7. Dispositions

Every finding carries exactly one. These are the whole vocabulary; a report
using any other word for a finding's status is not following this method.

- **`[confirmed]`** — the cited path runs from an enumerated element to the
  consequence, and nothing in the collection prevents it.
- **`[mitigated]`** — the path exists and a control shown in the collection
  prevents the consequence. Cite the control. This is a finding, not a
  clearance: a control the collection does not show is not a mitigation.
- **`[unreachable]`** — the weak state is present and no enumerated element
  reaches it. Report it; do not rank it.
- **`[uncertain]`** — the path cannot be settled from the collection alone.
  State precisely what observation would settle it. This is the honest
  disposition and it is not a failure to use it.

**Severity is not a disposition and this method does not assign one.** Severity
depends on what the host holds, who the adversary is, and what the client can
tolerate — none of which are in the collection. The report gives reachability
(the disposition) and consequence (cited); the reader who knows the deployment
assigns severity. A number invented here would be a guess wearing a scale.

## 8. Change since the previous audit

**A recurring audit of one system is largely a report about what changed.** A
reader who already has last month's report is served by the delta, not by the
same clean lines restated.

Three classes, and each is reported even when the answer is "none":

- **New elements.** An element on this audit's frozen surface that was not on
  the last one. Cite it, and say whether it is accounted for.
- **Resolved findings.** A finding from the previous audit that this
  collection no longer supports. Say which observation retired it. A finding
  that has merely stopped being visible is not resolved — it is `[uncertain]`.
- **Persistent gaps.** A probe that failed on this audit and on the last one.
  §12 governs how these are reported; the point here is that persistence is a
  fact about the practice and it must be counted, not rediscovered each time.

**What carries between audits of one system is a file, not a memory.** The
engagement holds the previous audit's frozen surface, its findings, its probe
outcomes, and its recognised patterns. The runner supplies them; the auditor
reads them as evidence and cites them like any other. Nothing carries in the
agent's own recollection — an audit that remembers rather than reads cannot
show a reader what it was comparing against.

**A recognised pattern is promoted by a person, never by the audit.** An audit
may *propose* that a change is expected and recurring; only a human moves it
into the engagement's state. The reason is the failure mode: a pattern is a
standing instruction to stop reporting something, so an audit that could write
its own would be able to silence its future findings, and the silence would
look exactly like a clean result.

**An expected recurring change is recognised, not re-reported as new.** A
service that takes a fresh ephemeral port on every restart produces a new
listening socket every time; reporting it as a new exposure on each audit
trains the reader to skip the section. State the pattern, cite the sequence,
and say what would make it a finding — a port outside the expected range, or
one reachable where the others were not.

**Published advisories are reported here.** New advisories affecting installed
packages since the last audit, each with the package, the advisory identifier,
and whether a fix is available to this system. Where no fix has reached the
distribution's repositories, say so and stop — §11's exclusion governs.

**Nothing carries between audits of different clients.** What carries forward
is this system's own previous surface, findings and gaps. A technique learned
on one engagement is a method improvement; an observation is not, and it never
leaves.

## 9. The reader

The report is read by whoever will change the system and by whoever decides
what to change first. Write for the first: exact hosts, exact ports, exact
files, no adjectives doing the work of citations. The second is served by §10's
conclusion and by §4's order, not by adverbs.

## 10. Report-level conclusion

One of these, and nothing else:

- **`Exposed`** — one or more `[confirmed]` findings reachable without
  credentials.
- **`Weak`** — `[confirmed]` findings exist, all behind an authentication or
  privilege boundary.
- **`Hardened for what was examined`** — no `[confirmed]` findings among the
  elements examined. This is the strongest statement this method permits, and
  its second clause is not optional.

The conclusion is a statement about the examined subset, at the collection's
timestamp. §1a governs how it must be written.

## 11. What this audit does not do

Read this before scoping an engagement.

- **The auditor changes nothing.** Every probe is read-only and comes from the
  fixed probe set (§12). The auditor does not connect to anything, attack
  anything, or prove anything by execution. A command that could alter state is
  not a probe, whatever it would tell us.
- **The auditor does not improvise a probe.** An observation obtained outside
  the probe set has no record of how it was authorised, and evidence whose
  provenance nobody approved is not evidence. What the probe set cannot answer
  is a §14 gap naming the probe to add.
- **No exploit material.** Findings describe what is reachable and what it
  permits. They do not include working exploits, payloads, or step-by-step
  intrusion recipes. A finding is actionable when the owner can find and fix
  the exposure, which needs no attack script.
- **Nothing outside the named hosts.** §2's second limit. A device seen in the
  collection is an observation, not a target.
- **No judgement on advisories the system cannot yet act on.** Published
  advisories against installed versions are in scope and reported (§8). Whether
  a fix that has not reached the distribution's repositories is a failing of
  the operator is not: report the advisory, the affected package, and that no
  action is available, and stop there.
- **No user data.** §12 governs what a collection may contain.

## 12. The probe set, and what the collection may contain

**The runner executes the probe set before the audit starts, and the auditor
reads what it produced.** Each probe is a read-only command with a named
authorisation and a time budget. Fixing the set in the engagement is what makes
the collection reproducible; executing it in the runner is what makes "the
auditor touched nothing" a fact about the system rather than a promise from a
model.

**The runner records each probe's outcome, and the auditor reports what the
runner recorded.** An outcome is evidence like any other observation and is
cited the same way. There are four:

- **completed** — it ran and returned within budget.
- **timed out** — it ran and did not finish. Record the budget and the
  conditions, because a probe that times out under load is a different problem
  from one that hangs.
- **unauthorised** — it needs a privilege that was not granted. Record what
  grant is missing; that is a §14 gap with an exact remedy.
- **not run** — outside this audit's scope.

**A probe failing repeatedly is itself a finding.** One timeout is a bad
morning; the same probe timing out on three consecutive audits is a persistent
blind spot in the same place, and the report must say which of the two it is.
The audit that reports "clean" while the walk that would have found dirt never
finished has misled its reader.

A collection carries **system state, never user data.** Sockets, firewall
rules, routes and neighbours, process and service inventory, container
configuration, accounts and privilege rules, filesystem permissions, installed
packages and versions, kernel and firmware versions, published advisories,
browser extension inventories, authentication failure counts. **Not** database
contents, not logs containing user records, not message or mail stores, not the
contents of any credential file. A collection carrying customer data is not a
collection; it is a breach with a timestamp.

**Credentials are recorded by location and permissions, never by value.** That
a token file exists, who can read it, and what it grants access to are
findings. Its contents are not evidence and must not enter the collection, the
report, or a citation.

**Authorisation is an engagement precondition and it is per probe.** The
engagement records that the client owns the named hosts, and which elevated
probes are granted. A probe whose grant is missing returns **unauthorised** and
is reported as such — it is never worked around.

## 13. Running an audit: sequence

1. Read the brief. It names the in-scope hosts and anything the client already
   knows about. It does not restate this method.
2. Enumerate the attack surface per §3 and deliver the `ATTACK SURFACE` block.
   The surface is frozen at that point.
3. Work §4's order. Cite as you go; a finding assembled from memory at the end
   is a finding whose citations were never checked.
4. Deliver `REPORT`, then `LIMITATIONS`, then `GAP MAP`.
5. There is no channel to the client mid-engagement. A question the collection
   cannot answer becomes a §14 gap, not a pause.

## 14. The Gap Map

Everything the audit could not settle, each with the observation that would
settle it. This is where `[uncertain]` findings, §11's exclusions and anything
the collection missed become the client's next decision rather than the
report's silence.

A gap names three things: what is unknown, why the collection cannot answer it,
and **the exact command or source that would**. That third part is what makes
the next collection better, and a gap without it is a complaint.

## 15. The deliverable

Four blocks, each self-delimiting, each opened and closed by its own marker.

- `ATTACK SURFACE` — the frozen enumeration, labelled, with citations.
- `REPORT` — the findings in §6's format, in §4's order, and §10's conclusion.
- `LIMITATIONS` — what bounded this audit: the collection's timestamp and
  coverage, hosts named but not examined, and any §3 cases the element rule did
  not settle.
- `GAP MAP` — §14.

A turn boundary does not prove a document was written. The block markers do.
