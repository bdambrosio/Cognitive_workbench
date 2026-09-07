# The Record Is Not the Report

*Draft. Third in a series on agent workflows. The first said a method is not a script; the second said a deliverable should be a continuation, not a PDF. This one is about what happened when I built a second workflow to see whether any of that was architecture or just the shape of the first one.*

I run a due-diligence workflow in production: a text method of about twenty thousand characters, and a code runner that drives a language-model agent until the method's deliverables exist. It works well enough that people pay for the output. The question I could not answer from inside it was whether the arrangement was an architecture or a program. A method-and-runner pair that fits only one job is a program with a lot of prose in it.

So I built a second one, for a different job, on the same arrangement, and then read the two side by side. The second job is a security review of a running system: probes are executed against a host, the output becomes a collection of text files, and the agent enumerates the ways in and examines each one against what the probes recorded. Nothing about it is a claims audit. There is no seller, no claim sheet, no buyer, and no transaction.

Three things came out of the comparison. Two of them were the good news I was looking for. The third was a mistake I had already avoided once and made again as soon as I stopped checking for it.

## What carried: the shape of the method

The two method documents were written a month apart, for unrelated subjects, and they have the same section structure. Laid side by side, section by section, the recurring parts are these, in the order they recur.

The operation in one sentence, set off as a quotation. *Compare what the seller asserts against what the materials show, and cite.* *Attack surface enumerated, each element examined against the collection, with citations.*

What you have, and what you do not. The frozen list handed back. The collection and nothing else. No tools and no access.

A scope rule as one bold sentence, followed by limits placed on the examiner rather than on the thing examined. Both documents say so in so many words: the limits are on the auditor.

A denominator, enumerated first and frozen before anything is judged. One calls it the claim surface, the other the attack surface. Both say the freeze is a control and not a convenience, and give the same reason: deciding what counts with the verdicts in view is choosing the sample after seeing the result. Every element on the frozen list then gets exactly one record.

A closed vocabulary in a table, followed by a sentence saying what the vocabulary does not express. Severity is not a disposition. Confidence is not a verdict. Whether a gap matters is judged later, by someone else.

A citation contract: a location, a verbatim span, and what it shows. Every citation must resolve, and the check that resolves it confirms the text exists, not that it supports anything.

An honest disposition for what cannot be settled, with its own reason vocabulary, and the two sentences that keep it honest: not found is not a gap, and not examined is not safe.

A numbered order of work with the stopping point stated, and a list of what the stage does not do, which the reader is told to read first.

The output as a table of fields, introduced by the sentence that matters most in either document: *the schema enforces the shape; this document says what makes a field correct.*

I did not design that list. I found it by reading, after the fact, and the fact that it was arrived at twice rather than designed once is the evidence that it is a form and not a habit. If you are writing a method for an agent to execute, those are the sections, and I would now start from them.

## What carried: the runner's role

No code carried. The second runner began as a verbatim copy of the first and the two diverged from the first hour. I expected that to be the bad news, and it turned out to be the interesting news, because what did carry was the division of labour between the method and the code.

Every clause in either method that begins *the client's process* or *the runner* is a clause about something the agent is not trusted to do: cut the source into sections, assign the identifiers, freeze the list, hand it back, file the evidence under the thing it bears on, execute the probes, resolve the citations, ask again. The security method states the principle in one line. *The runner executes; the auditor reads.* The auditor's inability to touch the system is a property of the harness rather than a rule it is asked to follow.

That is the runner's specification, and it is the same in both workflows: **the runner owns every part of the method that is enforced rather than asked.** Written that way, the runner template is short. Gather or freeze the inputs so the agent cannot. Drive turns until the deliverables are proven to exist, by a marker or a schema and never by a turn ending. Assemble the record from what was emitted. Run the checks the method promises. Hand back what needs doing again. Write down what model was served, at what version, with what settings, and what went wrong on the wire. Both runners do this and nothing else, and the copy needed changing only where the inputs differed.

So the answer to the question I started with is: the architecture is a template for the text and a role for the code. The code itself is not the reusable part, and I have stopped trying to make it one.

## What did not carry, and why that was the finding

The first version of the security review produced a document that passed every check I had. Two blocks delivered in the required order, seven findings in the required format, four dispositions from the required vocabulary, every finding citing a collection file by name and line. I read it and did not understand a word of it. That is not hyperbole. I could see that it was correct and I could not have told you what to change on the machine.

Here is what the claims audit does that the security review did not. The claims audit's method says, in its first section: *do not write for a reader. Fill the fields.* Its agent produces a record, structured and cited and complete over the frozen list, addressed to nobody. A second method, a short one, turns the record into a document for a named reader. A renderer computes the classes, the ordering, the key findings and the figures, and a model writes seven passages between those parts under a list of prohibitions: do not restate a finding, do not assert what the record has not established, do not write the figures, do not give advice. A third method answers questions over the record afterwards, and computes nothing new without saying so.

The security review had one method, and it asked one agent call to produce the record and the document at once. The section titled *The reader* was a paragraph. Every mechanical check passed because every mechanical check was about the record, and the record was fine. Readability was a property of the document, and there was no document, only a record under a document's headings.

I want to be precise about why this counts as a finding about architecture and not just a bug. The three layers have distinct properties that the others cannot supply. The record is complete and cited and written by nobody, which is what makes it checkable. The document is a view over the record for a named audience, which is what makes it usable, and its rule is that everything in it is either copied from the record or introduces it. The query layer answers from both, which is what the second essay in this series was about. Collapse the first two into one call and you get output that is conformant and unreadable, and no check will tell you, because conformance is measured and comprehensibility is not. I had built the split once, for the claims audit, for exactly this reason. I then wrote a second method without it, because the second job seemed simpler and the reader seemed obvious.

## The repair

The repair was to make the security review's record data and to add the two methods it was missing, without touching the first workflow. That last constraint was deliberate: if the second workflow could only be fixed by changing shared code, the architecture claim would be weaker than I wanted.

The agent now delivers two prose blocks: the enumeration, and its working notes over the frozen list. Each, once delivered, is handed to a schema-constrained call that sees the method, the block and the evidence the agent read, and nothing the agent's loop did, and answers under a schema. The runner assigns the labels and freezes the list after the first call. After the second, it resolves every citation into the collection, refuses a gap that does not say what would settle it, and computes the conclusion from the dispositions. The agent no longer writes the conclusion at all.

A new document method names the reader: the person who will change the system first, the person who decides what to change first second. A renderer assembles the document from the record, in the order those two people need, and a model writes five passages between the parts under the same kind of prohibitions the claims report carries, plus two of its own: no severity, and no instruction about what to change a setting to. The remedy locus names where; the owner decides what. A query method binds the collection and the record and answers over them.

Here is what the second run produced, from a review of my own workstation, with the local model. The record has thirteen elements on the frozen surface, twelve findings, five of them mitigated and seven uncertain, and eleven gaps. Every citation resolves into the collection; the runner corrected the line numbers on ten of them and recorded that it did. The computed conclusion is *Hardened for what was examined*, which is the strongest statement the method allows and, as the method says, is not a statement about what was not examined.

One finding, as the document renders it, lightly shortened:

> **S9 — Cloudflare Tunnel creates a global-Internet return path with unknown routing configuration — [uncertain]**
>
> Element S9: cloudflared, running as root, reachable from outside the LAN.
>
> **Exposure.** `collection/processes.txt:29`: "4370 root … /usr/bin/cloudflared --no-autoupdate tunnel run --token-file /etc/cloudflared/token" — cloudflared runs as root and establishes a persistent outbound connection to Cloudflare's edge, creating a return path by which the edge delivers inbound traffic to this host without traversing UFW.
>
> **Path.** `collection/listener_owners.txt:70`: the tunnel's admin endpoint is loopback-only; the data-plane traffic arrives on the established outbound connection, not on a listening port, so UFW does not gate it.
>
> **Consequence.** The tunnel configuration file, which names the services routed to the public edge, is not in the collection; what the tunnel exposes to the global internet is unknown.
>
> **Remedy locus.** Cloudflare tunnel configuration (config.yml or dashboard routing rules).

And the gap that goes with it, from the gap map: *what services or internal ports the tunnel routes to the edge; the configuration file is not in the collection; settled by reading it.* With the command.

That is readable by the person the document method names, and it says what to open. The first version of this review said, in effect, the same things, and I could not find them.

The query layer, asked which finding to act on first, answered S9, quoted the summary's sentence about it with its line number, named the remedy locus and the settling command, and then listed the next three by the order the method sets, with a note that no severity is assigned anywhere in the review and the ordering was by exposure. Asked whether the Bluetooth service was safe, it said: examined, but not enough to say it is safe, and cited the finding's own sentence, *cannot settle*, with the line. It also added one sentence of general knowledge about Docker and firewalls that the record does not contain. The query method discourages that and does not yet forbid it, and I would rather have seen it marked as not from the record.

## What the second workflow settled

The method template carries. I would now write a third method by starting from the ten sections rather than from a blank page, and I expect the first draft to be better for it.

The runner role carries, and the code does not, and that is fine. A runner is a few hundred lines of enforcement around a loop, and copying it is cheaper than abstracting it until a third instance shows which parts are shared.

The record, document, query split is the part of the first workflow that was general, and the stage sequence inside it, enumerate then adjudicate then review then rate, was the part that was particular. I had the two confused. The security review does not enumerate claims or rate materiality and never will, and it needs all three layers anyway.

And the failure mode has a name now, which makes it easier to avoid: one call, two audiences. A record written for a reader is a worse record, and a document written by the record's author is a worse document, and a check that passes both is measuring the wrong thing.

## Where this stops

Two instances are a sample of two. The list of ten sections is what two documents share, not what every method needs, and a third job, a different one, might drop some of them or add one I have not seen. The runner role was derived from two runners that share an ancestor, so the similarity is partly inherited. And the security review's document has been read by one person, me, once. That it is readable is my judgement and not yet a measurement, which is the same gap this essay is about, one level up.
