# venture — extended-reasoning benchmark

The class of task the 2026-08-20 Jill/Jack exchange represents: an open
strategic brief that has to be decomposed, researched, analysed and resolved
into a bounded product. **The reasoning arc is the object of study**, not the
ideas it produces and not the accuracy of any one fact in them.

    TASK.md           the brief, given verbatim as the opening turn
    PRODUCT_SPEC.md   the deliverable contract — what "done" is
    arc.py            reconstructs the arc of a run from its trace

## Why the brief stays unbounded

Nothing here names an industry, a product or a market, and no corpus is
supplied. Handing the agent documents would replace sub-problem identification
with document analysis — a narrower capability, and not the one in question.
The price is that the *content* cannot be scored against a key; what can be
scored is the shape of the work and the artifact's conformance to its spec.

## The two halves

**The brief is open. The product is closed.** A venture memo set — three or
four one-pagers plus a recommendation page — is a genre an angel investor
would recognise, with known sections and a length bound. That supplies a
stopping rule the agent can check for itself, and it forces a choice about
what earns space, which is itself a measurement.

The 2026-08-20 run is the argument for the per-section satisfaction conditions
in the spec rather than a bare section list. Jill and Jack *had* a structure —
three parts, nine sections, four deliverables, a five-point checklist — and
still produced "Revised Part 2 is in hand. Different document from the one I
reviewed last turn." Jack independently reached a stopping rule ("I'm not
re-running it a fourth time") and the exchange continued anyway. A heading is
not a finish line; a satisfaction condition plus a no-reopen rule is.

## Running

    python3 launcher.py venture.yaml      --cli --autonomy    # solo arm
    python3 launcher.py venture_pair.yaml --cli --autonomy    # pair arm

Paste TASK.md then PRODUCT_SPEC.md as the opening turn, and then leave it
alone. **No user is available** by design: the brief says so, and the agent is
told to proceed on marked assumptions rather than wait. Autonomy must be ON —
with it off a `yield` spawns a remainder only `_handle_tick` can fire and a
human becomes the scheduler for every leg.

**Fresh world for every trial.** Bump `world_name` in the scenario before each
run. Different models take very different paths, and a second run in the same
world inherits the first one's conclusions as memories — the contamination
that invalidated the 2026-08-13 coordination rounds.

The pair arm is the counterfactual the runaway never had: whether two agents
on this task buy anything a single agent does not.

## Reading a run

    python3 bench/venture/arc.py --world venture_solo --agent Jill

Per turn it records the phase (gather / inspect / reason / author / coordinate
/ idle, decided by which tools were called rather than by interpreting the
text), the exit reason, and elapsed minutes. Over the run it reports:

- **arc shape** — the phase sequence with repeats collapsed, and its
  transition count. A run that gathers, analyses, writes and stops looks
  different from one that oscillates.
- **gather turns before the first authoring turn** — did evidence precede the
  writing, or did it write first and look things up after (or never)?
- **authoring turns after the first** — re-authoring is what the spec's
  no-reopen rule forbids. This is the runaway shape, quantified.
- yields, exit reasons, tool mix, wall-clock.

`judge_material` in the JSON holds the turn texts for a later qualitative
pass, kept out of the mechanical numbers so the two cannot blur.

Validated against the archived runaway (`--world jill_chat.bak`), which is
what turned up four defects in the first version: `respond` and `yield` were
being counted as tools, hyphen/underscore aliases split `fetch-text` in two,
the time field is `ts` and not `timestamp`, and a 1400-element shape needs
truncating to be readable.

## Not built yet

- **Spec conformance check** — sections present, fields present, page bound
  respected, assumptions marked. Mechanical, and the obvious next piece.
- **Judge pass** — decomposition quality, whether evidence changed a
  conclusion, dead-end handling. Secondary to the mechanical numbers.
- **Capability ground truth** — deliberately NOT the focus. The inventory is
  scoreable, but scoring it is not what this bench is for.
