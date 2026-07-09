# M1 Fire-Outcome Collection — Exemplar User Inputs

**Purpose:** drive fire-outcome data accumulation (roadmap M1, target
≥50 judged outcomes) through ordinary conversation with Jill running
`--autonomy`. This is a *form* template — the sentences are exemplars,
not a script. **React honestly; the ledger is only worth what your
reactions actually mean.** The only deliberate discipline is to make
your reaction *visible in words* within the judgment window.

## The mechanics that shape the template

- A fire registers a pending outcome. Reflection **Stage 6 judges it
  from your subsequent turns only** — at most 3 per reflection, and only
  from evidence in the exchange. Silence is not evidence *per turn*, but
  after **3 of your turns** (or 7 days) an unjudged fire expires to
  `unobserved`, and a fire "plausibly seen across ≥2 turns and never
  acknowledged" can be judged `ignored`.
- So: **if you have a reaction, voice it within a turn or two.** If you
  deliberately don't react, that's also a datum (`ignored`) — just don't
  let *everything* age out silently, or coverage collapses and M1 stalls.
- Outcomes are two-ledger: the same act can be useful on its domain and
  annoying relationally ("useful but intrusive") — say both when true.

## 1. Seed and bump variety (fires need concerns worth firing)

591 of 844 historical fires are the single PV-monitor concern. Variety
in fires needs variety in what Jill durably cares about. Exemplars:

- "Keep an eye on <topic> for me — anything notable weekly is enough."
- "I care about <project X> going quiet; nudge me if I haven't
  mentioned it in a few days."
- "Track <ticker/feed/metric> daily, but only speak up on real moves."
- Revisit an existing concern's terms: "The PV checks are useful but
  hourly is too chatty — once a day unless something's wrong."
  (Evidence-bumps the concern AND is itself outcome evidence.)

## 2. Reaction exemplars, by intended outcome

After Jill fires something visible, within 1–2 turns:

**helped** — name the value concretely:
- "Good catch on the controller voltage — restarted it, it was wedged."
- "That Ramana one actually landed this morning. Keep those coming."
- "Useful summary; I hadn't seen the second item."

**neutral** — acknowledge without valence:
- "Noted."
- "Saw it. Nothing to do there for now."

**hindered** — say what it cost you (domain or relational or both):
- "Please don't interrupt with stock lines while we're mid-debug —
  that one broke my train of thought." (−relational)
- "That alert was a false alarm; the threshold's too twitchy."
  (−domain)
- "Useful info, bad timing." (+domain, −relational — the two-ledger case)

**ignored** — do nothing, on purpose, for 2+ turns while continuing
your own topics. Don't announce it; the point is unacknowledged
visibility. Use sparingly and deliberately.

## 3. Vary the latency

Sometimes react immediately, sometimes one turn later after your own
topic — this spreads `latency_turns` so the window tuning in M2 has a
distribution to look at, not a spike at zero.

## 4. Anti-patterns

- **Don't** bundle a reaction to her act inside a message that also
  launches a heavy new task — stage 6 caps at 3 judgments per
  reflection and rides the same pass; keep reaction turns light.
- **Don't** perform sentiments you don't hold to "fill categories."
  A skewed-but-honest distribution beats a balanced fake one.
- **Don't** reference fires older than ~3 of your turns and expect a
  record — the pending entry has likely expired to `unobserved`
  (reaching further back is fine conversationally, it just won't land
  in the ledger).

## 5. Cadence and readout

A 10–20 minute session most days, with reactions voiced per §2, should
produce judged outcomes at a few per day. Check progress with:

```bash
python bench/autonomy_review/outcomes.py          # table, target ≥50 judged
```

Watch **coverage** first: if fires are mostly aging out `unobserved`,
the fix is reacting sooner (or, in M2, widening the window) — not more
fires.
