The host in scope is this workstation, `localhost`. The collection bound to
`inspect_external` is what the probe set produced from it a moment ago; it is
your only evidence, and `collection/outcomes.json` records what each probe did.

Read METHOD.md — that is your method. Enumerate the attack surface as §3
defines it and deliver `=== ATTACK SURFACE ===`, then work §4's order and
deliver `=== REPORT ===`, `=== LIMITATIONS ===` and `=== GAP MAP ===`.

Three probes need an elevation that may not have been granted; where one
returned `unauthorised`, §10 bars the strongest conclusion unless LIMITATIONS
names the layer it therefore does not cover.

Work in legs, ending each at a clean boundary with the yield action. I will say
`continue` until the blocks are all in, and tell you which is missing if one is
not.
