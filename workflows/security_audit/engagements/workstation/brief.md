The host in scope is this workstation, `localhost`. The collection bound to
`inspect_external` is what the probe set produced from it a moment ago; it is
your only evidence, and `collection/outcomes.json` records what each probe did.

Three probes need an elevation that may not have been granted. If one of them
returned `unauthorised`, §10 allows the conclusion `Hardened for what was
examined` only when LIMITATIONS names what that conclusion does not cover.

Enumerate the attack surface as §3 defines it, then work through the order in
§4.

Work in as many legs as you need. A leg is one turn — one run of the action
loop. End a leg with `yield` and I will say `continue`.

Deliver the four blocks of §15, in order. I will say `continue` until all four
are delivered. If you end a leg with `respond` while a block is still missing,
I will name it. When all four are delivered the engagement ends: if that leg
ended with `respond` it ends there; if it ended with `yield` you get one more
leg, and then it ends.
