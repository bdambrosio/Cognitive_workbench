# Blocks That Are NOT Guaranteed to Drop Items (Common Pitfalls)

Breaking these blocks is often misleading for “did I collect something?” tests.
Many are chance-based, tool-dependent, or otherwise not guaranteed.

---

## Not-Guaranteed Drops (Common)

- snow (layers) *(not a reliable “no drop” test case)*
- tall_grass *(chance-based seeds)*
- fern / large_fern *(chance-based seeds)*
- dead_bush *(often sticks; not guaranteed across contexts)*
- fire
- water
- lava
- air

---

## Notes

- Snow layers are especially deceptive:
  - Appear solid
  - Breakable
  - Drop behavior is tool/context dependent; avoid as a “no drop” assumption
- Decorative and plant blocks frequently drop nothing.

---

## Agent Rule

If a block is NOT listed in `guaranteed_drops.md`,
assume it may drop nothing unless explicitly known otherwise.
