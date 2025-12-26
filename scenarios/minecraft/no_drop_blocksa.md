# Blocks That Drop Nothing (Common Pitfalls)

Breaking these blocks normally results in NO item entity.

---

## No-Drop Blocks (Common)

- snow (layers)
- tall_grass
- fern
- large_fern
- dead_bush
- fire
- water
- lava
- air

---

## Notes

- Snow layers are especially deceptive:
  - Appear solid
  - Breakable
  - Produce NO drops
- Decorative and plant blocks frequently drop nothing.

---

## Agent Rule

If a block is NOT listed in `guaranteed_drops.md`,
assume it may drop nothing unless explicitly known otherwise.
