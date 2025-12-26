# Observation Limits and Epistemic Constraints

Defines what can and cannot be inferred from observation tools.

---

## mc-observe Characteristics

- Reports:
  - A sampled subset of nearby blocks
  - Agent position and orientation
- Does NOT report:
  - All nearby blocks
  - Item entities
  - Guarantees of completeness

---

## Sampling Rule

- Absence from observation output ≠ non-existence
- Observation is:
  - Partial
  - Opportunistic
  - Non-exhaustive

---

## Common Epistemic Error

Invalid inference:
> "The block is not listed, therefore it is not present"

This inference is not permitted.

---

## Correct Usage

- Use `mc-observe` to:
  - Gain context
  - Identify nearby block *types*
  - Confirm presence when observed

- Use coordinate reasoning and prior knowledge to:
  - Infer block existence
  - Choose dig targets

---

## Agent Rule (Strong)

Observation supplements reasoning.
It does not replace it.
