---
name: skill-creator
description: Guide for creating new skills that extend cognitive capabilities
---

# Skill Creator

This skill helps you design and structure new skills for the cognition system.

## What Makes a Good Skill

A well-designed skill should:
- Have a clear, specific purpose
- Include concrete examples
- Define when it should be used
- Provide step-by-step guidelines

## Skill Structure

Every skill requires a SKILL.md file with:

1. **YAML Frontmatter** (required)
   - name: unique identifier (lowercase, hyphens)
   - description: what the skill does and when to use it

2. **Markdown Content**
   - Clear instructions
   - Examples
   - Guidelines or best practices

## Creating a New Skill

1. Identify a specific cognitive task or workflow
2. Create a directory with a descriptive name
3. Write SKILL.md with frontmatter and instructions
4. Add supporting files if needed (templates, examples, scripts)
5. Test with relevant scenarios

## Example Template

```
---
name: my-skill-name
description: A clear description of what this skill does
---

# My Skill Name

[Purpose and context]

## When to Use This Skill

- Scenario 1
- Scenario 2

## Instructions

1. First step
2. Second step
3. Third step

## Examples

[Concrete examples of the skill in action]
```
```

---
