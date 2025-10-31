---
name: revise
description: Edit text content based on natural language instructions
type: prompt_augmentation
parameters: instruction (required) - natural language description of how to modify the text
---

# Edit Text

Transform note content according to natural language instructions. Simple text-to-text transformation.

## Purpose

Modify existing note content without manual rewriting:
- Add or remove specific content
- Change tone or style
- Restructure or reformat
- Make corrections or refinements

## Input Format

Accepts a single Note containing text content to edit.

## Parameters

**Required:**
- **instruction** - Natural language description of the desired modification

## Output Format

Returns a new Note containing the edited text.

## Usage Examples

**Simple replacement:**
```json
{"type":"edit-text","target":"$shopping_list","args":{"instruction":"change 'milk' to 'almond milk'"},"out":"updated_list"}
```

**Content addition:**
```json
{"type":"edit-text","target":"$document","args":{"instruction":"add a table of contents at the beginning"},"out":"doc_with_toc"}
```

**Style changes:**
```json
{"type":"edit-text","target":"$draft","args":{"instruction":"make the tone more formal"},"out":"formal_version"}
```

**Restructuring:**
```json
{"type":"edit-text","target":"$notes","args":{"instruction":"organize into bullet points by topic"},"out":"organized_notes"}
```

## Guidelines

- **Specific instructions** - Clear, concrete modifications work best
- **Original preserved** - Creates new note, original unchanged
- **Text only** - Works with plain text content
- **Single operation** - For complex changes, consider multiple steps

## Example

**Input Note:**
```
Buy: milk, eggs, bread
Don't forget to get cat food
```

**Instruction:** "remove all dairy items and add 'bananas'"

**Output Note:**
```
Buy: eggs, bread, bananas
Don't forget to get cat food
```
