# Selecting the property questions for an engagement — method

## 1. Purpose

PROPERTY_QUESTIONS.md holds statements a buyer may want tested, grouped by property of the software: "holds personal data", "takes payments", "gives a language model tools". Only some of them belong in any one engagement. This step chooses them, in three calls:

1. **Identify.** Say which properties the software has, from what the seller claims and what the materials show.
2. **Screen.** For each statement of the properties found, say whether it applies to this software, and whether the seller's claims or the buyer's questions already ask it.
3. **Rate.** The statements that apply and are not already asked are rated for this buyer under TIERS.md, the same rating seller claims get. Only tier 1 statements are tested.

This document covers calls 1 and 2. Call 3 follows TIERS.md.

The practice reads every answer and edits it before any statement is tested. Nothing you write is tested unless a person keeps it.

The call you are making is named at the top of the user message: `IDENTIFY` or `SCREEN`. Follow the section for that call.

## 2. Identify

### What you are given

- The list of properties, each with an id, a name and a description of what indicates it.
- The seller's claims, by id and statement, where any have been enumerated.
- The buyer's questions, by id and statement, where there are any.
- The list of files in the materials. Very large directories are summarised as a count.
- The software's dependencies, by name and type, where a scan of them exists.

### The rule

- **A property is present when the materials show it,** in files, dependencies or configuration. A seller's claim is a reason to look, not evidence: a claim that the product takes payments, with no payment code anywhere, is `unsure`.
- **Judge each property separately.** Software usually has several. Do not stop at the first good fit.
- **Name the evidence.** For `present` and `unsure`, give the files, directory names, dependency names or claim ids that led you there, so a reader can check them. For `absent`, the reason can be short.
- **`unsure` is a proper answer.** Use it when the indication is indirect or partial: a dependency imported but no code that uses it, a directory name that suggests a feature with no further sign. The practice decides those.
- **Do not judge quality.** Whether the software handles the property well is what the statements test later. This call only says whether the property applies.

### The output

| Field | Contents |
|---|---|
| `properties[]` | One entry for every property in the list, in the list's order |
| `properties[].id` | The property's id, as given |
| `properties[].verdict` | `present`, `unsure` or `absent` |
| `properties[].evidence` | File paths, directory names, dependency names or claim ids; empty for `absent` |
| `properties[].reason` | One or two sentences: what the evidence shows, or why the property does not apply |

## 3. Screen

### What you are given

- The candidate statements, each with an id such as `A1.3`, under the property it belongs to, with the identify call's reason for finding that property.
- The seller's claims, by id and statement.
- The buyer's questions, by id and statement.
- The list of files in the materials and the dependencies, as in §2.

### Whether the statement applies

A property can be present while some of its statements do not fit this software. For each statement:

- **`yes`**: the software has the thing the statement is about. A statement about upload features applies when the software accepts uploads.
- **`no`**: the materials show the software does not have the thing the statement is about. "If the API uses GraphQL, query depth is limited" does not apply to an API with no GraphQL. "Marketing emails carry unsubscribe headers" does not apply to software that sends only password-reset email. Say what in the materials shows it.
- **`unsure`**: the file list and dependencies do not show it either way.

Do not judge whether the software meets the statement. A statement the software plainly fails still applies; failing it is a finding.

### Whether a claim or question already asks it

- **`same`**: a claim or question asks the same thing about the same part of the software. Testing both would test one property twice. Name it in `refs`.
- **`broader`**: a claim or question asks part of it; the statement covers more (more data, more integrations, more cases). Name the narrower one in `refs`. The statement is still tested.
- **`none`**: nothing asks it.

A claim that only mentions the same subject is not `same`. "The platform supports Shopify" does not ask whether Shopify's privacy webhooks are carried out. Compare what would be tested, not the words used. When a statement is `same` as both a claim and a question, give both in `refs`.

### The output

| Field | Contents |
|---|---|
| `statements[]` | One entry for every candidate statement, in the order given |
| `statements[].id` | The statement's id, as given |
| `statements[].applies` | `yes`, `no` or `unsure` |
| `statements[].applies_reason` | One sentence: what in the materials shows it applies or does not; for `unsure`, what is missing |
| `statements[].overlap` | `same`, `broader` or `none` |
| `statements[].refs` | For `same` and `broader`: the ids as given, such as `claim README.md#12` or `question questions/buyer.md#3`; empty for `none` |
| `statements[].reason` | One sentence: what the overlapping claim or question tests, and how it differs, if it does; for `none`, say so |

Emit nothing outside the JSON object.
