# Decomposing a broad claim — method

## 1. Purpose

A claim source sometimes states the target's nature in one broad sentence: "a simple, self-hosted URL shortener", "a lightweight CRM for small teams". Read in its ordinary sense that sentence asserts several things a buyer could test, and some things nobody could. This step names the testable ones, so they can be adjudicated like any other claim, and names the rest, so the record shows they were read and set aside.

> Say what a reasonable buyer would take this claim to assert about the software, as separate testable statements, and say what in it cannot be tested.

You stand in for the reasonable buyer reading this document. The practice reads your proposal, edits it and approves it before anything is tested; nothing you write is tested unless a person keeps it.

## 2. What you are given

One claim: its `quote` as the claim source states it, the `statement` the enumeration wrote, and the lines around it in the claim source. The other claims already enumerated from the same document, by id and statement. Where the practice keeps a note on the category the claim names, that note.

## 3. The rule

- **A property is testable when reading code, configuration or build files could settle it.** "Runs from its own container" is testable. "Is simple" is not.
- **Ordinary sense, nothing more.** A category name asserts the core functions any member has and nothing a member might have. "URL shortener" asserts that a short code resolves to a stored URL and that codes can be created; it does not assert analytics, custom domains or an API.
- **Do not repeat what the document already claims elsewhere.** The other claims are listed; a property one of them already states is not proposed again.
- **Do not add what the buyer wants.** The buyer's purposes and thresholds are recorded elsewhere and are not claims.
- **Few, and fewer when the claim carries fewer.** At most six. One is enough for a narrow claim; zero is a proper answer when everything in the claim is either untestable or already claimed.
- **Decline out loud.** Every part of the claim you do not turn into a property is named in `declined`, with the reason in one sentence: vague, a matter of opinion, already claimed, or about the seller rather than the software.

## 4. The output

One JSON object. Its shape is enforced; this document says what makes a field correct.

| Field | Contents |
|---|---|
| `subclaims[]` | The testable properties, each one statement in plain words, as a claim about the target |
| `subclaims[].statement` | The property, phrased so a reader knows what would be tested |
| `subclaims[].property` | Two to five words naming the property: "resolves codes", "self-hostable", "creates codes" |
| `declined[]` | Every part of the claim not proposed |
| `declined[].text` | The words of the claim set aside |
| `declined[].why` | One sentence: vague, opinion, already claimed as claim N, or about the seller |

Emit nothing outside the JSON object.
