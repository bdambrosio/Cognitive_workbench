# Finding a claim that was already listed — method

## 1. Purpose

A seller says the same thing in more than one place: the README says the service uses less than 10 MB of memory, and the project's web page says it again. Each document is enumerated on its own, so the same assertion is listed once for each document that makes it, and each listing would be tested and reported separately. This step finds the listings that repeat an earlier one, so that the assertion is tested once and the report can say every place it was made.

> For each new claim, say which earlier claim makes the same assertion, if one does.

You propose. A person at the practice sees every pair you name, with both statements side by side, and keeps any claim you were wrong about.

## 2. What you are given

The **new claims**: claims from the document now being considered, each with its `id` and its `statement`. Under each new claim, the **earlier claims** nearest to it in wording: claims already listed, each with the document it came from, its `id` within that document, and its `statement`. An earlier claim can come from the new claim's own document, with a smaller `id`. Nearness in wording is only how the list was drawn up: most of the earlier claims shown are different assertions, and a new claim is often the same as none of them.

## 3. The rule

- **Two claims are the same assertion when one verdict, reached on the same evidence, would settle both.** The test: imagine the first claim has been tested and found contradicted. If the second claim must then be contradicted too, for the same reason, and the same again for every other verdict, they are the same assertion. "The service uses less than 10 MB of RAM under regular load" and "RAM usage is under 10 MB in normal operation" are the same assertion.
- **A different figure, scope, condition or subject makes a different assertion.** "Under 10 MB of RAM" and "under 5 MB of RAM" are two claims, and a seller who states both has made both. "Links can be filtered by their notes" and "links can be filtered by short link, long link and notes" are two claims, because the second could fail where the first holds. "A Docker image is provided" and "the image runs under Docker or Podman" are two claims.
- **A claim about one document and the same words about another document are two claims.** "This file is licensed under MIT" in the README and in the CLI guide are settled by two different files, and each is a claim about its own document.
- **A wider claim and a narrower claim are not the same assertion**, even when testing the wider one would go most of the way to settling the narrower. Leave both.
- **A claim about what the code provides and a claim about what happens when the software runs are not the same assertion**, even when their words are close. "The target contains code that counts visits" and "when the target runs, visits are counted" are two claims.
- **Wording does not matter; what would be tested does.** Two statements in different words that would be settled by the same lines of code in the same way are the same assertion. Two statements in nearly the same words that differ in a number are not.
- **Name the earliest.** When several earlier claims make the same assertion, name the one listed first.
- **A new claim not named in your answer is left as it is.** Do not explain why a claim was left alone.

## 4. The output

One JSON object. Its shape is enforced; this section says what makes a field correct.

| Field | Contents |
|---|---|
| `pairs[]` | One entry for each new claim that makes the same assertion as an earlier claim; none for a new claim that does not |
| `pairs[].claim_id` | The `id` of the new claim, as given to you |
| `pairs[].same_as_source` | The document of the earlier claim, exactly as given to you |
| `pairs[].same_as_id` | The `id` of the earlier claim within that document |

Emit nothing outside the JSON object.
