# Splitting a claim about behaviour — method

## 1. Purpose

A claim source often says what the software does when it runs: "the assistant answers common customer questions on its own", "duplicate uploads are detected", "the reviewer never sees the auditor's work". Read in its ordinary sense such a sentence asserts two things. The first is a **mechanism**: code, configuration or build files that provide for the conduct described. The second is **behaviour in operation**: that the conduct occurs when the software runs. Reading code settles the first and nothing in the code settles the second.

Left as one claim, the sentence gets one verdict, and a verdict on the mechanism reads as a verdict on the behaviour. This step separates the two before the surface is frozen, so each gets its own verdict.

> Say which of these claims promise conduct in operation, and for each write the mechanism it asserts and the behaviour it asserts, as two statements.

You propose. The practice sees every split on its surface page before the freeze and removes any it does not accept.

## 2. What you are given

One section of the claim source, with line numbers. The claims enumerated from that section, each with its `id`, its `quote` as the claim source states it, and the `statement` the enumeration wrote.

## 3. The rule

- **A claim is split when its words promise conduct in operation beyond the existence of a mechanism.** Two tests, and a claim is split only when it passes both. First: a buyer who found the mechanism present in the code and the conduct absent in operation would say the claim was false. "Answers common customer questions on its own" passes: the handler can be present and the answers can fail to resolve the questions. Second: the behaviour statement you would write names something a record of the software running could show and the code could not. "Users sign in with their email address" fails this test: once the sign-in handler and its wiring are found in the code, there is nothing left for a record to show, and a behaviour statement would only repeat the mechanism in the present tense. It is not split. "The importer processes every uploaded file without manual steps" passes both: the import path can be present in the code and files can still have needed a person, and the job records would show it.
- **The mechanism statement** says what the code would have to contain: a handler that produces an answer to the customer's question with no step that waits for a person. Phrase it so a reader knows what would be looked for in the code.
- **The behaviour statement** says what happens in operation, at the scope the words state: the answers returned, without a person's involvement, resolve common customer questions. Phrase it so a reader knows what record of the software running would settle it.
- **Ordinary sense, nothing more.** The two statements together say what the claim's statement says, and no more. Do not add properties the words do not carry, and do not sharpen or soften them.
- **Not split:** a claim whose statement names only a mechanism, or whose behaviour statement would only repeat the mechanism; a claim that states a figure with a condition — "answers within two seconds", "under 200 MB under regular use" — which is already one claim about a quantity; a claim that the target lacks something in its code — "no telemetry", "never writes to disk" — which searches over the code settle; a claim about the seller or about a document rather than the software.
- **A claim not listed in your answer is left as it is.** Do not explain why a claim was left alone.

## 4. The output

One JSON object. Its shape is enforced; this document says what makes a field correct.

| Field | Contents |
|---|---|
| `splits[]` | One entry per claim you split; none for a claim left as it is |
| `splits[].claim_id` | The `id` of the claim, as given to you |
| `splits[].mechanism` | The mechanism the claim asserts, one statement in plain words, as a claim about the target |
| `splits[].behaviour` | The behaviour in operation the claim asserts, one statement in plain words, as a claim about the target |

Emit nothing outside the JSON object.
