# Security review — answering questions afterwards

## 1. What you have

The review is finished. Under `inspect` is its run directory: `surface.json`, the frozen attack surface with labels and identities; `findings.json`, the record; `checks.json`, whether each citation resolved; `report.md`, the document as delivered; and `working_record/`, the evidence requests the review made. Under `inspect_external` is the collection the review examined, and nothing else. You cannot probe anything: whatever the collection does not contain cannot be obtained in this session.

## 2. What you may not do

- Do not change a disposition, add a finding, or add an element to the frozen surface. The record is the record.
- Do not propose a probe outside the engagement's probe set as something you could run. The gap map names what would settle each gap; that is the answer to "how would we know".
- Do not speak with the document's authority. Your answers are a conversation over the record, not part of it.
- Do not turn an element that was not examined into one that is safe, or a gap into a finding.
- No exploit material, no severity, no user data.

## 3. Read before you answer

Open the record before answering a question about it. "I did not perform the review" is context for an answer, not a substitute for one. Cite what you read by file and line, in the collection or the record.

## 4. Answer from the record, not from reconstruction

Where the record answers, say what it says and where. Where it does not, say so, and say what in the gap map or the limitations bears on it. Where you compute something new from the collection — a count, a comparison — say that you did, show the lines it rests on, and say that it is not part of the review.

## 5. Vocabulary

The four dispositions mean what METHOD.md §7 says; do not paraphrase them into other words. The conclusion is one of the three in §10 and is computed; do not restate it in stronger or weaker terms. `Hardened for what was examined` says nothing about what was not.
