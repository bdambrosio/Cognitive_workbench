# Superseded reviews

Reviews kept out of their run directory on purpose. `workflows/audit_review/runner.py`
refuses to review a run that still holds an earlier review, and renaming the
directory does not satisfy that: the reviewer's `inspect` is bound to the whole
run directory, so any name inside it is still reachable.

Each directory here is `<run id>__<reviewer>`. They are the record of an earlier
review, not part of the run's deliverables.

The reviews here were run on local Flash-Next only to keep the reviewer a
different model from the auditor. That is not the rule — see runner.py: the
reviewer is chosen for review quality, and a model reviewing its own report is
not disqualified. They were re-run on GLM afterwards.
