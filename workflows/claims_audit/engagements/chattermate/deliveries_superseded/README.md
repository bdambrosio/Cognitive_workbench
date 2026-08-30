# Superseded deliveries

Kept out of the run directory on purpose, for the reason
`reviews_superseded/README.md` gives about reviews: the delivery agent's
`inspect` is bound to the whole run directory, so a previous delivery left
inside it is still reachable and the next agent writes against prose it can
read rather than against the report.

Each directory is `<run id>__<model and run>`. They are the record of an
earlier delivery, not part of the run's deliverables.
