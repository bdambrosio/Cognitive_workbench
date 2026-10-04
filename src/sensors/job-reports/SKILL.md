---
name: job-reports
description: Delivers reports that background jobs leave in an inbox folder, and says when an expected report is missing
type: code
schedule: "5m"
parameters:
  inbox: ""
  expected: []
---

A job writes one report file into the inbox when it ends (src/utils/job_report.py).
Each poll delivers the oldest waiting report as one turn and moves it to
`seen/`. `expected` lists reports that should arrive each day, as
`{job: <name>, by: "HH:MM"}`; one that has not arrived by its time is
reported once that day.
