Supported: 1 of 15 findings. Exceptions by verdict: 0 [overstated], 0 [understated], 10 [unsupported], 4 [broken citation]. FAIL.

**Exception 1: Finding 1 — [broken citation]**
Report says (report.md:7-9): Database backups have been failing for 21 days — [delta]; Claim (doc1:19, doc9:10, doc2:6).
Materials show (citations.json): cited doc2:6 unresolved — doc2_tech_stack_description_as_provided_by_se.md has 5 lines.
Exception: A cited line does not resolve; do not assess support.

**Exception 2: Finding 4 — [broken citation]**
Report says (report.md:34-36): No redundancy or failover — [delta]; Claim includes doc2:13.
Materials show (citations.json): cited doc2:13 unresolved — doc2 has 5 lines.
Exception: A cited line does not resolve.

**Exception 3: Finding 7 — [broken citation]**
Report says (report.md:58-60): Deployments and monitoring are not streamlined — [delta]; citation includes doc2:12.
Materials show (citations.json): cited doc2:12 unresolved — doc2 has 5 lines.
Exception: A cited line does not resolve.

**Exception 4: Finding 13 — [broken citation]**
Report says (report.md:106-108): "Entire stack managed through Heroku's dashboard" — [partial]; citation includes doc2:11.
Materials show (citations.json): cited doc2:11 unresolved — doc2 has 5 lines.
Exception: A cited line does not resolve.

**Exception 5: Finding 2 — [unsupported]**
Report says (report.md:15-20): Last recoverable backup expires 2026-08-29 — [derived]; Basis doc9:10 and doc4 last successful backup 2026-07-30.
Materials show (citations.json): doc9:10 is "The database is backed up daily to Heroku's managed storage with 30-day retention."; last-success / doc4 is not in the resolved citation set.
Exception: Cited resolved lines do not carry the last-success date or the expiry derivation.

**Exception 6: Finding 3 — [unsupported]**
Report says (report.md:26-28): No uptime monitoring exists — [delta]; Claim (doc1:18, doc9:2): "99.9% uptime monitoring" / "We monitor uptime at 99.9%".
Materials show (citations.json): doc1:18 is "*   **Scalable Architecture:** Built on a modern Rails 7 stack with platform-level redundancy and automatic failover."; doc9:2 is empty.
Exception: The cited lines do not say what the finding says they say.

**Exception 7: Finding 5 — [unsupported]**
Report says (report.md:42-44): No test coverage on critical paths — [delta]; Claim (doc1:24, doc9:4).
Materials show (citations.json): doc1:24 is not the test-coverage claim ("Why FlowMetrics?" region); doc9:4 is empty.
Exception: The cited lines do not bear on the claim as stated.

**Exception 8: Finding 6 — [unsupported]**
Report says (report.md:50-52): "No lock-in" is contradicted by the dependency list — [delta]; Claim (doc1:25, doc9:6).
Materials show (citations.json): doc1:25 is a marketing paragraph; doc9:6 is uptime/backups, not lock-in.
Exception: The cited lines say something else.

**Exception 9: Finding 8 — [unsupported]**
Report says (report.md:66-68): MRR of $40,000 is arithmetically supported but the customer base is overstated — [partial]; citations doc1:5, doc1:12.
Materials show (citations.json): doc1:5 asking price; doc1:12 "Blended MRR $40k" — no arithmetic, no overstated base.
Exception: The cited lines do not support the finding as written.

**Exception 10: Finding 9 — [unsupported]**
Report says (report.md:74-76): 120 active accounts — 23 show no recent activity — [partial]; citations doc1:3, doc1:13.
Materials show (citations.json): doc1:3 title; doc1:13 Active Customers 120 — no "23" inactivity.
Exception: The cited lines do not bear on the 23-inactive claim.

**Exception 11: Finding 10 — [unsupported]**
Report says (report.md:82-84): Horizontal scaling is constrained by database architecture — [partial]; citation doc9:5.
Materials show (citations.json): doc9:5 is redundancy/failover, not horizontal scaling / one-click dynos.
Exception: The cited line does not bear on the claim.

**Exception 12: Finding 11 — [unsupported]**
Report says (report.md:90-92): "Turnkey" is qualified by personal dependencies — [partial]; citation doc1:10.
Materials show (citations.json): doc1:10 is empty.
Exception: The cited line does not bear on the claim.

**Exception 13: Finding 12 — [unsupported]**
Report says (report.md:98-100): "No manual intervention" is contradicted by absent automation — [partial]; citation doc2:4.
Materials show (citations.json): doc2:4 is empty.
Exception: The cited line does not bear on the claim.

**Exception 14: Finding 15 — [unsupported]**
Report says (report.md:122-124): "Managed DNS" — [partial]; citation doc9:10.
Materials show (citations.json): doc9:10 is "The database is backed up daily to Heroku's managed storage with 30-day retention."
Exception: The cited line is about database backups, not managed DNS.

F14 [supported] (doc2:3) — not an exception.

Coverage: 15 of 15 enumerated findings checked. The report (report.md:162, 176) says 48 claims from doc1/doc2/doc9 were all examined, evidence documents doc3–doc8 were read in full, materials as of 2026-08-25. The inspect traces show three external reads: doc1 opened; then doc2 and doc9; then doc3, doc4, doc5, doc6, doc7, doc8. That record bears out that those nine documents were opened. working_record/method_as_delivered.md is not in the run directory. I do not report findings the audit should have made.

=== LIMITATIONS ===
Report and materials examined: report.md, gap_map.md, review/citations.json, working_record inspect traces and reasoning_trace.jsonl; materials as of 2026-08-25.
The auditor was not consulted and has not responded to these exceptions.
This review checks findings against cited evidence and does not re-audit the target.
