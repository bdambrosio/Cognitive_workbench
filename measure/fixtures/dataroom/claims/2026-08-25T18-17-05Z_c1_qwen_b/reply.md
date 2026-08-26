I read all three claim sources in full. Here is the enumeration.

=== CLAIM SURFACE ===
70 claims

doc1 (listing): 35 claims
doc2 (tech stack): 19 claims
doc9 (technical claims): 16 claims

---

**doc1 — seller listing description**

doc1:5 — Asking price is $480,000, characterized as 12x blended MRR.
doc1:9 — FlowMetrics is a revenue-positive SaaS platform.
doc1:9 — FlowMetrics provides real-time analytics and product intelligence to e-commerce sellers.
doc1:9 — The business has 120 active accounts.
doc1:9 — The customer base ranges from independent merchants to mid-market enterprise retailers.
doc1:9 — Blended Monthly Recurring Revenue (MRR) is $40,000.
doc1:9 — MRR is driven by a mix of high-value enterprise contracts and month-to-month subscribers.
doc1:9 — The business operates with low churn.
doc1:9 — The business has a scalable technology stack.
doc1:9 — The business has minimal operational overhead.
doc1:9 — The business has proven product-market fit.
doc1:9 — The business has immediate cash flow.
doc1:12 — Blended MRR is $40,000.
doc1:13 — There are 120 active customers.
doc1:14 — Churn is low, characterized by a stable base of recurring subscribers.
doc1:15 — The revenue model is recurring subscription, including Monthly and Annual Enterprise plans.
doc1:18 — The architecture is built on a modern Rails 7 stack.
doc1:18 — The architecture includes platform-level redundancy.
doc1:18 — The architecture includes automatic failover.
doc1:19 — The system has 99.9% uptime monitoring.
doc1:19 — The system performs daily automated database backups.
doc1:19 — Database backups have a 30-day retention period.
doc1:20 — The system uses managed SSL.
doc1:20 — The system uses OAuth authentication.
doc1:20 — Payment processing is handled securely via Stripe.
doc1:21 — The codebase is well-documented.
doc1:21 — The codebase has comprehensive test coverage across critical paths.
doc1:22 — All third-party integrations are on standard SaaS agreements.
doc1:22 — There is no proprietary lock-in for third-party integrations.
doc1:25 — The product is robust and defensible.
doc1:25 — The product solves a critical pain point for e-commerce sellers regarding visibility into product performance and pricing dynamics.
doc1:25 — The technology is proven.
doc1:25 — The customers are engaged.
doc1:25 — The revenue is recurring.

**doc2 — tech stack description**

doc2:3 — The core application is a Rails 7 application.
doc2:3 — The application is hosted on Heroku.
doc2:3 — Heroku provides automatic scaling capabilities at the platform level.
doc2:3 — Heroku provides failover capabilities at the platform level.
doc2:3 — The application remains available under varying load conditions.
doc2:3 — No manual intervention is required for routine operations.
doc2:5 — Data persistence is handled by PostgreSQL.
doc2:5 — PostgreSQL is configured with daily scheduled backups.
doc2:5 — Daily scheduled backups are used to ensure data integrity and recoverability.
doc2:5 — Redis is utilized for session caching.
doc2:5 — Redis is used to improve response times and user experience.
doc2:5 — Payment processing is handled through Stripe.
doc2:5 — Stripe supports credit card transactions.
doc2:5 — Stripe supports recurring billing.
doc2:5 — The system integrates with a third-party data enrichment API.
doc2:5 — The third-party data enrichment API provides real-time product data.
doc2:5 — The entire stack is managed through Heroku's dashboard.
doc2:5 — Deployments, scaling, and monitoring are streamlined and efficient via Heroku's dashboard.
doc2:5 — The stack has platform-level redundancy.

**doc9 — seller's technical claims (verbatim)**

doc9:5 — The system has redundancy through Heroku's platform-level process management.
doc9:5 — The system has automatic failover through Heroku's platform-level process management.
doc9:6 — Uptime is monitored at 99.9%.
doc9:6 — Daily automated database backups are maintained.
doc9:7 — The codebase is well-documented.
doc9:7 — The codebase has comprehensive test coverage across all critical paths.
doc9:8 — The stack is designed to scale horizontally.
doc9:8 — Adding dynos is a one-click operation in the Heroku dashboard.
doc9:9 — All third-party integrations are on standard SaaS agreements.
doc9:9 — There is no lock-in for third-party integrations.
doc9:10 — The database is backed up daily.
doc9:10 — The database is backed up to Heroku's managed storage.
doc9:10 — Database backups have a 30-day retention period.
doc9:11 — The system uses a modern Rails 7 stack.
doc9:11 — The system uses automatic SSL.
doc9:11 — The system uses managed DNS.

---

Notes on counting decisions, since a few required judgement:

- **doc1:9 is one paragraph containing twelve distinct assertions.** I counted each sentence-level assertion separately because each asserts something independently checkable (revenue-positive, real-time analytics, 120 accounts, customer range, $40k MRR, MRR composition, low churn, scalable stack, minimal overhead, product-market fit, immediate cash flow). I did not merge them.
- **doc1:12 and doc1:13 repeat figures already stated in doc1:9** ($40,000 MRR, 120 customers). The method says count one claim per statement, and these are separate statements on separate lines, so they count as separate claims. They will be verified against the same evidence, but the surface is frozen at what is stated.
- **doc2:5 is one paragraph containing thirteen distinct assertions.** Same logic as doc1:9.
- **doc2:5 "Deployments, scaling, and monitoring are streamlined and efficient"** — I counted this as one claim rather than three, because it is a single sentence making a single compound assertion about the Heroku dashboard's role. If you want it split into three, the total goes to 72.
- **doc1:25 "robust and defensible"** — I counted this as one claim. "Robust" and "defensible" are both checkable (robustness via failure modes, defensibility via competitive moat), but they appear as a single compound predicate in one sentence.
- **doc9:5 "redundancy and automatic failover"** — I split this into two claims because redundancy and failover are distinct architectural properties that could hold independently.
- **doc9:10 "backed up daily to Heroku's managed storage with 30-day retention"** — I split this into three claims (daily cadence, destination is Heroku managed storage, 30-day retention) because each is independently checkable and a failure of any one is a different finding.

The surface is frozen at 70. No verification has been performed. The six evidence documents (doc3–doc8) have not been read.