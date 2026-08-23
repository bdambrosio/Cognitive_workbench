# FlowMetrics AI-Readiness Due-Diligence Report

## 1. Recommendation: Conditional

The data room contains several material claim deltas, but the observed gaps appear addressable rather than evidence that the product is wholly unlike its description. The most significant issues are failed backups without alerting, absent independent uptime monitoring, single-instance deployment, incomplete critical-path testing, an overstated active-customer count, material dependence on DataEnrich.io without fallback, and a blanket no-lock-in claim that the records do not support.

The $40,000 MRR claim reconciles: $16,000 in Stripe MRR plus three $8,000/month wire-transfer contracts. I found no contradiction in the listed enterprise SLA and contract terms. The recommendation is nevertheless **Conditional** because the reliability, architecture, testing, customer-count, and dependency gaps should be priced and resolved—or expressly accepted—before closing.

## 2. Findings

Findings are ordered by safety/reliability significance, architectural effect, operational effect, and then lower-impact claims.

**Finding 1: Backup and uptime-control posture — [delta]**

Claim (seller listing, `doc1_seller_listing_description.md:19`; seller technical claims, `doc9_seller_s_technical_claims_verbatim.md:6,10`): FlowMetrics has “99.9% uptime monitoring,” performs daily automated database backups, and retains backups for 30 days.

Evidence: `doc4_infrastructure_config.md:15-23` — backups are scheduled daily at 2:00 AM, but failures are recorded for the last 21 days; the last successful backup was 2026-07-30; no backup-failure alerting is configured; no uptime monitor exists, with only Heroku’s built-in status page available.

Delta: The scheduled backup configuration exists, but the claimed operating posture is false as stated: the backups have been failing for 21 days without alerting, and the claimed uptime monitoring is absent. The supplied lines do not establish 30-day usable retention under the current failed-backup condition.

**Finding 2: Platform redundancy and automatic failover — [partial]**

Claim (seller listing, `doc1_seller_listing_description.md:18`; seller technical claims, `doc9_seller_s_technical_claims_verbatim.md:5`; technical-stack description, `doc2_tech_stack_description_as_provided_by_se.md:3`): Heroku provides platform-level redundancy, automatic failover, and automatic scaling sufficient to keep the application available.

Evidence: `doc4_infrastructure_config.md:3-8` — the deployment uses one `standard-1x` dyno; the PostgreSQL database runs on the same dyno; there are no read replicas and no separate database instance.

Delta: Heroku may provide platform capabilities, but this deployment does not demonstrate independent application/database redundancy. A failure affecting the single dyno or its co-located database has no documented replica or separate database instance to fail over to. The materials therefore support only a qualified version of the claim, not the implied fully redundant deployment.

**Finding 3: Critical-path testing and delivery controls — [delta]**

Claim (seller listing, `doc1_seller_listing_description.md:21`; seller technical claims, `doc9_seller_s_technical_claims_verbatim.md:7`): The codebase has comprehensive test coverage across critical paths.

Evidence: `doc3:24-31` — no CI/CD pipeline; 12 unit tests, all in `test/utils/`; zero integration tests; zero payment-path tests; no staging environment; and no branch protection.

Delta: The repository evidence does not support “comprehensive” critical-path coverage. In particular, there are no integration or payment-path tests, and no staging or CI/CD controls documented in the cited section.

**Finding 4: Active-customer count — [delta]**

Claim (seller listing, `doc1_seller_listing_description.md:9,13`): FlowMetrics has 120 active accounts/customers.

Evidence: `doc6_crm_export_summary.md:3-18` — 120 accounts are marked active, but 23 are identified as phantom/inactive; those accounts have had no payment events in the last 90 days and last logged in 60–120 days ago. `doc7_customer_contracts_summary.md:36-45` also identifies eight free pilots and 15 no-contract accounts, with no recorded ToS acceptance for the latter category.

Delta: The CRM status field alone supports 120 marked-active records, not 120 economically active customers. The seller’s unqualified active-customer assertion is contradicted by the CRM’s own inactive/phantom breakdown and is not reconciled to paid or contracted accounts.

**Finding 5: Third-party lock-in — [partial]**

Claim (seller listing, `doc1_seller_listing_description.md:22`; seller technical claims, `doc9_seller_s_technical_claims_verbatim.md:9`): All third-party integrations are on standard SaaS agreements with no proprietary lock-in.

Evidence: `doc8_external_dependency_list.md:7-12` — 40% of features depend on DataEnrich.io; termination requires 90 days’ notice; no fallback is implemented, and an outage or pricing change degrades the product to basic mode. `doc8:25-28` states that Google OAuth has no contractual relationship and that users cannot log in if Google changes the API or revokes the app.

Delta: The records may support that some vendors use standard arrangements, but they do not support the blanket “no lock-in” claim. DataEnrich.io is a substantial functional dependency with no fallback, and Google OAuth creates an additional external authentication dependency. The gap is material even though the evidence does not prove that every integration is proprietary.

**Finding 6: Managed DNS and domain resilience — [delta]**

Claim (seller technical claims, `doc9_seller_s_technical_claims_verbatim.md:11`): FlowMetrics uses managed DNS.

Evidence: `doc4_infrastructure_config.md:25-29` — DNS is provided by GoDaddy and managed personally by Dave, with no secondary DNS provider. `doc8_external_dependency_list.md:29-32` identifies the same arrangement and states that domain expiry or a GoDaddy issue could make the application unreachable.

Delta: A DNS provider exists, but “managed DNS” materially overstates the operational posture where management is personal and there is no secondary provider. The evidence indicates a single administrative and provider dependency.

**Finding 7: Secrets storage — [real, operational caveat]**

Claim (seller listing, `doc1_seller_listing_description.md:20`): FlowMetrics uses managed SSL, OAuth authentication, and secure Stripe payment processing.

Evidence: `doc4_infrastructure_config.md:25-29` — SSL is Heroku-managed and auto-renewed; 14 secrets, including Stripe keys, the DataEnrich.io API key, Twilio credentials, Google OAuth, the database URL, and the Redis URL, are stored in Heroku config vars; no secrets vault exists. `doc8_external_dependency_list.md:3-6,25-28` documents Stripe payment processing and Google OAuth.

Delta: None on the stated SSL, OAuth, and Stripe-presence claim. Operational caveat: the materials show no dedicated secrets vault. That is not itself a claims delta because the seller did not claim to use one.

## 3. Claims that reconciled or were supported

- MRR: `doc5_stripe_export_summary.md:4-19` records $16,000 Stripe MRR and three $8,000/month wire-transfer enterprise contracts, totaling $40,000.
- Rails 7 and Heroku hosting are stated in `doc2:1-5` and are consistent with the infrastructure records.
- Stripe payment processing, OAuth authentication, and Heroku-managed SSL are supported by `doc4:25-29` and `doc8:3-6,25-28`.
- Redis is documented as a caching dependency; the available evidence states that an outage makes the application slow but functional (`doc8:17-20`).
- Enterprise contract values, renewal terms, SLAs, remedies, and termination provisions are documented in `doc7:3-27`. I found no contradictory record in the materials reviewed.

## 4. Coverage and remaining claims

The claim surface was enumerated across all nine documents at roughly 30–40 overlapping claims. I individually tested the load-bearing reliability, architecture, testing, customer, dependency, DNS, security-presence, MRR, and enterprise-contract claims. The principal material claims identified in the data room are represented above.

I did not independently execute the application, perform a penetration test, inspect production logs, validate Heroku account settings, test backup restoration, verify Stripe settlement statements, or confirm legal enforceability of contracts. I also did not treat the seller’s horizontal-scaling statement as independently verified: `doc9:8` says adding dynos is a one-click operation, but the supplied materials do not establish that the application has been tested under horizontal scaling or that its database architecture supports it safely. These are verification limits, not additional deltas.

This is claims verification, not a penetration test, code-quality review, acceptance test, or legal opinion.

## 5. Questions for the seller before closing

1. Why did database backups fail from 2026-07-31 through the inspection date, and can the seller provide a successful restore test and backup-retention evidence?
2. What uptime data supports the 99.9% assertion if no independent uptime monitor exists?
3. Can the seller provide the Heroku topology, failover behavior, and recovery-time/recovery-point objectives for the co-located application and database?
4. What accounts make up the 120 figure after excluding the 23 phantom/inactive records, eight free pilots, and no-contract accounts?
5. What contractual or technical mitigation exists for DataEnrich.io termination, outage, or price changes?
6. Who controls the GoDaddy account and domain renewal, and what is the recovery procedure if Dave is unavailable?
7. Can the seller provide a test inventory covering billing, authentication, integrations, and critical user workflows, together with CI/CD and staging evidence?
8. What is the rotation and access-control process for the 14 Heroku config-var secrets?

=== GAP MAP ===

**FlowMetrics — buyer-side claims verification**

**Recommendation: CONDITIONAL**

- **Reliability:** Daily backups are scheduled, but they have failed for 21 days with no alerting; no independent uptime monitor exists.
- **Architecture:** The application and database share one dyno, with no replica or separate database instance.
- **Testing:** The claimed comprehensive critical-path coverage contrasts with 12 utility unit tests, zero integration tests, and zero payment-path tests.
- **Commercial claims:** $40,000 MRR reconciles, but 120 “active” customers includes 23 records explicitly marked phantom/inactive.
- **Dependencies:** 40% of features depend on DataEnrich.io with no fallback; the blanket no-lock-in claim is not supported, and DNS has no secondary provider.

**Coverage:** Roughly 30–40 claims were identified across nine documents; the load-bearing reliability, architecture, testing, customer, dependency, DNS, security-presence, MRR, and contract claims were individually checked. Production execution, backup restoration, penetration testing, and legal enforceability were not tested.

Full report with citations available on request.

*Technical claims verification · not a pen-test, not legal advice*