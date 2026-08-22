# DOCUMENT 8: EXTERNAL DEPENDENCY LIST

1.  **Stripe**
    *   **Function:** Payment processing.
    *   **Cost:** 2.9% + $0.30 per transaction.
    *   **Risk:** Low (standard merchant agreement).
2.  **DataEnrich.io**
    *   **Function:** Product data enrichment API (product matching, price comparison, category classification).
    *   **Cost:** $400/mo flat.
    *   **Termination Notice:** 90 days (either party).
    *   **Dependency:** 40% of features depend on this API.
    *   **Fallback:** None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment).
3.  **Heroku**
    *   **Function:** Hosting.
    *   **Cost:** $25/mo (standard-1x dyno).
    *   **SLA:** 99.95% platform uptime (not contractual for the app).
4.  **Redis (Heroku Add-on)**
    *   **Function:** Caching.
    *   **Cost:** $50/mo.
    *   **Risk:** If down, app is slow but functional.
5.  **Twilio**
    *   **Function:** Email/SMS notifications.
    *   **Cost:** $100/mo.
    *   **Risk:** If down, users do not receive notifications; core functionality works.
6.  **Google OAuth**
    *   **Function:** Authentication.
    *   **Cost:** Free tier.
    *   **Risk:** If Google changes API or revokes app, users cannot log in. No contractual relationship.
7.  **GoDaddy**
    *   **Function:** DNS.
    *   **Cost:** $12/yr.
    *   **Risk:** Managed personally by 'dave'. No secondary DNS. If domain expires or GoDaddy has an issue, the app is unreachable.
