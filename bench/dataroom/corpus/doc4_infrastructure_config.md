# DOCUMENT 4: INFRASTRUCTURE CONFIG

**Hosting:**
*   **Platform:** Heroku
*   **Dyno:** `standard-1x` (1GB RAM, 0.5 CPU)
*   **Database:** `heroku-postgresql:standard-0` (running on the same dyno as the application)
*   **Read Replicas:** None.
*   **Separate DB Instance:** No.

**Caching:**
*   **Service:** Heroku Redis Add-on
*   **Plan:** `heroku-redis:bb-1`
*   **Cost:** $50/mo

**Backups:**
*   **Schedule:** Daily at 2:00 AM via `heroku pg:backups schedule`
*   **Status:** Failures recorded for the last 21 days.
*   **Last Successful Backup:** 2026-07-30
*   **Alerting:** None configured for backup failures.

**Monitoring:**
*   **Uptime Monitor:** None (No Pingdom, UptimeRobot, or custom checks).
*   **Status Page:** Heroku built-in status page only.

**Security & DNS:**
*   **SSL:** Heroku-managed, auto-renewed.
*   **DNS Provider:** GoDaddy
*   **DNS Management:** Managed personally by 'dave'. No secondary DNS provider.
*   **Secrets Management:** 14 secrets stored in Heroku config vars (Stripe keys, DataEnrich.io API key, Twilio credentials, Google OAuth, DB URL, Redis URL, etc.). No secrets vault.
