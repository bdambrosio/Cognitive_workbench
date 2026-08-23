[degraded reply — a model call failed mid-loop and this was synthesized from a partial working log. HTTPSConnectionPool(host='api.deepinfra.com', port=443): Read timed out. (read timeout=300)]

**Jill:**  
I started pulling the Tier 3 operational‑parameter claims from the document set (uptime monitoring, backup schedule, retention, alerting, secrets handling, DNS, etc.). The first batch came back cleanly and shows the key discrepancies:

**Uptime Monitoring**  
- **Seller claim:** “99.9 % uptime monitoring” (Doc 1 L19; Doc 9 L6).  
- **Observed infrastructure:** No external uptime monitor is configured—no Pingdom, UptimeRobot, or custom checks; only Heroku’s built‑in status page (Doc 4 L21‑23).

The request to fetch the remaining sections (backup schedule, retention policies, alerting config, secrets management, DNS setup) hit a read‑timeout on the external API (HTTPSConnectionPool to api.deepinfra.com timed out after 300 s). Because of that, I don’t yet have the full Tier 3 claim set.

**Next step:** I’ll retry the lookup for the remaining parameters once the service responds, then compare each claim against the observed state and give you a complete verification summary.