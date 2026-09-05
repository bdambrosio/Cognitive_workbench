# Security

Admin passwords are hashed with argon2id before storage. The redirect
endpoint is rate limited by default at 60 requests per minute per client.
Report vulnerabilities to security@example.invalid.
