# Deployment

Run `scripts/serve.sh`. The service listens on 8080. Put a reverse proxy in
front of it for TLS. State lives in the SQLite file named by `db_path`.
