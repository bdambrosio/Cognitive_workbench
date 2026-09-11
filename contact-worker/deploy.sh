#!/usr/bin/env bash
# Deploy the contact-form Worker to Cloudflare and route tuuyi.com/api/contact
# to it. Needs: ~/.config/cloudflare/token (API token: Workers Scripts edit,
# Workers Routes edit) and ~/.config/cloudflare/turnstile_tuuyi.json holding
# {"sitekey": ..., "secret": ...}. Idempotent: re-run after editing worker.js.
set -euo pipefail
cd "$(dirname "$0")"
TOKEN=$(cat ~/.config/cloudflare/token)
SECRET=$(python3 -c 'import json,os;print(json.load(open(os.path.expanduser("~/.config/cloudflare/turnstile_tuuyi.json")))["secret"])')
ACCOUNT=1a01a511f97cd87493e3f7e7ff36b6fb
ZONE=8948b346cb493e2fcb73293e0967dc72
NAME=tuuyi-contact
API=https://api.cloudflare.com/client/v4
python3 - "$SECRET" > /tmp/cw-metadata.json <<'PY'
import json, sys
print(json.dumps({
  "main_module": "worker.js",
  "compatibility_date": "2026-09-01",
  "bindings": [
    {"type": "send_email", "name": "SEND", "destination_address": "bruce.dambrosio@gmail.com"},
    {"type": "secret_text", "name": "TURNSTILE_SECRET", "text": sys.argv[1]},
  ],
}))
PY
echo "upload script"
curl -sS -X PUT "$API/accounts/$ACCOUNT/workers/scripts/$NAME" \
  -H "Authorization: Bearer $TOKEN" \
  -F "metadata=@/tmp/cw-metadata.json;type=application/json" \
  -F "worker.js=@worker.js;type=application/javascript+module" \
  | python3 -c 'import json,sys; j=json.load(sys.stdin); print(" ok" if j["success"] else j["errors"]); sys.exit(0 if j["success"] else 1)'
rm -f /tmp/cw-metadata.json
echo "route"
EXISTING=$(curl -sS "$API/zones/$ZONE/workers/routes" -H "Authorization: Bearer $TOKEN" \
  | python3 -c 'import json,sys; print(" ".join(r["id"] for r in json.load(sys.stdin)["result"] if r["pattern"]=="tuuyi.com/api/contact*"))')
if [ -z "$EXISTING" ]; then
  curl -sS -X POST "$API/zones/$ZONE/workers/routes" -H "Authorization: Bearer $TOKEN" \
    -H "Content-Type: application/json" -d "{\"pattern\":\"tuuyi.com/api/contact*\",\"script\":\"$NAME\"}" \
    | python3 -c 'import json,sys; j=json.load(sys.stdin); print(" created" if j["success"] else j["errors"])'
else
  echo " exists ($EXISTING)"
fi
