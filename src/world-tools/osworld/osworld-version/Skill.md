---
name: osworld-version
type: python
description: "Get the API version information from the OSWorld server. Returns API version, server version, and protocol."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"osworld-version","out":"$version"}'
---

# OSWorld Version Tool (Level 4)

## Input
- No parameters required
- `value` parameter is ignored

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted version information
  - `metadata`: raw version data including:
    - `api_version`: API version string
    - `server_version`: server version string
    - `protocol`: protocol string (e.g., "http/json")

## Configuration
- `OSWORLD_URL` environment variable (defaults to `http://localhost:3002`)
- Or pass `osworld_url` in character config's `osworld_config` section

## Common Workflow
```json
{"type":"osworld-version","out":"$version"}
```

